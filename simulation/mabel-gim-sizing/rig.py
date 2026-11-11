"""GIM actuator layer ported to the MABEL MuJoCo model (base locked), plus the stock-servo baseline.

The electrical and firmware model is the one in simulation/openarm_v2/gim/actuated.py (rigid mode), generalised from
14 OpenArm joints to 15 MABEL joints: Torso plus Left/Right Arm 1..7. Only the MABEL XML is read (never executed).
"""
import copy
import sys
from dataclasses import dataclass, field
from collections import deque
from pathlib import Path

import mujoco as mj
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gim_params as G  # noqa: E402

import os  # noqa: E402
# Set OPEN_MABEL to a clone of https://github.com/robotmabel/open_mabel
XML = str(Path(os.environ.get('OPEN_MABEL', HERE.parent / 'open_mabel')) / 'simulation' / 'mabel_mujoco' / 'models' / 'mabel_full.xml')
ARM_J = [f'{s} Arm {i}' for s in ('Left', 'Right') for i in range(1, 8)]
JN = ['Torso'] + ARM_J                      # the 15 joints that get a GIM actuator (index 0 = torso)
IDX = [0] + list(range(14))                 # torso uses the GIM8115-36 (arm J1 row); arms map by joint index 1..7
DT_E = 25e-6                                # electrical substep, as in the OpenArm layer
SWERVE_ACT = ['FL Steer', 'FL Drive', 'FR Steer', 'FR Drive', 'B Steer', 'B Drive']

# worst static torso pose from size_mabel.py / sizing.json
A1_WORST = -2.2689285
ELBOW = 0.9
WORST = {'Torso': -1.0472, 'Left Arm 1': A1_WORST, 'Right Arm 1': 2.2689285,   # right: clip(-a1) is unclipped here
         'Left Arm 4': ELBOW, 'Right Arm 4': -0.86 * ELBOW}


# GIM torso PD gains (firmware, MIT style): kp = I wn^2, kd = 2 zeta I wn with I = 6.7 kg m^2 (3 kg/hand worst pose,
# armature included), wn = 25 rad/s, zeta = 1. Arms keep the stock MABEL per-joint gains (kp 1205/1880/760/940/269/249/315).
TORSO_KP, TORSO_KD = 4200., 335.


@dataclass
class Config:
    dt: float = 0.5e-3
    vbus: float = 48.
    wc: float = 2 * np.pi * 1000
    drive_hz: float = 10e3
    delay_ms: int = 1
    eta: float = 0.9
    eta_hat: float = 0.95
    lambda_source: str = 'kt'
    friction: bool = True
    i_lsb: float = 0.0806
    v_cut_hz: float = 1000.
    enc_bits: int = 21
    kp: object = None
    kd: object = None


def min_jerk(t, t0, T, a, b):
    """Position, velocity, acceleration of a minimum-jerk move a->b starting at t0, lasting T (arrays allowed)."""
    s = np.clip((np.asarray(t, float) - t0) / T, 0., 1.)
    d = np.asarray(b, float) - np.asarray(a, float)
    p = a + d * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5)
    inside = ((t - t0) > 0) & ((t - t0) < T)
    v = np.where(inside, d / T * (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4), 0.)
    a_ = np.where(inside, d / T ** 2 * (60 * s - 180 * s ** 2 + 120 * s ** 3), 0.)
    return p, v, a_


class Rig:
    """MABEL with fixed chassis. actuation = 'gim' (15 joints on GIM actuators) or 'stock' (shipped servos)."""

    def __init__(self, payload=0., actuation='gim', cfg=None):
        assert actuation in ('gim', 'stock')
        self.cfg = cfg = cfg or Config()
        self.actuation, self.payload = actuation, payload
        self.P = {k: v[IDX] for k, v in G.arm_params(cfg.eta, cfg.lambda_source).items()}
        self.n_sub = round(cfg.dt / DT_E)
        self.fw_every = max(1, round(1 / (cfg.drive_hz * DT_E)))
        self.a_i = 1 - np.exp(-cfg.wc * DT_E)
        self.a_v = 1 - np.exp(-2 * np.pi * cfg.v_cut_hz / cfg.drive_hz)
        self.vmax = 0.95 / np.sqrt(3) * cfg.vbus
        self.etahat_kt = self.P['N'] * self.P['kt_m'] * cfg.eta_hat
        self.lsb_enc = 2 * np.pi / 2 ** cfg.enc_bits
        self._build()

    # ------------------------------------------------------------------ model
    def _build(self):
        cfg, P = self.cfg, self.P
        spec = mj.MjSpec.from_file(XML)
        for s in list(spec.sensors):                       # 360 lidar rays cost time and are not needed
            spec.delete(s)
        spec.delete(spec.joint('base_root'))               # chassis welded to the world at its keyframe pose
        spec.option.timestep = cfg.dt
        spec.option.disableflags |= mj.mjtDisableBit.mjDSBL_CONTACT   # free-space motion; joint limits stay on
        if self.payload > 0:
            for palm in ('left_palm', 'right_palm'):
                b = spec.body(palm).add_body(name=f'payload_{palm}', pos=[0, 0, 0])
                b.add_geom(type=mj.mjtGeom.mjGEOM_SPHERE, size=[0.03, 0, 0], contype=0, conaffinity=0,
                           mass=self.payload)
        m = self.m = spec.compile()
        self.d = d = mj.MjData(m)
        self.dof = np.array([m.joint(n).dofadr[0] for n in JN])
        self.qadr = np.array([m.joint(n).qposadr[0] for n in JN])
        self.act = np.array([m.actuator(n).id for n in JN])
        self.lift_act = m.actuator('Lift').id
        self.lift_q = [m.joint('Lift Lower').qposadr[0], m.joint('Lift Upper').qposadr[0]]
        self.lift_dof = [m.joint('Lift Lower').dofadr[0], m.joint('Lift Upper').dofadr[0]]
        self.lo, self.hi = m.jnt_range[[m.joint(n).id for n in JN]].T
        # base / wheels disabled (chassis is fixed)
        for n in SWERVE_ACT:
            a = m.actuator(n).id
            m.actuator_gainprm[a, :3] = 0
            m.actuator_biasprm[a, :3] = 0
        self.kp_stock = m.actuator_gainprm[self.act, 0].copy()
        self.kd_stock = -m.actuator_biasprm[self.act, 2].copy()
        self.frange_stock = m.actuator_forcerange[self.act].copy()
        self.stock_joint = dict(arm=m.dof_armature[self.dof].copy(), damp=m.dof_damping[self.dof].copy(),
                                fric=m.dof_frictionloss[self.dof].copy())
        if self.actuation == 'gim':
            m.actuator_gainprm[self.act, :3] = 0
            m.actuator_biasprm[self.act, :3] = 0
            m.actuator_forcerange[self.act] = 0
            m.actuator_forcelimited[self.act] = 0
            m.dof_armature[self.dof] = P['J_rotor'] * P['N'] ** 2
            m.dof_damping[self.dof] = P['b'] if cfg.friction else 0
            m.dof_frictionloss[self.dof] = P['tau_c'] if cfg.friction else 0
            m.dof_solref[self.dof] = [0.002, 1]
        kp = self.kp_stock.copy() if cfg.kp is None else np.asarray(cfg.kp, float)
        kd = self.kd_stock.copy() if cfg.kd is None else np.asarray(cfg.kd, float)
        if self.actuation == 'gim' and cfg.kp is None:
            kp[0], kd[0] = TORSO_KP, TORSO_KD        # arms keep MABEL's own per-joint gains, torso is re-tuned
        self.kp, self.kd = kp, kd
        # constraint-free copy for mj_inverse (rigid-body feedforward); same masses/armature as the simulated model
        self.m_id = copy.copy(m)
        self.m_id.opt.disableflags |= mj.mjtDisableBit.mjDSBL_CONSTRAINT
        self.d_id = mj.MjData(self.m_id)
        self.q_default = d.qpos.copy()

    # ------------------------------------------------------------------ state
    def set_pose(self, q15, lift=0.3175, lift_ctrl=None):
        """Put the robot at the given 15-joint pose and lift height, zero velocity, stock actuators holding."""
        m, d = self.m, self.d
        mj.mj_resetData(m, d)
        d.qpos[self.qadr] = q15
        d.qpos[self.lift_q] = lift / 2
        d.ctrl[self.lift_act] = lift if lift_ctrl is None else lift_ctrl
        d.ctrl[self.act] = q15
        mj.mj_forward(m, d)
        self.ext_ctrl_q = q15.copy()
        self._reset_electrical(q15)

    def _reset_electrical(self, q15):
        z = np.zeros(15)
        self.iq, self.iq_cmd, self.tau_cmd = z.copy(), z.copy(), z.copy()
        self.sub_clock = 0
        self.rows = []
        th = q15.copy()
        self.cmd = dict(q_ref=th, v_ref=z.copy(), tau_ff=z.copy())
        self.queue = deque([dict(self.cmd) for _ in range(self.cfg.delay_ms)])
        self.q_meas = np.round(self.P['N'] * th / self.lsb_enc) * self.lsb_enc / self.P['N']
        self.q_prev, self.v_meas = self.q_meas.copy(), z.copy()

    def preload(self, q_ref, tau_ff):
        """Start the command pipe, firmware and current loop in steady state at the held pose."""
        z = np.zeros(15)
        for c in [self.cmd, *self.queue]:
            c['q_ref'], c['v_ref'], c['tau_ff'] = np.asarray(q_ref, float).copy(), z.copy(), np.asarray(tau_ff, float).copy()
        self._firmware(self.d.qpos[self.qadr])
        self.iq = np.clip(self.iq_cmd, -self.P['i_max'], self.P['i_max'])

    @property
    def q(self):
        return self.d.qpos[self.qadr].copy()

    @property
    def v(self):
        return self.d.qvel[self.dof].copy()

    # ------------------------------------------------------------------ rigid-body feedforward
    def inverse_torque(self, q15, v15, a15, lift, lift_v=0., lift_a=0., base_q=None):
        """mj_inverse torque at the 15 joints for a given joint state (constraints disabled, hands/neck at base_q)."""
        d = self.d_id
        d.qpos[:] = self.q_default if base_q is None else base_q
        d.qvel[:] = 0
        d.qacc[:] = 0
        d.qpos[self.qadr], d.qvel[self.dof], d.qacc[self.dof] = q15, v15, a15
        d.qpos[self.lift_q], d.qvel[self.lift_dof], d.qacc[self.lift_dof] = lift / 2, lift_v / 2, lift_a / 2
        mj.mj_inverse(self.m_id, d)
        return d.qfrc_inverse[self.dof].copy()

    # ------------------------------------------------------------------ electrical
    def volt_limits(self, w):
        P = self.P
        we = P['pp'] * P['N'] * w
        A = P['R'] ** 2 + (we * P['L']) ** 2
        B = P['R'] * we * P['lam']
        D = A * self.vmax ** 2 - (we ** 2 * P['L'] * P['lam']) ** 2
        s = np.sqrt(np.maximum(D, 0.))
        return (-B - s) / A, (-B + s) / A, we

    def envelope(self, w):
        """Maximum delivered torque (listed Kt x iq) available at output speed w (>=0 motoring), id = 0."""
        lo, hi, _ = self.volt_limits(np.full(15, w))
        i = np.minimum(hi, self.P['i_max'])
        return np.maximum(i, 0) * G.TILE(G.KT_OUT)[IDX]

    def _firmware(self, th):
        P, cfg = self.P, self.cfg
        q = np.round(P['N'] * th / self.lsb_enc) * self.lsb_enc / P['N']
        self.v_meas = self.v_meas + self.a_v * ((q - self.q_prev) * cfg.drive_hz - self.v_meas)
        self.q_prev = self.q_meas = q
        c = self.cmd
        self.tau_cmd = self.kp * (c['q_ref'] - q) + self.kd * (c['v_ref'] - self.v_meas) + c['tau_ff']
        self.iq_cmd = self.tau_cmd / self.etahat_kt

    # ------------------------------------------------------------------ stepping
    def step_ms(self, q_ref, v_ref, tau_ff, lift_cmd):
        """Advance 1 ms. GIM: PD command with 1 ms delay. Stock: position-servo ctrl = q_ref (+ tau_ff / kp)."""
        d = self.d
        d.ctrl[self.lift_act] = lift_cmd
        if self.actuation == 'stock':
            d.ctrl[self.act] = q_ref + tau_ff / self.kp
        else:
            self.queue.append(dict(q_ref=np.asarray(q_ref, float).copy(), v_ref=np.asarray(v_ref, float).copy(),
                                   tau_ff=np.asarray(tau_ff, float).copy()))
            self.cmd = self.queue.popleft()
        self._q_ref_now = np.asarray(q_ref, float).copy()
        for _ in range(round(1e-3 / self.cfg.dt)):
            self._phys_step() if self.actuation == 'gim' else self._stock_step()

    def _stock_step(self):
        d, m = self.d, self.m
        mj.mj_step(m, d)
        self.rows.append(dict(t=d.time, q=self.q, v=self.v, q_ref=self._q_ref_now, force=d.actuator_force[self.act].copy(),
                              lift=d.qpos[self.lift_q].sum()))

    def _phys_step(self):
        cfg, P, d, m = self.cfg, self.P, self.d, self.m
        w, th = d.qvel[self.dof].copy(), d.qpos[self.qadr].copy()
        lo, hi, we = self.volt_limits(w)
        imax, iq, a = P['i_max'], self.iq, self.a_i
        s_iq = s_viq = s_i2 = 0.
        fv = np.zeros(15, bool)
        fi = np.zeros(15, bool)
        for s in range(self.n_sub):
            if self.sub_clock % self.fw_every == 0:
                self._firmware(th + w * (s * DT_E))
            self.sub_clock += 1
            want = np.clip(self.iq_cmd, -imax, imax)
            tar = np.clip(want, lo, hi)
            new = np.clip(iq + a * (tar - iq), lo, hi)
            vq = P['R'] * new + we * P['lam'] + P['L'] * (new - iq) / DT_E
            fv |= tar != want
            fi |= np.abs(self.iq_cmd) > imax
            s_iq = s_iq + new
            s_viq = s_viq + vq * new
            s_i2 = s_i2 + new * new
            iq = new
        self.iq = iq
        n = self.n_sub
        iq_m = s_iq / n
        tau_m = P['N'] * P['kt_m'] * iq_m
        if cfg.friction:
            m.dof_frictionloss[self.dof] = P['tau_c'] + (1 - P['eta']) * np.abs(tau_m)
        p_dc = 1.5 * s_viq / n + 1.5 * (s_i2 / n) * G.R_INVERTER
        d.qfrc_applied[:] = 0
        d.qfrc_applied[self.dof] += tau_m
        mj.mj_step(m, d)
        self.rows.append(dict(t=d.time, q=self.q, v=self.v, q_ref=self._q_ref_now, iq=iq_m, iq_cmd=self.iq_cmd.copy(),
                              tau_m=tau_m, p_dc=p_dc, vflag=fv, iflag=fi, vlo=lo, vhi=hi, tau_cmd=self.tau_cmd.copy(),
                              lift=d.qpos[self.lift_q].sum()))

    def log(self):
        return {k: np.array([r[k] for r in self.rows]) for k in self.rows[0]}
