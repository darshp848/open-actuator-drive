"""OpenArm v2 with GIM actuators (rotor + gearbox + dq current loop + drive firmware) replacing the stock
Damiao position servos on the 14 arm joints. Fingers keep the vendor position control. See SPEC.md."""
import sys
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import mujoco as mj
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import robot  # noqa: E402
import params as G  # noqa: E402

DT_E = 25e-6            # electrical substep


@dataclass
class Config:
    transmission: str = 'rigid'      # 'rigid' | 'compliant'
    mode: str = 'pd'                 # 'pd' | 'torque'
    contact: bool = False
    dt: float = 0.5e-3               # physics step, a multiple of DT_E
    vbus: float = 48.
    wc: float = 2 * np.pi * 1000     # current loop bandwidth
    drive_hz: float = 10e3
    delay_ms: int = 1
    eta: float = 0.9
    eta_hat: float = 0.95
    lambda_source: str = 'kt'
    friction: bool = True
    gim_masses: bool = False
    i_lsb: float = 0.0806            # 1 mOhm shunt, 10 V/V, 12 bit at 3.3 V
    v_cut_hz: float = 1000.
    enc_bits: int = 21               # MT6835 rotor encoder, per motor turn
    out_enc_bits: int = 17           # ASSUMED output-shaft encoder, per output turn
    backlash: float = G.BACKLASH_RAD
    zeta: float = 0.05
    soft_w: float = 1e-5             # softplus width of the backlash edge, rad
    wn_dt_max: float = 0.3
    kp: object = None                # pd gains, default = vendor servo gains
    kd: object = None
    log: bool = True


def softdead(x, h, w):
    return w * (np.logaddexp(0., (x - h) / w) - np.logaddexp(0., (-x - h) / w))


class GimArm:
    def __init__(self, cfg=None, **kw):
        self.cfg = cfg = cfg or Config(**kw)
        assert cfg.transmission in ('rigid', 'compliant') and cfg.mode in ('pd', 'torque')
        assert abs(cfg.dt / DT_E - round(cfg.dt / DT_E)) < 1e-9 and abs(1e-3 / cfg.dt - round(1e-3 / cfg.dt)) < 1e-9
        self.P = G.arm_params(cfg.eta, cfg.lambda_source)
        self.warnings = []
        if cfg.vbus > 40:
            self.warnings.append(f'bus {cfg.vbus} V exceeds the 40 V rating of the GIM4310 (J5-J7)')
        self.compliant = cfg.transmission == 'compliant'
        self.n_sub = round(cfg.dt / DT_E)
        self.fw_every = max(1, round(1 / (cfg.drive_hz * DT_E)))
        self.a_i = 1 - np.exp(-cfg.wc * DT_E)
        self.a_v = 1 - np.exp(-2 * np.pi * cfg.v_cut_hz / cfg.drive_hz)
        self.vmax = 0.95 / np.sqrt(3) * cfg.vbus
        self.etahat_kt = self.P['N'] * self.P['kt_m'] * cfg.eta_hat
        self.lsb_enc = 2 * np.pi / 2 ** cfg.enc_bits
        self.lsb_out = 2 * np.pi / 2 ** cfg.out_enc_bits
        self._build()
        self.reset()

    # ------------------------------------------------------------------ model
    def _build(self):
        cfg, P = self.cfg, self.P
        spec = mj.MjSpec.from_file(str(robot.CONTACT_MODEL if cfg.contact else robot.MODEL))
        for k in list(spec.keys):
            spec.delete(k)
        ref = spec.compile()
        if self.compliant:
            for name in robot.ARMS:
                jid = ref.joint(name).id
                bid = int(ref.jnt_bodyid[jid])
                parent = spec.body(ref.body(int(ref.body_parentid[bid])).name)
                rotor = parent.add_body(name=name + '_rotor', pos=ref.body_pos[bid], quat=ref.body_quat[bid],
                                        mass=1e-6, inertia=[1e-9] * 3, ipos=[0, 0, 0])
                rotor.add_joint(name=name + '_rotor', type=mj.mjtJoint.mjJNT_HINGE, axis=ref.jnt_axis[jid],
                                pos=ref.jnt_pos[jid])
        spec.option.timestep = cfg.dt
        vm, vd = robot.load(contact=cfg.contact, step=cfg.dt)
        m = self.m = spec.compile()
        self.link_dof = np.array([m.joint(n).dofadr[0] for n in robot.ARMS])
        self.link_qadr = np.array([m.joint(n).qposadr[0] for n in robot.ARMS])
        if self.compliant:
            self.rot_dof = np.array([m.joint(n + '_rotor').dofadr[0] for n in robot.ARMS])
            self.rot_qadr = np.array([m.joint(n + '_rotor').qposadr[0] for n in robot.ARMS])
        else:
            self.rot_dof, self.rot_qadr = self.link_dof, self.link_qadr
        self.child = np.array([m.jnt_bodyid[m.joint(n).id] for n in robot.ARMS])
        self.arm_act = np.array([m.actuator(f"{n.split('_')[1]}_joint{n[-1]}_ctrl").id for n in robot.ARMS])
        self.fing_act = np.array([m.actuator(f'{s}_finger1_ctrl').id for s in ('left', 'right')])
        self.kp_v = m.actuator_gainprm[self.arm_act, 0].copy()          # vendor gains, then switch the servos off
        self.kd_v = -m.actuator_biasprm[self.arm_act, 2].copy()
        self.force_range_v = m.actuator_forcerange[self.arm_act].copy()
        m.actuator_gainprm[self.arm_act, :3] = 0
        m.actuator_biasprm[self.arm_act, :3] = 0
        self.kp = self.kp_v if cfg.kp is None else np.asarray(cfg.kp, float)
        self.kd = self.kd_v if cfg.kd is None else np.asarray(cfg.kd, float)
        if cfg.gim_masses:
            for b, dm in zip(self.child, np.maximum(P['motor_mass'] - P['stock_mass'], 0)):
                m0 = m.body_mass[b]
                m.body_ipos[b] *= m0 / (m0 + dm)
                m.body_mass[b] = m0 + dm
        jr = P['J_rotor'] * P['N'] ** 2
        link, rot = self.link_dof, self.rot_dof
        m.dof_armature[link] = 0 if self.compliant else jr
        for dofs in (link, rot):
            m.dof_damping[dofs] = 0
            m.dof_frictionloss[dofs] = 0
        if self.compliant:
            m.dof_armature[rot] = jr
        if cfg.friction:
            m.dof_damping[rot] = P['b']
            m.dof_frictionloss[rot] = P['tau_c']
            m.dof_solref[rot] = [0.002, 1]            # stiffer friction constraint: default 0.02 s creeps 1.4 mrad/s under load
        self.d = d = mj.MjData(m)
        mj.mj_setConst(m, d)                           # resets d to qpos0, so do it before copying the state
        free, ball = int(mj.mjtJoint.mjJNT_FREE), int(mj.mjtJoint.mjJNT_BALL)
        for j in range(vm.njnt):                       # copy initial state by joint name
            vj = vm.joint(j)
            n, nv = {free: (7, 6), ball: (4, 3)}.get(int(vm.jnt_type[j]), (1, 1))
            tj = m.joint(vj.name)
            d.qpos[tj.qposadr[0]:tj.qposadr[0] + n] = vd.qpos[vj.qposadr[0]:vj.qposadr[0] + n]
            d.qvel[tj.dofadr[0]:tj.dofadr[0] + nv] = vd.qvel[vj.dofadr[0]:vj.dofadr[0] + nv]
        for i in range(vm.nu):
            d.ctrl[m.actuator(vm.actuator(i).name).id] = vd.ctrl[i]
        if self.compliant:
            d.qpos[self.rot_qadr] = d.qpos[self.link_qadr]
        mj.mj_forward(m, d)
        self._init_state = (d.qpos.copy(), d.qvel.copy(), d.ctrl.copy())
        self._gearbox_numbers()

    def _gearbox_numbers(self):
        """Link inertia, gearbox damping, omega_n*dt and the gearbox force substep count."""
        cfg, P, m, d = self.cfg, self.P, self.m, self.d
        self.n_g, self.c_g, self.wn_dt, self.i_link = 1, np.zeros(14), np.zeros(14), np.zeros(14)
        if not self.compliant:
            return
        rng = np.random.default_rng(0)
        lo, hi = m.jnt_range[[m.joint(n).id for n in robot.ARMS]].T
        keep = d.qpos.copy()
        i_min = np.full(14, np.inf)
        for k in range(40):
            d.qpos[self.link_qadr] = keep[self.link_qadr] if k == 0 else rng.uniform(lo, hi)
            d.qpos[self.rot_qadr] = d.qpos[self.link_qadr]
            mj.mj_forward(m, d)
            M = np.zeros((m.nv, m.nv))
            mj.mj_fullM(m, M, d.qM)
            i_min = np.minimum(i_min, np.diag(M)[self.link_dof])
        d.qpos[:] = keep
        mj.mj_forward(m, d)
        jr = P['J_rotor'] * P['N'] ** 2
        self.i_link = i_min
        self.c_g = 2 * cfg.zeta * np.sqrt(P['k_g'] * jr * i_min / (jr + i_min))
        self.wn = np.sqrt(P['k_g'] * (1 / jr + 1 / i_min))
        self.wn_dt = self.wn * cfg.dt
        self.n_g = int(np.ceil(self.wn_dt.max() / cfg.wn_dt_max))
        m.opt.timestep = cfg.dt / self.n_g

    # ------------------------------------------------------------------ state
    def reset(self):
        q0, v0, c0 = self._init_state
        d = self.d
        d.qpos[:], d.qvel[:], d.ctrl[:] = q0, v0, c0
        d.qfrc_applied[:] = 0
        d.time = 0.
        mj.mj_forward(self.m, d)
        self.ext = np.zeros(self.m.nv)
        z = np.zeros(14)
        self.iq, self.iq_cmd, self.tau_cmd, self.i_meas = z.copy(), z.copy(), z.copy(), z.copy()
        self.sub_clock = 0
        self.rows = []
        self.last = {}
        self.reset_cmd()

    def reset_cmd(self):
        th = self.d.qpos[self.rot_qadr].copy()
        self.cmd = dict(q_ref=th, v_ref=np.zeros(14), tau_ff=np.zeros(14), tau_cmd=np.zeros(14))
        self.queue = deque([dict(self.cmd) for _ in range(self.cfg.delay_ms)])
        self.q_meas = np.round(self.P['N'] * th / self.lsb_enc) * self.lsb_enc / self.P['N']
        self.q_prev, self.v_meas = self.q_meas.copy(), np.zeros(14)
        self.iq_cmd = np.zeros(14)

    def set_arm_q(self, q):
        d = self.d
        d.qpos[self.link_qadr] = q
        d.qpos[self.rot_qadr] = q
        d.qvel[:] = 0
        mj.mj_forward(self.m, d)
        self.reset_cmd()

    @property
    def q(self):
        return self.d.qpos[self.link_qadr].copy()

    @property
    def v(self):
        return self.d.qvel[self.link_dof].copy()

    @property
    def theta_r(self):
        return self.d.qpos[self.rot_qadr].copy()

    def q_out_meas(self):
        return np.round(self.q / self.lsb_out) * self.lsb_out

    # ------------------------------------------------------------------ electrical
    def volt_limits(self, w):
        """Admissible iq interval at output speed w (id = 0, |v| <= vmax) and the electrical speed."""
        P = self.P
        we = P['pp'] * P['N'] * w
        A = P['R'] ** 2 + (we * P['L']) ** 2
        B = P['R'] * we * P['lam']
        D = A * self.vmax ** 2 - (we ** 2 * P['L'] * P['lam']) ** 2
        s = np.sqrt(np.maximum(D, 0.))
        return (-B - s) / A, (-B + s) / A, we

    def _firmware(self, th):
        P, cfg = self.P, self.cfg
        q = np.round(P['N'] * th / self.lsb_enc) * self.lsb_enc / P['N']
        self.v_meas = self.v_meas + self.a_v * ((q - self.q_prev) * cfg.drive_hz - self.v_meas)
        self.q_prev = self.q_meas = q
        c = self.cmd
        if cfg.mode == 'pd':
            self.tau_cmd = self.kp * (c['q_ref'] - q) + self.kd * (c['v_ref'] - self.v_meas) + c['tau_ff']
        else:
            self.tau_cmd = c['tau_cmd'].copy()
        self.iq_cmd = self.tau_cmd / self.etahat_kt

    # ------------------------------------------------------------------ stepping
    def step_ms(self, cmd=None, fingers=None):
        """Advance 1 ms. cmd: dict of 14-vectors (pd: q_ref, v_ref, tau_ff; torque: tau_cmd), held until replaced
        and delayed by cfg.delay_ms; fingers: 2 vendor position ctrl values (left, right finger1)."""
        new = dict(self.cmd if not self.queue else self.queue[-1])
        for k, v in (cmd or {}).items():
            new[k] = np.asarray(v, float).copy()
        self.queue.append(new)
        self.cmd = self.queue.popleft()
        if fingers is not None:
            self.d.ctrl[self.fing_act] = fingers
        for _ in range(round(1e-3 / self.cfg.dt)):
            self._phys_step()

    def _phys_step(self):
        cfg, P, d, m = self.cfg, self.P, self.d, self.m
        w, th = d.qvel[self.rot_dof].copy(), d.qpos[self.rot_qadr].copy()
        lo, hi, we = self.volt_limits(w)
        imax, iq, a = P['i_max'], self.iq, self.a_i
        s_iq = s_viq = s_i2 = 0.
        flag = np.zeros(14, bool)
        for s in range(self.n_sub):
            if self.sub_clock % self.fw_every == 0:
                self._firmware(th + w * (s * DT_E))
            self.sub_clock += 1
            want = np.clip(self.iq_cmd, -imax, imax)
            tar = np.clip(want, lo, hi)
            new = np.clip(iq + a * (tar - iq), lo, hi)
            vq = P['R'] * new + we * P['lam'] + P['L'] * (new - iq) / DT_E
            flag |= tar != want
            s_iq = s_iq + new
            s_viq = s_viq + vq * new
            s_i2 = s_i2 + new * new
            iq = new
        self.iq = iq
        n = self.n_sub
        iq_m = s_iq / n
        self.i_meas = np.round(iq / cfg.i_lsb) * cfg.i_lsb
        # gear loss as load-dependent Coulomb friction (set-valued at standstill, solved by MuJoCo):
        # motoring output = eta * N*kt_m*iq = listed Kt * iq; static holding gets the friction band on top
        tau_m = P['N'] * P['kt_m'] * iq_m
        if cfg.friction:
            m.dof_frictionloss[self.rot_dof] = P['tau_c'] + (1 - P['eta']) * np.abs(tau_m)
        p_dc = 1.5 * s_viq / n + 1.5 * (s_i2 / n) * G.R_INVERTER
        tg_sum = np.zeros(14)
        for _ in range(self.n_g):
            d.qfrc_applied[:] = self.ext
            if self.compliant:
                x = d.qpos[self.rot_qadr] - d.qpos[self.link_qadr]
                tg = P['k_g'] * softdead(x, cfg.backlash / 2, cfg.soft_w) + \
                    self.c_g * (d.qvel[self.rot_dof] - d.qvel[self.link_dof])
                d.qfrc_applied[self.rot_dof] += tau_m - tg
                d.qfrc_applied[self.link_dof] += tg
                tg_sum += tg
            else:
                d.qfrc_applied[self.link_dof] += tau_m
            mj.mj_step(m, d)
        self.last = dict(tau_out=tau_m, tau_gear=tg_sum / self.n_g, iq=iq_m, p_dc=p_dc, vlim=flag)
        if cfg.log:
            self.rows.append(dict(t=d.time, q=self.q, v=self.v, theta_r=self.theta_r, iq=iq_m,
                                  iq_cmd=self.iq_cmd.copy(), tau_out=tau_m, tau_gear=tg_sum / self.n_g,
                                  p_dc=p_dc, vlim=flag, q_meas=self.q_meas.copy(), v_meas=self.v_meas.copy(),
                                  i_meas=self.i_meas.copy(), tau_cmd=self.tau_cmd.copy()))

    def log(self):
        return {k: np.array([r[k] for r in self.rows]) for k in self.rows[0]} if self.rows else {}

    def spring_energy(self):
        if not self.compliant:
            return 0.
        x = self.d.qpos[self.rot_qadr] - self.d.qpos[self.link_qadr]
        return float(np.sum(0.5 * self.P['k_g'] * softdead(x, self.cfg.backlash / 2, self.cfg.soft_w) ** 2))

    def gravity_torque(self):
        """Link-dof torque that holds the current pose statically (friction ignored)."""
        d = self.d
        qv = d.qvel.copy()
        d.qvel[:] = 0
        mj.mj_forward(self.m, d)
        t = d.qfrc_bias[self.link_dof].copy()
        d.qvel[:] = qv
        mj.mj_forward(self.m, d)
        return t

    def preload(self, tau):
        """Torque mode: start with command, firmware and current loop already in steady state at `tau` (14-vector)."""
        tau = np.asarray(tau, float)
        for c in [self.cmd, *self.queue]:
            c['tau_cmd'] = tau.copy()
        self.tau_cmd = tau.copy()
        self.iq_cmd = tau / self.etahat_kt
        self.iq = np.clip(self.iq_cmd, -self.P['i_max'], self.P['i_max'])
