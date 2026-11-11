"""Actuator-set extension of the GIM rig: any set of 15 actuators (Torso + Left/Right Arm 1..7) loaded from
designs/*.json (exported from the per_joint study), actuator mass change at each joint, torso brake lock.
rig.py, scen.py etc. are unchanged; this file subclasses Rig.

Mass method (as the OpenArm GIM layer's gim_masses option, but signed): for each actuated joint a point mass
dm = m_set - m_stock is added at the origin of the joint's child body (the body that carries the joint). The body
mass, centre of mass and inertia tensor are recombined exactly (parallel-axis theorem) and mj_setConst refreshes the
derived constants, so the gravity/inertia feedforward (a copy of the edited model) sees the same masses.
m_stock is the mass of the actuator that MABEL's MJCF was built with (DM-J8009P 0.896 kg on J1-J2, DM-J4340 0.362 on
J3-J4, DM-J4310 0.30 on J5-J7, published with driver; torso DM-J10422P 2.7 kg from mabel_full.xml via per_joint).
m_set is the whole actuator mass of the set (for the designs: the per_joint total including gearbox m_gear,
housing and board). The gearbox mass is therefore inside dm.
"""
import copy
import json
import sys
from pathlib import Path

import mujoco as mj
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import rig as R0  # noqa: E402
from rig import JN, IDX, Config, Rig, XML, SWERVE_ACT, TORSO_KP, TORSO_KD  # noqa: E402
import gim_params as G  # noqa: E402

import os
DESIGNS = Path(os.environ.get('MABEL_DESIGNS', str(HERE / 'designs')))
STOCK_MASS_MABEL = np.array([0.896, 0.896, 0.362, 0.362, 0.30, 0.30, 0.30])    # Damiao in the MJCF (published, with driver)
TORSO_STOCK_MASS = 2.7                                                          # DM-J10422P-2EC
GIM_MASS_DRIVER = np.array([0.96, 0.96, 0.76, 0.76, 0.3645, 0.3645, 0.3645])    # GIM with driver (per_joint gearbox_data)
JOINT_GROUPS = {'Torso': [0], 'Arm1-2': [1, 2, 8, 9], 'Arm3-4': [3, 4, 10, 11], 'Arm5-7': [5, 6, 7, 12, 13, 14]}
KEYS = ['Torso'] + [f'Arm{i}' for i in range(1, 8)] * 2


def stock_set(masses=True):
    """The GIM set the rig has always used, optionally with the signed mass change relative to the Damiao in the MJCF."""
    P = {k: v[IDX] for k, v in G.arm_params(0.9, 'kt').items()}
    P['kt_out'] = G.TILE(G.KT_OUT)[IDX]
    m_set = np.concatenate([[GIM_MASS_DRIVER[0]], GIM_MASS_DRIVER, GIM_MASS_DRIVER])      # torso uses GIM8115
    m_stock = np.concatenate([[TORSO_STOCK_MASS], STOCK_MASS_MABEL, STOCK_MASS_MABEL])
    P['dm'] = (m_set - m_stock) if masses else np.zeros(15)
    P['m_set'], P['m_stock'] = m_set, m_stock
    P['gear'] = ['planetary2'] * 15
    P['w_design'] = np.zeros(15)
    return dict(name='stock_gim' if masses else 'stock_gim_nomass', P=P, meta=dict(kind='stock GIM', masses=masses))


def design_set(name, inertia='cad'):
    """Build the rig parameter dict from designs/<name>.json (see export_params.py for the meaning of each field)."""
    J = json.loads((DESIGNS / f'{name}.json').read_text())
    assert inertia in ('cad', 'model')
    P = {}
    cols = ['N', 'eta', 'kt_m', 'lam', 'pp', 'R', 'L', 'i_max', 'J_rotor']
    for c in cols:
        P[c] = np.array([J['joints'][k][c] for k in KEYS], float)
    if inertia == 'cad':      # armature = CAD rotor + gearbox input, reflected to the output; and the CAD inertia tensor of the whole actuator
        P['J_rotor'] = np.array([J['joints'][k]['cad']['reflected_inertia_output_cad_kgm2'] for k in KEYS], float) / P['N'] ** 2
        P['I_lump'] = np.array([J['joints'][k]['cad']['inertia_about_com_kgm2'] for k in KEYS], float)
        assert all(abs(J['joints'][k]['cad']['mass_kg'] - J['joints'][k]['mass_kg']) < 1e-6 for k in KEYS)
    else:
        P['I_lump'] = None
    P['tau_c'] = np.array([J['joints'][k]['tau_coulomb_out'] for k in KEYS], float)
    P['k_g'] = np.array([J['joints'][k]['gearbox']['k_gear_Nm_rad'] for k in KEYS], float)
    P['backlash'] = np.array([J['joints'][k]['gearbox']['backlash_rad'] for k in KEYS], float)
    # viscous term ASSUMED: same b / tau_c ratio as the stock GIM of the joint class (the per_joint code has no viscous term)
    ratio = np.concatenate([[G.B_VISC[0] / G.TAU_COULOMB[0]], G.B_VISC / G.TAU_COULOMB, G.B_VISC / G.TAU_COULOMB])
    P['b'] = P['tau_c'] * ratio
    P['kt_out'] = P['N'] * P['kt_m'] * P['eta']                      # delivered torque per A while moving (as the GIM KT_OUT)
    P['tau_max'] = P['kt_out'] * P['i_max']
    m_set = np.array([J['joints'][k]['mass_kg'] for k in KEYS], float)
    m_stock = np.concatenate([[TORSO_STOCK_MASS], STOCK_MASS_MABEL, STOCK_MASS_MABEL])
    P['dm'] = m_set - m_stock
    P['m_set'], P['m_stock'] = m_set, m_stock
    P['gear'] = [J['joints'][k]['gearbox']['type'] for k in KEYS]
    P['w_design'] = np.array([J['joints'][k]['w_out_design_peak'] for k in KEYS], float)
    return dict(name=name, P=P, meta=J['provenance'] | dict(kind='design', magnet=J['magnet'], case=J['case'], inertia=inertia))


def all_sets():
    names = ['main_ndfeb', 'main_ferrite']
    for case in ('mass_50', 'mass_100', 'size', 'backdrive', 'qdd'):
        names += [f'{case}_ndfeb', f'{case}_ferrite']
    return ['stock_gim'] + names


INERTIA_MODE = 'cad'      # 'cad' (default now) or 'model' (first run, 20261105_112812_sets)


def get_set(name):
    if name == 'stock_gim':
        return stock_set(True)
    if name == 'stock_gim_nomass':
        return stock_set(False)
    if name == 'stock_gim_nominal':        # geometric estimate, 5 Nov, +/-30 %: GIM8115 0.25, GIM8108 0.14 kg m2 reflected; GIM4310 as before
        a = stock_set(True)
        Jm = np.array([0.25, 0.25, 0.14, 0.14, 0.032, 0.032, 0.032])
        a['P']['J_rotor'] = np.concatenate([[Jm[0]], Jm, Jm]) / a['P']['N'] ** 2
        a['name'] = name
        return a
    if name == 'stock_gim_pjinertia':      # stock GIM with the reflected inertia the per_joint code models (0.318, 0.170, 0.030 kg m2)
        a = stock_set(True)
        Jm = np.array([0.318, 0.318, 0.1696, 0.1696, 0.03007, 0.03007, 0.03007])
        Jm15 = np.concatenate([[Jm[0]], Jm, Jm])
        a['P']['J_rotor'] = Jm15 / a['P']['N'] ** 2
        a['name'] = name
        return a
    return design_set(name, INERTIA_MODE)


def add_point_mass_at_origin(m, b, dm, I_lump=None):
    """Recombine body b with a lump of mass dm (may be negative) whose centre of mass is at the body-frame origin.
    I_lump (3x3, about its own COM, axes parallel to the body frame) is added if given; None means a point mass."""
    m1 = m.body_mass[b]
    m2 = m1 + dm
    assert m2 > 1e-3, f'body {m.body(b).name} would get mass {m2}'
    c1 = m.body_ipos[b].copy()
    q = m.body_iquat[b].copy()
    Rm = np.zeros(9)
    mj.mju_quat2Mat(Rm, q)
    Rm = Rm.reshape(3, 3)
    I1 = Rm @ np.diag(m.body_inertia[b]) @ Rm.T
    c = m1 * c1 / m2
    d1, d2 = c1 - c, -c
    I = I1 + m1 * (d1 @ d1 * np.eye(3) - np.outer(d1, d1)) + dm * (d2 @ d2 * np.eye(3) - np.outer(d2, d2))
    if I_lump is not None:
        I = I + np.asarray(I_lump)
    w, V = np.linalg.eigh(I)
    if w.min() <= 1e-9 or not np.all(np.isfinite(w)):
        # removing a point mass at the origin is not physical for this body (its inertia is not that of a lump there):
        # remove the mass by scaling the body's own mass distribution instead (mass and inertia times m2/m1)
        m.body_mass[b] = m2
        m.body_inertia[b] = m.body_inertia[b] * (m2 / m1)
        return 'scaled'
    if np.linalg.det(V) < 0:
        V[:, 0] *= -1
    qn = np.zeros(4)
    mj.mju_mat2Quat(qn, V.flatten())
    m.body_mass[b], m.body_ipos[b], m.body_inertia[b], m.body_iquat[b] = m2, c, w, qn
    return 'point' if I_lump is None else 'lump'


class SetRig(Rig):
    """Rig with a chosen actuator set. Same electrical/firmware model as Rig; parameters come from the set."""

    def __init__(self, aset, payload=0., cfg=None, brake_eq=True):
        self.aset = aset
        self.cfg = cfg = cfg or Config()
        self.actuation, self.payload = 'gim', payload
        self.P = {k: (v.copy() if isinstance(v, np.ndarray) else v) for k, v in aset['P'].items()}
        self.n_sub = round(cfg.dt / R0.DT_E)
        self.fw_every = max(1, round(1 / (cfg.drive_hz * R0.DT_E)))
        self.a_i = 1 - np.exp(-cfg.wc * R0.DT_E)
        self.a_v = 1 - np.exp(-2 * np.pi * cfg.v_cut_hz / cfg.drive_hz)
        self.vmax = 0.95 / np.sqrt(3) * cfg.vbus
        self.etahat_kt = self.P['N'] * self.P['kt_m'] * cfg.eta_hat
        self.lsb_enc = 2 * np.pi / 2 ** cfg.enc_bits
        self.torso_motor_off = False
        self.brake_eq = brake_eq
        self._build()

    # ---------------------------------------------------------------- model
    def _build(self):
        cfg, P = self.cfg, self.P
        spec = mj.MjSpec.from_file(XML)
        for s in list(spec.sensors):
            spec.delete(s)
        spec.delete(spec.joint('base_root'))
        spec.option.timestep = cfg.dt
        spec.option.disableflags |= mj.mjtDisableBit.mjDSBL_CONTACT
        if self.payload > 0:
            for palm in ('left_palm', 'right_palm'):
                b = spec.body(palm).add_body(name=f'payload_{palm}', pos=[0, 0, 0])
                b.add_geom(type=mj.mjtGeom.mjGEOM_SPHERE, size=[0.03, 0, 0], contype=0, conaffinity=0, mass=self.payload)
        # torso brake: joint equality q_torso = const, switched on and off at run time
        eq = spec.add_equality()
        eq.type = mj.mjtEq.mjEQ_JOINT
        eq.name = 'torso_brake'
        eq.name1 = 'Torso'
        eq.data[:5] = [0, 0, 0, 0, 0]
        eq.solref = [0.004, 1]
        eq.active = False
        m = self.m = spec.compile()
        self.d = d = mj.MjData(m)
        self.eq_id = m.equality('torso_brake').id
        self.dof = np.array([m.joint(n).dofadr[0] for n in JN])
        self.qadr = np.array([m.joint(n).qposadr[0] for n in JN])
        self.act = np.array([m.actuator(n).id for n in JN])
        self.lift_act = m.actuator('Lift').id
        self.lift_q = [m.joint('Lift Lower').qposadr[0], m.joint('Lift Upper').qposadr[0]]
        self.lift_dof = [m.joint('Lift Lower').dofadr[0], m.joint('Lift Upper').dofadr[0]]
        self.lo, self.hi = m.jnt_range[[m.joint(n).id for n in JN]].T
        for n in SWERVE_ACT:
            a = m.actuator(n).id
            m.actuator_gainprm[a, :3] = 0
            m.actuator_biasprm[a, :3] = 0
        self.kp_stock = m.actuator_gainprm[self.act, 0].copy()
        self.kd_stock = -m.actuator_biasprm[self.act, 2].copy()
        self.frange_stock = m.actuator_forcerange[self.act].copy()
        m.actuator_gainprm[self.act, :3] = 0
        m.actuator_biasprm[self.act, :3] = 0
        m.actuator_forcerange[self.act] = 0
        m.actuator_forcelimited[self.act] = 0
        # reflected rotor inertia (the gearbox own inertia is not modelled by the per_joint code), friction
        m.dof_armature[self.dof] = P['J_rotor'] * P['N'] ** 2
        m.dof_damping[self.dof] = P['b'] if cfg.friction else 0
        m.dof_frictionloss[self.dof] = P['tau_c'] if cfg.friction else 0
        m.dof_solref[self.dof] = [0.002, 1]
        # actuator mass change at the child body of each joint
        self.body_ids = [int(m.jnt_bodyid[m.joint(n).id]) for n in JN]
        self.mass_before = np.array([m.body_mass[b] for b in self.body_ids])
        self.mass_method = []
        for jj, (b, dm) in enumerate(zip(self.body_ids, P['dm'])):
            IL = P.get('I_lump')
            m0m, m0i, m0p, m0q = m.body_mass[b], m.body_inertia[b].copy(), m.body_ipos[b].copy(), m.body_iquat[b].copy()
            if IL is None:
                self.mass_method.append(add_point_mass_at_origin(m, b, float(dm)) if dm != 0 else 'none')
            else:
                # net change as one lump at the origin: mass m_set - m_stock with the CAD tensor of the new actuator.
                # Orientation: the CAD actuator is axisymmetric about its axis z; z is put on the joint axis (local z of the
                # child body, all MABEL joint axes are [0 0 1]). The axial COM offset (0 to 30 mm) has no sign reference
                # in the CAD index, so the lump COM is at the child body origin.
                r = add_point_mass_at_origin(m, b, float(dm), IL[jj])
                if r == 'scaled':    # (torso) the point removal was not physical: undo, scale away the stock mass, add the lump
                    m.body_mass[b], m.body_inertia[b] = m0m, m0i
                    m.body_ipos[b], m.body_iquat[b] = m0p, m0q
                    k = (m.body_mass[b] - float(P['m_stock'][jj])) / m.body_mass[b]
                    m.body_mass[b] *= k
                    m.body_inertia[b] = m.body_inertia[b] * k
                    add_point_mass_at_origin(m, b, float(P['m_set'][jj]), IL[jj])
                    r = 'scaled-stock+lump'
                self.mass_method.append(r)
        self.mass_after = np.array([m.body_mass[b] for b in self.body_ids])
        mj.mj_setConst(m, d)
        self.total_mass = float(m.body_mass.sum())
        kp = self.kp_stock.copy() if cfg.kp is None else np.asarray(cfg.kp, float)
        kd = self.kd_stock.copy() if cfg.kd is None else np.asarray(cfg.kd, float)
        if cfg.kp is None:
            kp[0], kd[0] = TORSO_KP, TORSO_KD
        self.kp, self.kd = kp, kd
        self.m_id = copy.copy(m)
        self.m_id.opt.disableflags |= mj.mjtDisableBit.mjDSBL_CONSTRAINT
        self.d_id = mj.MjData(self.m_id)
        self.q_default = d.qpos.copy()
        self.no_load_speed = self.vmax / (P['pp'] * P['N'] * P['lam'])

    # ---------------------------------------------------------------- helpers
    def envelope(self, w):
        lo, hi, _ = self.volt_limits(np.full(15, w))
        i = np.minimum(hi, self.P['i_max'])
        return np.maximum(i, 0) * self.P['kt_out']

    def set_pose(self, q15, lift=0.3175, lift_ctrl=None):
        super().set_pose(q15, lift, lift_ctrl)
        self.d.eq_active[self.eq_id] = 0
        self.torso_motor_off = False

    def brake(self, on):
        """Lock the torso joint where it is (on) or release it."""
        d, m = self.d, self.m
        if on and not d.eq_active[self.eq_id]:
            m.eq_data[self.eq_id, 0] = d.qpos[self.qadr[0]]
            d.eq_active[self.eq_id] = 1
        elif not on:
            d.eq_active[self.eq_id] = 0

    def _firmware(self, th):
        super()._firmware(th)
        if self.torso_motor_off:
            self.iq_cmd = self.iq_cmd.copy()
            self.iq_cmd[0] = 0.
