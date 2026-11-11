"""Detailed-plant side of the two-joint arm study: two virtual actuators (actuator.py) driving the
planar arm of arm_task.py, with a contact puck. See arm_task.detailed_xml for the model layout.

qpos = [r1, r2, q1, q2, px, py]; dofs 0,1 are the rotors, 2,3 the arm, 4,5 the puck.
The command reference time constant defaults to 30 ms (the arm training reference); override
with the T_REF_S environment variable before importing this module.
VEL_HZ=1000 (environment, default 50) makes deploy() estimate joint velocity every 1 ms from the quantized encoder angle
(first-order filtered difference, 50 Hz cutoff) instead of the 20 ms difference.
"""
import math
import os
from dataclasses import replace
import numpy as np

os.environ.setdefault('T_REF_S', '0.030')
VEL_HZ = int(os.environ.get('VEL_HZ', '50'))
assert VEL_HZ in (50, 1000), VEL_HZ
VEL_A = math.exp(-2 * math.pi * 50.0 * 1e-3)
import mujoco as mj
import actuator as A
from actuator import (NOMINAL, HELD_OUT, Gains, FrictionMap, LocalController, Driver, stribeck_hump,
                      transmitted, DT_LOCAL, DT_ELEC)
import arm_task as T


class VirtualArm:
    def __init__(self, task, p1, p2, h=25e-6):
        self.task, self.ps, self.h = task, (p1, p2), h
        if h >= DT_ELEC:
            assert abs(h / DT_ELEC - round(h / DT_ELEC)) < 1e-9
            self.n_elec, self.per_v = round(h / DT_ELEC), 1
        else:
            assert abs(DT_ELEC / h - round(DT_ELEC / h)) < 1e-9
            self.n_elec, self.per_v = 1, round(DT_ELEC / h)
        assert abs(DT_LOCAL / h - round(DT_LOCAL / h)) < 1e-9
        self.n_phys = round(DT_LOCAL / h)
        self.m = mj.MjModel.from_xml_string(T.detailed_xml(task, p1, p2, h))
        self.d = mj.MjData(self.m)
        mj.mj_forward(self.m, self.d)
        self.drv = (Driver(p1), Driver(p2))
        self.peak_shaft = [0.0, 0.0]
        self.t = 0.0

    def shaft(self, j):
        d, p = self.d, self.ps[j]
        return transmitted(float(d.qpos[j] - d.qpos[2 + j]), float(d.qvel[j] - d.qvel[2 + j]), p)

    def step_ms(self, i1, i2):
        d, ps, drv = self.d, self.ps, self.drv
        icmd = (i1, i2)
        for k in range(self.n_phys):
            for j in (0, 1):
                p, w_r = ps[j], float(d.qvel[j])
                if self.per_v == 1:
                    i_mean = 0.0
                    for _ in range(self.n_elec):
                        i_mean += drv[j].substep(icmd[j], p.ratio * w_r)
                    i_mean /= self.n_elec
                else:
                    if k % self.per_v == 0:
                        drv[j].update_voltage(icmd[j])
                    i_mean = drv[j].advance(self.h, p.ratio * w_r)
                tau_m = p.ratio * p.kt * i_mean
                if p.cogging:
                    tau_m += p.cogging * math.sin(12 * p.ratio * float(d.qpos[j]))
                d.qfrc_applied[j] = tau_m - stribeck_hump(w_r, p)
            mj.mj_step(self.m, d)
            for j in (0, 1):
                s = abs(self.shaft(j))
                if s > self.peak_shaft[j]:
                    self.peak_shaft[j] = s
        self.t += DT_LOCAL

    def ok(self):
        d = self.d
        return bool(np.all(np.isfinite(np.r_[d.qpos, d.qvel])) and abs(self.drv[0].i) < 1e3 and abs(self.drv[1].i) < 1e3)

    def tip(self):
        return T.tip_xy(float(self.d.qpos[2]), float(self.d.qpos[3]))


class _LiveData:
    """d.qpos -> live [rotor_j, load_j] (what Sensors/LocalController index as qpos[0], qpos[1])."""
    def __init__(self, arm, j):
        self._arm, self._j = arm, j

    @property
    def qpos(self):
        q = self._arm.d.qpos
        return np.array([q[self._j], q[2 + self._j]])

    @property
    def qvel(self):
        v = self._arm.d.qvel
        return np.array([v[self._j], v[2 + self._j]])


class JointView:
    """Adapter so actuator.Sensors / LocalController run unmodified on joint j of a VirtualArm."""
    def __init__(self, arm, j):
        self.arm, self.j = arm, j
        self.d = _LiveData(arm, j)
        self.drv = arm.drv[j]
        self.p = arm.ps[j]

    def shaft(self):
        return self.arm.shaft(self.j)


# ---------------------------------------------------------------- deployment
def deploy(args):
    name, task, p1, p2, ctl_kind, gains, fmap, theta, init_idx, seed = args
    arm = VirtualArm(task, p1, p2, 25e-6)
    d = arm.d
    q1, q2, px, py = T.INITS[init_idx]
    d.qpos[:] = [q1, q2, q1, q2, px, py]
    mj.mj_forward(arm.m, d)
    kind = 'S' if ctl_kind == 'S' else 'H'
    ctls = [LocalController(kind, Gains(**gains), FrictionMap(**fmap), 'torque', 2 * seed + j) for j in (0, 1)]
    views = [JointView(arm, j) for j in (0, 1)]
    rng = np.random.default_rng(seed)             # puck position noise
    rng_j = np.random.default_rng(seed + 10_000)  # command jitter
    enc = 2 * np.pi / 16384
    qp = None; pp = None; w = np.zeros(2); v = np.zeros(2)
    u = np.zeros(2); ret = 0.0; tail = []
    pending = []                 # (arrival_ms, command pair)
    cur = np.zeros(2); min_d = float('inf')
    nan = float('nan')
    for k in range(T.N_STEPS):
        qm = np.round(np.array([d.qpos[2], d.qpos[3]]) / enc) * enc
        pn = np.array([d.qpos[4], d.qpos[5]]) + rng.normal(0, 0.001, 2)
        if k == 0:
            w[:] = 0.0; v[:] = 0.0
            qp = qm                  # VEL_HZ=1000: joint filter runs in the 1 ms loop below, starting from the start pose
        else:
            if VEL_HZ == 50:
                w = 0.5 * w + 0.5 * (qm - qp) / T.DT_TASK
            v = 0.5 * v + 0.5 * (pn - pp) / T.DT_TASK
        if VEL_HZ == 50:
            qp = qm
        pp = pn
        obs = T.observe(qm[0], qm[1], w[0], w[1], pn[0], pn[1], v[0], v[1], u[0], u[1])
        u = np.asarray(T.act(theta, obs), dtype=float)
        dly = p1.delay_ms + (int(rng_j.integers(0, p1.jitter_ms + 1)) if p1.jitter_ms else 0)
        pending.append((k * 20 + dly, u.copy()))
        pending.sort(key=lambda x: x[0])
        for j in range(20):
            t_ms = k * 20 + j
            while pending and pending[0][0] <= t_ms:
                cur = pending.pop(0)[1]
            ic1, _ = ctls[0](cur[0], views[0])
            ic2, _ = ctls[1](cur[1], views[1])
            arm.step_ms(ic1, ic2)
            if VEL_HZ == 1000:
                qm1 = np.round(np.array([d.qpos[2], d.qpos[3]]) / enc) * enc
                w = VEL_A * w + (1.0 - VEL_A) * (qm1 - qp) / 1e-3
                qp = qm1
            if not arm.ok():
                return dict(plant=name, ok=False, success=False, ret=nan, final_dist=nan,
                            min_tip_puck_dist=min_d, peak_shaft1=arm.peak_shaft[0], peak_shaft2=arm.peak_shaft[1])
            tx, ty = arm.tip()
            dd = math.hypot(float(d.qpos[4]) - tx, float(d.qpos[5]) - ty)
            if dd < min_d:
                min_d = dd
        ret += T.reward(float(d.qpos[2]), float(d.qpos[3]), float(d.qvel[2]), float(d.qvel[3]),
                        float(d.qpos[4]), float(d.qpos[5]), float(u[0]), float(u[1]))
        if k >= T.N_STEPS - T.TAIL_STEPS:
            tail.append(T.goal_dist(float(d.qpos[4]), float(d.qpos[5])))
    return dict(plant=name, ok=True, success=bool(max(tail) < T.SUCCESS_TOL), ret=ret, final_dist=float(tail[-1]),
                min_tip_puck_dist=min_d, peak_shaft1=arm.peak_shaft[0], peak_shaft2=arm.peak_shaft[1])


# ---------------------------------------------------------------- plant sets: (name, task, p1, p2)
def nominal():
    return [('nominal', T.NOMINAL_TASK, NOMINAL, NOMINAL)]


def held_out():
    pay = {'load_light': 0.05, 'load_heavy': 0.45, 'combined': 0.3}
    out = []
    for p in HELD_OUT:
        task = replace(T.NOMINAL_TASK, payload=pay[p.name]) if p.name in pay else T.NOMINAL_TASK
        out.append((p.name, task, p, p))
    return out


def new_plants(n=30):
    from exp6_confirm import draw_plants
    ps = draw_plants(2 * n, seed=2026)
    rng = np.random.default_rng(2027)
    out = []
    for i in range(n):
        task = T.Task(payload=float(rng.uniform(*T.RAND['payload'])), puck_mass=float(rng.uniform(*T.RAND['puck_mass'])),
                      puck_mu=float(rng.uniform(*T.RAND['puck_mu'])))
        out.append((f'new_{i:02d}', task, ps[2 * i], ps[2 * i + 1]))
    return out


def out_of_range():
    N, NT = NOMINAL, T.NOMINAL_TASK
    fr = dict(coulomb=0.2, stribeck=0.1, viscous=0.04)
    soft = dict(stiffness=120.0, damping=0.12)
    return [
        ('ood_payload_0.8', replace(NT, payload=0.8), N, N),
        ('ood_payload_1.0', replace(NT, payload=1.0), N, N),
        ('ood_puckmass_0.6', replace(NT, puck_mass=0.6), N, N),
        ('ood_mu_0.6', replace(NT, puck_mu=0.6), N, N),
        ('ood_mu_0.05', replace(NT, puck_mu=0.05), N, N),
        ('ood_delay_11ms', NT, replace(N, delay_ms=11), N),
        ('ood_delay_12ms', NT, replace(N, delay_ms=12), N),
        ('ood_jitter_4to12ms', NT, replace(N, delay_ms=4, jitter_ms=8), N),
        ('ood_payload_0.8_delay_10ms', replace(NT, payload=0.8), replace(N, delay_ms=10), N),
        ('ood_puckmass_0.5_mu_0.5', replace(NT, puck_mass=0.5, puck_mu=0.5), N, N),
        ('ood_payload_0.8_friction', replace(NT, payload=0.8), replace(N, **fr), replace(N, **fr)),
        ('ood_payload_0.8_softgear', replace(NT, payload=0.8), replace(N, **soft), replace(N, **soft)),
    ]


def all_plants():
    """[(plant_set, name, task, p1, p2)] for nominal, held_out, new, out_of_range (53 plants)."""
    sets = (('nominal', nominal()), ('held_out', held_out()), ('new', new_plants()), ('out_of_range', out_of_range()))
    return [(s, *p) for s, ps in sets for p in ps]
