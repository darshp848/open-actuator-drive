"""Training-side actuator models and their torque predictions.

R  prescribed simple reference: first-order lag T_REF on the command delayed by D_NOM_MS.
C  fitted compact model (PACE-style): gain, delay, lag, Coulomb + viscous friction, armature.

Predictions are evaluated at every physics step of a detailed-plant run, using the
same command sequence. Velocity-dependent terms of C are teacher-forced with the
detailed plant's load velocity so the comparison isolates the actuator model.
"""
from dataclasses import dataclass, asdict
import math
import numpy as np
import mujoco as mj
from scipy.signal import lfilter, cont2discrete, ss2tf
from scipy.optimize import lsq_linear
from actuator import DT_LOCAL, DT_TASK, T_REF, D_NOM_MS, TAU_CAP

PER = round(DT_TASK / DT_LOCAL)


def ms_commands(c_task, delay_ms):
    ms = np.repeat(c_task, PER)
    return np.r_[np.zeros(delay_ms), ms[:len(ms) - delay_ms]]


def first_order_fine(u_ms, tau, n_phys, h):
    """Exact response of a first-order lag to a 1 ms zero-order-hold input, sampled after each physics step."""
    a = np.exp(-(np.arange(1, n_phys + 1) * h) / tau)      # decay within one ms at each sample
    a_ms = math.exp(-DT_LOCAL / tau)
    x = lfilter([0.0, 1 - a_ms], [1.0, -a_ms], u_ms)        # state at the start of each ms
    return a[None, :] * x[:, None] + (1 - a)[None, :] * u_ms[:, None]


def reference_prediction(c_task, n_phys, h):
    return first_order_fine(ms_commands(c_task, D_NOM_MS), T_REF, n_phys, h)


@dataclass
class Compact:
    gain: float = 1.0
    tau: float = 0.005
    delay_ms: int = 4
    coulomb: float = 0.0
    viscous: float = 0.0
    armature: float = 0.0

    def predict(self, c_task, w_fine, n_phys, h):
        lag = first_order_fine(ms_commands(c_task, self.delay_ms), self.tau, n_phys, h)
        w = w_fine.ravel()
        acc = np.gradient(w, h).reshape(w_fine.shape)
        return self.gain * lag - self.coulomb * np.tanh(w_fine / 0.02) - self.viscous * w_fine - self.armature * acc


def _friction_columns(r):
    w = r['w_fine'].ravel()
    acc = np.gradient(w, r['h'])
    return np.c_[-np.tanh(w / 0.02), -w, -acc][::5]


def _fit_linear(cols, y):
    """Gain free; Coulomb, viscous and armature constrained non-negative (physical)."""
    lb = [-np.inf] + [0.0] * (cols.shape[1] - 1)
    res = lsq_linear(cols, y, bounds=(lb, np.inf), method='bvls')
    return res.x, float(np.sum((cols @ res.x - y) ** 2))


def fit_compact(runs):
    """runs: list of (c_task, result). Bounded least squares over gain/friction/armature, grid over tau and delay."""
    best = None
    taus = np.geomspace(0.5e-3, 60e-3, 40)
    fric = np.vstack([_friction_columns(r) for _, r in runs])
    y = np.concatenate([r['shaft'].ravel()[::5] for _, r in runs])
    for dly in range(0, 13):
        for tau in taus:
            lag = np.concatenate([first_order_fine(ms_commands(c, dly), tau, r['n_phys'], r['h']).ravel()[::5]
                                  for c, r in runs])
            coef, sse = _fit_linear(np.c_[lag, fric], y)
            if best is None or sse < best[0]:
                best = (sse, Compact(float(coef[0]), float(tau), int(dly), float(coef[1]), float(coef[2]), float(coef[3])))
    return best[1], best[0]


# ---------------------------------------------------------------- second-order compact model (A2)
def second_order_fine(u_ms, wn, zeta, n_phys, h):
    """Unity-DC-gain second-order lag driven by a 1 ms zero-order-hold input, sampled after each physics step."""
    A = np.array([[0.0, 1.0], [-wn * wn, -2 * zeta * wn]])
    B = np.array([[0.0], [wn * wn]]); C = np.array([[1.0, 0.0]]); D = np.array([[0.0]])
    Ad, Bd, Cd, Dd, _ = cont2discrete((A, B, C, D), h, method='zoh')
    b, a = ss2tf(Ad, Bd, Cd, Dd)
    return lfilter(b[0], a, np.repeat(u_ms, n_phys)).reshape(len(u_ms), n_phys)


@dataclass
class Compact2:
    gain: float = 1.0
    wn: float = 300.0
    zeta: float = 0.5
    delay_ms: int = 4
    coulomb: float = 0.0
    viscous: float = 0.0
    armature: float = 0.0

    def predict(self, c_task, w_fine, n_phys, h):
        lag = second_order_fine(ms_commands(c_task, self.delay_ms), self.wn, self.zeta, n_phys, h)
        acc = np.gradient(w_fine.ravel(), h).reshape(w_fine.shape)
        return self.gain * lag - self.coulomb * np.tanh(w_fine / 0.02) - self.viscous * w_fine - self.armature * acc


def fit_compact2(runs):
    best = None
    fric = np.vstack([_friction_columns(r) for _, r in runs])
    y = np.concatenate([r['shaft'].ravel()[::5] for _, r in runs])
    for dly in range(0, 11):
        for wn in 2 * np.pi * np.geomspace(10, 200, 18):
            for zeta in [0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0]:
                lag = np.concatenate([second_order_fine(ms_commands(c, dly), wn, zeta, r['n_phys'], r['h']).ravel()[::5]
                                      for c, r in runs])
                coef, sse = _fit_linear(np.c_[lag, fric], y)
                if best is None or sse < best[0]:
                    best = (sse, Compact2(float(coef[0]), float(wn), zeta, int(dly), float(coef[1]), float(coef[2]),
                                          float(coef[3])))
    return best[1], best[0]


def error_metrics(err, h):
    """RMS and peak over every physics step; peak also reported at 1 ms samples for contrast."""
    e = err[np.isfinite(err)]
    if e.size == 0:
        return dict(rms=np.nan, peak=np.nan, peak_1ms=np.nan)
    return dict(rms=float(np.sqrt(np.mean(e ** 2))), peak=float(np.max(np.abs(e))),
                peak_1ms=float(np.nanmax(np.abs(err[:, -1]))))


# ---------------------------------------------------------------- training simulators (rigid single joint)
def train_xml(mass, h, kind, compact=None):
    arm = compact.armature if kind in ('compact', 'compact2') else 0.0
    damp = compact.viscous if kind in ('compact', 'compact2') else 0.0
    if kind == 'reference':
        act = f'<general joint="load" dyntype="filterexact" dynprm="{T_REF}" gainprm="1" ctrllimited="true" ctrlrange="-{TAU_CAP} {TAU_CAP}"/>'
    elif kind == 'compact':
        act = (f'<general joint="load" dyntype="filterexact" dynprm="{compact.tau}" gainprm="{compact.gain}" '
               f'ctrllimited="true" ctrlrange="-{TAU_CAP} {TAU_CAP}"/>')
    else:   # bare torque source (also drives compact2, whose filter runs in Python)
        act = f'<motor joint="load" gear="1" ctrllimited="true" ctrlrange="-{TAU_CAP} {TAU_CAP}"/>'
    return f'''<mujoco model="train_{kind}"><option timestep="{h}" integrator="implicitfast" gravity="0 0 -9.81"/>
<worldbody><body name="link"><inertial pos="0 0 -.25" mass="{mass}" diaginertia="{mass/48} {mass/48} {mass/1000}"/>
<joint name="load" type="hinge" axis="0 1 0" damping="{0.002 + damp}" armature="{arm}"/>
<geom type="capsule" fromto="0 0 0 0 0 -.5" size=".02" contype="0" conaffinity="0"/></body></worldbody>
<actuator>{act}</actuator></mujoco>'''


class TrainingSim:
    """Rigid pendulum driven by a reduced actuator model; time-based command delay buffer."""
    def __init__(self, kind, mass=1.0, h=1e-3, compact=None):
        self.kind, self.h, self.cp = kind, h, compact
        self.m = mj.MjModel.from_xml_string(train_xml(mass, h, kind, compact))
        self.d = mj.MjData(self.m)
        self.delay = (compact.delay_ms if kind in ('compact', 'compact2') else D_NOM_MS) * 1e-3
        if kind == 'bare':
            self.delay = 0.0
        if kind == 'compact2':
            wn, z = compact.wn, compact.zeta
            Ad, Bd, *_ = cont2discrete((np.array([[0.0, 1.0], [-wn * wn, -2 * z * wn]]), np.array([[0.0], [wn * wn]]),
                                        np.eye(2), np.zeros((2, 1))), h, method='zoh')
            self.Ad, self.Bd = Ad, Bd[:, 0]

    def rollout(self, c_task, seconds):
        """Returns load angle at each 1 ms tick."""
        m, d, h = self.m, self.d, self.h
        n_ms = round(seconds / DT_LOCAL)
        th = np.empty(n_ms)
        t, k_ms = 0.0, 0
        steps = round(seconds / h)
        coul = self.cp.coulomb if self.kind in ('compact', 'compact2') else 0.0
        z = np.zeros(2)
        for s in range(steps):
            tc = t - self.delay
            j = int(math.floor(tc / DT_TASK + 1e-9))
            u = c_task[j] if j >= 0 else 0.0
            if self.kind == 'compact2':
                d.ctrl[0] = max(-TAU_CAP, min(TAU_CAP, self.cp.gain * z[0]))
                z = self.Ad @ z + self.Bd * u
            else:
                d.ctrl[0] = u
            if coul:
                d.qfrc_applied[0] = -coul * math.tanh(float(d.qvel[0]) / 0.02)
            mj.mj_step(m, d)
            t = (s + 1) * h
            while k_ms < n_ms and (k_ms + 1) * DT_LOCAL <= t + 1e-12:
                th[k_ms] = d.qpos[0]; k_ms += 1
        th[k_ms:] = d.qpos[0]
        return th
