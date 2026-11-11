"""Shared scenarios: Codex's 1.6 s bimanual motion profile and the interaction-impedance hold, run on the stock
vendor servo model and on the GIM layer."""
import sys
from pathlib import Path

import mujoco as mj
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import robot  # noqa: E402
from actuated import GimArm, Config  # noqa: E402

ARM_ACT = [f'{s}_joint{i}_ctrl' for s in ('left', 'right') for i in range(1, 8)]
FING_ACT = ['left_finger1_ctrl', 'right_finger1_ctrl']
J = {n: i for i, n in enumerate(robot.ARMS)}


def blend(a, b, t, start, duration):                     # same blend as run_diagnostics.py
    s = np.clip((t - start) / duration, 0., 1.)
    return a + (b - a) * (10 * s**3 - 15 * s**4 + 6 * s**5)


def motion_targets(q0):
    tgt = np.array(q0, float)
    tgt[J['openarm_left_joint1']] -= 0.10
    tgt[J['openarm_right_joint1']] += 0.10
    tgt[J['openarm_left_joint4']] -= 0.15
    tgt[J['openarm_right_joint4']] -= 0.15
    return tgt


def desired(q0, tgt, t):
    """14 arm references and the two finger1 ctrl values (left, right) at time t."""
    fing = []
    for opening in (.35, -.35):
        f = blend(opening, 0., t, .35, .35)
        fing.append(blend(f, opening, t, .95, .35))
    return blend(q0, tgt, t, 0.3, 0.6), np.array(fing)


def run_stock(dt=0.5e-3, duration=1.6):
    m, d = robot.load(step=dt)
    act = [m.actuator(n).id for n in ARM_ACT]
    fact = [m.actuator(n).id for n in FING_ACT]
    qadr = [m.joint(n).qposadr[0] for n in robot.ARMS]
    q0 = d.qpos[qadr].copy()
    tgt = motion_targets(q0)
    n_sub = round(1e-3 / dt)
    T, Q, QD, TAU = [], [], [], []
    for k in range(round(duration / 1e-3)):
        qd, fing = desired(q0, tgt, k * 1e-3)
        d.ctrl[act], d.ctrl[fact] = qd, fing
        for _ in range(n_sub):
            mj.mj_step(m, d)
            T.append(d.time); Q.append(d.qpos[qadr].copy()); QD.append(qd); TAU.append(d.actuator_force[act].copy())
    return dict(t=np.array(T), q=np.array(Q), qdes=np.array(QD), tau=np.array(TAU))


def run_gim(cfg, duration=1.6, cmd_fn=None):
    arm = GimArm(cfg)
    q0 = arm.q
    tgt = motion_targets(q0)
    QD = []
    for k in range(round(duration / 1e-3)):
        qd, fing = desired(q0, tgt, k * 1e-3)
        c = {'q_ref': qd, 'v_ref': np.zeros(14)} if cfg.mode == 'pd' else cmd_fn(arm, qd)
        arm.step_ms(c, fing)
        QD += [qd] * round(1e-3 / cfg.dt)
    L = arm.log()
    return dict(t=L['t'], q=L['q'], qdes=np.array(QD), tau=L['tau_out'], log=L, arm=arm)


def rms_diff(a, b):
    """RMS joint difference between two runs on the common 1 ms grid."""
    ia = np.round(a['t'] / 1e-3).astype(int)
    ib = np.round(b['t'] / 1e-3).astype(int)
    ka = {i: n for n, i in enumerate(ia)}
    kb = {i: n for n, i in enumerate(ib)}
    common = sorted(set(ka) & set(kb))
    qa = a['q'][[ka[i] for i in common]]
    qb = b['q'][[kb[i] for i in common]]
    return float(np.sqrt(np.mean((qa - qb) ** 2)))


# ---------------------------------------------------------------- interaction impedance
def impedance_metrics(t, q, t_on, t_off, j, sign):
    """Deflection of joint j (along the pushed direction) relative to the settled pre-step pose."""
    pre = np.mean(q[(t > t_on - 0.05) & (t <= t_on), j])
    defl = sign * (q[:, j] - pre)
    on = (t > t_on) & (t <= t_off)
    steady = float(np.mean(defl[on & (t > t_off - 0.05)]))
    peak = float(np.max(defl[on]))
    band = 0.05 * abs(steady)
    out = np.abs(defl[on] - steady) > band
    settle = float(t[on][np.flatnonzero(out)[-1] + 1] - t_on) if out.any() else 0.
    if out.any() and np.flatnonzero(out)[-1] + 1 >= on.sum():
        settle = float('nan')
    resid = float(np.mean(defl[t > t[-1] - 0.1]))
    return dict(peak_deflection_rad=peak, steady_deflection_rad=steady, settle_time_s=settle,
                residual_offset_rad=resid, pre_step_rad=float(pre), stiffness_Nm_per_rad=None)


def run_impedance(kind, joint, torque, t_on=0.5, t_off=0.8, t_end=1.5, dt=0.5e-3):
    """kind: 'stock' | 'rigid' | 'compliant'. Hold home with gravity feedforward, push `joint` with `torque`."""
    j = J[joint]
    if kind == 'stock':
        m, d = robot.load(step=dt)
        act = [m.actuator(n).id for n in ARM_ACT]
        qadr = np.array([m.joint(n).qposadr[0] for n in robot.ARMS])
        dof = np.array([m.joint(n).dofadr[0] for n in robot.ARMS])
        kp = m.actuator_gainprm[act, 0]
        g = d.qfrc_bias[dof].copy()
        d.ctrl[act] = d.qpos[qadr] + g / kp
        T, Q, tau = [], [], []
        for k in range(round(t_end / 1e-3)):
            t = k * 1e-3
            d.qfrc_applied[:] = 0
            if t_on <= t < t_off:
                d.qfrc_applied[dof[j]] = torque
            for _ in range(round(1e-3 / dt)):
                mj.mj_step(m, d)
                T.append(d.time); Q.append(d.qpos[qadr].copy()); tau.append(d.actuator_force[act].copy())
    else:
        arm = GimArm(Config(transmission=kind, dt=dt))
        g = arm.gravity_torque()
        hold = {'q_ref': arm.theta_r, 'v_ref': np.zeros(14), 'tau_ff': g}
        for k in range(round(t_end / 1e-3)):
            t = k * 1e-3
            arm.ext[:] = 0
            if t_on <= t < t_off:
                arm.ext[arm.link_dof[j]] = torque
            arm.step_ms(hold)
        L = arm.log()
        T, Q, tau = L['t'], L['q'], L['tau_out']
    T, Q = np.array(T), np.array(Q)
    res = impedance_metrics(T, Q, t_on, t_off, j, np.sign(torque))
    res['stiffness_Nm_per_rad'] = float(abs(torque) / res['steady_deflection_rad']) if res['steady_deflection_rad'] else None
    return res, dict(t=T, q=Q[:, j])
