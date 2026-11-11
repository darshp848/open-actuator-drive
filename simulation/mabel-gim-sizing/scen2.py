"""Scenarios and metrics for the actuator-set comparison (optimised per-joint actuators vs stock GIM).
Uses rig2.SetRig; trajectories B and C come from scen.py unchanged. New here: the arm-only reach test D, the torso
brake lock during the hold of B, the backdrive test, and the per-joint / per-group metrics.
"""
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import scen as S  # noqa: E402
from scen import Traj, worst_pose, traj_bend, traj_reach, feedforward, T_SETTLE  # noqa: E402
from rig2 import SetRig, get_set, JN, JOINT_GROUPS  # noqa: E402
import mujoco as mj  # noqa: E402

PRELOAD_S = 0.05            # motor current is restored this long before the brake releases (brake still holding)
SAT_LIMIT = 0.02            # pass rule: no joint saturates for more than 2 % of the active window
ERR_FACTOR, ERR_FLOOR = 1.5, 0.010   # pass rule: peak error <= max(1.5 x stock, 10 mrad)

# arm-only reach: Arm 1 swing to forward horizontal (Left Arm 1 = -0.40 rad puts the palm level with the shoulder, straight elbow)
A1_HORIZ = -0.40
A1_START = 0.24             # D: 0.64 rad swing, 0.6 s, peak 2.0 rad/s (62 % of the GIM8115 no-load speed, like the torso T = 1 s test)
A1_HANG = 1.20              # D_fast: hand hanging at the side, 1.6 rad swing, 0.6 s, peak 5.0 rad/s (above the GIM8115 no-load speed)


def arm_pose(a1):
    q = np.zeros(15)
    q[JN.index('Left Arm 1')] = a1
    q[JN.index('Right Arm 1')] = -a1       # right arm mirrored; all other joints 0 (elbows straight)
    return q


def traj_arm(start, T=0.6, hold=1.0, tail=0.5):
    t0 = T_SETTLE
    total = t0 + 2 * T + hold + tail
    jl, jr = JN.index('Left Arm 1'), JN.index('Right Arm 1')
    d = A1_HORIZ - start
    moves = [(jl, t0, T, 0., d), (jr, t0, T, 0., -d), (jl, t0 + T + hold, T, 0., -d), (jr, t0 + T + hold, T, 0., d)]
    w = {'out': (t0, t0 + T), 'hold': (t0 + T, t0 + T + hold), 'back': (t0 + T + hold, t0 + 2 * T + hold),
         'tail': (t0 + 2 * T + hold, total), 'active': (t0, total)}
    return Traj(f'D_{start:+.2f}', arm_pose(start), 0.3175, total, moves, windows=w)


SCENARIOS = {   # key: (trajectory factory, brake policy)   brake: None | 'hold' (B hold window) | 'always' (torso never moves)
    'B_T2': (lambda: traj_bend(2.0), 'hold'), 'B_T1': (lambda: traj_bend(1.0), 'hold'),
    'B_T2_nobrake': (lambda: traj_bend(2.0), None), 'B_T1_nobrake': (lambda: traj_bend(1.0), None),
    'C': (traj_reach, None),
    'D': (lambda: traj_arm(A1_START), 'always'), 'D_fast': (lambda: traj_arm(A1_HANG), 'always'),
}
RULE_SCENARIOS = ['B_T2', 'B_T1', 'C', 'D']      # the pass rule covers these (0 kg and 3 kg each); the others are reference


def run(aset_name, scen_key, payload):
    aset = get_set(aset_name)
    fac, policy = SCENARIOS[scen_key]
    tr = fac()
    rig = SetRig(aset, payload)
    rig.set_pose(tr.q0, lift=tr.lift0)
    ff = feedforward(rig, tr, 'grav')
    rig.preload(tr.q0, ff[0])
    K = len(tr.t)
    locked = np.zeros(K, bool)
    hold = tr.windows.get('hold')
    for k in range(K):
        t = tr.t[k]
        if policy == 'always':
            on, off = True, True
        elif policy == 'hold':
            on = hold[0] <= t < hold[1]
            off = hold[0] <= t < hold[1] - PRELOAD_S
        else:
            on = off = False
        rig.brake(on)
        rig.torso_motor_off = off
        locked[k] = on
        rig.step_ms(tr.q[k], tr.v[k], ff[k], tr.lift[k])
    return rig, rig.log(), tr, locked, ff


def _mask(L, win, locked_ms, j):
    t = L['t']
    mk = (t > win[0]) & (t <= win[1])
    if j == 0:
        lk = np.repeat(locked_ms, len(t) // len(locked_ms))
        mk = mk & ~lk
    return mk


def joint_metrics(rig, L, tr, locked_ms, j, win):
    P, dt = rig.P, rig.cfg.dt
    mk = _mask(L, win, locked_ms, j)
    if not mk.any():
        return dict(n=0)
    err = L['q'][mk, j] - L['q_ref'][mk, j]
    iq = L['iq'][mk, j]
    vf, ifl = L['vflag'][mk, j], L['iflag'][mk, j]
    return dict(n=int(mk.sum()), peak_err=float(np.abs(err).max()), rms_err=float(np.sqrt(np.mean(err ** 2))),
                peak_iq_frac=float(np.abs(iq).max() / P['i_max'][j]), frac_v=float(vf.mean()), frac_i=float(ifl.mean()),
                frac_sat=float((vf | ifl).mean()), peak_speed=float(np.abs(L['v'][mk, j]).max()),
                speed_over_noload=float(np.abs(L['v'][mk, j]).max() / rig.no_load_speed[j]),
                peak_torque=float(np.abs(P['kt_out'][j] * iq).max()), torque_limit=float(P['tau_max'][j]),
                cu_J=float(np.sum(1.5 * P['R'][j] * iq ** 2) * dt))


def bus_metrics(L, win, dt):
    t = L['t']
    mk = (t > win[0]) & (t <= win[1])
    p = L['p_dc'][mk].sum(1)
    return dict(peak_motoring_W=float(max(p.max(), 0)), peak_regen_W=float(min(p.min(), 0)),
                E_motoring_J=float(np.sum(np.maximum(p, 0)) * dt), E_regen_J=float(np.sum(np.minimum(p, 0)) * dt),
                E_net_J=float(np.sum(p) * dt))


def metrics(rig, L, tr, locked):
    dt = rig.cfg.dt
    locked_ms = locked
    W = tr.windows
    out = {'joints': {}, 'groups': {}, 'bus': bus_metrics(L, W['active'], dt), 'torso_locked_fraction': float(locked.mean())}
    for j in range(15):
        out['joints'][JN[j]] = joint_metrics(rig, L, tr, locked_ms, j, W['active'])
    for g, idx in JOINT_GROUPS.items():
        ms = [out['joints'][JN[j]] for j in idx if out['joints'][JN[j]].get('n')]
        if not ms:
            out['groups'][g] = dict(n=0)
            continue
        out['groups'][g] = dict(peak_err=max(m['peak_err'] for m in ms), rms_err=float(np.sqrt(np.mean([m['rms_err'] ** 2 for m in ms]))),
                                peak_iq_frac=max(m['peak_iq_frac'] for m in ms), frac_sat=max(m['frac_sat'] for m in ms),
                                frac_v=max(m['frac_v'] for m in ms), frac_i=max(m['frac_i'] for m in ms),
                                peak_speed=max(m['peak_speed'] for m in ms),
                                speed_over_noload=max(m['speed_over_noload'] for m in ms),
                                peak_torque=max(m['peak_torque'] for m in ms), cu_J=sum(m['cu_J'] for m in ms))
    if 'hold' in W:   # hold segment reported separately (torso: current, copper loss, bus; brake runs read zero)
        h = W['hold']
        mk = (L['t'] > h[0]) & (L['t'] <= h[1])
        j = 0
        iq = L['iq'][mk, j]
        out['hold_torso'] = dict(mean_abs_iq_frac=float(np.abs(iq).mean() / rig.P['i_max'][0]), mean_iq_A=float(iq.mean()),
                                 cu_J=float(np.sum(1.5 * rig.P['R'][0] * iq ** 2) * rig.cfg.dt),
                                 bus_E_J=float(np.sum(L['p_dc'][mk, 0]) * rig.cfg.dt),
                                 peak_err=float(np.abs(L['q'][mk, j] - L['q_ref'][mk, j]).max()),
                                 locked=bool(locked[(tr.t >= h[0]) & (tr.t < h[1])].any()))
    if tr.name.startswith('C'):
        out['lift_final_m'] = float(L['lift'][-1])
    return out


def series_for_plot(rig, L, tr, locked, win=None):
    t = L['t']
    mk = (t >= T_SETTLE - 0.2)
    sl = slice(None, None, 2)
    return dict(t=(t[mk] - T_SETTLE)[sl], q=L['q'][mk, 0][sl], q_ref=L['q_ref'][mk, 0][sl], iq_frac=(L['iq'][mk, 0] / rig.P['i_max'][0])[sl],
                p_dc_total=L['p_dc'][mk].sum(1)[sl], p_dc_torso=L['p_dc'][mk, 0][sl], v=L['v'][mk, 0][sl])


# ---------------------------------------------------------------------------- backdrive
def backdrive(aset_name, force=10., secs=1.0, pre=0.2):
    """Zero current at every joint, torso locked, gravity off (so the response shows friction and inertia, not sag),
    arms in a mid-range pose; push `force` N horizontally forward (-x, the direction the head moves in the bend) at each
    palm for `secs`. Returns joint displacement with and without the push."""
    from rig2 import Config
    aset = get_set(aset_name)
    res = {}
    q0 = np.zeros(15)
    for a, b in (('Left Arm 1', 1.0), ('Left Arm 2', 0.3), ('Left Arm 4', 0.3)):
        q0[JN.index(a)] = b
        q0[JN.index(a.replace('Left', 'Right'))] = -b
    for push in (True, False):
        rig = SetRig(aset, 0., Config(kp=np.zeros(15), kd=np.zeros(15)))
        rig.m.opt.gravity[:] = 0
        rig.m_id.opt.gravity[:] = 0
        rig.set_pose(q0)
        rig.preload(q0, np.zeros(15))
        rig.brake(True)
        rig.torso_motor_off = True
        palms = [rig.m.body(n).id for n in ('left_palm', 'right_palm')]
        p0 = np.array([rig.d.xpos[b].copy() for b in palms])
        z = np.zeros(15)
        rows = []
        for k in range(round((pre + secs) / 1e-3)):
            t = k * 1e-3
            rig.d.xfrc_applied[:] = 0
            if push and t >= pre:
                for b in palms:
                    rig.d.xfrc_applied[b, 0] = -force
            rig.step_ms(q0, z, z, 0.3175)
            rows.append((t, rig.q.copy()))
        q1 = rig.q
        res['push' if push else 'nopush'] = dict(dq=(q1 - q0).tolist(), palm_move_m=(np.array([rig.d.xpos[b] for b in palms]) - p0).tolist(),
                                                 q_at_025=None)
        if push:
            qs = np.array([r[1] for r in rows])
            ts = np.array([r[0] for r in rows])
            i = np.searchsorted(ts, pre + 0.25)
            res['push']['dq_at_025s'] = (qs[i] - q0).tolist()
            res['push']['hit_limit'] = [bool(x) for x in ((q1 <= rig.lo + 1e-3) | (q1 >= rig.hi - 1e-3))]
    # static check at q0: joint torques from the push vs Coulomb friction (breakaway ratio)
    rig = SetRig(aset, 0.)
    rig.m.opt.gravity[:] = 0
    rig.set_pose(q0)
    tau = np.zeros(rig.m.nv)
    for b in palms:
        jacp = np.zeros((3, rig.m.nv))
        mj.mj_jacBody(rig.m, rig.d, jacp, None, b)
        tau += jacp.T @ np.array([-force, 0, 0])
    tj = tau[rig.dof]
    res['push_torque_Nm'] = tj.tolist()
    res['coulomb_Nm'] = rig.P['tau_c'].tolist()
    res['breakaway_ratio'] = (np.abs(tj) / rig.P['tau_c']).tolist()
    res['reflected_inertia_kgm2'] = (rig.P['J_rotor'] * rig.P['N'] ** 2).tolist()
    return res


# ---------------------------------------------------------------------------- gains sanity step
def gains_check(aset_name, payload=3.):
    """Small position steps at the worst pose (as check_gains.py): overshoot and tail ripple per joint."""
    aset = get_set(aset_name)
    out = {}
    q0 = worst_pose(-0.6)
    for jn, amp in (('Torso', 0.05), ('Left Arm 1', 0.02), ('Left Arm 4', 0.02), ('Left Arm 7', 0.02)):
        j = JN.index(jn)
        rig = SetRig(aset, payload)
        rig.set_pose(q0)
        g = rig.inverse_torque(q0, 0 * q0, 0 * q0, 0.3175)
        rig.preload(q0, g)
        T, ts = 1.2, 0.3
        for k in range(round(T / 1e-3)):
            qr = q0.copy()
            if k * 1e-3 >= ts:
                qr[j] += amp
            gg = rig.inverse_torque(qr, 0 * qr, 0 * qr, 0.3175)
            rig.step_ms(qr, 0 * qr, gg, 0.3175)
        L = rig.log()
        t = L['t']
        y = (L['q'][:, j] - q0[j]) / amp
        after = t >= ts
        tail = t > T - 0.3
        out[jn] = dict(overshoot=float(max(0., y[after].max() - 1.)), tail_ripple_rad=float(np.std(L['q'][tail, j])),
                       final_frac=float(y[tail].mean()), peak_iq_frac=float(np.abs(L['iq'][:, j]).max() / rig.P['i_max'][j]))
    return out


def worker(task):
    kind = task[0]
    if kind == 'scen':
        _, sname, key, payload = task
        rig, L, tr, locked, ff = run(sname, key, payload)
        m = metrics(rig, L, tr, locked)
        m['mass'] = dict(total_kg=rig.total_mass, dm=rig.P['dm'].tolist(), method=rig.mass_method)
        ser = series_for_plot(rig, L, tr, locked) if key in ('B_T1', 'B_T2', 'B_T1_nobrake') else None
        return task, m, ser
    if kind == 'backdrive':
        return task, backdrive(task[1]), None
    if kind == 'gains':
        return task, gains_check(task[1]), None
    raise KeyError(kind)
