"""Trajectories, runs, metrics and plots for the MABEL GIM scenarios (A hold, B torso bend, C combined reach)."""
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from rig import *  # noqa: E402,F401,F403
import gim_params as G  # noqa: E402
import miniplot  # noqa: E402

KT = G.TILE(G.KT_OUT)[IDX]
T_SETTLE = 1.0
TORSO_MAX = 1.0472


def worst_pose(torso=0.0, arm1_scale=1.0):
    q = np.zeros(15)
    for n, v in WORST.items():
        q[JN.index(n)] = v
    q[0] = torso
    q[JN.index('Left Arm 1')] *= arm1_scale
    q[JN.index('Right Arm 1')] *= arm1_scale
    return q


class Traj:
    """1 kHz reference for the 15 joints and the lift."""

    def __init__(self, name, q0, lift0, total, moves, lift_move=None, windows=None):
        self.name, self.q0, self.lift0 = name, np.asarray(q0, float), lift0
        K = round(total / 1e-3)
        self.t = np.arange(K) * 1e-3
        self.q = np.tile(self.q0, (K, 1))
        self.v = np.zeros((K, 15))
        self.a = np.zeros((K, 15))
        for (j, t0, T, a, b) in moves:
            p, v, ac = min_jerk(self.t, t0, T, a, b)
            self.q[:, j] += p - a
            self.v[:, j] += v
            self.a[:, j] += ac
        self.lift = np.full(K, float(lift0))
        self.lift_v = np.zeros(K)
        self.lift_a = np.zeros(K)
        if lift_move:
            t0, T, a, b = lift_move
            p, v, ac = min_jerk(self.t, t0, T, a, b)
            self.lift, self.lift_v, self.lift_a = p, v, ac
        self.windows = windows or {}


def traj_hold(torso, secs=2.0):
    q = worst_pose(torso)
    return Traj(f'A_torso{torso:+.4f}', q, 0.3175, T_SETTLE + secs, [], windows={'hold': (T_SETTLE, T_SETTLE + secs)})


def traj_bend(T):
    t0 = T_SETTLE
    hold, tail = 1.0, 0.5
    total = t0 + 2 * T + hold + tail
    q0 = worst_pose(0.0)
    moves = [(0, t0, T, 0., -TORSO_MAX), (0, t0 + T + hold, T, 0., TORSO_MAX)]
    w = {'fwd': (t0, t0 + T), 'hold': (t0 + T, t0 + T + hold), 'ret': (t0 + T + hold, t0 + 2 * T + hold),
         'tail': (t0 + 2 * T + hold, total), 'active': (t0, total)}
    return Traj(f'B_T{T:g}', q0, 0.3175, total, moves, windows=w)


def traj_reach():
    t0 = T_SETTLE
    total = t0 + 5.0 + 0.5
    q0 = worst_pose(0.0, arm1_scale=0.0)       # Arm 1 starts at 0, elbows already at the worst-pose value
    jl, jr = JN.index('Left Arm 1'), JN.index('Right Arm 1')
    moves = [(0, t0, 2.0, 0., -0.8), (jl, t0, 2.0, 0., WORST['Left Arm 1']), (jr, t0, 2.0, 0., WORST['Right Arm 1'])]
    w = {'bend_swing': (t0, t0 + 2.0), 'rest': (t0 + 2.0, total), 'active': (t0, total)}
    return Traj('C_reach', q0, 0.0, total, moves, lift_move=(t0, 5.0, 0.0, 0.635), windows=w)


# ------------------------------------------------------------------ feedforward
def feedforward(rig, tr, kind):
    """kind 'grav': gravity + payload only (zero vel/acc); 'full': full rigid inverse dynamics on the reference."""
    K = len(tr.t)
    out = np.zeros((K, 15))
    z = np.zeros(15)
    for k in range(K):
        if kind == 'grav':
            out[k] = rig.inverse_torque(tr.q[k], z, z, tr.lift[k])
        else:
            out[k] = rig.inverse_torque(tr.q[k], tr.v[k], tr.a[k], tr.lift[k], tr.lift_v[k], tr.lift_a[k])
    return out


VARIANTS = {   # name: (actuation, ff kind, use v_ref)
    'gim_grav': ('gim', 'grav', True),       # main: MIT PD with trajectory velocity, gravity feedforward
    'gim_full': ('gim', 'full', True),       # full inverse-dynamics feedforward
    'gim_grav_v0': ('gim', 'grav', False),   # gravity ff, v_ref = 0 (damps against motion like the stock servo)
    'stock_plain': ('stock', None, False),   # shipped servo, ctrl = q_ref
    'stock_ff': ('stock', 'grav', False),    # shipped servo, ctrl = q_ref + gravity/kp
}


def run(tr, payload, variant, ff_cache=None):
    act, ffk, use_v = VARIANTS[variant]
    rig = Rig(payload, act)
    rig.set_pose(tr.q0, lift=tr.lift0)
    if ffk is None:
        ff = np.zeros((len(tr.t), 15))
    else:
        key = (payload, ffk)
        if ff_cache is not None and key in ff_cache:
            ff = ff_cache[key]
        else:
            ff = feedforward(rig, tr, ffk)
            if ff_cache is not None:
                ff_cache[key] = ff
    if act == 'gim':
        rig.preload(tr.q0, ff[0])
    z = np.zeros(15)
    for k in range(len(tr.t)):
        rig.step_ms(tr.q[k], tr.v[k] if use_v else z, ff[k], tr.lift[k])
    return rig, rig.log(), ff


# ------------------------------------------------------------------ metrics
def power_split(p, dt):
    return dict(E_motoring_J=float(np.sum(np.maximum(p, 0)) * dt), E_regen_J=float(np.sum(np.minimum(p, 0)) * dt),
                peak_motoring_W=float(max(p.max(), 0)), peak_regen_W=float(min(p.min(), 0)),
                net_J=float(np.sum(p) * dt))


def joint_metrics(rig, L, j, win):
    t = L['t']
    mk = (t > win[0]) & (t <= win[1])
    dt = rig.cfg.dt
    err = (L['q'][mk, j] - L['q_ref'][mk, j])
    out = dict(peak_abs_err_rad=float(np.abs(err).max()), rms_err_rad=float(np.sqrt(np.mean(err ** 2))),
               mean_err_rad=float(err.mean()), peak_speed_rad_s=float(np.abs(L['v'][mk, j]).max()))
    if rig.actuation == 'gim':
        imax = rig.P['i_max'][j]
        iq = L['iq'][mk, j]
        out.update(peak_iq_over_imax=float(np.abs(iq).max() / imax),
                   peak_iqcmd_over_imax=float(np.abs(L['iq_cmd'][mk, j]).max() / imax),
                   frac_voltage_limit=float(L['vflag'][mk, j].mean()), frac_current_limit=float(L['iflag'][mk, j].mean()),
                   frac_any_limit=float((L['vflag'][mk, j] | L['iflag'][mk, j]).mean()),
                   peak_output_torque_Nm=float(np.abs(KT[j] * iq).max()), listed_torque_Nm=float(rig.P['tau_max'][j]),
                   bus=power_split(L['p_dc'][mk, j], dt))
    else:
        f = L['force'][mk, j]
        fr = rig.frange_stock[j, 1]
        out.update(peak_output_torque_Nm=float(np.abs(f).max()), listed_torque_Nm=float(fr),
                   frac_at_force_limit=float((np.abs(f) >= 0.999 * fr).mean()),
                   mech=power_split(f * L['v'][mk, j], dt))
    return out


def arm_summary(rig, L, win):
    res = {}
    for j in range(1, 15):
        m = joint_metrics(rig, L, j, win)
        keep = ['peak_abs_err_rad', 'peak_output_torque_Nm', 'listed_torque_Nm', 'peak_speed_rad_s']
        keep += ['peak_iq_over_imax', 'frac_any_limit', 'frac_voltage_limit', 'frac_current_limit'] if rig.actuation == 'gim' \
            else ['frac_at_force_limit']
        res[JN[j]] = {k: m[k] for k in keep}
    return res


def feasibility(rig_gim, tr, ff_full):
    """Rigid-body demand on the torso (full inverse dynamics + Coulomb friction) against the voltage-limited
    torque envelope of the GIM8115-36, evaluated along the reference. ratio > 1 means the envelope cannot supply it."""
    j = 0
    ratio = np.zeros(len(tr.t))
    tau_req = np.zeros(len(tr.t))
    for k in range(len(tr.t)):
        v = tr.v[k, j]
        req = ff_full[k, j] + rig_gim.P['tau_c'][j] * np.sign(v)
        lo, hi, _ = rig_gim.volt_limits(np.full(15, v))
        ip = max(min(hi[j], rig_gim.P['i_max'][j]), 0.)
        im = min(max(lo[j], -rig_gim.P['i_max'][j]), 0.)
        cap = KT[j] * (ip if req >= 0 else -im)
        ratio[k] = abs(req) / cap if cap > 1e-9 else np.inf
        tau_req[k] = req
    return ratio, tau_req


# ------------------------------------------------------------------ plots
def ds(x, n=1200):
    x = np.asarray(x)
    if len(x) <= n:
        return x
    return x[np.linspace(0, len(x) - 1, n).astype(int)]


def plot_case(path, title, tr, runs, rig_gim, win, torso_only=False):
    """runs: dict variant -> (rig, L). Panels: torso q vs ref, tracking error, iq vs limits, bus power, torque-speed."""
    t0, t1 = win
    g = runs.get('gim_grav')
    s_plain, s_ff = runs.get('stock_plain'), runs.get('stock_ff')
    panels = []
    sel = lambda L: (L['t'] >= t0) & (L['t'] <= t1)
    series = []
    if g:
        L = g[1]; m = sel(L)
        series.append(('ref', ds(L['t'][m] - t0), ds(L['q_ref'][m, 0])))
        series.append(('GIM grav ff', ds(L['t'][m] - t0), ds(L['q'][m, 0])))
    for lab, r in (('stock plain', s_plain), ('stock ff', s_ff)):
        if r:
            L = r[1]; m = sel(L)
            series.append((lab, ds(L['t'][m] - t0), ds(L['q'][m, 0])))
    panels.append(dict(title=title + ': torso q vs ref', xlabel='t (s)', ylabel='rad', series=series))
    ser = []
    for lab, v in (('GIM grav', 'gim_grav'), ('GIM full-ID', 'gim_full'), ('GIM v_ref=0', 'gim_grav_v0'), ('stock plain', 'stock_plain'),
                   ('stock ff', 'stock_ff')):
        if runs.get(v):
            L = runs[v][1]; m = sel(L)
            ser.append((lab, ds(L['t'][m] - t0), ds(1e3 * (L['q'][m, 0] - L['q_ref'][m, 0]))))
    panels.append(dict(title='torso tracking error', xlabel='t (s)', ylabel='mrad', series=ser))
    if g:
        L = g[1]; m = sel(L)
        imax = rig_gim.P['i_max'][0]
        panels.append(dict(title='torso iq vs limits (GIM grav ff)', xlabel='t (s)', ylabel='A',
                           series=[('iq', ds(L['t'][m] - t0), ds(L['iq'][m, 0])),
                                   ('iq cmd', ds(L['t'][m] - t0), ds(np.clip(L['iq_cmd'][m, 0], -2 * imax, 2 * imax))),
                                   ('voltage hi', ds(L['t'][m] - t0), ds(np.minimum(L['vhi'][m, 0], 2 * imax))),
                                   ('voltage lo', ds(L['t'][m] - t0), ds(np.maximum(L['vlo'][m, 0], -2 * imax)))],
                           hline=[imax, -imax]))
        panels.append(dict(title='torso DC bus power (GIM grav ff)', xlabel='t (s)', ylabel='W',
                           series=[('GIM8115 torso', ds(L['t'][m] - t0), ds(L['p_dc'][m, 0]))], hline=[0.]))
        w = np.linspace(-4.5, 4.5, 91)
        hi_t, lo_t = [], []
        for x in w:
            lo, hi, _ = rig_gim.volt_limits(np.full(15, x))
            hi_t.append(KT[0] * max(min(hi[0], rig_gim.P['i_max'][0]), 0))
            lo_t.append(KT[0] * min(max(lo[0], -rig_gim.P['i_max'][0]), 0))
        panels.append(dict(title='torso torque-speed (GIM grav ff)', xlabel='speed (rad/s)', ylabel='Nm',
                           series=[('envelope +', w, hi_t), ('envelope -', w, lo_t),
                                   ('trace Kt*iq', ds(L['v'][m, 0]), ds(KT[0] * L['iq'][m, 0]))]))
        # arm J1 / J2 iq fraction
        ser = []
        for jn in ('Left Arm 1', 'Left Arm 2', 'Right Arm 1', 'Right Arm 2'):
            j = JN.index(jn)
            ser.append((jn, ds(L['t'][m] - t0), ds(L['iq'][m, j] / rig_gim.P['i_max'][j])))
        panels.append(dict(title='arm J1/J2 iq / i_max (GIM grav ff)', xlabel='t (s)', ylabel='iq/imax', series=ser, hline=[1, -1]))
    miniplot.plot(str(path), panels, ncols=2, cell=(700, 420))
