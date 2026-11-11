"""Switching-level J1 (shoulder, GIM8115) bench in PLECS, replaying an averaged openarm_system run, and the comparison.

For a scenario saved by batch_run.m (results/<name>.mat) this
  1. replays J1's motor speed, iq_ref and bank voltage over the whole run through sw_shoulder.plecs (100 kHz PWM, dead
     time, sampled FOC; plecs_shoulder_model.py), logging per-period integrals;
  2. reruns a short window around J1's peak regenerating power with fine raw logging, for ripple;
  3. compares with the averaged model: board power and energy, iq tracking, C_local and hop RMS currents.
The position loop is not closed here (iq_ref comes from the averaged run), so this tests the actuator board's power
stage and current loop, not the arm motion. Needs PLECS running with its XML-RPC interface on port 1080.
Usage: python run_sw_shoulder.py drop6 [td_ns ...]      writes results/sw_<name>[_td<ns>].json and .png
"""
import csv
import json
import os
import subprocess
import sys
import time
import xmlrpc.client as x

import numpy as np
import scipy.io as sio

HERE = os.path.dirname(os.path.abspath(__file__))
J = 0                                            # J1
FSW, T_SW = 100e3, 25e-9                         # board PWM; averaged model's ASSUMED edge time (openarm_params.m)


def load_avg(name):
    R = sio.loadmat(os.path.join(HERE, 'results', name + '.mat'), squeeze_me=True, struct_as_record=False)['R']
    P = R.P
    imag = np.abs(R.iq[:, J])                    # averaged id ~ 0
    # averaged board power at the boards' 100 kHz (older runs used 30 kHz in the switching-loss term)
    pdc = R.pdc[:, J] + 3 * R.vloc[:, J] * imag * (2 / np.pi) * T_SW * (FSW - P.fsw)
    return dict(t=R.t, wm=P.N[J] * R.qd[:, J], iq=R.iq[:, J], iq_ref=R.iq_ref[:, J], v_bus=R.v_bus, vloc=R.vloc[:, J],
                pdc=pdc, P=P)


def model_vars(A, P, sel, td):
    t = A['t'][sel] - A['t'][sel][0]
    return {'ref_t': list(map(float, t)), 'ref_wm': list(map(float, A['wm'][sel])),
            'ref_iq': list(map(float, A['iq_ref'][sel])), 'ref_vb': list(map(float, A['v_bus'][sel])),
            'pp': float(P.pp[J]), 'Rs': float(P.R[J]), 'Ls': float(P.L[J]), 'lam': float(P.__dict__['lambda'][J]),
            'kp_i': float(P.kp_i[J]), 'ki_i': float(P.ki_i[J]), 'v_frac': float(P.v_frac), 'ron': float(P.rds),
            'fsw': FSW, 'td': td, 'r_hop': float(P.r_hop), 'l_hop': float(P.l_hop), 'c_local': float(P.c_local),
            'esr_local': float(P.esr_local), 'v0': float(A['vloc'][sel][0]), 'wm0': float(A['wm'][sel][0])}


def simulate(s, mv, stop, raw_ts):
    for f in ('sw_avg.csv', 'sw_raw.csv'):
        if os.path.exists(os.path.join(HERE, f)):
            os.remove(os.path.join(HERE, f))
    mv = dict(mv, raw_ts=raw_ts)
    t0 = time.time()
    s.plecs.simulate('sw_shoulder', {'ModelVars': mv, 'SolverOpts': {'StopTime': stop}})
    wall = time.time() - t0
    A = np.array(list(csv.reader(open(os.path.join(HERE, 'sw_avg.csv'))))[1:], float)
    W = np.array(list(csv.reader(open(os.path.join(HERE, 'sw_raw.csv'))))[1:], float)
    return A, W, wall


def per_period(A):
    """sw_avg.csv columns: t | integrals of i_board, v*i_board, i_cap^2, ia^2, i_board^2 | id iq iq_ref (sampled)."""
    t, I = A[:, 0], A[:, 1:6]
    d = np.diff(I, axis=0) / np.diff(t)[:, None]
    return dict(t=t[1:], i_board=d[:, 0], p_board=d[:, 1], i_cap_ms=d[:, 2], ia_ms=d[:, 3], i_board_ms=d[:, 4],
                id=A[1:, 6], iq=A[1:, 7], iq_ref=A[1:, 8])


def rms_windows(x_ms, t, win=1e-3):
    """RMS over consecutive windows from per-period mean squares."""
    n = max(1, int(round(win / np.median(np.diff(t)))))
    m = len(x_ms) // n * n
    return np.sqrt(np.clip(x_ms[:m].reshape(-1, n).mean(axis=1), 0, None))


def pp_period(v, n=20):
    """Largest peak-to-peak within one PWM period (n raw samples at 0.5 us = 10 us)."""
    m = len(v) // n * n
    return float(np.ptp(v[:m].reshape(-1, n), axis=1).max())


def main(name, tds):
    s = x.ServerProxy('http://localhost:1080/RPC2')
    subprocess.run([sys.executable, os.path.join(HERE, 'plecs_shoulder_model.py'),
                    os.path.join(HERE, 'sw_shoulder.plecs')], check=True)
    try:
        s.plecs.close('sw_shoulder')
    except Exception:
        pass
    s.plecs.load(os.path.join(HERE, 'sw_shoulder.plecs'))
    A = load_avg(name)
    P = A['P']
    out = {}
    for td in tds:
        key = f'td{int(round(td * 1e9))}ns'
        # 1. whole run
        sel = np.ones_like(A['t'], bool)
        mv = model_vars(A, P, sel, td)
        Av, _, wall = simulate(s, mv, float(A['t'][-1] - A['t'][0]), 1e-4)
        S = per_period(Av)
        S['p_board'] = S['p_board'] + P.p_board      # the averaged boards also carry P.p_board of logic and drive
        tt = S['t'] + A['t'][0]
        p_avg = np.interp(tt, A['t'], A['pdc'])
        iq_avg = np.interp(tt, A['t'], A['iq'])
        dt = np.diff(Av[:, 0])
        res = dict(wall_full_s=wall,
                   p_board_mean_sw=float(np.mean(S['p_board'])), p_board_mean_avg=float(np.mean(p_avg)),
                   e_regen_sw_j=float(np.sum(np.minimum(S['p_board'], 0) * dt)),
                   e_regen_avg_j=float(np.sum(np.minimum(p_avg, 0) * dt)),
                   e_motor_sw_j=float(np.sum(np.maximum(S['p_board'], 0) * dt)),
                   e_motor_avg_j=float(np.sum(np.maximum(p_avg, 0) * dt)),
                   p_peak_regen_sw_w=float(np.min(np.convolve(S['p_board'], np.ones(100) / 100, 'same'))),
                   p_peak_regen_avg_w=float(np.min(p_avg)),
                   p_err_rms_w=float(np.sqrt(np.mean((np.convolve(S['p_board'], np.ones(100) / 100, 'same')
                                                      - p_avg) ** 2))),
                   iq_track_rms_a=float(np.sqrt(np.mean((S['iq'] - S['iq_ref']) ** 2))),
                   iq_vs_avg_rms_a=float(np.sqrt(np.mean((S['iq'] - iq_avg) ** 2))),
                   id_rms_a=float(np.sqrt(np.mean(S['id'] ** 2))),
                   i_cap_rms_1ms_max_a=float(rms_windows(S['i_cap_ms'], S['t']).max()),
                   i_board_rms_1ms_max_a=float(rms_windows(S['i_board_ms'], S['t']).max()),
                   ia_rms_1ms_max_a=float(rms_windows(S['ia_ms'], S['t']).max()))
        # 2. detail window: 20 ms ending 5 ms after J1's peak regenerating power (first 5 ms is current-loop settling)
        k = int(np.argmin(A['pdc']))
        t1 = A['t'][k] + 5e-3
        sel = (A['t'] >= t1 - 20e-3) & (A['t'] <= t1)
        mv = model_vars(A, P, sel, td)
        Aw, W, wall_w = simulate(s, mv, float(A['t'][sel][-1] - A['t'][sel][0]), 0.5e-6)
        Sw = per_period(Aw)
        Sw['p_board'] = Sw['p_board'] + P.p_board
        keep_p = Sw['t'] >= 5e-3
        keep_w = W[:, 0] >= 5e-3
        Wk = W[keep_w]
        ia, ib, ic, ibd, vb, icap, thm = (Wk[:, i] for i in range(1, 8))
        th = P.pp[J] * thm
        iq_raw = -2 / 3 * (ia * np.sin(th) + ib * np.sin(th - 2 * np.pi / 3) + ic * np.sin(th + 2 * np.pi / 3))
        tau_raw = 1.5 * P.pp[J] * P.__dict__['lambda'][J] * iq_raw * P.N[J]           # output side, before gear loss
        res.update(window_s=[float(A['t'][sel][0]), float(A['t'][sel][-1])], wall_window_s=wall_w,
                   win_p_board_sw=float(np.mean(Sw['p_board'][keep_p])),
                   win_p_board_avg=float(np.mean(np.interp(Sw['t'][keep_p] + A['t'][sel][0], A['t'], A['pdc']))),
                   win_i_cap_rms_a=float(np.sqrt(np.mean(icap ** 2))),
                   win_i_board_mean_a=float(np.mean(ibd)),
                   win_i_board_pp_per_period_a=pp_period(ibd), win_v_board_pp_per_period_v=pp_period(vb),
                   win_ia_pp_per_period_a=pp_period(ia), win_tau_out_pp_per_period_nm=pp_period(tau_raw),
                   win_tau_out_mean_nm=float(np.mean(tau_raw)))
        out[key] = res
        print(key, json.dumps(res, indent=1))
        if td == tds[0]:
            plot(name, A, S, tt, p_avg, Wk, td)
    json.dump(out, open(os.path.join(HERE, 'results', f'sw_{name}.json'), 'w'), indent=1)


def plot(name, A, S, tt, p_avg, Wk, td):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    f, ax = plt.subplots(3, 1, figsize=(11, 10))
    pm = np.convolve(S['p_board'], np.ones(100) / 100, 'same')
    ax[0].plot(A['t'], A['pdc'], 'k', lw=2, label='averaged model')
    ax[0].plot(tt, pm, 'r', lw=1, label='switching (1 ms mean)')
    ax[0].set_ylabel('J1 board power, W (negative = regen)')
    ax[0].legend()
    ax[0].grid()
    ax[1].plot(tt, S['iq'], 'r', lw=0.6, label='switching iq (sampled)')
    ax[1].plot(A['t'], A['iq'], 'k', lw=1.5, label='averaged iq')
    ax[1].set_ylabel('iq, A')
    ax[1].legend()
    ax[1].grid()
    tw = Wk[:, 0] * 1e3
    m = tw < tw[0] + 0.2
    ax[2].plot(tw[m], Wk[m, 1], label='ia')
    ax[2].plot(tw[m], Wk[m, 4], label='board input current')
    ax[2].plot(tw[m], Wk[m, 6], label='C_local current')
    ax[2].set_xlabel('ms into detail window')
    ax[2].set_ylabel('A')
    ax[2].legend()
    ax[2].grid()
    f.suptitle(f'{name}: J1 switching (100 kHz, {td * 1e9:.0f} ns dead time) vs averaged')
    f.tight_layout()
    f.savefig(os.path.join(HERE, 'results', f'sw_{name}.png'), dpi=110)


if __name__ == '__main__':
    main(sys.argv[1], [float(v) * 1e-9 for v in sys.argv[2:]] or [100e-9])
