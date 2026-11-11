"""Run scenarios A (gravity hold), B (torso bend), C (combined reach) for 0 and 3 kg per hand, GIM layer vs stock servos.
Run: python -I -B mabel_gim/run_all.py [A] [B] [C]   -> results/<timestamp>_scenarios/{metrics.json, summary.md, *.png, *.npz}"""
import json, sys, time
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import numpy as np
from scen import *

GIM_V = ['gim_grav', 'gim_full', 'gim_grav_v0']
STOCK_V = ['stock_plain', 'stock_ff']


def hold_metrics(rig, L, win, j=0):
    t = L['t']
    mk = (t > win[0]) & (t <= win[1])
    q = L['q'][mk, j]
    slope = np.polyfit(t[mk], q, 1)[0]
    res = dict(steady_err_rad=float(np.mean(L['q'][mk, j] - L['q_ref'][mk, j])),
               creep_mrad_s=float(slope * 1e3), q_std_urad=float(np.std(q) * 1e6),
               frac_stuck=float(np.mean(np.abs(L['v'][mk, j]) < 1e-3)),
               frac_at_joint_limit=float(np.mean(q <= rig.lo[j] + 1e-4)))
    if rig.actuation == 'gim':
        res.update(mean_iq_A=float(L['iq'][mk, j].mean()), mean_iq_over_imax=float(L['iq'][mk, j].mean() / rig.P['i_max'][j]),
                   peak_iq_over_imax=float(np.abs(L['iq'][mk, j]).max() / rig.P['i_max'][j]),
                   mean_motor_torque_Nm=float(L['tau_m'][mk, j].mean()),
                   mean_output_torque_Nm=float((KT[j] * L['iq'][mk, j]).mean()),
                   friction_band_Nm=float(rig.P['tau_c'][j] + (1 - rig.P['eta'][j]) * abs(L['tau_m'][mk, j].mean())))
    else:
        fr = rig.frange_stock[j, 1]
        res.update(mean_torque_Nm=float(L['force'][mk, j].mean()), peak_torque_Nm=float(np.abs(L['force'][mk, j]).max()),
                   frac_at_force_limit=float((np.abs(L['force'][mk, j]) >= 0.999 * fr).mean()))
    return res


def run_case(tag, tr, payload, variants, outdir, plot_win, scenario):
    cache = {}
    runs, met = {}, {}
    for v in variants:
        t = time.time()
        rig, L, ff = run(tr, payload, v, cache)
        runs[v] = (rig, L)
        m = {}
        if scenario == 'A':
            w = tr.windows['hold']
            m['torso_hold'] = hold_metrics(rig, L, w)
            m['arms_hold'] = {JN[j]: hold_metrics(rig, L, w, j) for j in range(1, 15)}
            # gravity torque at the settled actual pose (mj_inverse, all DOFs as simulated)
            qa = np.array([L['q'][(L['t'] > w[0]) & (L['t'] <= w[1]), j].mean() for j in range(15)])
            m['gravity_torque_at_actual_pose_Nm'] = dict(zip(JN, rig.inverse_torque(qa, 0 * qa, 0 * qa, tr.lift0).round(3).tolist()))
        else:
            m['torso'] = {n: joint_metrics(rig, L, 0, w) for n, w in tr.windows.items()}
            m['arms_active'] = arm_summary(rig, L, tr.windows['active'])
            if scenario == 'C':
                m['lift_final_m'] = float(L['lift'][-1])
        met[v] = m
        print(f'  {tag} {v:12s} {time.time()-t:5.1f} s', flush=True)
    if scenario in ('B', 'C'):
        rg = runs['gim_grav'][0]
        ratio, req = feasibility(rg, tr, cache[(payload, 'full')])
        mv = np.abs(tr.v[:, 0]) > 0
        met['feasibility_torso'] = dict(peak_demand_over_capability=float(ratio.max()),
                                        peak_required_torque_Nm=float(np.abs(req).max()),
                                        peak_ref_speed_rad_s=float(np.abs(tr.v[:, 0]).max()),
                                        frac_of_moving_time_demand_gt_1=float((ratio[mv] > 1).mean()) if mv.any() else 0.)
    return runs, met


def save_npz(path, runs):
    d = {}
    for v, (rig, L) in runs.items():
        for k in ('t', 'q', 'q_ref', 'v', 'iq', 'iq_cmd', 'p_dc', 'vlo', 'vhi', 'tau_m', 'force', 'lift'):
            if k in L:
                a = L[k][::2]
                d[f'{v}/{k}'] = a[:, :5] if a.ndim == 2 and k != 'lift' else a
    np.savez_compressed(path, **d)


def fmt(x, n=3):
    return f'{x:.{n}g}'


def main():
    which = [a for a in sys.argv[1:] if a in 'ABC'] or ['A', 'B', 'C']
    out = HERE / 'results' / (time.strftime('%Y%m%d_%H%M%S') + '_scenarios')
    out.mkdir(parents=True)
    allm = {}
    P0 = Rig(0., 'gim')
    allm['setup'] = dict(torso_kp=TORSO_KP, torso_kd=TORSO_KD, arm_kp=dict(zip(JN[1:], P0.kp[1:].round(1).tolist())),
                         arm_kd=dict(zip(JN[1:], P0.kd[1:].round(2).tolist())), dt=P0.cfg.dt, vbus=P0.cfg.vbus,
                         i_max=dict(zip(JN, P0.P['i_max'].round(2).tolist())), tau_listed=dict(zip(JN, P0.P['tau_max'].round(1).tolist())),
                         worst_pose=WORST)
    for payload in (0., 3.):
        pk = f'{payload:g}kg'
        if 'A' in which:
            for torso in (-1.0472, -1.0):
                tr = traj_hold(torso)
                tag = f'A_{pk}_torso{torso:+.4f}'
                print(tag, flush=True)
                runs, met = run_case(tag, tr, payload, ['gim_grav', *STOCK_V], out, tr.windows['hold'], 'A')
                allm[tag] = met
                panels = [dict(title=tag + ' torso err', xlabel='t (s)', ylabel='mrad',
                               series=[(v, ds(runs[v][1]['t'] - T_SETTLE), ds(1e3 * (runs[v][1]['q'][:, 0] - runs[v][1]['q_ref'][:, 0]))) for v in runs]),
                          dict(title='torso iq/imax (GIM)', xlabel='t (s)', ylabel='iq/imax',
                               series=[('iq', ds(runs['gim_grav'][1]['t'] - T_SETTLE), ds(runs['gim_grav'][1]['iq'][:, 0] / runs['gim_grav'][0].P['i_max'][0]))])]
                miniplot.plot(str(out / f'{tag}.png'), panels, ncols=2, cell=(700, 400))
        if 'B' in which:
            for T in (2.0, 1.0):
                tr = traj_bend(T)
                tag = f'B_{pk}_T{T:g}'
                print(tag, flush=True)
                runs, met = run_case(tag, tr, payload, GIM_V + STOCK_V, out, tr.windows['active'], 'B')
                allm[tag] = met
                plot_case(out / f'{tag}.png', tag, tr, runs, runs['gim_grav'][0], (T_SETTLE - 0.2, tr.t[-1]))
                save_npz(out / f'{tag}.npz', runs)
        if 'C' in which:
            tr = traj_reach()
            tag = f'C_{pk}'
            print(tag, flush=True)
            runs, met = run_case(tag, tr, payload, GIM_V + STOCK_V, out, tr.windows['active'], 'C')
            allm[tag] = met
            plot_case(out / f'{tag}.png', tag, tr, runs, runs['gim_grav'][0], (T_SETTLE - 0.2, tr.t[-1]))
            save_npz(out / f'{tag}.npz', runs)
        (out / 'metrics.json').write_text(json.dumps(allm, indent=1))
    (out / 'metrics.json').write_text(json.dumps(allm, indent=1))
    print(out)


if __name__ == '__main__':
    main()
