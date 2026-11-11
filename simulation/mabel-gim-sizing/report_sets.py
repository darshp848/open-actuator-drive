"""Tables and plots for the actuator-set comparison. Reads results/<ts>_sets/{metrics.json, series.npz}.
Run with the system python (matplotlib): python -I -B mabel_gim/report_sets.py [results_dir]
Writes comparison.md, rule.json and PNGs into the same folder."""
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
RESDIR = Path(sys.argv[1]) if len(sys.argv) > 1 else sorted((HERE / 'results').glob('*_sets'))[-1]
M = json.loads((RESDIR / 'metrics.json').read_text())
SER = np.load(RESDIR / 'series.npz')

SETS = ['stock_gim', 'main_ndfeb', 'main_ferrite'] + [f'{c}_{f}' for c in ('mass_50', 'mass_100', 'size', 'backdrive', 'qdd') for f in ('ndfeb', 'ferrite')]
LABEL = {'stock_gim': 'stock GIM', 'main_ndfeb': 'NdFeB main (64:1 lightest)', 'main_ferrite': 'ferrite main (64:1 lightest)',
         'mass_50_ndfeb': 'NdFeB <=50:1', 'mass_50_ferrite': 'ferrite <=50:1', 'mass_100_ndfeb': 'NdFeB <=100:1', 'mass_100_ferrite': 'ferrite <=100:1',
         'size_ndfeb': 'NdFeB fits envelope', 'size_ferrite': 'ferrite fits envelope', 'backdrive_ndfeb': 'NdFeB ratio+inertia <= ref',
         'backdrive_ferrite': 'ferrite ratio+inertia <= ref', 'qdd_ndfeb': 'NdFeB QDD (<=10:1)', 'qdd_ferrite': 'ferrite QDD (<=10:1)'}
RULE = ['B_T2', 'B_T1', 'C', 'D']
PAY = ['0kg', '3kg']
JN = ['Torso'] + [f'{s} Arm {i}' for s in ('Left', 'Right') for i in range(1, 8)]
GROUPS = ['Torso', 'Arm1-2', 'Arm3-4', 'Arm5-7']
SAT_LIMIT, ERR_FACTOR, ERR_FLOOR = 0.02, 1.5, 0.010


def cause(m):
    if m['frac_sat'] > SAT_LIMIT:
        return 'current' if m['frac_i'] >= m['frac_v'] else 'speed (voltage)'
    return 'tracking'


def evaluate():
    out = {}
    for s in SETS[1:]:
        out[s] = {}
        for key in RULE:
            for p in PAY:
                k = f'{key}_{p}'
                fails = []
                for jn in JN:
                    m, ms = M[s][k]['joints'][jn], M['stock_gim'][k]['joints'][jn]
                    if not m.get('n'):
                        continue
                    sat = m['frac_sat'] > SAT_LIMIT
                    lim = max(ERR_FACTOR * ms['peak_err'], ERR_FLOOR)
                    err = m['peak_err'] > lim
                    if sat or err:
                        fails.append(dict(joint=jn, sat=sat, err=err, frac_sat=m['frac_sat'], peak_err=m['peak_err'], err_limit=lim,
                                          cause=('current' if m['frac_i'] >= m['frac_v'] else 'speed (voltage)') if sat else 'tracking'))
                out[s][k] = dict(passed=not fails, fails=fails)
    return out


R = evaluate()
(RESDIR / 'rule.json').write_text(json.dumps(R, indent=1))
f2 = lambda x, n=1: f'{x:.{n}f}'


def group_cell(s, k, g, field, scale=1.0, n=1):
    return f2(M[s][k]['groups'][g][field] * scale, n)


L = []
L.append(f'# Optimised actuators vs stock: comparison tables\n\nSource: `{RESDIR.name}/metrics.json`. Rule: no joint saturated more than 2 % of active time and peak error <= max(1.5 x stock, 10 mrad), per joint, in B T=2 s, B T=1 s, C and D at 0 kg and 3 kg per hand.\n')
# pass/fail
L.append('## Pass / fail per set\n')
L.append('| set | ' + ' | '.join(f'{k} {p}' for k in RULE for p in PAY) + ' | passes all |')
L.append('|---|' + '---|' * (len(RULE) * len(PAY) + 1))
for s in SETS[1:]:
    cells = []
    for key in RULE:
        for p in PAY:
            r = R[s][f'{key}_{p}']
            cells.append('pass' if r['passed'] else 'FAIL (%d)' % len(r['fails']))
    allp = all(R[s][k]['passed'] for k in R[s])
    L.append(f'| {LABEL[s]} | ' + ' | '.join(cells) + f' | **{"yes" if allp else "no"}** |')
L.append('\nNumber in brackets = joints that break the rule in that scenario.\n')
L.append('## What breaks the rule\n')
L.append('| set | scenario | joint | saturated time % | cause | peak err mrad | error limit mrad |')
L.append('|---|---|---|---|---|---|---|')
for s in SETS[1:]:
    for k, r in R[s].items():
        seen = {}
        for f in r['fails']:
            seen.setdefault(f['joint'].replace('Left ', 'L').replace('Right ', 'R'), f)
        for jn, f in list(seen.items())[:6]:
            L.append(f"| {LABEL[s]} | {k} | {jn} | {100*f['frac_sat']:.1f} | {f['cause']} | {1e3*f['peak_err']:.1f} | {1e3*f['err_limit']:.1f} |")
        if len(seen) > 6:
            L.append(f'| {LABEL[s]} | {k} | (+{len(seen)-6} more joints) | | | | |')
# per scenario tables
for key in RULE + ['D_fast']:
    for p in PAY:
        k = f'{key}_{p}'
        L.append(f'\n## {k}: group metrics (worst joint in group)\n')
        L.append('| set | group | peak err mrad | peak iq/imax | sat % | peak speed / no-load | peak torque Nm | Cu loss J (group sum) |')
        L.append('|---|---|---|---|---|---|---|---|')
        for s in SETS:
            for g in GROUPS:
                G = M[s][k]['groups'][g]
                if not G.get('peak_err') and G.get('n', 1) == 0:
                    continue
                L.append(f"| {LABEL[s]} | {g} | {1e3*G['peak_err']:.1f} | {G['peak_iq_frac']:.2f} | {100*G['frac_sat']:.1f} | {G['speed_over_noload']:.2f} | {G['peak_torque']:.0f} | {G['cu_J']:.0f} |")
        L.append('\nDC bus, all 15 actuated joints summed: ' + '; '.join(
            f"{LABEL[s]}: {M[s][k]['bus']['peak_motoring_W']:.0f} / {M[s][k]['bus']['peak_regen_W']:.0f} W, {M[s][k]['bus']['E_motoring_J']:.0f} / {M[s][k]['bus']['E_regen_J']:.0f} J" for s in SETS) + ' (peak motoring / peak regen W, energy motoring / regen J).')
# masses
L.append('\n## Mass bookkeeping (3 kg run, signed change at the child body, kg)\n')
L.append('| set | dm Torso | dm Arm1 | dm Arm3 | dm Arm5 | dm Arm7 | robot total kg | method torso |')
L.append('|---|---|---|---|---|---|---|---|')
for s in SETS:
    ms = M[s]['B_T1_3kg']['mass']
    d = ms['dm']
    L.append(f"| {LABEL[s]} | {d[0]:+.2f} | {d[1]:+.2f} | {d[3]:+.2f} | {d[5]:+.2f} | {d[7]:+.2f} | {ms['total_kg']:.1f} | {ms['method'][0]} |")
# hold reference
L.append('\n## Torso hold, brake vs no brake (reference only), T = 1 s bend\n')
L.append('| set | payload | brake: motor current during hold | no brake: mean abs iq/imax | no brake: Cu loss in 1 s hold J | no brake: bus energy J | no brake: hold error mrad |')
L.append('|---|---|---|---|---|---|---|')
for s in SETS:
    for p in PAY:
        hb, hn = M[s][f'B_T1_{p}']['hold_torso'], M[s][f'B_T1_nobrake_{p}']['hold_torso']
        L.append(f"| {LABEL[s]} | {p} | {hb['mean_abs_iq_frac']:.3f} of imax (last 50 ms pre-load) | {hn['mean_abs_iq_frac']:.2f} | {hn['cu_J']:.1f} | {hn['bus_E_J']:.1f} | {1e3*hn['peak_err']:.2f} |")
# backdrive
L.append('\n## Backdrive test (zero current, torso locked, gravity off, 10 N forward push on each palm, 1 s)\n')
L.append('Pose: Left Arm 1 = 1.0, Arm 2 = 0.3, Arm 4 = 0.3 rad (right mirrored). Joint rotation in rad at 0.25 s after the push starts, with the value at 1 s in brackets (the 1 s values include free swinging of the wrist and are less meaningful). Breakaway ratio = joint torque from the push divided by the Coulomb friction (above 1 means the joint moves).\n')
L.append('| set | Arm 1 dq at 0.25 s (1 s) | Arm 4 dq at 0.25 s (1 s) | Arm 7 dq at 0.25 s (1 s) | joints hitting a limit | Arm 1 breakaway ratio | Arm 4 breakaway ratio | Arm 1 reflected inertia kg m2 | Arm 1 Coulomb Nm | Arm 4 Coulomb Nm |')
L.append('|---|---|---|---|---|---|---|---|---|---|')
for s in SETS:
    b = M[s]['backdrive']
    dq = np.array(b['push']['dq']) - np.array(b['nopush']['dq'])
    dq25 = np.array(b['push']['dq_at_025s'])
    jl = {n: JN.index(f'Left Arm {n}') for n in (1, 2, 4, 7)}
    hit = sum(b['push']['hit_limit'])
    c = lambda n: f"{dq25[jl[n]]:+.2f} ({dq[jl[n]]:+.2f})"
    L.append(f"| {LABEL[s]} | {c(1)} | {c(4)} | {c(7)} | {hit} | {b['breakaway_ratio'][jl[1]]:.1f} | {b['breakaway_ratio'][jl[4]]:.1f} | {b['reflected_inertia_kgm2'][jl[1]]:.3f} | {b['coulomb_Nm'][jl[1]]:.2f} | {b['coulomb_Nm'][jl[4]]:.2f} |")
# gains
L.append('\n## Gain sanity (0.02 rad / 0.05 rad steps at the worst pose, 3 kg): overshoot %, tail ripple urad\n')
L.append('| set | Torso | Left Arm 1 | Left Arm 4 | Left Arm 7 |')
L.append('|---|---|---|---|---|')
for s in SETS:
    g = M[s]['gains']
    L.append(f'| {LABEL[s]} | ' + ' | '.join(f"{100*g[j]['overshoot']:.1f} %, {1e6*g[j]['tail_ripple_rad']:.0f}" for j in ('Torso', 'Left Arm 1', 'Left Arm 4', 'Left Arm 7')) + ' |')
# regression
L.append('\n## Regression: stock GIM with the old mass bookkeeping (Damiao masses in the links) vs earlier README numbers\n')
L.append('| case | torso peak err mrad | torso peak iq/imax | torso peak torque Nm | earlier README |')
L.append('|---|---|---|---|---|')
ref = {'B_T1_nobrake_3kg': '13.8 / 0.67 / 154', 'B_T1_nobrake_0kg': '6.6 / 0.34 / 79', 'B_T2_nobrake_3kg': '4.7 / 0.50 / 115', 'B_T2_nobrake_0kg': '2.8 / 0.28 / 64', 'C_3kg': '5.0 / 0.35 / 79', 'C_0kg': '2.5 / 0.19 / 44'}
for k, r in ref.items():
    j = M['stock_gim_nomass'][k]['joints']['Torso']
    L.append(f"| {k} | {1e3*j['peak_err']:.1f} | {j['peak_iq_frac']:.2f} | {j['peak_torque']:.0f} | {r} |")
(RESDIR / 'comparison.md').write_text('\n'.join(L) + '\n')

# ---------------------------------------------------------------- plots
COL = {'stock_gim': 'k'}
blues = plt.cm.Blues(np.linspace(0.45, 0.95, 6))
oranges = plt.cm.Oranges(np.linspace(0.45, 0.95, 6))
for i, c in enumerate(('main', 'mass_50', 'mass_100', 'size', 'backdrive', 'qdd')):
    COL[f'{c}_ndfeb'] = blues[i]
    COL[f'{c}_ferrite'] = oranges[i]
for key in RULE + ['D_fast']:
    for p in PAY:
        k = f'{key}_{p}'
        fig, ax = plt.subplots(1, 2, figsize=(13, 4.2))
        w = 0.075
        for i, s in enumerate(SETS):
            xs = np.arange(4) + (i - 6) * w
            ax[0].bar(xs, [M[s][k]['groups'][g].get('peak_iq_frac', 0) for g in GROUPS], w, color=COL[s], label=LABEL[s], edgecolor='none')
            ax[1].bar(xs, [1e3 * M[s][k]['groups'][g].get('peak_err', 0) for g in GROUPS], w, color=COL[s], edgecolor='none')
        for a, t in zip(ax, ('peak |iq| / i_max (worst joint in group)', 'peak tracking error, mrad (worst joint in group)')):
            a.set_xticks(range(4)); a.set_xticklabels(GROUPS); a.set_title(t); a.grid(axis='y', alpha=.3)
        ax[0].axhline(1, color='r', lw=0.8)
        ax[1].set_yscale('log')
        ax[0].legend(fontsize=6, ncol=2)
        fig.suptitle(f'{k}: actuator sets compared')
        fig.tight_layout()
        fig.savefig(RESDIR / f'summary_{k}.png', dpi=110)
        plt.close(fig)
for key in ('B_T1',):
    for p in PAY:
        k = f'{key}_{p}'
        fig, ax = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
        for s in SETS:
            t = SER[f'{s}/{k}/t']
            ls = '-' if s != 'stock_gim' else '-'
            lw = 2.2 if s == 'stock_gim' else 1.1
            ax[0].plot(t, SER[f'{s}/{k}/q'], color=COL[s], lw=lw, label=LABEL[s])
            ax[1].plot(t, SER[f'{s}/{k}/iq_frac'], color=COL[s], lw=lw)
            ax[2].plot(t, SER[f'{s}/{k}/p_dc_total'], color=COL[s], lw=lw)
            ax[3].plot(t, 1e3 * (SER[f'{s}/{k}/q'] - SER[f'{s}/{k}/q_ref']), color=COL[s], lw=lw)
        ax[0].plot(t, SER[f'stock_gim/{k}/q_ref'], 'g--', lw=0.8, label='reference')
        ax[0].set_ylabel('torso q, rad'); ax[1].set_ylabel('torso iq / i_max'); ax[2].set_ylabel('DC bus power, all joints, W'); ax[3].set_ylabel('torso error, mrad')
        ax[3].set_xlabel('t, s (bend starts at 0.2 s; the flat middle part is the 1 s brake hold)')
        ax[0].legend(fontsize=6, ncol=3)
        for a in ax: a.grid(alpha=.3)
        fig.suptitle(f'{k}: torso q, iq/i_max, DC bus power (sum over all actuated joints), torso tracking error')
        fig.tight_layout()
        fig.savefig(RESDIR / f'torso_{k}.png', dpi=110)
        plt.close(fig)
print('wrote', RESDIR)
