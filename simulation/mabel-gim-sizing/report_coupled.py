"""Tables and plots for a coupled run. System python:
python -I -B mabel_gim/report_coupled.py <coupled_dir> [uncoupled_cad_dir]
Writes comparison.md, rule.json, torso and summary PNGs into <coupled_dir>."""
import json
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

RD = Path(sys.argv[1])
M = json.loads((RD / 'metrics.json').read_text())
SER = np.load(RD / 'series.npz')
M.pop('_designs_dir', None)
STK = {'low': 'stock_gim', 'nominal': 'stock_gim_nominal', 'high': 'stock_gim_pjinertia'}
SETS = [s for s in M if not s.startswith('stock_')]
RULE, PAY = ['B_T2', 'B_T1', 'C', 'D'], ['0kg', '3kg']
JN = ['Torso'] + ['%s Arm %d' % (s, i) for s in ('Left', 'Right') for i in range(1, 8)]
GR = ('Torso', 'Arm1-2', 'Arm3-4', 'Arm5-7')


def verdict(s, st):
    f = []
    for key in RULE:
        for p in PAY:
            k = '%s_%s' % (key, p)
            for jn in JN:
                m, ms = M[s][k]['joints'][jn], M[st][k]['joints'][jn]
                if not m.get('n'):
                    continue
                lim = max(1.5 * ms['peak_err'], 0.010)
                if m['frac_sat'] > 0.02 or m['peak_err'] > lim:
                    cause = 'tracking'
                    if m['frac_sat'] > 0.02:
                        cause = 'current' if m['frac_i'] >= m['frac_v'] else 'speed'
                    f.append(dict(k=k, joint=jn, sat=m['frac_sat'], err=m['peak_err'], lim=lim, cause=cause))
    return f


V = {s: {n: verdict(s, st) for n, st in STK.items()} for s in SETS}
json.dump(V, open(RD / 'rule.json', 'w'), indent=1)


def cell(f):
    if not f:
        return 'pass'
    x = f[0]
    j = x['joint'].replace('Left ', 'L').replace('Right ', 'R')
    if x['sat'] > 0.02:
        return 'FAIL (%d): %s %s sat %.0f %% (%s)' % (len(f), x['k'], j, 100 * x['sat'], x['cause'])
    return 'FAIL (%d): %s %s %.1f vs %.1f mrad' % (len(f), x['k'], j, 1e3 * x['err'], 1e3 * x['lim'])


L = ['# Coupled sets (picks sized for their own arm masses), rule unchanged\n',
     'Pass/fail against three stock reflected inertias (low 0.156/0.130, nominal 0.25/0.14, high 0.318/0.170 kg m2).\n',
     '| set | low | nominal | high | verdict changes with stock |', '|---|---|---|---|---|']
for s in SETS:
    ch = len({bool(V[s][n]) for n in STK}) > 1
    L.append('| %s | %s | %s | %s | %s |' % (s, cell(V[s]['low']), cell(V[s]['nominal']), cell(V[s]['high']), 'YES' if ch else 'no'))
L.append('\n## Torso and arms, B T=1 s (brake), nominal stock first\n')
L.append('| set | robot mass kg | 0 kg torso iq/imax | 0 kg err mrad | 3 kg torso iq/imax | 3 kg err mrad | 3 kg sat % | worst arm err 3 kg mrad | bus peak motoring / regen W (3 kg) |')
L.append('|---|---|---|---|---|---|---|---|---|')
names = ['stock_gim_nominal'] + SETS
for s in names:
    a, b = M[s]['B_T1_0kg'], M[s]['B_T1_3kg']
    arms = max(b['groups'][g]['peak_err'] for g in GR[1:])
    L.append('| %s | %.1f | %.2f | %.1f | %.2f | %.1f | %.1f | %.1f | %.0f / %.0f |' % (
        s, b['mass']['total_kg'], a['groups']['Torso']['peak_iq_frac'], 1e3 * a['groups']['Torso']['peak_err'],
        b['groups']['Torso']['peak_iq_frac'], 1e3 * b['groups']['Torso']['peak_err'], 100 * b['groups']['Torso']['frac_sat'],
        1e3 * arms, b['bus']['peak_motoring_W'], b['bus']['peak_regen_W']))
for key in ('B_T2_3kg', 'C_3kg', 'D_3kg', 'D_fast_3kg'):
    L.append('\n## %s: worst-joint metrics per group (peak err mrad / peak iq/imax / sat %%), nominal stock first\n' % key)
    L.append('| set | Torso | Arm1-2 | Arm3-4 | Arm5-7 | bus peak motoring / regen W |')
    L.append('|---|---|---|---|---|---|')
    for s in names:
        g = M[s][key]['groups']

        def c(n):
            if g[n].get('peak_err') is None:
                return '-'
            return '%.1f / %.2f / %.1f' % (1e3 * g[n]['peak_err'], g[n]['peak_iq_frac'], 100 * g[n]['frac_sat'])
        L.append('| %s | %s | %s | %s | %s | %.0f / %.0f |' % (s, c('Torso'), c('Arm1-2'), c('Arm3-4'), c('Arm5-7'),
                                                               M[s][key]['bus']['peak_motoring_W'], M[s][key]['bus']['peak_regen_W']))
L.append('\n## Backdrive (zero current, torso locked, gravity off, 10 N forward push): rotation at 0.25 s (rad) of Arm 1, Arm 4, Arm 7; Arm 1 reflected inertia; Arm 4 breakaway ratio\n')
L.append('| set | Arm 1 | Arm 4 | Arm 7 | Arm 1 inertia kg m2 | Arm 4 breakaway ratio |')
L.append('|---|---|---|---|---|---|')
for s in names:
    b = M[s]['backdrive']
    d = lambda n: '%+.2f' % b['push']['dq_at_025s'][JN.index('Left Arm %d' % n)]
    L.append('| %s | %s | %s | %s | %.3f | %.1f |' % (s, d(1), d(4), d(7), b['reflected_inertia_kgm2'][1], b['breakaway_ratio'][JN.index('Left Arm 4')]))
if len(sys.argv) > 2:
    U = json.loads((Path(sys.argv[2]) / 'metrics.json').read_text())
    UR = json.loads((Path(sys.argv[2]) / 'rule.json').read_text())
    L.append('\n## Change versus the uncoupled CAD run (same set names; verdict against the low stock, as that run was)\n')
    L.append('| set | uncoupled verdict | coupled verdict (low stock) | torso iq/imax 3 kg uncoupled -> coupled | torso err mrad uncoupled -> coupled |')
    L.append('|---|---|---|---|---|')
    for s in SETS:
        if s in U:
            uf = [k for k, r in UR[s].items() if not r['passed']]
            a, b = U[s]['B_T1_3kg']['groups']['Torso'], M[s]['B_T1_3kg']['groups']['Torso']
            L.append('| %s | %s | %s | %.2f -> %.2f | %.1f -> %.1f |' % (
                s, 'pass' if not uf else 'FAIL ' + ','.join(uf[:3]), cell(V[s]['low']),
                a['peak_iq_frac'], b['peak_iq_frac'], 1e3 * a['peak_err'], 1e3 * b['peak_err']))
(RD / 'comparison.md').write_text('\n'.join(L) + '\n')
print('\n'.join(L))
cm = plt.cm.tab20(np.linspace(0, 1, 20))
fig, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
for i, s in enumerate(names):
    k = '%s/B_T1_3kg/' % s
    if k + 't' not in SER.files:
        continue
    lw = 2.4 if s.startswith('stock') else 1.0
    c = 'k' if s.startswith('stock') else cm[i % 20]
    ax[0].plot(SER[k + 't'], SER[k + 'iq_frac'], color=c, lw=lw, label=s)
    ax[1].plot(SER[k + 't'], SER[k + 'p_dc_total'], color=c, lw=lw)
    ax[2].plot(SER[k + 't'], 1e3 * (SER[k + 'q'] - SER[k + 'q_ref']), color=c, lw=lw)
ax[0].set_ylabel('torso iq / i_max')
ax[1].set_ylabel('bus power, all joints, W')
ax[2].set_ylabel('torso error, mrad')
ax[2].set_xlabel('t, s')
ax[0].legend(fontsize=5, ncol=4)
for a in ax:
    a.grid(alpha=.3)
fig.suptitle('Coupled sets, B T=1 s, 3 kg per hand (brake during the flat middle)')
fig.tight_layout()
fig.savefig(RD / 'torso_B_T1_3kg.png', dpi=110)
plt.close(fig)
for key in ('B_T1_3kg', 'D_3kg'):
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.2))
    w = 0.8 / len(names)
    for i, s in enumerate(names):
        g = M[s][key]['groups']
        xs = np.arange(4) + (i - len(names) / 2) * w
        c = 'k' if s.startswith('stock') else cm[i % 20]
        ax[0].bar(xs, [g[n].get('peak_iq_frac') or 0 for n in GR], w, color=c, label=s)
        ax[1].bar(xs, [1e3 * (g[n].get('peak_err') or 0) for n in GR], w, color=c)
    for a, t in zip(ax, ('peak |iq| / i_max', 'peak tracking error mrad')):
        a.set_xticks(range(4))
        a.set_xticklabels(GR)
        a.set_title(t)
        a.grid(axis='y', alpha=.3)
    ax[1].set_yscale('log')
    ax[0].axhline(1, color='r', lw=.8)
    ax[0].legend(fontsize=5, ncol=3)
    fig.suptitle(key + ': coupled sets')
    fig.tight_layout()
    fig.savefig(RD / ('summary_%s.png' % key), dpi=110)
    plt.close(fig)
print('wrote', RD)
