"""Pass/fail of every CAD-run design set against three stock reflected-inertia values (same pre-registered rule).
python -I -B mabel_gim/verdict_three_stocks.py <cad_sets_dir> <stock_nominal_dir> <stock_pjinertia_dir> -> three_stocks.md in the first folder"""
import json, sys
from pathlib import Path
D = [Path(a) for a in sys.argv[1:4]]
cad = json.loads((D[0] / 'metrics.json').read_text())
STK = {'low (reseller) 0.156/0.130': cad['stock_gim'],
       'nominal (geometric) 0.25/0.14': json.loads((D[1] / 'metrics.json').read_text())['stock_gim_nominal'],
       'high (per_joint) 0.318/0.170': json.loads((D[2] / 'metrics.json').read_text())['stock_gim_pjinertia']}
RULE = ['B_T2', 'B_T1', 'C', 'D']
PAY = ['0kg', '3kg']
JN = ['Torso'] + [f'{s} Arm {i}' for s in ('Left', 'Right') for i in range(1, 8)]
SETS = [s for s in cad if s not in ('stock_gim', 'stock_gim_nomass')]
def verdict(s, st):
    fails = []
    for key in RULE:
        for p in PAY:
            k = f'{key}_{p}'
            for jn in JN:
                m, ms = cad[s][k]['joints'][jn], st[k]['joints'][jn]
                if not m.get('n'): continue
                if m['frac_sat'] > 0.02 or m['peak_err'] > max(1.5 * ms['peak_err'], 0.010):
                    fails.append((k, jn, m['peak_err'], max(1.5 * ms['peak_err'], 0.010), m['frac_sat']))
    return fails
L = ['# Verdict per set against three stock inertia values (CAD design run, rule unchanged)\n',
     'Stock reflected inertia (GIM8115 / GIM8108, GIM4310 0.032 in all): low 0.156/0.130 (reseller-based), nominal 0.25/0.14 (geometric estimate, 5 Nov, +/-30 %), high 0.318/0.170 (per_joint surrogate). Designs identical in all three columns; only the stock reference errors change the limit.\n',
     '| set | ' + ' | '.join(STK) + ' | verdict changes with stock value |', '|---|' + '---|' * (len(STK) + 1)]
V = {s: {n: verdict(s, st) for n, st in STK.items()} for s in SETS}
for s in SETS:
    cells = []
    for n in STK:
        f = V[s][n]
        if not f: cells.append('pass')
        else:
            k, jn, e, lim, sat = f[0]
            cells.append(f"FAIL ({len(f)}): {k} {jn.replace('Left ','L').replace('Right ','R')} {1e3*e:.1f} vs {1e3*lim:.1f} mrad" if sat <= 0.02 else f"FAIL ({len(f)}): {k} {jn} saturated {100*sat:.0f} %")
    changes = len({bool(V[s][n]) for n in STK}) > 1
    L.append(f'| {s} | ' + ' | '.join(cells) + f" | {'YES' if changes else 'no'} |")
L.append('\nStock D 0 kg Arm 1 peak error (mrad) and the 1.5x limit: ' + '; '.join(f"{n}: {1e3*st['D_0kg']['joints']['Left Arm 1']['peak_err']:.1f} -> limit {1e3*max(1.5*st['D_0kg']['joints']['Left Arm 1']['peak_err'],0.01):.1f}" for n, st in STK.items()))
(D[0] / 'three_stocks.md').write_text('\n'.join(L) + '\n')
print('\n'.join(L))
