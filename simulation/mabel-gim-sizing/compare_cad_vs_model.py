"""What changed between the first run (model inertia, point-mass) and the CAD run. Writes cad_vs_model.md into the CAD folder.
Run with system python: python -I -B mabel_gim/compare_cad_vs_model.py <old_dir> <new_dir>"""
import json, sys
from pathlib import Path
import numpy as np
old, new = Path(sys.argv[1]), Path(sys.argv[2])
A, B = json.loads((old / 'metrics.json').read_text()), json.loads((new / 'metrics.json').read_text())
RA, RB = json.loads((old / 'rule.json').read_text()), json.loads((new / 'rule.json').read_text())
SETS = [s for s in A if s in B and s not in ('stock_gim_nomass',)]
L = ['# CAD inertia and actuator tensor run vs first run (model inertia, point mass)\n']
L.append('## Pass/fail flips\n')
flips = 0
L.append('| set | scenario | first run | CAD run | joints failing now |')
L.append('|---|---|---|---|---|')
for s in SETS[1:]:
    for k in RA[s]:
        a, b = RA[s][k]['passed'], RB[s][k]['passed']
        if a != b:
            flips += 1
            L.append(f"| {s} | {k} | {'pass' if a else 'FAIL'} | {'pass' if b else 'FAIL'} | {', '.join(f['joint'] for f in RB[s][k]['fails'][:6])} |")
if not flips: L.append('| none | | | | |')
L.append('\nOverall pass (all scenarios): ' + '; '.join(f"{s}: {'yes' if all(v['passed'] for v in RA[s].values()) else 'no'} -> {'yes' if all(v['passed'] for v in RB[s].values()) else 'no'}" for s in SETS[1:]))
L.append('\n## Torso, B T=1 s 3 kg: peak iq/i_max, peak error mrad, saturated %, peak bus motoring W\n')
L.append('| set | iq first | iq CAD | err first | err CAD | sat first | sat CAD | bus first | bus CAD |')
L.append('|---|---|---|---|---|---|---|---|---|')
for s in SETS:
    a, b = A[s]['B_T1_3kg'], B[s]['B_T1_3kg']
    ta, tb = a['groups']['Torso'], b['groups']['Torso']
    L.append(f"| {s} | {ta['peak_iq_frac']:.2f} | {tb['peak_iq_frac']:.2f} | {1e3*ta['peak_err']:.1f} | {1e3*tb['peak_err']:.1f} | {100*ta['frac_sat']:.1f} | {100*tb['frac_sat']:.1f} | {a['bus']['peak_motoring_W']:.0f} | {b['bus']['peak_motoring_W']:.0f} |")
L.append('\n## Arm groups, D 3 kg: peak error mrad / peak iq (first -> CAD)\n')
L.append('| set | Arm1-2 | Arm3-4 | Arm5-7 |')
L.append('|---|---|---|---|')
for s in SETS:
    c = lambda g: f"{1e3*A[s]['D_3kg']['groups'][g]['peak_err']:.1f} / {A[s]['D_3kg']['groups'][g]['peak_iq_frac']:.2f} -> {1e3*B[s]['D_3kg']['groups'][g]['peak_err']:.1f} / {B[s]['D_3kg']['groups'][g]['peak_iq_frac']:.2f}"
    L.append(f"| {s} | {c('Arm1-2')} | {c('Arm3-4')} | {c('Arm5-7')} |")
L.append('\n## Backdrive: rotation at 0.25 s of Arm 1, Arm 4, Arm 7 (rad), first -> CAD; Arm 1/4 reflected inertia kg m2\n')
L.append('| set | Arm 1 | Arm 4 | Arm 7 | Arm 1 inertia | Arm 4 inertia |')
L.append('|---|---|---|---|---|---|')
JN = ['Torso'] + [f'{s} Arm {i}' for s in ('Left', 'Right') for i in range(1, 8)]
for s in SETS:
    a, b = A[s]['backdrive'], B[s]['backdrive']
    c = lambda n: f"{a['push']['dq_at_025s'][JN.index(f'Left Arm {n}')]:+.2f} -> {b['push']['dq_at_025s'][JN.index(f'Left Arm {n}')]:+.2f}"
    L.append(f"| {s} | {c(1)} | {c(4)} | {c(7)} | {a['reflected_inertia_kgm2'][1]:.3f} -> {b['reflected_inertia_kgm2'][1]:.3f} | {a['reflected_inertia_kgm2'][4]:.3f} -> {b['reflected_inertia_kgm2'][4]:.3f} |")
(new / 'cad_vs_model.md').write_text('\n'.join(L) + '\n')
print('\n'.join(L))
