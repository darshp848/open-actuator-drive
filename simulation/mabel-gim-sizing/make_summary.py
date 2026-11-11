"""Markdown tables from the latest scenarios/sweep/gate results. Run: python -I -B mabel_gim/make_summary.py <scenarios_dir>"""
import json, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
d = Path(sys.argv[1])
m = json.loads((d / 'metrics.json').read_text())
L = []
f = lambda x, n=3: f'{x:.{n}g}'
L.append('## A. Gravity hold (2 s after 1 s settling)\n')
L.append('| case | model | torso steady err (mrad) | creep (mrad/s) | stuck fraction | at joint limit | mean iq / i_max | mean motor torque (Nm) | gravity at pose (Nm) | friction band (Nm) |')
L.append('|---|---|---|---|---|---|---|---|---|---|')
for tag, c in m.items():
    if not tag.startswith('A_'):
        continue
    g = c['gim_grav']['gravity_torque_at_actual_pose_Nm']['Torso']
    for v in ('gim_grav', 'stock_plain', 'stock_ff'):
        h = c[v]['torso_hold']
        if v == 'gim_grav':
            L.append(f"| {tag[2:]} | GIM | {f(h['steady_err_rad']*1e3)} | {f(h['creep_mrad_s'])} | {f(h['frac_stuck'])} | {f(h['frac_at_joint_limit'])} | {f(h['mean_iq_over_imax'])} | {f(h['mean_motor_torque_Nm'])} | {f(g)} | {f(h['friction_band_Nm'])} |")
        else:
            L.append(f"| {tag[2:]} | {v} | {f(h['steady_err_rad']*1e3)} | {f(h['creep_mrad_s'])} | {f(h['frac_stuck'])} | {f(h['frac_at_joint_limit'])} | n/a | {f(h['mean_torque_Nm'])} (actuator) | {f(g)} | n/a |")
L.append('')
for scn, title, win in (('B', 'B. Torso bend (settle 1 s, bend T, hold 1 s, return T, 0.5 s tail); metrics over the whole move', 'active'),
                        ('C', 'C. Combined reach (lift 5 s, torso -0.8 rad and both Arm 1 in 2 s); metrics over the whole 5.5 s', 'active')):
    L.append(f'## {title}\n')
    L.append('| case | model | peak err (mrad) | RMS err (mrad) | peak abs iq / i_max | time on voltage limit | time on current limit | peak output torque (Nm) | peak speed (rad/s) |')
    L.append('|---|---|---|---|---|---|---|---|---|')
    for tag, c in m.items():
        if not tag.startswith(scn + '_'):
            continue
        for v in ('gim_grav', 'gim_full', 'gim_grav_v0', 'stock_plain', 'stock_ff'):
            a = c[v]['torso'][win]
            if v.startswith('gim'):
                L.append(f"| {tag[2:]} | {v} | {f(a['peak_abs_err_rad']*1e3)} | {f(a['rms_err_rad']*1e3)} | {f(a['peak_iq_over_imax'])} | {f(a['frac_voltage_limit'])} | {f(a['frac_current_limit'])} | {f(a['peak_output_torque_Nm'])} | {f(a['peak_speed_rad_s'])} |")
            else:
                L.append(f"| {tag[2:]} | {v} | {f(a['peak_abs_err_rad']*1e3)} | {f(a['rms_err_rad']*1e3)} | n/a | n/a (force limit {f(a['frac_at_force_limit'])}) | | {f(a['peak_output_torque_Nm'])} | {f(a['peak_speed_rad_s'])} |")
    L.append('')
L.append('## B. Torso DC bus power and energy (GIM, gravity feedforward), by phase\n')
L.append('| case | phase | motoring energy (J) | regen energy (J) | net (J) | peak motoring (W) | peak regen (W) |')
L.append('|---|---|---|---|---|---|---|')
for tag, c in m.items():
    if not tag.startswith('B_'):
        continue
    for ph in ('fwd', 'hold', 'ret', 'active'):
        b = c['gim_grav']['torso'][ph]['bus']
        L.append(f"| {tag[2:]} | {ph} | {f(b['E_motoring_J'])} | {f(b['E_regen_J'])} | {f(b['net_J'])} | {f(b['peak_motoring_W'])} | {f(b['peak_regen_W'])} |")
L.append('')
L.append('## C. Arm J1/J2 and torso, GIM gravity feedforward, whole move\n')
L.append('| case | joint | peak err (mrad) | peak abs iq / i_max | time on any limit | peak output torque (Nm) | peak speed (rad/s) | stock servo: peak torque (Nm) / force limit (Nm) / time at limit |')
L.append('|---|---|---|---|---|---|---|---|')
for tag, c in m.items():
    if not tag.startswith('C_'):
        continue
    for jn in ('Left Arm 1', 'Left Arm 2', 'Right Arm 1', 'Right Arm 2'):
        a, s = c['gim_grav']['arms_active'][jn], c['stock_plain']['arms_active'][jn]
        L.append(f"| {tag[2:]} | {jn} | {f(a['peak_abs_err_rad']*1e3)} | {f(a['peak_iq_over_imax'])} | {f(a['frac_any_limit'])} | {f(a['peak_output_torque_Nm'])} | {f(a['peak_speed_rad_s'])} | {f(s['peak_output_torque_Nm'])} / {f(s['listed_torque_Nm'])} / {f(s['frac_at_force_limit'])} |")
    a = c['gim_grav']['torso']['active']
    L.append(f"| {tag[2:]} | Torso | {f(a['peak_abs_err_rad']*1e3)} | {f(a['peak_iq_over_imax'])} | {f(a['frac_any_limit'])} | {f(a['peak_output_torque_Nm'])} | {f(a['peak_speed_rad_s'])} | see table above |")
L.append('')
L.append('## Arm joints: highest peak iq / i_max and saturation over all GIM runs (A, B, C)\n')
worst = {}
for tag, c in m.items():
    if tag[:2] not in ('B_', 'C_'):
        continue
    for v in ('gim_grav', 'gim_full', 'gim_grav_v0'):
        for jn, a in c[v]['arms_active'].items():
            w = worst.setdefault(jn, [0, 0])
            w[0] = max(w[0], a['peak_iq_over_imax']); w[1] = max(w[1], a['frac_any_limit'])
L.append('| joint | max peak iq / i_max | max time on a limit |')
L.append('|---|---|---|')
for jn, w in worst.items():
    L.append(f'| {jn} | {f(w[0])} | {f(w[1])} |')
L.append('')
L.append('## Stock servo arm joints at their force limit (any scenario, stock_plain and stock_ff)\n')
L.append('| case | joint | time at force limit | peak torque (Nm) | limit (Nm) |')
L.append('|---|---|---|---|---|')
for tag, c in m.items():
    if tag[:2] not in ('B_', 'C_'):
        continue
    for v in ('stock_plain', 'stock_ff'):
        for jn, a in c[v]['arms_active'].items():
            if a['frac_at_force_limit'] > 0:
                L.append(f"| {tag} {v} | {jn} | {f(a['frac_at_force_limit'])} | {f(a['peak_output_torque_Nm'])} | {f(a['listed_torque_Nm'])} |")
(d / 'summary.md').write_text('\n'.join(L))
print('\n'.join(L))
