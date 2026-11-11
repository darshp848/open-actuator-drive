"""Stock vendor servo vs GIM layer: (a) 1.6 s motion, (b) reflected inertia, (c) torque-speed envelope,
(d) interaction impedance. Writes results/<timestamp>_compare/{comparison.md,comparison.json,*.png}.
Run from simulation/openarm_v2:  python gim/compare.py"""
import json
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import robot  # noqa: E402
from actuated import GimArm, Config  # noqa: E402
import params as G  # noqa: E402
import scenarios as S  # noqa: E402
import miniplot  # noqa: E402

HERE = Path(__file__).resolve().parent
VEL_LIMIT = [16.755, 16.755, 5.4454, 5.4454, 20.944, 20.944, 20.944]       # run_diagnostics.py


def table(head, rows):
    return '\n'.join(['| ' + ' | '.join(head) + ' |', '|' + '---|' * len(head)] +
                     ['| ' + ' | '.join(str(c) for c in r) + ' |' for r in rows]) + '\n'


def motion(out):
    stock = S.run_stock()
    runs = {'stock': stock, 'GIM rigid': S.run_gim(Config(transmission='rigid')),
            'GIM compliant': S.run_gim(Config(transmission='compliant'))}
    res = {}
    for k, r in runs.items():
        err = np.abs(r['q'] - r['qdes'])
        pk = np.abs(r['tau']).max(0)
        res[k] = dict(max_err_rad=float(err.max()), max_err_by_joint=err.max(0).reshape(2, 7).max(0).tolist(),
                      peak_tau_by_joint=pk.reshape(2, 7).max(0).tolist())
    rows = []
    for k in ('stock', 'GIM rigid', 'GIM compliant'):
        rows.append([k, f"{res[k]['max_err_rad']:.4f}"] + [f'{x:.1e}' if x < 1e-3 else f'{x:.2f}' for x in res[k]['peak_tau_by_joint']])
    for k in ('GIM rigid', 'GIM compliant'):
        rows.append([f'{k} / stock peak torque', ''] + [f"{a / b:.2f}" if b > 0.05 else 'n/a (<0.05 Nm)'
                                                      for a, b in zip(res[k]['peak_tau_by_joint'], res['stock']['peak_tau_by_joint'])])
    md = '## (a) Codex 1.6 s bimanual motion, vendor kp/kd (v_ref = 0), 0.5 ms physics, 1 kHz commands\n\n'
    md += table(['run', 'max arm tracking err (rad)'] + [f'peak |tau| J{i} (Nm)' for i in range(1, 8)], rows)
    md += ('\nTorque is the actuator output torque (stock: `actuator_force`, GIM: `tau_out` before friction); peak over both arms. '
           'GIM compliant tracking is measured at the link, controlled from the rotor encoder.\n')
    j4 = robot.ARMS.index('openarm_right_joint4')
    miniplot.plot(out / 'motion.png', [
        dict(title='right J4 position (rad)', xlabel='t (s)', ylabel='rad',
             series=[(k, r['t'], r['q'][:, j4]) for k, r in runs.items()]),
        dict(title='right J4 output torque (Nm)', xlabel='t (s)', ylabel='Nm',
             series=[(k, r['t'], r['tau'][:, j4]) for k, r in runs.items()]),
        dict(title='right J4 tracking error (rad)', xlabel='t (s)', ylabel='rad',
             series=[(k, r['t'], r['q'][:, j4] - r['qdes'][:, j4]) for k, r in runs.items()]),
        dict(title='right J1 output torque (Nm)', xlabel='t (s)', ylabel='Nm',
             series=[(k, r['t'], r['tau'][:, 7]) for k, r in runs.items()])])
    return md, res


def inertia():
    m, _ = robot.load()
    P = G.arm_params()
    rows, res = [], []
    for i in range(7):
        j = m.joint(robot.ARMS[7 + i])
        va = float(m.dof_armature[j.dofadr[0]])
        jr = float(P['J_rotor'][i] * P['N'][i] ** 2)
        rows.append([f'J{i + 1}', G.MOTOR_NAMES[i], f'{va:.3f}', f'{P["J_rotor"][i]:.1e}', int(P['N'][i]), f'{jr:.3f}', f'{jr / va:.1f}x',
                     f"{m.dof_damping[j.dofadr[0]]:.2f} / {m.dof_frictionloss[j.dofadr[0]]:.2f}",
                     f"{P['b'][i]:.2f} / {P['tau_c'][i]:.2f}"])
        res.append(dict(joint=i + 1, vendor_armature=va, gim_reflected=jr))
    md = '## (b) Reflected rotor inertia and joint friction\n\n'
    md += table(['joint', 'motor', 'vendor armature (kg m2)', 'J_rotor (ASSUMED)', 'N', 'J_rotor N^2 (kg m2)', 'ratio',
                 'vendor damping / frictionloss', 'GIM b / tau_c (ASSUMED)'], rows)
    return md, res


def envelope(out):
    m, _ = robot.load()
    res, rows, curves = [], [], {}
    for src in ('kt', 'kv'):
        arm = GimArm(Config(lambda_source=src, log=False))
        P = arm.P
        for i in (0, 2, 4):
            w_nl = arm.vmax / (P['pp'][i] * P['lam'][i] * P['N'][i])
            ws = np.linspace(0, 1.05 * w_nl, 300)
            tau, his = [], []
            for w in ws:
                lo, hi, _ = arm.volt_limits(np.full(14, w))
                iq = min(P['i_max'][i], hi[i])
                his.append(hi[i])
                tau.append(P['N'][i] * P['kt_m'][i] * iq * P['eta'][i])
            tau = np.array(tau)
            corner = float(ws[np.argmax(np.array(his) < P['i_max'][i])])
            aj = m.actuator(f'right_joint{i + 1}_ctrl')
            fr = float(m.actuator_forcerange[aj.id, 1])
            rows.append([G.MOTOR_NAMES[i], src, f'{tau[0]:.1f}', f'{fr:.0f}', f'{corner:.2f}', f'{w_nl:.2f} ({w_nl * 30 / np.pi:.0f} rpm)',
                         f'{P["w_max_listed"][i]:.2f} ({P["w_max_listed"][i] * 30 / np.pi:.0f} rpm)', f'{VEL_LIMIT[i]:.2f}',
                         f'{np.max(np.maximum(tau, 0) * ws):.0f}'])
            res.append(dict(motor=G.MOTOR_NAMES[i], lambda_source=src, stall_nm=float(tau[0]), vendor_forcerange=fr,
                            corner_rad_s=corner, no_load_rad_s=float(w_nl), listed_rad_s=float(P['w_max_listed'][i]),
                            vendor_vel_limit=VEL_LIMIT[i], peak_mech_power_w=float(np.max(np.maximum(tau, 0) * ws))))
            if src == 'kt':
                curves[G.MOTOR_NAMES[i]] = (ws, tau, fr, VEL_LIMIT[i])
    md = '## (c) Torque-speed envelope at 48 V bus (id = 0, no field weakening, 95 % SVPWM)\n\n'
    md += table(['motor', 'lambda from', 'GIM motoring tau at 0 rpm (Nm)', 'vendor forcerange (Nm)', 'corner speed, current -> voltage limited (rad/s)',
                 'no-load speed (rad/s)', 'listed max out speed', 'vendor velocity limit (rad/s)', 'peak mech power (W)'], rows)
    md += ('\nVendor envelope is the rectangle forcerange x velocity limit from run_diagnostics.py (no voltage limit). '
           'lambda "kt" follows openarm_params.m (eta = 0.9 folded into kt_m); "kv" uses the listed rpm/V.\n')
    pan = []
    for name, (ws, tau, fr, vl) in curves.items():
        pan.append(dict(title=f'{name}: torque-speed at 48 V', xlabel='output speed (rad/s)', ylabel='Nm',
                        series=[('GIM (kt lambda)', ws, tau), ('vendor rectangle', [0, vl, vl, 0, 0], [fr, fr, 0, 0, fr])]))
    miniplot.plot(out / 'envelope.png', pan, ncols=3, cell=(560, 400))
    return md, res


def impedance(out):
    cases = [('openarm_right_joint4', 5.0), ('openarm_right_joint6', 1.0)]
    res, rows, pan = {}, [], []
    for jn, tq in cases:
        series = []
        for kind in ('stock', 'rigid', 'compliant'):
            r, ts = S.run_impedance(kind, jn, tq)
            res[f'{jn}_{kind}'] = r
            rows.append([jn.split('_', 1)[1], f'{tq:g}', 'stock servo' if kind == 'stock' else f'GIM {kind}',
                         f"{r['peak_deflection_rad'] * 1e3:.1f}", f"{r['steady_deflection_rad'] * 1e3:.1f}",
                         f"{r['stiffness_Nm_per_rad']:.0f}", f"{r['settle_time_s'] * 1e3:.0f}", f"{r['residual_offset_rad'] * 1e3:.2f}"])
            series.append((kind, ts['t'], (ts['q'] - r['pre_step_rad']) * 1e3))
        pan.append(dict(title=f'{jn[-6:]}: deflection (mrad), {tq:g} Nm from 0.5 s to 0.8 s', xlabel='t (s)', ylabel='mrad', series=series))
    miniplot.plot(out / 'impedance.png', pan, ncols=2)
    md = '## (d) Interaction impedance: hold at home with gravity feedforward, torque step 0.5-0.8 s through qfrc_applied\n\n'
    md += table(['joint', 'step (Nm)', 'plant', 'peak defl (mrad)', 'steady defl (mrad)', 'secant stiffness (Nm/rad)',
                 'settle to 5 % (ms)', 'residual after release (mrad)'], rows)
    md += ('\nAll plants use the vendor kp/kd (J4: 190 / 2.2, J6: 30 / 1.5). Deflection is measured at the link relative to the settled pre-step pose; '
           'residual is the mean over the last 0.1 s of the 1.5 s run (friction/backlash stiction).\n')
    return md, res


if __name__ == '__main__':
    out = HERE / 'results' / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_compare')
    out.mkdir(parents=True, exist_ok=False)
    md, js = '# Stock vendor servo vs GIM actuator layer\n\n', {}
    for name, fn, a in [('motion', motion, (out,)), ('inertia', inertia, ()), ('envelope', envelope, (out,)), ('impedance', impedance, (out,))]:
        m_, js[name] = fn(*a)
        md += m_ + '\n'
        print(m_, flush=True)
    md += '\nFigures: motion.png, envelope.png, impedance.png.\n'
    (out / 'comparison.md').write_text(md)
    (out / 'comparison.json').write_text(json.dumps(js, indent=2, default=float))
    print('wrote', out)
