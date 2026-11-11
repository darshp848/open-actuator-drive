"""Consistency gate: GIM layer holding a static pose. Steady motor torque (N*kt_m*iq, the electromagnetic torque at the
output) vs the mj_inverse gravity torque at the settled pose. Friction off: must agree within 2 %. Friction on (the
production model): reported, and checked against the admissible static-friction interval.
Run: python -I -B mabel_gim/gate.py   -> results/<timestamp>_gate/gate.json"""
import json, sys, time
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import numpy as np
from scen import *


def hold(payload, friction, torso, secs=4.0):
    rig = Rig(payload, 'gim', Config(friction=friction))
    q0 = worst_pose(torso)
    rig.set_pose(q0)
    z = np.zeros(15)
    g = rig.inverse_torque(q0, z, z, 0.3175)
    rig.preload(q0, g)
    for k in range(round(secs / 1e-3)):
        rig.step_ms(q0, z, g, 0.3175)
    L = rig.log()
    mk = L['t'] > secs - 1.0
    qm = L['q'][mk].mean(0)
    gact = rig.inverse_torque(qm, z, z, 0.3175)
    tm = L['tau_m'][mk].mean(0)
    P = rig.P
    rows = {}
    for j, n in enumerate(JN):
        r = dict(gravity_Nm=float(gact[j]), motor_torque_Nm=float(tm[j]), delivered_Kt_iq_Nm=float(KT[j] * L['iq'][mk, j].mean()),
                 pos_err_mrad=float((qm[j] - q0[j]) * 1e3), peak_to_peak_v_rad_s=float(np.ptp(L['v'][mk, j])))
        if abs(gact[j]) > 2.0:
            r['rel_err_motor_vs_gravity'] = float((tm[j] - gact[j]) / gact[j])
        if friction:
            a = abs(gact[j]); tc, eta = P['tau_c'][j], P['eta'][j]
            lo, hi = max(a - tc, 0) / (2 - eta), (a + tc) / eta
            r['admissible_motor_torque_abs_Nm'] = [float(lo), float(hi)]
            r['inside_friction_band'] = bool(lo - 1e-3 <= abs(tm[j]) <= hi + 1e-3) if abs(gact[j]) > 2.0 else None
        rows[n] = r
    big = [r['rel_err_motor_vs_gravity'] for r in rows.values() if 'rel_err_motor_vs_gravity' in r]
    return dict(rows=rows, max_abs_rel_err=float(np.max(np.abs(big))), n_joints_checked=len(big))


def main():
    out = HERE / 'results' / (time.strftime('%Y%m%d_%H%M%S') + '_gate')
    out.mkdir(parents=True)
    res = dict(pose='worst-pose arms, torso -1.0 rad (0.047 rad inside the joint limit), lift 0.3175 m',
               criterion='|tau_m - tau_gravity| / |tau_gravity| <= 2 % on joints with |tau_gravity| > 2 Nm, friction off', cases={})
    for payload in (0., 3.):
        for fr in (False, True):
            r = hold(payload, fr, -1.0)
            res['cases'][f'{payload:g}kg_friction_{"on" if fr else "off"}'] = r
            print(f'{payload:g} kg friction {"on " if fr else "off"}: max |rel err| = {r["max_abs_rel_err"]*100:.2f} % over {r["n_joints_checked"]} joints; torso {r["rows"]["Torso"]["motor_torque_Nm"]:.2f} vs {r["rows"]["Torso"]["gravity_Nm"]:.2f} Nm')
    res['gate_pass_friction_off'] = bool(all(res['cases'][k]['max_abs_rel_err'] <= 0.02 for k in res['cases'] if k.endswith('off')))
    (out / 'gate.json').write_text(json.dumps(res, indent=1))
    print('PASS' if res['gate_pass_friction_off'] else 'FAIL', out)


if __name__ == '__main__':
    main()
