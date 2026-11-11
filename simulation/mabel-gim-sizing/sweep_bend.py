"""Where does one GIM8115-36 at 48 V run out? Torso bend 0 -> -1.0472 -> 0 at shorter times T (hold 1 s between).
Reports the analytic demand/capability along the reference (full inverse dynamics + Coulomb friction vs the
voltage-limited envelope) and the simulated saturation with full inverse-dynamics feedforward.
Run: python -I -B mabel_gim/sweep_bend.py   -> results/<timestamp>_sweep/sweep.json"""
import json, sys, time
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import numpy as np
from scen import *


def main():
    out = HERE / 'results' / (time.strftime('%Y%m%d_%H%M%S') + '_sweep')
    out.mkdir(parents=True)
    res = {}
    for payload in (0., 3.):
        for T in (1.0, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3):
            tr = traj_bend(T)
            cache = {}
            rig, L, ff = run(tr, payload, 'gim_full', cache)
            ratio, req = feasibility(rig, tr, cache[(payload, 'full')])
            a = joint_metrics(rig, L, 0, tr.windows['active'])
            fw, rt = joint_metrics(rig, L, 0, tr.windows['fwd']), joint_metrics(rig, L, 0, tr.windows['ret'])
            r = dict(peak_ref_speed_rad_s=float(np.abs(tr.v[:, 0]).max()), peak_required_torque_Nm=float(np.abs(req).max()),
                     analytic_peak_demand_over_capability=float(ratio.max()), sim_peak_err_mrad=a['peak_abs_err_rad'] * 1e3,
                     sim_rms_err_mrad=a['rms_err_rad'] * 1e3, sim_peak_iq_over_imax=a['peak_iq_over_imax'],
                     sim_frac_voltage_limit=a['frac_voltage_limit'], sim_frac_current_limit=a['frac_current_limit'],
                     sim_peak_speed_rad_s=a['peak_speed_rad_s'], return_stroke_peak_motoring_W=rt['bus']['peak_motoring_W'],
                     forward_stroke_peak_regen_W=fw['bus']['peak_regen_W'])
            res[f'{payload:g}kg_T{T:g}'] = r
            print(f'{payload:g} kg T={T:4.2f}  v={r["peak_ref_speed_rad_s"]:.2f}  req {r["peak_required_torque_Nm"]:5.0f} Nm  demand/cap {r["analytic_peak_demand_over_capability"]:.2f}  '
                  f'peak err {r["sim_peak_err_mrad"]:6.1f} mrad  iq/imax {r["sim_peak_iq_over_imax"]:.2f}  fV {r["sim_frac_voltage_limit"]:.3f} fI {r["sim_frac_current_limit"]:.3f}', flush=True)
    (out / 'sweep.json').write_text(json.dumps(res, indent=1))
    print(out)


if __name__ == '__main__':
    main()
