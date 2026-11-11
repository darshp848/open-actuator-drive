"""Sensitivity: stock GIM with the per_joint code's modelled reflected inertia instead of the rig's ASSUMED J_rotor.
Run: python -I -B mabel_gim/sens_stock_inertia.py -> results/<ts>_stock_inertia/metrics.json"""
import json, sys, time
from multiprocessing import Pool
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from scen2 import worker, RULE_SCENARIOS

if __name__ == '__main__':
    out = HERE / 'results' / (time.strftime('%Y%m%d_%H%M%S') + '_stock_inertia')
    out.mkdir(parents=True)
    T = [('scen', s, k, p) for s in ('stock_gim_pjinertia',) for k in RULE_SCENARIOS for p in (0., 3.)]
    res = {}
    with Pool(8) as pool:
        for task, m, _ in pool.imap_unordered(worker, T):
            res.setdefault(task[1], {})[f'{task[2]}_{task[3]:g}kg'] = m
    (out / 'metrics.json').write_text(json.dumps(res, indent=1))
    print(out)
