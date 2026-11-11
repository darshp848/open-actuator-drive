"""Nominal stock: geometric-estimate reflected inertia (GIM8115 0.25, GIM8108 0.14 kg m2).
Run: python -I -B mabel_gim/sens_stock_nominal.py -> results/<ts>_stock_inertia/metrics.json"""
import json, sys, time
from multiprocessing import Pool
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from scen2 import worker, RULE_SCENARIOS

if __name__ == '__main__':
    out = HERE / 'results' / (time.strftime('%Y%m%d_%H%M%S') + '_stock_nominal')
    out.mkdir(parents=True)
    T = [('scen', s, k, p) for s in ('stock_gim_nominal',) for k in RULE_SCENARIOS for p in (0., 3.)]
    res = {}
    with Pool(8) as pool:
        for task, m, _ in pool.imap_unordered(worker, T):
            res.setdefault(task[1], {})[f'{task[2]}_{task[3]:g}kg'] = m
    (out / 'metrics.json').write_text(json.dumps(res, indent=1))
    print(out)
