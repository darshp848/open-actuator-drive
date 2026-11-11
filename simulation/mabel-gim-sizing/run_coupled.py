"""Run the coupled sets (designs_coupled/*.json) plus the three stock inertia values through the pre-registered scenarios.
python -I -B mabel_gim/run_coupled.py [designs_dir] [nproc] -> results/<ts>_coupled/{metrics.json, series.npz}"""
import json, os, sys, time
from multiprocessing import Pool
from pathlib import Path
HERE = Path(__file__).resolve().parent
D = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / 'designs_coupled'
os.environ['MABEL_DESIGNS'] = str(D)
sys.path.insert(0, str(HERE))
import numpy as np
STOCKS = ['stock_gim_nominal', 'stock_gim', 'stock_gim_pjinertia']


def tasks():
    names = json.loads((D / 'sets.json').read_text())
    T = []
    for s in STOCKS + names:
        full = s == 'stock_gim_nominal' or s in names
        keys = ('B_T2', 'B_T1', 'C', 'D') + (('B_T2_nobrake', 'B_T1_nobrake', 'D_fast') if full else ())
        for key in keys:
            for p in (0., 3.):
                T.append(('scen', s, key, p))
        if full:
            T += [('backdrive', s), ('gains', s)]
    return T


def main():
    from scen2 import worker
    nproc = int(sys.argv[2]) if len(sys.argv) > 2 else 12
    out = HERE / 'results' / (time.strftime('%Y%m%d_%H%M%S') + '_coupled')
    out.mkdir(parents=True)
    T = tasks()
    print(len(T), 'tasks ->', out, flush=True)
    res, ser = {}, {}
    t0 = time.time()
    with Pool(nproc) as pool:
        for i, (task, m, s) in enumerate(pool.imap_unordered(worker, T)):
            if task[0] == 'scen':
                _, sn, key, p = task
                res.setdefault(sn, {})[f'{key}_{p:g}kg'] = m
                for k, v in (s or {}).items():
                    ser[f'{sn}/{key}_{p:g}kg/{k}'] = v
            else:
                res.setdefault(task[1], {})[task[0]] = m
            if i % 20 == 0:
                print(i, len(T), f'{time.time()-t0:.0f}s', flush=True)
    res['_designs_dir'] = str(D)
    (out / 'metrics.json').write_text(json.dumps(res, indent=1))
    np.savez_compressed(out / 'series.npz', **ser)
    print('done', out, f'{time.time()-t0:.0f}s', flush=True)


if __name__ == '__main__':
    main()
