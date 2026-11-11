"""Run every actuator set through the rule scenarios (B, C, D at 0 and 3 kg), reference runs, backdrive and gain checks.
Run: python -I -B mabel_gim/run_sets.py [nproc]  -> results/<timestamp>_sets/{metrics.json, series.npz}"""
import json, sys, time
from multiprocessing import Pool
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import numpy as np


def tasks():
    from rig2 import all_sets
    T = []
    for s in all_sets():
        for key in ('B_T2', 'B_T1', 'C', 'D', 'B_T2_nobrake', 'B_T1_nobrake', 'D_fast'):
            for p in (0., 3.):
                T.append(('scen', s, key, p))
        T.append(('backdrive', s))
        T.append(('gains', s))
    for key in ('B_T2_nobrake', 'B_T1_nobrake', 'C'):
        for p in (0., 3.):
            T.append(('scen', 'stock_gim_nomass', key, p))
    return T


def main():
    from scen2 import worker
    nproc = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    out = HERE / 'results' / (time.strftime('%Y%m%d_%H%M%S') + '_sets_cad')
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
                if s is not None:
                    for k, v in s.items():
                        ser[f'{sn}/{key}_{p:g}kg/{k}'] = v
            else:
                res.setdefault(task[1], {})[task[0]] = m
            if i % 20 == 0:
                print(i, len(T), f'{time.time()-t0:.0f}s', flush=True)
    (out / 'metrics.json').write_text(json.dumps(res, indent=1))
    np.savez_compressed(out / 'series.npz', **ser)
    print('done', out, f'{time.time()-t0:.0f}s')


if __name__ == '__main__':
    main()
