"""Speed of the detailed plant: current CPU path (one run per process, 18 processes)
versus the mjbatch batched plant (one process, C++ thread pool). Same plant, controller,
25 us step and 4 s command sequences. Run with simulation/.venv-mjbatch."""
import json, sys, time
from multiprocessing import Pool
from pathlib import Path
import numpy as np
import actuator as A
from batched_plant import BatchedRun

OUT = Path(__file__).with_name('results')
cfg = json.loads((OUT / 'exp2_config.json').read_text())['cfg']
sel = json.loads((OUT / 'exp5c_damping_wide.json').read_text())
G = A.Gains(**sel['torque/H3']['selected']); FM = A.FrictionMap(**cfg['fmap'])


def one(seed):
    c = A.command_signal('random', 1.0, seed=seed)
    return A.run(A.NOMINAL, 'H', G, FM, 'torque', c, h=25e-6, seed=seed)['shaft'][-1, -1]


if __name__ == '__main__':
    res = {}
    n_cpu = 144
    t0 = time.perf_counter()
    with Pool(18) as pool:
        pool.map(one, range(n_cpu))
    res['cpu_pool18'] = dict(runs=n_cpu, wall_s=time.perf_counter() - t0)
    res['cpu_pool18']['runs_per_s'] = n_cpu / res['cpu_pool18']['wall_s']
    print(res, flush=True)
    for n in [144, 576, 2304]:
        cs = np.stack([A.drive_commands(A.command_signal('random', 1.0, seed=s), A.NOMINAL, s) for s in range(n)])
        br = BatchedRun(A.NOMINAL, n)
        t0 = time.perf_counter(); br.run('H', G, FM, cs, list(range(n))); wall = time.perf_counter() - t0
        res[f'mjbatch_{n}'] = dict(runs=n, wall_s=wall, runs_per_s=n / wall)
        print(n, res[f'mjbatch_{n}'], flush=True)
    for k in res:
        res[k]['speedup_vs_cpu'] = res[k]['runs_per_s'] / res['cpu_pool18']['runs_per_s']
    (OUT / 'bench_mjbatch.json').write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=1))
