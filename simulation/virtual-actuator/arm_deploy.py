"""Deploy GPU-trained arm policies on the detailed two-joint plant (arm_plant.py).
Usage: python arm_deploy.py <reference json> <compact json> <tag>
"""
import json, sys, time
from multiprocessing import Pool
from pathlib import Path
import numpy as np
import pandas as pd
import arm_plant as AP
import arm_task as T

OUT = Path(__file__).with_name('results')

if __name__ == '__main__':
    ref, cmp_, tag = sys.argv[1], sys.argv[2], sys.argv[3]
    cfg = json.loads((OUT / 'exp2_config.json').read_text())['cfg']
    sel = json.loads((OUT / 'exp5c_damping_wide.json').read_text())
    thetas = {'reference': json.loads((OUT / ref).read_text()), 'compact': json.loads((OUT / cmp_).read_text())}
    plants = AP.all_plants()            # (set, name, task, p1, p2)
    deployments = [('R+H3', 'reference', 'H', sel['torque/H3']['selected']),
                   ('R+E3', 'reference', 'H', sel['torque/E3']['selected']),
                   ('A2+S', 'compact', 'S', cfg['gains']['torque']['S'])]
    jobs, keys = [], []
    for name, kind, ck, g in deployments:
        for s, th in enumerate(thetas[kind]['theta']):
            th = np.array(th, dtype=float)
            assert th.size == T.P, th.size
            for pset, pn, task, p1, p2 in plants:
                for i in range(len(T.INITS)):
                    jobs.append((pn, task, p1, p2, ck, g, cfg['fmap'], th, i, 100 * s + i))
                    keys.append((name, s, i, pset))
    print(len(jobs), 'episodes', flush=True)
    t0 = time.perf_counter()
    with Pool(18) as pool:
        res = pool.map(AP.deploy, jobs, chunksize=1)
    wall = time.perf_counter() - t0
    df = pd.DataFrame([dict(case=k[0], seed=k[1], init=k[2], plant_set=k[3], **r) for k, r in zip(keys, res)])
    df.to_csv(OUT / f'arm_deploy_{tag}.csv', index=False)
    summary = df.groupby(['case', 'plant_set']).success.mean().unstack().round(3)
    by_seed = df.groupby(['case', 'seed']).success.mean().unstack().round(3)
    print(summary); print(by_seed)
    print('not ok:', int((~df.ok).sum()))
    (OUT / f'arm_deploy_{tag}.json').write_text(json.dumps(dict(
        n_episodes=len(df), n_not_ok=int((~df.ok).sum()),
        success_by_case_and_set={c: r.to_dict() for c, r in summary.iterrows()},
        success_by_case_and_seed={c: {str(k): v for k, v in r.to_dict().items()} for c, r in by_seed.iterrows()},
        deploy_wall_s=wall), indent=2))
