"""Deploy GPU-trained policies (gpu_es.py output) on the detailed virtual actuator.

Same plants, initial states, success test and deployment loops as rl_pilot.py.
Usage: python rl_deploy.py <reference json> <compact2 json> <tag>
"""
import json, sys, time
from multiprocessing import Pool
from pathlib import Path
import numpy as np
import pandas as pd
import rl_pilot as R
from actuator import NOMINAL, HELD_OUT

OUT = Path(__file__).with_name('results')

if __name__ == '__main__':
    from exp6_confirm import draw_plants
    ref, cp2, tag = sys.argv[1], sys.argv[2], sys.argv[3]
    cfg = json.loads((OUT / 'exp2_config.json').read_text())['cfg']
    sel = json.loads((OUT / 'exp5c_damping_wide.json').read_text())
    thetas = {'reference': json.loads((OUT / ref).read_text()), 'compact2': json.loads((OUT / cp2).read_text())}
    from dataclasses import replace
    # plants outside the training randomisation range (load 0.6-1.5 kg, delay 2-8 ms)
    ood = [replace(NOMINAL, name='ood_light_0.4kg', mass=0.4), replace(NOMINAL, name='ood_heavy_1.8kg', mass=1.8),
           replace(NOMINAL, name='ood_delay_11ms', delay_ms=11), replace(NOMINAL, name='ood_jitter_4to12ms', delay_ms=4, jitter_ms=8),
           replace(NOMINAL, name='ood_heavy_slow', mass=1.8, delay_ms=10),
           replace(NOMINAL, name='ood_heavy_friction', mass=1.8, coulomb=0.2, stribeck=0.1, viscous=0.04)]
    confirm = len(sys.argv) > 4 and sys.argv[4] == 'confirm'
    if confirm:
        # extended out-of-range set for the confirmation run (fixed before running)
        ood += [replace(NOMINAL, name='ood_light_0.3kg', mass=0.3), replace(NOMINAL, name='ood_heavy_1.7kg', mass=1.7),
                replace(NOMINAL, name='ood_heavy_2.0kg', mass=2.0), replace(NOMINAL, name='ood_delay_12ms', delay_ms=12),
                replace(NOMINAL, name='ood_light_slow', mass=0.4, delay_ms=10),
                replace(NOMINAL, name='ood_heavy_soft', mass=1.8, stiffness=120.0, damping=0.12)]
    plants = [NOMINAL] + HELD_OUT + draw_plants()[:20] + ood
    deployments = [('R+H3', 'reference', 'H', sel['torque/H3']['selected']),
                   ('R+E3', 'reference', 'H', sel['torque/E3']['selected']),
                   ('A2+S', 'compact2', 'S', cfg['gains']['torque']['S']),
                   ('R+S', 'reference', 'S', cfg['gains']['torque']['S'])]
    if confirm:
        deployments = deployments[:3]
    jobs, keys = [], []
    for name, kind, ck, g in deployments:
        for s, th in enumerate(thetas[kind]['theta']):
            for p in plants:
                for i, q0 in enumerate(R.INITS):
                    jobs.append((p, ck, g, cfg['fmap'], np.array(th), q0, 100 * s + i))
                    keys.append((name, s, p.name, q0))
    t0 = time.perf_counter()
    with Pool(18) as pool:
        res = pool.map(R.deploy, jobs)
    df = pd.DataFrame([dict(case=k[0], seed=k[1], init=k[3], **r) for k, r in zip(keys, res)])
    df['plant_set'] = np.select([df.plant == 'nominal', df.plant.str.startswith('fresh'), df.plant.str.startswith('ood')],
                                ['nominal', 'fresh', 'out_of_range'], 'held_out')
    df.to_csv(OUT / f'rl_deploy_{tag}.csv', index=False)
    summary = df.groupby(['case', 'plant_set']).success.mean().unstack().round(3)
    by_seed = df.groupby(['case', 'seed']).success.mean().unstack().round(3)
    print(summary); print(by_seed)
    (OUT / f'rl_deploy_{tag}.json').write_text(json.dumps(dict(
        training_final={k: v['final'] for k, v in thetas.items()},
        training_wall_s={k: v['wall_s'] for k, v in thetas.items()},
        success_by_case_and_set={c: r.to_dict() for c, r in summary.iterrows()},
        success_by_case_and_seed={c: {str(k): v for k, v in r.to_dict().items()} for c, r in by_seed.iterrows()},
        deploy_wall_s=time.perf_counter() - t0), indent=2))
