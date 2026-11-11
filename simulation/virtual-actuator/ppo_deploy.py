"""Deploy rsl_rl PPO policies (exported by rslrl_train.py) on the detailed virtual actuator.

Reuses rl_pilot.deploy and the Follow-up 7 plant sets; only the policy function changes to the
exported ELU network with a tanh-squashed output. Run with the main .venv:
  python ppo_deploy.py <reference tag> <compact tag> <seeds> <out tag>
"""
import json, math, sys, time
from multiprocessing import Pool
from pathlib import Path
from dataclasses import replace
import numpy as np
import pandas as pd
import rl_pilot as R
from actuator import NOMINAL, HELD_OUT

OUT = Path(__file__).with_name('results')


def act_ppo(theta, obs):
    x = np.asarray(obs, dtype=float)
    n = len(theta['W'])
    for k, (W, b) in enumerate(zip(theta['W'], theta['b'])):
        x = x @ W + b
        if k < n - 1:
            x = np.where(x > 0, x, np.expm1(x))       # ELU
    return R.A_MAX * math.tanh(float(x[0]))


R.act = act_ppo        # rl_pilot.deploy calls act(theta, obs); patched in every worker on import


def deploy(args):
    return R.deploy(args)


if __name__ == '__main__':
    from exp6_confirm import draw_plants
    ref_tag, cp_tag, n_seeds, tag = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
    cfg = json.loads((OUT / 'exp2_config.json').read_text())['cfg']
    sel = json.loads((OUT / 'exp5c_damping_wide.json').read_text())
    load = lambda t, kind, s: json.loads((OUT / f'rslrl_{t}_{kind}_s{s}.json').read_text())
    pol = {k: [] for k in ('reference', 'compact2')}
    train = {}
    for s in range(n_seeds):
        for kind, t in (('reference', ref_tag), ('compact2', cp_tag)):
            d = load(t, kind, s)
            pol[kind].append({'W': [np.array(w) for w in d['actor']['W']], 'b': [np.array(b) for b in d['actor']['b']]})
            train[f'{kind}/{s}'] = dict(final_det_success_nominal=d['final_det_success_nominal'],
                                        final_det_success_random=d['final_det_success_random'], wall_s=d['wall_s'])
    ood = [replace(NOMINAL, name='ood_light_0.4kg', mass=0.4), replace(NOMINAL, name='ood_heavy_1.8kg', mass=1.8),
           replace(NOMINAL, name='ood_delay_11ms', delay_ms=11), replace(NOMINAL, name='ood_jitter_4to12ms', delay_ms=4, jitter_ms=8),
           replace(NOMINAL, name='ood_heavy_slow', mass=1.8, delay_ms=10),
           replace(NOMINAL, name='ood_heavy_friction', mass=1.8, coulomb=0.2, stribeck=0.1, viscous=0.04),
           replace(NOMINAL, name='ood_light_0.3kg', mass=0.3), replace(NOMINAL, name='ood_heavy_1.7kg', mass=1.7),
           replace(NOMINAL, name='ood_heavy_2.0kg', mass=2.0), replace(NOMINAL, name='ood_delay_12ms', delay_ms=12),
           replace(NOMINAL, name='ood_light_slow', mass=0.4, delay_ms=10),
           replace(NOMINAL, name='ood_heavy_soft', mass=1.8, stiffness=120.0, damping=0.12)]
    plants = [NOMINAL] + HELD_OUT + draw_plants()[:20] + ood
    deployments = [('R+H3', 'reference', 'H', sel['torque/H3']['selected']),
                   ('R+E3', 'reference', 'H', sel['torque/E3']['selected']),
                   ('A2+S', 'compact2', 'S', cfg['gains']['torque']['S'])]
    jobs, keys = [], []
    for name, kind, ck, g in deployments:
        for s, th in enumerate(pol[kind]):
            for p in plants:
                for i, q0 in enumerate(R.INITS):
                    jobs.append((p, ck, g, cfg['fmap'], th, q0, 100 * s + i))
                    keys.append((name, s, p.name, q0))
    t0 = time.perf_counter()
    with Pool(18) as pool:
        res = pool.map(deploy, jobs)
    df = pd.DataFrame([dict(case=k[0], seed=k[1], init=k[3], **r) for k, r in zip(keys, res)])
    df['plant_set'] = np.select([df.plant == 'nominal', df.plant.str.startswith('fresh'), df.plant.str.startswith('ood')],
                                ['nominal', 'fresh', 'out_of_range'], 'held_out')
    df.to_csv(OUT / f'ppo_deploy_{tag}.csv', index=False)
    summary = df.groupby(['case', 'plant_set']).success.mean().unstack().round(3)
    print(summary); print(df.groupby(['case', 'seed']).success.mean().unstack().round(3))
    (OUT / f'ppo_deploy_{tag}.json').write_text(json.dumps(dict(training=train, deploy_wall_s=time.perf_counter() - t0,
        success_by_case_and_set={c: r.to_dict() for c, r in summary.iterrows()}), indent=2))
