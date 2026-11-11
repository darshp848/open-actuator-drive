"""Merge the two 4-seed halves (_a: seeds 0-3, _b: seeds 4-7) of an arm_gpu_es.py stage into one 8-seed file."""
import json, sys
from pathlib import Path
OUT = Path(__file__).with_name('results')
kind, tag = sys.argv[1], sys.argv[2]
a, b = (json.loads((OUT / f'arm_es_{kind}_{tag}_{h}.json').read_text()) for h in 'ab')
m = dict(a)
m['theta'] = a['theta'] + b['theta']; m['final'] = a['final'] + b['final']
m['wall_s'] = max(a['wall_s'], b['wall_s']); m['nworld'] = a['nworld'] + b['nworld']
m['env_steps_1ms'] = a['env_steps_1ms'] + b['env_steps_1ms']
m['history'] = [dict(gen=x['gen'], wall_s=max(x['wall_s'], y['wall_s']),
                     **{k: x[k] + y[k] for k in x if k not in ('gen', 'wall_s')}) for x, y in zip(a['history'], b['history'])]
m['config'] = dict(a['config'], seeds=a['config']['seeds'] + b['config']['seeds'], halves=[a['config'], b['config']])
(OUT / f'arm_es_{kind}_{tag}.json').write_text(json.dumps(m))
print(kind, tag, [round(f['train_success'], 2) for f in m['final']])
