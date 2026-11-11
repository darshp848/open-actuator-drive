"""Experiment 5d: usable envelope of the damped shaping loop (H3).

Sweeps the reference time constant T (faster and slower than 30 ms) and the command
amplitude (0.5 to 2.0 Nm) on the held-out plants, with the H3 gains selected in
exp5c at T = 30 ms (not retuned per T). Reports the pass rate for every (T, amplitude)
cell and the 10-90 % rise time each T implies, so any accuracy gain from a slower
reference is shown with its loss of response speed.

Run without arguments: it relaunches itself once per T with T_REF_S set, because the
reference constant is read at import time in every worker.
"""
import json, os, subprocess, sys
from multiprocessing import Pool
from pathlib import Path
import numpy as np
import pandas as pd

OUT = Path(__file__).with_name('results')
TS = [0.015, 0.030, 0.060]
AMPS = [0.5, 1.0, 1.5, 2.0]


def job(args):
    from actuator import NOMINAL, Gains, FrictionMap, run, command_signal
    from models import reference_prediction, error_metrics
    plant, sig, seed, amp, sensing, g, fmap = args
    c = command_signal(sig, amp, seed=seed)
    r = run(plant, 'H', Gains(**g), FrictionMap(**fmap), sensing, c, h=25e-6, seed=seed)
    m = error_metrics(r['shaft'] - reference_prediction(c, r['n_phys'], 25e-6), 25e-6)
    return dict(plant=plant.name, signal=sig, amp=amp, sensing=sensing, ok=bool(r['ok']), rms=m['rms'], peak=m['peak'],
                saturated_frac=float(np.mean(np.abs(r['i_cmd']) >= NOMINAL.i_max - 1e-9)),
                passed=bool(r['ok'] and m['rms'] <= 0.1 and m['peak'] <= 0.3))


def one_T():
    from actuator import HELD_OUT, T_REF, D_NOM_MS
    cfg = json.loads((OUT / 'exp2_config.json').read_text())['cfg']
    sel = json.loads((OUT / 'exp5c_damping_wide.json').read_text())
    gains = {s: sel[f'{s}/H3']['selected'] for s in ('torque', 'estimator')}
    signals = [('reversal', 0), ('steps', 0), ('chirp', 0), ('random', 7)]
    jobs = [(p, s, sd, a, sens, gains[sens], cfg['fmap']) for p in HELD_OUT for (s, sd) in signals for a in AMPS
            for sens in ('torque', 'estimator')]
    with Pool(18) as pool:
        df = pd.DataFrame(pool.map(job, jobs))
    df['T_ms'] = round(T_REF * 1e3)
    df['t90_ms'] = D_NOM_MS + T_REF * np.log(10) * 1e3
    df.to_csv(OUT / f'exp5d_T{round(T_REF * 1e3)}.csv', index=False)


if __name__ == '__main__':
    if 'T_REF_S' in os.environ:
        one_T()
    else:
        for T in TS:
            subprocess.run([sys.executable, __file__], env={**os.environ, 'T_REF_S': str(T)}, check=True)
        df = pd.concat([pd.read_csv(OUT / f'exp5d_T{round(T * 1e3)}.csv') for T in TS])
        tab = df.pivot_table(index=['sensing', 'T_ms'], columns='amp', values='passed').round(3)
        per_plant = df.pivot_table(index=['plant'], columns=['sensing', 'T_ms'], values='passed').round(2)
        print(tab); print(per_plant)
        (OUT / 'exp5d_envelope.json').write_text(json.dumps(dict(
            pass_by_T_amp={f'{s}/T{t}': {str(a): v for a, v in row.items()} for (s, t), row in tab.iterrows()},
            t90_ms={str(round(T * 1e3)): 4 + T * np.log(10) * 1e3 for T in TS},
            per_plant={f'{s}/T{t}': col.to_dict() for (s, t), col in per_plant.items()}), indent=2))
