"""Experiment 6: confirm the envelope from Follow-up 1 on newly drawn plants.

Fixed before running (written 21 Oct 2026, before any result from this script):
- 40 new plants drawn with seed 2026 from ranges spanning the held-out set; none of
  them was used for tuning, fitting or envelope selection.
- Controllers frozen from exp5c: H3 (damped shaping loop) and E3 (damped plain PI).
- Signals: reversal, steps, chirp, and random levels with new seed 11.
- Primary claim: H3 with a torque sensor, 60 ms reference, 0.5 Nm commands passes
  >= 90 % of cases (RMS <= 0.1 Nm, peak <= 0.3 Nm). Also reported, not gated:
  1.0 Nm, the 30 ms reference, the no-torque-sensor condition, and E3.
- Uncertainty: plants resampled as clusters (bootstrap, 5000 draws).
"""
import json, os, subprocess, sys
from dataclasses import replace, asdict
from multiprocessing import Pool
from pathlib import Path
import numpy as np
import pandas as pd

OUT = Path(__file__).with_name('results')
TS = [0.030, 0.060]
AMPS = [0.5, 1.0]
SIGNALS = [('reversal', 0), ('steps', 0), ('chirp', 0), ('random', 11)]


def draw_plants(n=40, seed=2026):
    from actuator import NOMINAL
    rng = np.random.default_rng(seed)
    plants = []
    for i in range(n):
        gap = float(rng.uniform(0.001, 0.005)) if rng.random() < 0.3 else 0.0
        plants.append(replace(NOMINAL, name=f'fresh_{i:02d}',
                              coulomb=float(rng.uniform(0.06, 0.22)), stribeck=float(rng.uniform(0.02, 0.12)),
                              v_stribeck=float(rng.uniform(0.04, 0.15)), viscous=float(rng.uniform(0.01, 0.045)),
                              dry=float(rng.uniform(0.0, 0.06)), stiffness=float(np.exp(rng.uniform(np.log(100), np.log(400)))),
                              damping=float(rng.uniform(0.1, 0.3)), gap=gap, mass=float(rng.uniform(0.6, 1.5)),
                              kt=float(rng.uniform(0.07, 0.085)), cogging=float(rng.uniform(0.0, 0.03)),
                              delay_ms=int(rng.integers(2, 9)), jitter_ms=int(rng.choice([0, 0, 3, 6]))))
    return plants


def job(args):
    from actuator import NOMINAL, Gains, FrictionMap, run, command_signal
    from models import reference_prediction, error_metrics
    plant, sig, seed, amp, sensing, case, g, fmap = args
    c = command_signal(sig, amp, seed=seed)
    r = run(plant, 'H', Gains(**g), FrictionMap(**fmap), sensing, c, h=25e-6, seed=seed)
    m = error_metrics(r['shaft'] - reference_prediction(c, r['n_phys'], 25e-6), 25e-6)
    return dict(plant=plant.name, signal=sig, amp=amp, sensing=sensing, case=case, ok=bool(r['ok']), rms=m['rms'],
                peak=m['peak'], passed=bool(r['ok'] and m['rms'] <= 0.1 and m['peak'] <= 0.3),
                coulomb_total=plant.coulomb + plant.dry, gap=plant.gap, stiffness=plant.stiffness)


def one_T():
    from actuator import T_REF
    cfg = json.loads((OUT / 'exp2_config.json').read_text())['cfg']
    sel = json.loads((OUT / 'exp5c_damping_wide.json').read_text())
    plants = draw_plants()
    jobs = [(p, s, sd, a, sens, case, sel[f'{sens}/{case}']['selected'], cfg['fmap'])
            for p in plants for (s, sd) in SIGNALS for a in AMPS for sens in ('torque', 'estimator') for case in ('H3', 'E3')]
    with Pool(18) as pool:
        df = pd.DataFrame(pool.map(job, jobs))
    df['T_ms'] = round(T_REF * 1e3)
    df.to_csv(OUT / f'exp6_T{round(T_REF * 1e3)}.csv', index=False)


def cluster_ci(df, n=5000, seed=0):
    rng = np.random.default_rng(seed)
    groups = [g.passed.to_numpy() for _, g in df.groupby('plant')]
    stats = [np.mean(np.concatenate([groups[i] for i in rng.integers(0, len(groups), len(groups))])) for _ in range(n)]
    return float(df.passed.mean()), float(np.percentile(stats, 2.5)), float(np.percentile(stats, 97.5))


if __name__ == '__main__':
    if 'T_REF_S' in os.environ:
        one_T()
    else:
        for T in TS if not os.environ.get('SUMMARY_ONLY') else []:
            subprocess.run([sys.executable, __file__], env={**os.environ, 'T_REF_S': str(T)}, check=True)
        df = pd.concat([pd.read_csv(OUT / f'exp6_T{round(T * 1e3)}.csv') for T in TS])
        cells = {}
        for (sens, case, T, amp), g in df.groupby(['sensing', 'case', 'T_ms', 'amp']):
            m, lo, hi = cluster_ci(g)
            cells[f'{sens}/{case}/T{T}/{amp}Nm'] = dict(pass_rate=m, ci95=[lo, hi], n=len(g))
        primary = cells['torque/H3/T60/0.5Nm']
        hi_fric = df[(df.coulomb_total > 0.18)]
        res = dict(primary=primary, primary_passes_gate=primary['pass_rate'] >= 0.9, cells=cells,
                   high_friction_plants_pass={'/'.join(map(str, k)): v for k, v in hi_fric.groupby(
                       ['sensing', 'case', 'T_ms', 'amp']).passed.mean().to_dict().items()},
                   backlash_plants_pass={'/'.join(map(str, k)): v for k, v in df[df.gap > 0].groupby(
                       ['sensing', 'case', 'T_ms', 'amp']).passed.mean().to_dict().items()},
                   plants=[asdict(p) for p in draw_plants()])
        (OUT / 'exp6_confirm.json').write_text(json.dumps(res, indent=2))
        print(json.dumps({k: v for k, v in res.items() if k != 'plants'}, indent=1))
