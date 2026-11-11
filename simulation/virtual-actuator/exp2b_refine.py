"""Numerical check of the final Experiment 2 cases with the tuned controllers.

Reruns a set of evaluation cases at 12.5 us (half the 25 us step) with every
controller decision recomputed (complete closed loop, not a command replay) and
compares the per-case metrics. Selection, fixed before running: per sensing
condition, the 8 cases with the largest D RMS error, the 8 with the largest A2 RMS
error, and 8 further cases drawn with seed 0. Gate: |dRMS| <= 0.01 Nm, |dPeak| <= 0.03 Nm.
"""
import json
from multiprocessing import Pool
from pathlib import Path
import numpy as np
import pandas as pd
import exp2_accuracy as e2
from actuator import NOMINAL, HELD_OUT

OUT = Path(__file__).with_name('results')
H_FINE = 12.5e-6


def set_fine():
    e2.H = H_FINE


if __name__ == '__main__':
    rows = pd.read_csv(OUT / 'exp2_rows.csv')
    cfg = json.loads((OUT / 'exp2_config.json').read_text())['cfg']
    keys = ['plant', 'signal', 'amp', 'sensing']
    chosen = []
    rng = np.random.default_rng(0)
    for sens in ['torque', 'estimator']:
        r = rows[rows.sensing == sens]
        pick = pd.concat([r[r.case == 'D'].nlargest(8, 'rms')[keys], r[r.case == 'A2'].nlargest(8, 'rms')[keys]])
        rest = r[r.case == 'D'][keys].drop_duplicates()
        rest = rest.iloc[rng.choice(len(rest), 8, replace=False)]
        chosen.append(pd.concat([pick, rest]).drop_duplicates())
    chosen = pd.concat(chosen)
    plants = {p.name: p for p in [NOMINAL] + HELD_OUT}
    seeds = dict(reversal=0, steps=0, chirp=0, random=7)
    jobs = [(plants[k.plant], k.signal, seeds[k.signal], k.amp, k.sensing, cfg) for k in chosen.itertuples()]
    with Pool(18, initializer=set_fine) as pool:
        res = pool.map(e2.eval_job, jobs)
    fine = pd.DataFrame([row for out, _ in res for row in out])
    m = fine.merge(rows, on=keys + ['case'], suffixes=('_fine', ''))
    m['d_rms'] = (m.rms_fine - m.rms).abs()
    m['d_peak'] = (m.peak_fine - m.peak).abs()
    m['pass_changed'] = m.passed_fine != m.passed
    m['within_gate'] = (m.d_rms <= 0.01) & (m.d_peak <= 0.03)
    summary = m.groupby('case').agg(cases=('d_rms', 'size'), within_gate=('within_gate', 'mean'),
                                    worst_d_rms=('d_rms', 'max'), worst_d_peak=('d_peak', 'max'),
                                    verdict_changes=('pass_changed', 'sum'))
    print(summary)
    m.to_csv(OUT / 'exp2b_refinement_rows.csv', index=False)
    (OUT / 'exp2b_refinement.json').write_text(json.dumps(dict(
        h_coarse_us=25, h_fine_us=12.5, gate=dict(d_rms=0.01, d_peak=0.03),
        summary=json.loads(summary.to_json(orient='index')),
        failures=m[~m.within_gate][keys + ['case', 'rms', 'rms_fine', 'peak', 'peak_fine']].to_dict('records')), indent=2))
