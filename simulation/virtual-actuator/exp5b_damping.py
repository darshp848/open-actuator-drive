"""Experiment 5b: does active gearbox damping (and a disturbance observer) fix the failures?

H2 = shaping loop (reference, lead, friction feedforward) + deflection-rate damping from
     the motor and output encoders, + optional disturbance observer (torque sensor only).
E2 = equal-speed plain PI on the same reference, with the same damping and observer options
     but no lead and no friction feedforward. Keeps the shaping-vs-PI comparison fair.
Each gets the same grid on the calibration set: Kp {0, .25, .5} x Ki {10, 30, 100} x
Kd {0, .15, .3, .6} x DOB {off, 20 Hz} (DOB only with a torque sensor): 72 / 36 candidates.
Held-out evaluation is identical to Experiment 2.
"""
import itertools, json, time
from dataclasses import asdict
from multiprocessing import Pool
from pathlib import Path
import numpy as np
import pandas as pd
import exp2_accuracy as e2
from actuator import NOMINAL, CALIBRATION, HELD_OUT, Gains, FrictionMap, run, command_signal
from models import reference_prediction, error_metrics, TrainingSim

OUT = Path(__file__).with_name('results')
KP, KI, KD = [0.0, 0.25, 0.5], [10.0, 30.0, 100.0], [0.0, 0.15, 0.3, 0.6]


def eval_job(args):
    plant, sig, seed, amp, sensing, case, g, fmap = args
    c = command_signal(sig, amp, seed=seed)
    r = run(plant, 'H', Gains(**g), FrictionMap(**fmap), sensing, c, h=e2.H, seed=seed)
    ref = reference_prediction(c, r['n_phys'], e2.H)
    m = error_metrics(r['shaft'] - ref, e2.H)
    th = TrainingSim('reference', plant.mass, 1e-3).rollout(c, len(c) * 0.02)
    return dict(plant=plant.name, signal=sig, amp=amp, sensing=sensing, case=case, ok=bool(r['ok']), rms=m['rms'],
                peak=m['peak'], peak_1ms=m['peak_1ms'], angle_rmse=float(np.sqrt(np.nanmean((r['th_l'] - th) ** 2))),
                saturated_frac=float(np.mean(np.abs(r['i_cmd']) >= NOMINAL.i_max - 1e-9)),
                passed=bool(r['ok'] and m['rms'] <= e2.RMS_LIM and m['peak'] <= e2.PEAK_LIM))


if __name__ == '__main__':
    cfg = json.loads((OUT / 'exp2_config.json').read_text())['cfg']
    fmap = FrictionMap(**cfg['fmap'])
    cal_cases = [(p, s, sd, a) for p in CALIBRATION for (s, sd) in e2.CAL_SIGNALS for a in e2.AMPS]
    log, gains = {}, {}
    with Pool(18) as pool:
        for sensing in e2.SENSING:
            dobs = [0.0, 20.0] if sensing == 'torque' else [0.0]
            for case, lead, ff in [('H2', e2.LEAD, 1.0), ('E2', 0.0, 0.0)]:
                t0 = time.perf_counter()
                cands = [Gains(kp, ki, lead, ff, kd, dob) for kp, ki, kd, dob in itertools.product(KP, KI, KD, dobs)]
                jobs = [('H', g, fmap, sensing) + cc for g in cands for cc in cal_cases]
                sc = np.array(pool.map(e2.tune_job, jobs)).reshape(len(cands), len(cal_cases)).mean(axis=1)
                best = int(np.argmin(sc)); gains[(sensing, case)] = asdict(cands[best])
                log[f'{sensing}/{case}'] = dict(candidates=len(cands), selected=asdict(cands[best]), score=float(sc[best]),
                                                unstable=int(np.sum(~np.isfinite(sc))), wall_s=time.perf_counter() - t0)
                print(sensing, case, log[f'{sensing}/{case}'], flush=True)
        plants = [NOMINAL] + HELD_OUT
        jobs = [(p, s, sd, a, sens, case, gains[(sens, case)], cfg['fmap']) for p in plants for (s, sd) in e2.EVAL_SIGNALS
                for a in e2.AMPS for sens in e2.SENSING for case in ('H2', 'E2')]
        rows = pool.map(eval_job, jobs)
    df = pd.DataFrame(rows)
    df.to_csv(OUT / 'exp5b_rows.csv', index=False)
    held = df[df.plant != 'nominal']
    log['held_out_pass_rate'] = held.groupby(['sensing', 'case']).passed.mean().unstack().to_dict()
    log['held_out_rms_median'] = held.groupby(['sensing', 'case']).rms.median().unstack().to_dict()
    log['per_plant_pass'] = {f'{s}/{c}': v for (s, c), v in held.pivot_table(index='plant', columns=['sensing', 'case'], values='passed').round(3).to_dict().items()}
    (OUT / 'exp5b_damping.json').write_text(json.dumps(log, indent=2, default=str))
    print(held.pivot_table(index='plant', columns=['sensing', 'case'], values='passed').round(2))
    print(held.groupby(['sensing', 'case'])[['passed', 'rms', 'peak']].agg(['mean', 'median']).round(3))
