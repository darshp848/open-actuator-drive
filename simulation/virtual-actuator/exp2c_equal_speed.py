"""Equal-speed control for Experiment 2.

E: conventional PI torque loop that tracks the same 30 ms reference as D (equal
response speed) but has no lead and no friction feedforward. It gets its own
25-candidate grid on the calibration set, like S and H, and is scored against the
prescribed reference R on the same held-out cases. This separates the effect of
slowing the commands from the effect of the shaping loop's model-based terms.
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


def eval_E(args):
    plant, sig, seed, amp, sensing, g, fmap = args
    c = command_signal(sig, amp, seed=seed)
    r = run(plant, 'H', Gains(**g), FrictionMap(**fmap), sensing, c, h=e2.H, seed=seed)
    ref = reference_prediction(c, r['n_phys'], e2.H)
    m = error_metrics(r['shaft'] - ref, e2.H)
    th = TrainingSim('reference', plant.mass, 1e-3).rollout(c, len(c) * 0.02)
    return dict(plant=plant.name, signal=sig, amp=amp, sensing=sensing, case='E', ok=bool(r['ok']), rms=m['rms'],
                peak=m['peak'], peak_1ms=m['peak_1ms'], angle_rmse=float(np.sqrt(np.nanmean((r['th_l'] - th) ** 2))),
                saturated_frac=float(np.mean(np.abs(r['i_cmd']) >= NOMINAL.i_max - 1e-9)),
                passed=bool(r['ok'] and m['rms'] <= e2.RMS_LIM and m['peak'] <= e2.PEAK_LIM))


if __name__ == '__main__':
    cfg = json.loads((OUT / 'exp2_config.json').read_text())['cfg']
    fmap = FrictionMap(**cfg['fmap'])
    cal_cases = [(p, s, sd, a) for p in CALIBRATION for (s, sd) in e2.CAL_SIGNALS for a in e2.AMPS]
    log = {}
    with Pool(18) as pool:
        gains = {}
        for sensing in e2.SENSING:
            t0 = time.perf_counter()
            cands = [Gains(kp, ki, 0.0, 0.0) for kp, ki in itertools.product(e2.KP, e2.KI)]
            jobs = [('H', g, fmap, sensing) + cc for g in cands for cc in cal_cases]
            sc = np.array(pool.map(e2.tune_job, jobs)).reshape(len(cands), len(cal_cases)).mean(axis=1)
            best = int(np.argmin(sc)); gains[sensing] = asdict(cands[best])
            log[sensing] = dict(selected=gains[sensing], score=float(sc[best]), wall_s=time.perf_counter() - t0)
            print(sensing, log[sensing], flush=True)
        plants = [NOMINAL] + HELD_OUT
        jobs = [(p, s, sd, a, sens, gains[sens], cfg['fmap']) for p in plants for (s, sd) in e2.EVAL_SIGNALS
                for a in e2.AMPS for sens in e2.SENSING]
        rows = pool.map(eval_E, jobs)
    df = pd.DataFrame(rows)
    df.to_csv(OUT / 'exp2c_rows.csv', index=False)
    held = df[df.plant != 'nominal']
    log['held_out_pass_rate'] = held.groupby('sensing').passed.mean().to_dict()
    log['held_out_rms_median'] = held.groupby('sensing').rms.median().to_dict()
    log['nominal_pass_rate'] = df[df.plant == 'nominal'].groupby('sensing').passed.mean().to_dict()
    (OUT / 'exp2c_equal_speed.json').write_text(json.dumps(log, indent=2))
    print(json.dumps(log, indent=2))
