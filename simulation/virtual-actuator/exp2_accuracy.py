"""Experiment 2: torque accuracy of each training actuator model on held-out plants.

Parameter selection (calibration plants and calibration signals only):
  1. friction map for feedforward / estimator, from a bench torque sensor on calibration runs
  2. controller gains: equal 25-candidate grids for conventional (S) and shaping (H) feedback
  3. compact model fit (A) to S-controlled calibration runs
Evaluation (held-out plants, separate signals, no retuning). Per run, against the exact
shaft torque at every physics step:
  A  fitted compact model, first-order lag (PACE-style)  vs plant under S
  A2 fitted compact model, second-order lag              vs plant under S
  B  detailed nominal-actuator model vs plant under S (same load, nominal actuator)
  C  prescribed reference           vs plant under S   (negative control)
  D  prescribed reference           vs plant under H   (proposed)
  D0 prescribed reference           vs plant under H without friction feedforward (ablation)
Pilot limits, fixed before running: RMS <= 0.1 Nm and peak <= 0.3 Nm.
Both sensing conditions are run: 'torque' (noisy delayed shaft torque) and 'estimator'
(motor current + encoders). Every controller in a condition gets the same measurements.
"""
import json, time, itertools
from dataclasses import asdict, replace
from multiprocessing import Pool
from pathlib import Path
import numpy as np
from actuator import (NOMINAL, CALIBRATION, HELD_OUT, Gains, FrictionMap, run, command_signal, DT_LOCAL,
                      T_REF, D_NOM_MS)
from models import reference_prediction, Compact, Compact2, fit_compact, fit_compact2, error_metrics, TrainingSim

OUT = Path(__file__).with_name('results'); OUT.mkdir(exist_ok=True)
H = 25e-6                      # from exp1
RMS_LIM, PEAK_LIM = 0.1, 0.3
AMPS = [0.5, 1.0, 1.5]
CAL_SIGNALS = [('multisine', 1), ('random', 1)]
EVAL_SIGNALS = [('reversal', 0), ('steps', 0), ('chirp', 0), ('random', 7)]
SENSING = ['torque', 'estimator']
KP = [0.0, 0.25, 0.5, 1.0, 2.0]
KI = [0.0, 10.0, 30.0, 100.0, 300.0]
LEAD = 0.0015                  # 1 ms sensor latency + 0.5 ms hold, set a priori


def score(err):
    m = error_metrics(err, H)
    return m['rms'] + 0.2 * m['peak'], m


# ---------------------------------------------------------------- calibration jobs
def friction_job(args):
    plant, sig, seed, amp = args
    c = command_signal(sig, amp, seed=seed)
    r = run(plant, 'S', Gains(0.5, 30), FrictionMap(0.1, 0.02), 'torque', c, h=H, seed=seed)
    tau_m = NOMINAL.ratio * NOMINAL.kt * r['i_cmd']            # fast current loop: i ~= i_cmd
    a = np.gradient(r['w_r'], DT_LOCAL)
    resid = tau_m - r['shaft'][:, -1] - NOMINAL.j_rotor_out * a
    return r['w_r'], resid


def tune_job(args):
    kind, gains, fmap, sensing, plant, sig, seed, amp = args
    c = command_signal(sig, amp, seed=seed)
    r = run(plant, kind, gains, fmap, sensing, c, h=H, seed=seed)
    if not r['ok']:
        return np.inf
    if kind == 'S':
        target = np.repeat(r['c_drive'][:, None], r['n_phys'], 1)
    else:
        target = reference_prediction(c, r['n_phys'], H)
    return score(r['shaft'] - target)[0]


def compact_job(args):
    gains, fmap, sensing, plant, sig, seed, amp = args
    c = command_signal(sig, amp, seed=seed)
    r = run(plant, 'S', gains, fmap, sensing, c, h=H, seed=seed)
    return c, dict(shaft=r['shaft'], w_fine=r['w_fine'], n_phys=r['n_phys'], h=H)


# ---------------------------------------------------------------- evaluation job
def eval_job(args):
    plant, sig, seed, amp, sensing, cfg = args
    c = command_signal(sig, amp, seed=seed)
    fmap = FrictionMap(**cfg['fmap'])
    gS, gH = Gains(**cfg['gains'][sensing]['S']), Gains(**cfg['gains'][sensing]['H'])
    cp = Compact(**cfg['compact'][sensing])
    cp2 = Compact2(**cfg['compact2'][sensing])
    out = []
    t0 = time.perf_counter()
    rS = run(plant, 'S', gS, fmap, sensing, c, h=H, seed=seed)
    rH = run(plant, 'H', gH, fmap, sensing, c, h=H, seed=seed)
    rH0 = run(plant, 'H', replace(gH, ff_friction=0.0), fmap, sensing, c, h=H, seed=seed)
    nominal_actuator = replace(NOMINAL, name='nominal_actuator', mass=plant.mass)
    rB = run(nominal_actuator, 'S', gS, fmap, sensing, c, h=H, seed=seed)
    wall = time.perf_counter() - t0
    n_phys = rS['n_phys']
    ref = reference_prediction(c, n_phys, H)
    preds = dict(
        A=(rS, cp.predict(c, rS['w_fine'], n_phys, H), TrainingSim('compact', plant.mass, 1e-3, cp)),
        A2=(rS, cp2.predict(c, rS['w_fine'], n_phys, H), TrainingSim('compact2', plant.mass, 1e-3, cp2)),
        B=(rS, rB['shaft'], None),
        C=(rS, ref, TrainingSim('reference', plant.mass, 1e-3)),
        D=(rH, ref, TrainingSim('reference', plant.mass, 1e-3)),
        D0=(rH0, ref, TrainingSim('reference', plant.mass, 1e-3)),
    )
    th_cache = {}
    for case, (r, pred, sim) in preds.items():
        m = error_metrics(r['shaft'] - pred, H)
        if case == 'B':
            th_model = rB['th_l']
        else:
            key = 'R' if case in ('C', 'D', 'D0') else case
            if key not in th_cache:
                th_cache[key] = sim.rollout(c, len(c) * 0.02)
            th_model = th_cache[key]
        angle = float(np.sqrt(np.nanmean((r['th_l'] - th_model) ** 2)))
        sat = float(np.mean(np.abs(r['i_cmd']) >= NOMINAL.i_max - 1e-9))
        out.append(dict(plant=plant.name, signal=sig, amp=amp, sensing=sensing, case=case, ok=bool(r['ok']),
                        rms=m['rms'], peak=m['peak'], peak_1ms=m['peak_1ms'], angle_rmse=angle,
                        saturated_frac=sat,
                        passed=bool(r['ok'] and m['rms'] <= RMS_LIM and m['peak'] <= PEAK_LIM)))
    return out, wall


def rise_time(gains, fmap, sensing):
    """10-90 % rise of shaft torque for a 1 Nm request step on the nominal plant, from the request time."""
    c = np.zeros(50); c[5:] = 1.0
    out = {}
    for kind in ('S', 'H'):
        r = run(NOMINAL, kind, gains[kind], fmap, sensing, c, h=H, seed=3)
        tau = r['shaft'].ravel(); t = (np.arange(tau.size) + 1) * H - 0.1
        out[kind] = dict(t10_ms=float(t[np.argmax(tau >= 0.1)] * 1e3), t90_ms=float(t[np.argmax(tau >= 0.9)] * 1e3),
                         overshoot_pct=float((tau.max() - 1) * 100))
    return out


if __name__ == '__main__':
    log = {}
    pool = Pool(18)
    cal_cases = [(p, s, sd, a) for p in CALIBRATION for (s, sd) in CAL_SIGNALS for a in AMPS]

    # 1. friction map (bench torque sensor on calibration plants)
    t0 = time.perf_counter()
    res = pool.map(friction_job, cal_cases)
    w = np.concatenate([x[0] for x in res]); y = np.concatenate([x[1] for x in res])
    keep = np.abs(w) > 0.05
    A = np.c_[np.tanh(w[keep] / 0.02), w[keep]]
    coef = np.linalg.lstsq(A, y[keep], rcond=None)[0]
    fmap = FrictionMap(float(coef[0]), float(coef[1]))
    log['friction_map'] = dict(asdict(fmap), wall_s=time.perf_counter() - t0, calibration_seconds=4.0 * len(cal_cases))
    print('friction map', log['friction_map'], flush=True)

    # 2. controller tuning: equal grids
    gains, tuning = {}, {}
    for sensing in SENSING:
        gains[sensing] = {}
        for kind in ('S', 'H'):
            t0 = time.perf_counter()
            cands = [Gains(kp, ki, LEAD if kind == 'H' else 0.0, 1.0 if kind == 'H' else 0.0)
                     for kp, ki in itertools.product(KP, KI)]
            jobs = [(kind, g, fmap, sensing) + cc for g in cands for cc in cal_cases]
            sc = np.array(pool.map(tune_job, jobs)).reshape(len(cands), len(cal_cases))
            mean = sc.mean(axis=1)
            best = int(np.argmin(mean))
            gains[sensing][kind] = asdict(cands[best])
            tuning[f'{sensing}/{kind}'] = dict(candidates=len(cands), runs=len(jobs), wall_s=time.perf_counter() - t0,
                                               selected=asdict(cands[best]), score=float(mean[best]),
                                               unstable_candidates=int(np.sum(~np.isfinite(mean))),
                                               on_grid_edge=bool(cands[best].kp in (KP[0], KP[-1]) or cands[best].ki in (KI[0], KI[-1])))
            print('tuned', sensing, kind, tuning[f'{sensing}/{kind}'], flush=True)
    log['tuning'] = tuning; log['gains'] = gains

    # 3. compact model fit to S-controlled calibration runs
    compact = {}; compact2 = {}; fits = {}
    for sensing in SENSING:
        t0 = time.perf_counter()
        data = pool.map(compact_job, [(Gains(**gains[sensing]['S']), fmap, sensing) + cc for cc in cal_cases])
        t_sim = time.perf_counter() - t0
        t0 = time.perf_counter()
        cp, sse = fit_compact(data)
        t1 = time.perf_counter()
        cp2, sse2 = fit_compact2(data)
        compact[sensing] = asdict(cp); compact2[sensing] = asdict(cp2)
        fits[sensing] = dict(model=asdict(cp), fit_wall_s=t1 - t0, sse=sse,
                             model2=asdict(cp2), fit2_wall_s=time.perf_counter() - t1, sse2=sse2,
                             data_sim_wall_s=t_sim, calibration_seconds=4.0 * len(cal_cases))
        print('compact', sensing, fits[sensing], flush=True)
    log['compact_fit'] = fits

    # response speed check
    log['rise_time'] = {s: rise_time({k: Gains(**gains[s][k]) for k in ('S', 'H')}, fmap, s) for s in SENSING}
    log['reference_rise_ms'] = dict(t10_ms=D_NOM_MS + T_REF * np.log(1 / 0.9) * 1e3,
                                    t90_ms=D_NOM_MS + T_REF * np.log(10) * 1e3)
    print('rise', log['rise_time'], flush=True)

    # 4. evaluation on held-out plants (+ nominal plant with evaluation signals)
    cfg = dict(fmap=asdict(fmap), gains=gains, compact=compact, compact2=compact2)
    plants = [NOMINAL] + HELD_OUT
    jobs = [(p, s, sd, a, sens, cfg) for p in plants for (s, sd) in EVAL_SIGNALS for a in AMPS for sens in SENSING]
    t0 = time.perf_counter()
    res = pool.map(eval_job, jobs)
    rows = [row for out, _ in res for row in out]
    log['eval_wall_s'] = time.perf_counter() - t0
    pool.close()
    (OUT / 'exp2_config.json').write_text(json.dumps(dict(log=log, cfg=cfg, h=H, limits=[RMS_LIM, PEAK_LIM]), indent=2))
    import csv
    with (OUT / 'exp2_rows.csv').open('w', newline='') as f:
        wtr = csv.DictWriter(f, fieldnames=list(rows[0])); wtr.writeheader(); wtr.writerows(rows)
    print('rows', len(rows), 'eval wall', log['eval_wall_s'])
