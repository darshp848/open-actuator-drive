"""Experiment 1: closed-loop numerical accuracy versus physics step.

Controller (1 kHz), driver (20 kHz) and command (50 Hz) clocks stay fixed. Complete
closed-loop runs are repeated at physics steps 1000 us down to 12.5 us and
compared with the 12.5 us run. Gate (set before running, 10 % of the pilot limits):
  |change in RMS error vs reference| <= 0.01 Nm, |change in peak error| <= 0.03 Nm,
  RMS difference of the 1 ms shaft-torque trace <= 0.01 Nm.
Preliminary controller gains are used here; tuned gains are rechecked in exp2.
"""
import json, sys, time
from multiprocessing import Pool
from pathlib import Path
import numpy as np
from actuator import HELD_OUT, NOMINAL, Gains, FrictionMap, run, command_signal
from models import reference_prediction, error_metrics

OUT = Path(__file__).with_name('results'); OUT.mkdir(exist_ok=True)
STEPS = [1000e-6, 500e-6, 250e-6, 100e-6, 50e-6, 25e-6, 12.5e-6]
FMAP = FrictionMap(0.10, 0.02)
GAINS = {'S': Gains(0.5, 30), 'H': Gains(0.5, 30, 0.0015, 1.0)}
PLANTS = [NOMINAL] + [p for p in HELD_OUT if p.name in ('sticky_lowspeed', 'soft_gearbox', 'backlash', 'combined')]
SIGNALS = ['reversal', 'steps', 'chirp']
GATE = dict(d_rms=0.01, d_peak=0.03, trace=0.01)


def job(args):
    plant, kind, sig, h = args
    c = command_signal(sig, 1.5)
    t0 = time.perf_counter()
    r = run(plant, kind, GAINS[kind], FMAP, 'torque', c, h=h, seed=1)
    wall = time.perf_counter() - t0
    ref = reference_prediction(c, r['n_phys'], h)
    m = error_metrics(r['shaft'] - ref, h)
    return dict(plant=plant.name, ctl=kind, signal=sig, h_us=round(h * 1e6), ok=r['ok'], wall_s=wall,
                rms=m['rms'], peak=m['peak'], tau_1ms=r['shaft'][:, -1].tolist(), th_1ms=r['th_l'].tolist())


if __name__ == '__main__':
    jobs = [(p, k, s, h) for p in PLANTS for k in GAINS for s in SIGNALS for h in STEPS]
    with Pool(16) as pool:
        res = pool.map(job, jobs)
    ref = {(r['plant'], r['ctl'], r['signal']): r for r in res if r["h_us"] == 12}
    rows = []
    for r in res:
        b = ref[(r['plant'], r['ctl'], r['signal'])]
        tr = np.sqrt(np.nanmean((np.array(r['tau_1ms']) - np.array(b['tau_1ms'])) ** 2))
        th = np.sqrt(np.nanmean((np.array(r['th_1ms']) - np.array(b['th_1ms'])) ** 2))
        row = {k: r[k] for k in ('plant', 'ctl', 'signal', 'h_us', 'ok', 'wall_s', 'rms', 'peak')}
        row.update(d_rms=abs(r['rms'] - b['rms']), d_peak=abs(r['peak'] - b['peak']), trace_rms_diff=float(tr),
                   angle_rms_diff=float(th))
        row['pass'] = bool(r['ok'] and row['d_rms'] <= GATE['d_rms'] and row['d_peak'] <= GATE['d_peak']
                           and tr <= GATE['trace'])
        rows.append(row)
    summary = {}
    for h in STEPS:
        hu = round(h * 1e6)
        sel = [x for x in rows if x['h_us'] == hu]
        summary[hu] = dict(passed=sum(x['pass'] for x in sel), cases=len(sel),
                           worst_d_rms=max(x['d_rms'] for x in sel), worst_d_peak=max(x['d_peak'] for x in sel),
                           worst_trace=max(x['trace_rms_diff'] for x in sel),
                           worst_angle=max(x['angle_rms_diff'] for x in sel),
                           failures=[f"{x['plant']}/{x['ctl']}/{x['signal']}" for x in sel if not x['pass']],
                           mean_wall_s=float(np.mean([x['wall_s'] for x in sel])))
    # 100 us vs 50 us also bounds how far the 50 us baseline itself is from convergence.
    (OUT / 'exp1_numerics.json').write_text(json.dumps(dict(gate=GATE, summary=summary, rows=rows), indent=2))
    print(json.dumps(summary, indent=2))
