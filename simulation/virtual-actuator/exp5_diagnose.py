"""Experiment 5a: where do the shaping loop's (D) large errors occur?

Reruns every held-out D case that failed in Experiment 2 and labels each physics
step whose error exceeds the 0.3 Nm peak limit by what was happening nearby
(within +-20 ms): a sign change of the reference torque, a load-velocity zero
crossing, current saturation, or backlash flank change (gap plants only). Also
reports whether the error is a slow bias (1 ms means) or oscillation (residual).
"""
import json
from multiprocessing import Pool
from pathlib import Path
import numpy as np
import pandas as pd
from actuator import NOMINAL, HELD_OUT, Gains, FrictionMap, run, command_signal, DT_LOCAL
from models import reference_prediction

OUT = Path(__file__).with_name('results')
H = 25e-6
WIN = 20     # ms


def near(mask_ms, win=WIN):
    k = np.convolve(mask_ms.astype(float), np.ones(2 * win + 1), mode='same')
    return k > 0


def job(args):
    plant, sig, seed, amp, sensing, cfg = args
    c = command_signal(sig, amp, seed=seed)
    r = run(plant, 'H', Gains(**cfg['gains'][sensing]['H']), FrictionMap(**cfg['fmap']), sensing, c, h=H, seed=seed)
    ref = reference_prediction(c, r['n_phys'], H)
    err = r['shaft'] - ref                                   # (ms, n_phys)
    big = np.abs(err).max(axis=1) > 0.3                     # ms windows containing a peak violation
    ref_ms = ref[:, -1]
    w_ms = r['w_fine'][:, -1]
    ref_flip = np.r_[False, np.sign(ref_ms[1:]) != np.sign(ref_ms[:-1])]
    vel_zero = np.r_[False, np.sign(w_ms[1:]) != np.sign(w_ms[:-1])]
    sat = np.abs(r['i_cmd']) >= NOMINAL.i_max - 1e-9
    shaft_ms = r['shaft'][:, -1]
    flank = np.r_[False, np.sign(shaft_ms[1:]) != np.sign(shaft_ms[:-1])] if plant.gap else np.zeros_like(big)
    mean_err = err.mean(axis=1)
    osc = err - mean_err[:, None]
    labels = dict(ref_sign_change=near(ref_flip), load_velocity_zero=near(vel_zero), saturation=near(sat),
                  backlash_flank=near(flank))
    n_big = int(big.sum())
    out = dict(plant=plant.name, signal=sig, amp=amp, sensing=sensing, ms_over_peak=n_big,
               rms_bias=float(np.sqrt(np.mean(mean_err ** 2))), rms_oscillation=float(np.sqrt(np.mean(osc ** 2))),
               saturated_frac=float(sat.mean()))
    for k, m in labels.items():
        out[f'frac_peaks_near_{k}'] = float((big & m).sum() / n_big) if n_big else np.nan
    unexplained = big & ~np.any(np.vstack(list(labels.values())), axis=0)
    out['frac_peaks_unexplained'] = float(unexplained.sum() / n_big) if n_big else np.nan
    # Does the shaft torque oscillate near the gearbox mode? dominant frequency of the oscillatory residual
    spec = np.abs(np.fft.rfft(osc.ravel()))
    f = np.fft.rfftfreq(osc.size, H)
    out['osc_peak_hz'] = float(f[np.argmax(spec[1:]) + 1])
    return out


if __name__ == '__main__':
    rows = pd.read_csv(OUT / 'exp2_rows.csv')
    cfg = json.loads((OUT / 'exp2_config.json').read_text())['cfg']
    fail = rows[(rows.case == 'D') & (~rows.passed) & (rows.plant != 'nominal')]
    plants = {p.name: p for p in HELD_OUT}
    seeds = dict(reversal=0, steps=0, chirp=0, random=7)
    jobs = [(plants[k.plant], k.signal, seeds[k.signal], k.amp, k.sensing, cfg) for k in fail.itertuples()]
    with Pool(18) as pool:
        res = pd.DataFrame(pool.map(job, jobs))
    res = res.merge(fail[['plant', 'signal', 'amp', 'sensing', 'rms', 'peak']], on=['plant', 'signal', 'amp', 'sensing'])
    res['failed_rms'] = res.rms > 0.1
    res.to_csv(OUT / 'exp5a_diagnosis.csv', index=False)
    cols = [c for c in res.columns if c.startswith('frac_peaks')]
    summary = dict(
        failing_cases=len(res), failing_on_rms=int(res.failed_rms.sum()),
        failing_on_peak_only=int((~res.failed_rms).sum()),
        peak_attribution_mean=res[cols].mean().to_dict(),
        by_plant=res.groupby('plant')[cols + ['rms_bias', 'rms_oscillation', 'osc_peak_hz']].mean().to_dict(orient='index'),
        bias_vs_oscillation=dict(median_bias=float(res.rms_bias.median()), median_osc=float(res.rms_oscillation.median())),
        saturation_any=int((res.saturated_frac > 0).sum()))
    (OUT / 'exp5a_diagnosis.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps({k: v for k, v in summary.items() if k != 'by_plant'}, indent=2))
    print(res.groupby('plant')[['rms_bias', 'rms_oscillation', 'osc_peak_hz', 'frac_peaks_near_load_velocity_zero',
                                'frac_peaks_near_ref_sign_change', 'frac_peaks_unexplained']].mean().round(3))
