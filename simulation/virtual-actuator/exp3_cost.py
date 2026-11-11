"""Experiment 3: simulation cost of each actuator representation (CPU only).

3a  Same simulator (MuJoCo 3.3.7), same physics step, batch size 1, identical command
    sequences: detailed plant (+ local controller), fitted compact model, prescribed
    reference, and a bare torque source. Wall time per simulated second, median of
    repeats, and the actuator share = 1 - t_bare / t_model.
3b  Largest admissible physics step for each reduced model: load-angle RMS difference
    against a 0.1 ms run of the same model, gate 0.01 rad, eval signals at 1.5 Nm.
3c  Actuator update alone, batched in NumPy (a CPU stand-in for a vectorised trainer),
    cost per environment per 1 ms of simulated time.
A CPU result is not a GPU result.
"""
import json, time, statistics
from pathlib import Path
import numpy as np
from actuator import NOMINAL, Gains, FrictionMap, command_signal, VirtualActuator, LocalController
from models import Compact, TrainingSim

OUT = Path(__file__).with_name('results')
cfg = json.loads((OUT / 'exp2_config.json').read_text())
CP = Compact(**cfg['cfg']['compact']['torque'])
FMAP = FrictionMap(**cfg['cfg']['fmap'])
GS = Gains(**cfg['cfg']['gains']['torque']['S'])
SIGNALS = [('reversal', 0), ('steps', 0), ('chirp', 0), ('random', 7)]
REPEATS = 5


def time_training(kind, c, h):
    sim = TrainingSim(kind, NOMINAL.mass, h, CP if kind == 'compact' else None)
    t0 = time.perf_counter(); sim.rollout(c, len(c) * 0.02); return time.perf_counter() - t0


def time_detailed(c, h):
    act = VirtualActuator(NOMINAL, h); ctl = LocalController('S', GS, FMAP, 'torque', 0)
    per = 20
    t0 = time.perf_counter()
    for j in range(len(c)):
        for _ in range(per):
            act.step_ms(ctl(c[j], act)[0])
    return time.perf_counter() - t0


def exp3a():
    out = {}
    for h in [1e-3, 25e-6]:
        res = {}
        for kind in ['bare', 'reference', 'compact', 'detailed']:
            ts = []
            for sig, seed in SIGNALS:
                c = command_signal(sig, 1.0, seed=seed)
                reps = [time_detailed(c, h) if kind == 'detailed' else time_training(kind, c, h)
                        for _ in range(REPEATS if kind != 'detailed' or h > 1e-4 else 2)]
                ts.append(statistics.median(reps) / (len(c) * 0.02))
            res[kind] = dict(wall_per_sim_s=float(np.mean(ts)))
        for kind in res:
            res[kind]['actuator_share'] = 1 - res['bare']['wall_per_sim_s'] / res[kind]['wall_per_sim_s']
            res[kind]['relative_to_reference'] = res[kind]['wall_per_sim_s'] / res['reference']['wall_per_sim_s']
        out[f'{h*1e6:g}us'] = res
        print(h, json.dumps(res, indent=1), flush=True)
    return out


def exp3b():
    steps = [0.5e-3, 1e-3, 2.5e-3, 5e-3, 10e-3, 20e-3]
    out = {}
    for kind in ['reference', 'compact']:
        rows = []
        for sig, seed in SIGNALS:
            c = command_signal(sig, 1.5, seed=seed)
            base = TrainingSim(kind, NOMINAL.mass, 1e-4, CP).rollout(c, len(c) * 0.02)
            for h in steps:
                th = TrainingSim(kind, NOMINAL.mass, h, CP).rollout(c, len(c) * 0.02)
                rows.append(dict(signal=sig, h_ms=h * 1e3, angle_rms_diff=float(np.sqrt(np.mean((th - base) ** 2))),
                                 finite=bool(np.all(np.isfinite(th)))))
        largest = None
        for h in steps:
            sel = [r for r in rows if r['h_ms'] == h * 1e3]
            if all(r['finite'] and r['angle_rms_diff'] <= 0.01 for r in sel):
                largest = h * 1e3
            else:
                break
        out[kind] = dict(rows=rows, largest_passing_step_ms=largest)
        print(kind, 'largest passing step', largest, flush=True)
    return out


def exp3c():
    """Per-env cost of advancing only the actuator state by 1 ms (policy-free)."""
    out = {}
    for n in [1, 128, 4096]:
        rng = np.random.default_rng(0)
        c = rng.uniform(-1, 1, n); w = rng.uniform(-3, 3, n); a = np.zeros(n)
        x = np.zeros(n); i = np.zeros(n); integ = np.zeros(n); th_r = np.zeros(n); th_l = np.zeros(n)
        w_r = np.zeros(n); w_l = np.zeros(n)
        aR = np.exp(-1e-3 / 0.030); aC = np.exp(-1e-3 / CP.tau)
        reps = 200

        def ref():
            nonlocal x
            x = aR * x + (1 - aR) * c
            return x

        def compact():
            nonlocal x
            x = aC * x + (1 - aC) * c
            return CP.gain * x - CP.coulomb * np.tanh(w / 0.02) - CP.viscous * w - CP.armature * a

        def detailed():
            # 40 substeps of 25 us: exact R-L update, PI current loop, Stribeck friction, spring, two inertias
            nonlocal i, integ, th_r, th_l, w_r, w_l
            p = NOMINAL; h = 25e-6; E = np.exp(-p.R * h / p.L)
            for _ in range(40):
                err = c / (p.ratio * p.kt) - i
                integ = integ + 3770 * err * h
                v = np.clip(3.14 * err + integ, -p.v_max, p.v_max)
                iss = (v - p.kt * p.ratio * w_r) / p.R
                i = iss + (i - iss) * E
                f = (p.coulomb + p.stribeck * np.exp(-(w_r / p.v_stribeck) ** 2)) * np.tanh(w_r / 0.01) + p.viscous * w_r
                ts = p.stiffness * (th_r - th_l) + p.damping * (w_r - w_l)
                w_r = w_r + h * (p.ratio * p.kt * i - f - ts) / p.j_rotor_out
                w_l = w_l + h * (ts - 2.45 * np.sin(th_l)) / 0.0833
                th_r = th_r + h * w_r; th_l = th_l + h * w_l
            return ts

        res = {}
        for name, fn in [('reference', ref), ('compact', compact), ('detailed', detailed)]:
            fn()
            t0 = time.perf_counter()
            for _ in range(reps):
                fn()
            res[name] = (time.perf_counter() - t0) / reps / n * 1e9   # ns per env per simulated ms
        out[n] = res
        print('batch', n, res, flush=True)
    return out


if __name__ == '__main__':
    result = dict(note='CPU only, single process; batch size 1 unless stated',
                  same_step=exp3a(), admissible_step=exp3b(), batched_actuator_ns_per_env_ms=exp3c())
    (OUT / 'exp3_cost.json').write_text(json.dumps(result, indent=2))
