"""Experiment 7 (pilot): pendulum swing-up policies trained on reduced actuator models,
frozen, then run on the detailed virtual actuator.

Provisional: started while the interface gate (Follow-up 2) is under review. Labelled a
pilot with 3 seeds per training model, as the plan prescribes before any learning claim.

Policies (MLP 4-16-1, tanh, 50 Hz, torque request limited to +-1 Nm, inside the
confirmed amplitude range) are trained with antithetic evolution strategies (NumPy only):
  R-policy  trained on the prescribed reference (T = 60 ms, 4 ms delay)
  A2-policy trained on the fitted second-order compact model of the conventional loop
Deployment on the detailed plant (torque sensor, 25 us physics):
  R  + H3 shaping loop + damping   (proposed)
  R  + E3 plain PI + damping       (equal-speed control)
  A2 + S  conventional loop        (compact-model baseline)
  R  + S  conventional loop        (negative control)
Plants: nominal, the 10 held-out plants, and the first 20 fresh plants of Follow-up 2.
Success: |angle from upright| < 0.3 rad for the whole final 1.5 s of an 8 s episode.
"""
import json, math, os, sys, time
from multiprocessing import Pool
from pathlib import Path
import numpy as np

os.environ.setdefault('T_REF_S', '0.060')
import mujoco as mj
from actuator import (NOMINAL, HELD_OUT, Gains, FrictionMap, VirtualActuator, LocalController, DT_LOCAL, DT_TASK,
                      D_NOM_MS, T_REF, TAU_CAP)
from models import Compact2, train_xml
from cont import Ad_Bd

OUT = Path(__file__).with_name('results')
EP_S = 8.0
N_STEPS = round(EP_S / DT_TASK)
A_MAX = 1.0
HID = 16
INITS = [0.0, 0.25, -0.25]


# ---------------------------------------------------------------- policy
def n_params():
    return 4 * HID + HID + HID + 1


def act(theta, obs):
    W1 = theta[:4 * HID].reshape(HID, 4); b1 = theta[4 * HID:5 * HID]
    w2 = theta[5 * HID:6 * HID]; b2 = theta[-1]
    return A_MAX * math.tanh(float(w2 @ np.tanh(W1 @ obs + b1) + b2))


def observe(q, w, u_prev):
    return np.array([math.cos(q), math.sin(q), w / 8.0, u_prev / A_MAX])


def upright_err(q):
    return (q - math.pi + math.pi) % (2 * math.pi) - math.pi


def reward(q, w, u):
    e = upright_err(q)
    return -(e * e + 0.05 * w * w + 0.02 * u * u) * DT_TASK


# ---------------------------------------------------------------- training simulator
class TrainEnv:
    """Rigid pendulum with a reduced actuator model, 1 ms physics, ms-resolution delay buffer."""
    def __init__(self, kind, compact=None, mass=1.0):
        self.kind, self.cp = kind, compact
        mk = 'reference' if kind == 'reference' else 'bare'
        self.m = mj.MjModel.from_xml_string(train_xml(mass, 1e-3, mk if kind == 'reference' else 'compact2', compact))
        self.d = mj.MjData(self.m)
        self.delay = D_NOM_MS if kind == 'reference' else compact.delay_ms
        if kind == 'compact2':
            self.Ad, self.Bd = Ad_Bd(compact.wn, compact.zeta, 1e-3)

    def episode(self, theta, q0):
        m, d = self.m, self.d
        mj.mj_resetData(m, d); d.qpos[0] = q0; mj.mj_forward(m, d)
        buf = [0.0] * (self.delay + 1)
        z = np.zeros(2); u = 0.0; ret = 0.0; tail = []
        for k in range(N_STEPS):
            u = act(theta, observe(float(d.qpos[0]), float(d.qvel[0]), u))
            for _ in range(20):
                buf.append(u); uin = buf.pop(0)
                if self.kind == 'reference':
                    d.ctrl[0] = uin
                else:
                    d.ctrl[0] = max(-TAU_CAP, min(TAU_CAP, self.cp.gain * z[0]))
                    z = self.Ad @ z + self.Bd * uin
                    d.qfrc_applied[0] = -self.cp.coulomb * math.tanh(float(d.qvel[0]) / 0.02)
                mj.mj_step(m, d)
            ret += reward(float(d.qpos[0]), float(d.qvel[0]), u)
            if k * DT_TASK >= EP_S - 1.5:
                tail.append(abs(upright_err(float(d.qpos[0]))))
        return ret, max(tail) < 0.3


_ENV = {}


def train_fitness(args):
    kind, cp, theta = args
    key = kind
    if key not in _ENV:
        _ENV[key] = TrainEnv(kind, Compact2(**cp) if cp else None)
    env = _ENV[key]
    rs = [env.episode(theta, q0) for q0 in INITS]
    return sum(r for r, _ in rs) / len(rs), sum(s for _, s in rs) / len(rs)


def train(kind, cp, seed, pool, gens=500, pairs=32, sigma=0.15, lr=0.05):
    rng = np.random.default_rng(seed)
    theta = rng.normal(0, 0.3, n_params())
    m = np.zeros_like(theta); v = np.zeros_like(theta)
    hist = []
    t0 = time.perf_counter(); episodes = 0
    for g in range(gens):
        eps = rng.normal(0, 1, (pairs, theta.size))
        cands = [theta + sigma * e for e in eps] + [theta - sigma * e for e in eps]
        fit = np.array([f for f, _ in pool.map(train_fitness, [(kind, cp, c) for c in cands])])
        episodes += len(cands) * len(INITS)
        ranks = np.argsort(np.argsort(fit)) / (len(fit) - 1) - 0.5
        grad = ((ranks[:pairs] - ranks[pairs:])[:, None] * eps).sum(0) / (pairs * sigma)
        m = 0.9 * m + 0.1 * grad; v = 0.999 * v + 0.001 * grad ** 2
        theta = theta + lr * (m / (1 - 0.9 ** (g + 1))) / (np.sqrt(v / (1 - 0.999 ** (g + 1))) + 1e-8)
        if g % 10 == 9 or g == gens - 1:
            f, s = train_fitness((kind, cp, theta))
            hist.append(dict(gen=g + 1, train_return=f, train_success=s))
            print(kind, seed, hist[-1], flush=True)
    return theta, dict(history=hist, wall_s=time.perf_counter() - t0, episodes=episodes,
                       env_steps_1ms=episodes * N_STEPS * 20)


# ---------------------------------------------------------------- deployment on the detailed plant
def deploy(args):
    plant, ctl_kind, gains, fmap, theta, q0, seed = args
    a = VirtualActuator(plant, 25e-6, q0=0.0)
    a.d.qpos[:] = q0; mj.mj_forward(a.m, a.d)
    ctl = LocalController('S' if ctl_kind == 'S' else 'H', Gains(**gains), FrictionMap(**fmap), 'torque', seed)
    rng = np.random.default_rng(seed)
    enc = 2 * np.pi / 16384
    q_prev = None; w_f = 0.0; u = 0.0; ret = 0.0; tail = []
    pending = []          # (arrival_ms, value)
    cur = 0.0; peak_i = 0.0
    for k in range(N_STEPS):
        q = round(float(a.d.qpos[1]) / enc) * enc
        w_raw = 0.0 if q_prev is None else (q - q_prev) / DT_TASK
        q_prev = q; w_f = 0.5 * w_f + 0.5 * w_raw if k else 0.0
        u = act(theta, observe(q, w_f, u))
        dly = plant.delay_ms + (int(rng.integers(0, plant.jitter_ms + 1)) if plant.jitter_ms else 0)
        pending.append((k * 20 + dly, u))
        for j in range(20):
            t_ms = k * 20 + j
            while pending and pending[0][0] <= t_ms:
                cur = pending.pop(0)[1]
            ic, _ = ctl(cur, a)
            a.step_ms(ic)
            if not a.ok():
                return dict(plant=plant.name, ok=False, success=False, ret=float('nan'))
        ret += reward(float(a.d.qpos[1]), float(a.d.qvel[1]), u)
        if k * DT_TASK >= EP_S - 1.5:
            tail.append(abs(upright_err(float(a.d.qpos[1]))))
    return dict(plant=plant.name, ok=True, success=bool(max(tail) < 0.3), ret=ret, final_err=float(tail[-1]))


if __name__ == '__main__':
    from exp6_confirm import draw_plants
    cfg = json.loads((OUT / 'exp2_config.json').read_text())['cfg']
    sel = json.loads((OUT / 'exp5c_damping_wide.json').read_text())
    cp2 = cfg['compact2']['torque']
    seeds = [0, 1, 2]
    policies, train_log = {}, {}
    with Pool(18) as pool:
        for kind, cp in [('reference', None), ('compact2', cp2)]:
            for s in seeds:
                theta, log = train(kind, cp, s, pool)
                policies[(kind, s)] = theta.tolist(); train_log[f'{kind}/{s}'] = log
        np.save(OUT / 'rl_policies.npy', np.array([policies[k] for k in sorted(policies)]))
        plants = [NOMINAL] + HELD_OUT + draw_plants()[:20]
        deployments = [('R+H3', 'reference', 'H', sel['torque/H3']['selected']),
                       ('R+E3', 'reference', 'H', sel['torque/E3']['selected']),
                       ('A2+S', 'compact2', 'S', cfg['gains']['torque']['S']),
                       ('R+S', 'reference', 'S', cfg['gains']['torque']['S'])]
        jobs, keys = [], []
        for name, kind, ck, g in deployments:
            for s in seeds:
                for p in plants:
                    for i, q0 in enumerate(INITS):
                        jobs.append((p, ck, g, cfg['fmap'], np.array(policies[(kind, s)]), q0, 100 * s + i))
                        keys.append((name, s, p.name, q0))
        t0 = time.perf_counter()
        res = pool.map(deploy, jobs)
    rows = [dict(case=k[0], seed=k[1], init=k[3], **r) for k, r in zip(keys, res)]
    import pandas as pd
    df = pd.DataFrame(rows)
    df['plant_set'] = np.where(df.plant == 'nominal', 'nominal', np.where(df.plant.str.startswith('fresh'), 'fresh', 'held_out'))
    df.to_csv(OUT / 'rl_pilot_rows.csv', index=False)
    summary = df.groupby(['case', 'plant_set']).success.mean().unstack().round(3)
    by_seed = df.groupby(['case', 'seed']).success.mean().unstack().round(3)
    train_final = {k: v['history'][-1] for k, v in train_log.items()}
    print(summary); print(by_seed); print(json.dumps(train_final, indent=1))
    (OUT / 'rl_pilot.json').write_text(json.dumps(dict(
        T_ref_ms=T_REF * 1e3, success_by_case_and_set={c: r.to_dict() for c, r in summary.iterrows()},
        success_by_case_and_seed={c: {str(k): v for k, v in r.to_dict().items()} for c, r in by_seed.iterrows()},
        training=train_log, deploy_wall_s=time.perf_counter() - t0), indent=2, default=float))
