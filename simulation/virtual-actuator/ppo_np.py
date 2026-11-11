"""PPO in NumPy on the GPU training simulator (gpu_es.GpuTrainer), for the cross-check of the
route comparison. Standard components: Gaussian policy on a pre-tanh action, clipped surrogate,
GAE(lambda), value loss, advantage normalisation, Adam, gradient-norm clipping. Several seeds
train in one batch, each with its own networks.

Run with simulation/.venv-warp:
  python ppo_np.py <kind> <iters> <seeds> <envs_per_seed> [init json|-] [rand|nominal] [tag]
"""
import json, math, sys, time
from pathlib import Path
import numpy as np
import warp as wp
import gpu_es as G

OUT = Path(__file__).with_name('results')
import os
H = int(os.environ.get('PPO_H', '32'))
GAMMA, LAM, CLIP, MAXGRAD = 0.99, 0.95, 0.2, 0.5
EPOCHS = int(os.environ.get('PPO_EPOCHS', '4'))
MINIB = int(os.environ.get('PPO_MINIB', '4'))
LR = float(os.environ.get('PPO_LR', '1e-3'))
F = np.float32


# ---------------------------------------------------------------- batched MLPs with manual backprop
def init_mlp(rng, sizes, S, out_scale):
    Ws, bs = [], []
    for k, (a, b) in enumerate(zip(sizes[:-1], sizes[1:])):
        scale = out_scale if k == len(sizes) - 2 else math.sqrt(2.0)
        W = np.stack([np.linalg.qr(rng.normal(size=(max(a, b), max(a, b))))[0][:a, :b] * scale for _ in range(S)])
        Ws.append(W.astype(F)); bs.append(np.zeros((S, 1, b), F))
    return Ws, bs


def forward(Ws, bs, x):
    """x (S, B, in) -> output and cache. tanh hidden layers, linear output."""
    acts = [x]
    for k, (W, b) in enumerate(zip(Ws, bs)):
        z = acts[-1] @ W + b
        acts.append(np.tanh(z) if k < len(Ws) - 1 else z)
    return acts[-1], acts


def backward(Ws, acts, gout):
    gWs, gbs = [], []
    g = gout
    for k in range(len(Ws) - 1, -1, -1):
        gWs.insert(0, acts[k].transpose(0, 2, 1) @ g)
        gbs.insert(0, g.sum(1, keepdims=True))
        if k:
            g = (g @ Ws[k].transpose(0, 2, 1)) * (1 - acts[k] ** 2)
    return gWs, gbs


class Adam:
    def __init__(self, params):
        self.m = [np.zeros_like(p) for p in params]; self.v = [np.zeros_like(p) for p in params]; self.t = 0

    def step(self, params, grads, lr):
        self.t += 1
        # per-seed global gradient-norm clipping
        norm = np.sqrt(sum((g ** 2).reshape(g.shape[0], -1).sum(1) for g in grads))
        scale = np.minimum(1.0, MAXGRAD / (norm + 1e-8))
        for p, g, m, v in zip(params, grads, self.m, self.v):
            g = g * scale.reshape((-1,) + (1,) * (g.ndim - 1))
            m *= 0.9; m += 0.1 * g
            v *= 0.999; v += 0.001 * g * g
            p -= lr * (m / (1 - 0.9 ** self.t)) / (np.sqrt(v / (1 - 0.999 ** self.t)) + 1e-8)


class Agent:
    def __init__(self, S, seed0):
        rng = np.random.default_rng(1000 + seed0)
        self.aW, self.ab = init_mlp(rng, [4, H, H, 1], S, 0.01)
        self.cW, self.cb = init_mlp(rng, [5, H, H, 1], S, 1.0)
        self.log_std = np.full((S, 1, 1), -0.5, F)
        self.opt_a = Adam(self.aW + self.ab + [self.log_std]); self.opt_c = Adam(self.cW + self.cb)

    def mean(self, obs):
        return forward(self.aW, self.ab, obs)[0]

    def value(self, cobs):
        return forward(self.cW, self.cb, cobs)[0][..., 0]

    def to_json(self):
        return dict(aW=[w.tolist() for w in self.aW], ab=[b.tolist() for b in self.ab], log_std=self.log_std.tolist())


def act_deterministic(params, obs):
    """Deployment policy: A_MAX * tanh(mean). params: one seed's actor (lists)."""
    x = np.asarray(obs)[None]
    for k, (W, b) in enumerate(zip(params['aW'], params['ab'])):
        x = x @ np.asarray(W) + np.asarray(b)[0]
        if k < len(params['aW']) - 1:
            x = np.tanh(x)
    return G.A_MAX * math.tanh(float(x[0, 0]))


# ---------------------------------------------------------------- environment wrapper
class Env:
    def __init__(self, kind, S, E, compact, randomise):
        self.S, self.E, self.n = S, E, S * E
        self.tr = G.GpuTrainer(kind, self.n, compact, external=True)
        self.randomise = randomise

    def rollout(self, agent, rng):
        S, E, tr = self.S, self.E, self.tr
        q0 = rng.choice(G.INITS, self.n)
        if self.randomise:
            tr.set_randomisation(rng.uniform(0.6, 1.5, self.n), rng.integers(-2, 5, self.n))
        else:
            tr.set_randomisation(np.ones(self.n), np.zeros(self.n, dtype=int))
        tr.reset(q0)
        T = G.N_BLOCKS
        obs = np.zeros((T, S, E, 4), F); acts = np.zeros((T, S, E), F); logp = np.zeros((T, S, E), F); rew = np.zeros((T, S, E), F)
        u_last = np.zeros(self.n)
        std = np.exp(agent.log_std)[:, 0, 0]
        for t in range(T):
            q = tr.d.qpos.numpy()[:, 0]; w = tr.d.qvel.numpy()[:, 0]
            o = np.stack([np.cos(q), np.sin(q), w / 8.0, u_last / G.A_MAX], -1).reshape(S, E, 4)
            m = agent.mean(o)[..., 0]
            pre = m + std[:, None] * rng.normal(size=(S, E))
            a = G.A_MAX * np.tanh(pre)
            obs[t], acts[t] = o, pre
            logp[t] = -0.5 * ((pre - m) / std[:, None]) ** 2 - np.log(std)[:, None] - 0.5 * math.log(2 * math.pi)
            tr.u_prev.assign(u_last.astype(np.float32)); tr.u_new.assign(a.reshape(-1).astype(np.float32))
            wp.capture_launch(tr.graph)
            u_last = a.reshape(-1)
            q2 = tr.d.qpos.numpy()[:, 0]; w2 = tr.d.qvel.numpy()[:, 0]
            e = (q2 - math.pi + math.pi) % (2 * math.pi) - math.pi
            rew[t] = (-(e * e + 0.05 * w2 * w2 + 0.02 * u_last * u_last) * 0.02).reshape(S, E)          # 20 ms policy period, as in rl_pilot.reward
        wp.synchronize()
        success = (tr.tail.numpy() < 0.3).reshape(S, E)
        return obs, acts, logp, rew, success


def update(agent, obs, acts, logp_old, rew, rng):
    T, S, E = rew.shape
    tfeat = np.broadcast_to((np.arange(T, dtype=F) / T)[:, None, None, None], (T, S, E, 1))
    cobs = np.concatenate([obs, tfeat], -1)
    v = agent.value(cobs.transpose(1, 0, 2, 3).reshape(S, T * E, 5)).reshape(S, T, E).transpose(1, 0, 2)
    adv = np.zeros_like(rew); last = 0.0
    for t in range(T - 1, -1, -1):
        nv = v[t + 1] if t + 1 < T else 0.0
        delta = rew[t] + GAMMA * nv - v[t]
        last = delta + GAMMA * LAM * last
        adv[t] = last
    ret = adv + v
    # flatten per seed
    f = lambda x: x.transpose(1, 0, 2, *range(3, x.ndim)).reshape(S, T * E, *x.shape[3:])
    O, CO, A, LP, ADV, RET = f(obs), f(cobs), f(acts), f(logp_old), f(adv), f(ret)
    ADV = (ADV - ADV.mean(1, keepdims=True)) / (ADV.std(1, keepdims=True) + 1e-8)
    N = T * E; mb = N // MINIB
    for _ in range(EPOCHS):
        perm = rng.permutation(N)
        for k in range(MINIB):
            idx = perm[k * mb:(k + 1) * mb]
            o, co, a, lp, ad, rt = O[:, idx], CO[:, idx], A[:, idx], LP[:, idx], ADV[:, idx], RET[:, idx]
            # actor
            m, acts_a = forward(agent.aW, agent.ab, o); m = m[..., 0]
            std = np.exp(agent.log_std)[:, :, 0]
            z = (a - m) / std
            lpn = -0.5 * z ** 2 - np.log(std) - 0.5 * math.log(2 * math.pi)
            ratio = np.exp(lpn - lp)
            unclipped = ratio * ad; clipped = np.clip(ratio, 1 - CLIP, 1 + CLIP) * ad
            use = (unclipped <= clipped).astype(float)          # gradient flows only through the min term
            dL_dlpn = -(use * ratio * ad) / mb                   # minimise -surrogate
            g_m = dL_dlpn * (z / std)
            g_logstd = (dL_dlpn * (z ** 2 - 1)).sum(1).reshape(S, 1, 1)
            gW, gb = backward(agent.aW, acts_a, g_m[..., None])
            agent.opt_a.step(agent.aW + agent.ab + [agent.log_std], gW + gb + [g_logstd], LR)
            agent.log_std[:] = np.clip(agent.log_std, -3.0, 0.5)
            # critic
            vv, acts_c = forward(agent.cW, agent.cb, co)
            g_v = (vv[..., 0] - rt) / mb
            cW, cb = backward(agent.cW, acts_c, g_v[..., None])
            agent.opt_c.step(agent.cW + agent.cb, cW + cb, LR)


def main():
    kind, iters, S, E = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
    init = sys.argv[5] if len(sys.argv) > 5 and sys.argv[5] != '-' else None
    rand = len(sys.argv) > 6 and sys.argv[6] == 'rand'
    tag = sys.argv[7] if len(sys.argv) > 7 else f'{kind}'
    cfg = json.loads((OUT / 'exp2_config.json').read_text())['cfg']
    env = Env(kind, S, E, cfg['compact2']['torque'] if kind == 'compact2' else None, rand)
    if init:
        import pickle
        agent = pickle.loads((OUT / init).read_bytes())
    else:
        agent = Agent(S, 0)
    rng = np.random.default_rng(7)
    hist = []; t0 = time.perf_counter()
    for it in range(iters):
        obs, acts, logp, rew, succ = env.rollout(agent, rng)
        update(agent, obs, acts, logp, rew, rng)
        if it % 10 == 9 or it == iters - 1:
            hist.append(dict(iter=it + 1, wall_s=time.perf_counter() - t0, ret=rew.sum(0).mean(1).round(3).tolist(),
                             success=succ.mean(1).round(3).tolist(), std=np.exp(agent.log_std).ravel().round(3).tolist()))
            print(tag, hist[-1], flush=True)
    import pickle
    (OUT / f'ppo_{tag}.pkl').write_bytes(pickle.dumps(agent))
    (OUT / f'ppo_{tag}.json').write_text(json.dumps(dict(kind=kind, iters=iters, seeds=S, envs=E, randomise=rand,
                                                         wall_s=time.perf_counter() - t0, history=hist,
                                                         policies=[dict(aW=[w[s].tolist() for w in agent.aW],
                                                                        ab=[b[s].tolist() for b in agent.ab])
                                                                   for s in range(S)])))


if __name__ == '__main__':
    main()
