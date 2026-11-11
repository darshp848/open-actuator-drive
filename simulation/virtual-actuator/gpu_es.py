"""Experiment 8: the RL pilot's policy training on the GPU (MuJoCo Warp 3.16, Warp 1.18).

Same task, policy, reward, episode, initial states, actuator models and ES update as
rl_pilot.py; only the simulator backend changes. Every world runs one (candidate, initial
state) pair. One 20 ms control block (policy MLP, delay buffer, reduced actuator model,
20 physics steps of 1 ms, reward) is captured once as a CUDA graph and replayed 400 times
per episode, so the CPU only does the ES update between generations.

Run with simulation/.venv-warp. Writes results/gpu_es_<kind>_<pairs>.json and the
trained parameter vectors; deployment on the detailed plant reuses rl_pilot.deploy (CPU).
"""
import json, math, sys, time
from pathlib import Path
import numpy as np
import warp as wp
import mujoco
import mujoco_warp as mjw

wp.config.quiet = True
OUT = Path(__file__).with_name('results')
HID = 16
P = 4 * HID + HID + HID + 1
A_MAX = 1.0
STEPS_PER_BLOCK = 20
N_BLOCKS = 400                    # 8 s episode
INITS = [0.0, 0.25, -0.25]
T_REF = float(__import__('os').environ.get('T_REF_S', '0.060'))   # reference time constant (env override)
D_NOM = 4


def model_xml(mass, arm, damp):
    return f'''<mujoco><option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81"/>
<worldbody><body name="link"><inertial pos="0 0 -.25" mass="{mass}" diaginertia="{mass/48} {mass/48} {mass/1000}"/>
<joint name="load" type="hinge" axis="0 1 0" damping="{0.002 + damp}" armature="{arm}"/>
<geom type="capsule" fromto="0 0 0 0 0 -.5" size=".02" contype="0" conaffinity="0"/></body></worldbody>
<actuator><motor joint="load" gear="1" ctrllimited="true" ctrlrange="-2 2"/></actuator></mujoco>'''


def expm(M):
    """Matrix exponential by scaling and squaring of a Taylor series (small matrices; avoids SciPy here)."""
    k = max(0, int(np.ceil(np.log2(max(np.abs(M).sum(1).max(), 1e-12)))) + 1)
    A = M / 2 ** k
    E, T = np.eye(len(M)), np.eye(len(M))
    for n in range(1, 20):
        T = T @ A / n; E = E + T
    for _ in range(k):
        E = E @ E
    return E


def ad_bd(wn, zeta, h):
    """Same zero-order-hold discretisation as cont.Ad_Bd."""
    M = np.zeros((3, 3)); M[:2, :2] = [[0.0, 1.0], [-wn * wn, -2 * zeta * wn]]; M[1, 2] = wn * wn
    E = expm(M * h)
    return E[:2, :2], E[:2, 2]


@wp.func
def wrap_pi(x: float):
    y = x + 3.14159265
    return y - 6.2831853 * wp.floor(y / 6.2831853) - 3.14159265


@wp.kernel
def policy_kernel(params: wp.array2d(dtype=float), qpos: wp.array2d(dtype=float), qvel: wp.array2d(dtype=float),
                  u_prev: wp.array(dtype=float), u_new: wp.array(dtype=float)):
    i = wp.tid()
    q = qpos[i, 0]
    o0 = wp.cos(q); o1 = wp.sin(q); o2 = qvel[i, 0] / 8.0; o3 = u_new[i] / 1.0
    u_prev[i] = u_new[i]
    acc = params[i, 96]
    for h in range(16):
        z = params[i, h * 4 + 0] * o0 + params[i, h * 4 + 1] * o1 + params[i, h * 4 + 2] * o2             + params[i, h * 4 + 3] * o3 + params[i, 64 + h]
        acc = acc + params[i, 80 + h] * wp.tanh(z)
    u_new[i] = 1.0 * wp.tanh(acc)


@wp.kernel
def actuator_kernel(kind: int, s: int, delay: wp.array(dtype=int), u_prev: wp.array(dtype=float), u_new: wp.array(dtype=float),
                    x: wp.array2d(dtype=float), qvel: wp.array2d(dtype=float), ctrl: wp.array2d(dtype=float),
                    qfrc: wp.array2d(dtype=float), a1: float, gain: float, ad00: float, ad01: float, ad10: float,
                    ad11: float, bd0: float, bd1: float, coul: float):
    i = wp.tid()
    inp = u_new[i]
    if s < delay[i]:
        inp = u_prev[i]
    if kind == 0:
        # first-order reference: force uses the state before the update (filterexact, actearly off)
        ctrl[i, 0] = wp.clamp(x[i, 0], -2.0, 2.0)
        x[i, 0] = a1 * x[i, 0] + (1.0 - a1) * inp
    else:
        ctrl[i, 0] = wp.clamp(gain * x[i, 0], -2.0, 2.0)
        z0 = ad00 * x[i, 0] + ad01 * x[i, 1] + bd0 * inp
        z1 = ad10 * x[i, 0] + ad11 * x[i, 1] + bd1 * inp
        x[i, 0] = z0; x[i, 1] = z1
        qfrc[i, 0] = -coul * wp.tanh(qvel[i, 0] / 0.02)


@wp.kernel
def reward_kernel(block: wp.array(dtype=int), qpos: wp.array2d(dtype=float), qvel: wp.array2d(dtype=float),
                  u_new: wp.array(dtype=float), ret: wp.array(dtype=float), tail: wp.array(dtype=float)):
    i = wp.tid()
    e = wrap_pi(qpos[i, 0] - 3.14159265)
    w = qvel[i, 0]; u = u_new[i]
    ret[i] = ret[i] - (e * e + 0.05 * w * w + 0.02 * u * u) * 0.02
    if block[0] >= 325:
        tail[i] = wp.max(tail[i], wp.abs(e))


@wp.kernel
def tick(block: wp.array(dtype=int)):
    block[0] = block[0] + 1


class GpuTrainer:
    def __init__(self, kind, nworld, compact=None, external=False):
        self.kind, self.nw, self.external = kind, nworld, external   # external: actions written from the host (PPO)
        cp = compact or {}
        m = mujoco.MjModel.from_xml_string(model_xml(1.0, cp.get('armature', 0.0), cp.get('viscous', 0.0)))
        self.m_host, self.d_host = m, mujoco.MjData(m)
        self.m = mjw.put_model(m)
        self.d = mjw.put_data(m, self.d_host, nworld=nworld)
        self.params = wp.zeros((nworld, P), dtype=float)
        self.u_prev = wp.zeros(nworld, dtype=float); self.u_new = wp.zeros(nworld, dtype=float)
        self.x = wp.zeros((nworld, 2), dtype=float)
        self.ret = wp.zeros(nworld, dtype=float); self.tail = wp.zeros(nworld, dtype=float)
        self.block = wp.zeros(1, dtype=int)
        if kind == 'reference':
            self.k, self.delay = 0, D_NOM
            self.coef = dict(a1=math.exp(-1e-3 / T_REF), gain=1.0, ad00=0., ad01=0., ad10=0., ad11=0., bd0=0., bd1=0., coul=0.)
        else:
            Ad, Bd = ad_bd(cp['wn'], cp['zeta'], 1e-3)
            self.k, self.delay = 1, int(cp['delay_ms'])
            self.coef = dict(a1=0., gain=cp['gain'], ad00=Ad[0, 0], ad01=Ad[0, 1], ad10=Ad[1, 0], ad11=Ad[1, 1],
                             bd0=Bd[0], bd1=Bd[1], coul=cp['coulomb'])
        self.coef = {k: float(v) for k, v in self.coef.items()}
        # per-world load mass and command delay (randomisation); nominal until set_randomisation is called
        self.bm = np.repeat(m.body_mass[None].astype(np.float32), nworld, 0)
        self.bi = np.repeat(m.body_inertia[None].astype(np.float32), nworld, 0)
        self.m.body_mass = wp.array(self.bm, dtype=float)
        self.m.body_inertia = wp.array(self.bi, dtype=wp.vec3f)
        self.delay_arr = wp.array(np.full(nworld, self.delay, dtype=np.int32), dtype=int)
        self.reset(np.zeros(nworld))
        mjw.step(self.m, self.d); wp.synchronize()
        with wp.ScopedCapture() as cap:
            self._block()
        self.graph = cap.graph

    def _block(self):
        d, n = self.d, self.nw
        if not self.external:
            wp.launch(policy_kernel, n, [self.params, d.qpos, d.qvel, self.u_prev, self.u_new])
        for s in range(STEPS_PER_BLOCK):
            c = self.coef
            wp.launch(actuator_kernel, n, [self.k, s, self.delay_arr, self.u_prev, self.u_new, self.x, d.qvel, d.ctrl,
                                           d.qfrc_applied, c['a1'], c['gain'], c['ad00'], c['ad01'], c['ad10'],
                                           c['ad11'], c['bd0'], c['bd1'], c['coul']])
            mjw.step(self.m, d)
        wp.launch(reward_kernel, n, [self.block, d.qpos, d.qvel, self.u_new, self.ret, self.tail])
        wp.launch(tick, 1, [self.block])

    def set_randomisation(self, mass, delay_offset):
        """mass (kg) and transport-delay offset from the nominal 4 ms (ms), one value per world."""
        self.bm[:, 1] = mass
        self.bi[:, 1] = np.stack([mass / 48, mass / 48, mass / 1000], 1)
        self.m.body_mass.assign(self.bm); self.m.body_inertia.assign(self.bi)
        self.delay_arr.assign(np.clip(self.delay + delay_offset, 0, STEPS_PER_BLOCK - 1).astype(np.int32))

    def reset(self, q0):
        d = self.d
        d.qpos.assign(q0.reshape(-1, 1).astype(np.float32))
        for a in (d.qvel, d.qacc, d.qacc_warmstart, d.ctrl, d.qfrc_applied, d.time):
            a.zero_()
        for a in (self.u_prev, self.u_new, self.x, self.ret, self.tail, self.block):
            a.zero_()

    def evaluate(self, params, q0):
        self.params.assign(params.astype(np.float32))
        self.reset(q0)
        for _ in range(N_BLOCKS):
            wp.capture_launch(self.graph)
        wp.synchronize()
        return self.ret.numpy().copy(), self.tail.numpy().copy() < 0.3


def run(kind, compact, seeds, pairs, gens, sigma=0.15, lr=0.05, init=None, tag='', randomise=False):
    pop = 2 * pairs
    n_seed = len(seeds)
    tr = GpuTrainer(kind, n_seed * pop * len(INITS), compact)
    rngs = [np.random.default_rng(s) for s in seeds]
    theta = np.stack([r.normal(0, 0.3, P) for r in rngs])
    if init is not None:
        theta = np.array(init, dtype=float)       # warm start (one row per seed)
    mom = np.zeros_like(theta); vel = np.zeros_like(theta)
    q0 = np.tile(np.repeat(INITS, 1), n_seed * pop)            # world = (seed, candidate, init)
    hist = []; t0 = time.perf_counter()
    for g in range(gens):
        eps = np.stack([r.normal(0, 1, (pairs, P)) for r in rngs])          # (seed, pairs, P)
        cands = np.concatenate([theta[:, None] + sigma * eps, theta[:, None] - sigma * eps], axis=1)  # (seed, pop, P)
        params = np.repeat(cands.reshape(-1, P), len(INITS), axis=0)
        if randomise:
            # one draw per (seed, pair, initial state), shared by the + and - candidate of a pair
            mass = np.stack([r.uniform(0.6, 1.5, (pairs, len(INITS))) for r in rngs])
            dly = np.stack([r.integers(-2, 5, (pairs, len(INITS))) for r in rngs])      # total delay 2..8 ms
            tr.set_randomisation(np.concatenate([mass, mass], 1).ravel(), np.concatenate([dly, dly], 1).ravel())
        ret, _ = tr.evaluate(params, q0)
        fit = ret.reshape(n_seed, pop, len(INITS)).mean(-1)
        for j in range(n_seed):
            ranks = np.argsort(np.argsort(fit[j])) / (pop - 1) - 0.5
            grad = ((ranks[:pairs] - ranks[pairs:])[:, None] * eps[j]).sum(0) / (pairs * sigma)
            mom[j] = 0.9 * mom[j] + 0.1 * grad; vel[j] = 0.999 * vel[j] + 0.001 * grad ** 2
            theta[j] += lr * (mom[j] / (1 - 0.9 ** (g + 1))) / (np.sqrt(vel[j] / (1 - 0.999 ** (g + 1))) + 1e-8)
        if g % 10 == 9 or g == gens - 1:
            hist.append(dict(gen=g + 1, wall_s=time.perf_counter() - t0, mean_candidate_return=fit.mean(1).tolist()))
            print(kind, pairs, hist[-1], flush=True)
    wall = time.perf_counter() - t0
    # final check of each seed's mean policy on the GPU training model
    params = np.repeat(np.repeat(theta, pop, axis=0), len(INITS), axis=0)
    tr.set_randomisation(np.ones(tr.nw), np.zeros(tr.nw, dtype=int))     # report training success on the nominal model
    ret, succ = tr.evaluate(params, q0)
    final = [dict(train_return=float(ret.reshape(n_seed, -1)[j].mean()), train_success=float(succ.reshape(n_seed, -1)[j].mean()))
             for j in range(n_seed)]
    res = dict(kind=kind, pairs=pairs, gens=gens, nworld=tr.nw, wall_s=wall, history=hist, final=final,
               env_steps_1ms=gens * tr.nw * N_BLOCKS * STEPS_PER_BLOCK, theta=theta.tolist())
    (OUT / f'gpu_es_{kind}_{pairs}{tag}.json').write_text(json.dumps(res))
    return res


if __name__ == '__main__':
    kind = sys.argv[1]; pairs = int(sys.argv[2]); gens = int(sys.argv[3])
    cfg = json.loads((OUT / 'exp2_config.json').read_text())['cfg']
    # optional: argv[4] warm-start json or '-', argv[5] 'rand' for load/delay randomisation, argv[6] number of seeds
    init = json.loads((OUT / sys.argv[4]).read_text())['theta'] if len(sys.argv) > 4 and sys.argv[4] != '-' else None
    rand = len(sys.argv) > 5 and sys.argv[5] == 'rand'
    n_seeds = int(sys.argv[6]) if len(sys.argv) > 6 else 3
    tag = ('' if abs(T_REF - 0.060) < 1e-9 or kind != 'reference' else f'_T{round(T_REF * 1e3)}') +           ('_warm' if init is not None else '') + ('_rand' if rand else '') + (f'_s{n_seeds}' if n_seeds != 3 else '')
    res = run(kind, cfg['compact2']['torque'] if kind == 'compact2' else None, list(range(n_seeds)), pairs, gens,
              init=init, tag=tag, randomise=rand)
    print(json.dumps({k: v for k, v in res.items() if k not in ('theta', 'history')}, indent=1))
