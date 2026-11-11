"""Arm task (arm_task.py) evolution-strategies trainer on the GPU (MuJoCo Warp), generalising gpu_es.py.

Worlds = (seed, candidate, init). One 20 ms control block (observation + MLP policy, delay switch,
per-joint reduced actuator model, 20 physics steps of 1 ms, reward/tail) is captured as a CUDA graph
and replayed arm_task.N_STEPS times per episode.

Kinds: 'reference' (per-joint first-order filter, T_REF, 4 ms nominal delay) and
       'compact2'  (per-joint two-state ZOH model + armature/viscous in the MJCF + Coulomb through
                    qfrc_applied on the arm dofs, fitted delay).
Per-world batched model arrays used for randomisation: body_mass (payload, puck), dof_frictionloss
(puck slide dofs), and, for fidelity of the constraint reference acceleration, body_invweight0
(payload, puck) and dof_invweight0 (puck dofs), which MuJoCo derives from the masses at set_const time.

VEL_HZ=1000 (environment, default 50) switches the joint-velocity estimate used with sensing='est' from the 20 ms
difference to a first-order filtered 1 ms difference (50 Hz cutoff) updated after every physics step (vel_kernel).

CLI: arm_gpu_es.py <kind> <pairs> <gens> [warm json|-] [rand|nominal] [n_seeds] [tag]
Run with ..\\.venv-warp\\Scripts\\python.exe.
"""
import json, math, os, sys, time
from pathlib import Path
import numpy as np
import warp as wp
import mujoco
import mujoco_warp as mjw
import arm_task as T

wp.config.quiet = True
OUT = Path(__file__).with_name('results')
STEPS_PER_BLOCK = 20
N_BLOCKS = T.N_STEPS
INITS = T.INITS
NI = len(INITS)
P = T.P
T_REF = float(os.environ.get('T_REF_S', '0.030'))
D_NOM = 4
VEL_HZ = int(os.environ.get('VEL_HZ', '50'))
assert VEL_HZ in (50, 1000), VEL_HZ
VEL_A = math.exp(-2 * math.pi * 50.0 * 1e-3)       # 50 Hz first-order filter on the 1 ms finite difference
TAU = float(T.TAU_CAP)

L1 = wp.constant(T.L1); L2 = wp.constant(T.L2)
GX = wp.constant(T.GOAL[0]); GY = wp.constant(T.GOAL[1])
HID = wp.constant(T.HID); NOBS = wp.constant(T.N_OBS)
AMAX = wp.constant(T.A_MAX)
CONTACT_R = wp.constant(T.TIP_R + T.PUCK_R)
DTT = wp.constant(T.DT_TASK)
TAIL0 = T.N_STEPS - T.TAIL_STEPS
NBODY_PAYLOAD, NBODY_PUCK = 3, 4        # body ids in arm_task.train_xml (world=0, link1, link2, payload, puck)


def expm(M):
    k = max(0, int(np.ceil(np.log2(max(np.abs(M).sum(1).max(), 1e-12)))) + 1)
    A = M / 2 ** k
    E, Tm = np.eye(len(M)), np.eye(len(M))
    for n in range(1, 20):
        Tm = Tm @ A / n; E = E + Tm
    for _ in range(k):
        E = E @ E
    return E


def ad_bd(wn, zeta, h):
    """Same zero-order-hold discretisation as gpu_es.ad_bd / cont.Ad_Bd."""
    M = np.zeros((3, 3)); M[:2, :2] = [[0.0, 1.0], [-wn * wn, -2 * zeta * wn]]; M[1, 2] = wn * wn
    E = expm(M * h)
    return E[:2, :2], E[:2, 2]


# ---------------------------------------------------------------- kernels
NOISE_SD = 0.001


@wp.kernel
def policy_kernel(params: wp.array2d(dtype=float), qpos: wp.array2d(dtype=float), qvel: wp.array2d(dtype=float),
                  u_prev: wp.array2d(dtype=float), u_new: wp.array2d(dtype=float), obs: wp.array2d(dtype=float),
                  est: int, fast: int, noise_sd_a: wp.array(dtype=float), block: wp.array(dtype=int), nseed: wp.array(dtype=int), nkey: wp.array(dtype=int),
                  w_est: wp.array2d(dtype=float), v_est: wp.array2d(dtype=float), qm_prev: wp.array2d(dtype=float),
                  pn_prev: wp.array2d(dtype=float), noise_log: wp.array3d(dtype=float)):
    i = wp.tid()
    q1 = qpos[i, 0]; q2 = qpos[i, 1]; px = qpos[i, 2]; py = qpos[i, 3]
    k = block[0]
    w1 = qvel[i, 0]; w2 = qvel[i, 1]; vx = qvel[i, 2]; vy = qvel[i, 3]
    if est == 1:
        # deployment-matched sensing: exact joint angles, puck position + N(0, 1 mm), filtered finite-difference velocities
        rs = wp.rand_init(nseed[i], nkey[i] * 256 + k)
        nx = noise_sd_a[0] * wp.randn(rs); ny = noise_sd_a[0] * wp.randn(rs)
        noise_log[i, k, 0] = nx; noise_log[i, k, 1] = ny
        pnx = px + nx; pny = py + ny
        if k == 0:
            w_est[i, 0] = 0.0; w_est[i, 1] = 0.0; v_est[i, 0] = 0.0; v_est[i, 1] = 0.0
        else:
            if fast == 0:
                w_est[i, 0] = 0.5 * w_est[i, 0] + 0.5 * (q1 - qm_prev[i, 0]) / DTT
                w_est[i, 1] = 0.5 * w_est[i, 1] + 0.5 * (q2 - qm_prev[i, 1]) / DTT
            v_est[i, 0] = 0.5 * v_est[i, 0] + 0.5 * (pnx - pn_prev[i, 0]) / DTT
            v_est[i, 1] = 0.5 * v_est[i, 1] + 0.5 * (pny - pn_prev[i, 1]) / DTT
        if fast == 0 or k == 0:         # fast: vel_kernel owns qm_prev after the start pose
            qm_prev[i, 0] = q1; qm_prev[i, 1] = q2
        pn_prev[i, 0] = pnx; pn_prev[i, 1] = pny
        px = pnx; py = pny
        w1 = w_est[i, 0]; w2 = w_est[i, 1]; vx = v_est[i, 0]; vy = v_est[i, 1]
    tx = L1 * wp.cos(q1) + L2 * wp.cos(q1 + q2)
    ty = L1 * wp.sin(q1) + L2 * wp.sin(q1 + q2)
    obs[i, 0] = wp.cos(q1); obs[i, 1] = wp.sin(q1); obs[i, 2] = wp.cos(q2); obs[i, 3] = wp.sin(q2)
    obs[i, 4] = w1 / 5.0; obs[i, 5] = w2 / 5.0
    obs[i, 6] = (px - tx) / 0.2; obs[i, 7] = (py - ty) / 0.2
    obs[i, 8] = (GX - px) / 0.2; obs[i, 9] = (GY - py) / 0.2
    obs[i, 10] = vx / 0.5; obs[i, 11] = vy / 0.5
    obs[i, 12] = u_new[i, 0] / AMAX; obs[i, 13] = u_new[i, 1] / AMAX       # previous action
    u_prev[i, 0] = u_new[i, 0]; u_prev[i, 1] = u_new[i, 1]
    a0 = params[i, 544]; a1 = params[i, 545]
    for h in range(HID):
        z = params[i, 448 + h]
        for k in range(NOBS):
            z = z + params[i, h * 14 + k] * obs[i, k]
        t = wp.tanh(z)
        a0 = a0 + params[i, 480 + h] * t
        a1 = a1 + params[i, 512 + h] * t
    u_new[i, 0] = AMAX * wp.tanh(a0)
    u_new[i, 1] = AMAX * wp.tanh(a1)


@wp.kernel
def vel_kernel(qpos: wp.array2d(dtype=float), qm_prev: wp.array2d(dtype=float), w_est: wp.array2d(dtype=float), a: float):
    """VEL_HZ=1000: joint velocity estimate, one step per 1 ms physics step (exact joint angles)."""
    i = wp.tid()
    for j in range(2):
        q = qpos[i, j]
        w_est[i, j] = a * w_est[i, j] + (1.0 - a) * (q - qm_prev[i, j]) / 1.0e-3
        qm_prev[i, j] = q


@wp.kernel
def actuator_kernel(kind: int, s: int, delay: wp.array(dtype=int), u_prev: wp.array2d(dtype=float),
                    u_new: wp.array2d(dtype=float), x: wp.array2d(dtype=float), qvel: wp.array2d(dtype=float),
                    ctrl: wp.array2d(dtype=float), qfrc: wp.array2d(dtype=float), a1: float, gain: float,
                    ad00: float, ad01: float, ad10: float, ad11: float, bd0: float, bd1: float, coul: float):
    i = wp.tid()
    for j in range(2):
        inp = u_new[i, j]
        if s < delay[i]:
            inp = u_prev[i, j]
        if kind == 0:
            # first-order reference: force uses the state before the update
            ctrl[i, j] = wp.clamp(x[i, j], -TAU_C, TAU_C)
            x[i, j] = a1 * x[i, j] + (1.0 - a1) * inp
        else:
            x0 = x[i, 2 * j]; x1 = x[i, 2 * j + 1]
            ctrl[i, j] = wp.clamp(gain * x0, -TAU_C, TAU_C)
            x[i, 2 * j] = ad00 * x0 + ad01 * x1 + bd0 * inp
            x[i, 2 * j + 1] = ad10 * x0 + ad11 * x1 + bd1 * inp
            qfrc[i, j] = -coul * wp.tanh(qvel[i, j] / 0.02)


TAU_C = wp.constant(TAU)


@wp.kernel
def reward_kernel(block: wp.array(dtype=int), qpos: wp.array2d(dtype=float), qvel: wp.array2d(dtype=float),
                  u_new: wp.array2d(dtype=float), ret: wp.array(dtype=float), tail: wp.array(dtype=float)):
    i = wp.tid()
    q1 = qpos[i, 0]; q2 = qpos[i, 1]; px = qpos[i, 2]; py = qpos[i, 3]
    w1 = qvel[i, 0]; w2 = qvel[i, 1]
    tx = L1 * wp.cos(q1) + L2 * wp.cos(q1 + q2)
    ty = L1 * wp.sin(q1) + L2 * wp.sin(q1 + q2)
    d_pg = wp.sqrt((px - GX) * (px - GX) + (py - GY) * (py - GY))
    d_tp = wp.sqrt((px - tx) * (px - tx) + (py - ty) * (py - ty))
    u1 = u_new[i, 0]; u2 = u_new[i, 1]
    r = 2.0 * d_pg + 0.5 * wp.max(d_tp - CONTACT_R, 0.0) + 0.01 * (w1 * w1 + w2 * w2) + 0.02 * (u1 * u1 + u2 * u2)
    ret[i] = ret[i] - r * DTT
    if block[0] >= TAIL0:
        tail[i] = wp.max(tail[i], d_pg)


@wp.kernel
def tick(block: wp.array(dtype=int)):
    block[0] = block[0] + 1


@wp.kernel
def mon_nefc(nefc: wp.array(dtype=int), nacon: wp.array(dtype=int), nefc_max: wp.array(dtype=int),
             nacon_max: wp.array(dtype=int)):
    i = wp.tid()
    nefc_max[i] = wp.max(nefc_max[i], nefc[i])
    if i == 0:
        nacon_max[0] = wp.max(nacon_max[0], nacon[0])


@wp.kernel
def mon_count_zero(cnt: wp.array(dtype=int)):
    cnt[wp.tid()] = 0


@wp.kernel
def mon_count(wid: wp.array(dtype=int), nacon: wp.array(dtype=int), cnt: wp.array(dtype=int)):
    c = wp.tid()
    if c < nacon[0]:
        wp.atomic_add(cnt, wid[c], 1)


@wp.kernel
def mon_max(cnt: wp.array(dtype=int), cmax: wp.array(dtype=int), touched: wp.array(dtype=int)):
    i = wp.tid()
    cmax[i] = wp.max(cmax[i], cnt[i])
    if cnt[i] > 0:
        touched[i] = 1


# ---------------------------------------------------------------- trainer
def _invweight_tables(task_proto=T.NOMINAL_TASK):
    """body_invweight0 of payload/puck (and dof_invweight0 of the puck dofs) as a function of the masses, read from
    CPU MuJoCo models on a grid (MuJoCo computes them at set_const; the GPU model needs them per world)."""
    pgrid = np.linspace(0.0, 0.5, 51)
    tab_pay = []
    for pl in pgrid:
        m = mujoco.MjModel.from_xml_string(T.train_xml(T.Task(payload=float(pl), puck_mass=task_proto.puck_mass)))
        tab_pay.append(m.body_invweight0[NBODY_PAYLOAD].copy())
    mgrid = np.linspace(0.1, 0.4, 31)
    tab_puck = []
    for pm in mgrid:
        m = mujoco.MjModel.from_xml_string(T.train_xml(T.Task(puck_mass=float(pm))))
        tab_puck.append((m.body_invweight0[NBODY_PUCK].copy(), m.dof_invweight0[2:4].copy()))
    return pgrid, np.array(tab_pay), mgrid, tab_puck


class ArmGpuTrainer:
    def __init__(self, kind, nworld, compact=None, monitor=1, task=T.NOMINAL_TASK, sensing='est'):
        self.kind, self.nw = kind, nworld
        assert sensing in ('est', 'exact')
        self.noise_sd_a = wp.array([NOISE_SD], dtype=float)      # array so it can change after graph capture
        self.sensing = sensing; self.est = 1 if sensing == 'est' else 0
        self.fast = 1 if (VEL_HZ == 1000 and self.est) else 0
        cp = compact or {}
        self.cp = cp
        m = mujoco.MjModel.from_xml_string(T.train_xml(task, compact=cp if kind == 'compact2' else None))
        self.m_host, self.d_host = m, mujoco.MjData(m)
        self.nconmax, self.njmax = 6, 24
        self.m = mjw.put_model(m)
        self.d = mjw.put_data(m, self.d_host, nworld=nworld, nconmax=self.nconmax, njmax=self.njmax)
        self.params = wp.zeros((nworld, P), dtype=float)
        self.obs = wp.zeros((nworld, T.N_OBS), dtype=float)
        self.u_prev = wp.zeros((nworld, 2), dtype=float); self.u_new = wp.zeros((nworld, 2), dtype=float)
        self.x = wp.zeros((nworld, 4), dtype=float)
        self.ret = wp.zeros(nworld, dtype=float); self.tail = wp.zeros(nworld, dtype=float)
        self.block = wp.zeros(1, dtype=int)
        self.w_est = wp.zeros((nworld, 2), dtype=float); self.v_est = wp.zeros((nworld, 2), dtype=float)
        self.qm_prev = wp.zeros((nworld, 2), dtype=float); self.pn_prev = wp.zeros((nworld, 2), dtype=float)
        self.noise_log = wp.zeros((nworld, N_BLOCKS, 2), dtype=float)       # puck-position noise actually applied
        self.nseed = wp.array(np.arange(nworld, dtype=np.int32), dtype=int)  # per-world noise seed (set per generation)
        self.nkey = wp.array(np.arange(nworld, dtype=np.int32), dtype=int)   # per-world noise stream key
        if kind == 'reference':
            self.k, self.d_nom = 0, D_NOM
            self.coef = dict(a1=math.exp(-1e-3 / T_REF), gain=1.0, ad00=0., ad01=0., ad10=0., ad11=0., bd0=0., bd1=0., coul=0.)
        else:
            Ad, Bd = ad_bd(cp['wn'], cp['zeta'], 1e-3)
            self.k, self.d_nom = 1, int(cp['delay_ms'])
            self.coef = dict(a1=0., gain=cp['gain'], ad00=Ad[0, 0], ad01=Ad[0, 1], ad10=Ad[1, 0], ad11=Ad[1, 1],
                             bd0=Bd[0], bd1=Bd[1], coul=cp['coulomb'])
        self.coef = {k: float(v) for k, v in self.coef.items()}
        # per-world model arrays
        self.bm = np.repeat(m.body_mass[None].astype(np.float32), nworld, 0)
        self.biw = np.repeat(m.body_invweight0[None].astype(np.float32), nworld, 0)
        self.dfl = np.repeat(m.dof_frictionloss[None].astype(np.float32), nworld, 0)
        self.diw = np.repeat(m.dof_invweight0[None].astype(np.float32), nworld, 0)
        self._upload_model()
        self.tabs = _invweight_tables()
        self.delay_arr = wp.array(np.full(nworld, self.d_nom, dtype=np.int32), dtype=int)
        self.monitor = monitor
        self.nefc_max = wp.zeros(nworld, dtype=int); self.nacon_max = wp.zeros(1, dtype=int)
        self.cnt = wp.zeros(nworld, dtype=int); self.ncon_max = wp.zeros(nworld, dtype=int)
        self.touched = wp.zeros(nworld, dtype=int)
        self.reset(np.tile(np.array(INITS[:1]), (nworld, 1)))
        mjw.step(self.m, self.d); wp.synchronize()
        with wp.ScopedCapture() as cap:
            self._block()
        self.graph = cap.graph

    def _upload_model(self):
        self.m.body_mass = wp.array(self.bm, dtype=float)
        self.m.body_invweight0 = wp.array(self.biw, dtype=wp.vec2f)
        self.m.dof_frictionloss = wp.array(self.dfl, dtype=float)
        self.m.dof_invweight0 = wp.array(self.diw, dtype=float)

    def _block(self):
        d, n = self.d, self.nw
        wp.launch(policy_kernel, n, [self.params, d.qpos, d.qvel, self.u_prev, self.u_new, self.obs, self.est, self.fast, self.noise_sd_a, self.block,
                                     self.nseed, self.nkey, self.w_est, self.v_est, self.qm_prev, self.pn_prev, self.noise_log])
        c = self.coef
        for s in range(STEPS_PER_BLOCK):
            wp.launch(actuator_kernel, n, [self.k, s, self.delay_arr, self.u_prev, self.u_new, self.x, d.qvel, d.ctrl,
                                           d.qfrc_applied, c['a1'], c['gain'], c['ad00'], c['ad01'], c['ad10'],
                                           c['ad11'], c['bd0'], c['bd1'], c['coul']])
            mjw.step(self.m, d)
            if self.fast:
                wp.launch(vel_kernel, n, [d.qpos, self.qm_prev, self.w_est, VEL_A])
            if self.monitor:
                wp.launch(mon_nefc, n, [d.nefc, d.nacon, self.nefc_max, self.nacon_max])
            if self.monitor >= 2:
                wp.launch(mon_count_zero, n, [self.cnt])
                wp.launch(mon_count, d.contact.worldid.shape[0], [d.contact.worldid, d.nacon, self.cnt])
                wp.launch(mon_max, n, [self.cnt, self.ncon_max, self.touched])
        wp.launch(reward_kernel, n, [self.block, d.qpos, d.qvel, self.u_new, self.ret, self.tail])
        wp.launch(tick, 1, [self.block])

    # ------------------------------------------------------------ randomisation
    def set_world_params(self, payload, puck_mass, mu, delay_total):
        """Per-world payload mass [kg], puck mass [kg], puck-table mu, and TOTAL command delay [ms]."""
        pay = np.maximum(np.asarray(payload, dtype=np.float64), 1e-4)
        self.bm[:, NBODY_PAYLOAD] = pay
        self.bm[:, NBODY_PUCK] = puck_mass
        fl = np.asarray(mu) * np.asarray(puck_mass) * 9.81
        self.dfl[:, 2] = fl; self.dfl[:, 3] = fl
        pgrid, tab_pay, mgrid, tab_puck = self.tabs
        for c in range(2):
            self.biw[:, NBODY_PAYLOAD, c] = np.interp(np.asarray(payload), pgrid, tab_pay[:, c])
        bw = np.array([t[0] for t in tab_puck]); dw = np.array([t[1] for t in tab_puck])
        for c in range(2):
            self.biw[:, NBODY_PUCK, c] = np.interp(puck_mass, mgrid, bw[:, c])
        for c in range(2):
            self.diw[:, 2 + c] = np.interp(puck_mass, mgrid, dw[:, c])
        self.m.body_mass.assign(self.bm); self.m.body_invweight0.assign(self.biw)
        self.m.dof_frictionloss.assign(self.dfl); self.m.dof_invweight0.assign(self.diw)
        self.delay_arr.assign(np.clip(np.asarray(delay_total), 0, STEPS_PER_BLOCK - 1).astype(np.int32))

    def set_noise_sd(self, sd):
        self.noise_sd_a.assign(np.array([sd], dtype=np.float32))

    def set_noise(self, seed, key):
        """seed, key: per-world int arrays (or scalars). Noise of world i at tick k is a function of (seed[i], key[i], k) only."""
        self.nseed.assign(np.broadcast_to(np.asarray(seed, dtype=np.int32), (self.nw,)).copy())
        self.nkey.assign(np.broadcast_to(np.asarray(key, dtype=np.int32), (self.nw,)).copy())

    def set_nominal(self, task=T.NOMINAL_TASK):
        n = self.nw
        self.set_world_params(np.full(n, task.payload), np.full(n, task.puck_mass), np.full(n, task.puck_mu),
                              np.full(n, self.d_nom))

    # ------------------------------------------------------------ run
    def reset(self, q0):
        """q0: (nworld, 4) rows (q1, q2, px, py); velocities and actuator states zero."""
        d = self.d
        d.qpos.assign(np.asarray(q0, dtype=np.float32))
        for a in (d.qvel, d.qacc, d.qacc_warmstart, d.ctrl, d.qfrc_applied, d.time):
            a.zero_()
        for a in (self.u_prev, self.u_new, self.x, self.ret, self.tail, self.block, self.w_est, self.v_est, self.qm_prev, self.pn_prev):
            a.zero_()

    def evaluate(self, params, q0, record=False):
        self.params.assign(params.astype(np.float32))
        self.reset(q0)
        traj = []
        for _ in range(N_BLOCKS):
            wp.capture_launch(self.graph)
            if record:
                traj.append(self.d.qpos.numpy().copy())
        wp.synchronize()
        succ = self.tail.numpy().copy() < T.SUCCESS_TOL
        out = (self.ret.numpy().copy(), succ)
        if record:
            return out + (np.stack(traj, 1),)         # (nworld, N_BLOCKS, 4)
        return out

    def monitor_report(self):
        rep = dict(max_nefc_per_world=int(self.nefc_max.numpy().max()), njmax=self.njmax,
                   max_nacon_all_worlds=int(self.nacon_max.numpy()[0]), naconmax=int(self.d.naconmax))
        if self.monitor >= 2:
            rep['max_contacts_per_world'] = int(self.ncon_max.numpy().max())
            rep['worlds_with_contact'] = int(self.touched.numpy().sum())
        return rep


def q0_for(n_cells):
    """Initial (q1, q2, px, py) for world order (cell, init) flattened, n_cells = number of (seed, candidate)."""
    return np.tile(np.array(INITS, dtype=np.float64), (n_cells, 1))


def draw_world_params(rngs, pairs):
    """One draw per (seed, pair, init), shared by the + and - candidates. Returns arrays shaped (n_seed, 2*pairs*NI)
    flattened per world order (seed, candidate, init) -> (n_seed*pop*NI,)."""
    R = T.RAND
    out = {k: [] for k in ('payload', 'puck_mass', 'mu', 'delay')}
    for r in rngs:
        pay = r.uniform(*R['payload'], (pairs, NI))
        pm = r.uniform(*R['puck_mass'], (pairs, NI))
        mu = r.uniform(*R['puck_mu'], (pairs, NI))
        dly = r.integers(R['delay_ms'][0], R['delay_ms'][1] + 1, (pairs, NI))
        for k, v in zip(out, (pay, pm, mu, dly)):
            out[k].append(np.concatenate([v, v], 0).ravel())          # (+ candidates, then - candidates)
    return {k: np.concatenate(v) for k, v in out.items()}


def run(kind, compact, seeds, pairs, gens, sigma=0.15, lr=0.05, init=None, randomise=False, log=print, sensing='est'):
    pop = 2 * pairs
    n_seed = len(seeds)
    tr = ArmGpuTrainer(kind, n_seed * pop * NI, compact, sensing=sensing)
    nkey = ((np.arange(n_seed)[:, None, None] * pairs + (np.arange(pop) % pairs)[None, :, None]) * NI + np.arange(NI)[None, None, :]).ravel()
    draw_noise = lambda: tr.set_noise(np.repeat(np.array([int(r.integers(0, 2 ** 31 - 1)) for r in rngs]), pop * NI), nkey)
    tr.set_nominal()
    rngs = [np.random.default_rng(s) for s in seeds]
    theta = np.stack([r.normal(0, 0.3, P) for r in rngs])
    if init is not None:
        theta = np.array(init, dtype=float)
        if theta.shape[0] != n_seed:
            theta = np.repeat(theta[:1], n_seed, 0) if theta.shape[0] == 1 else theta[:n_seed]
    mom = np.zeros_like(theta); vel = np.zeros_like(theta)
    q0 = q0_for(n_seed * pop)
    hist = []; t0 = time.perf_counter()
    for g in range(gens):
        eps = np.stack([r.normal(0, 1, (pairs, P)) for r in rngs])
        cands = np.concatenate([theta[:, None] + sigma * eps, theta[:, None] - sigma * eps], axis=1)
        params = np.repeat(cands.reshape(-1, P), NI, axis=0)
        if randomise:
            w = draw_world_params(rngs, pairs)
            tr.set_world_params(w['payload'], w['puck_mass'], w['mu'], w['delay'])
        if sensing == 'est':
            draw_noise()                 # after eps / randomisation draws: those streams are unchanged
        ret, succ = tr.evaluate(params, q0)
        fit = ret.reshape(n_seed, pop, NI).mean(-1)
        for j in range(n_seed):
            ranks = np.argsort(np.argsort(fit[j])) / (pop - 1) - 0.5
            grad = ((ranks[:pairs] - ranks[pairs:])[:, None] * eps[j]).sum(0) / (pairs * sigma)
            mom[j] = 0.9 * mom[j] + 0.1 * grad; vel[j] = 0.999 * vel[j] + 0.001 * grad ** 2
            theta[j] += lr * (mom[j] / (1 - 0.9 ** (g + 1))) / (np.sqrt(vel[j] / (1 - 0.999 ** (g + 1))) + 1e-8)
        if g % 10 == 9 or g == gens - 1:
            sc = succ.reshape(n_seed, pop, NI)
            hist.append(dict(gen=g + 1, wall_s=time.perf_counter() - t0, mean_candidate_return=fit.mean(1).tolist(),
                             best_candidate_return=fit.max(1).tolist(),
                             mean_candidate_success=sc.mean((1, 2)).tolist()))
            log(kind, pairs, hist[-1])
    wall = time.perf_counter() - t0
    # final nominal evaluation of each seed's mean policy
    params = np.repeat(np.repeat(theta, pop, axis=0), NI, axis=0)
    tr.set_nominal()
    if sensing == 'est':
        draw_noise()
    ret, succ = tr.evaluate(params, q0)
    final = [dict(train_return=float(ret.reshape(n_seed, -1)[j].mean()), train_success=float(succ.reshape(n_seed, -1)[j].mean()))
             for j in range(n_seed)]
    cfg = dict(sigma=sigma, lr=lr, T_REF=T_REF, rand=randomise, sensing=sensing, VEL_HZ=VEL_HZ, noise_sd=NOISE_SD, seeds=list(seeds), nominal_task=T.NOMINAL_TASK.__dict__,
               RAND=T.RAND, compact=compact if kind == 'compact2' else None, d_nom=tr.d_nom, P=P,
               nconmax=tr.nconmax, njmax=tr.njmax, monitor=tr.monitor_report())
    return dict(kind=kind, pairs=pairs, gens=gens, nworld=tr.nw, wall_s=wall, history=hist, final=final,
                env_steps_1ms=gens * tr.nw * N_BLOCKS * STEPS_PER_BLOCK, theta=theta.tolist(), config=cfg)


if __name__ == '__main__':
    kind = sys.argv[1]; pairs = int(sys.argv[2]); gens = int(sys.argv[3])
    cfg = json.loads((OUT / 'exp2_config.json').read_text())['cfg']
    warm = sys.argv[4] if len(sys.argv) > 4 else '-'
    init = json.loads(Path(warm if os.path.exists(warm) else OUT / warm).read_text())['theta'] if warm != '-' else None
    rand = len(sys.argv) > 5 and sys.argv[5] == 'rand'
    n_seeds = int(sys.argv[6]) if len(sys.argv) > 6 else 3
    tag = sys.argv[7] if len(sys.argv) > 7 else ('rand' if rand else 'nominal')
    seed0 = int(os.environ.get('SEED0', '0'))     # first seed (pilot and confirmation use disjoint seeds)
    res = run(kind, cfg['compact2']['torque'] if kind == 'compact2' else None, list(range(seed0, seed0 + n_seeds)), pairs, gens,
              init=init, randomise=rand, sensing=os.environ.get('SENSING', 'est'))
    (OUT / f'arm_es_{kind}_{tag}.json').write_text(json.dumps(res))
    print(json.dumps({k: v for k, v in res.items() if k not in ('theta', 'history')}, indent=1, default=float))
