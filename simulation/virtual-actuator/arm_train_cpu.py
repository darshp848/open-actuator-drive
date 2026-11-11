"""CPU MuJoCo version of the arm training episode (pattern of rl_pilot.TrainEnv); the reference implementation
that arm_gpu_es.py is checked against. Run with ..\\.venv\\Scripts\\python.exe (or any env with mujoco + numpy).

episode(kind, theta, init_idx, task, delay_ms, compact) -> (return, success, puck trajectory)
  kind     'reference' | 'compact2'
  compact  dict with gain, wn, zeta, delay_ms, coulomb, viscous, armature (compact2 only)
  delay_ms total command transport delay [ms]; None -> nominal (4 reference, compact['delay_ms'] compact2)
Policy blocks of 20 x 1 ms; the new command replaces the previous one at substep s >= delay_ms (as on the GPU).
puck trajectory: (N_STEPS, 2) end-of-block puck (x, y).
"""
import math
import numpy as np
import mujoco as mj
import arm_task as T

T_REF = float(__import__('os').environ.get('T_REF_S', '0.030'))
VEL_HZ = int(__import__('os').environ.get('VEL_HZ', '50'))      # 1000: joint velocity filtered every 1 ms (as arm_gpu_es.py)
assert VEL_HZ in (50, 1000), VEL_HZ
VEL_A = math.exp(-2 * math.pi * 50.0 * 1e-3)
D_NOM = 4
STEPS = 20


def _expm(M):
    k = max(0, int(np.ceil(np.log2(max(np.abs(M).sum(1).max(), 1e-12)))) + 1)
    A = M / 2 ** k
    E, Tm = np.eye(len(M)), np.eye(len(M))
    for n in range(1, 20):
        Tm = Tm @ A / n; E = E + Tm
    for _ in range(k):
        E = E @ E
    return E


def ad_bd(wn, zeta, h):
    M = np.zeros((3, 3)); M[:2, :2] = [[0.0, 1.0], [-wn * wn, -2 * zeta * wn]]; M[1, 2] = wn * wn
    E = _expm(M * h)
    return E[:2, :2], E[:2, 2]


_MODELS = {}


def _model(kind, task, compact):
    key = (kind, task, tuple(sorted(compact.items())) if (compact and kind == 'compact2') else None)
    if key not in _MODELS:
        m = mj.MjModel.from_xml_string(T.train_xml(task, compact=compact if kind == 'compact2' else None))
        _MODELS[key] = (m, mj.MjData(m))
    return _MODELS[key]


def episode(kind, theta, init_idx, task=T.NOMINAL_TASK, delay_ms=None, compact=None, record_state=False,
            sensing='est', noise=None):
    m, d = _model(kind, task, compact)
    theta = np.asarray(theta, dtype=np.float64)
    if delay_ms is None:
        delay_ms = D_NOM if kind == 'reference' else int(compact['delay_ms'])
    delay_ms = int(delay_ms)
    q1, q2, px, py = T.INITS[init_idx]
    mj.mj_resetData(m, d)
    d.qpos[:] = [q1, q2, px, py]
    mj.mj_forward(m, d)
    if kind == 'reference':
        a1 = math.exp(-1e-3 / T_REF)
    else:
        Ad, Bd = ad_bd(compact['wn'], compact['zeta'], 1e-3)
        gain, coul = compact['gain'], compact['coulomb']
    x = np.zeros((2, 2))                      # per-joint state (reference uses x[j, 0])
    u_prev = np.zeros(2); u = np.zeros(2)
    # sensing='est' (deployment-matched): qm = exact q, pn = puck + noise[k] (noise: (N_STEPS, 2) metres, None = zeros),
    # w, v = 0.5*prev + 0.5*finite difference/0.02 (zero at k = 0); 'exact': true velocities. VEL_HZ=1000: w is instead
    # the 1 ms filtered difference, updated after every physics step; w_hist = w seen by the policy at each block.
    noise = np.zeros((T.N_STEPS, 2)) if noise is None else np.asarray(noise, dtype=float)
    w = np.zeros(2); v = np.zeros(2); qm_p = pn_p = None
    ret = 0.0; tail = []; traj = np.zeros((T.N_STEPS, 2)); contact_blocks = 0; w_hist = np.zeros((T.N_STEPS, 2))
    for k in range(T.N_STEPS):
        qp, qv = d.qpos, d.qvel
        if sensing == 'est':
            qm = np.array([qp[0], qp[1]], dtype=float); pn = np.array([qp[2], qp[3]], dtype=float) + noise[k]
            if k == 0:
                w[:] = 0.0; v[:] = 0.0
                qm_p = qm
            else:
                if VEL_HZ == 50:
                    w = 0.5 * w + 0.5 * (qm - qm_p) / T.DT_TASK
                v = 0.5 * v + 0.5 * (pn - pn_p) / T.DT_TASK
            if VEL_HZ == 50:
                qm_p = qm
            pn_p = pn
            w_hist[k] = w
            obs = T.observe(qm[0], qm[1], w[0], w[1], pn[0], pn[1], v[0], v[1], float(u[0]), float(u[1]))
        else:
            obs = T.observe(float(qp[0]), float(qp[1]), float(qv[0]), float(qv[1]), float(qp[2]), float(qp[3]),
                            float(qv[2]), float(qv[3]), float(u[0]), float(u[1]))
        u_prev = u.copy()
        u = T.act(theta, obs)
        for s in range(STEPS):
            for j in range(2):
                inp = u_prev[j] if s < delay_ms else u[j]
                if kind == 'reference':
                    d.ctrl[j] = max(-T.TAU_CAP, min(T.TAU_CAP, x[j, 0]))
                    x[j, 0] = a1 * x[j, 0] + (1.0 - a1) * inp
                else:
                    d.ctrl[j] = max(-T.TAU_CAP, min(T.TAU_CAP, gain * x[j, 0]))
                    x[j] = Ad @ x[j] + Bd * inp
                    d.qfrc_applied[j] = -coul * math.tanh(float(qv[j]) / 0.02)
            mj.mj_step(m, d)
            if VEL_HZ == 1000 and sensing == 'est':
                qm1 = np.array([qp[0], qp[1]], dtype=float)
                w = VEL_A * w + (1.0 - VEL_A) * (qm1 - qm_p) / 1e-3
                qm_p = qm1
            if d.ncon:
                contact_blocks += 1
        ret += T.reward(float(qp[0]), float(qp[1]), float(qv[0]), float(qv[1]), float(qp[2]), float(qp[3]),
                        float(u[0]), float(u[1]))
        traj[k] = qp[2], qp[3]
        if k >= T.N_STEPS - T.TAIL_STEPS:
            tail.append(T.goal_dist(float(qp[2]), float(qp[3])))
    success = bool(max(tail) < T.SUCCESS_TOL)
    if record_state:
        return ret, success, traj, dict(final_qpos=d.qpos.copy(), contact_substeps=contact_blocks, w_hist=w_hist)
    return ret, success, traj
