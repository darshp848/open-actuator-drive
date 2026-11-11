"""Diagnosis of the Task B pilot: why do reference-trained policies fail on the detailed arm?

Steps the same policy through a ladder of simulators, nominal task, all 6 starts:
  A  training model, 1 ms, reference actuator (what ES optimised)
  B  training model, 25 us, reference actuator (physics step only)
  C  detailed plant, local loop, exact observations (actuator only)
  D  detailed plant, local loop, deployment observations (= arm_deploy)
Usage: python arm_diag.py <policy json> <case H3|E3|S> [kind reference|compact2]
"""
import json, math, sys
from multiprocessing import Pool
from pathlib import Path
import numpy as np
import arm_plant as AP
import mujoco as mj
import arm_task as T
import arm_train_cpu as C
from actuator import NOMINAL, Gains, FrictionMap, LocalController

OUT = Path(__file__).with_name('results')
assert AP.VEL_HZ == 50, 'arm_diag.py builds 20 ms velocity estimates itself; VEL_HZ=1000 is not supported here'


def train_fine(theta, init_idx, kind, compact, h):
    """Training actuator model on the rigid arm at physics step h (command and filter still per 1 ms)."""
    m = mj.MjModel.from_xml_string(T.train_xml(T.NOMINAL_TASK, h=h, compact=compact if kind == 'compact2' else None))
    d = mj.MjData(m)
    q1, q2, px, py = T.INITS[init_idx]
    d.qpos[:] = [q1, q2, px, py]; mj.mj_forward(m, d)
    n = round(1e-3 / h)
    a1 = math.exp(-1e-3 / C.T_REF)
    if kind == 'compact2':
        Ad, Bd = C.ad_bd(compact['wn'], compact['zeta'], 1e-3)
    dly = 4 if kind == 'reference' else int(compact['delay_ms'])
    x = np.zeros((2, 2)); u = np.zeros(2); tail = []
    for k in range(T.N_STEPS):
        obs = T.observe(*d.qpos[:2], *d.qvel[:2], *d.qpos[2:4], *d.qvel[2:4], *u)
        u_prev = u.copy(); u = T.act(theta, obs)
        for s in range(20):
            for j in range(2):
                inp = u_prev[j] if s < dly else u[j]
                if kind == 'reference':
                    d.ctrl[j] = np.clip(x[j, 0], -2, 2); x[j, 0] = a1 * x[j, 0] + (1 - a1) * inp
                else:
                    d.ctrl[j] = np.clip(compact['gain'] * x[j, 0], -2, 2); x[j] = Ad @ x[j] + Bd * inp
            for _ in range(n):
                if kind == 'compact2':
                    for j in range(2):
                        d.qfrc_applied[j] = -compact['coulomb'] * math.tanh(d.qvel[j] / 0.02)
                mj.mj_step(m, d)
        if k >= T.N_STEPS - T.TAIL_STEPS:
            tail.append(T.goal_dist(d.qpos[2], d.qpos[3]))
    return max(tail) < T.SUCCESS_TOL, float(tail[-1]), d.qpos[2:4].copy()


def detailed(theta, init_idx, gains, fmap, kind, mode, seed=0):
    arm = AP.VirtualArm(T.NOMINAL_TASK, NOMINAL, NOMINAL, 25e-6)
    d = arm.d
    q1, q2, px, py = T.INITS[init_idx]
    d.qpos[:] = [q1, q2, q1, q2, px, py]; mj.mj_forward(arm.m, d)
    ctls = [LocalController(kind, Gains(**gains), FrictionMap(**fmap), 'torque', 2 * seed + j) for j in (0, 1)]
    views = [AP.JointView(arm, j) for j in (0, 1)]
    rng = np.random.default_rng(seed); enc = 2 * np.pi / 16384
    qp = pp = None; w = np.zeros(2); v = np.zeros(2); u = np.zeros(2); tail = []
    pend = []; cur = np.zeros(2); terr = []
    a = math.exp(-1e-3 / 0.030); r = np.zeros(2)
    for k in range(T.N_STEPS):
        if mode == 'C':
            qm, pn = d.qpos[2:4].copy(), d.qpos[4:6].copy(); w, v = d.qvel[2:4].copy(), d.qvel[4:6].copy()
        else:
            # D: deployment sensing; E: exact positions, filtered-difference velocities; F: noise on exact velocities
            qm = np.round(d.qpos[2:4] / enc) * enc if mode != 'E' else d.qpos[2:4].copy()
            pn = d.qpos[4:6] + (rng.normal(0, 0.001, 2) if mode != 'E' else 0.0)
            if k:
                w = 0.5 * w + 0.5 * (qm - qp) / T.DT_TASK; v = 0.5 * v + 0.5 * (pn - pp) / T.DT_TASK
            qp, pp = qm, pn
            if mode == 'F':
                w, v = d.qvel[2:4].copy(), d.qvel[4:6].copy()
        u = np.asarray(T.act(theta, T.observe(*qm, *w, *pn, *v, *u)))
        pend.append((k * 20 + 4, u.copy()))
        for j in range(20):
            while pend and pend[0][0] <= k * 20 + j:
                cur = pend.pop(0)[1]
            ic = [ctls[i](cur[i], views[i])[0] for i in (0, 1)]
            sh = np.array([arm.shaft(0), arm.shaft(1)])
            terr.append(sh - r)           # shaft torque minus the 30 ms reference of the delayed command
            r = a * r + (1 - a) * cur
            arm.step_ms(*ic)
        if k >= T.N_STEPS - T.TAIL_STEPS:
            tail.append(T.goal_dist(d.qpos[4], d.qpos[5]))
    terr = np.array(terr)
    return (max(tail) < T.SUCCESS_TOL, float(tail[-1]), d.qpos[4:6].copy(),
            np.sqrt((terr ** 2).mean(0)).tolist(), np.abs(terr).max(0).tolist())


def job(a):
    step, theta, i, kind, gains, fmap, ck, cp = a
    if step == 'A':
        return train_fine(theta, i, kind, cp, 1e-3)
    if step == 'B':
        return train_fine(theta, i, kind, cp, 25e-6)
    return detailed(theta, i, gains, fmap, ck, step)


if __name__ == '__main__':
    pol, case = sys.argv[1], sys.argv[2]
    kind = sys.argv[3] if len(sys.argv) > 3 else 'reference'
    cfg = json.loads((OUT / 'exp2_config.json').read_text())['cfg']
    sel = json.loads((OUT / 'exp5c_damping_wide.json').read_text())
    gains = cfg['gains']['torque']['S'] if case == 'S' else sel[f'torque/{case}']['selected']
    ck = 'S' if case == 'S' else 'H'
    cp = cfg['compact2']['torque']
    thetas = json.loads((OUT / pol).read_text())['theta']
    STEPS = sys.argv[4] if len(sys.argv) > 4 else 'ABCD'
    jobs = [(st, np.array(th), i, kind, gains, cfg['fmap'], ck, cp) for s, th in enumerate(thetas)
            for st in STEPS for i in range(len(T.INITS))]
    with Pool(18) as p:
        res = p.map(job, jobs)
    out = {}
    for (st, _, i, *_), r, s in zip(jobs, res, [j // (len(STEPS) * len(T.INITS)) for j in range(len(jobs))]):
        out.setdefault(f'seed{s}/{st}', []).append(r)
    for k, v in out.items():
        line = f'{k}: success {np.mean([x[0] for x in v]):.2f}, final dist mm ' + ' '.join(f'{1e3 * x[1]:.0f}' for x in v)
        if len(v[0]) > 3:
            line += f'  torque err RMS {np.mean([x[3] for x in v], 0).round(3)} peak {np.max([x[4] for x in v], 0).round(2)}'
        print(line)
