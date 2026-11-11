"""Sanity checks of the two-joint detailed plant (arm_plant.py). Writes results/arm_plant_check.json.
 a. torque interface on the free arm (shaft torque vs first-order 30 ms reference of the request)
 b. sim-to-sim gap: same open-loop 50 Hz request sequence on detailed (25 us, H3) and train_xml (1 ms, reference actuator)
 c. wall time of one deploy() episode; smoke test over all 53 plants
"""
import json, math, time
from dataclasses import replace
from multiprocessing import Pool
from pathlib import Path
import numpy as np
import arm_plant as AP
import arm_task as T
import mujoco as mj
from actuator import NOMINAL, HELD_OUT, Gains, FrictionMap, LocalController

OUT = Path(__file__).with_name('results')
CFG = json.loads((OUT / 'exp2_config.json').read_text())['cfg']
SEL = json.loads((OUT / 'exp5c_damping_wide.json').read_text())
G_H3 = SEL['torque/H3']['selected']
G_S = CFG['gains']['torque']['S']
FMAP = CFG['fmap']
A_REF = math.exp(-1e-3 / 0.030)


def _arm(p, h=25e-6, init=0, puck=None, task=T.NOMINAL_TASK):
    arm = AP.VirtualArm(task, p, p, h)
    q1, q2, px, py = T.INITS[init]
    if puck is not None:
        px, py = puck
    arm.d.qpos[:] = [q1, q2, q1, q2, px, py]
    mj.mj_forward(arm.m, arm.d)
    return arm


def _ctls(arm, kind, gains, seed=0):
    ctls = [LocalController(kind, Gains(**gains), FrictionMap(**FMAP), 'torque', 2 * seed + j) for j in (0, 1)]
    return ctls, [AP.JointView(arm, j) for j in (0, 1)]


def torque_step(p, kind, gains):
    arm = _arm(p, puck=(2.0, 2.0))
    ctls, views = _ctls(arm, kind, gains)
    n = 1000
    c = np.zeros((n, 2)); c[:500, 0] = 0.5; c[500:, 0] = -0.5
    dly = p.delay_ms
    cd = np.vstack([np.zeros((dly, 2)), c[:n - dly]])
    r = np.zeros(2); ref = np.zeros((n, 2)); sh = np.zeros((n, 2)); ok = True
    for k in range(n):
        r = A_REF * r + (1 - A_REF) * cd[k]
        ref[k] = r
        i1, _ = ctls[0](cd[k, 0], views[0]); i2, _ = ctls[1](cd[k, 1], views[1])
        arm.step_ms(i1, i2)
        sh[k] = [arm.shaft(0), arm.shaft(1)]
        if not arm.ok():
            ok = False; break
    e = sh - ref
    er = sh - cd                      # error against the (delayed) request itself, no reference filter
    # stable: finite, |shaft| < 2 Nm and no sustained oscillation (std < 0.1 Nm) over the last 100 ms of each half
    # (steady-state offsets are not part of this test; S has ki = 0 and keeps a friction-dependent offset)
    seg = [slice(400, 500), slice(900, 1000)]
    settled = all(np.all(np.std(sh[s_], 0) < 0.1) for s_ in seg) and np.abs(sh).max() < 2.0
    stable = bool(ok and np.all(np.isfinite(e)) and settled)
    return dict(ok=bool(ok), stable=stable,
                rms_vs_request=[float(np.sqrt(np.mean(er[:, j] ** 2))) for j in (0, 1)],
                peak_vs_request=[float(np.abs(er[:, j]).max()) for j in (0, 1)],
                rms=[float(np.sqrt(np.mean(e[:, j] ** 2))) for j in (0, 1)],
                peak=[float(np.abs(e[:, j]).max()) for j in (0, 1)],
                peak_shaft=[float(np.abs(sh[:, j]).max()) for j in (0, 1)])


def open_loop_requests(f=1.5, a1=-0.4, a2=-0.4):
    """One sine period at f Hz on both joints, then zero (2 s at 50 Hz). Chosen on the training model so that the
    tip reaches the puck and pushes it ~10 cm; one full period returns the free-arm velocity to ~0, so the arm
    stays in the workspace (a free-running sinusoid spins the arm several radians, which is chaotic)."""
    t = np.arange(100) * T.DT_TASK
    env = t < 1.0 / f
    return np.stack([a1 * np.sin(2 * np.pi * f * t) * env, a2 * np.sin(2 * np.pi * f * t) * env], 1)


def detailed_traj(U, h=25e-6, kind='H', gains=G_H3):
    p = NOMINAL
    arm = _ctl_arm = _arm(p, h)
    ctls, views = _ctls(arm, kind, gains)
    d = arm.d
    traj = []
    pend = []; cur = np.zeros(2)
    for k, u in enumerate(U):
        pend.append((k * 20 + p.delay_ms, u))
        for j in range(20):
            tm = k * 20 + j
            while pend and pend[0][0] <= tm:
                cur = pend.pop(0)[1]
            i1, _ = ctls[0](cur[0], views[0]); i2, _ = ctls[1](cur[1], views[1])
            arm.step_ms(i1, i2)
            tx, ty = arm.tip()
            traj.append([tx, ty, d.qpos[4], d.qpos[5]])
    return np.array(traj)


def train_traj(U, task=T.NOMINAL_TASK):
    m = mj.MjModel.from_xml_string(T.train_xml(task, 1e-3)); d = mj.MjData(m)
    q1, q2, px, py = T.INITS[0]
    d.qpos[:] = [q1, q2, px, py]; mj.mj_forward(m, d)
    x = np.zeros(2); buf = [np.zeros(2)] * 4
    traj = []
    for u in U:
        for _ in range(20):
            buf.append(u); ui = buf.pop(0)
            d.ctrl[:] = np.clip(x, -2.0, 2.0)
            x = A_REF * x + (1 - A_REF) * ui
            mj.mj_step(m, d)
            tx, ty = T.tip_xy(float(d.qpos[0]), float(d.qpos[1]))
            traj.append([tx, ty, d.qpos[2], d.qpos[3]])
    return np.array(traj)


def compare(a, b):
    dt = np.hypot(a[:, 0] - b[:, 0], a[:, 1] - b[:, 1]); dp = np.hypot(a[:, 2] - b[:, 2], a[:, 3] - b[:, 3])
    cmin = lambda x: float(np.hypot(x[:, 0] - x[:, 2], x[:, 1] - x[:, 3]).min())
    disp = lambda x: float(np.hypot(x[-1, 2] - x[0, 2], x[-1, 3] - x[0, 3]))
    return dict(tip_max=float(dt.max()), tip_rms=float(np.sqrt(np.mean(dt ** 2))),
                puck_max=float(dp.max()), puck_rms=float(np.sqrt(np.mean(dp ** 2))),
                contact_A=bool(cmin(a) < T.TIP_R + T.PUCK_R), contact_B=bool(cmin(b) < T.TIP_R + T.PUCK_R),
                min_tip_puck_A=cmin(a), min_tip_puck_B=cmin(b), puck_disp_A=disp(a), puck_disp_B=disp(b),
                puck_final_A=a[-1, 2:].tolist(), puck_final_B=b[-1, 2:].tolist())


def smoke(job):
    return AP.deploy(job)


if __name__ == '__main__':
    res = {}
    # ---- a
    plants = {'nominal': NOMINAL, **{p.name: p for p in HELD_OUT if p.name in ('soft_gearbox', 'backlash')}}
    res['a_torque_interface'] = {}
    for pn, p in plants.items():
        for lab, kind, g in (('H3', 'H', G_H3), ('S', 'S', G_S)):
            r = torque_step(p, kind, g)
            res['a_torque_interface'][f'{pn}/{lab}'] = r
            print(f'(a) {pn:13s} {lab:2s} stable={r["stable"]} rms=[{r["rms"][0]:.4f},{r["rms"][1]:.4f}] '
                  f'peak=[{r["peak"][0]:.4f},{r["peak"][1]:.4f}] vs-raw-request rms=[{r["rms_vs_request"][0]:.4f},{r["rms_vs_request"][1]:.4f}]  peak|shaft|={np.round(r["peak_shaft"], 3).tolist()}', flush=True)
    # ---- b
    res['b_sim2sim'] = {}
    for sname, kw in (('seqA_1.5Hz_-0.4_-0.4', dict(f=1.5, a1=-0.4, a2=-0.4)), ('seqB_1.0Hz_-0.2_-0.4', dict(f=1.0, a1=-0.2, a2=-0.4))):
      U = open_loop_requests(**kw)
      ref = train_traj(U)
      for lab, h in (('25us', 25e-6), ('100us', 100e-6), ('1ms', 1e-3)):
        t0 = time.perf_counter()
        det = detailed_traj(U, h)
        c = compare(det, ref); c['wall_s'] = time.perf_counter() - t0
        res['b_sim2sim'][f'{sname}/detailed_{lab}_vs_train_1ms'] = c
        print(f'(b) {sname} detailed {lab} vs train: tip max {c["tip_max"]*1e3:.1f} mm rms {c["tip_rms"]*1e3:.1f} mm | puck max '
              f'{c["puck_max"]*1e3:.1f} mm rms {c["puck_rms"]*1e3:.1f} mm | contact det/train {c["contact_A"]}/{c["contact_B"]} '
              f'min d {c["min_tip_puck_A"]*1e3:.1f}/{c["min_tip_puck_B"]*1e3:.1f} mm, puck disp {c["puck_disp_A"]*1e3:.0f}/{c["puck_disp_B"]*1e3:.0f} mm '
              f'({c["wall_s"]:.1f} s)', flush=True)
    # ---- c
    th = np.random.default_rng(0).normal(0, 0.3, T.P)
    job = ('nominal', T.NOMINAL_TASK, NOMINAL, NOMINAL, 'H', G_H3, FMAP, th, 0, 0)
    t0 = time.perf_counter(); r = AP.deploy(job); wall = time.perf_counter() - t0
    res['c_wall_one_episode'] = dict(wall_s=wall, result=r)
    print(f'(c) one deploy episode: {wall:.1f} s  {r}', flush=True)
    jobs = [(n, tk, p1, p2, 'H', G_H3, FMAP, th, 0, 0) for _, n, tk, p1, p2 in AP.all_plants()]
    t0 = time.perf_counter()
    with Pool(18) as pool:
        sm = pool.map(smoke, jobs, chunksize=1)
    sw = time.perf_counter() - t0
    bad = [s for s in sm if (not s['ok']) or not np.isfinite([s['ret'], s['final_dist']]).all()]
    res['smoke'] = dict(n=len(sm), wall_s=sw, not_ok=[s['plant'] for s in bad], rows=sm)
    print(f'smoke: {len(sm)} plants in {sw:.0f} s, ok=False or NaN: {[s["plant"] for s in bad]}', flush=True)
    (OUT / 'arm_plant_check.json').write_text(json.dumps(res, indent=2, default=float))
