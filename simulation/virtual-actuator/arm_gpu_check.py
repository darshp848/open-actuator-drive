"""Checks for arm_gpu_es.py. Run with ..\\.venv-warp\\Scripts\\python.exe (it imports the CPU reference
arm_train_cpu.py, which only needs mujoco 3.x + numpy, so a single env is enough).

  python arm_gpu_check.py friction   # (0) per-world dof_frictionloss / body_mass take effect (analytic + CPU)
  python arm_gpu_check.py agree      # (a) GPU vs CPU agreement, both kinds, nominal + randomised, contact monitored
  python arm_gpu_check.py speed      # (b) throughput at pairs=256, 1 and 8 seeds
  python arm_gpu_check.py sanity     # (c) 1 seed reference nominal 256 pairs 100 gens (also written by arm_gpu_es CLI)
Writes results/arm_gpu_check_<mode>.json.
"""
import json, sys, time
from pathlib import Path
import numpy as np
import mujoco as mj
import warp as wp
import arm_task as T
import arm_gpu_es as G
import arm_train_cpu as C

OUT = Path(__file__).with_name('results')
CFG = json.loads((OUT / 'exp2_config.json').read_text())['cfg']
CP = CFG['compact2']['torque']
NI = G.NI


def pushing_thetas(kind):
    """Constant-output policies (W=0, b2 = atanh of constant torque requests), picked by a CPU grid search so that the
    tip touches and moves the puck in all three initial states; returns list of thetas."""
    cp = CP if kind == 'compact2' else None
    grid = np.arctanh(np.array([-0.9, -0.5, -0.2, 0.2, 0.5, 0.9]))
    best = []
    for b1 in grid:
        for b2 in grid:
            th = np.zeros(T.P); th[544] = b1; th[545] = b2
            moved = 0.0; touch = 0
            for i in range(NI):
                r, s, tj, info = C.episode(kind, th, i, T.NOMINAL_TASK, None, cp, record_state=True)
                disp = float(np.hypot(*(tj[-1] - np.array(T.INITS[i][2:]))))
                moved += disp; touch += info['contact_substeps'] > 0
            best.append((touch, moved, th))
    best.sort(key=lambda t: (-t[0], -t[1]))
    return [best[0][2], best[1][2]], [(float(b1), float(b2)) for b1, b2 in [(t[2][544], t[2][545]) for t in best[:2]]], [(t[0], t[1]) for t in best[:2]]


def check_friction():
    """Puck launched at 1 m/s (+x) far from the tip; stopping distance must be v^2/(2 mu g), independent of puck mass."""
    cases = [(0.15, 0.2, 0.2), (0.45, 0.2, 0.2), (0.30, 0.1, 0.2), (0.30, 0.4, 0.2), (0.30, 0.2, 0.0), (0.30, 0.2, 0.5)]
    n = len(cases)
    tr = G.ArmGpuTrainer('reference', n, monitor=0)
    mu = np.array([c[0] for c in cases]); pm = np.array([c[1] for c in cases]); pl = np.array([c[2] for c in cases])
    tr.set_world_params(pl, pm, mu, np.full(n, 4))
    q0 = np.tile([1.2, -1.0, 0.0, -0.3], (n, 1))
    tr.params.assign(np.zeros((n, T.P), np.float32)); tr.reset(q0)
    v = np.zeros((n, 4), np.float32); v[:, 2] = 1.0
    tr.d.qvel.assign(v)
    for _ in range(100):                     # 2 s
        wp.capture_launch(tr.graph)
    wp.synchronize()
    gx = tr.d.qpos.numpy()[:, 2]
    rows = []
    for c, g in zip(cases, gx):
        m = mj.MjModel.from_xml_string(T.train_xml(T.Task(payload=c[2], puck_mass=c[1], puck_mu=c[0])))
        d = mj.MjData(m); d.qpos[:] = [1.2, -1.0, 0.0, -0.3]; d.qvel[2] = 1.0
        for _ in range(2000):
            mj.mj_step(m, d)
        rows.append(dict(mu=c[0], puck_mass=c[1], payload=c[2], gpu_x=float(g), cpu_x=float(d.qpos[2]),
                         analytic=float(1.0 / (2 * c[0] * 9.81) - 0.0)))
    for r in rows:
        print(r)
    (OUT / 'arm_gpu_check_friction.json').write_text(json.dumps(rows, indent=1))
    return rows


def check_agree():
    res = {}
    rng = np.random.default_rng(123)
    rand_thetas = [rng.normal(0, 0.3, T.P) for _ in range(4)]
    settings = {
        'nominal': dict(task=T.NOMINAL_TASK, delay=None),
        'random': dict(task=T.Task(payload=0.41, puck_mass=0.13, puck_mu=0.42), delay=7),
    }
    for kind in ('reference', 'compact2'):
      cp = CP if kind == 'compact2' else None
      push, push_b, push_stats = pushing_thetas(kind)
      for nmode in ('zero', 'copied', 'exact'):
            thetas = [('rand%d' % k, t) for k, t in enumerate(rand_thetas)] + [('push%d' % k, t) for k, t in enumerate(push)]
            cases = [(tn, th, i, sn) for tn, th in thetas for sn in settings for i in range(NI)]
            n = len(cases)
            tr = G.ArmGpuTrainer(kind, n, cp, monitor=2, sensing='exact' if nmode == 'exact' else 'est')
            tr.set_noise_sd(0.0 if nmode == 'zero' else 0.001)
            tr.set_noise(123, np.arange(n))
            pay = np.array([settings[c[3]]['task'].payload for c in cases]); pm = np.array([settings[c[3]]['task'].puck_mass for c in cases])
            mu = np.array([settings[c[3]]['task'].puck_mu for c in cases])
            dl = np.array([settings[c[3]]['delay'] if settings[c[3]]['delay'] is not None else tr.d_nom for c in cases])
            tr.set_world_params(pay, pm, mu, dl)
            params = np.stack([c[1] for c in cases])
            q0 = np.array([T.INITS[c[2]] for c in cases])
            ret, succ, traj = tr.evaluate(params, q0, record=True)
            nlog = tr.noise_log.numpy()
            mon = tr.monitor_report()
            rows = []
            touched_w = tr.touched.numpy(); cmax_w = tr.ncon_max.numpy()
            for w, (tn, th, i, sn) in enumerate(cases):
                st = settings[sn]
                r, s, tj, info = C.episode(kind, th, i, st['task'], st['delay'], cp, record_state=True,
                                           sensing='exact' if nmode == 'exact' else 'est', noise=nlog[w] if nmode == 'copied' else None)
                gp = traj[w, :, 2:4]
                rows.append(dict(theta=tn, setting=sn, init=i, ret_gpu=float(ret[w]), ret_cpu=float(r),
                                 rel_err=float(abs(ret[w] - r) / max(abs(r), 1e-9)),
                                 puck_final_gpu=gp[-1].tolist(), puck_final_cpu=tj[-1].tolist(),
                                 puck_final_err=float(np.hypot(*(gp[-1] - tj[-1]))),
                                 puck_traj_maxerr=float(np.abs(gp - tj).max()),
                                 succ_gpu=bool(succ[w]), succ_cpu=bool(s),
                                 gpu_contact=bool(touched_w[w]), gpu_max_contacts=int(cmax_w[w]),
                                 cpu_contact=bool(info['contact_substeps'] > 0),
                                 puck_moved=float(np.hypot(*(tj[-1] - np.array(T.INITS[i][2:]))))))
            res[f'{kind}/{nmode}'] = dict(rows=rows, monitor=mon, push_bias=push_b, push_search=push_stats)
            print('=== ', kind, nmode, mon, 'push b2/(b1,b2)=', push_b)
            for lab, sel in (('no-contact', [r for r in rows if not r['cpu_contact']]), ('contact', [r for r in rows if r['cpu_contact']])):
                if sel:
                    print(f"  {lab:10s} n={len(sel):2d} max rel ret err {max(r['rel_err'] for r in sel):.1e} median {np.median([r['rel_err'] for r in sel]):.1e}"
                          f" max puck final err {max(r['puck_final_err'] for r in sel):.1e} median {np.median([r['puck_final_err'] for r in sel]):.1e}"
                          f" flags g!=c contact {sum(r['gpu_contact'] != r['cpu_contact'] for r in sel)} succ {sum(r['succ_gpu'] != r['succ_cpu'] for r in sel)}", flush=True)
    (OUT / 'arm_gpu_check_agree.json').write_text(json.dumps(res, indent=1))
    return res


def check_speed():
    res = {}
    for kind in ('reference', 'compact2'):
        cp = CP if kind == 'compact2' else None
        for ns in (1, 8):
            t0 = time.perf_counter()
            r = G.run(kind, cp, list(range(ns)), 256, 10, randomise=True, log=lambda *a: None)
            tot = time.perf_counter() - t0
            gen_s = r['wall_s'] / 10
            res[f'{kind}_s{ns}'] = dict(nworld=r['nworld'], s_per_gen=gen_s, env_steps_per_s=r['nworld'] * G.N_BLOCKS * 20 / gen_s,
                                        total_incl_setup_s=tot, monitor=r['config']['monitor'])
            print(kind, ns, res[f'{kind}_s{ns}'], flush=True)
    (OUT / 'arm_gpu_check_speed.json').write_text(json.dumps(res, indent=1))
    return res


if __name__ == '__main__':
    mode = sys.argv[1]
    {'friction': check_friction, 'agree': check_agree, 'speed': check_speed}[mode]()
