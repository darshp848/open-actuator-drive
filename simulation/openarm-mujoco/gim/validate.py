"""Six validation gates for the GIM actuator layer (SPEC.md). Writes results/<timestamp>_validate/{gates.json,gates.png}.
Run from simulation/openarm_v2:  python gim/validate.py [gate numbers]"""
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import mujoco as mj
import numpy as np
from scipy.optimize import brentq

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import robot  # noqa: E402
from actuated import GimArm, Config  # noqa: E402
import scenarios as S  # noqa: E402
import miniplot  # noqa: E402

HERE = Path(__file__).resolve().parent
TYPES = {'GIM8115-36': 0, 'GIM8108-36': 2, 'GIM4310-40': 4}      # left-arm joint index of each motor type


def quiet(arm, gravity=True, freeze_fingers=False):
    m = arm.m
    if freeze_fingers:                                               # fingers rigid and passive: no servo work or sag
        for side in ('left', 'right'):
            for k in (1, 2):
                m.dof_armature[m.joint(f'openarm_{side}_finger_joint{k}').dofadr[0]] += 1e4
        m.actuator_gainprm[arm.fing_act, :3] = 0
        m.actuator_biasprm[arm.fing_act, :3] = 0
    m.opt.disableflags |= int(mj.mjtDisableBit.mjDSBL_CONTACT)
    m.jnt_limited[:] = 0
    if not gravity:
        m.opt.gravity[:] = 0


def gate1(plots):
    """Gravity hold, torque mode, rigid, friction off."""
    out = {}
    for name, eta, eta_hat, fr in [('strict_eta1_etahat1', 1., 1., False), ('eta0.9_etahat0.95_gearloss', .9, .95, True),
                                   ('eta0.9_etahat0.95_nofriction', .9, .95, False)]:
        arm = GimArm(Config(transmission='rigid', mode='torque', friction=fr, eta=eta, eta_hat=eta_hat))
        quiet(arm, freeze_fingers=True)
        lo, hi = arm.m.jnt_range[[arm.m.joint(n).id for n in robot.ARMS]].T
        arm.m.jnt_limited[:] = 1                                   # restore range only to build the pose
        q = lo + 0.4 * (hi - lo)
        arm.m.jnt_limited[:] = 0
        arm.set_arm_q(q)
        arm.d.qacc[:] = 0
        mj.mj_inverse(arm.m, arm.d)
        tau = arm.d.qfrc_inverse[arm.link_dof].copy()
        assert np.allclose(tau, arm.gravity_torque(), atol=1e-6)
        arm.preload(tau)
        for _ in range(1000):
            arm.step_ms({'tau_cmd': tau})
        L = arm.log()
        drift = np.abs(L['q'] - q).max(0)
        out[name] = dict(max_drift_rad=float(drift.max()), worst_joint=robot.ARMS[int(drift.argmax())],
                         max_gravity_torque_nm=float(np.abs(tau).max()))
        plots.append(dict(name=name, t=L['t'], drift=np.abs(L['q'] - q).max(1)))
    out['passed'] = bool(out['strict_eta1_etahat1']['max_drift_rad'] < 1e-3 and out['eta0.9_etahat0.95_gearloss']['max_drift_rad'] < 1e-3)
    out['note'] = ('Gear loss is load-dependent Coulomb friction (1-eta)|tau|, so the realistic case holds inside the '
                   'friction band; without friction eta_hat != 1 over-commands by 1/eta_hat and drifts (reported only).')
    return out


def analytic_speed(arm, i):
    P = arm.P

    def f(w):
        lo, hi, _ = arm.volt_limits(np.full(14, w))
        iq = min(P['i_max'][i], hi[i])
        eff = P['eta'][i]                      # motoring: gear loss (1-eta)|tau| as friction
        return P['N'][i] * P['kt_m'][i] * iq * eff - P['tau_c'][i] - P['b'][i] * w
    w_nl = arm.vmax / (P['pp'][i] * P['lam'][i] * P['N'][i])
    return brentq(f, 1e-6, 1.3 * w_nl, xtol=1e-9), w_nl


def gate2(plots):
    out = {}
    for tname, i in TYPES.items():
        arm = GimArm(Config(transmission='rigid', mode='torque'))
        quiet(arm, gravity=False)
        tau = np.zeros(14)
        tau[i] = 1e4
        for _ in range(1200):
            arm.step_ms({'tau_cmd': tau})
        L = arm.log()
        w = L['v'][:, i]
        w_sim = float(np.mean(w[L['t'] > 1.0]))
        w_an, w_nl = analytic_speed(arm, i)
        flag = bool(L['vlim'][L['t'] > 1.0, i].all())
        out[tname] = dict(joint=robot.ARMS[i], sim_rad_s=w_sim, analytic_rad_s=w_an, ideal_no_load_rad_s=w_nl,
                          error_pct=100 * (w_sim - w_an) / w_an, sim_rpm=w_sim * 30 / np.pi,
                          listed_rpm=float(arm.P['w_max_listed'][i] * 30 / np.pi), voltage_limited=flag,
                          bus_V=arm.cfg.vbus)
        plots.append(dict(name=tname, t=L['t'], w=w, w_an=w_an))
    out['passed'] = bool(all(abs(v['error_pct']) < 2 for v in out.values() if isinstance(v, dict)))
    return out


def gate3(plots):
    out = {}
    for tname, i in TYPES.items():
        arm = GimArm(Config(transmission='rigid', mode='torque', dt=25e-6, delay_ms=0, drive_hz=40e3))
        quiet(arm, gravity=False)
        arm.m.dof_armature[arm.link_dof] += 1e6                    # standstill: no back-EMF
        target = 0.25 * arm.P['i_max'][i]
        tau = np.zeros(14)
        tau[i] = target * arm.etahat_kt[i]
        for _ in range(8):
            arm.step_ms({'tau_cmd': tau})
        L = arm.log()
        t, iq = L['t'], L['iq'][:, i] / target
        k = int(np.argmax(iq >= 1 - np.exp(-1)))
        t63 = t[k - 1] + (t[k] - t[k - 1]) * ((1 - np.exp(-1)) - iq[k - 1]) / (iq[k] - iq[k - 1]) - 25e-6 * 0   # step enters at t=0
        out[tname] = dict(rise_63_us=float(t63 * 1e6), expected_us=float(1e6 / arm.cfg.wc),
                          error_pct=float(100 * (t63 * arm.cfg.wc - 1)), target_A=float(target))
        plots.append(dict(name=tname, t=t, iq=iq))
    out['passed'] = bool(all(abs(v['error_pct']) < 10 for v in out.values() if isinstance(v, dict)))
    return out


def gate4(plots):
    out = {}
    for label, zeta in [('no_gearbox_damping', 0.), ('zeta_0.05', 0.05)]:
        arm = GimArm(Config(transmission='compliant', mode='torque', friction=False, zeta=zeta, vbus=1e6))   # no back-EMF clamp
        quiet(arm)
        arm.m.opt.enableflags |= int(mj.mjtEnableBit.mjENBL_ENERGY)
        arm.m.dof_damping[:] = 0                                    # includes the vendor finger damping
        arm.m.dof_frictionloss[:] = 0
        arm.m.actuator_gainprm[arm.fing_act, :3] = 0                # the finger servos are active elements
        arm.m.actuator_biasprm[arm.fing_act, :3] = 0
        E, T, KE = [], [], []
        for k in range(2000):
            arm.step_ms({'tau_cmd': np.zeros(14)})
            mj.mj_forward(arm.m, arm.d)
            E.append(arm.d.energy[0] + arm.d.energy[1] + arm.spring_energy())
            KE.append(arm.d.energy[1] + arm.spring_energy())
            T.append(arm.d.time)
        E, KE = np.array(E), np.array(KE)
        out[label] = dict(drift_J=float(np.abs(E - E[0]).max()), exchanged_J=float(KE.max()),
                          drift_pct=float(100 * np.abs(E - E[0]).max() / KE.max()), omega_n_dt_max_unsub=float(arm.wn_dt.max()),
                          gear_substeps=arm.n_g)
        plots.append(dict(name=label, t=np.array(T), dE=(E - E[0]) / KE.max() * 100))
    out['passed'] = bool(out['no_gearbox_damping']['drift_pct'] < 0.5)
    out['note'] = ('Gate uses the undamped gearbox (c_g = 0); with zeta = 0.05 the damper dissipates energy by design. '
                   'Drift is relative to the peak kinetic + gearbox spring energy reached (energy actually exchanged).')
    return out


def gate5(plots):
    i = 4
    arm = GimArm(Config(transmission='compliant', mode='torque'))
    quiet(arm, gravity=False)
    arm.m.dof_armature[arm.link_dof] += 1e4                          # held link
    h = arm.cfg.backlash
    amp = arm.P['k_g'][i] * (h + 0.003)
    f = 0.25
    for k in range(8000):
        tau = np.zeros(14)
        tau[i] = amp * np.sin(2 * np.pi * f * k * 1e-3)
        arm.step_ms({'tau_cmd': tau})
    L = arm.log()
    x, tg = L['theta_r'][:, i] - L['q'][:, i], L['tau_gear'][:, i]
    sel = L['t'] > 1 / f                                              # skip the first cycle
    big = np.abs(tg) > 0.6 * np.abs(tg[sel]).max()
    br = {}
    for s in (1, -1):
        m_ = sel & big & (np.sign(tg) == s)
        A = np.c_[np.ones(m_.sum()), tg[m_]]
        br[s] = np.linalg.lstsq(A, x[m_], rcond=None)[0]
    width = br[1][0] - br[-1][0]
    plots.append(dict(name='backlash', x=x, tg=tg))
    out = dict(configured_arcmin=h * 60 * 180 / np.pi, measured_arcmin=float(width * 60 * 180 / np.pi),
               error_pct=float(100 * (width / h - 1)), fitted_stiffness_Nm_per_rad=float(1 / br[1][1]),
               configured_stiffness=float(arm.P['k_g'][i]), joint=robot.ARMS[i])
    out['passed'] = bool(abs(out['error_pct']) < 5)
    return out


def gate6(plots):
    out = {}
    for tr in ('rigid', 'compliant'):
        runs = {dt: S.run_gim(Config(transmission=tr, dt=dt)) for dt in (0.5e-3, 0.25e-3, 0.125e-3)}
        ks = list(runs)
        d1 = S.rms_diff(runs[ks[0]], runs[ks[1]])
        d2 = S.rms_diff(runs[ks[1]], runs[ks[2]])
        out[tr] = dict(rms_05_vs_025_ms_rad=d1, rms_025_vs_0125_ms_rad=d2,
                       max_tracking_error_by_dt_rad=[float(np.abs(r['q'] - r['qdes']).max()) for r in runs.values()],
                       gear_substeps_at_0p5ms=runs[ks[0]]['arm'].n_g,
                       omega_n_dt_unsub_max=float(runs[ks[0]]['arm'].wn_dt.max()),
                       passed=bool(d1 < 0.005 and d2 < 0.005))
        plots.append(dict(name=tr, t=runs[ks[0]]['t'], q=[r['q'] for r in runs.values()]))
    out['passed'] = bool(all(out[t]['passed'] for t in ('rigid', 'compliant')))
    return out


def figure(path, P):
    pan = []
    if 1 in P:
        pan.append(dict(title='G1 gravity hold: max |q - q0| (rad)', xlabel='t (s)', ylabel='rad', hline=[1e-3],
                        series=[(d['name'], d['t'], d['drift']) for d in P[1]]))
    if 2 in P:
        pan.append(dict(title='G2 terminal speed (rad/s), dashed = analytic', xlabel='t (s)', ylabel='rad/s',
                        series=[(d['name'], d['t'], d['w']) for d in P[2]] +
                               [(d['name'] + ' analytic', [0, d['t'][-1]], [d['w_an']] * 2) for d in P[2]]))
    if 3 in P:
        pan.append(dict(title='G3 current step, iq / target', xlabel='t (s)', ylabel='-', hline=[1 - np.exp(-1)],
                        series=[(d['name'], d['t'], d['iq']) for d in P[3]]))
    if 4 in P:
        pan.append(dict(title='G4 energy drift (% of exchanged)', xlabel='t (s)', ylabel='%',
                        series=[(d['name'], d['t'], d['dE']) for d in P[4]]))
    if 5 in P:
        d = P[5][0]
        pan.append(dict(title='G5 backlash: theta_r - q (rad) vs gear torque', xlabel='tau_gear (Nm)', ylabel='rad',
                        series=[('x(tau)', d['tg'], d['x'])]))
    if 6 in P:
        for d in P[6]:
            pan.append(dict(title=f"G6 {d['name']}: J4 right q (rad) at 0.5/0.25/0.125 ms", xlabel='t (s)', ylabel='rad',
                            series=[(f'dt{k}', d['t'][::round(1e-3 / (d['t'][1] - d['t'][0]))] if False else
                                     np.linspace(0, 1.6, len(q))[::max(1, len(q) // 800)], q[:, 10][::max(1, len(q) // 800)])
                                    for k, q in zip(('0.5', '0.25', '0.125'), d['q'])]))
    miniplot.plot(path, pan, ncols=2)


if __name__ == '__main__':
    which = [int(a) for a in sys.argv[1:]] or [1, 2, 3, 4, 5, 6]
    out = HERE / 'results' / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_validate')
    out.mkdir(parents=True, exist_ok=False)
    res, plots = {}, {}
    for g in which:
        t0 = time.perf_counter()
        plots[g] = []
        res[f'gate{g}'] = globals()[f'gate{g}'](plots[g])
        res[f'gate{g}']['wall_s'] = time.perf_counter() - t0
        print(f'gate {g}:', 'PASS' if res[f'gate{g}']['passed'] else 'FAIL', json.dumps(res[f'gate{g}'], default=float), flush=True)
    res['mujoco'] = mj.__version__
    (out / 'gates.json').write_text(json.dumps(res, indent=2, default=float))
    figure(out / 'gates.png', plots)
    print('wrote', out)
