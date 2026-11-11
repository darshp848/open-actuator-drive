"""Size the lift and torso-pitch actuators for OpenArm v2 on a Pixel-style lift/torso.

Rigid inverse dynamics (mj_inverse) on models/mobile_lift_torso.xml with the base held still.
Payload: point mass added 0.12 m along each end-effector's local -z (estimate of grasp point).
Outputs results/lift_torso_sizing/{sizing.json, *.png}.
"""
import json
from pathlib import Path

import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parent
XML = ROOT / 'models' / 'mobile_lift_torso.xml'
OUT = ROOT / 'results' / 'lift_torso_sizing'
G = 9.81


def build(payload):
    spec = mj.MjSpec.from_file(str(XML))
    if payload > 0:
        for side in ('left', 'right'):
            ee = spec.body(f'openarm_{side}_ee_base_link')
            b = ee.add_body(name=f'payload_{side}', pos=[0, 0, -0.12])
            b.add_geom(type=mj.mjtGeom.mjGEOM_SPHERE, size=[0.03, 0, 0], contype=0, conaffinity=0, mass=payload)
    m = spec.compile()
    # sizing wants actuator effort only: no joint-limit/contact/equality forces in mj_inverse
    m.opt.disableflags |= mj.mjtDisableBit.mjDSBL_CONSTRAINT
    return m, mj.MjData(m)


def adr(m, name):
    return m.joint(name).qposadr[0], m.joint(name).dofadr[0]


def set_arms(m, q, j1, j4):
    # mirrored arms: left joint1 range [-3.49, 1.40], right [-1.40, 3.49]
    q[adr(m, 'openarm_left_joint1')[0]] = -j1
    q[adr(m, 'openarm_right_joint1')[0]] = j1
    q[adr(m, 'openarm_left_joint4')[0]] = j4
    q[adr(m, 'openarm_right_joint4')[0]] = j4


def inverse(m, d, q, v, a):
    d.qpos[:] = q; d.qvel[:] = v; d.qacc[:] = a
    mj.mj_inverse(m, d)
    return d.qfrc_inverse.copy()


def minjerk(T, n=600):
    t = np.linspace(0, T, n); s = t / T
    p = 10 * s**3 - 15 * s**4 + 6 * s**5
    v = (30 * s**2 - 60 * s**3 + 30 * s**4) / T
    a = (60 * s - 180 * s**2 + 120 * s**3) / T**2
    return t, p, v, a


def static_grid(payload):
    m, d = build(payload)
    ql, vl = adr(m, 'lift'); qp, vp = adr(m, 'torso_pitch')
    j1s = np.linspace(-1.39, 3.40, 49)   # arm swing about shoulder pitch (sign checked below)
    pitches = np.linspace(0, np.pi / 2, 37)
    out = {}
    for j4 in (0.0, np.pi / 2):
        tau = np.zeros((len(pitches), len(j1s))); force = np.zeros_like(tau)
        for i, p in enumerate(pitches):
            for k, j1 in enumerate(j1s):
                q = np.zeros(m.nq); q[qp] = p; q[ql] = 0.2
                set_arms(m, q, j1, j4)
                f = inverse(m, d, q, np.zeros(m.nv), np.zeros(m.nv))
                tau[i, k] = f[vp]; force[i, k] = f[vl]
        out[j4] = (tau, force)
    return m, pitches, j1s, out


def ee_forward_sign(m, d):
    """Which joint1 sign swings the arms forward (+x)."""
    q = np.zeros(m.nq); set_arms(m, q, 1.0, 0.0); d.qpos[:] = q; mj.mj_forward(m, d)
    return np.sign(d.xpos[m.body('openarm_right_ee_base_link').id][0])


def worst_pose(tau, pitches, j1s):
    i, k = np.unravel_index(np.argmax(np.abs(tau)), tau.shape)
    return pitches[i], j1s[k], tau[i, k]


def bend_motion(payload, j1, j4, T):
    m, d = build(payload)
    qp, vp = adr(m, 'torso_pitch')
    t, p, v, a = minjerk(T)
    tau = []
    for pi, vi, ai in zip(p * 1.57, v * 1.57, a * 1.57):
        q = np.zeros(m.nq); q[adr(m, 'lift')[0]] = 0.2; set_arms(m, q, j1, j4); q[qp] = pi
        vv = np.zeros(m.nv); vv[vp] = vi; aa = np.zeros(m.nv); aa[vp] = ai
        tau.append(inverse(m, d, q, vv, aa)[vp])
    tau = np.array(tau); w = v * 1.57
    return dict(T=T, peak_torque=float(np.max(np.abs(tau))), peak_speed=float(np.max(np.abs(w))),
                peak_power=float(np.max(np.abs(tau * w))), t=t, tau=tau, w=w)


def lift_motion(payload, T, stroke=0.43):
    m, d = build(payload)
    ql, vl = adr(m, 'lift')
    t, p, v, a = minjerk(T)
    f = []
    for pi, vi, ai in zip(p * stroke, v * stroke, a * stroke):
        q = np.zeros(m.nq); q[ql] = pi
        vv = np.zeros(m.nv); vv[vl] = vi; aa = np.zeros(m.nv); aa[vl] = ai
        f.append(inverse(m, d, q, vv, aa)[vl])
    f = np.array(f); s = v * stroke
    return dict(T=T, peak_force=float(np.max(np.abs(f))), peak_speed=float(np.max(np.abs(s))),
                peak_power=float(np.max(np.abs(f * s))), t=t, f=f, s=s)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    m0, d0 = build(0)
    fwd = ee_forward_sign(m0, d0)
    above_pitch = float(sum(m0.body_mass[m0.body('torso').id:]))
    above_lift = float(sum(m0.body_mass[m0.body('lift_carriage').id:]))
    res = dict(model=str(XML.name), forward_sign_joint1=float(fwd), mass_above_pitch_kg=above_pitch,
               mass_on_lift_kg=above_lift, cases={})
    arrays = {}
    for col, payload in enumerate((0.0, 3.0)):
        m, pitches, j1s, grid = static_grid(payload)
        case = dict(payload_per_hand_kg=payload)
        for j4, (tau, force) in grid.items():
            p, j1, t = worst_pose(tau, pitches, j1s)
            key = 'elbow_straight' if j4 == 0 else 'elbow_90'
            case[key] = dict(max_static_pitch_torque_Nm=float(abs(t)), at_pitch_rad=float(p), at_joint1_rad=float(j1),
                             lift_static_force_N=float(np.max(np.abs(force))))
        worst = max(('elbow_straight', 0.0), ('elbow_90', np.pi / 2), key=lambda kv: case[kv[0]]['max_static_pitch_torque_Nm'])
        wj1 = case[worst[0]]['at_joint1_rad']
        case['bend_0_to_90deg'] = [{k: v for k, v in bend_motion(payload, wj1, worst[1], T).items() if not isinstance(v, np.ndarray)}
                                   for T in (2.0, 1.0, 0.6)]
        case['lift_full_stroke'] = [{k: v for k, v in lift_motion(payload, T).items() if not isinstance(v, np.ndarray)}
                                    for T in (5.73, 1.5, 0.8)]
        res['cases'][f'{payload:g}kg'] = case
        arrays[f'grid_tau_straight_{payload:g}'] = grid[0.0][0]
        arrays[f'grid_tau_elbow90_{payload:g}'] = grid[np.pi / 2][0]
        for T in (2.0, 1.0, 0.6):
            r = bend_motion(payload, wj1, worst[1], T)
            arrays[f'bend_t_{payload:g}_{T:g}'] = r['t']; arrays[f'bend_tau_{payload:g}_{T:g}'] = r['tau']
        arrays['pitches'] = pitches; arrays['j1s'] = j1s * fwd
    np.savez(OUT / 'sizing_arrays.npz', **arrays)
    (OUT / 'sizing.json').write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))


if __name__ == '__main__':
    main()
