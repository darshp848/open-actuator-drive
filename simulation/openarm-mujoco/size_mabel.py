"""Torso-pitch and lift sizing on MABEL's own MJCF (open_mabel commit 2f9229f).

Same method as size_lift_torso.py: rigid mj_inverse, constraints disabled, base still,
hands at their zero pose, payload point mass at each palm body origin.
The model is loaded read-only from ../mabel/open_mabel; nothing there is modified.
"""
import json
from pathlib import Path

import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parent
XML = ROOT.parent / 'mabel' / 'open_mabel' / 'simulation' / 'mabel_mujoco' / 'models' / 'mabel_full.xml'
OUT = ROOT / 'results' / 'mabel_sizing'


def build(payload):
    spec = mj.MjSpec.from_file(str(XML))
    if payload > 0:
        for palm in ('left_palm', 'right_palm'):
            b = spec.body(palm).add_body(name=f'payload_{palm}', pos=[0, 0, 0])
            b.add_geom(type=mj.mjtGeom.mjGEOM_SPHERE, size=[0.03, 0, 0], contype=0, conaffinity=0, mass=payload)
    m = spec.compile()
    m.opt.disableflags |= mj.mjtDisableBit.mjDSBL_CONSTRAINT
    d = mj.MjData(m)
    mj.mj_resetDataKeyframe(m, d, 0)
    return m, d


def jq(m, n):
    return m.joint(n).qposadr[0], m.joint(n).dofadr[0]


def forward_dir(m, d):
    """Unit vector the torso bends toward at its negative limit (MABEL's 'bend' side)."""
    q0 = d.qpos.copy(); qt, _ = jq(m, 'Torso')
    d.qpos[qt] = 0; mj.mj_forward(m, d); a = d.xpos[m.body('torso').id].copy() + 0
    p0 = d.xipos[m.body('torso').id].copy()
    d.qpos[qt] = -0.5; mj.mj_forward(m, d); p1 = d.xipos[m.body('torso').id].copy()
    d.qpos[:] = q0
    v = p1 - p0; v[2] = 0
    return v / np.linalg.norm(v)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    m0, d0 = build(0)
    fwd = forward_dir(m0, d0)
    qt, vt = jq(m0, 'Torso')
    ql, vl = jq(m0, 'Lift Lower'); qu, vu = jq(m0, 'Lift Upper')
    torso_rng = m0.joint('Torso').range
    res = dict(model='open_mabel 2f9229f mabel_full.xml', total_mass_kg=float(sum(m0.body_mass)),
               torso_subtree_kg=float(m0.body_subtreemass[m0.body('torso').id]),
               lift_top_subtree_kg=float(m0.body_subtreemass[m0.body('lift_top').id]),
               bend_direction_world=fwd.round(3).tolist(), cases={})
    arrays = {}
    for payload in (0.0, 3.0):
        m, d = build(payload)
        base_q = d.qpos.copy()
        torso = np.linspace(torso_rng[0], torso_rng[1], 31)
        j1L = np.linspace(*m.joint('Left Arm 1').range, 25)
        best = (0, None); grid = np.zeros((len(torso), len(j1L), 2)); lift = []
        for e, elbow in enumerate((0.0, 0.9)):   # elbow straight / bent (within both arms' ranges by sign)
            for i, t in enumerate(torso):
                for k, a1 in enumerate(j1L):
                    q = base_q.copy(); q[qt] = t
                    q[jq(m, 'Left Arm 1')[0]] = a1
                    q[jq(m, 'Right Arm 1')[0]] = np.clip(-a1, *m.joint('Right Arm 1').range)
                    q[jq(m, 'Left Arm 4')[0]] = elbow
                    q[jq(m, 'Right Arm 4')[0]] = -elbow * 0.86 if elbow else 0.0
                    d.qpos[:] = q; d.qvel[:] = 0; d.qacc[:] = 0
                    mj.mj_inverse(m, d)
                    grid[i, k, e] = d.qfrc_inverse[vt]
                    lift.append(d.qfrc_inverse[vl])
                    if abs(grid[i, k, e]) > abs(best[0]):
                        best = (grid[i, k, e], dict(torso_rad=float(t), left_arm1_rad=float(a1), elbow=float(elbow)))
        hang = np.abs(grid[:, len(j1L) // 2, 0])
        res['cases'][f'{payload:g}kg'] = dict(
            payload_per_hand_kg=payload,
            max_static_torso_torque_Nm=float(abs(best[0])), worst_pose=best[1],
            torso_torque_vs_angle_arm1_mid=dict(zip([f'{np.degrees(t):.0f}deg' for t in torso[::5]], hang[::5].round(1).tolist())),
            lift_lower_stage_static_force_N=float(np.max(np.abs(lift))))
        arrays[f'grid_{payload:g}'] = grid
    arrays['torso'] = torso; arrays['j1L'] = j1L
    np.savez(OUT / 'mabel_arrays.npz', **arrays)
    (OUT / 'sizing.json').write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))


if __name__ == '__main__':
    main()
