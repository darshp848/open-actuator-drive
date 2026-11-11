"""Bounded contact diagnostics; no claim of calibrated real-world contact."""
import argparse
import json
import mujoco as mj
import numpy as np
import robot
from run_diagnostics import require_mechanics, blend


def episode(kind, step, output):
    if kind == 'clamp':
        path = robot.ROOT / 'models/gripper_fixture.xml'
        evidence = json.loads((robot.ROOT / 'results/fixture.json').read_text())
        assert evidence['compiled'] and robot.sha256(path) == evidence['source_sha256']
        model = mj.MjModel.from_xml_path(str(path))
        data = mj.MjData(model)
        mj.mj_resetDataKeyframe(model, data, 0)
        object_id = model.geom('clamp_probe').id
        duration = 1.
    else:
        model, data = robot.load(contact=True)
        object_id = model.geom('puck_geom').id
        # Explicit diagnostic hard-table assumption; vendor assets remain untouched.
        for geom_name in ('table_top', 'puck_geom'):
            model.geom(geom_name).solref[:] = [.002, 1.]
        puck_q = int(model.jnt_qposadr[model.body('puck').jntadr[0]])
        data.qpos[puck_q + 2] += .04
        duration = .5
    model.opt.timestep = step
    mj.mj_forward(model, data)
    desired = data.qpos.copy()
    rows, contact_rows, touched = [], [], set()
    impulse, penetration, mimic = 0., 0., 0.
    substeps = round(.001 / step)
    for tick in range(round(duration / .001)):
        if kind == 'clamp':
            angle = blend(-.35, 0., tick * .001, .1, .3)
            angle = blend(angle, -.35, tick * .001, .6, .3)
            for finger in (1, 2):
                desired[model.joint(f'openarm_right_finger_joint{finger}').qposadr[0]] = angle
        robot.sync_position_commands(model, data, desired)
        for _ in range(substeps):
            # Forces/contact geometry from mj_step correspond to the pre-integration state.
            mj.mj_step(model, data)
            force = 0.
            sample_penetration = 0.
            for i, contact in enumerate(data.contact):
                if object_id in (contact.geom1, contact.geom2):
                    wrench = np.zeros(6)
                    mj.mj_contactForce(model, data, i, wrench)
                    force += max(0., wrench[0])
                    penetration = max(penetration, -float(contact.dist))
                    sample_penetration = max(sample_penetration, -float(contact.dist))
                    other = contact.geom2 if contact.geom1 == object_id else contact.geom1
                    if wrench[0] > 1e-6:
                        touched.add(model.geom(other).name)
            impulse += force * step
            contact_rows.append([data.time-step, force, sample_penetration])
            for side in ('left', 'right'):
                a, b = [data.qpos[model.joint(f'openarm_{side}_finger_joint{k}').qposadr[0]] for k in (1, 2)]
                mimic = max(mimic, abs(a-b))
            assert np.isfinite(np.r_[data.qpos, data.qvel, force]).all()
            assert robot.warning_count(data) == 0
        rows.append(np.r_[data.time, data.qpos])
    trace = np.array(rows)
    np.savetxt(output / f'{kind}_{step:g}.csv', trace, delimiter=',')
    np.savetxt(output / f'{kind}_{step:g}_contact.csv', contact_rows, delimiter=',',
               header='pre_step_time_s,total_object_normal_force_n,max_penetration_m', comments='')
    contact_ok = (any('finger_inner_right' in n for n in touched) and any('finger_outer_right' in n for n in touched)) if kind == 'clamp' else 'table_top' in touched
    passed = bool(contact_ok and impulse > 0 and penetration < .001 and mimic < .003 and abs(data.time-duration) < 1e-9)
    if kind == 'drop':
        passed = passed and abs(data.qpos[puck_q+2] - .422) < .001
    result = dict(kind=kind, physics_step_s=step, actual_duration_s=data.time, passed=bool(passed),
                  impulse_ns=impulse, max_penetration_m=penetration, max_mimic_error_rad=mimic,
                  touched=sorted(touched), warnings=robot.warning_count(data))
    return result, trace


def run(output):
    require_mechanics()
    output.mkdir(parents=True, exist_ok=False)
    results, comparisons = [], []
    for kind in ('clamp', 'drop'):
        previous = None
        for step in (.0005, .00025, .000125):
            result, trace = episode(kind, step, output)
            results.append(result)
            if previous is not None:
                old, old_trace = previous
                impulse_error = abs(result['impulse_ns']-old['impulse_ns']) / max(result['impulse_ns'], 1e-12)
                joint_rms = float(np.sqrt(np.mean((trace[:, 1:19]-old_trace[:, 1:19])**2)))
                penetration_delta = abs(result['max_penetration_m']-old['max_penetration_m'])
                object_rms = float(np.sqrt(np.mean((trace[:, 19:22]-old_trace[:, 19:22])**2))) if kind == 'drop' else 0.
                comparisons.append(dict(kind=kind, fine_step_s=step, impulse_relative_error=impulse_error,
                    joint_rms_rad=joint_rms, object_position_rms_m=object_rms, penetration_delta_m=penetration_delta,
                    passed=bool(impulse_error < .05 and joint_rms < .005 and object_rms < .001 and penetration_delta < .0002)))
            previous = result, trace
    report = dict(passed=all(r['passed'] for r in results+comparisons), episodes=results,
                  convergence=comparisons, controller_period_s=.001, gravity_feedforward=False,
                  drop_table_and_puck_solref=[.002, 1.],
                  hardware_accuracy_qualified=False,
                  source_sha256={n: robot.sha256(robot.ROOT/n) for n in ['run_contacts.py','models/gripper_fixture.xml']})
    (output/'result.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report))
    assert report['passed'], 'Contact diagnostic failed; inspect retained traces before changes'
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=robot.Path, required=True)
    run(parser.parse_args().output.resolve())
