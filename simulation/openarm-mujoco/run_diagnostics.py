"""Bounded stock-servo bimanual motion and gripper test after mechanics validation."""
import argparse
import json
import time

import mujoco as mj
import numpy as np

import robot


def require_mechanics():
    evidence = json.loads((robot.ROOT / 'results' / 'mechanics.json').read_text())
    assert evidence['passed'] and evidence['mujoco'] == mj.__version__
    for name, expected in evidence['source_sha256'].items():
        assert robot.sha256(robot.ROOT / name) == expected, f'Stale mechanics validation: {name}'
    robot.verify_assets()


def blend(a, b, t, start, duration):
    s = np.clip((t - start) / duration, 0., 1.)
    return a + (b - a) * (10 * s**3 - 15 * s**4 + 6 * s**5)


def run(output, step=0.0005, duration=1.6):
    assert np.isfinite(step) and 0 < step <= .001
    assert np.isfinite(duration) and duration >= 1.6, 'Full gripper observation windows require at least 1.6 s'
    assert abs(duration / .001 - round(duration / .001)) < 1e-9
    assert not output.exists(), 'Refuse to overwrite diagnostic evidence'
    require_mechanics()
    model, data = robot.load(step=step)
    mapping = robot.joint_mapping(model)
    start = data.qpos.copy()
    target = start.copy()
    for side, sign in [('left', -1), ('right', 1)]:
        target[model.joint(f'openarm_{side}_joint1').qposadr[0]] += sign * 0.10
        target[model.joint(f'openarm_{side}_joint4').qposadr[0]] -= 0.15
    dt_control = 0.001
    substeps = round(dt_control / step)
    assert abs(substeps * step - dt_control) < 1e-12
    rows, max_speed_ratio, max_effort_ratio, max_mimic, max_limit_penetration = [], 0., 0., 0., 0.
    max_tracking = 0.
    close_error = 0.
    open_error = 0.
    start_wall = time.perf_counter()
    for tick in range(round(duration / dt_control)):
        t = tick * dt_control
        desired = blend(start, target, t, 0.3, 0.6)
        for side, opening in [('left', .35), ('right', -.35)]:
            finger = blend(opening, 0., t, .35, .35)
            finger = blend(finger, opening, t, .95, .35)
            for number in (1, 2):
                desired[model.joint(f'openarm_{side}_finger_joint{number}').qposadr[0]] = finger
        robot.sync_position_commands(model, data, desired)
        for _ in range(substeps):
            mj.mj_step(model, data)
            assert np.all(np.isfinite(np.r_[data.qpos, data.qvel]))
            assert robot.warning_count(data) == 0, 'MuJoCo solver warning'
            for side in ('left', 'right'):
                a, b = [data.qpos[model.joint(f'openarm_{side}_finger_joint{k}').qposadr[0]] for k in (1, 2)]
                max_mimic = max(max_mimic, abs(a - b))
            for name in [model.joint(i).name for i in range(model.njnt)]:
                joint = model.joint(name)
                entry = dict(joint=name, qpos=joint.qposadr[0], dof=joint.dofadr[0])
                j = model.joint(entry['joint'])
                lower, upper = j.range
                actual = data.qpos[entry['qpos']]
                max_limit_penetration = max(max_limit_penetration, lower - actual, actual - upper)
                velocity_limit = 5. if 'finger' in entry['joint'] else (16.755 if entry['joint'].endswith(('joint1', 'joint2')) else
                                  5.4454 if entry['joint'].endswith(('joint3', 'joint4')) else 20.944)
                max_speed_ratio = max(max_speed_ratio, abs(data.qvel[entry['dof']]) / velocity_limit)
            for entry in mapping:
                force_limit = max(abs(v) for v in entry['output_torque_limits_nm'])
                max_effort_ratio = max(max_effort_ratio, abs(data.actuator_force[entry['actuator']]) / force_limit)
            arm_indices = [model.joint(name).qposadr[0] for name in robot.ARMS]
            max_tracking = max(max_tracking, float(np.max(np.abs(data.qpos[arm_indices] - desired[arm_indices]))))
        for side, opening in [('left', .35), ('right', -.35)]:
            for k in (1, 2):
                actual = data.qpos[model.joint(f'openarm_{side}_finger_joint{k}').qposadr[0]]
                if .90 <= data.time <= .94:
                    close_error = max(close_error, abs(actual))
                if data.time >= 1.5:
                    open_error = max(open_error, abs(actual-opening))
        rows.append(np.r_[data.time, data.qpos, data.qvel, data.ctrl, data.actuator_force])
    output.mkdir(parents=True, exist_ok=False)
    np.savetxt(output / 'motion.csv', rows, delimiter=',',
               header=','.join(['time'] + [f'q_{i}' for i in range(model.nq)] +
                               [f'v_{i}' for i in range(model.nv)] + [f'u_{i}' for i in range(model.nu)] +
                               [f'tau_{i}' for i in range(model.nu)]), comments='')
    passed = bool(abs(data.time - duration) < 1e-9 and max_effort_ratio <= 1.00001 and
                  max_speed_ratio <= 1 and max_mimic < 0.002 and max_limit_penetration < 0.002 and max_tracking < .10 and close_error < .05 and open_error < .05)
    result = dict(passed=passed, stage='stock_servo_motion', mujoco=mj.__version__,
                  command_interface='joint_position_rad_stock_Damiao_PD', requested_duration_s=duration,
                  actual_duration_s=data.time, physics_step_s=step, controller_period_s=dt_control,
                  max_arm_tracking_error_rad=max_tracking,
                  settled_gripper_close_error_rad=close_error, settled_gripper_open_error_rad=open_error, max_speed_ratio=max_speed_ratio,
                  max_effort_ratio=max_effort_ratio, max_finger_mimic_error_rad=max_mimic,
                  max_joint_limit_penetration_rad=max_limit_penetration, solver_warnings=robot.warning_count(data),
                  wall_s=time.perf_counter() - start_wall,
                  source_sha256={name: robot.sha256(robot.ROOT / name) for name in ['robot.py', 'run_diagnostics.py']},
                  hardware_accuracy_qualified=False)
    (output / 'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result))
    assert passed, 'Motion diagnostic failed; retained raw trace must be reviewed'
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=robot.Path, required=True)
    run(parser.parse_args().output.resolve())
