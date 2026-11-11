"""Bounded MATLAB/MuJoCo integration probe. Synthetic two-joint plant, no training."""
import argparse
import hashlib
import json
import platform
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

import actuator
import arm_plant
import arm_task


def digest(file_path):
    return hashlib.sha256(file_path.read_bytes()).hexdigest()


def run(stage, output):
    here = Path(__file__).resolve().parent
    source_hashes = {name: digest(here / name) for name in
                     [Path(__file__).name, 'arm_plant.py', 'arm_task.py', 'actuator.py']}
    arm = arm_plant.VirtualArm(arm_task.NOMINAL_TASK, actuator.NOMINAL,
                               actuator.NOMINAL, h=25e-6)
    q1, q2, _, _ = arm_task.INITS[0]
    tip_x, tip_y = arm_task.tip_xy(q1, q2)
    arm.d.qpos[:] = [q1, q2, q1, q2,
                     tip_x + arm_task.TIP_R + arm_task.PUCK_R - 0.005, tip_y]
    mujoco.mj_forward(arm.m, arm.d)
    metadata = dict(scope='integration_only_synthetic_two_joint_contact',
                    python=platform.python_version(), executable=sys.executable,
                    mujoco=mujoco.__version__, numpy=np.__version__,
                    source_sha256=source_hashes, nq=arm.m.nq,
                    physics_step_s=arm.h, electrical_step_s=actuator.DT_ELEC,
                    local_step_s=actuator.DT_LOCAL, requested_duration_s=0.1,
                    initial_overlap_m=0.005, current_command_a=[0.0, 0.0])
    assert arm.m.nq == 6 and arm.d.ncon > 0
    assert np.all(np.isfinite(np.r_[arm.d.qpos, arm.d.qvel]))
    if stage == 'validate':
        output.mkdir(parents=True, exist_ok=False)
        (output / 'validation.json').write_text(json.dumps(metadata, indent=2))
        print(json.dumps(dict(stage=stage, passed=True, output=str(output))))
        return
    previous = json.loads((output / 'validation.json').read_text())
    assert metadata == previous, 'Model, environment, or diagnostic inputs changed after validation'
    assert not (output / 'result.json').exists(), 'Refusing to overwrite a result'
    initial_puck = arm.d.qpos[4:6].copy()
    max_contacts = arm.d.ncon
    rows = [np.r_[arm.d.time, arm.d.qpos, arm.d.qvel]]
    started = time.perf_counter()
    for _ in range(100):
        arm.step_ms(0.0, 0.0)
        assert arm.ok(), 'Nonfinite or divergent plant state'
        max_contacts = max(max_contacts, arm.d.ncon)
        rows.append(np.r_[arm.d.time, arm.d.qpos, arm.d.qvel])
    wall_s = time.perf_counter() - started
    trajectory = np.asarray(rows)
    displacement = float(np.linalg.norm(arm.d.qpos[4:6] - initial_puck))
    passed = bool(np.all(np.isfinite(trajectory)) and
                  abs(arm.d.time - 0.1) < 1e-10 and max_contacts > 0 and displacement > 1e-8)
    trace_file = output / 'trace.csv'
    np.savetxt(trace_file, trajectory, delimiter=',',
               header='time,r1,r2,q1,q2,puck_x,puck_y,vr1,vr2,vq1,vq2,puck_vx,puck_vy', comments='')
    result = dict(metadata, passed=passed, actual_duration_s=arm.d.time,
                  max_contacts=int(max_contacts), puck_displacement_m=displacement,
                  wall_s=wall_s, trace_sha256=digest(trace_file),
                  accuracy_qualified=False, openarm_model=False)
    (output / 'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result))
    assert passed, 'Integration probe failed; raw output retained'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', choices=['validate', 'solve'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.stage, args.output.resolve())
