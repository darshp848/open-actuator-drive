# OpenArm v2 upper-body numerical baseline

The actual bimanual OpenArm v2 model runs headlessly in MuJoCo 3.6.0, with
MATLAB R2026a launching complete jobs. There are 14 arm joints and two grippers:
18 coordinates, 16 independent position commands, and two finger mimic constraints.
The torso is fixed. This is the stock Damiao position-servo baseline, not a
custom GIM torque-controlled robot or a calibrated hardware digital twin.

In MATLAB, set Current Folder to this directory and run:

```matlab
[result, result_dir] = run_openarm_v2();
```

Each run creates a timestamped `results/matlab_*` directory. Four separate
process stages validate mechanics, run motion, check contact convergence, and
render `preview.png`. A failed stage stops MATLAB and retains process logs;
only a complete successful run writes the final `result.json`.
The batch does not change `pyenv` or the existing electrical Simulink model.

The separate environment is `../.venv-openarm`, Python 3.11.9. Direct
requirements are in `requirements.txt`; `requirements-lock.txt` records all
installed package versions. Vendor data is pinned to Enactic commit
`06b4c8ad6d547c3faaa3bf780629ee3e1ac2b0d0`, with hashes in
`assets-manifest.json`. `fetch_assets.py` restores those data assets if needed.
Vendor license is retained under `vendor/openarm_mujoco/LICENSE`.

## Reviewed checks

- Independent URDF Jacobian dynamics and body poses at 13 deterministic poses:
  worst gravity difference 7.1e-6 Nm, mass-matrix relative difference 6.4e-7.
  Armature is accounted for separately. This proves source consistency, not
  that either source accurately describes the hardware.
- A 1.6 s bimanual motion test checks every moving coordinate at every physics
  step, actuator effort, finite state, solver warnings, tracking, and actual
  gripper closing/reopening. Maximum arm tracking error is about 0.030 rad.
- Fixed-sphere clamp contacts both right fingers. A separate 40 mm puck drop
  contacts the table. Physics steps 0.5/0.25/0.125 ms share a 1 ms command clock.
  Joint/object trajectory, penetration and impulse refinement gates pass.
  These tests do not demonstrate grasping, task success, or policy transfer.

The original table scene failed the 1 mm penetration criterion (up to 6.3 mm).
The drop diagnostic therefore explicitly uses table/puck `solref=[0.002,1]`,
representing an assumed hard table. It reaches at most 0.642 mm penetration.
This setting stays identical across timesteps; source XML is untouched.
The first failed traces remain in `results/contact_first`. A subsequent report
serialization error is preserved by its raw traces in `contact_rigid_table`;
the Python/NumPy boolean conversion was corrected before the passing rerun.

The review corrected two false-pass gaps in the earlier motion check: omitted
intermediate physics samples/passive fingers, and lack of actual gripper-motion
verification. All limits are now sampled at each physics step and both fingers
must reach close/open windows within 0.05 rad.

## Interface and remaining research work

`robot.joint_mapping(model)` supplies explicit joint, position, velocity, and
actuator indices. Inputs are joint positions in radians. `data.actuator_force`
is servo output effort; do not interpret `data.ctrl` as torque. Existing arm
gains, friction and armature are vendor assumptions. No gravity feedforward is
applied by these tests. The only altered contact parameters are documented above.

Before comparison studies, identify drive dynamics, friction, delays and contact
parameters from measurements, then define matched calibration and evaluation
tasks. Fixed-body COM differences and torso treatment are recorded in
`WORKFLOW.md`. This setup does not qualify a mobile or floating-base model.
The separate one-arm electrical model still has its previously identified
multi-joint solver stall; passing this MuJoCo suite does not resolve it.

## Lift, torso bend and drive base (5 Nov 2026)

`models/mobile_lift_torso.xml` puts these arms on a Pixel-style holonomic base, 430 mm lift and torso-pitch joint. Sizing: `size_lift_torso.py` (this venv) then `python plot_lift_torso.py` (system Python, matplotlib). See `LIFT_TORSO_BENCHMARK.md`.
