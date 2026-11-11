# OpenArm v2 model construction and validation

User authorizes the actual bimanual OpenArm model, MATLAB batch orchestration,
verified mechanics, grippers, and contact tests. No training or hardware work.

## Research before implementation

Inspected the existing electrical model, synthetic MuJoCo plant, known-good
MATLAB-to-Python probe, and pinned original URDF. Original URDF commit is
`81fe64755f78777c9d55d7e5cc5a238a143f7950`; SHA256 is
`045093bd5a08311d9ff81616b5504e643185f7d8872e33f924e552cde9ddb3cd`.
Both independent source audits found 14 arm joints, four finger coordinates,
two finger mimic constraints, and 16 independent commands.

Reuse the official [OpenArm MJCF](https://github.com/enactic/openarm_mujoco)
at commit `06b4c8ad6d547c3faaa3bf780629ee3e1ac2b0d0`. Its pedestal wrapper supplies
the fixed body geometry and 0.697 m arm mounting height. Its ready-made collision
meshes and grippers avoid a guessed URDF conversion. The source package requires
MuJoCo >=3.6.0. Use a separate `.venv-openarm` and pin 3.6.0, leaving all older
environments/results unchanged. Download data assets only; do not execute vendor
launch scripts or install its package automatically.

Supported workflows were checked against official
[MuJoCo 3.6 Python](https://mujoco.readthedocs.io/en/3.6.0/python.html),
[MJCF reference](https://mujoco.readthedocs.io/en/3.6.0/XMLreference.html),
the vendor XML/defaults, and MATLAB R2026a `system` used successfully by the
existing probe. MATLAB starts whole Python jobs and checks their exit/result;
it does not start a process for each physics step or change persistent `pyenv`.

## Stages and evidence

1. Acquire immutable source/assets with Git blob verification and SHA256 manifest.
2. Compile without stepping. Check expected joints, actuators, equality constraints,
   names, limits, masses, positive inertias, and geometry counts.
3. Compare body FK and gravity/mass matrix to an independent URDF Jacobian model,
   including armature separately. Test zero and deterministic nonzero poses.
   Report source rounding and fixed-body discrepancies rather than silently
   editing either vendor source. Initial tolerances: 2 um body-position error,
   1e-5 rotation error, 1e-4 relative mass-matrix error and 1e-4 Nm gravity error.
4. Only after mechanics passes, run bounded hold/motion/gripper diagnostics.
   Monitor effort/speed limits, mimic residuals, contacts and solver warnings.
5. Add one bounded gripper/object contact case and timestep refinement with
   unchanged controller clocks and contact solref/solimp. Compare impulse,
   penetration, object trajectory and joint trajectory, not peak force alone.
6. Verify the complete staged path through MATLAB batch, render a preview, and
   document the command interface and limits before any research training.

Plausible failure causes: wrong mirrored joint signs, incorrect frame height,
mesh units, source-version mismatch, omitted armature, mimic indexing, treating
position commands as torques, and changing effective contact stiffness during
timestep refinement. First diagnostic is compile/introspection only.
Stop after two repeated failures or an application crash, preserve evidence,
and re-check the vendor workflow before further mutations.

## Scope and source differences

The official baseline uses stock Damiao position servos, not custom GIM drives.
Its friction, armature, gains, contact and actuator response remain assumptions.
The URDF contains 16.0000073 kg including the 3.74 kg fixed torso; MJCF dynamic
arm/gripper mass is 12.260006 kg, with torso geometry fixed to the world.
Fixed arm-base COMs differ by 5.063 mm between sources. Moving-link COMs agree
within 1 um. Fixed masses do not change the moving-joint dynamics on this fixed
pedestal; this does not qualify a floating-base or mobile robot model.

Contact diagnostic gates (before first solve): clamp both fingers on fixed sphere; drop puck 40 mm onto table; 1/.5/.25 ms physics with fixed 1 ms controller. Require finite complete runs, no warnings, penetration <1 mm, mimic error <.003 rad. Successive refinements: impulse <5%, joint RMS <.005 rad, object-position RMS <1 mm, penetration difference <.2 mm. Keep native contact settings unchanged. These are numerical checks, not calibrated physical accuracy.

First contact evidence retained in results/contact_first: clamp checker used incorrect geom-name substring (corrected to actual finger_inner_right/finger_outer_right). Native table/puck default contact time constant .02 s produces 6.3 mm penetration under 40 mm drop. Official 3.6 modeling solver-parameters documentation confirms compliance and time-constant/timestep dependence. Next bounded diagnostic explicitly sets only table and puck solref to [.002,1], an uncalibrated hard-table assumption; runs .5/.25/.125 ms physics unchanged across refinements, retaining original acceptance thresholds and vendor files.
