# OpenArm simulator setup review, 2 November 2026

## Purpose and sources

Resume the paused local Claude session `290041a1-1a01-4026-96a8-9c50006e316f`.
User priority: configure the simulator properly before further research runs.
The source scripts, saved SLX, previous logs, and original `_baseline_20261101`
have been inspected. `capsule check REVA-PCB` reports FRESH at `25842df`;
the worktree has substantial unrelated changes, which must be preserved.
An additional source snapshot is in `results/setup_20261102/source_before`.

Read the Drive [literature review](https://docs.google.com/document/d/1GhpBmaXnb4zmagCBXsdcIGcLNh25rjAT/edit)
and [revised research question](https://docs.google.com/document/d/17zWAgiYKPLfp3367pUvM1ARaBX4xY1rqF7eL1cgZ7E0/edit).
Their current direction is a matched-calibration comparison of actuator-side
sim-to-real methods on contact tasks. The earlier GPU-saving thesis is unsupported.
Proposed plain-language sentence: "I study which actuator models and local
controllers let robot policies trained in simulation succeed at real contact
tasks with the same calibration effort."

## Scope

`openarm_system.slx` is a one-right-arm, rigid-transmission dynamics and averaged
drive/power-network diagnostic. It is not yet the full bimanual upper body or a
contact research benchmark. The mesh-free URDF intentionally omits collision
geometry. Contact, gearbox compliance/backlash, realistic sensing and delay,
independent held-out evaluation plants, and matched calibration protocols are
still required before this model can support the main research comparison.
Do not label a completed electrical diagnostic as validation of those capabilities.
No hardware, CAD, training, component selection, or release work is authorized by
this diagnostic milestone. Existing model qualification gates remain open.

## Supported workflow and first diagnostic

Installed MATLAB: R2026a Update 5. An isolated `matlab -batch` read-only check
completed and confirmed Simulink, Simscape, and Robotics System Toolbox licenses.
Installed `help Simulink.SimulationInput.setModelParameter` confirms `TimeOut`,
`CaptureErrors`, and nonpersistent parameter overrides. Use `SimulationInput`
variables instead of assigning scenario variables to the shared base workspace.
Generate models only in an explicit build stage, validate separately, then solve.
The existing saved model is retained for this milestone; no implicit rebuild.

Official sources consulted before implementation:

- [R2026a physical-model solvers](https://www.mathworks.com/help/releases/R2026a/simscape/ug/setting-up-solvers-for-physical-models.html):
  implicit `ode23t`, `ode15s`, and `daessc` are supported choices.
- [R2026a anti-windup example](https://www.mathworks.com/help/releases/R2026a/simulink/slref/anti-windup-control-using-a-pid-controller.html):
  conditional integration and back-calculation are distinct supported methods;
  a controller with feedforward needs its saturation feedback interpreted correctly.
- Local load-study workflow retains the upstream URDF commit and independent
  gravity/potential-energy checks in `../openarm-load-workflow-20261025.md`.

Plausible stall causes, not established diagnoses: discontinuous current-loop
integrator freezing at `s >= 0.999`, power-network switching stiffness, sampled
ideal-diode/chopper behavior, or drive saturation. Original `multi_fast` log
stalls at 0.70404794 s with step size near 2.5e-15 s. Do not simply raise the
allowed minimum-step violations or loosen tolerances to hide this failure.

First fix is run integrity: bounded wall time, explicit preparation/validation,
and rejection of missing/nonfinite/truncated logs. First solve diagnostic then
crosses the known transition only to 0.75 s using a supported implicit solver
override. Success means reaching the requested time with finite outputs and no
captured solver error. A full scenario and step/tolerance refinement are separate
gates. After two repeated failures or an application crash, preserve evidence and
re-check documentation before further mutations.

## Physics issues retained as open

Motor current/Kt/Kv conventions are not resolved by replacing one number with
another. Preserve the linked energy-consistent PMSM torque and flux relation
until winding and current conventions are established. GIM4310's cited 12–40 V
rating does not qualify it on the modeled 48 V bus. Motor mass changes update
mass/COM but not all adapted inertias. Current/position loops are continuous
idealizations, while supervisor sampling is 20 us and logging is 50 us. Backup
propagation delays therefore have limited resolution. Original simple-case
results are conditional historical evidence, not a validated worst-case envelope.

## Verification results

- Preflight passed in MATLAB R2026a Update 5: six scenario input sets, required
  licenses, matching embedded controller/supervisor source, no unconnected
  top-level ports, and seven run-integrity regression cases.
- `multi_fast`, requested 0.75 s with `ode15s`: timed out at 0.7040 s after the
  configured 90 s execution budget, 125.7 s including initialization. ErrorMessage
  was empty, but StopEvent was `Timeout`. The new completion gate correctly failed.
  Raw log and MAT are in `results/setup_20261102/multi_fast_ode15s_075.*`.
- Independent state capture to 0.704 s completed with no warnings. Exact d-axis
  current/integrator states are zero, voltage demand is below saturation, and
  all protection resistance-filter states are steady. No evidence currently
  justifies changing anti-windup, motor constants, or chopper thresholds.
  `before_transition.mat` and `.log` retain the states.
- A 0.1 s `drop6` runner/plot smoke passed, with `ReachedStopTime`, finite monotonic
  logs, and no solver warnings. `full_scenario` correctly remains false. Separate
  timestamped output contains both MAT and PNG, leaving old results untouched.
- Repository structure check fails on existing unrelated items: a missing raw-waveform
  ZIP link, retired Altium path references, and unexpected root directories. These
  are outside this simulator change and were not modified.

After the repeated transition failure, official R2026a
[Solver Profiler](https://www.mathworks.com/help/releases/R2026a/simulink/slref/solverprofiler.profilemodel.html)
and [applyToModel](https://www.mathworks.com/help/releases/R2026a/simulink/slref/simulink.simulationinput.applytomodel.html)
documentation and installed help were rechecked. `profile_openarm_stall` is a
bounded diagnostic to 0.7041 s with state, Jacobian, and zero-crossing recording,
90 s timeout, and no profiler UI. The first 10 s capture stopped at 0.2643 s,
before the transition, so it cannot identify the stall. It applies the prepared scenario
only in a disposable batch process and discards the in-memory model afterward.
Profile completion must not be confused with scenario completion.

The 90 s capture reached 0.7040 s and retained
`results/profile_20261102_015628_432/profile.mat` plus `summary.mat`.
It recorded 1,333,837 solver steps, zero zero-crossing events, 35,203 resets,
35,206 Jacobian updates, and 3,540 solver exceptions. This narrows the next
investigation to solver exceptions/DAE or error-control behavior; it does not
support a zero-crossing-chatter diagnosis. The detailed profile is opened with
the supported `solverprofiler.exploreResult` viewer. No further physics edits
or ordinary solve retries were made. The saved SLX and all five original
physics/build scripts still match their before-snapshot hashes.

The MATLAB-driven MuJoCo contact probe in Actuator-controls now passes. The tool
recommendation is MuJoCo for robot/contact experiments, with MATLAB orchestration
and Simulink retained for electrical diagnostics. This is not a model migration
or a comparative speed benchmark. Full bimanual OpenArm setup remains open.
