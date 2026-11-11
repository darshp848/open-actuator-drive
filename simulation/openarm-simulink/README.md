# OpenArm Simulink setup

This folder contains a one-right-arm dynamics and averaged electrical diagnostic.
It is the existing X1 support-board/actuator model. Full upper-body geometry,
two-arm interactions, collision/contact mechanics, compliant transmissions, and
research-grade sensing are not implemented here yet. Read
[the setup review](setup-review-20261102.md) before interpreting results.

## Validate without solving

Use MATLAB R2026a Update 5 with Simulink, Simscape, and Robotics System Toolbox.
Compiled robotics blocks also require the configured MSVC toolchain for a solve.
Set Current Folder to this directory. Do not clear/reset the MATLAB search path.

```matlab
report = check_openarm_setup();
```

This checks toolbox licenses, scenario references, SimulationInput overrides,
top-level connectivity, source syntax, and rejection of incomplete logs. It does
not compile or solve the dynamics, validate physical accuracy, or qualify hardware.
The saved SLX is used as-is. An absent model is an error. Model generation is an
explicit separate action with `build_openarm_system`, followed by validation.
The old `build=true` option is rejected to keep generation and solving separate.

## Run a bounded diagnostic

Use an isolated MATLAB process while the desktop is in use. For example, from
PowerShell with this folder as the working directory:

```powershell
& 'C:\Program Files\MATLAB\R2026a\bin\matlab.exe' -batch "setenv('NoDefaultCurrentDirectoryInExePath',''); R=run_openarm('multi_fast',stop_time=0.75,solver='ode15s',timeout_s=90); save('results/multi_fast_diagnostic.mat','R'); disp(R.validation);"
```

The timeout limits solver execution; startup and compilation can take additional
time. Simulation variables and solver settings use `SimulationInput`, which
reverts its overrides afterward. The runner checks for a different or dirty loaded
model, so it cannot silently use an unsaved model from another task.

Inspect `R.validation`, `R.sim_error`, and
`R.simulation_metadata.ExecutionInfo`. A valid completed run needs the
`ReachedStopTime` stop event, finite monotonic logs, and the requested time covered.
A timeout can leave an empty error message, so checking only `sim_error` is
insufficient. `R.screens.full_scenario` stays false for shortened diagnostics.
Solver warnings are reported separately and must be reviewed before accepting
numerical results. A failed physics screen can also be an expected negative
control, such as `drop6_nobrake`; completion and physical performance are distinct.

## Preserve results

```matlab
batch_run({'drop6'});
```

`batch_run` creates a new timestamped directory under `results`, retains partial
run data, and stops on an incomplete run. It never rebuilds or overwrites the
historical October results. Raw data contain requested duration, options, MATLAB
version, and simulation metadata. Archive source hashes alongside any result
used as evidence. The resumed-work before snapshot and logs are under
`results/setup_20261102`.

## Gates before research training

1. Complete the multi-joint and gates-off cases without solver warnings.
2. Demonstrate timestep/tolerance convergence and mechanical/electrical energy
   consistency. Hold controller and sensor clocks fixed during refinement.
3. Resolve motor winding/current conventions, adapted inertias, voltage limits,
   and duty assumptions against measured or primary-source data.
4. Add and independently validate sensing, gearbox/contact dynamics, and the
   chosen command interface before comparing methods on contact tasks.
5. Freeze calibration budgets and held-out evaluation plants. Report task success
   and uncertainty; torque tracking alone does not establish transfer.

The [revised research plan](https://docs.google.com/document/d/17zWAgiYKPLfp3367pUvM1ARaBX4xY1rqF7eL1cgZ7E0/edit)
uses matched calibration and task success as the main comparison. It does not
assume a simpler actuator model reduces total training cost.
