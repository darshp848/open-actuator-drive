# GIM actuator layer for the OpenArm v2 MuJoCo model: spec (2 Nov 2026)

Goal: replace the vendor's ideal Damiao position servos with a model of the actuators Darsh will actually build:
SteadyWin GIM motors through 36:1 / 40:1 planetary gearboxes, driven by the X1/P96 boards (DRV8363, 48 V bus,
dq current loop, shunt current sensing, MT6835 rotor encoder plus an output encoder). This is the actuator plant
that the research question needs (sim-to-real on contact tasks); it is not a calibrated digital twin.

## Ground rules
- New code lives only in `simulation/openarm_v2/gim/`. Do not edit vendor files, `robot.py`, `run_*.py`,
  `batch_stage.py`, `validate_mechanics.py`, `build_contact_fixture.py` or `run_openarm_v2.m` (Codex's
  validated stock-servo pipeline hashes them). Import from `robot.py` instead.
- Python: `simulation/.venv-openarm/Scripts/python.exe` (MuJoCo 3.6.0). No new installs.
- Every parameter cites its source or is marked ASSUMED, as in
  `Codex_June26/48V_inverter_project/hardware/kicad/x1_redesign/system_model/openarm_params.m`.
- Never overwrite results; write to new timestamped folders under `gim/results/`.

## Joint map (OpenArm v2, both arms; fingers keep the vendor servo)
| Joint | Motor | N | listed output Kt (Nm/A) | stall A | tau_max Nm | R_ll ohm | L_ll mH | pp | max out speed rpm |
|---|---|---|---|---|---|---|---|---|---|
| J1-J2 | GIM8115-36 | 36 | 11.05 | 21 | 228 | 0.98 | 0.55 | 21 | 36 |
| J3-J4 | GIM8108-36 | 36 | 7.32 | 19.8 | 109.5 | 0.22 | 0.21 | 21 | 60 |
| J5-J7 | GIM4310-40 | 40 | 1.79 | 17.1 | 30.59 | 0.27 | 0.8 | 14 | 86 |
Source: openarm_params.m (SteadyWin pages read 2026-10-25). Same conventions as that file:
eta = 0.9 (ASSUMED), kt_m = Kt_out/(N*eta), lambda = kt_m/(1.5*pp), R = R_ll/2, L = L_ll/2 (per-phase star),
J_rotor = [1.2e-4, 1.0e-4, 2.0e-5] kg m^2 (ASSUMED), tau_coulomb out = [0.8, 0.6, 0.15] Nm, b_visc out =
[0.5, 0.4, 0.05] Nm s/rad (ASSUMED). Add a `lambda_source` option: "kt" (default, as above) or "kv" (lambda from
the listed output rpm/V: 0.75 / 1.25 / 3.58 rpm/V; motor Kv = N * that; pp*lambda = 1/(sqrt(3)*Kv_rad_per_s_per_V)).
Note: the GIM4310 is rated 12-40 V; flag it when bus > 40 V but still run.

## Model
1. **Mechanics.** Two transmission modes, chosen by config:
   - `rigid`: set the arm joint's `dof_armature = J_rotor*N^2`, `dof_frictionloss = tau_coulomb`,
     `dof_damping = b_visc` (replacing the vendor values), and apply the output torque via `qfrc_applied`.
   - `compliant`: add one rotor hinge DOF per arm joint with MjSpec (child body of the joint's parent body, same
     axis and anchor, tiny mass, `armature = J_rotor*N^2`, expressed in OUTPUT-angle units so no gearing appears in
     MuJoCo). Friction (frictionloss/damping) goes on the rotor DOF. Gearbox torque between rotor and link
     `tau_g = k_g * deadzone(theta_r - q, backlash/2) + c_g*(w_r - w)` applied equal and opposite with
     `qfrc_applied`. Backlash = 15 arcmin full width for the GIMs (SteadyWin listing for GIM8115; ASSUMED same for the
     others). k_g = [3000, 2000, 400] Nm/rad, c_g chosen for zeta ~0.05 (ASSUMED). Smooth the deadzone edge (e.g.
     softplus width 1e-5 rad) and check explicit stability: report omega_n*dt for rotor-vs-lightest-link and keep it
     < 0.3, substepping the gearbox force if needed.
   - Optional `gim_masses=True`: add the GIM-minus-stock motor mass to each joint's child body as in
     openarm_params.m (stock masses ASSUMED [0.80,0.80,0.36,0.36,0.30,0.30,0.30] kg), recomputing COM.
2. **Electrical (per joint, substep 25 us inside each physics step).** id = 0. The current loop is a first-order
   lag with bandwidth `wc` (default 2*pi*1 kHz) toward `iq_target = clip(iq_cmd, +-i_max)`, further limited by the
   voltage circle: with id = 0, |v|^2 = (R*iq + we*lambda)^2 + (we*L*iq)^2 <= (v_frac*Vbus)^2, v_frac = 0.95/sqrt(3),
   we = pp*N*w_rotor_output; solve for the admissible iq interval each substep. Vbus default 48 V (option 50.4).
   Motor torque at output: `tau_out = N*kt_m*iq*eta^tanh(P_m/2)` with P_m = kt_m*iq*N*w (motoring eta,
   generating 1/eta, as ctrl_fcn.m). Log DC power per joint = 1.5*(vq*iq) + 1.5*iq^2*R_inverter(9 mOhm) for later
   bus work.
3. **Drive firmware (runs at `drive_hz`, default 10 kHz; held between updates).**
   - `torque` mode: iq_cmd = tau_cmd/(N*kt_m*eta_hat), eta_hat = 0.95.
   - `pd` mode (MIT style, what OpenArm/Damiao use): tau_cmd = kp*(q_ref - q_meas) + kd*(v_ref - v_meas) + tau_ff.
   - Measurements: q_meas from the rotor MT6835 (21 bit per motor turn) divided by N, v_meas from a first-order
     filtered difference (cutoff option, default 1 kHz). Current measurement quantized at `i_lsb` (default 0.0806 A
     = 1 mOhm shunt, gain 10 V/V, 12-bit ADC at 3.3 V; option 0.016 A for the 5 mOhm P96 plan). Expose these for
     torque-observer work later.
4. **Command path.** Commands arrive at 1 kHz with a configurable integer delay in ms (default 1 ms) and are held.

## Interface
`gim/actuated.py`: `GimArm(config)` builds the model (vendor pedestal model via robot.load(), or the contact
scene), with `.step_ms(commands)` taking per-arm-joint q_ref/v_ref/tau_ff (pd) or tau_cmd (torque) for the 14 arm
joints; fingers keep vendor position control. Logs per step: q, v, theta_r, iq, iq_cmd, tau_out, tau_gear, P_dc,
voltage-limit flag.

## Validation gates (`gim/validate.py`, writes JSON + PNG, each gate pass/fail)
1. **Gravity hold (torque mode, rigid):** command mj_inverse gravity torque at a non-trivial pose with friction off;
   drift < 1 mrad over 1 s. Confirms the torque scaling chain including eta_hat vs eta (expect the documented
   residual from eta_hat != eta; report it rather than hide it, run once with eta_hat = eta for the strict gate).
2. **Torque-speed:** one GIM8115 joint, gravity off, full current command; terminal output speed within 2 % of
   the analytic voltage-limited speed for that lambda and Vbus. Repeat for each motor type.
3. **Current step:** 63 % rise time within 10 % of 1/wc (at standstill).
4. **Energy:** friction off, zero current, gravity on, compliant mode; total mechanical energy (incl. gearbox
   spring energy) drifts < 0.5 % over 2 s.
5. **Backlash:** slow sinusoidal torque reversal on a held link; measured dead band of theta_r - q within 5 % of
   the configured width.
6. **Timestep refinement:** Codex's motion profile (copy the blend/targets from run_diagnostics.py) in pd mode at
   physics 0.5 / 0.25 / 0.125 ms, fixed 1 kHz commands: joint RMS difference < 0.005 rad between successive steps.

## Before/after comparison (`gim/compare.py`): stock vendor servo vs GIM layer, same scenarios
a. Codex's 1.6 s bimanual motion with the vendor kp/kv (pd mode for GIM): max tracking error, peak torque ratio.
b. Reflected inertia per joint (vendor armature vs J_rotor*N^2) as a table.
c. Torque-speed envelope per joint (vendor: forcerange and velocity limits from run_diagnostics.py; GIM: computed).
d. Interaction impedance (the Idea 1 quantity): hold the right arm at the home pose, apply a 5 Nm external torque
   step on right J4 (and 1 Nm on J6) through qfrc_applied for 0.3 s then release; report peak and steady
   deflection, settle time, and residual offset after release (friction/backlash stiction) for stock vs GIM rigid
   vs GIM compliant.
Write `gim/results/<timestamp>/comparison.md` with tables and figures, plus `README.md` in gim/ with how to run.
