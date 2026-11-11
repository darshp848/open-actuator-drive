# GIM actuator layer for OpenArm v2 (MuJoCo)

Replaces the vendor Damiao position servos on the 14 arm joints with SteadyWin GIM motors (GIM8115-36, GIM8108-36,
GIM4310-40) behind planetary gearboxes, a dq current loop on a 48 V bus and MIT-style drive firmware. Fingers keep the
vendor servo. Spec: `SPEC.md`. This is an actuator plant for sim-to-real contact work, not a calibrated digital twin.
Nothing outside `gim/` is modified; `robot.py` is imported.

## Run (from `simulation/openarm_v2`, venv `../.venv-openarm`)

```
python gim/validate.py [gate numbers]   # six gates -> results/<timestamp>_validate/{gates.json,gates.png}, ~1 min
python gim/compare.py                   # stock vs GIM -> results/<timestamp>_compare/{comparison.md,.json,*.png}, ~1 min
```

```python
from actuated import GimArm, Config
arm = GimArm(Config(transmission='compliant', mode='pd'))        # 'rigid' | 'compliant', 'pd' | 'torque'
arm.step_ms({'q_ref': q14, 'v_ref': v14, 'tau_ff': t14}, fingers=[0.35, -0.35])   # 1 ms
log = arm.log()    # q, v, theta_r, iq, iq_cmd, tau_out, tau_gear, p_dc, vlim, q_meas, v_meas, i_meas, tau_cmd
```

Files: `params.py` (motor table, sources/ASSUMED), `actuated.py` (model + `GimArm`), `scenarios.py` (Codex motion
profile, impedance hold), `validate.py`, `compare.py`, `miniplot.py` (Pillow plots; matplotlib is not installed).

## Implementation notes

- Electrical substep 25 us, drive firmware 10 kHz (every 4th substep, independent of the physics step, with the
  encoder reading extrapolated to the substep), commands at 1 kHz with a 1 ms delay. Motor torque is the mean current over the
  physics step. Voltage limit solved in closed form each step (admissible `iq` interval, `id = 0`); if the speed is
  above the no-load speed the interval collapses to the least-voltage current, which brakes (see below).
- Compliant mode adds a rotor hinge (MjSpec) per arm joint in output-angle units. Gearbox force is explicit
  (`qfrc_applied`), substepped so that `omega_n*dt <= 0.3` (at 0.5 ms the lightest case is 0.39, so 2 substeps).
  `c_g = 2*zeta*sqrt(k_g*J_eff)` with `J_eff` the series rotor/lightest-link inertia (min over 40 poses).
- `i_max = min(stall A, listed torque / Kt)`: 20.6 / 14.96 / 17.1 A, so the output torque limit equals the listed tau_max.
- Link-side joint damping/friction/armature of the vendor model are zeroed; friction and reflected inertia are the
  GIM values (rigid: on the link DOF; compliant: on the rotor DOF).
- `gim_masses=True` adds the GIM-minus-stock motor mass at each child body origin and scales the COM as in
  `openarm_params.m` (inertia about the COM left unchanged, as there). Not used by the gates or comparison.

## Things that look physically suspicious

1. `eta**tanh(P/2)` is 1 at zero power, so the output torque at standstill is `kto/eta * iq` : 253 Nm for the
   GIM8115 at `i_max`, above the listed 228 Nm, and torque drops ~10 % as soon as the shaft moves. Gravity hold
   with eta = 0.9 therefore leaves ~11 % torque residual even for `eta_hat = eta` (gate 1 variants).
2. The `kt`-derived back-EMF constant (eta folded into kt_m) gives lower no-load speeds than the listed rpm/V
   (GIM8115 31 vs 36 rpm at 48 V). `lambda_source='kv'` is available. J1/J2 top out at 3.2-3.6 rad/s vs 16.8 rad/s
   in the stock velocity limit, and the GIM4310 would run at ~190 rpm on 48 V against its 40 V rating.
3. Above the no-load speed (e.g. a freely falling J4 > 4.9 rad/s) zero current is not available: the clamp forces
   a regenerative braking current. This is what an inverter with `id = 0` and no field weakening does, but gate 4
   had to use a very high bus voltage to isolate energy conservation.
4. Gearbox damper `c_g` acts also inside the backlash gap (spec formula); it is small (zeta 0.05) but slightly
   couples rotor and link there.
5. Rotor/gearbox parameters (J_rotor, friction, k_g, 15 arcmin for J3-J7, eta) are ASSUMED; impedance results depend on them.

## Gate-specific choices (see `gates.json`)

- Gate 1 strict variant uses eta = eta_hat = 1 (see 1.); eta 0.9/0.9 and 0.9/0.95 residuals are reported. Fingers are frozen
  (their servo sag otherwise couples into the arm) and the command, firmware and current loop start in steady state.
- Gate 4 uses `c_g = 0`, friction off, contacts and joint limits off, finger servos off, vbus = 1e6 V; drift is relative to the peak
  kinetic + gearbox spring energy reached. With zeta 0.05 the damper dissipates 0.38 %.
- Gate 3 runs the model at dt = 25 us with the link locked (large armature); rise time is the interpolated 63 % crossing.
- Gate 2 compares with a root-find of the same torque law, so it checks implementation consistency, not the law itself.

## Review changes (2 Nov 2026, after the first build)

- Gear efficiency is now load-dependent Coulomb friction, `frictionloss = tau_c + (1 - eta)|N kt_m iq|`, updated every
  physics step and solved by MuJoCo. The first build used `eta^tanh(P/2)` from ctrl_fcn.m, which gives efficiency 1
  at standstill (253 Nm GIM8115 stall output against the listed 228 Nm). A smooth blend toward eta at standstill was
  tried and rejected: it made the efficiency non-monotonic in power and the gravity hold unstable. Motoring output is now
  exactly the listed Kt x iq (228 / 109.5 / 30.6 Nm), and a held joint sits inside the friction band.
- Friction constraint `dof_solref = [0.002, 1]` on the friction DOF. MuJoCo's default soft friction (0.02 s) lets a
  loaded joint creep about 1.4 mrad/s; with 0.002 s it is about 0.14 mrad/s. Residual creep is a simulator artifact.
- Gate 1 now passes with eta 0.9 / eta_hat 0.95 and friction on (0.14 mrad in 1 s). Current evidence:
  `results/20261102_034915_validate`, `results/20261102_035016_compare`. Earlier result folders are superseded.
