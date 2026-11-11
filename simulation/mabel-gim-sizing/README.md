# GIM actuators on the MABEL MuJoCo model: torso bend and reach tests

This folder ports the GIM actuator layer from `simulation/openarm_v2/gim/` (rigid transmission only) onto MABEL's own
MuJoCo model and runs a gravity hold, a torso bend and a combined reach, for 0 kg and 3 kg per hand, against the stock
MABEL position servos. It is an actuator plant for sizing and sim-to-real work, not a calibrated digital twin.
Nothing in `openarm_v2/` was edited or imported (its files still have their original hashes and dates), and nothing from
the MABEL repo was run: only `mabel_full.xml` is loaded, by absolute path.

## What was built

| file | purpose |
|---|---|
| `rig.py` | MABEL with a fixed chassis, the 15-joint GIM layer (Torso + Left/Right Arm 1..7) and the stock-servo baseline |
| `scen.py` | reference trajectories (minimum jerk), feedforward by `mj_inverse`, metrics, plots |
| `run_all.py` | scenarios A, B, C for both payloads and all controller variants |
| `gate.py` | consistency gate (static motor torque vs `mj_inverse` gravity torque) |
| `sweep_bend.py` | shorter bend times to find where one GIM8115-36 runs out |
| `check_gains.py` | PD step-response stability check |
| `make_summary.py` | markdown tables from a results folder |
| `gim_params.py`, `miniplot.py` | unchanged copies of `openarm_v2/gim/params.py` (SHA-256 of the body 9f360146...) and `miniplot.py` (02e92c48...) so the pinned folder is never imported |
| `dev/` | two small probe scripts used while building (static hold, payload lever arm) |

Run from anywhere with the venv python, always isolated:
`C:/Users/darsh/TAMU/EMPE_Lab/Actuator-controls/simulation/.venv-openarm/Scripts/python.exe -I -B mabel_gim/run_all.py [A] [B] [C]`
(`-B` stops bytecode being written). Results of the runs quoted here:

- `results/20261105_072645_scenarios/` : `metrics.json`, `summary.md` (all tables), PNGs (`B_*`, `C_*`: torso q vs ref, tracking error, iq vs limits, DC bus power, torque-speed with the envelope, arm J1/J2 iq/i_max; `A_*`: hold error and iq), `.npz` time series
- `results/20261105_072418_gate/gate.json`, `results/20261105_072546_sweep/sweep.json`, `results/20261105_071724_gains/gains.json`

## Model decisions

- **Locked base.** The `base_root` free joint is deleted in MjSpec, so the chassis is welded to the world at its model pose
  (z = 0.07775 m). The six swerve actuators are zeroed. Contacts are disabled (free-space motion; joint limits and
  friction stay on). The 360 lidar sensors are deleted for speed. Physics step is 0.5 ms, integrator `implicitfast` as in the XML.
  The stock baseline also runs at 0.5 ms (the XML default is 2 ms) so both models share the numerics.
- **Actuators.** Arm J1-J2 GIM8115-36, J3-J4 GIM8108-36, J5-J7 GIM4310-40 (same `params.py` values, mapped by joint index
  for both arms). Torso GIM8115-36 (N = 36, listed 228 Nm). The GIM replaces the stock actuator, armature, damping and
  friction of those 15 joints (torso armature 1.0 -> 0.156 kg m^2, damping 2.0 -> 0.5, friction 0.8 -> 0.8 Nm; arm values
  from `params.py`). Lift, neck, hands and base keep the stock actuators. All ASSUMED labels in `gim_params.py` carry over
  (eta 0.9, J_rotor, Coulomb and viscous friction, 15 arcmin backlash unused in rigid mode, R_INVERTER 9 mOhm).
- **Electrical layer unchanged in substance.** 25 us substeps, dq current loop 1 kHz, closed-form voltage-circle limit at
  48 V (`0.95/sqrt(3) Vbus`), firmware at 10 kHz, commands at 1 kHz with 1 ms delay, MT6835 quantisation, eta_hat 0.95,
  load-dependent gear friction `tau_c + (1 - eta)|N kt_m iq|` with `dof_solref 0.002`. With the kt-derived back-EMF constant
  the GIM8115 no-load output speed at 48 V is about 3.2 rad/s (31 rpm; listed 36 rpm).
  The GIM4310 (J5-J7) is rated 40 V and is run at 48 V as specified, so the J5-J7 results are outside its rating.
- **Control.** Per joint MIT PD in firmware: `tau = kp (q_ref - q) + kd (v_ref - v) + tau_ff`.
  Torso kp = 4200 Nm/rad, kd = 335 Nm s/rad (I = 6.7 kg m^2 at the 3 kg worst pose, wn = 25 rad/s, zeta = 1).
  Arms keep MABEL's own per-joint gains (kp 1205 / 1880 / 760 / 940 / 269 / 249 / 315, kd 40 / 63 / 25 / 31 / 9.0 / 8.3 / 10.5).
  Step tests (`check_gains.py`, torso 0.05 rad, arms 0.02 rad, both payloads) show no oscillation: overshoot at most 5.5 %,
  2 % settling 0.09 to 0.29 s, tail ripple under 40 urad.
  Variants run in every scenario: `gim_grav` (main: v_ref = reference velocity, tau_ff = `mj_inverse` gravity and payload
  at zero velocity and acceleration on the commanded pose), `gim_full` (tau_ff = full inverse dynamics including velocity,
  acceleration and the lift motion), `gim_grav_v0` (gravity ff, v_ref = 0, which damps against motion the way the stock
  servo does). Stock baselines: `stock_plain` (ctrl = q_ref, as shipped) and `stock_ff` (ctrl = q_ref + gravity/kp, the same
  gravity feedforward).
  The feedforward comes from a constraint-free copy of the model, so it contains gravity, payload, inertia (with the GIM
  armature) and viscous damping but not Coulomb friction.
- **Worst pose and payload** are those of `size_mabel.py`: torso -1.0472, Left Arm 1 = -2.2689, Left Arm 4 = 0.9, right arm
  mirrored (Right Arm 1 = +2.2689, Right Arm 4 = -0.774). Payload is a 3 kg point mass (0.03 m sphere) at each palm origin.
  The torso static torque reproduces the sizing: 59.1 Nm and 104.4 Nm.
- **Scenario timing.** Every run holds the start pose for 1 s first and metrics exclude that. B: bend 0 to -1.0472 in T,
  hold 1 s, return in T, 0.5 s tail, T = 2 s and 1 s. C: starts with lift 0, torso 0, Arm 1 = 0 and the elbows already at
  the worst-pose value (my reading, since only Arm 1 is said to swing); lift 0 to 0.635 m in 5 s on the stock lift servo,
  torso to -0.8 rad and both Arm 1 to their worst-pose values in 2 s, then 0.5 s of hold after the lift ends.
- **Metric definitions.** "Output torque" is the listed Kt times iq (what the shaft delivers while moving). Voltage limit
  time is the fraction of physics steps in which the voltage circle clipped the commanded current; current limit time is the
  fraction in which the command exceeded i_max (20.6 A for GIM8115). Bus power is `1.5 vq iq + 1.5 iq^2 R_INVERTER`
  (copper loss included), positive when drawing from the bus. The stock model has no electrical side, so only mechanical
  power could be computed for it (in `metrics.json`), not bus power.

## Consistency gate (passes)

Static hold, worst-pose arms, torso at -1.0 rad (inside the limit), lift 0.3175 m, 4 s, last 1 s averaged. The steady
electromagnetic torque `N kt_m iq` against the `mj_inverse` gravity torque at the settled pose, on every joint with more
than 2 Nm of gravity torque (5 joints at 0 kg, 9 at 3 kg):

| payload | friction off, worst joint error | friction on, worst joint error | torso, friction on (motor vs gravity) |
|---|---|---|---|
| 0 kg | 0.26 % | 1.03 % | 58.97 vs 58.37 Nm |
| 3 kg | 0.19 % | 1.34 % | 104.56 vs 103.18 Nm |

The 2 % criterion is met even with friction on, but that is partly luck: a joint at rest can legitimately sit anywhere in
the friction band `|tau_m - tau_g| <= tau_c + (1 - eta)|tau_m|`, about 6.7 Nm at the torso for 0 kg and 11 Nm for 3 kg,
roughly 11 % of the load. All joints sit inside their admissible interval (`gate.json`, `inside_friction_band`).
With friction off the residual is below 0.3 %. The firmware's eta_hat (0.95) differs from eta (0.9), so the delivered torque
is 5 % low while moving and 5 % high with friction off; the PD term absorbs that as a small position
offset (0.6 to 1.5 mrad here).

## Results

Full tables are in `results/20261105_072645_scenarios/summary.md`. Main GIM variant (`gim_grav`), torso, whole move:

| case | peak err (mrad) | RMS err (mrad) | peak abs iq/i_max | time on voltage / current limit | peak output torque (Nm) | peak speed (rad/s) | peak bus motoring / regen (W) |
|---|---|---|---|---|---|---|---|
| B 0 kg, T = 2 s | 2.8 | 1.7 | 0.28 | 0 / 0 | 64 | 0.99 | 76 / -33 |
| B 0 kg, T = 1 s | 6.6 | 2.7 | 0.34 | 0 / 0 | 79 | 1.98 | 159 / -84 |
| B 3 kg, T = 2 s | 4.7 | 2.7 | 0.50 | 0 / 0 | 115 | 0.99 | 163 / -46 |
| B 3 kg, T = 1 s | 13.8 | 5.8 | 0.67 | 0 / 0 | 154 | 2.00 | 361 / -136 |
| C 0 kg | 2.5 | 1.8 | 0.19 | 0 / 0 | 44 | 0.75 | |
| C 3 kg | 5.0 | 3.2 | 0.35 | 0 / 0 | 79 | 0.75 | |

Same cases, other controllers (torso peak error in mrad; B cases are 0 kg T2, 0 kg T1, 3 kg T2, 3 kg T1):

| controller | B 0 kg T2 | B 0 kg T1 | B 3 kg T2 | B 3 kg T1 | C 0 kg | C 3 kg |
|---|---|---|---|---|---|---|
| GIM, gravity ff | 2.8 | 6.6 | 4.7 | 13.8 | 2.5 | 5.0 |
| GIM, full inverse-dynamics ff | 2.7 | 3.5 | 4.3 | 5.5 | 2.3 | 3.7 |
| GIM, gravity ff, v_ref = 0 | 81 | 156 | 83 | 161 | 62 | 63 |
| stock DM-J10422P servo, as shipped | 30 | 51 | 39 | 62 | 11 | 20 |
| stock servo with gravity ff | 20 | 41 | 21 | 46 | 15 | 16 |

Torso DC bus energy (GIM, gravity ff). The forward bend lowers the load, so it is the regenerating stroke; the return
stroke lifts it and is motoring. Per phase (J, regen is negative):

| case | forward stroke: net | hold 1 s: net | return stroke: motoring / regen / net | whole move: motoring / regen |
|---|---|---|---|---|
| 0 kg, T = 2 s | -23.2 | +14.8 | +76.4 / 0 / +76.4 | 95 / -26 |
| 0 kg, T = 1 s | -29.0 | +19.2 | +67.2 / 0 / +67.2 | 88 / -30 |
| 3 kg, T = 2 s | -18.2 | +48.3 | +168 / 0 / +168 | 233 / -33 |
| 3 kg, T = 1 s | -33.9 | +59.0 | +146 / -1.0 / +145 | 216 / -44 |

So the net energy returned to the bus on the return stroke is essentially zero (at most 1 J at T = 1 s, 3 kg); the 26 to 43 J
of regeneration happens on the forward stroke. The hold costs 15 to 59 J because holding gravity is copper loss at standstill.

Gravity hold (scenario A, torso at -1.0472, 2 s after settling): the GIM torso sits 0.58 mrad (0 kg) and 0.84 mrad (3 kg)
above the target, with mean iq/i_max 0.235 and 0.42, creep under 0.08 mrad/s, and the joint is stuck (speed under 1 mrad/s)
for 100 % of the window. It is held inside its friction band, so the exact holding current depends on the approach direction
(by up to 6.8 and 11.4 Nm). It is not resting on the joint limit. The stock servo, plain, rests on the -1.0472 limit and its
actuator carries only 4 to 7 Nm; the joint limit takes the rest, which hides the real holding load. At -1.0 rad (inside the
limit) the stock plain servo sags 11.8 mrad (0 kg) and 20.3 mrad (3 kg), as kp = 5000 Nm/rad predicts.

Where the GIM8115-36 runs out (`sweep_bend.py`, full inverse-dynamics feedforward, same bend with shorter T):

| T (s) | peak ref speed (rad/s) | 0 kg: required torque (Nm) / demand over capability / time on voltage limit / peak err (mrad) | 3 kg: same |
|---|---|---|---|
| 1.0 | 1.96 | 77 / 0.34 / 0 / 3.5 | 145 / 0.63 / 0 / 5.5 |
| 0.8 | 2.45 | 87 / 0.44 / 0 / 3.9 | 167 / 0.80 / 0 / 6.1 |
| 0.7 | 2.80 | 96 / 0.76 / 0 / 4.2 | 187 / 1.38 / 5.6 % / 18.6 |
| 0.6 | 3.27 | 109 / beyond no-load speed / 6.7 % / 36 | 217 / beyond no-load speed / 9.7 % / 82 |
| 0.5 | 3.93 | 131 / beyond / 17 % / 127 | 267 / beyond / 18 % / 205 |

"Demand over capability" is the rigid-body torque (inverse dynamics plus Coulomb friction) divided by the torque the
voltage-limited envelope can give at that speed; above 1 the envelope cannot supply it, and "beyond no-load speed" means the
reference speed exceeds the roughly 3.2 rad/s back-EMF limit.

## Findings

**Does one GIM8115-36 cover the torso bend at 48 V?** Yes for every requested case, on both torque and speed. At 3 kg and
T = 1 s, the hardest case, the torso needs 145 Nm including friction at a peak of 2.0 rad/s, uses 67 % of the current limit
(154 Nm delivered of the 228 Nm listed), never touches the voltage circle or the current limit, and tracks within 14 mrad
with plain gravity feedforward (5.5 mrad with full inverse dynamics). The 0 kg and T = 2 s cases have more margin. Torque is
not the first thing to go: speed is. The back-EMF limit near 3.2 rad/s starts to bite when the bend is shortened to about
0.7 s with 3 kg (peak speed 2.8 rad/s, 5.6 % of the time on the voltage limit) and at about 0.6 s with 0 kg; below that
tracking fails quickly, and at 0.6 s with 3 kg the current limit is reached too. At the requested T = 1 s the peak speed is 62 % of the no-load speed and, at 3 kg, the torque demand is 63 % of what the
envelope can give; do not read the sweep as a design rule for faster moves without
re-checking with a bus above 48 V or the kv-derived back-EMF constant (which gives a higher no-load speed, 36 rpm).
Scenario C is easy for the torso (peak 79 Nm at 3 kg) because the bend is slower and shallower.

**Stock DM-J10422P model.** It never saturates in these runs: its 400 Nm force limit is far above the 154 Nm peak and the model has no
speed limit at all (the XML comment gives 12.6 rad/s but nothing enforces it), so it cannot reveal a speed shortfall.
Its tracking is 30 to 62 mrad with the shipped controller, worse than the GIM runs, but that is a statement about the
controller structure (no velocity reference, no feedforward, kv lag of 20 ms), not about the actuators. With
v_ref = 0 the GIM layer is worse than the stock servo (80 to 160 mrad) because the torso kd/kp of 0.08 s lags more than the
stock 0.02 s. The honest comparison is the saturation and torque rows, not the error rows.

**Arm joints.** No GIM arm joint saturates in A, B or C (largest peak iq/i_max: Left Arm 7 0.41, Left Arm 4 0.25, Arm 1 0.20,
all at 3 kg). In scenario C the Arm 1 swing peaks at 2.1 rad/s and 34 Nm (iq/i_max 0.15, 8 mrad peak error); Arm 2 barely moves.
The stock arm servos do hit their force limits at 3 kg: Arm 7 sits at its 7 Nm limit for 77 to 96 % of the time in every 3 kg
run, because the point payload at the palm origin is 0.255 m from the J7 axis and needs 7.5 to 8.9 Nm statically; Arm 1 reaches
its 40 Nm limit for 1 to 41 % of the time during the torso bend at 3 kg (the shoulder load changes with the torso angle; the GIM
J1 peaks at about 46 Nm). The GIM motors have the headroom (J7 uses 41 % of 30.6 Nm), so this is a real difference between the
stock arms and the GIM arms for payload work, with the caveat that the palm-origin payload convention is what loads J7.

## Things that did not go as hoped or that you should not over-read

- Only the GIM torso and arm actuators are physical models; the lift droops about 4 mm under load (ends at 0.631 m of 0.635 m)
  because that is the stock servo, and it is not part of the question.
- The hold at -1.0472 is exactly on the torso joint limit. The GIM holds 0.6 to 0.8 mrad inside it, so the limit never carries
  load for the GIM, but any tracking error that would overshoot toward the limit is clipped in the bend metrics.
- Gravity feedforward uses the nominal hand and neck pose. The stock wrist servo (kp = 1) sags 0.02 to 0.12 rad under payload;
  the gate uses the actual settled pose, and the effect on the torso is small, but the feedforward is not exact.
- Not modelled: thermal limits (holding 3 kg at the bend limit draws about 8.7 A in the torso, roughly 56 W of copper loss, with no
  temperature model), sensor noise, bus sag, gearbox compliance (rigid mode only, as decided), contacts, and base motion. The
  GIM4310 runs at 48 V against a 40 V rating.
- Most of the gearbox and rotor numbers (J_rotor, friction, eta, k_g) are ASSUMED, as in the OpenArm layer, and
  the 2 % gate does not validate them; it only checks that the torque chain is consistent.
- The requested "regen on the return stroke" does not occur for this pose: gravity assists the forward bend, so regeneration is on the
  forward stroke. Both are reported above.
- Gains were chosen for stability and reasonable tracking, not optimised; torso tracking error scales with them.

## Optimised actuators vs stock

This section tests, in the same rig, whether the per-joint actuators found by the per-joint design study (NdFeB-optimal and
ferrite spoke-optimal, under several constraint choices) would work like the stock GIM actuators on MABEL. The design study is the
`robot-joint-rotor-study` branch of Motor_Control_NN_Pipeline at commit db5e76c (read only). Everything below is simulation of
that study's models, not a measured actuator.

Scope note: the swerve drive in the base was not changed in any variant because it was not in the design scope of this feasibility study (only arm and torso actuators were swapped; the base is welded in these runs).

### Files added (existing files and results are untouched)

| file | purpose |
|---|---|
| `designs/export_params.py` | ran on the Inspiron (`~/scratch/pj_export/`), reuses the per-joint code (`actuator.py` Motor class) to export rig parameters |
| `designs/*.json` | 12 actuator sets (`main_*`, `mass_50_*`, `mass_100_*`, `size_*`, `backdrive_*`, `qdd_*`, each `ndfeb` and `ferrite`) with provenance; `stock_reference.json` for the stock references |
| `rig2.py` | `SetRig`: the same rig with any actuator set loaded from the JSON, actuator mass change per joint, torso brake lock |
| `scen2.py` | scenarios (B with brake, C, new arm-only reach D, backdrive test), per-joint and per-group metrics |
| `run_sets.py`, `report_sets.py`, `sens_stock_inertia.py` | driver (parallel), tables/plots, sensitivity run of the stock set with the study's inertia |

### Pass rule (written before any set was run, not changed afterwards)

A set "works like stock" if, in every rule scenario, for every one of the 15 actuated joints:

1. the joint does not saturate for more than 2 % of the scenario's active time. Saturated means the voltage circle clipped the
   commanded current or the commanded current exceeded i_max (the rig's `vflag | iflag`). Active time is everything after the
   1 s start-up settling; for the torso, time when the brake is engaged is left out (the motor is not commanded then).
2. the peak tracking error is at most max(1.5 x the stock set's peak error for the same joint, scenario and payload, 10 mrad).

Rule scenarios, each at 0 kg and 3 kg per hand: B bend T = 2 s, B bend T = 1 s (worst arm pose, torso brake during the 1 s hold),
C combined lift + torso + arm reach, D arm-only reach. "Stock" means the GIM set (GIM8115-36 on J1, J2, torso; GIM8108-36 on J3,
J4; GIM4310-40 on J5-J7) with the same mass bookkeeping as the designs. Reference-only runs (not in the rule): B without brake,
D_fast (a 5 rad/s swing that exceeds the GIM8115 no-load speed), the backdrive test, and a small-step gain check.

D definition: both arms, elbows straight, all other arm joints 0, torso 0 and braked. Left Arm 1 goes from 0.24 rad to -0.40 rad
(palm level with the shoulder, straight forward) in 0.6 s, holds 1 s, returns in 0.6 s (right arm mirrored); peak 2.0 rad/s. D_fast
goes from 1.20 rad (hand hanging) to -0.40 rad in 0.6 s (peak 5.0 rad/s).

### How the sets are built

* **Electrical parameters** come from the per-joint code, not invented. Peak phase current `i_max = I_peak`; torque constant
  `kt_m` = motor torque at the design's peak current density divided by `I_peak`; flux linkage `lam = N_turns * psi_d(J_p)` from the
  8 mm FEMM table scaled by (L - dL)/8 (this equals `kt_m / (1.5 pp)` within 0.6 % except the ferrite <=50:1 torso, where the MTPA
  angle is not zero and the two differ by 8 %); phase resistance `R = Pcu_peak / (1.5 I_peak^2)`; inductance `L = N^2 Lq8 (L-dL)/8`;
  pole pairs from the motor name. At the study's design point (peak torque at 3 rad/s, 2 rad/s torso) the rig's voltage circle
  needs 96 to 99.7 % of the 48 V limit (`design_point_V_over_vmax` in each JSON), so the export reproduces the code's own sizing.
* **Gearbox, per joint, inside every set** (JSON field `gearbox`): type (planetary1, planetary2, strain_wave), ratio, efficiency,
  mass `m_gear`, backlash, stiffness. Backlash and stiffness are exported but not used (the rig is rigid-transmission only, as
  before). Efficiency enters the rig as the load-dependent gear friction `(1 - eta)|N kt_m iq|`, and `backdrive_Nm` (the gearbox
  start torque in the study) enters as the output-side Coulomb friction `tau_c`. The ratio enters the back-EMF (`pp N w`), the
  torque (`N kt_m`) and the reflected rotor inertia `J_rotor N^2` (MuJoCo `dof_armature`). **The per-joint code has no gearbox
  inertia of its own**, so none is added: `J_reflected = J_rotor N^2` is the rotor alone. Viscous friction is not in the study; it
  is ASSUMED with the same b/tau_c ratio as the stock GIM of that joint class.
* **Mass change.** At each actuated joint a mass `dm = m_set - m_stock` is added at the child body origin. `m_set` is the whole
  actuator mass of the study (it contains the gearbox `m_gear`, housing, magnets, copper and board); `m_stock` is the mass of the actuator that MABEL's
  MJCF was built with: DM-J8009P 0.896 kg (J1, J2), DM-J4340 0.362 kg (J3, J4), DM-J4310 0.30 kg (J5-J7), DM-J10422P 2.7 kg (torso).
  GIM sets use the published GIM masses with driver (0.96, 0.76, 0.3645 kg; torso GIM8115 0.96 kg). `dm` can be negative (the
  designs are lighter than the stock Damiao at J1, J2 and the torso). The body's mass, centre of mass and inertia are
  recombined exactly by the parallel-axis rule; where removing a lump at the origin would give a non-physical inertia (the torso
  link), mass and inertia are instead scaled by m2/m1. The feedforward model is a copy made after the edit.
  The old rig kept the Damiao masses in the links for the GIM runs; `stock_gim_nomass` reproduces that and reproduces the earlier
  numbers (torso B 3 kg T = 1 s: 13.8 mrad, iq/i_max 0.67, 154 Nm).
* **Torso brake.** In B the brake locks the torso (a MuJoCo joint equality, torso held where it is) for the 1 s hold; motor
  current is forced to zero, except for the last 50 ms before release, when current is restored with the brake still engaged
  so the motor takes the load before the release. In D the torso never moves and is braked throughout. In C no brake is used.
  The B runs without brake are reference only.
* **Controller** is unchanged for every set: MABEL arm gains, torso kp 4200 / kd 335, gravity feedforward, v_ref = reference
  velocity, 1 kHz, 1 ms delay, eta_hat 0.95, 48 V.
* **Joint groups.** The main set uses each joint's own J1..J7 and Torso pick. For the alternatives cases only J1, J3, J5 and Torso
  exist: the J1 case is applied to Arm 1 and Arm 2, J3 to Arm 3 and 4, J5 to Arm 5, 6 and 7. Where an alternatives case is
  infeasible for the torso (QDD NdFeB and ferrite, backdrive-case ferrite) the main torso pick is kept.

### Results

Run folder: `results/20261105_112812_sets/` (`metrics.json`, `rule.json`, `comparison.md` with every table, `summary_<scenario>.png`
bar charts of peak iq/i_max and peak error per joint group, `torso_B_T1_0kg.png` and `torso_B_T1_3kg.png` with torso q, iq/i_max,
total DC bus power and error for every set). 214 runs, 4.4 s each. Torso bend T = 1 s with 3 kg per hand, brake during the hold
(torso numbers are over the moving part; bus power is the sum over all 15 actuated joints):

| set | robot mass kg | torso peak err mrad | torso peak iq/i_max | torso peak speed / no-load | torso saturated % | worst arm group err mrad | D 3 kg Arm 1-2 err mrad / iq/i_max | bus peak motoring / regen W | passes all |
|---|---|---|---|---|---|---|---|---|---|
| stock GIM | 68.2 | 14.2 | 0.70 | 0.62 | 0 | 17.1 | 18.2 / 0.17 | 427 / -126 | reference |
| NdFeB main (64:1) | 66.7 | 15.1 | 0.84 | 0.79 | 0 | 17.0 | 20.3 / 0.62 | 344 / -149 | yes |
| ferrite main (64:1) | 70.2 | 16.7 | 0.93 | 0.63 | 0 | 17.2 | 23.5 / 0.69 | 450 / -143 | no (D 0 kg, marginal) |
| NdFeB <=50:1 | 67.5 | 15.2 | 0.86 | 0.77 | 0 | 17.0 | 19.9 / 0.62 | 361 / -149 | yes |
| ferrite <=50:1 | 71.3 | 15.8 | 0.92 | 0.54 | 0 | 15.9 | 22.4 / 0.67 | 398 / -136 | no (D 0 kg, marginal) |
| NdFeB <=100:1 | 65.5 | 15.5 | 0.84 | 0.84 | 0 | 17.1 | 21.4 / 0.64 | 318 / -159 | yes |
| ferrite <=100:1 | 68.3 | 17.0 | 0.91 | 0.68 | 0 | 17.3 | 26.0 / 0.73 | 395 / -157 | no (D 0 kg, marginal) |
| NdFeB fits envelope | 66.8 | 15.2 | 0.85 | 0.80 | 0 | 17.1 | 20.7 / 0.63 | 347 / -152 | yes |
| ferrite fits envelope | 70.6 | 16.4 | 0.93 | 0.69 | 0 | 16.6 | 22.9 / 0.68 | 398 / -158 | no (D 0 kg, marginal) |
| NdFeB ratio+inertia <= ref | 70.4 | 15.3 | 0.89 | 0.71 | 0 | 17.0 | 19.6 / 0.62 | 439 / -136 | yes |
| ferrite ratio+inertia <= ref | 77.6 | 60.4 | 1.00 | 0.73 | 14.8 | 23.5 | 22.4 / 0.71 | 667 / -148 | no (torso, current) |
| NdFeB QDD (<=10:1) | 76.1 | 20.5 | 1.00 | 0.80 | 4.6 | 18.6 | 18.0 / 0.61 | 550 / -136 | no (torso, current) |
| ferrite QDD (<=10:1) | 88.6 | 246.0 | 1.00 | 0.78 | 26.0 | 45.2 | 20.0 / 0.67 | 701 / -136 | no (torso and all arm groups) |

Mass change method and the gearbox are described above; the per-joint `dm` values are in `comparison.md`. Pass/fail per scenario and
the joints that break the rule are in `comparison.md` ("Pass / fail per set", "What breaks the rule").

**Which sets work like stock.** The five NdFeB sets of the lightest-design cases (main, <=50:1, <=100:1, fits envelope, ratio and
inertia no worse than reference) pass the rule in every scenario, torso brake included. The NdFeB QDD set fails on the torso at 3 kg,
T = 1 s (4.6 % of the time on the current limit). No ferrite set passes. The four lightest-design ferrite sets (main, <=50:1,
<=100:1, fits envelope) fail only the 0 kg arm reach D, on Arm 1 of each arm and only on tracking (12.5 to 16.1 mrad against a
12.4 mrad limit; the stock error there is only 8.3 mrad, so 1.5 x stock is tight). They never saturate. With the study's own,
larger reflected inertia for the stock GIM8115 (`sens_stock_inertia.py`: stock error 9.9 mrad, limit 14.9 mrad) the 13.5, 12.5 and
12.9 mrad cases would pass and only the 16.1 mrad case (<=100:1) would still fail, so these four are borderline, not clear failures.
The two ferrite sets with heavy shoulders fail clearly (below).

**What limits the failures.**
* Torque (current limit) on the torso, not speed. The torso was sized for the stock arm masses. The heavier-arm sets add mass at the
  shoulders and elbows (ferrite ratio+inertia +0.75 kg on Arm 1 and Arm 2, ferrite QDD +2.6 kg each, NdFeB QDD +1.0 kg), which raises
  the load the torso lifts on the return stroke. The torso (the main pick in these sets for ferrite QDD and ferrite ratio+inertia,
  because the study found no feasible alternative) then sits on i_max for 15 % and 26 % of the active time and loses the trajectory
  (60 and 246 mrad). In the ferrite QDD set the arm joints also exceed 1.5 x stock error at 3 kg, T = 1 s, because they ride a torso
  that is off the reference.
* Speed does not fail the rule in B, C or D: peak torso speed is 54 to 84 % of the no-load speed, against 62 % for stock. The NdFeB
  main torso has a lower no-load speed (2.55 rad/s against 3.22) and runs at 79 % of it without clipping.
* In the reference run D_fast (5 rad/s swing, beyond the reach of any of these actuators) every set including stock saturates on
  Arm 1 and Arm 2 on voltage for 23 to 31 % of the time (peak speed 1.02 to 1.05 times no-load), so the shoulder speed limit is the
  same for all of them. The designs also hit i_max on Arm 3 to 7 for 1 to 12 % of the time where stock does not, because they are
  sized to the study's requirement (peak 38.7 Nm on J3 and J4, 18.5 Nm on J5 to J7) and the GIM has more torque headroom.
* Headroom: at peak torque and speed the designs ask for 96 to 99.7 % of the 48 V limit by construction. In D at 3 kg the NdFeB designs
  use 0.6 to 0.7 of i_max on Arm 1 to 4 (stock 0.17 to 0.25), the ferrite designs 0.7; peak torso current is 0.84 to 0.93 of i_max
  against 0.70 for stock.

**Energy.** Torso hold with the brake costs essentially nothing (motor current zero; only the last 50 ms before release carry
current, 1 to 4 % of i_max). Without the brake the 1 s hold at 3 kg costs 64 J of copper loss in the stock torso, 15 to 48 J in the
NdFeB sets and 31 to 84 J in the ferrite sets (reference runs only). Peak total bus power for the T = 1 s, 3 kg bend is 318 to 450 W
for the lightest-design sets against 427 W stock; regen peaks 136 to 159 W against 126 W.

**Backdrive test** (zero current, torso locked, gravity off, 10 N forward push on each palm). In every set except one, every joint
breaks away (the joint torque from the push is 5.7 to 15.6 times the Coulomb friction), so with 10 N no set is locked. The
exception is the ferrite <=50:1 set: its strain-wave 160:1 gearbox (4.8 Nm start torque on Arm 3 to 7) gives a breakaway ratio of
0.9 at Arm 4 and the forearm stays put. What differs elsewhere is the response. Arm 1 rotation at 0.25 s: stock 0.11 rad, NdFeB main
0.09, ferrite main 0.07, ferrite <=100:1 0.06, NdFeB QDD 0.10, ferrite QDD 0.07 (reflected inertia 0.156, 0.36, 0.61, 0.87, 0.092,
0.13 kg m2). The high-ratio designs move 18 to 45 % less than stock in the first quarter second because their reflected rotor inertia is
2 to 5.5 times larger; the NdFeB QDD set moves about as much as stock. The 64:1 planetary designs have Coulomb friction of 0.56 to
0.98 Nm on J1 to J4, about the same as the GIM (0.60 to 0.80 Nm), so the backdrivability difference the rig shows comes mainly from
rotor inertia. The friction model is the study's gearbox start torque only; motor cogging is not included.

**Caveats.**
* Iron loss and cogging are not in the per-joint models, so copper-loss energy and bus power are low for every design (the study's own
  actuator efficiency omits them too) and backdrive friction is the gearbox term only.
* Assumed values: viscous friction (same b/tau_c ratio as the GIM class), eta_hat 0.95 and the controller gains (unchanged from the
  stock rig, not retuned per set), the 50 ms brake pre-load, and the `m_stock` masses in the MJCF (published Damiao values; the old rig
  assumed 0.80, 0.36, 0.30 kg). The stock set therefore differs slightly from the earlier numbers (the torso link loses 1.74 kg in
  the stock GIM set, because the GIM8115 replaces the 2.7 kg DM-J10422P); `stock_gim_nomass` reproduces the earlier README numbers.
* The per-joint code has no electrical parameters for the stock actuators and models the GIM8115 reflected inertia at 0.318 kg m2
  against the rig's assumed 0.156 (GIM8108 0.170 against 0.130; GIM4310 equal). Ratio, listed torque and bare mass match exactly.
  See `designs/stock_vs_gim_params.md`.
* kt for the designs is the torque per ampere at the peak operating point; for the ferrite spokes the torque per ampere at low
  current is up to 6 % higher (saturation), so holding currents are slightly over-estimated.
* Gearbox backlash and stiffness are exported but the rig is rigid-transmission only. The per-joint code has no gearbox inertia.
* The alternatives cases exist only for J1, J3, J5 and Torso and are applied to the whole group. The J1 pick on Arm 2 delivers
  99.6 % of the study's J2 peak torque, the J5 pick on Arm 6 and 7 101 %: negligible. The study's "fixed 66 mm outline" case was not run.
* The torso in the QDD sets (both magnets) and in the ferrite ratio+inertia set is the main pick, so those failures also reflect a torso
  that was never redesigned for heavier arms.
* The D start (0.24 rad, peak 2 rad/s) was fixed before running. The hanging-to-horizontal swing needs 5 rad/s, which the stock
  GIM8115 cannot reach (no-load 3.2 rad/s); it is reported as D_fast and is not in the rule.

## Optimised actuators vs stock, rerun with CAD gearbox inertia and actuator tensors

The Inspiron thread built CAD for every pick (`robot_rotor/cad/per_joint/index.json`, read only). The first run above used the
per-joint code's rotor-only inertia and a point mass at each joint. This rerun uses the CAD data. Scope note: the swerve drive in
the base was not changed in any variant because it was not in the design scope of this feasibility study (only arm and torso
actuators were swapped; the base is welded in these runs).

* **Data.** `designs/*.json` now carry a `cad` block per joint (index key, mass, COM, inertia tensor about the COM, CAD rotor
  inertia, CAD reflected inertia, gearbox mass, COM, tensor, motor-side and reflected inertia). The old per-joint values stay in the
  same files under `J_rotor`, `J_reflected` and `cad.rotor_inertia_model_kgm2`, and the first-run files are kept unchanged in
  `designs/v1_model_inertia/`. All 8 joints of all 12 sets matched an index entry (case names, gear, ratio, stack and mass checked).
  For alternatives applied to a group, the group's J1, J3 or J5 entry is used for each joint of that group.
* **Armature.** The MuJoCo armature of each actuated joint is now the CAD reflected inertia (CAD rotor plus gearbox input, reflected
  to the output), not `J_rotor N^2` from the model. Examples (output side): NdFeB main J1 0.360 to 0.438 kg m2, ferrite main J1 0.611 to 0.552,
  ferrite <=50:1 J3 160:1 strain wave 0.678 to 2.415 (the study's earlier number for the strain wave was 0.26 to 0.68 at J5).
* **Mass and tensor.** The CAD mass equals the study's mass for every pick, so only the inertia changes. Each actuator is placed as one lump
  of mass `m_set - m_stock` at the child body origin that also carries the CAD inertia tensor about its COM. The CAD actuator is axisymmetric
  about its axis, so the orientation is fixed by putting that axis on the joint axis (every MABEL joint axis is the child body's local z). The
  COM offset along the axis (up to about 30 mm) has no sign reference in the index, so the COM is kept at the child body origin (the point-mass
  position of the first run); only the tensor is new. The body mass, COM and inertia are recombined exactly. The torso link no longer needs the
  mass-scaling fallback because the added tensor keeps its inertia physical. Stock actuators have no index entry, so the stock set is
  unchanged (point-mass bookkeeping and the rig's assumed J_rotor).
* **Rule.** Unchanged (written before the first run). Run folder `results/20261105_114636_sets_cad/` (same files as the first run) and
  `cad_vs_model.md` there with every difference.

**What changed versus `20261105_112812_sets`.** Three pass/fail flips, all in the borderline 0 kg arm reach D, all on Arm 1 tracking by 0.2 to 0.3 mrad:
ferrite <=50:1 and ferrite fits envelope now pass (they were failing at 12.5 and 12.9 mrad), and NdFeB <=100:1 now fails (12.7 mrad against a 12.4 mrad limit).
The set-level results become: NdFeB main, <=50:1, fits envelope and ratio+inertia pass; ferrite <=50:1 and ferrite fits envelope pass; NdFeB <=100:1 fails
on D only; ferrite main and ferrite <=100:1 still fail on D only (12.9 and 15.2 mrad); ferrite ratio+inertia, both QDD sets still fail on the torso. The pass
or fail of a set in D at 0 kg therefore moves with 0.2 to 1 mrad, which is smaller than the uncertainty of the inertia itself; the earlier statement that these cases are borderline stands.
* Torso peak iq/i_max moves by at most 0.01 for the pass sets (NdFeB main 0.84 to 0.85, ferrite main 0.93 to 0.92), the torso error by at most 0.3 mrad,
  and the NdFeB QDD torso saturation goes from 4.6 % to 5.9 % (error 20.5 to 23.6 mrad). Ferrite ratio+inertia 14.8 to 14.5 %, ferrite QDD 26.0 to 25.9 %.
* Arm errors in D at 3 kg change by 0.2 to 1.2 mrad (largest: NdFeB <=100:1 Arm 1-2, 21.4 to 22.6 mrad, because its Arm 1 reflected inertia goes from 0.485 to 0.599).
* Backdrive (rotation at 0.25 s): Arm 1 moves 0.01 rad less for the NdFeB sets whose inertia rose (main 0.09 to 0.08) and is unchanged elsewhere; Arm 4 of the NdFeB main set 0.15 to 0.14.
  The ferrite <=50:1 elbow and wrist still do not move (breakaway ratio 0.9, now with 2.4 kg m2 reflected at Arm 3). The conclusion stays: the high-ratio designs resist a push more than stock through inertia.
* NdFeB 64:1 reflected inertia rises about 20 to 25 % with the gearbox included (J1 0.360 to 0.438, J3 0.170 to 0.217), and falls for the ferrite 64:1 and 100:1 picks (0.611 to 0.552, 0.867 to 0.787) because the CAD rotor is lighter than the model rotor for the spokes.

## Simulink bus check with the optimised actuators (X1 one-arm model)

Scope note: the swerve drive in the base was not changed in any variant because it was not in the design scope of this feasibility study (only arm and torso actuators were swapped; the base is welded in these runs). The X1 model has no base at all, only one right arm.

The X1 support-board and inverter Simulink model (`.../x1_redesign/system_model`, with the diode fix and its fresh baseline) was not edited.
It was copied (without `results/`, `slprj/` and the backup folders) to `simulation/x1_variants/<set>/` for `stock`, `ndfeb_main` and `ferrite_main`,
together with a copy of `build/openarm-v2-load/` one level up so that the model's relative URDF path still resolves. In the two design copies only the actuator block of
`openarm_params.m` differs (the stock copy is byte-identical to the original). In each copy `build_openarm_system()` was run, then `check_openarm_setup()` (passed for all three),
then `run_openarm(name, timeout_s=900)` for `lift6`, `drop6` and `drop6_auxfail`, in separate MATLAB R2026a processes started by `x1_variants/runner_<set>.m`
(`launch.ps1`). Results are in `x1_variants/<set>/results_x1/20261105_115114/` (`summary.json`, one `.mat` per scenario, `log.txt`), and `x1_variants/summary.md`.

**Parameter mapping** (J1 to J7 of the right arm, the main set's own J1..J7 entries): gear ratio N (64:1 on every joint), efficiency (0.9025), torque constant
`kt_m` (motor Nm per peak-phase A at the design's peak point), pole pairs, flux linkage `lambda`, phase R and L, current limit `i_max = I_peak`, torque limit
`tau_max = N kt_m eta i_max` (the design's peak torque), rotor inertia `J_rotor` = CAD reflected inertia (rotor + gearbox input) divided by N^2, Coulomb friction (study gear start torque), viscous friction (ASSUMED, same
ratio to Coulomb as the GIM class), whole-actuator mass, and `w_max` (the model uses it for the speed screen; the stock listed speed is replaced by the no-load speed at 48 V computed from `lambda`: 3.5 to 3.8 rad/s on J1 to J7 for NdFeB).
Units are those of the file (output Nm, per-phase ohm and henry, Wb). The model adds mass only through `dm = max(motor_mass - stock, 0)` at each child body with its own assumed stock masses (0.80, 0.36, 0.30 kg), so a
design lighter than that stock mass gets no reduction here (NdFeB J1 and J2 at 0.77 kg), unlike the MuJoCo runs; the model has no inertia tensor input, so the CAD tensor is not used and the armature carries the inertia. Nothing else in the model was changed.

| scenario | variant | completed, full scenario | solver warning free | peak bus V | brake energy J | source peak A | peak regen W | peak tracking error deg (joint) | joints at current limit | hot-swap trips |
|---|---|---|---|---|---|---|---|---|---|---|
| lift6 | stock | yes | yes | 57.15 | 26.4 | 5.1 | -47 | 1.20 (J6) | none | 0 |
| lift6 | NdFeB main | yes | yes | 57.15 | 24.9 | 5.1 | -45 | 0.09 (J6) | none | 0 |
| lift6 | ferrite main | yes | yes | 57.15 | 26.0 | 5.1 | -47 | 0.07 (J6) | none | 0 |
| drop6 | stock | yes | yes | 57.15 | 26.6 | 5.2 | -47 | 1.13 (J6) | none | 0 |
| drop6 | NdFeB main | yes | yes | 57.15 | 24.9 | 5.2 | -45 | 0.09 (J6) | none | 0 |
| drop6 | ferrite main | yes | yes | 57.15 | 26.0 | 5.4 | -47 | 0.07 (J6) | none | 0 |
| drop6_auxfail | stock | yes | yes | 61.19 | 36.8 | 5.2 | -47 | 1.13 (J6) | none | 0 |
| drop6_auxfail | NdFeB main | yes | yes | 61.19 | 34.3 | 5.2 | -45 | 0.09 (J6) | none | 0 |
| drop6_auxfail | ferrite main | yes | yes | 61.19 | 34.9 | 5.4 | -47 | 0.07 (J6) | none | 0 |

**Reading.** The stock copy reproduces the diode-fix baseline exactly (57.15 V / 26.4 J on lift6, 57.15 V / 26.6 J on drop6, 61.19 V / 36.8 J on drop6_auxfail), so the rebuild path is sound.
With the optimised arm actuators every run completes the full scenario without solver warnings, no hot-swap trips, peak bus voltage is the same (the chopper clamps it at 57.15 V, and at 61.19 V with the aux supply lost),
source peak current is 5.1 to 5.4 A against 5.1 to 5.2, regen peak is 45 to 47 W against 47 W, and brake energy is 6 to 7 % lower (NdFeB) or 1 to 5 % lower (ferrite) than stock. Only J1, J4 and J6 carry load in these scenarios, none reaches its current limit with the designs'
smaller i_max (J1 peak iq 4.0 to 4.5 A of 6.3 to 6.7 A), and all screens pass for all three variants. Tracking is better for the designs (0.07 to 0.09 deg against 1.1 to 1.2 deg on J6, where the stock wrist sags
under the 6 kg payload with the model's controller), J1 peak speed is 0.80 to 0.85 of the design's no-load speed against 0.78 of the stock listed speed. The bus side of the X1 board is therefore not sensitive to which of these actuators are fitted in this one-arm model; this does not test the drop6_nobrake case, the multi-joint cases or a two-arm robot.
Caveats: the model's controller is the original computed-torque loop (not retuned), iron loss and cogging are absent, and the J5 to J7 stock actuators are run at 48 V as in the original model.

## Stock reflected inertia: three values, and what it changes

Scope note: the swerve drive in the base was not changed in any variant because it was not in the design scope of this feasibility study (only arm and torso actuators were swapped; the base is welded in these runs).

**Inertia source.** No verified SteadyWin rotor inertia exists (the manual page is CAPTCHA-blocked; only an unverified reseller listing of 1200 g cm^2 for the GIM8115 and 800 g cm^2 for the GIM8108). The nominal stock values are now a geometric estimate made on 5 Nov 2026
from rotor geometry (81 mm rotor OD, 2 mm back iron, 2.5 mm magnets at 85 % fill, 2 mm aluminium end cap, stack 15 mm and 8 mm): GIM8115 1.9e-4 kg m2 motor side, 0.25 kg m2 reflected at 36:1; GIM8108 1.1e-4, 0.14 kg m2 (+/-30 %). GIM4310 is unchanged (0.032). The bounds already run are 0.156/0.130 (reseller-based) and 0.318/0.170 (per_joint surrogate).
The torso uses the GIM8115 value. The design sets are the CAD-inertia run (`20261105_114636_sets_cad`) and do not change; only the stock reference errors, and so the rule's 1.5 x limit, change. The stock runs for the nominal value are in `results/20261105_120904_stock_nominal/`, the high value in `results/20261105_113201_stock_inertia/`.

Verdict of each set (pre-registered rule, unchanged), full table with the failing joints in `results/20261105_114636_sets_cad/three_stocks.md`:

| set | stock 0.156/0.130 | stock 0.25/0.14 (nominal) | stock 0.318/0.170 | changes with stock value |
|---|---|---|---|---|
| NdFeB main, <=50:1, fits envelope, ratio+inertia | pass | pass | pass | no |
| ferrite <=50:1, ferrite fits envelope | pass | pass | pass | no |
| ferrite main | FAIL (D 0 kg Arm 1, 12.9 vs 12.4 mrad) | pass | pass | yes |
| NdFeB <=100:1 | FAIL (D 0 kg Arm 1, 12.7 vs 12.4 mrad) | pass | pass | yes |
| ferrite <=100:1 | FAIL (15.2 vs 12.4) | FAIL (15.2 vs 13.8) | FAIL (15.2 vs 14.8) | no |
| ferrite ratio+inertia | FAIL (torso B T=1 s, 0 kg tracking and 3 kg current) | FAIL | FAIL | no |
| NdFeB QDD | FAIL (torso 6 % on current limit) | FAIL | FAIL | no |
| ferrite QDD | FAIL (torso and arms) | FAIL | FAIL | no |

At the nominal stock value the stock D 0 kg Arm 1 error is 9.2 mrad (limit 13.8) against 8.3 (12.4) and 9.9 (14.8). Two verdicts change (ferrite main and NdFeB <=100:1, both from fail to pass); the six clear failures do not, and ferrite <=100:1 fails at all three values.
At the nominal value the sets that pass every scenario are NdFeB main, <=50:1, <=100:1, fits envelope, ratio+inertia, and ferrite main, <=50:1, fits envelope; the failing ones are ferrite <=100:1, ferrite ratio+inertia and both QDD sets.
The borderline sets (ferrite main, NdFeB and ferrite <=100:1) differ from the limit by 0.3 to 1.4 mrad, which is smaller than the +/-30 % uncertainty of the stock inertia, so the rig cannot settle them. **A bench measurement of the real GIM8115 and GIM8108 reflected inertia (and a step-response check of the D 0 kg reach) is needed to decide the borderline sets.**

### Simulink stock copy at the nominal inertia

In `x1_variants/stock/openarm_params.m` only `J_rotor` of J1 to J4 changed (0.25/36^2 and 0.14/36^2, GIM4310 unchanged; the old line is kept as a comment); the copy was rebuilt, `check_openarm_setup()` passed, and lift6, drop6 and drop6_auxfail were rerun (`x1_variants/stock/results_x1/20261105_120947/`; the earlier stock run stays in `20261105_115114`). All three complete the full scenario, solver-warning-free, all screens pass.

| scenario | stock 0.156/0.130 | stock 0.25/0.14 (nominal) |
|---|---|---|
| lift6 peak bus V / brake J / source A / regen W / worst tracking deg | 57.15 / 26.4 / 5.1 / -47 / 1.20 | 57.15 / 27.0 / 5.1 / -47 / 1.28 |
| drop6 | 57.15 / 26.6 / 5.2 / -47 / 1.13 | 57.15 / 26.9 / 5.2 / -47 / 1.23 |
| drop6_auxfail | 61.19 / 36.8 / 5.2 / -47 / 1.13 | 61.19 / 36.6 / 5.2 / -47 / 1.23 |

The bus voltage, source current and regen do not move; brake energy changes by +2.4 %, +1.0 % and -0.6 %, and tracking by 0.08 to 0.10 deg. Against the new stock, the NdFeB main brake energy is 6 to 8 % lower (24.9, 24.9, 34.3 J) and the ferrite main 3 to 5 % lower (26.0, 26.0, 34.9 J); the earlier conclusion (bus side insensitive to the swap) stands. The design variants were not rerun (they do not use the stock inertia).

## Simulink bus check on the final coupled picks (6 Nov 2026)

Sets: every coupled set that passed the pre-registered MuJoCo rule at nominal stock inertia, plus qdd_ferrite and backdrive_ferrite as the heavy extremes. Each ran lift6, drop6 and drop6_auxfail in its own copy of the fixed X1 model (`simulation/x1_variants/c_<set>`), arm J1-J7 only, one right arm. Table: `simulation/x1_variants/summary_coupled.md`.

- All 51 runs completed the full scenario with no solver warnings, and every screen passes.
- Peak bus is set by the chopper thresholds, so it does not change: 57.14 to 57.16 V, and 61.19 to 61.21 V with the aux supply lost. It is not a useful discriminator between sets.
- Brake energy is the number that moves. Stock is 27.0 / 26.9 / 36.6 J. Lightest-type sets are 24.4 to 27.1 J (33.7 to 37.6 J with aux lost). Quasi-direct-drive ferrite is the worst at 32.6 J and 44.6 J, about 21 % above stock.
- Source peak stays between 5.1 and 6.1 A. No J1 current limit is reached (peak iq 3.8 to 6.4 A).
- Limits of this check: one arm only, no torso, no two-arm or multi-joint case, no thermal model. The model only adds mass (`max(motor_mass - stock, 0)`), so lighter NdFeB joints get no weight credit, and `w_max` is the 48 V no-load speed from lambda.

The swerve drive in the base was not changed in any variant, because it was not in the design scope of this feasibility study.

`variants/<set>.xml` holds one MuJoCo robot model per set, built by `export_variants.py` from MABEL's unmodified `mabel_full.xml` (credit: Qilong Cheng, Rooholla Khorrambakht, Mohamed Yassine Kabouri, Quang Nam Nguyen, Luca Macesanu, Haoran Zhou, Yipeng Wang, Ludovic Righetti; arXiv 2611.04083). In the NdFeB sets the torso body tensor needs `balanceinertia` because a lighter actuator is subtracted from a lump; the change is tiny against the torso subtree plus armature.

## Low-cogging ferrite sets (6 Nov 2026)

The mass-only optimiser chose 24-pole spokes for most ferrite arm joints, with cogging of 20 to 38% of continuous torque. The four low-cogging sets (`designs_lowcog`, no 24-pole) cost about 1% more arm mass. MuJoCo with stock gains (`results/20261106_062806_lowcog`): main, mass_64 and size pass at nominal and high stock inertia and fail at the low bound; 100:1 fails at all three. With gains retuned after seeing results (`results/20261106_063910_lowcog_retuned`): all four pass. Simulink (`../x1_variants/summary_lowcog.md`): all 12 runs complete with no solver warnings; peak bus 57.15 V and 61.19 to 61.20 V with aux lost; brake energy 26.0 to 27.8 J (35.9 to 37.5 J with aux lost), against stock 27.0 / 26.9 / 36.6 J. The full Simulink table now covers all 18 coupled sets (`summary_coupled.md`).

The swerve drive in the base was not changed in any variant, because it was not in the design scope of this feasibility study.
