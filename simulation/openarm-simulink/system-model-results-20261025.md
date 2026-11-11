# OpenArm v2 system model: first results (2026-10-25)

## Model

`openarm_system.slx` models one OpenArm v2 arm on seven SteadyWin GIM actuators. Each actuator is driven by the two-story actuator boards, and the arm is fed from one support board. `build_openarm_system.m` generates the model; parameters and their sources are in `openarm_params.m`.

**Arm.** Codex's validated OpenArm v2 `rigidBodyTree` (`build/openarm-v2-load`), with each GIM motor's mass above the stock Damiao motor added at its joint. The payload is a point mass 150 mm below the wrist.

**Actuators.** Averaged FOC, one per joint:

| Joints | Motor | Gear ratio |
|---|---|---|
| J1-J2 | GIM8115-36 | 36:1 |
| J3-J4 | GIM8108-36 | 36:1 |
| J5-J7 | GIM4310-40 | 40:1 |

- **Control:** computed-torque PID position loop (5 Hz triple pole). Below it, dq PI current loops at 1 kHz with back-EMF feedforward. The SVPWM voltage limit comes from each board's local bus.
- **Losses and dynamics:** gear efficiency 0.9 motoring and 1/0.9 generating, Coulomb and viscous friction, and reflected rotor inertia.
- **Board power:** inverter conduction and switching loss plus 3 W of board load.

**Power network (Simscape):**
- Source and lead, then BAT+ with 80 uF lumped (C1, C59, C60).
- LM5069 hot-swap: current limit 61 A typical, power limit 305 W, 0.71 ms timer, LM5069-2 retry.
- LTC4359 ideal diode, then the 1.248 mF minimum bank.
- Brake chopper: primary at 57.1 / 55.0 V (needs the aux); bus-powered backup at 61.13 / 60.29 V with TPS3700 delays. 2.5 ohm brake.
- Seven daisy-chain hops (0.2 uH, 10 mOhm) to seven boards with 76 uF each.

**Run.** `run_openarm.m` defines the scenarios and screens; `batch_run.m` runs them in a separate MATLAB; `plot_run.m` saves `results/<scenario>.png`. A 1.8-3.5 s scenario takes 0.5-2 min.

## Results (6 kg total end load, 48 V bench PSU, minimum bank)

| Scenario | Bus peak | Brake energy | Source peak / mean | Hop-1 peak | Regen peak | Tracking | Screens |
|---|---|---|---|---|---|---|---|
| lift6: J1 0 -> 90 deg in 1 s, hold, lower in 1 s | 57.2 V (primary) | 27.7 J | 5.0 / 0.84 A | 7.4 A | 48 W | 1.2 deg | all pass |
| drop6: lower from horizontal in 1 s | 57.1 V (primary) | 27.9 J | 5.2 / 0.35 A | 7.4 A | 48 W | 1.1 deg | all pass |
| drop6_auxfail: same, aux lost at 0.1 s | **61.2 V (backup)** | 38.0 J | 5.2 / 0.32 A | 7.8 A | 48 W | 1.1 deg | all pass |
| drop6_nobrake: aux lost, no backup channel | **142 V** | 0 | 5.2 / 0.18 A | 2.3 A | 48 W | 1.1 deg | **bus > 80 V fails** |

## What this says about the boards

- **Brake channel.** An ordinary 6 kg lowering at the GIM8115 speed limit puts about 48 W back on the bus. With the source blocked by the LTC4359 and only 1.248 mF of bank, that energy has to go to the brake:
  - Primary channel working: the bus is held at 57 V.
  - Aux dead, backup present: held at 61 V.
  - Aux dead, no backup: the bus reaches 142 V. That would destroy the 80 V bank.
  This confirms the aux-loss finding and the backup fix with arm dynamics rather than a constant current source.
- **Source and hot-swap.** This duty is far below the 25 A / 50 A support rating: 5 A peak, under 1 A mean. No hot-swap trips.
- **Brake current.** The first daisy-chain hop (XT30) sees -7.8 A peaks during chopping, as the boards' local capacitors discharge into the brake. That is within XT30 ratings.
- **Supply margin.** Board-side voltage stays within 47.7-61.2 V with the backup.

## Limits of this first version

These cases exercise only J1 against gravity; multi-joint fast motion, collisions, payload release and e-stop deceleration are next. Assumed values:
- Rotor inertia, gear efficiency and friction (SteadyWin does not publish them).
- 3 W per board.
- Lead and hop impedance.

The GIM8108 page's stall torque disagrees with its Kt x stall current. The GIM4310 is rated 12-40 V, below the 48 V bus.

The inverters are averaged: there is no PWM ripple, dead time or current-sense model. A switching-level shoulder variant is planned.

Arm dynamics terms are evaluated at 5 kHz and held between samples. The supervisor runs at 20 us, so backup delays round to 20-40 us.

## Reproduce

From `system_model/` in PowerShell, clear `NoDefaultCurrentDirectoryInExePath` first. Some agent shells set it, and it stops the MEX build from running its own .bat.

```powershell
$env:NoDefaultCurrentDirectoryInExePath = $null
& "C:\Program Files\MATLAB\R2026a\bin\matlab.exe" -batch "batch_run({'lift6','drop6','drop6_auxfail','drop6_nobrake'})"
```

Compiled robotics blocks need a C compiler. MSVC 2022 build tools were added to Visual Studio Community on 2026-10-25, and MATLAB was set up with `mex -setup`.
