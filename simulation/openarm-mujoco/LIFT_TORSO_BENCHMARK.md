# OpenArm v2 on a Pixel-style lift, torso bend and drive base

5 November 2026.

## Benchmark decision

The arms stay OpenArm v2, because its CAD is open and its motors are named Damiao parts. The body layout below the arms follows CMU Pixel (arXiv 2611.03648): a holonomic drive base, a 430 mm vertical lift, and a cantilevered torso-pitch joint so the robot can bend over. Pixel's own arms are AgileX NERO units. They run at 24 V, their motors are closed, and AgileX publishes no joint torques, so they cannot serve as an actuator benchmark.

A drive base was chosen instead of legs. Legs would need balance control and a much larger actuator class, and Pixel shows a wheeled base is enough for household manipulation. Legs can be added later as a separate study.

## Model

`models/mobile_lift_torso.xml` attaches the unchanged vendor `openarm_bimanual.xml`:

| Joint | Type | Range | Notes |
|---|---|---|---|
| base_x, base_y, base_yaw | planar slide/slide/hinge | free | Abstract holonomic base with velocity actuators; swerve modules are not modelled |
| lift | slide | 0 to 0.43 m | Pixel travel. Shoulders at 0.85 to 1.28 m |
| torso_pitch | hinge about y | -20 to 100 deg | Positive is a forward bend |
| 14 arm joints + 2 grippers | vendor | vendor | Unchanged OpenArm v2 |

**Estimates in the model (from neither Pixel nor Enactic):**
- Base mass is 30 kg and the carriage is 3 kg.
- The torso is 6 kg, with its centre of mass 0.20 m above the pitch axis.
- The shoulder mount sits 0.45 m above the pitch axis.

The two arms weigh 12.3 kg together in the vendor model. That puts 18.3 kg above the pitch axis and 21.3 kg on the lift.

## Sizing results (`size_lift_torso.py`, `results/lift_torso_sizing/`)

These are rigid inverse dynamics with no joint-limit forces. The payload is a point mass 0.12 m beyond each end-effector frame, and the arm sweep covers the full shoulder range with the elbows straight or at 90 deg.

| Case | 0 kg per hand | 3 kg per hand |
|---|---|---|
| Worst static holding torque at the torso pitch | 89 Nm | 148 Nm |
| Bend 0 to 90 deg in 2 s, peak torque / speed / power | 102 Nm, 1.5 rad/s, 114 W | 175 Nm, 1.5 rad/s, 195 W |
| Bend in 1 s | 144 Nm, 2.9 rad/s, 292 W | 258 Nm, 2.9 rad/s, 523 W |
| Bend in 0.6 s | 245 Nm, 4.9 rad/s, 778 W | 457 Nm, 4.9 rad/s, 1454 W |
| Lift static force | 209 N | 267 N |
| Full lift stroke in 5.7 s (Pixel's 75 mm/s average), peak force / power | 237 N, 33 W | 296 N, 42 W |
| Full stroke in 1.5 s | 322 N, 171 W | 384 N, 203 W |

The worst static case is a 90 deg bend with the arms reaching straight out, so they are horizontal in the world. Even with the arms hanging straight down in the world, for example to pick something off the floor, a 90 deg bend still needs 67 Nm with empty hands and 94 Nm with 3 kg per hand. At a 30 deg bend those figures are 32 and 45 Nm. The map is in `torso_pitch_sizing.png`.

## What this means for the actuators

- **Torso pitch is the critical new joint.** Pixel uses a MyActuator X10-100 rated at 50 Nm. With OpenArm arms that only covers bends up to about 30 to 40 deg. A bend in about 2 s with 3 kg per hand needs about 175 Nm peak and 150 Nm continuous at the worst pose. That is more than three times any OpenArm arm joint (DM8009 at 40 Nm), so it needs its own high-ratio actuator. This is a candidate for one of the project's own motor and controller designs.
- **A gravity counterbalance is worth studying.** The holding torque is mostly gravity. A spring or gas-strut counterbalance on the pitch axis would cut the continuous rating, and the robot needs a brake or a non-backdrivable stage anyway. Otherwise a power loss at 90 deg folds it under 90 to 150 Nm.
- **The lift is not a custom actuator problem.** A self-locking lead screw holds 210 to 270 N without power, and Pixel's Kraken X60 already covers the speed. Faster lifts mainly cost power: about 200 W for a 1.5 s stroke.
- **Bus:** the 48 V LFP pack matches the project's 20 to 54 V controller boards. Peak bend power of 0.2 to 0.5 kW for 1 to 2 s bends adds to the regeneration and brake-resistor study, since lowering the torso returns energy.

## Not yet done

- No torso or lift actuator dynamics yet: the GIM layer only covers the arm joints.
- Masses are estimates until a torso CAD exists.
- No contact tasks, base dynamics or tipping check. A 3 kg-per-hand reach at a 90 deg bend moves the centre of mass forward, and base stability against tipping should be checked next.

## Cross-check on MABEL's own model (5 Nov 2026)

MABEL (arXiv 2611.04083, github.com/robotmabel/open_mabel, commit 2f9229f) is an open robot with this exact layout. Its 7-DOF arms are adapted from OpenArm, it has a DM-J10422P torso-pitch joint (1:22, 100 Nm rated, 400 Nm peak, -60 to +15 deg), a 0.635 m two-stage desk-column lift, a 3-module swerve base and 24 V power. `size_mabel.py` runs the same inverse-dynamics sizing on MABEL's unmodified MJCF (61.7 kg total, 20.3 kg above the torso joint, 27.7 kg on the lift top), with the payload at each palm.

| Case | 0 kg per hand | 3 kg per hand |
|---|---|---|
| Worst static torso torque within MABEL's range (at the 60 deg forward limit) | 59 Nm | 104 Nm |
| Torso torque with arms mid-range: upright, 30 deg, 60 deg bend | 26, 39, 42 Nm | 52, 74, 77 Nm |
| Lift lower-stage static force | 294 N | 353 N |

- **MABEL's motor choice:** the 100 Nm rated torso covers every empty-hand pose. It reaches its rated torque with 3 kg per hand at full bend, which matches MABEL's own 3.5 kg static load bound.
- **My estimated layout was conservative:** it gave 80 Nm and 136 Nm over the same 60 deg range. The main reason is the shoulder height above the torso axis, which is 0.45 m in my layout and about 0.15 m in MABEL's model, so MABEL's arms sit closer to the pivot.
- **Recommendation:** use MABEL's MJCF as the benchmark geometry from here on, and keep `mobile_lift_torso.xml` only as the stock-OpenArm variant. Two differences from the project's hardware still apply. MABEL runs at 24 V, against the 48 V the controller boards target, and its torso has no brake or counterbalance.

## Scope note

The swerve drive in the base was not changed in any variant, because it was not in the design scope of this feasibility study. Only the arm and torso actuators were swapped. In these runs the base is also welded in place.
