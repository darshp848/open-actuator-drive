# Open actuator drive

> Working name. Pick the final name before the first push, and keep "moteus" and
> "mjbots" out of it (see [NOTICE](NOTICE.md)).

KiCad hardware and simulation models for a 12S (10 to 54 V) brushless actuator
drive and the support electronics around it, built for a humanoid upper body
(OpenArm v2 class arms).

**Warning.** These boards switch tens of amps from a 50 V battery. They are
research prototypes and have not been fully qualified. Build and test them at
your own risk.

## What is here

| Path | Contents | Status |
|---|---|---|
| `hardware/support-board/` | Hot-swap and precharge input, bulk DC link, two power and CAN chain ports, brake chopper, discharge control. 180 x 45 mm. | Routed. DRC 0 errors, 0 unconnected. Circuit qualification still open. |
| `hardware/brake-resistor/` | 2.5 ohm bank (4x LTO100 on RA-T2X sinks) with an 85 C thermostat. 98 x 86 mm, 2 layers. 41 W average at 40 C. | Routed. DRC clean. |
| `hardware/df40-breakouts/` | Bench breakouts for the Hirose DF40 80-pin stack connector, header and receptacle halves. 2 layers, JLCPCB standard rules. | Routed. DRC and ERC clean. |
| `hardware/capacitor-bank/` | External bulk capacitor board. | Routed. DRC clean. |
| `hardware/wip/` | Two-story drive stack: logic board, WGG power board, P96 power board. Derived from mjbots moteus X1. | Placement stage, not routed. |
| `fab/` | Gerbers and drill (zipped), schematic PDF, BOM CSV and pick-and-place for each routed board. | Generated with KiCad 10.0.7. |
| `simulation/openarm-simulink/` | One-arm Simulink/Simscape dynamics plus an averaged electrical model of the drive and support board. `variants/` holds the parameter sets for the motor and magnet studies. | Checks pass. Not validated against hardware. |
| `simulation/openarm-mujoco/` | OpenArm v2 upper-body MuJoCo setup driven from MATLAB. Fetches the upstream OpenArm model with `fetch_assets.py`. | Numerical checks pass. |
| `simulation/mabel-gim-sizing/` | Actuator sizing against MABEL arm loads. Needs a local clone of open_mabel. | |
| `simulation/virtual-actuator/` | Synthetic two-joint arm and actuator plant for control experiments. | Synthetic model. |

## Tools

- KiCad 9 or newer for `hardware/`. The `fab/` outputs were made with KiCad 10.0.7.
- MATLAB R2026a with Simulink, Simscape and Robotics System Toolbox for
  `simulation/openarm-simulink/`.
- Python 3.11 with the pinned requirements in each simulation folder. MuJoCo 3.6.0 for
  `openarm-mujoco`.

## Licence

| What | Licence |
|---|---|
| Hardware design files (KiCad sources, footprints, 3D bodies, fab outputs) | [CERN-OHL-S-2.0](LICENSE-HARDWARE) |
| Code (Python, MATLAB, scripts) and simulation models | [Apache-2.0](LICENSE-APACHE) |
| Documentation and images | [CC BY 4.0](LICENSE-DOCS) |

Files derived from mjbots moteus X1 keep their Apache-2.0 licence and notice. See
[NOTICE](NOTICE.md) for upstream projects and their licences.
