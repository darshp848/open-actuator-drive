# Open actuator drive

KiCad designs and simulation models for a nominal 48 V brushless actuator
drive and its support electronics. See the [humanoid upper-body project
page](https://darshpatel.xyz/projects/humanoid-upper-body) for board renders,
system context, and the simulations behind the designs.

**All designs in this repository are unfinished, unvalidated research prototypes.**
No hardware validation has been performed. Routing and KiCad checks describe the
state of the design files. Fabrication, assembly, and hardware bring-up and
testing are still open.

## Boards

| Board | KiCad source and notes | Gerbers and drill | Current state |
|---|---|---|---|
| DC bus support | [180 × 45 mm board](hardware/support-board/) | [ZIP](fab/support-board/SupportBoard/SupportBoard-gerbers.zip) | Routed. No DRC findings or open connections. Three ERC library-symbol warnings. |
| Brake resistor Rev B | [190 × 38 mm strip](hardware/brake-resistor/) | [ZIP](fab/brake-resistor/BrakeResistor/BrakeResistor-gerbers.zip) | Routed. No DRC or ERC findings. Strip cooling and the thermostat interlock need testing. |
| External capacitor bank | [101.6 × 25.4 mm board](hardware/capacitor-bank/) | [ZIP](fab/capacitor-bank/CapacitorBank/CapacitorBank-gerbers.zip) | Routed. No DRC or ERC findings. Capacitor fit and electrical behavior need testing. |
| DF40 header breakout | [Bench breakout](hardware/df40-breakouts/DF40_Header_Breakout/) | [ZIP](fab/df40-breakouts/DF40_Header_Breakout/DF40_Header_Breakout-gerbers.zip) | Routed. No DRC or ERC findings. Mating orientation and fit need checking. |
| DF40 receptacle breakout | [Bench breakout](hardware/df40-breakouts/DF40_Receptacle_Breakout/) | [ZIP](fab/df40-breakouts/DF40_Receptacle_Breakout/DF40_Receptacle_Breakout-gerbers.zip) | Routed. No DRC or ERC findings. Mating orientation and fit need checking. |
| Common logic | [Controller stack draft](hardware/wip/logic-board/) | | Partially routed, 177 unconnected items. |
| P96 power | [Controller stack draft](hardware/wip/power-p96/) | | Partially routed, 212 unconnected items and one silkscreen DRC finding. |
| WGG power | [Controller stack draft](hardware/wip/power-wgg/) | | Partially routed, 258 unconnected items. |

Open each project's `.kicad_pro` in KiCad 10. The five routed boards also have
schematic PDFs, BOMs and position files. See [fabrication files](fab/) for
export settings and the scope of the checks. See the [DF40 breakout
notes](hardware/df40-breakouts/) for the pin map and bench setup.

## Work still open

- Finish routing the common logic, P96 and WGG boards. Close remaining ERC and
  DRC findings, confirm motor and stack fit, and test the firmware and
  controller interfaces.
- Review the routed boards with the fabricator, check the stackup and part
  footprints, and build and inspect first articles. Check the DF40 pin mapping
  against the connector mating view and verify fit and continuity.
- Bench-test the power input and precharge, overcurrent and voltage protection,
  bus ripple and discharge, the regen chopper and the brake thermostat. Measure
  thermal behavior at the intended voltage, current, duty cycle and airflow.
  The simulations and calculations do not establish safe operating limits.
- Test the assembled drive with the intended motors and loads. Correlate the
  electrical, mechanical and thermal models with measured data.

## Simulations

| Model | What it covers |
|---|---|
| [OpenArm Simulink](simulation/openarm-simulink/) | One-arm dynamics and averaged electrical model. MATLAB R2026a with Simulink, Simscape and Robotics System Toolbox. |
| [OpenArm MuJoCo](simulation/openarm-mujoco/) | Numerical baseline with pinned upstream OpenArm assets. |
| [Virtual actuator](simulation/virtual-actuator/) | Synthetic two-joint arm and actuator plant for control experiments. |

## Licence

| Files | Licence |
|---|---|
| Original hardware sources, footprints, 3D bodies and fabrication outputs | [CERN-OHL-S-2.0](LICENSE-HARDWARE) |
| Original Python and MATLAB code and simulation models | [Apache-2.0](LICENSE-APACHE) |
| Original documentation and images | [CC BY 4.0](LICENSE-DOCS) |

The mjbots-derived controller boards and the upstream simulation assets have
their own terms. See [third-party notices](NOTICE.md).
