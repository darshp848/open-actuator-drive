# DC bus support board

One board per arm. It handles input hot-swap and precharge, bulk capacitance,
two power/CAN chain ports, housekeeping power, the brake chopper and controlled
discharge. The [brake resistor strip](../brake-resistor/) connects to J4 for
BUS+ and BRK_SW and to J6 for the thermostat.

- [KiCad project](SupportBoard.kicad_pro) and [parts list](bom.csv)
- [Schematic PDF](../../fab/support-board/SupportBoard/SupportBoard-schematic.pdf)
- [Gerber and drill ZIP](../../fab/support-board/SupportBoard/SupportBoard-gerbers.zip)

The 180 × 45 mm, two-layer board is routed, with components on both faces.
KiCad 10.0.7 reported zero DRC findings, unconnected items and schematic
mismatches. ERC reported three library-symbol mismatch warnings for D5, D6,
and JP1. Those warnings need review before release for hardware.

## Work still open

Check arm fit, underside clearance, fabricator stackup and assembly. Validate
precharge, fault interruption, operating voltage and current, shunt sharing,
temperature rise, and the discharge circuit on hardware. The 20 A continuous
and 40 A peak figures used in the arm study were provisional design inputs.

Verify the chopper with the external resistor bank, including the relationship
between the input overvoltage lockout and the chopper threshold. The current
thermostat input inhibits forced discharge. Verify a system-level means to
stop regeneration when the thermostat opens. See the [brake resistor
notes](../brake-resistor/) for the circuit limitations identified during design.
