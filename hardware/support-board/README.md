# External actuator support board

Findings from a reverted 27 Oct routing attempt (copper weight, shunt sharing, XT30, floating grounds): [check](../support-board-finish-20261026.md).
Circuit review: [finalization review](../support-board-finalization-20261025.md).
Circuit qualification is OPEN; the nominal short and assumed Zener-corner passes
do not establish worst-case operation. C3 is now 47 nF / 100 V; see that review.
The [OpenArm v2 load study](../../../openarm-load-results-20261025.md) recommends
one board per arm at provisional 20 A / 40 A, pending actuator and duty selection.

Canonical native project: [SupportBoard.kicad_pro](SupportBoard.kicad_pro).
Open this project in KiCad 9. Its native files and reading copies are together:

- [PCB](SupportBoard.kicad_pcb), 180 x 45 mm with parts on both faces.
- [Readable schematic](SupportBoard.kicad_sch), six wired sections on A2.
- [Schematic PDF](SupportBoard-schematic.pdf) and [layout PDF](SupportBoard-layout.pdf).
- Local `Support.pretty/`, `Support.kicad_sym`, library tables and `bom.csv`.

The board provides input hot-swap/precharge, the bulk capacitor bank, two
power/CAN chain ports, the brake chopper and discharge control. Brake resistors
remain on the separate BrakeResistor board.

Read [the design record](../support-board-design.md) for the circuit values,
sources, current cleanup evidence and open electrical/thermal gates. This is a
working draft with 176 unrouted connections. Native ERC, non-unrouted DRC and
schematic parity have zero findings; all 278 compiled pin nodes match PCB pads.

The schematic is a single A2 sheet with six wired sections: input hot-swap,
DC link and chain ports, housekeeping supplies, brake command, and RUN/thermostat
interlocks, plus the battery-derived hot-swap enable. Related passives are wired to their ICs. Power symbols and named
debug nodes make the signal paths and logic polarity visible. The readability
redraw preserves all 278 pin-to-net assignments and component identities.

The [long, narrow arm layout](../arm-layout-concept.md) is now the active project,
with components on both faces; earlier 55/39 counts predate the protection additions.
Its capacitor rows face inward toward central BUS+ copper, with GND on the
outside and a stitched back return. It preserves the circuit and passes
placement/parity checks, with 176 connections elsewhere still unrouted.
Generated staging files remain under `../../../build/support-board/arm-placement/`.
Actual arm fit and underside clearance are open. The former 130 x 80 mm routed
project is archived under `../../../build/support-board/routed-130x80-before-arm-project/`,
including its libraries, verification reports and backup hashes.

From the parent `external_boards` directory, using KiCad 9's bundled Python:

```powershell
& 'C:/Program Files/KiCad/9.0/bin/python.exe' verify_support_board.py --allow-unrouted
& 'C:/Program Files/KiCad/9.0/bin/python.exe' readable_support_schematic.py --draft
```

The first command reports the draft's opens while requiring every other check
to pass. Omit `--allow-unrouted` for the strict completion check, which fails
until routing is finished. The second command writes a schematic source draft under
`../../../build/support-board/schematic-readable/` without changing the active
schematic or PCB. Omit `--draft` to apply the reviewed schematic layout.

`arm_support_placement.py` generates a staged arm-layout candidate for review;
it does not overwrite this active project. The legacy `support_board.py` and
`finish_support_board.py` describe the archived 130 x 80 mm version. The old
generator refuses to overwrite a differently sized active board.

Reports and plots are generated in `../../../build/support-board/`. The active
project uses 0.20 mm clearance and 0.50 mm local zone clearance; it does not yet
have a qualified current/voltage-specific rule set or verified Kelvin layout.
