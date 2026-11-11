# X1 first spin

**Archived, partially routed and unvalidated.** This 72 mm circular
controller combines logic and power on one six-layer PCB. It predates the
shared logic board and the separate WGG and P96 power boards.

Open [moteus_x1.kicad_pro](moteus_x1.kicad_pro) in KiCad 10. The native PCB,
hierarchical schematic, symbol and footprint libraries, drawing sheet and
local envelope models are included. See [attribution](NOTICE.md).

## Saved design status

| KiCad 10.0.7 check | Result |
|---|---|
| PCB design-rule findings, apart from connectivity and parity | 0 |
| Unconnected items | 322 |
| Schematic-to-PCB field mismatches | 9 |
| Schematic ERC | 26 errors and 516 warnings |

The checks used the saved board and its project rules. DRC included
schematic parity. ERC and DRC reported errors and warnings. The nine parity
findings are footprint/symbol field mismatches. The ERC errors include
unconnected or undriven pins and pin-type conflicts.

Routing, electrical review, stackup and mechanical fit remain open. Encoder
alignment, controller firmware and powered testing are unverified. This
archive contains editable source. Fabrication files were never released for
this snapshot.

## 3D display

The PCB retains references to ten optional STEP models from the pinned
[upstream X1 3D folder](https://github.com/mjbots/moteus/tree/8f747f4ac448d117c1390b77ca2d80c27e751a56/hw/x1/3d).
For a fuller 3D view, put that folder beside the project as `3d/`. The
remaining stock models use KiCad's 3D model libraries. The schematic and
board layout can be reviewed without the STEP bodies.
