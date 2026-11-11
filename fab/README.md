# Fabrication files

The five routed projects each have a Gerber and drill ZIP, a schematic PDF, a
schematic BOM CSV, and a KiCad position CSV. Use the BOMs next to the KiCad
sources for additional part numbers and off-board assembly hardware. The
position CSVs use KiCad's generic format. Review placement, rotations, DNP
parts, and manufacturer-specific file requirements before ordering assembly.

## Checks and exports

KiCad 10.0.7 reported zero PCB rule violations, unconnected items, and
schematic/PCB mismatches for all five projects. The support board had three ERC
warnings about embedded symbol copies (D5, D6 and JP1). The other four projects
had zero ERC findings. The included Gerber layers, drill files, BOMs and
positions matched fresh exports. These are checks of design files. Fabricator
DFM, first-article inspection, and all hardware validation remain open.

From the folder containing a routed board's KiCad project, substitute its
basename for BOARD. Use the same absolute origin for the Gerbers and drill
files. The commands below create review outputs in a new folder.

```sh
kicad-cli pcb drc --severity-error --severity-warning --all-track-errors --schematic-parity --exit-code-violations -o review-drc.rpt BOARD.kicad_pcb
kicad-cli sch erc --severity-error --severity-warning --exit-code-violations -o review-erc.rpt BOARD.kicad_sch
kicad-cli pcb export gerbers --layers F.Cu,B.Cu,F.Paste,B.Paste,F.Mask,B.Mask,F.Silkscreen,B.Silkscreen,Edge.Cuts -o review/gerbers/ BOARD.kicad_pcb
kicad-cli pcb export drill --generate-map -o review/gerbers/ BOARD.kicad_pcb
kicad-cli sch export pdf -o review/BOARD-schematic.pdf BOARD.kicad_sch
kicad-cli sch export bom --fields 'Reference,Value,Footprint,${QUANTITY},Manufacturer,MPN,${DNP}' --group-by Value,Footprint -o review/BOARD-bom.csv BOARD.kicad_sch
kicad-cli pcb export pos --format csv --units mm -o review/BOARD-positions.csv BOARD.kicad_pcb
```
