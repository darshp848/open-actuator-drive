# X1 stack DF40 breakouts

Two bench breakouts for the X1 two-story stack. The stack has one
board-to-board connector, J_STACK1, a Hirose DF40 80-contact pair at 0.4 mm
pitch:

| Board | Part | Side |
| --- | --- | --- |
| Common_Logic | DF40HC(3.0)-80DS-0.4V receptacle | bottom |
| Power_Reference (WGG) and Power_P96 | DF40C-80DP-0.4V header | top |

The other connectors in the X1 projects (XT30/XT90, JST GH/ZR/PH, 1.27 and
2.54 mm pin headers) are cable or debug connectors, not mezzanines.

No off-the-shelf DF40 80-pin to 0.1" breakout was found (2026-11-01 search:
distributor listings, Tindie, OSH Park; the only DF40 breakout found is a
30-pin OSH Park project), so these are custom.

## The boards

| Project | Connector | Use |
| --- | --- | --- |
| `DF40_Header_Breakout` | DF40C-80DP-0.4V on top | The logic board plugs onto it exactly as it plugs onto a power board. Bring up or probe the logic board alone. |
| `DF40_Receptacle_Breakout` | DF40HC(3.0)-80DS-0.4V on the bottom | Plugs onto a power board in place of the logic board. Drive or probe a power board from a bench MCU. |

Every DF40 contact gets one 0.1" through-hole point. J2 (2x20) carries the odd
row and J3 (2x20) the even row: J2 pin k is DF40 pin 2k-1 and J3 pin k is DF40
pin 2k. Each pin is labelled on the silkscreen with its DF40 pin number and net.
[`pinout.json`](pinout.json) has the full map.

Mechanical fit:

- The connector uses the stack boards' own land patterns (copied into
  `X1_Breakout.pretty`), at the same position, side and orientation as the board
  it stands in for. The generator asserts pad 1 lands where it does on the stack.
- H1/H2 are the two spacer positions common to all three stack boards
  ((135,115) and (165,115) in stack coordinates). With 3 mm spacers there they
  fix orientation, so the DF40 cannot be mated 180 degrees out. The other stack
  holes sit in the fan-out and are omitted; remove those spacers first.
- Near the connector the outline stays inside the logic board's footprint and
  clears P96's 5V IN pads. It covers P96's FAN1 socket, as the logic board
  itself does; that clash is open on the stack and both fit once FAN1 moves.
- The headers sit at least 2 mm beyond the rim of both power boards, so no
  through-hole pin lands over a power board. The tongue that crosses the rim
  rides 3 mm above the power board. Checked against P96 and Power_Reference:
  only parts under 2.6 mm tall are under it. Re-check this for any new power
  board.

Electrical:

- 2 layers, 1.6 mm, about 100 x 80 mm, ground pour on both sides. The even
  DF40 row fans out below the connector to two staggered via rows, so no via
  sits at the 0.4 mm pitch. All bends are 45 degrees. About 330 tented
  0.6/0.3 mm stitching vias on a 2.5 mm grid tie the two ground pours together
  wherever the pour has room on both layers.
- Every rule is inside JLCPCB's standard 2-layer process with no surcharge
  (jlcpcb.com/capabilities/pcb-capabilities, read 2026-11-02): trace/space
  0.127/0.127 mm (limit 0.10), vias 0.6/0.3 mm (0.15 mm annular ring), via hole
  to track at least 0.2 mm, copper to edge 0.3 mm (limit 0.2), silkscreen text
  1.0 mm with 0.17 mm lines. KiCad's DRC rules are set to these values.
- Signal integrity (hand estimates): with no plane under the 0.4 mm-pitch
  buses, neighbouring traces couple about 24% near-end to a ~1 ns edge. Near-end
  crosstalk scales with how much of the edge fits in the ~0.5-1 ns of coupled
  length, so with 10 ns edges it falls to roughly 2-3% (under 0.1 V from a
  3.3 V signal). Drive these lines with the STM32 GPIO speed set to low or
  medium and keep SPI to a few MHz. Every analog line (CUR1-3, VBAT_SNS,
  VTEMP_*) has GND traces on both sides; the signal-to-signal neighbours are
  the DRV SPI group, ENC_SCK/MOSI, ENC_MISO/ROT_CS, DRV_FAULT/BRAKE_MON and
  CAN_H/CAN_L next to +5V. No line needs controlled impedance. Commit f3ce734
  has a 4-layer build with ground inner layers (about 5% coupling) if fast
  edges are ever needed.
- The four +5V and four +3V contacts are tied together between the DF40 pad
  rows, as they are on the stack boards. Each contact is rated 0.3 A, and each
  0.127 mm trace carries about the same, so keep each rail under about 1 A
  through the breakout.
- Reserved contacts (`RSV_*`) are broken out but are unconnected on the stack
  boards today.

## Status

KiCad 10.0 DRC: 0 violations, 0 unconnected, schematic parity clean. ERC:
0 violations. Not fabricated, not mated with real parts, and not yet run
through JLCPCB's online DFM check.

Upload `fab/DF40_Header_Breakout-JLCPCB.zip` or
`fab/DF40_Receptacle_Breakout-JLCPCB.zip` (Gerbers plus Excellon drill) and
order 2 layers, 1.6 mm, HASL, OSP or ENIG. ENIG gives flatter pads for the 0.4 mm
DF40; JLCPCB assembly is the easier way to fit it than hand soldering.

The breakouts inherit the stack's open item: DF40 pad numbering versus Hirose's
mating view has not been checked against the datasheet. They copy the stack
footprints exactly, so they agree with the stack whichever way that resolves.

## Regenerating

```
"%LOCALAPPDATA%/Programs/KiCad/10.0/bin/python.exe" generate.py
```

[`generate.py`](generate.py) reads J_STACK1 from the three stack boards, fails if
their pinouts disagree, and rewrites both projects. It only reads the stack
boards. Routing is generated by the script; Freerouting could not escape the
0.4 mm pitch.
