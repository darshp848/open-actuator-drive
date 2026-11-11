# Earlier actuator controllers

These designs and studies predate or sit outside the current WGG and P96
controller stack. Hardware validation is still open.

| Design or motor target | Files | State |
|---|---|---|
| Single-board X1 first spin | [Native KiCad archive](x1-first-spin/) | Six-layer, 72 mm draft. Partially routed, with 322 open connections. Schematic ERC and parity findings remain open. |
| SteadyWin GIM4310-40 (P57) | [Design status](#p57-for-the-gim4310-40) | Packaging and schematic planning. |
| Unitree IM6014 | [Design status](#unitree-im6014) | Replacement-controller requirements and fit study. |
| MAB Robotics MD80 v3.0 mounting | [Published 60 V controller, pinned revision](https://github.com/darshp848/bldc-controller-60v/tree/422359e00a2230f3f6dcd59c5f6b1280d80ab7d3) | Six-layer Rev B is routed. Its DRC report has zero unconnected pads and one J3/MECH1 courtyard overlap. Firmware, fabrication and bench validation remain open. |

## P57 for the GIM4310-40

The plan called for a compact, single-board controller for the wrist joints.
The design still needs schematic capture, a native PCB layout, and motor
voltage, encoder and cavity fit checks. The P96 design for the larger
GIM8115-36 and GIM8108-36 motors is in [the current board collection](../wip/power-p96/).

## Unitree IM6014

The study considered a replacement controller and custom firmware for the
IM6014. The earlier X1 board was a layout reference. A motor-specific outline,
mount and encoder integration, voltage envelope, and PCB layout remain open.
