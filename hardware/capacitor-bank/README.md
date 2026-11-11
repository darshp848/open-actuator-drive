# External capacitor bank

This two-layer, 101.6 × 25.4 mm board puts twelve 130 µF, 80 V capacitors
across the DC bus for 1,560 µF nominal, with a 10 kΩ bleeder. The
[KiCad project](CapacitorBank.kicad_pro) includes the custom footprints and a
simplified capacitor body. See the [parts list](bom.csv), [schematic
PDF](../../fab/capacitor-bank/CapacitorBank/CapacitorBank-schematic.pdf) and
[Gerber and drill ZIP](../../fab/capacitor-bank/CapacitorBank/CapacitorBank-gerbers.zip).

The layout is routed. KiCad 10.0.7 reported zero DRC findings, unconnected
items, schematic mismatches and ERC findings. Footprint fit and polarity,
assembly, bus ripple, inrush, discharge time and temperature rise still need
verification on hardware.
