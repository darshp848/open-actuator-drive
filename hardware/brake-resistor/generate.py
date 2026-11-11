"""Generate the earlier 120 x 86 mm X1 brake resistor board.

This script overwrites the project in place. The current 190 x 38 mm strip
layout is maintained in the checked-in KiCad project.

Run with KiCad's bundled Python:
    "%LOCALAPPDATA%/Programs/KiCad/10.0/bin/python.exe" generate.py

The support board's chopper (Q3) switches this bank across BUS+. Wiring:
  J1 (two wire pads, 8 mm pitch)  -> support J4: 1 = BUS+, 2 = BRK_SW
  J2 (JST PH, 2 pin)              -> support J6: 1 = TH_OPEN, 2 = support GND

R1-R4 are Vishay LTO100 10 ohm (2.5 ohm bank), each screwed to its own Ohmite
RA-T2X-64E board-level heatsink. SW1, an Airpax 67L085P NC thermostat (opens at
85 C, plastic isolated bracket), shares R4's screw on the far face of HS4's
web. Sizing is in brake_sizing.py and README.md.

Copper: BUS+ is a plane on F.Cu, BRK_SW a plane on B.Cu, so the four
resistors sit between two parallel plates (low loop inductance for the
chopper). The thermostat pair runs on F.Cu with 1 mm clearance to the plane.
"""
import json
import math
import os
import sys
import uuid

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
NAME = "BrakeResistor"
PROJ = os.path.join(HERE, NAME)
LIB = os.path.join(HERE, "BrakeResistor.pretty")
KICAD_SHARE = os.path.normpath(
    os.path.join(os.path.dirname(sys.executable), "..", "share", "kicad"))
STOCK_FP = os.path.join(KICAD_SHARE, "footprints")
STOCK_SYM = os.path.join(KICAD_SHARE, "symbols")

# JLCPCB standard 2-layer limits (jlcpcb.com/capabilities/pcb-capabilities):
# 0.127 mm trace/space, 0.3 mm drill, 0.2 mm copper to edge.
# The extended board uses a 120 x 86 mm outline.
CLEARANCE = 0.5      # BUS+ to BRK_SW and to floating heatsink pins (57 V)
PLANE_CLEARANCE = 1.0  # plane to anything else, incl. the thermostat loop
TH_CLEARANCE = 0.5   # thermostat pair to itself (JST PH pads are 0.8 mm apart)
TH_TRACK = 0.4
SILK_H, SILK_W = 1.2, 0.18

X0, Y0, W, H = 100.0, 100.0, 120.0, 86.0
# Heatsink centres (web centre line). Row 1 has the resistor on the +y face,
# row 2 on the -y face, so both resistors face the gap between the rows and
# SW1 sits on HS4's outer (+y) face, nearest J2.
MID_X = X0 + W / 2
LEFT_X, RIGHT_X = X0 + 28.0, X0 + W - 28.0
SINKS = {"R1": (LEFT_X, 120.5, 0), "R2": (RIGHT_X, 120.5, 0),
         "R3": (LEFT_X, 155.5, 180), "R4": (RIGHT_X, 155.5, 180)}
HOLES = [(X0 + 4.5, 104.0), (X0 + W - 4.5, 104.0), (MID_X, 138.0),
         (X0 + 4.5, 182.0), (X0 + W - 4.5, 182.0)]
J1_POS = (MID_X - 4.0, 178.0)   # 8 mm pair centered on the board
J2_POS = (RIGHT_X - 2.0, 178.0) # directly below the thermostat

NETS = ["BUS+", "BRK_SW", "TH_OPEN", "TH_GND"]


# ---------------------------------------------------------------- footprints

def fp_text(kind, text, x, y, layer, hide=False, size=1.0):
    h = " (hide yes)" if hide else ""
    return (f'\t(property "{kind}" "{text}" (at {x} {y} 0) (layer "{layer}"){h}'
            f' (uuid "{uuid.uuid4()}")\n\t\t(effects (font (size {size} {size})'
            f' (thickness {0.15 if size < 1.1 else SILK_W}))))\n')


def rect(x0, y0, x1, y1, layer, w=0.12):
    return (f'\t(fp_rect (start {x0} {y0}) (end {x1} {y1})'
            f' (stroke (width {w}) (type solid)) (fill no) (layer "{layer}")'
            f' (uuid "{uuid.uuid4()}"))\n')


def line(x0, y0, x1, y1, layer, w=0.12):
    return (f'\t(fp_line (start {x0} {y0}) (end {x1} {y1})'
            f' (stroke (width {w}) (type solid)) (layer "{layer}")'
            f' (uuid "{uuid.uuid4()}"))\n')


def pad(num, x, y, size, drill, shape="circle"):
    sx, sy = size
    if isinstance(drill, tuple):
        d = f"(drill oval {drill[0]} {drill[1]})"
    else:
        d = f"(drill {drill})"
    return (f'\t(pad "{num}" thru_hole {shape} (at {x} {y}) (size {sx} {sy}) {d}'
            f' (layers "*.Cu" "*.Mask") (remove_unused_layers no)'
            f' (uuid "{uuid.uuid4()}"))\n')


def write_fp(name, descr, body, ref_y=-14.5, val_y=14.5, model=None):
    if model:
        body += (f'\t(model "${{KIPRJMOD}}/../models/{model}.wrl"'
                 ' (offset (xyz 0 0 0)) (scale (xyz 1 1 1)) (rotate (xyz 0 0 0)))\n')
    s = (f'(footprint "{name}"\n\t(version 20241229)\n\t(generator "x1_brake_resistor")\n'
         f'\t(layer "F.Cu")\n\t(descr "{descr}")\n'
         + fp_text("Reference", "REF**", 0, ref_y, "F.SilkS")
         + fp_text("Value", name, 0, val_y, "F.Fab")
         + '\t(attr through_hole)\n' + body + ')\n')
    with open(os.path.join(LIB, name + ".kicad_mod"), "w", encoding="utf8") as f:
        f.write(s)


# Ohmite RA-T2X-64E (sink-f-r.pdf p. 2): 42.0 x 25.0 mm, web 1.5 mm thick on
# the centre line, two 2.3 mm pins on the web line 25.4 mm apart, device
# hole 19.1 mm above the board. Origin = sink centre.
SINK_W, SINK_D, SINK_PIN_X = 42.0, 25.0, 12.7
# LTO100 (Vishay 50051): leads 10.16 mm apart, 1.5 x 0.6 mm, about 2.4 mm in
# from the moulded back face, which sits on the web face (y = 0.75).
# Round PTHs avoid the fabricator's minimum aspect ratio for plated slots.
LTO_LEAD_Y = 3.25
# 67L085P (Sensata 6700 p. 3): leads 5.08 mm apart, 0.76 x 0.33 mm, 2.79 mm
# in from the bracket back face, on the opposite face of the web.
TH_LEAD_Y = -(0.75 + 2.79)


def make_footprints():
    os.makedirs(LIB, exist_ok=True)
    hw, hd = SINK_W / 2, SINK_D / 2
    body = (
        pad("1", -5.08, LTO_LEAD_Y, (4.4, 4.0), 3.0, "oval")
        + pad("2", 5.08, LTO_LEAD_Y, (4.4, 4.0), 3.0, "oval")
        # Heatsink pins: unnumbered, so they carry no net and are left out of
        # schematic parity. The sink floats.
        + pad("", -SINK_PIN_X, 0, (4.0, 4.0), 2.7)
        + pad("", SINK_PIN_X, 0, (4.0, 4.0), 2.7)
        + rect(-hw, -hd, hw, hd, "F.SilkS")
        + rect(-hw, -hd, hw, hd, "F.Fab", 0.1)
        + rect(-7.88, 0.75, 7.88, 5.75, "F.Fab", 0.1)       # LTO100 body
        + line(-hw, 0, -SINK_PIN_X - 2.5, 0, "F.SilkS")     # web line
        + line(SINK_PIN_X + 2.5, 0, hw, 0, "F.SilkS")
        + line(-SINK_PIN_X + 2.5, 0, -2.0, 0, "F.SilkS")
        + line(2.0, 0, SINK_PIN_X - 2.5, 0, "F.SilkS")
        # Courtyard: the resistor half of the sink only, so SW1 can occupy the
        # other half on HS4. Nothing else is placed inside a sink outline.
        + rect(-hw - 0.25, -2.25, hw + 0.25, hd + 0.25, "F.CrtYd", 0.05)
    )
    write_fp("LTO100_on_Ohmite_RA-T2X-64E", "Vishay LTO100 (TO-247) screwed to an "
             "Ohmite RA-T2X-64E board-level heatsink; sink pins unnumbered", body,
             ref_y=hd + 1.6, val_y=hd - 1.5, model="lto100-sink-envelope")
    body = (
        pad("1", -2.54, TH_LEAD_Y, (2.6, 2.6), 1.8)
        + pad("2", 2.54, TH_LEAD_Y, (2.6, 2.6), 1.8)
        + rect(-5.08, -0.75, 5.08, -5.45, "F.Fab", 0.1)     # 67L body
        + rect(-hw - 0.25, -hd - 0.25, hw + 0.25, -2.3, "F.CrtYd", 0.05)
    )
    write_fp("Airpax_67L_TO-220-2_on_RA-T2X_web", "Sensata Airpax 67L thermostat, "
             "TO-220-2, on the back face of an Ohmite RA-T2X web; shares the "
             "resistor's M3 screw", body, ref_y=-8.5, val_y=-10.5, model="airpax-67l-envelope")
    body = (pad("1", 0, 0, (5.0, 5.0), 2.0) + pad("2", 0, 8, (5.0, 5.0), 2.0)
            + rect(-2.75, -2.75, 2.75, 10.75, "F.CrtYd", 0.05)
            + rect(-2.5, -2.5, 2.5, 10.5, "F.Fab", 0.1))
    write_fp("PowerWire_P8", "Two solder-wire pads, 2.0 mm holes, 8 mm pitch; "
             "same land as the support board J4", body, ref_y=-4.0, val_y=12.0)



# Draft 3D envelopes for checking placement. The fin details and fasteners are
# simplified. The envelope sizes and the screw height follow the vendor drawings.
def make_models():
    directory = os.path.join(HERE, "models")
    os.makedirs(directory, exist_ok=True)

    def prism(points, bottom, top, color):
        # KiCad VRML uses 2.54 mm units and +Y toward the top of the PCB.
        points = [(x, -y) for x, y in points]
        closed = points[1:] + points[:1]
        if sum(x*v - u*y for (x, y), (u, v) in zip(points, closed)) < 0:
            points.reverse()
        vertices = ", ".join(f"{x/2.54:.6f} {y/2.54:.6f} {z/2.54:.6f}"
                             for z in (bottom, top) for x, y in points)
        n = len(points)
        faces = [list(reversed(range(n))), list(range(n, 2*n))]
        faces.extend([i, (i+1) % n, (i+1) % n + n, i + n] for i in range(n))
        indices = ", ".join(" ".join(map(str, face)) + " -1" for face in faces)
        return (f"Shape {{ appearance Appearance {{ material Material {{ diffuseColor {color} }} }} "
                f"geometry IndexedFaceSet {{ coord Coordinate {{ point [ {vertices} ] }} "
                f"coordIndex [ {indices} ] }} }}\n")

    def box(x0, y0, x1, y1, bottom, top, color):
        return prism([(x0,y0),(x1,y0),(x1,y1),(x0,y1)], bottom, top, color)

    def fin(x0, y0, x1, y1, color):
        length = math.hypot(x1-x0, y1-y0)
        dx, dy = -(y1-y0)*.52/length, (x1-x0)*.52/length
        return prism([(x0+dx,y0+dy),(x1+dx,y1+dy),(x1-dx,y1-dy),(x0-dx,y0-dy)],
                     0, 63.5, color)

    aluminum = "0.23 0.25 0.27"
    package = "0.10 0.105 0.11"
    leads = "0.68 0.69 0.66"
    sink = box(-15, -.75, 15, .75, 0, 63.5, aluminum)
    for side in (-1, 1):
        sink += box(min(12.7*side,15.5*side), -5, max(12.7*side,15.5*side), 5,
                    0, 63.5, aluminum)
        for x, y, xf, yf in [(6.55,-.65,6.55,-12), (9.6,-.6,9.6,-12),
                            (12.7,-.55,13.3,-12), (14,-2,17.5,-11.3),
                            (14,-.7,20.5,-5), (14,0,20.5,0),
                            (14,1,20.5,5), (14,1.8,18.2,10.8),
                            (14,2,16.2,12), (13.3,1.8,13.3,12)]:
            sink += fin(side*x, y, side*xf, yf, aluminum)
    resistor_bottom = 19.1 - (20.7 - 5.33)
    sink += box(-7.88,.75,7.88,5.75,resistor_bottom,resistor_bottom+20.7,package)
    for x in (-5.08,5.08):
        sink += box(x-.75,LTO_LEAD_Y-.3,x+.75,LTO_LEAD_Y+.3,0,resistor_bottom,leads)

    stat_bottom = 19.1 - (15.24 - 2.79)
    stat = box(-5.08,-5.45,5.08,-.75,stat_bottom,21.89,package)
    for x in (-2.54,2.54):
        stat += box(x-.38,TH_LEAD_Y-.165,x+.38,TH_LEAD_Y+.165,0,stat_bottom,leads)

    for name, geometry in (("lto100-sink-envelope",sink),("airpax-67l-envelope",stat)):
        with open(os.path.join(directory, name + ".wrl"), "w", encoding="ascii") as model:
            model.write("#VRML V2.0 utf8\n# Simplified placement envelope in KiCad units\n" + geometry)


# ---------------------------------------------------------------- schematic
import re


def extract_symbol(lib, name):
    path = os.path.join(STOCK_SYM, lib + ".kicad_sym")
    txt = open(path, encoding="utf8").read()
    start = txt.index(f'(symbol "{name}"')
    depth = 0
    for i in range(start, len(txt)):
        if txt[i] == "(":
            depth += 1
        elif txt[i] == ")":
            depth -= 1
            if depth == 0:
                block = txt[start:i + 1]
                break
    pins = {}
    for m in re.finditer(r'\(pin \w+ \w+\s*\(at ([-\d.]+) ([-\d.]+) (\d+)\)'
                         r'.*?\(number "(\d+)"', block, re.S):
        pins[m.group(4)] = (float(m.group(1)), float(m.group(2)), int(m.group(3)))
    block = block.replace(f'(symbol "{name}"', f'(symbol "{lib}:{name}"', 1)
    return block, pins


PARTS = [
    # ref, lib, symbol, value, footprint, x, y, {pin: net}, fields, on_board
    *[(f"R{i}", "Device", "R", "10R LTO100", "BrakeResistor:LTO100_on_Ohmite_RA-T2X-64E",
       50.8 + 25.4 * (i - 1), 81.28, {"1": "BUS+", "2": "BRK_SW"},
       {"MPN": "LTO100F10R00JTE3", "Manufacturer": "Vishay Sfernice",
        "DigiKey": "LT100J-10-ND"}, True) for i in range(1, 5)],
    ("SW1", "Switch", "SW_SPST", "67L085P NC opens 85C",
     "BrakeResistor:Airpax_67L_TO-220-2_on_RA-T2X_web", 177.8, 81.28,
     {"1": "TH_GND", "2": "TH_OPEN"},
     {"MPN": "67L085P", "Manufacturer": "Sensata Airpax", "DigiKey": "723-1348-ND"}, True),
    ("J1", "Connector_Generic", "Conn_01x02", "to support J4",
     "BrakeResistor:PowerWire_P8", 50.8, 132.08, {"1": "BUS+", "2": "BRK_SW"}, {}, True),
    ("J2", "Connector_Generic", "Conn_01x02", "to support J6 (JST PH)",
     "Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical", 177.8, 132.08,
     {"1": "TH_OPEN", "2": "TH_GND"},
     {"MPN": "B2B-PH-K-S", "Manufacturer": "JST", "DigiKey": "455-1704-ND"}, True),
    *[(f"HS{i}", "Mechanical", "Heatsink", "RA-T2X-64E", "", 50.8 + 25.4 * (i - 1), 109.22,
       {}, {"MPN": "RA-T2X-64E", "Manufacturer": "Ohmite", "DigiKey": "RA-T2X-64E-ND"},
       False) for i in range(1, 5)],
    *[(f"H{i}", "Mechanical", "MountingHole", "M3",
       "MountingHole:MountingHole_3.2mm_M3", 101.6 + 15.24 * (i - 1), 160.02, {}, {}, True)
      for i in range(1, len(HOLES) + 1)],
]


def build_schematic(path):
    syms = {}
    for _, lib, sym, *_ in PARTS:
        syms.setdefault(f"{lib}:{sym}", extract_symbol(lib, sym))
    root = str(uuid.uuid4())
    sym_ids, out = {}, []
    for ref, lib, sym, value, fp, x, y, pin_nets, fields, on_board in PARTS:
        _, pins = syms[f"{lib}:{sym}"]
        sid = str(uuid.uuid4())
        sym_ids[ref] = sid
        in_bom = "no" if sym == "MountingHole" else "yes"
        ob = "yes" if on_board else "no"
        s = [f'\t(symbol (lib_id "{lib}:{sym}") (at {x:.2f} {y:.2f} 0) (unit 1)',
             f'\t\t(exclude_from_sim no) (in_bom {in_bom}) (on_board {ob}) (dnp no)',
             f'\t\t(uuid "{sid}")']
        props = [("Reference", ref, -6, False), ("Value", value, 6, False),
                 ("Footprint", fp, 8, True), ("Datasheet", "", 10, True)]
        props += [(k, v, 12, True) for k, v in fields.items()]
        for k, v, dy, hide in props:
            h = " (hide yes)" if hide else ""
            s.append(f'\t\t(property "{k}" "{v}" (at {x + 3:.2f} {y + dy:.2f} 0)'
                     f' (effects (font (size 1.27 1.27)) (justify left){h}))')
        for num in pins:
            s.append(f'\t\t(pin "{num}" (uuid "{uuid.uuid4()}"))')
        s.append(f'\t\t(instances (project "{NAME}"'
                 f' (path "/{root}" (reference "{ref}") (unit 1))))')
        s.append('\t)')
        out.append("\n".join(s))
        for num, net in pin_nets.items():
            px, py, ang = pins[num]
            lx, ly = x + px, y - py
            # Label points away from the pin's body side.
            rot = {0: 180, 180: 0, 90: 270, 270: 90}[ang]
            just = {180: "right", 0: "left", 270: "right", 90: "left"}[rot]
            out.append(f'\t(global_label "{net}" (shape passive)'
                       f' (at {lx:.2f} {ly:.2f} {rot}) (fields_autoplaced yes)'
                       f' (effects (font (size 1.27 1.27)) (justify {just}))'
                       f' (uuid "{uuid.uuid4()}"))')
    notes = [
        "Brake resistor bank for the X1 support board chopper (Q3). 4 x 10 ohm LTO100 in parallel = 2.5 ohm.",
        "J1 -> support J4 (1 = BUS+, 2 = BRK_SW), 16 AWG pair. J2 -> support J6 (JST PH to PH, 1 = TH_OPEN, 2 = GND).",
        "Each R on its own Ohmite RA-T2X-64E with silicone grease, M3 screw 1 Nm. SW1 (67L085P, plastic bracket)",
        "shares R4's screw on the far face of HS4's web. SW1 opens at 85 +/- 5 C: support blocks forced discharge.",
        "Rating (brake_sizing.py): 41 W average at 40 C ambient; events up to 150 J at <= 200 W from hot.",
    ]
    text = "".join(f'\t(text "{n}" (exclude_from_sim no) (at 30 {190 + 6 * i} 0)\n'
                   f'\t\t(effects (font (size 1.6 1.6)) (justify left bottom))'
                   f' (uuid "{uuid.uuid4()}"))\n' for i, n in enumerate(notes))
    lib_symbols = "\n".join(b for b, _ in syms.values())
    s = f"""(kicad_sch
\t(version 20250114)
\t(generator "x1_brake_resistor")
\t(generator_version "10.0")
\t(uuid "{root}")
\t(paper "A4")
\t(title_block (title "X1 brake resistor board") (date "2026-11-02") (rev "B")
\t\t(comment 1 "2.5 ohm bank for the support board chopper; sizing in brake_sizing.py"))
\t(lib_symbols
{lib_symbols}
\t)
{chr(10).join(out)}
{text}\t(sheet_instances (path "/" (page "1")))
)
"""
    with open(path, "w", encoding="utf8") as f:
        f.write(s)
    return sym_ids


# ---------------------------------------------------------------- board

def mm(v):
    return pcbnew.FromMM(v)


def pt(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))


def load_fp(lib_dir, name, nick):
    fp = pcbnew.FootprintLoad(lib_dir, name)
    if fp is None:
        raise SystemExit(f"footprint {name} not found in {lib_dir}")
    fp.SetFPID(pcbnew.LIB_ID(nick, name))
    return fp


def silk(board, text, x, y, size=SILK_H, layer=pcbnew.F_SilkS, justify=None):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(text)
    t.SetPosition(pt(x, y))
    t.SetLayer(layer)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
    t.SetTextThickness(mm(SILK_W))
    if justify == "left":
        t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT)
    if layer == pcbnew.B_SilkS:
        t.SetMirrored(True)
    board.Add(t)


def track(board, net, pts, width=TH_TRACK, layer=pcbnew.F_Cu):
    for a, b in zip(pts, pts[1:]):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pt(*a))
        t.SetEnd(pt(*b))
        t.SetWidth(mm(width))
        t.SetLayer(layer)
        t.SetNet(net)
        board.Add(t)


def build_board(sym_ids):
    board = pcbnew.BOARD()
    board.SetCopperLayerCount(2)
    ds = board.GetDesignSettings()
    ds.m_TrackMinWidth = mm(0.2)
    ds.m_MinClearance = mm(0.2)
    ds.m_ViasMinSize = mm(0.6)
    ds.m_MinThroughDrill = mm(0.3)
    ds.m_HoleClearance = mm(0.3)
    ds.m_HoleToHoleMin = mm(0.5)
    ds.m_CopperEdgeClearance = mm(0.5)
    ds.m_MinSilkTextHeight = mm(1.0)
    ds.m_MinSilkTextThickness = mm(0.15)
    nc = ds.m_NetSettings.GetDefaultNetclass()
    nc.SetClearance(mm(CLEARANCE))
    nc.SetTrackWidth(mm(1.0))
    th = pcbnew.NETCLASS("THERMO")
    th.SetClearance(mm(TH_CLEARANCE))
    th.SetTrackWidth(mm(TH_TRACK))
    ds.m_NetSettings.SetNetclass("THERMO", th)
    ds.m_NetSettings.SetNetclassPatternAssignment("TH_*", "THERMO")

    nets = {}
    for n in NETS:
        ni = pcbnew.NETINFO_ITEM(board, n)
        board.Add(ni)
        nets[n] = ni

    outline = [(X0, Y0), (X0 + W, Y0), (X0 + W, Y0 + H), (X0, Y0 + H)]
    for (a, b) in zip(outline, outline[1:] + outline[:1]):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(pt(*a))
        s.SetEnd(pt(*b))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(mm(0.1))
        board.Add(s)

    fps = {}
    for ref, (x, y, rot) in SINKS.items():
        fp = load_fp(LIB, "LTO100_on_Ohmite_RA-T2X-64E", "BrakeResistor")
        fp.SetReference(ref)
        fp.SetValue("10R LTO100")
        fp.SetPosition(pt(x, y))
        fp.SetOrientationDegrees(rot)
        board.Add(fp)
        fps[ref] = fp
    x4, y4, r4 = SINKS["R4"]
    sw = load_fp(LIB, "Airpax_67L_TO-220-2_on_RA-T2X_web", "BrakeResistor")
    sw.SetReference("SW1")
    sw.SetValue("67L085P")
    sw.SetPosition(pt(x4, y4))
    sw.SetOrientationDegrees(r4)
    board.Add(sw)
    fps["SW1"] = sw
    j1 = load_fp(LIB, "PowerWire_P8", "BrakeResistor")
    j1.SetReference("J1")
    j1.SetValue("to support J4")
    j1.SetPosition(pt(*J1_POS))
    j1.SetOrientationDegrees(90)   # pad 2 lands 8 mm to the right of pad 1
    board.Add(j1)
    fps["J1"] = j1
    j2 = load_fp(os.path.join(STOCK_FP, "Connector_JST.pretty"),
                 "JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical", "Connector_JST")
    j2.SetReference("J2")
    j2.SetValue("to support J6")
    j2.SetPosition(pt(*J2_POS))
    board.Add(j2)
    fps["J2"] = j2
    for i, (x, y) in enumerate(HOLES, 1):
        h = load_fp(os.path.join(STOCK_FP, "MountingHole.pretty"),
                    "MountingHole_3.2mm_M3", "MountingHole")
        h.SetReference(f"H{i}")
        h.SetValue("M3")
        h.Reference().SetVisible(False)
        h.SetPosition(pt(x, y))
        board.Add(h)
        fps[f"H{i}"] = h

    pin_nets = {ref: nets_ for ref, _, _, _, _, _, _, nets_, _, ob in PARTS if ob}
    parts = {p[0]: p for p in PARTS if p[9]}
    for ref, fp in fps.items():
        for p in fp.Pads():
            n = pin_nets.get(ref, {}).get(p.GetNumber())
            if n:
                p.SetNet(nets[n])
        sid = sym_ids.get(ref)
        if sid:
            fp.SetPath(pcbnew.KIID_PATH("/" + sid))
        part = parts.get(ref)
        if part:
            fp.SetValue(part[3])
            for k, v in part[8].items():
                fp.SetField(k, v)
                fp.GetField(k).SetVisible(False)
                fp.GetField(k).SetLayer(pcbnew.F_Fab)
        fp.Reference().SetTextSize(pcbnew.VECTOR2I(mm(SILK_H), mm(SILK_H)))
        fp.Reference().SetTextThickness(mm(SILK_W))
    for p in j1.Pads():
        p.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)

    def pad_xy(fp, num):
        p = [q for q in fp.Pads() if q.GetNumber() == num][0].GetPosition()
        return round(pcbnew.ToMM(p.x), 3), round(pcbnew.ToMM(p.y), 3)

    # Thermostat pair: SW1 -> J2, straight down past HS4's outer edge.
    s_open, s_gnd = pad_xy(sw, "2"), pad_xy(sw, "1")
    j_open, j_gnd = pad_xy(j2, "1"), pad_xy(j2, "2")
    assert s_open[0] < s_gnd[0] and j_open[0] < j_gnd[0], (s_open, s_gnd, j_open, j_gnd)
    yb = 171.0
    track(board, nets["TH_OPEN"], [s_open, (s_open[0], yb),
                                   (j_open[0], yb + abs(j_open[0] - s_open[0])), j_open])
    track(board, nets["TH_GND"], [s_gnd, (s_gnd[0], yb),
                                  (j_gnd[0], yb + abs(j_gnd[0] - s_gnd[0])), j_gnd])

    for ref, (x, y, rot) in SINKS.items():
        # Conservative R-series profile with margin around the web and fins.
        # Leave the central lead opening clear for the resistor and thermostat.
        keepout = pcbnew.ZONE(board)
        keepout.SetLayer(pcbnew.F_Cu)
        keepout.SetZoneName(f"{ref} heatsink base")
        keepout.SetIsRuleArea(True)
        keepout.SetDoNotAllowZoneFills(True)
        keepout.SetDoNotAllowTracks(True)
        keepout.SetDoNotAllowVias(True)
        keepout.SetDoNotAllowPads(False)
        keepout.SetDoNotAllowFootprints(False)
        contour = keepout.Outline()
        contour.NewOutline()
        sign = 1 if rot == 0 else -1
        for dx, dy in ((-21.5, -13), (-5.5, -13), (-5.5, -1.25),
                       (5.5, -1.25), (5.5, -13), (21.5, -13),
                       (21.5, 13), (12, 13), (12, 1.25),
                       (-12, 1.25), (-12, 13), (-21.5, 13)):
            contour.Append(pt(x + sign * dx, y + sign * dy))
        board.Add(keepout)

    for layer, net in ((pcbnew.F_Cu, "BUS+"), (pcbnew.B_Cu, "BRK_SW")):
        z = pcbnew.ZONE(board)
        z.SetLayer(layer)
        z.SetNet(nets[net])
        z.SetLocalClearance(mm(PLANE_CLEARANCE))
        z.SetMinThickness(mm(0.3))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
        z.SetThermalReliefGap(mm(0.6))
        z.SetThermalReliefSpokeWidth(mm(1.5))
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        ol = z.Outline()
        ol.NewOutline()
        for x, y in outline:
            ol.Append(mm(x), mm(y))
        board.Add(z)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())

    # Labels. Pin numbers at the connectors, function and assembly notes.
    for fp in (j1, j2):
        fp.Reference().SetVisible(False)   # the captions below name them
    jx, jy = J1_POS
    silk(board, "BUS+", jx, jy + 4.2)
    silk(board, "BRK_SW", jx + 8, jy + 4.2)
    silk(board, "J1 > support J4", jx + 4, jy - 4.2)
    silk(board, "1", jx - 3.6, jy)
    kx, ky = J2_POS
    silk(board, "J2 > J6", kx + 1, ky + 4.8)
    lines = ["X1 BRAKE RES 2.5R rev B",
             "4x LTO100 10R, grease",
             "M3 1 Nm  |  SW1 on HS4",
             "85C NC THERMOSTAT"]
    for i, t in enumerate(lines):
        silk(board, t, X0 + 11.0, 173.0 + 1.8 * i, size=1.0, justify="left")
    return board


def write_project():
    pro = {"meta": {"filename": f"{NAME}.kicad_pro", "version": 3},
           "board": {"design_settings": {"rules": {
               "min_clearance": 0.2, "min_track_width": 0.2,
               "min_via_diameter": 0.6, "min_via_annular_width": 0.15,
               "min_through_hole_diameter": 0.3, "min_hole_to_hole": 0.5,
               "min_hole_clearance": 0.3, "min_copper_edge_clearance": 0.5,
               "min_text_height": 1.0, "min_text_thickness": 0.15,
               "solder_mask_to_copper_clearance": 0.0, "min_silk_clearance": 0.0}}},
           "net_settings": {"meta": {"version": 3}, "classes": [
               {"name": "Default", "clearance": CLEARANCE, "track_width": 1.0,
                "via_diameter": 0.8, "via_drill": 0.4, "microvia_diameter": 0.3,
                "microvia_drill": 0.1, "diff_pair_width": 0.2, "diff_pair_gap": 0.25,
                "wire_width": 6, "bus_width": 12, "line_style": 0,
                "schematic_color": "rgba(0, 0, 0, 0.000)",
                "pcb_color": "rgba(0, 0, 0, 0.000)", "priority": 2147483647},
               {"name": "THERMO", "clearance": TH_CLEARANCE, "track_width": TH_TRACK,
                "via_diameter": 0.8, "via_drill": 0.4, "microvia_diameter": 0.3,
                "microvia_drill": 0.1, "diff_pair_width": 0.2, "diff_pair_gap": 0.25,
                "wire_width": 6, "bus_width": 12, "line_style": 0,
                "schematic_color": "rgba(0, 0, 0, 0.000)",
                "pcb_color": "rgba(0, 0, 0, 0.000)", "priority": 0}],
               "netclass_patterns": [{"netclass": "THERMO", "pattern": "TH_*"}]},
           "sheets": [], "boards": [], "libraries": {}}
    with open(os.path.join(PROJ, f"{NAME}.kicad_pro"), "w") as f:
        json.dump(pro, f, indent=2)
    with open(os.path.join(PROJ, "fp-lib-table"), "w") as f:
        f.write('(fp_lib_table\n\t(version 7)\n'
                '\t(lib (name "BrakeResistor") (type "KiCad") '
                '(uri "${KIPRJMOD}/../BrakeResistor.pretty") (options "") '
                '(descr "Brake resistor board land patterns"))\n)\n')


def main():
    os.makedirs(PROJ, exist_ok=True)
    make_models()
    make_footprints()
    board_path = os.path.join(PROJ, f"{NAME}.kicad_pcb")
    if sys.argv[1:] == ["--layout-only"]:
        existing = pcbnew.LoadBoard(board_path)
        sym_ids = {
            fp.GetReference(): fp.GetPath().AsString().rsplit("/", 1)[-1]
            for fp in existing.GetFootprints()
        }
    elif not sys.argv[1:]:
        write_project()
        sym_ids = build_schematic(os.path.join(PROJ, f"{NAME}.kicad_sch"))
    else:
        raise SystemExit("usage: generate.py [--layout-only]")
    board = build_board(sym_ids)
    board.Save(board_path)
    print("wrote", PROJ)


if __name__ == "__main__":
    main()
