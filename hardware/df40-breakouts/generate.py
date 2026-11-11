"""Generate the two X1 stack (Hirose DF40, 80 pin) breakout boards.

Run with KiCad's bundled Python (builds both boards, one process each):
    "%LOCALAPPDATA%/Programs/KiCad/10.0/bin/python.exe" generate.py

The pinout is read from J_STACK1 on the three stack boards, which must agree.
The X1 boards are only read, never written.

  DF40_Header_Breakout      DF40C-80DP-0.4V header on top. The logic board
                            plugs onto it exactly as it plugs onto a power board.
  DF40_Receptacle_Breakout  DF40HC(3.0)-80DS-0.4V receptacle on the bottom. It
                            plugs onto a power board in place of the logic board.

Both boards share one outline and one fan-out, mirrored between layers. Near the
connector the outline stays inside the area the logic board already occupies
and clears P96's FAN1 and 5V IN pads. The 0.1" headers sit beyond the rim of
both power boards, so no through-hole pin lands over a power board.

Routing is generated here rather than autorouted: Freerouting could not escape
the 0.4 mm pitch. The board is 2-layer. The odd DF40 row runs on the connector
layer to J2. The even row fans out below the connector, drops through two
staggered via rows and runs on the other layer to J3. Spare copper on both
layers is ground pour.
"""
import json
import os
import re
import shutil
import subprocess
import sys
import uuid

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
TWO_STORY = os.path.normpath(os.path.join(HERE, "..", "two_story"))
KICAD_SHARE = os.path.normpath(
    os.path.join(os.path.dirname(sys.executable), "..", "share", "kicad"))
STOCK_FP = os.path.join(KICAD_SHARE, "footprints")
STOCK_SYM = os.path.join(KICAD_SHARE, "symbols")
LOCAL_LIB = os.path.join(HERE, "X1_Breakout.pretty")

# Copied verbatim from the stack boards' X1_Draft library so the breakouts use
# the same land patterns as the boards they mate with.
DF40_FOOTPRINTS = {
    "Hirose_DF40C-80DP-0.4V_2x40-1MP_P0.4mm": "Power_P96",
    "Hirose_DF40HC(3.0)-80DS-0.4V_2x40_P0.4mm": "Common_Logic",
}
SOURCES = {
    "Common_Logic": "Common_Logic/moteus_x1.kicad_pcb",
    "Power_Reference": "Power_Reference/moteus_x1.kicad_pcb",
    "Power_P96": "Power_P96/moteus_x1.kicad_pcb",
}

# J_STACK1 sits at (150, 108), rotation 0, on every stack board. The breakouts
# keep those coordinates so the spacer holes copy across unchanged.
CONN = (150.0, 108.0)
# The two spacer positions present on Common_Logic, Power_Reference and P96.
# The other stack holes fall inside the fan-out and are left off.
STACK_HOLES = [(135.0, 115.0), (165.0, 115.0)]

# Below the connector the outline follows the logic board, which also covers
# P96's FAN1 socket; that FAN1/logic-board clash is open on the stack itself.
OUTLINE = [(68.5, 59.0), (122.5, 59.0), (122.5, 89), (168, 89), (168, 118.5),
           (131.5, 118.5), (131.5, 111), (134.5, 111), (134.5, 107),
           (121, 107), (121, 139), (68.5, 139)]

# J2 carries the odd DF40 row, J3 the even row; header pin k = DF40 pin 2k-1
# (J2) or 2k (J3). Pin 1 positions and rotations put the pins in bus order.
J2_PIN1, J2_ROT = (71.8, 76.54), 90     # pins 39/40 at the right, even pins on top
J3_PIN1, J3_ROT = (118.86, 121.0), -90  # pins 1/2 at the right, even pins below

# Two layers, kept inside JLCPCB's standard (no-surcharge) 2-layer limits
# (jlcpcb.com/capabilities/pcb-capabilities, read 2026-11-02): trace/space
# 0.10 mm; 0.3 mm via drill; via annular ring 0.15 mm minimum; via hole to
# track 0.2 mm; silkscreen text 1.0 mm tall with lines of at least 0.15 mm;
# copper to routed edge 0.2 mm. With no plane under the 0.4 mm-pitch buses,
# neighbours couple strongly to fast edges; drive these lines with slow slew
# (see README). A 4-layer build with ground inner layers is commit f3ce734.
TRACK = 0.127
SILK_H, SILK_W = 1.0, 0.17
CLEARANCE = 0.127
VIA_D, VIA_DRILL = 0.6, 0.3
BUS_TOP = 105.8         # y of bus slot 0; slot i is 0.4 mm higher per index
BUS_PITCH = 0.4
# The even row fans out below the connector from 0.4 mm to VIA_PITCH, centred
# on the connector, then drops through two staggered via rows.
VIA_PITCH = 0.513
FAN_START = 0.45         # straight stub below the even pads before the fan
VIA_GAP = 0.45           # fan end to via centre
VIA_ROW_STEP = 0.75      # odd-indexed vias sit this much lower

SHORT = {
    "VBAT_SENSE": "VBAT_SNS", "MOTOR_ENABLE": "MOTOR_EN",
    "DRV8323_FAULT": "DRV_FAULT", "DRV8323_SCLK": "DRV_SCLK",
    "DRV8323_MOSI": "DRV_MOSI", "DRV8323_MISO": "DRV_MISO",
    "DRV8323_CS": "DRV_CS", "ENC_ROTOR_CS": "ENC_ROT_CS",
    "ENC_OUTPUT_CS": "ENC_OUT_CS", "ENC_SPI_SCK": "ENC_SCK",
    "ENC_SPI_MISO": "ENC_MISO", "ENC_SPI_MOSI": "ENC_MOSI",
    "RSV_MATE1_RTN": "RSV_MATE1R", "RSV_MATE2_RTN": "RSV_MATE2R",
    "RSV_BRAKE_COIL_EN": "RSV_BRK_EN", "RSV_LIMIT_IN": "RSV_LIMIT",
    "RSV_HARD_INHIBIT": "RSV_INHIB",
}


def clean_net(name):
    m = re.match(r"unconnected-\(J_STACK1-RESERVE_(.+)-Pad\d+\)", name)
    if m:
        name = "RSV_" + m.group(1)
    name = name.replace("{slash}", "/").split("/")[-1]
    return SHORT.get(name, name)


def read_pinout():
    maps = {}
    for key, rel in SOURCES.items():
        board = pcbnew.LoadBoard(os.path.join(TWO_STORY, rel))
        fp = board.FindFootprintByReference("J_STACK1")
        maps[key] = {p.GetNumber(): p.GetNetname() for p in fp.Pads()
                     if p.GetNumber().isdigit()}
    ref = maps["Power_P96"]
    for key, m in maps.items():
        if m != ref:
            diff = [n for n in ref if m.get(n) != ref[n]]
            raise SystemExit(f"{key} J_STACK1 pinout differs at pads {diff}")
    pinout = {int(n): clean_net(v) for n, v in ref.items()}
    assert sorted(pinout) == list(range(1, 81))
    return pinout


def mm(v):
    return pcbnew.FromMM(v)


def pt(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))


def xy(p):
    return (round(pcbnew.ToMM(p.x), 4), round(pcbnew.ToMM(p.y), 4))


def load_fp(lib_dir, name, nickname):
    fp = pcbnew.FootprintLoad(lib_dir, name)
    if fp is None:
        raise SystemExit(f"footprint {name} not found in {lib_dir}")
    fp.SetFPID(pcbnew.LIB_ID(nickname, name))
    return fp


def pads_by_number(fp):
    return {int(p.GetNumber()): p for p in fp.Pads() if p.GetNumber().isdigit()}


def silk_text(board, text, x, y, layer=pcbnew.F_SilkS, size=SILK_H, thick=SILK_W,
              justify=None, angle=0):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(text)
    t.SetPosition(pt(x, y))
    t.SetLayer(layer)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
    t.SetTextThickness(mm(thick))
    if angle:
        t.SetTextAngleDegrees(angle)
    if justify == "left":
        t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT)
    elif justify == "right":
        t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_RIGHT)
    if layer == pcbnew.B_SilkS:
        t.SetMirrored(True)
    board.Add(t)


HOLE_FP = ("MountingHole", "MountingHole_2.7mm_M2.5")


def add_hole(board, ref, x, y):
    fp = load_fp(os.path.join(STOCK_FP, HOLE_FP[0] + ".pretty"), HOLE_FP[1],
                 HOLE_FP[0])
    fp.SetReference(ref)
    fp.SetValue("MountingHole")
    fp.Reference().SetVisible(False)
    fp.SetPosition(pt(x, y))
    board.Add(fp)


CHAMFER = 0.3  # 45 degree cut at each right-angle bend


def chamfer(points):
    """Replace each right-angle bend with a 45 degree segment. The cut is the
    same on every trace of a bundle, so parallel traces stay parallel."""
    out = [points[0]]
    for prev, cur, nxt in zip(points, points[1:], points[2:]):
        ax, ay = cur[0] - prev[0], cur[1] - prev[1]
        bx, by = nxt[0] - cur[0], nxt[1] - cur[1]
        la, lb = (ax * ax + ay * ay) ** 0.5, (bx * bx + by * by) ** 0.5
        if la == 0 or lb == 0 or abs(ax * bx + ay * by) > 1e-9:
            out.append(cur)
            continue
        d = min(CHAMFER, la / 2, lb / 2)
        out.append((round(cur[0] - ax / la * d, 4), round(cur[1] - ay / la * d, 4)))
        out.append((round(cur[0] + bx / lb * d, 4), round(cur[1] + by / lb * d, 4)))
    out.append(points[-1])
    return out


def add_track(board, net, layer, points, width=TRACK):
    points = chamfer(points)
    for a, b in zip(points, points[1:]):
        if a == b:
            continue
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pt(*a))
        t.SetEnd(pt(*b))
        t.SetWidth(mm(width))
        t.SetLayer(layer)
        t.SetNet(net)
        board.Add(t)


def add_via(board, net, x, y):
    v = pcbnew.PCB_VIA(board)
    v.SetPosition(pt(x, y))
    v.SetWidth(mm(VIA_D))
    v.SetDrill(mm(VIA_DRILL))
    v.SetViaType(pcbnew.VIATYPE_THROUGH)
    v.SetNet(net)
    v.SetFrontTentingMode(pcbnew.TENTING_MODE_TENTED)
    v.SetBackTentingMode(pcbnew.TENTING_MODE_TENTED)
    board.Add(v)


def add_planes(board):
    gnd = board.FindNet("GND")
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        z = pcbnew.ZONE(board)
        z.SetLayer(layer)
        z.SetNet(gnd)
        z.SetLocalClearance(mm(0.2))
        z.SetMinThickness(mm(0.2))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THT_THERMAL)  # DF40 GND pads already have traces
        z.SetThermalReliefGap(mm(0.25))
        z.SetThermalReliefSpokeWidth(mm(0.3))
        ol = z.Outline()
        ol.NewOutline()
        for x, y in OUTLINE:
            ol.Append(mm(x), mm(y))
        board.Add(z)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())


STITCH_PITCH = 2.5  # mm grid for ground stitching vias


def add_stitching(board):
    """Tie the two ground pours together with a grid of tented vias, placed only
    where the via and a 0.15 mm margin sit inside the filled pour on both layers
    and stay clear of every pad and existing via. Refills the pours after."""
    gnd = board.FindNet("GND")
    fills = []
    for z in board.Zones():
        fills.append(z.GetFilledPolysList(z.GetLayer()))
    pads = [(xy(p.GetPosition()), max(pcbnew.ToMM(p.GetSize().x), pcbnew.ToMM(p.GetSize().y)) / 2)
            for p in board.GetPads()]
    vias = [xy(t.GetPosition()) for t in board.GetTracks() if t.GetClass() == "PCB_VIA"]
    xs = [x for x, _ in OUTLINE]
    ys = [y for _, y in OUTLINE]
    r = VIA_D / 2 + 0.15
    ring = [(r * c, r * s_) for c, s_ in ((1, 0), (-1, 0), (0, 1), (0, -1),
                                         (0.7071, 0.7071), (-0.7071, 0.7071),
                                         (0.7071, -0.7071), (-0.7071, -0.7071))]
    placed = 0
    y = min(ys) + STITCH_PITCH / 2
    while y < max(ys):
        x = min(xs) + STITCH_PITCH / 2
        while x < max(xs):
            ok = all(fill.Contains(pt(x + dx, y + dy))
                     for fill in fills for dx, dy in ring + [(0, 0)])
            ok = ok and all((x - px) ** 2 + (y - py) ** 2 > (pr + 0.6) ** 2
                            for (px, py), pr in pads)
            ok = ok and all((x - vx) ** 2 + (y - vy) ** 2 > 1.2 ** 2 for vx, vy in vias)
            if ok:
                add_via(board, gnd, round(x, 3), round(y, 3))
                placed += 1
            x += STITCH_PITCH
        y += STITCH_PITCH
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    return placed


def route(board, j1, j2, j3, conn_layer, other_layer):
    """Odd row: connector layer, up out of the pads, left along the bus, up to
    J2. Even row: down to a via, then the other layer, up under the connector,
    left along the bus, down to J3. Slot i of both buses sits at
    BUS_TOP - 0.4 i, so the leftmost pad turns lowest and nothing crosses.
    Both buses turn off in pad order: the topmost odd trace turns first, the
    bottommost even trace turns first. The even fan is symmetric about the
    connector centre, so the 45 degree legs stay parallel and never cross."""
    p1, p2, p3 = pads_by_number(j1), pads_by_number(j2), pads_by_number(j3)
    for i in range(40):
        y_bus = BUS_TOP - BUS_PITCH * i
        for row, hdr, pad_no in (("odd", p2, 2 * i + 1), ("even", p3, 2 * i + 2)):
            src = p1[pad_no]
            net = src.GetNet()
            sx, sy = xy(src.GetPosition())
            dst = hdr[i + 1]
            dx, dy = xy(dst.GetPosition())
            # Header pins 2m-1 sit on the inner row (nearer the bus), 2m outer.
            outer = (i + 1) % 2 == 0
            if row == "odd":
                turn_x = dx + 1.27 if outer else dx
                add_track(board, net, conn_layer,
                          [(sx, sy), (sx, y_bus), (turn_x, y_bus),
                           (turn_x, dy), (dx, dy)])
            else:
                vx = round(CONN[0] + VIA_PITCH * (i - 19.5), 4)
                ys = round(sy + FAN_START, 4)
                ye = round(ys + abs(vx - sx), 4)
                vy = round(ye + VIA_GAP + VIA_ROW_STEP * (i % 2), 4)
                add_track(board, net, conn_layer,
                          [(sx, sy), (sx, ys), (vx, ye), (vx, vy)])
                add_via(board, net, vx, vy)
                turn_x = dx - 1.27 if outer else dx
                add_track(board, net, other_layer,
                          [(vx, vy), (vx, y_bus), (turn_x, y_bus),
                           (turn_x, dy), (dx, dy)])


def link_shared_pins(board, j1, conn_layer):
    """Tie the +5V and +3V contacts together under the connector, between the
    two pad rows, which the fan-out leaves empty. GND is tied by the planes."""
    pads = pads_by_number(j1)
    y_mid = round((xy(pads[1].GetPosition())[1] + xy(pads[2].GetPosition())[1]) / 2, 4)
    groups = {}
    for n, pad in pads.items():
        name = pad.GetNetname()
        if name != "GND":
            groups.setdefault(name, []).append(pad)
    for name, members in groups.items():
        if len(members) < 2:
            continue
        net = members[0].GetNet()
        at = {}
        for p in members:
            at.setdefault(xy(p.GetPosition())[0], []).append(xy(p.GetPosition()))
        xs = sorted(at)
        # A lone pad at either end joins the run as one bend, so it chamfers.
        line = [(xs[0], y_mid), (xs[-1], y_mid)]
        ends = set()
        if len(at[xs[0]]) == 1:
            line.insert(0, at[xs[0]][0])
            ends.add(at[xs[0]][0])
        if len(at[xs[-1]]) == 1:
            line.append(at[xs[-1]][0])
            ends.add(at[xs[-1]][0])
        add_track(board, net, conn_layer, line)
        for pads in at.values():
            for px, py in pads:
                if (px, py) not in ends:
                    add_track(board, net, conn_layer, [(px, py), (px, y_mid)])


def build_board(variant, pinout, sym_ids):
    board = pcbnew.BOARD()
    board.SetCopperLayerCount(2)
    ds = board.GetDesignSettings()
    ds.SetCopperLayerCount(2)
    ds.m_TrackMinWidth = mm(TRACK)
    ds.m_TentViasFront = True
    ds.m_TentViasBack = True
    ds.m_ViasMinAnnularWidth = mm(0.15)
    ds.m_MinSilkTextHeight = mm(SILK_H)
    ds.m_MinSilkTextThickness = mm(0.15)
    ds.m_MinClearance = mm(CLEARANCE)
    ds.m_ViasMinSize = mm(VIA_D)
    ds.m_MinThroughDrill = mm(VIA_DRILL)
    ds.m_HoleToHoleMin = mm(0.25)
    ds.m_HoleClearance = mm(0.2)
    ds.m_CopperEdgeClearance = mm(0.3)
    nc = ds.m_NetSettings.GetDefaultNetclass()
    nc.SetTrackWidth(mm(TRACK))
    nc.SetClearance(mm(CLEARANCE))
    nc.SetViaDiameter(mm(VIA_D))
    nc.SetViaDrill(mm(VIA_DRILL))

    nets = {}
    for name in sorted(set(pinout.values())):
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        nets[name] = ni

    for (x0, y0), (x1, y1) in zip(OUTLINE, OUTLINE[1:] + OUTLINE[:1]):
        seg = pcbnew.PCB_SHAPE(board)
        seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
        seg.SetStart(pt(x0, y0))
        seg.SetEnd(pt(x1, y1))
        seg.SetLayer(pcbnew.Edge_Cuts)
        seg.SetWidth(mm(0.1))
        board.Add(seg)

    # Stack connector: same footprint, side and orientation as the board it
    # stands in for, so pad n lands on the mating pad n.
    if variant == "header":
        j1 = load_fp(LOCAL_LIB, "Hirose_DF40C-80DP-0.4V_2x40-1MP_P0.4mm", "X1_Breakout")
        j1.SetPosition(pt(*CONN))
        board.Add(j1)
        j1.SetValue("DF40C-80DP-0.4V(51)")
        conn_layer, other_layer = pcbnew.F_Cu, pcbnew.B_Cu
        want = (142.2, 106.645)
    else:
        j1 = load_fp(LOCAL_LIB, "Hirose_DF40HC(3.0)-80DS-0.4V_2x40_P0.4mm",
                     "X1_Breakout")
        j1.SetPosition(pt(*CONN))
        board.Add(j1)  # Flip needs the footprint to belong to a board first.
        j1.Flip(pt(*CONN), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
        j1.SetValue("DF40HC(3.0)-80DS-0.4V(51)")
        conn_layer, other_layer = pcbnew.B_Cu, pcbnew.F_Cu
        want = (142.2, 106.46)
    j1.SetReference("J1")
    j1.Reference().SetVisible(False)
    for n, pad in pads_by_number(j1).items():
        pad.SetNet(nets[pinout[n]])
    # Pad 1 must sit where it sits on the stack board this one replaces.
    got = xy(pads_by_number(j1)[1].GetPosition())
    assert got == want, f"{variant}: J1 pad 1 at {got}, expected {want}"

    hdr_lib = os.path.join(STOCK_FP, "Connector_PinHeader_2.54mm.pretty")
    headers = {}
    for ref, pin1, rot, parity in (("J2", J2_PIN1, J2_ROT, 1),
                                   ("J3", J3_PIN1, J3_ROT, 0)):
        h = load_fp(hdr_lib, "PinHeader_2x20_P2.54mm_Vertical",
                    "Connector_PinHeader_2.54mm")
        h.SetReference(ref)
        h.SetValue("Conn_02x20 2.54mm")
        h.Reference().SetVisible(False)  # the J2/J3 captions name them
        h.SetPosition(pt(*pin1))
        h.SetOrientationDegrees(rot)
        board.Add(h)
        for k, pad in pads_by_number(h).items():
            pad.SetNet(nets[pinout[2 * k - 1 if parity else 2 * k]])
        headers[ref] = h
    j2, j3 = headers["J2"], headers["J3"]
    # Bus order needs J2 pins 39/40 and J3 pins 1/2 in the rightmost column.
    p2, p3 = pads_by_number(j2), pads_by_number(j3)
    assert xy(p2[40].GetPosition()) == (120.06, 74.0), xy(p2[40].GetPosition())
    assert xy(p3[2].GetPosition()) == (118.86, 123.54), xy(p3[2].GetPosition())

    for i, (x, y) in enumerate(STACK_HOLES, 1):
        add_hole(board, f"H{i}", x, y)

    for f in board.GetFootprints():
        sid = sym_ids.get(f.GetReference())
        if sid:
            f.SetPath(pcbnew.KIID_PATH("/" + sid))

    route(board, j1, j2, j3, conn_layer, other_layer)
    link_shared_pins(board, j1, conn_layer)
    add_planes(board)
    print(variant, "stitching vias:", add_stitching(board))

    # Labels: DF40 pin number and net beside every header pin, as vertical text.
    for hdr, parity, inner_y, outer_y, up in ((j2, 1, 78.3, 72.6, True),
                                              (j3, 0, 119.6, 124.9, False)):
        for k, pad in pads_by_number(hdr).items():
            df = 2 * k - 1 if parity else 2 * k
            x, _ = xy(pad.GetPosition())
            outer = k % 2 == 0
            y = outer_y if outer else inner_y
            # Text runs away from the pin: up above the header, down below it.
            away_up = up == outer
            silk_text(board, f"{df} {pinout[df]}", x, y,
                      angle=90, justify="left" if away_up else "right")

    if variant == "header":
        title = ["X1 DF40 HEADER", "BREAKOUT", "logic board on top"]
        silk_text(board, "LOGIC BOARD SIDE", 150, 104.0)
        silk_text(board, "1", 141.4, 105.5)
    else:
        title = ["X1 DF40 RECEPTACLE", "BREAKOUT", "onto a power board"]
        silk_text(board, "RECEPTACLE ON BOTTOM", 150, 104.0)
        silk_text(board, "POWER BOARD SIDE", 155.0, 116.6,
                  layer=pcbnew.B_SilkS)
        silk_text(board, "1", 141.4, 105.5, layer=pcbnew.B_SilkS)
    lines = title + ["J2 = odd pins", "J3 = even pins", "H1/H2 = stack spacers",
                     "Rev A  2026-11"]
    for n, line in enumerate(lines):
        silk_text(board, line, 131.5, 91.0 + 1.6 * n)
    return board


# ---------------------------------------------------------------- schematic

def extract_symbol(lib_path, name):
    txt = open(lib_path, encoding="utf8").read()
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
    lib = os.path.splitext(os.path.basename(lib_path))[0]
    block = block.replace(f'(symbol "{name}"', f'(symbol "{lib}:{name}"', 1)
    return block, pins


def build_schematic(variant, pinout, path):
    gen = os.path.join(STOCK_SYM, "Connector_Generic.kicad_sym")
    mech = os.path.join(STOCK_SYM, "Mechanical.kicad_sym")
    s80, p80 = extract_symbol(gen, "Conn_02x40_Odd_Even")
    s40, p40 = extract_symbol(gen, "Conn_02x20_Odd_Even")
    smh, _ = extract_symbol(mech, "MountingHole")
    project = os.path.basename(path)[:-len(".kicad_sch")]
    root = str(uuid.uuid4())
    sym_ids = {}
    out = []

    def place(lib_id, ref, value, fp, x, y, pins, pin_nets):
        sid = str(uuid.uuid4())
        sym_ids[ref] = sid
        props = [("Reference", ref, -3), ("Value", value, -1),
                 ("Footprint", fp, 1), ("Datasheet", "", 3)]
        top = y - max([p[1] for p in pins.values()] + [0]) - 4
        in_bom = "no" if lib_id.startswith("Mechanical:") else "yes"
        s = [f'\t(symbol (lib_id "{lib_id}") (at {x:.2f} {y:.2f} 0) (unit 1)',
             f'\t\t(exclude_from_sim no) (in_bom {in_bom}) (on_board yes) (dnp no)',
             f'\t\t(uuid "{sid}")']
        for k, v, dy in props:
            hide = " (hide yes)" if k in ("Footprint", "Datasheet") else ""
            s.append(f'\t\t(property "{k}" "{v}" (at {x:.2f} {top + dy:.2f} 0)'
                     f' (effects (font (size 1.27 1.27)){hide}))')
        for num in pins:
            s.append(f'\t\t(pin "{num}" (uuid "{uuid.uuid4()}"))')
        s.append(f'\t\t(instances (project "{project}"'
                 f' (path "/{root}" (reference "{ref}") (unit 1))))')
        s.append('\t)')
        out.append("\n".join(s))
        for num, net in pin_nets.items():
            px, py, ang = pins[num]
            lx, ly = x + px, y - py
            rot = 180 if ang == 0 else 0
            just = "right" if rot == 180 else "left"
            out.append(f'\t(global_label "{net}" (shape passive)'
                       f' (at {lx:.2f} {ly:.2f} {rot}) (fields_autoplaced yes)'
                       f' (effects (font (size 1.0 1.0)) (justify {just}))'
                       f' (uuid "{uuid.uuid4()}"))')

    if variant == "header":
        fp80 = "X1_Breakout:Hirose_DF40C-80DP-0.4V_2x40-1MP_P0.4mm"
        val80 = "DF40C-80DP-0.4V(51)"
    else:
        fp80 = "X1_Breakout:Hirose_DF40HC(3.0)-80DS-0.4V_2x40_P0.4mm"
        val80 = "DF40HC(3.0)-80DS-0.4V(51)"
    place("Connector_Generic:Conn_02x40_Odd_Even", "J1", val80, fp80, 78.74, 160.02,
          p80, {str(n): pinout[n] for n in range(1, 81)})
    hfp = "Connector_PinHeader_2.54mm:PinHeader_2x20_P2.54mm_Vertical"
    place("Connector_Generic:Conn_02x20_Odd_Even", "J2", "Conn_02x20 2.54mm", hfp,
          228.6, 101.6, p40, {str(k): pinout[2 * k - 1] for k in range(1, 41)})
    place("Connector_Generic:Conn_02x20_Odd_Even", "J3", "Conn_02x20 2.54mm", hfp,
          228.6, 200.66, p40, {str(k): pinout[2 * k] for k in range(1, 41)})
    for i in range(1, len(STACK_HOLES) + 1):
        place("Mechanical:MountingHole", f"H{i}", "MountingHole",
              ":".join(HOLE_FP), 325.12 + 15.24 * (i - 1), 101.6, {}, {})

    note = ("J2 pin k = DF40 pin 2k-1 (odd row); J3 pin k = DF40 pin 2k (even row). "
            "Pinout read from J_STACK1 on Common_Logic, Power_Reference and Power_P96.")
    hdr = f"""(kicad_sch
\t(version 20250114)
\t(generator "x1_df40_breakout")
\t(generator_version "10.0")
\t(uuid "{root}")
\t(paper "A3")
\t(title_block (title "X1 DF40 {variant} breakout") (date "2026-11-01") (rev "A")
\t\t(comment 1 "{note}"))
\t(lib_symbols
{s80}
{s40}
{smh}
\t)
"""
    tail = f"""
\t(text "{note}" (exclude_from_sim no) (at 40 275 0)
\t\t(effects (font (size 1.5 1.5)) (justify left bottom)) (uuid "{uuid.uuid4()}"))
\t(sheet_instances (path "/" (page "1")))
)
"""
    open(path, "w", encoding="utf8").write(hdr + "\n".join(out) + tail)
    return sym_ids


def write_project(dirname, name):
    pro = {"meta": {"filename": f"{name}.kicad_pro", "version": 3},
           "board": {"design_settings": {"rules": {
               "min_clearance": CLEARANCE, "min_track_width": TRACK,
               "min_via_diameter": VIA_D, "min_via_annular_width": 0.15,
               "min_through_hole_diameter": VIA_DRILL,
               "min_hole_to_hole": 0.25, "min_hole_clearance": 0.2,
               "min_copper_edge_clearance": 0.3,
               "min_text_height": SILK_H, "min_text_thickness": 0.15,
               "solder_mask_to_copper_clearance": 0.0,
               "min_silk_clearance": 0.0}}},
           "net_settings": {"meta": {"version": 3}, "classes": [{
               "name": "Default", "clearance": CLEARANCE, "track_width": TRACK,
               "via_diameter": VIA_D, "via_drill": VIA_DRILL,
               "microvia_diameter": 0.3, "microvia_drill": 0.1,
               "diff_pair_width": 0.2, "diff_pair_gap": 0.25,
               "wire_width": 6, "bus_width": 12, "line_style": 0,
               "schematic_color": "rgba(0, 0, 0, 0.000)",
               "pcb_color": "rgba(0, 0, 0, 0.000)", "priority": 2147483647}]},
           "sheets": [], "boards": [], "libraries": {}}
    with open(os.path.join(dirname, f"{name}.kicad_pro"), "w") as f:
        json.dump(pro, f, indent=2)
    with open(os.path.join(dirname, "fp-lib-table"), "w") as f:
        f.write('(fp_lib_table\n\t(version 7)\n'
                '\t(lib (name "X1_Breakout") (type "KiCad") '
                '(uri "${KIPRJMOD}/../X1_Breakout.pretty") '
                '(options "") (descr "DF40 land patterns copied from X1_Draft"))\n)\n')


def copy_footprints():
    os.makedirs(LOCAL_LIB, exist_ok=True)
    for name, board in DF40_FOOTPRINTS.items():
        src = os.path.join(TWO_STORY, board, "X1_Draft.pretty", name + ".kicad_mod")
        shutil.copyfile(src, os.path.join(LOCAL_LIB, name + ".kicad_mod"))


def main():
    # One board per process: pcbnew crashes when a second BOARD is built in the
    # same interpreter.
    wanted = sys.argv[1:] or ["header", "receptacle"]
    if len(wanted) > 1:
        copy_footprints()
        for v in wanted:
            subprocess.run([sys.executable, __file__, v], check=True)
        return
    pinout = read_pinout()
    with open(os.path.join(HERE, "pinout.json"), "w") as f:
        json.dump({str(k): v for k, v in sorted(pinout.items())}, f, indent=1)
    variant = wanted[0]
    name = {"header": "DF40_Header_Breakout",
            "receptacle": "DF40_Receptacle_Breakout"}[variant]
    d = os.path.join(HERE, name)
    os.makedirs(d, exist_ok=True)
    write_project(d, name)
    sym_ids = build_schematic(variant, pinout, os.path.join(d, f"{name}.kicad_sch"))
    board = build_board(variant, pinout, sym_ids)
    out = os.path.join(d, f"{name}.kicad_pcb")
    board.Save(out)
    print("wrote", out)


if __name__ == "__main__":
    main()
