"""Tool board of the OT2-Pipette: KiCad 10 board from netlist.py + layout.py, routed by route.py.

    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" build_pcb.py place     # board + pads.json
    uv run --with numpy --with scipy python route.py out/pads.json out/routes.json
    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" build_pcb.py finish    # tracks, pours, fill
    then make.py runs kicad-cli: DRC, Gerbers + drill (zip), BOM + CPL for JLCPCB, STEP, renders.

Coordinates: KiCad board mm (x right, y DOWN). layout.py places the parts; the board's origin
and orientation in the printed frame are in layout.FRAME (used by the CAD side).
"""
import json
import os
import sys
from pathlib import Path

import pcbnew

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import layout  # noqa: E402
from netlist import CLASSES, PARTS, ROUTE_FIRST  # noqa: E402

FP = Path(os.environ.get("KICAD10_FOOTPRINT_DIR", r"C:\Program Files\KiCad\10.0\share\kicad\footprints"))
OUT = HERE / "out"
BOARD = OUT / "ot2_pipette_board.kicad_pcb"
MM = pcbnew.FromMM


def vec(x, y):
    return pcbnew.VECTOR2I(MM(x), MM(y))


def load_fp(libname):
    lib, name = libname.split(":")
    return pcbnew.FootprintLoad(str(FP / f"{lib}.pretty"), name)


def xiao_fp():
    """XIAO ESP32-S3, 2 x 7 through-hole pads (pins or two 1x7 female sockets): rows 15.24 mm
    apart, pitch 2.54; pin 1 (D0) top left, 1-7 down the left row, 8 (D7) bottom right up to
    14 (5V) top right; USB at the top. 3D model: the XIAO soldered on its own pin headers."""
    fp = pcbnew.FOOTPRINT(None)
    fp.SetFPID(pcbnew.LIB_ID("OT2Pipette", "XIAO_ESP32S3_Socket_2x1x7"))
    sock = load_fp("Connector_PinSocket_2.54mm:PinSocket_1x07_P2.54mm_Vertical")
    proto = list(sock.Pads())[0]
    for k in range(14):
        x = -7.62 if k < 7 else 7.62
        y = -7.62 + 2.54 * (k if k < 7 else 13 - k)
        pad = pcbnew.PAD(proto)
        pad.SetParent(fp)
        pad.SetNumber(str(k + 1))
        pad.SetShape(pcbnew.PAD_SHAPE_RECT if k == 0 else pcbnew.PAD_SHAPE_CIRCLE)
        pad.SetFPRelativePosition(vec(x, y))
        fp.Add(pad)
    m = pcbnew.FP_3DMODEL()                         # XIAO on its own pin headers (no socket inside the tool)
    m.m_Filename = str(HERE / "3d" / "XIAO_ESP32S3_on_pins.step")
    fp.Models().push_back(m)
    for layer, w, hw, hl in ((pcbnew.F_SilkS, 0.15, 8.9, 10.5), (pcbnew.F_CrtYd, 0.05, 9.15, 12.0)):
        pts = [(-hw, -hl), (hw, -hl), (hw, hl), (-hw, hl)]
        for a, b in zip(pts, pts[1:] + pts[:1]):
            seg = pcbnew.PCB_SHAPE(fp)
            seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
            seg.SetStart(vec(*a))
            seg.SetEnd(vec(*b))
            seg.SetLayer(layer)
            seg.SetWidth(MM(w))
            fp.Add(seg)
    t = pcbnew.PCB_TEXT(fp)
    t.SetText("USB")
    t.SetFPRelativePosition(vec(0, -9.3))
    t.SetLayer(pcbnew.F_SilkS)
    fp.Add(t)
    return fp


def add_outline(board, pts, r=None):
    for a, b in zip(pts, pts[1:] + pts[:1]):
        seg = pcbnew.PCB_SHAPE(board)
        seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
        seg.SetStart(vec(*a))
        seg.SetEnd(vec(*b))
        seg.SetLayer(pcbnew.Edge_Cuts)
        seg.SetWidth(MM(0.1))
        board.Add(seg)


def place():
    OUT.mkdir(exist_ok=True)
    board = pcbnew.BOARD()
    ds = board.GetDesignSettings()
    ds.SetBoardThickness(MM(1.6))
    nets = {}

    def net(name):
        if name not in nets:
            n = pcbnew.NETINFO_ITEM(board, name)
            board.Add(n)
            nets[name] = n
        return nets[name]

    for ref, (lib, value, lcsc, padnets, populate, note) in PARTS.items():
        if ref not in layout.PLACE:
            continue
        fp = xiao_fp() if lib == "XIAO" else load_fp(lib)
        board.Add(fp)
        fp.SetReference(ref)
        fp.SetValue(value)
        x, y, rot, side = layout.PLACE[ref]
        if side == "B":
            fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
        fp.SetOrientationDegrees(rot)
        fp.SetPosition(vec(x, y))
        fp.Value().SetVisible(False)
        ref_t = fp.Reference()
        ref_t.SetTextSize(pcbnew.VECTOR2I(MM(0.7), MM(0.7)))
        ref_t.SetTextThickness(MM(0.1))
        if lcsc:
            fp.SetField("LCSC", lcsc)
            for fld in fp.GetFields():
                if fld.GetName() == "LCSC":
                    fld.SetVisible(False)
        if not populate:
            fp.SetDNP(True)
        if not populate or not lcsc:                    # DNP, or hand-fitted (the XIAO): not for JLCPCB
            fp.SetExcludedFromBOM(True)
            fp.SetExcludedFromPosFiles(True)
        for pad in fp.Pads():
            n = padnets.get(pad.GetNumber())
            if n:
                pad.SetNet(net(n))
    for k, (x, y, d) in enumerate(layout.HOLES, 1):
        fp = load_fp("MountingHole:MountingHole_2.7mm_M2.5" if d > 2.4 else "MountingHole:MountingHole_2.2mm_M2")
        board.Add(fp)
        fp.SetReference(f"H{k}")
        fp.SetPosition(vec(x, y))
    add_outline(board, layout.OUTLINE)
    for text, x, y in layout.SILK:
        t = pcbnew.PCB_TEXT(board)
        t.SetText(text)
        t.SetPosition(vec(x, y))
        t.SetLayer(pcbnew.F_SilkS)
        t.SetTextSize(pcbnew.VECTOR2I(MM(0.8), MM(0.8)))
        t.SetTextThickness(MM(0.12))
        board.Add(t)
    board.Save(str(BOARD))
    # pads for the router
    pads = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            n = pad.GetNetname() or None
            layers = []
            if pad.IsOnLayer(pcbnew.F_Cu):
                layers.append("F.Cu")
            if pad.IsOnLayer(pcbnew.B_Cu):
                layers.append("B.Cu")
            if not layers:
                continue
            shp = {pcbnew.PAD_SHAPE_CIRCLE: "circle", pcbnew.PAD_SHAPE_OVAL: "oval"}.get(pad.GetShape(), "rect")
            sz = pad.GetSize()
            pos = pad.GetPosition()
            pads.append({"ref": fp.GetReference(), "pad": pad.GetNumber(), "net": n, "x": pcbnew.ToMM(pos.x),
                         "y": pcbnew.ToMM(pos.y), "sx": pcbnew.ToMM(sz.x), "sy": pcbnew.ToMM(sz.y),
                         "rot": pad.GetOrientationDegrees(), "shape": shp, "layers": layers})
    holes = [{"x": x, "y": y, "d": d + 2.0} for x, y, d in layout.HOLES]          # + washer/screw head
    classes = {k: {"width": w, "via": v, "drill": dr} for k, (w, v, dr) in CLASSES.items()}
    spec = {"outline": layout.OUTLINE, "holes": holes, "pads": pads, "classes": classes,
            "rules": {"clearance": 0.2, "edge_clearance": 0.4}, "route_first": ROUTE_FIRST}
    (OUT / "pads.json").write_text(json.dumps(spec, indent=1))
    print("placed", len(board.GetFootprints()), "footprints;", len(pads), "pads ->", OUT / "pads.json")


def finish():
    board = pcbnew.LoadBoard(str(BOARD))
    routes = json.loads((OUT / "routes.json").read_text())
    nets = {board.FindNet(n).GetNetname(): board.FindNet(n) for n in
            {t["net"] for t in routes["tracks"]} | {v["net"] for v in routes["vias"]}}
    for t in routes["tracks"]:
        tr = pcbnew.PCB_TRACK(board)
        tr.SetStart(vec(t["x1"], t["y1"]))
        tr.SetEnd(vec(t["x2"], t["y2"]))
        tr.SetWidth(MM(t["w"]))
        tr.SetLayer(pcbnew.F_Cu if t["layer"] == "F.Cu" else pcbnew.B_Cu)
        tr.SetNet(nets[t["net"]])
        board.Add(tr)
    for v in routes["vias"]:
        via = pcbnew.PCB_VIA(board)
        via.SetPosition(vec(v["x"], v["y"]))
        via.SetWidth(MM(v["d"]))
        via.SetDrill(MM(v["drill"]))
        via.SetNet(nets[v["net"]])
        board.Add(via)
    gnd = board.FindNet("GND")
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        z = pcbnew.ZONE(board)
        z.SetLayer(layer)
        z.SetNet(gnd)
        z.SetLocalClearance(MM(0.3))
        z.SetMinThickness(MM(0.25))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
        z.SetThermalReliefGap(MM(0.3))
        z.SetThermalReliefSpokeWidth(MM(0.4))
        z.SetIsFilled(False)
        poly = z.Outline()
        poly.NewOutline()
        for x, y in layout.OUTLINE:
            poly.Append(MM(x), MM(y))
        board.Add(z)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(str(BOARD))
    print("finished:", len(routes["tracks"]), "segments,", len(routes["vias"]), "vias; failed:", routes["failed"] or "none")


if __name__ == "__main__":
    {"place": place, "finish": finish}[sys.argv[1]]()
