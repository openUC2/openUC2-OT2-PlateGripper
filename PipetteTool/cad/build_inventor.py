"""Native, parametric Inventor model of the OT2-Pipette tool (V0.5): parts and assemblies.

    uv run --with cadquery python pipette_tool.py                                 # first: build.json
    C:\\Users\\benir\\miniforge3\\envs\\pyinventor\\python.exe build_inventor.py   # then this
    uv run --with cadquery python check_native.py                                 # STEP comparison

1. "OT2-Pipette V0.5 - parameters.ipt": every value of pipette_tool.P plus the stack as Inventor
   expressions (pipette_params.py), exported. The one place to edit the design.
2. Native parts that link it (Parameters > Link): frame, yoke, yoke eject variant, spring pad,
   tip stripper, the pipette envelope and the switch placeholder. Every sketch
   dimension, plane offset and extent is an expression over the linked names; named work
   planes / axes in the frame carry the assembly.
3. Flex test: 15 values changed in the parameter part, every part updated and checked, reverted.
4. Two assemblies (rigid yoke + tip stripper; eject variant) that link the same parameters: the
   frame is grounded, everything else is held by constraints on the frame's work features
   (actuator on the motor seat and the pipette axis, inserts on their axes, ...). Assembly
   parameter `travel` (real mm below the robot's 'top') drives the yoke and the spring pad;
   interference is checked at several travels. The CAD-new YPF part is referenced in place.
5. AP214 STEP of every native part in out/native/ for check_native.py.
Works in the open session with invisible documents; never touches documents it did not open.
"""
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pipette_params  # noqa: E402
from cadlib.inventor_helpers import (CadNew, close_if_open, find_open, import_step, matrix, safe,  # noqa: E402
                                     session, tag)
from cadlib.native_part import (NativePart, export_step, kCut, kJoin, kSym,  # noqa: E402
                                write_parameter_part)

OUT = HERE / "out"
NATIVE = OUT / "native"
REPO = HERE.parents[1]
INV = HERE.parent / "INVENTOR"
NAME = "OT2-Pipette V0.5"
YPF = "BUY - Stepping motor - 20K-2AB-12.7 - YPF Motors"
BAR_REF = REPO / "INVENTOR" / "00_OT-2 Reference_Pipettte.ipt"
ADAPTER = REPO / "INVENTOR" / "OT2-Gripper V1-1 - OT2-interface adapter.ipt"
RUTHEX = REPO / "INVENTOR" / "Threaded Insert-Ruthex-M3.ipt"
DEVCORE = REPO / "PipetteGrabCad" / "ONiLAB-Dev-Core.ipt"
I = ((1, 0, 0), (0, 1, 0), (0, 0, 1))

FILES = {
    "params": INV / f"{NAME} - parameters.ipt",
    "frame": INV / f"{NAME} - frame.ipt",
    "yoke": INV / f"{NAME} - yoke.ipt",
    "yoke eject": INV / f"{NAME} - yoke eject variant.ipt",
    "spring pad": INV / f"{NAME} - spring pad.ipt",
    "tip stripper": INV / f"{NAME} - tip stripper.ipt",
    "envelope": INV / f"REF - {NAME} - ONiLAB envelope.ipt",
    "tool board": INV / f"REF - {NAME} - tool board (KiCad).ipt",
    "switch": INV / f"BUY - Omron - micro switch - D2F-L - {NAME} placeholder.ipt",
}
CQ_STEP = {   # CadQuery export each native part is compared with (check_native.py)
    "frame": OUT / f"{NAME} - frame.step", "yoke": OUT / f"{NAME} - yoke.step",
    "yoke eject": OUT / f"{NAME} - yoke eject variant.step", "spring pad": OUT / f"{NAME} - spring pad.step",
    "tip stripper": OUT / f"{NAME} - tip stripper.step", "envelope": OUT / "ref" / "REF - ONiLAB envelope.step",
    "switch": OUT / "ref" / "BUY - Omron - micro switch - D2F-L.step",
}
DESC = {
    "frame": "printed frame: mount wall with 3 M3 inserts, sleeve + ledge, header window, motor shelf (PETG/ASA)",
    "yoke": "printed yoke on the actuator shaft (M2.5 insert), rigid pad on the plunger knob, switch tab",
    "yoke eject": "printed yoke, eject variant: skirt for the spring pad, arm with M3 adjust screw over the ejector",
    "spring pad": "printed spring pad, eject variant (2 M2 pins in the skirt slots)",
    "tip stripper": "printed deck-slot tip stripper (SBS footprint, 3 entry holes + slots)",
    "envelope": "ONiLAB 0.1-2.5 ul envelope: Dev-Core body (hook cut), photo lower part, ejector button",
    "switch": "home switch placeholder (Omron D2F-L class body + lever)",
}
COLOURS = {"frame": ["Light Steel Blue"], "tip stripper": ["Light Steel Blue"], "yoke": ["Green", "Smooth - Green"],
           "yoke eject": ["Green", "Smooth - Green"], "spring pad": ["Yellow", "Smooth - Yellow", "Orange"],
           "envelope": ["Clear", "Glass - Clear", "Glass"], "tool board": ["Green - Dark", "Dark Green", "Green"],
           "switch": ["Black", "Smooth - Black"]}
FLEX = {"datum_y": 3.0, "body_len": 2.0, "body_r": 0.2, "nose_r": 0.3, "knob_hole_d": 0.6, "wall": 0.5,
        "clr": 0.1, "mot_len": 2.0, "hdr_y": 3.0, "eject_win_d": 0.5, "yoke_d": 1.0, "arm_r": 0.5,
        "sw_z0": -0.5, "ins_z": 0.25, "st_slot_w": 0.3}


# ----------------------------------------------------------------------------- sketch helpers
def rect(part, s, u0, u1, v0, v1, tag_):
    """Axis-aligned rectangle u0..u1 x v0..v1 (expressions), size and position bound. An edge at
    0 goes collinear with the projected axis (a distance from the origin would be redundant
    where Inventor merges the projected origin into a corner)."""
    kw = {"wu": f"({u1}) - ({u0})", "wv": f"({v1}) - ({v0})"}
    zu, zv = abs(part.x(u0)) < 1e-6, abs(part.x(v0)) < 1e-6
    if not zu:
        kw["pin_u0"] = u0
    if not zv:
        kw["pin_v0"] = v0
    lines = part.rect(s, u0, v0, u1, v1, tag_, **kw)
    m = s.model_uv
    gc = s.sk.GeometricConstraints
    for zero, axis, i in ((zu, s.va, 0), (zv, s.xa, 1)):
        if zero:
            edge = next(ln for ln in lines if abs(m(ln.StartSketchPoint)[i]) < 1e-6 and abs(m(ln.EndSketchPoint)[i]) < 1e-6)
            part.constrain(lambda: gc.AddCollinear(edge, axis), f"{tag_}: edge on the axis")
    return lines


def pin_u(part, s, sp, u_expr, what):
    """Bind only the first coordinate of a sketch point (the arc's other end follows its radius)."""
    u = part.x(u_expr)
    v = s.model_uv(sp)[1]
    if abs(u) < 1e-9:
        part.constrain(lambda: s.sk.GeometricConstraints.AddCoincident(sp, s.va), f"{what} on second axis")
    else:
        part.drive(lambda: s.sk.DimensionConstraints.AddTwoPointDistance(s.o, sp, s.dim_x, s.tp(u / 2, v - 2)),
                   str(u_expr) if u > 0 else f"-({u_expr})", f"{what} u")


def _arc(part, s, c, a, b, length):
    """Arc about c from a to b with the given length (picks the sweep direction)."""
    sk = s.sk
    arc = sk.SketchArcs.AddByCenterStartEndPoint(s.pt(*c), s.pt(*a), s.pt(*b), True)
    if abs(arc.Length * 10 - length) > 0.2:
        arc.Delete()
        arc = sk.SketchArcs.AddByCenterStartEndPoint(s.pt(*c), s.pt(*a), s.pt(*b), False)
    if abs(arc.Length * 10 - length) > 0.2:
        part.warnings.append(f"{s.name}: arc length {arc.Length * 10:.2f} != {length:.2f}")
    return arc


def _end(s, arc, uv):
    return min((arc.StartSketchPoint, arc.EndSketchPoint),
               key=lambda q: (s.model_uv(q)[0] - uv[0]) ** 2 + (s.model_uv(q)[1] - uv[1]) ** 2)


DZ = "sqrt(body_r * body_r - body_hw * body_hw)"
NZ = "nose_c + sqrt(nose_r * nose_r - nose_hw * nose_hw)"
OUTLINE = {"A1": ("pip_x + body_hw", f"pip_z - {DZ}"), "A2": ("pip_x - body_hw", f"pip_z - {DZ}"),
           "B2": ("pip_x - body_hw", "pip_z + body_flat_d"), "C2": ("pip_x - body_ch_w", "pip_z + body_ch_d"),
           "N2": ("pip_x - nose_hw", f"pip_z + {NZ}"), "N1": ("pip_x + nose_hw", f"pip_z + {NZ}"),
           "C1": ("pip_x + body_ch_w", "pip_z + body_ch_d"), "B1": ("pip_x + body_hw", "pip_z + body_flat_d")}


def body_loop(part, s, grow=None):
    """ONiLAB body section about the pipette axis on a y-plane sketch: round back (arc r body_r),
    flats at +-body_hw, chamfers, nose arc (r nose_r about D = nose_c). With `grow` the loop is
    offset outward by that expression (an offset dimension) and the body outline becomes
    construction geometry, so the profile is the grown loop."""
    sk = s.sk
    P = {k: (part.x(u), part.x(v)) for k, (u, v) in OUTLINE.items()}
    ax, az = part.x("pip_x"), part.x("pip_z")
    r, hw = part.x("body_r"), part.x("body_hw")
    nr, nhw, nc = part.x("nose_r"), part.x("nose_hw"), part.x("nose_c")
    back = _arc(part, s, (ax, az), P["A1"], P["A2"], r * (math.pi - 2 * math.atan2(math.sqrt(r * r - hw * hw), hw)))
    nose = _arc(part, s, (ax, az + nc), P["N2"], P["N1"],
                nr * (math.pi - 2 * math.atan2(math.sqrt(nr * nr - nhw * nhw), nhw)))
    L = sk.SketchLines
    l1 = L.AddByTwoPoints(_end(s, back, P["A2"]), s.pt(*P["B2"]))
    l2 = L.AddByTwoPoints(l1.EndSketchPoint, s.pt(*P["C2"]))
    l3 = L.AddByTwoPoints(l2.EndSketchPoint, _end(s, nose, P["N2"]))
    l4 = L.AddByTwoPoints(_end(s, nose, P["N1"]), s.pt(*P["C1"]))
    l5 = L.AddByTwoPoints(l4.EndSketchPoint, s.pt(*P["B1"]))
    l6 = L.AddByTwoPoints(l5.EndSketchPoint, _end(s, back, P["A1"]))
    pp = part._place_point
    pp(s, back.CenterSketchPoint, "pip_x", "pip_z", "back arc centre")
    pp(s, nose.CenterSketchPoint, "pip_x", "pip_z + nose_c", "nose arc centre")
    pp(s, _end(s, back, P["A1"]), *OUTLINE["A1"], "back arc end +x")
    pin_u(part, s, _end(s, back, P["A2"]), OUTLINE["A2"][0], "back arc end -x")
    pp(s, _end(s, nose, P["N1"]), *OUTLINE["N1"], "nose arc end +x")
    pin_u(part, s, _end(s, nose, P["N2"]), OUTLINE["N2"][0], "nose arc end -x")
    for k, ln in (("B2", l1.EndSketchPoint), ("C2", l2.EndSketchPoint), ("C1", l4.EndSketchPoint),
                  ("B1", l5.EndSketchPoint)):
        pp(s, ln, *OUTLINE[k], f"outline corner {k}")
    loop = [back, l1, l2, l3, nose, l4, l5, l6]
    if grow is None:
        return loop
    g = part.x(grow)
    coll = part.app.TransientObjects.CreateObjectCollection()
    for e in loop:
        coll.Add(e)
    flat_u = P["A2"][0] - g
    copy = None
    for natural in (True, False):
        res = sk.OffsetSketchEntitiesUsingDistance(coll, g / 10, natural, False, True)
        new = [res.Item(i) for i in range(1, res.Count + 1)]
        for e in new:
            try:
                a, b = s.model_uv(e.StartSketchPoint), s.model_uv(e.EndSketchPoint)
            except Exception:
                continue
            if abs(a[0] - flat_u) < 1e-3 and abs(b[0] - flat_u) < 1e-3:
                copy = e
        if copy is not None:
            break
        for e in new:                                   # went inward: undo, other side
            safe(lambda: e.Delete())
    if copy is None:
        part.warnings.append(f"{s.name}: outline offset failed")
        return loop
    part.drive(lambda: sk.DimensionConstraints.AddOffset(copy, l1, s.tp(flat_u - 3, P["B2"][1] - 3), False, False),
               grow, f"{s.name}: outline offset")
    for e in loop:
        e.Construction = True
    return loop


def datums(part):
    """Named, hidden work features of the frame that the assemblies constrain to."""
    out = {}
    for nm, fn, expr in (("WP mount face", part.plane_x, "mount_x"), ("WP pipette x", part.plane_x, "pip_x"),
                         ("WP pipette z", part.plane_z, "pip_z"), ("WP motor seat", part.plane_y, "mot_front")):
        wp, _ = fn(expr)
        wp.Name = nm
        out[nm] = wp
    ax = part.axis_y_at("pip_x", "pip_z", "WA pipette axis")
    out["WA pipette axis"] = ax
    for k, (y, z) in enumerate((("ins_top_y", "ins_z"), ("ins_top_y", "-ins_z"), ("ins_bot_y", "-ins_z")), 1):
        out[f"WA insert {k}"] = part.axis_x_at(y, z, f"WA insert {k}")
    return out


# ----------------------------------------------------------------------------- parts
def frame(part):
    datums(part)
    o = "cut_over"
    s = part.sketch_x("mount_x", "mount wall")
    rect(part, s, "plate_y0", "mot_rear", "-env_z", "env_z", "wall")
    part.extrude_toward(s, "wall_x1 - mount_x", kJoin, "mount wall", "x", True)
    top, ez = part.x("mot_rear"), part.x("env_z")
    part.fillet(part.edges_parallel("x", 1.0, lambda x, y, z: abs(y - top) < 0.01 and abs(abs(z) - ez) < 0.01),
                "wall_fillet", "wall top edges")
    s = part.sketch_y("sleeve_y0", "sleeve outline")
    body_loop(part, s, "wall_out")
    part.extrude_toward(s, "body_top + ledge_t - sleeve_y0", kJoin, "sleeve and ledge", "y", True)
    s = part.sketch_y("mot_front - shelf_t", "motor shelf")
    rect(part, s, "wall_x1 - shelf_back", "shelf_x1", "-env_z", "env_z", "shelf")
    part.extrude_toward(s, "shelf_t", kJoin, "motor shelf", "y", True)
    tri = [("wall_x1 - shelf_back", "mot_front - shelf_t"), ("shelf_x1", "mot_front - shelf_t"),
           ("wall_x1 - shelf_back", "mot_front - shelf_t - gusset_h")]
    for nm, z0 in (("gusset -z", "-env_z"), ("gusset +z", "env_z - gusset_t")):
        s = part.sketch_z(z0, nm)
        part.polygon(s, tri, nm)
        part.extrude_toward(s, "gusset_t", kJoin, nm, "z", True)
    s = part.sketch_x(f"wall_x1 - {o}", "rib")
    rect(part, s, "relief_y0", "shelf_y0", "rib_z0", "rib_z1", "rib")
    part.extrude_toward(s, f"rib_x1 - wall_x1 + {o}", kJoin, "rib behind the top-block relief", "x", True)
    # pipette: body clearance from below through wall and sleeve, knob hole in the ledge
    s = part.sketch_y("body_bottom - body_cut_below", "body clearance outline")
    body_loop(part, s, "clr")
    part.extrude_toward(s, "body_len + body_cut_below", kCut, "body clearance", "y", True)
    s = part.sketch_y(f"body_top - {o}", "knob hole")
    part.circle(s, "pip_x", "pip_z", "knob_hole_d", "knob hole")
    part.extrude_toward(s, f"ledge_t + 2 * {o}", kCut, "knob hole", "y", True)
    s = part.sketch_y(f"sleeve_y0 - {o}", "ejector slot")
    rect(part, s, "pip_x - ej_hw - ej_slot_clr", "pip_x + ej_hw + ej_slot_clr", "pip_z + ej_d0 - ej_slot_z",
         f"env_z + {o}", "ejector slot")
    part.extrude_toward(s, f"body_top - sleeve_y0 + {o}", kCut, "ejector button slot", "y", True)
    s = part.sketch_y("body_top - eject_win_h", "eject window")
    rect(part, s, "pip_x - eject_win_hw", "pip_x + eject_win_hw", "pip_z + eject_win_d", f"env_z + {o}", "eject window")
    part.extrude_toward(s, f"eject_win_h + ledge_t + {o}", kCut, "eject window", "y", True)
    s = part.sketch_y(f"body_top - {o}", "ledge relief")
    rect(part, s, "pip_x - mot_holes / 2", "pip_x + mot_holes / 2", "pip_z", "pip_z + eject_win_d", "ledge relief")
    part.extrude_toward(s, f"ledge_t + 2 * {o}", kCut, "ledge relief", "y", True)
    s = part.sketch_y("body_bottom + disp_y0", "volume window")
    rect(part, s, f"mount_x - {o}", "pip_x - body_hw", "pip_z - disp_hw", "pip_z + disp_hw", "volume window")
    part.extrude_toward(s, "disp_y1 - disp_y0", kCut, "volume window", "y", True)
    ins = (("ins_top_y", "ins_z"), ("ins_top_y", "-ins_z"), ("ins_bot_y", "-ins_z"))
    for nm, d, depth in (("insert holes", "insert_d", f"insert_x_end - mount_x + {o}"),
                         ("insert recesses", "insert_cb_d", f"insert_cb_h + {o}")):
        s = part.sketch_x(f"mount_x - {o}", nm)
        for k, (y, z) in enumerate(ins, 1):
            part.circle(s, y, z, d, f"{nm} {k}")
        part.extrude_toward(s, depth, kCut, nm, "x", True)
    s = part.sketch_x(f"mount_x - {o}", "grub screw holes")
    for k in (1, 2):
        part.circle(s, f"body_bottom + grub_y{k}", "pip_z", "grub_d", f"grub screw {k}")
    part.extrude_toward(s, f"pip_x - body_hw + grub_into - mount_x + {o}", kCut, "M3 grub screw holes", "x", True)
    # motor seat
    s = part.sketch_y("mot_front - mot_pilot_h - pilot_clr", "motor pilot recess")
    part.circle(s, "pip_x", "pip_z", "mot_pilot_d + pilot_clr", "pilot")
    part.extrude_toward(s, f"mot_pilot_h + pilot_clr + {o}", kCut, "motor pilot recess", "y", True)
    s = part.sketch_y(f"mot_front - shelf_t - {o}", "motor boss hole")
    part.circle(s, "pip_x", "pip_z", "mot_boss_d + boss_clr", "boss")
    part.extrude_toward(s, f"shelf_t + 2 * {o}", kCut, "motor boss hole", "y", True)
    sq = [(f"pip_x {a} mot_holes / 2", f"pip_z {b} mot_holes / 2") for a in "-+" for b in "-+"]
    s = part.sketch_y(f"mot_front - shelf_t - {o}", "motor screw holes")
    for k, (x, z) in enumerate(sq, 1):
        part.circle(s, x, z, "m2_clear_d", f"M2 hole {k}")
    part.extrude_toward(s, f"shelf_t + 2 * {o}", kCut, "4 x M2 motor screw holes", "y", True)
    s = part.sketch_y(f"body_top - {o}", "screwdriver channels")
    for k, (x, z) in enumerate(sq, 1):
        part.circle(s, x, z, "driver_d", f"channel {k}")
    part.extrude_toward(s, f"mot_front - shelf_t - body_top + 2 * {o}", kCut, "screwdriver channels", "y", True)
    # robot plug: pocket for its holder, window for the board's right-angle header; top-block relief
    s = part.sketch_x(f"mount_x - {o}", "plug pocket")
    rect(part, s, "hdr_y - plug_pocket_l / 2", "hdr_y + plug_pocket_l / 2", "hdr_z - plug_pocket_w / 2",
         "hdr_z + plug_pocket_w / 2", "plug pocket")
    part.extrude_toward(s, f"plug_pocket_d + {o}", kCut, "pocket for the robot's plug holder", "x", True)
    s = part.sketch_x(f"mount_x - {o}", "header window")
    rect(part, s, "hdr_y - hdr_l / 2", "hdr_y + hdr_l / 2", "hdr_z - hdr_w / 2", "hdr_z + hdr_w / 2", "header window")
    part.extrude_toward(s, f"wall_x1 - mount_x + 2 * {o}", kCut, "2x5 header window", "x", True)
    s = part.sketch_x(f"mount_x - {o}", "top-block relief")
    rect(part, s, "relief_y0", "relief_y1", "relief_z0", f"env_z + {o}", "relief")
    part.extrude_toward(s, f"relief_d + {o}", kCut, "relief for the mount's top block", "x", True)
    # tool board: room for the soldered XIAO pins behind it, two tabs under the shelf with M2 pilot holes
    s = part.sketch_z("brd_z_front", "XIAO pin clearance")
    rect(part, s, "brd_x_ref - 22 mm", f"brd_x_ref + {o}", "brd_xiao_y0", "brd_xiao_y1", "pin clearance")
    part.extrude_toward(s, "brd_back_clr", kCut, "XIAO pin clearance", "z", True)
    s = part.sketch_z("brd_z_front", "board tabs")
    for k in (1, 2):
        rect(part, s, f"brd_hole_x{k} - brd_tab_w / 2", f"brd_hole_x{k} + brd_tab_w / 2", "brd_hole_y - brd_tab_below",
             "shelf_y0 + 0.5 mm", f"board tab {k}")
    part.extrude_toward(s, "env_z - brd_z_front", kJoin, "board tabs", "z", True)
    s = part.sketch_z(f"brd_z_front - {o}", "board screw holes")
    for k in (1, 2):
        part.circle(s, f"brd_hole_x{k}", "brd_hole_y", "brd_pilot_d", f"board hole {k}")
    part.extrude_toward(s, f"env_z - brd_z_front + 2 * {o}", kCut, "tool board M2 holes", "z", True)
    s = part.sketch_x(f"wall_x1 + {o}", "switch screw holes")
    for k, z in enumerate(("sw_z0 + (sw_l - sw_pitch) / 2", "sw_z0 + (sw_l + sw_pitch) / 2"), 1):
        part.circle(s, "sw_y0 + sw_h - sw_hole_down", z, "sw_pilot_d", f"switch hole {k}")
    part.extrude_toward(s, f"hole_depth + {o}", kCut, "switch screw holes", "x", False)
    # trim to the GEN2 outline (the sleeve sticks out at +x and +-z)
    s = part.sketch_y(f"plate_y0 - {o}", "GEN2 outline trim")
    rect(part, s, "mount_x - 50 mm", "env_x1 + 50 mm", "-env_z - 50 mm", "env_z + 50 mm", "trim outer")
    rect(part, s, "mount_x", "env_x1", "-env_z", "env_z", "GEN2 outline")
    part.extrude_through(s, kSym, "trim to the GEN2 outline")


def yoke_common_top(part):
    s = part.sketch_y("top_y - disc_h", "disc")
    part.circle(s, "pip_x", "pip_z", "yoke_d", "disc")
    part.extrude_toward(s, "disc_h", kJoin, "disc", "y", True)


def yoke_common_tab(part):
    s = part.sketch_y("top_y - tab_h", "switch tab")
    rect(part, s, "tab_x0", "pip_x", "sw_z0 + tab_z_in", "pip_z", "switch tab")
    part.extrude_toward(s, "tab_h", kJoin, "switch tab", "y", True)
    s = part.sketch_y("top_y - insert_m25_depth", "M2.5 insert hole")
    part.circle(s, "pip_x", "pip_z", "insert_m25", "insert")
    part.extrude_toward(s, "insert_m25_depth + cut_over", kCut, "M2.5 insert hole", "y", True)


def yoke(part):
    yoke_common_top(part)
    s = part.sketch_y("pad_face", "post")
    part.circle(s, "pip_x", "pip_z", "pad_d", "post")
    part.extrude_toward(s, "top_y - disc_h - pad_face", kJoin, "post with pad face", "y", True)
    yoke_common_tab(part)


def yoke_eject(part):
    yoke_common_top(part)
    s = part.sketch_y("skirt_y0", "skirt")
    part.circle(s, "pip_x", "pip_z", "yoke_d", "skirt")
    part.extrude_toward(s, "top_y - disc_h - skirt_y0", kJoin, "skirt", "y", True)
    s = part.sketch_y("skirt_y0 - cut_over", "skirt bore")
    part.circle(s, "pip_x", "pip_z", "skirt_id", "bore")
    part.extrude_toward(s, "top_y - disc_h - skirt_y0 + cut_over", kCut, "skirt bore", "y", True)
    s = part.sketch_y("skirt_y0 + slot_y0", "pin slots")
    rect(part, s, "pip_x - slot_half_len", "pip_x + slot_half_len", "pip_z - slot_hw", "pip_z + slot_hw", "pin slots")
    part.extrude_toward(s, "slot_y1 - skirt_y0 - slot_y0", kCut, "pin slots", "y", True)
    s = part.sketch_y("arm_y0", "arm")
    rect(part, s, "pip_x - arm_w / 2", "pip_x + arm_w / 2", "pip_z + skirt_id / 2", "pip_z + arm_r + arm_t / 2", "arm")
    part.extrude_toward(s, "arm_t", kJoin, "arm", "y", True)
    s = part.sketch_y("arm_boss_y", "arm drop")
    rect(part, s, "pip_x - arm_w / 2", "pip_x + arm_w / 2", "pip_z + arm_r - arm_t / 2", "pip_z + arm_r + arm_t / 2",
         "arm drop")
    part.extrude_toward(s, "arm_y0 + arm_t - arm_boss_y", kJoin, "arm drop", "y", True)
    s = part.sketch_y("arm_boss_y", "screw boss")
    part.circle(s, "pip_x", "pip_z + arm_r", "boss_d", "boss")
    part.extrude_toward(s, "boss_h", kJoin, "adjust screw boss", "y", True)
    s = part.sketch_y("arm_boss_y - cut_over", "adjust screw hole")
    part.circle(s, "pip_x", "pip_z + arm_r", "m3_tap_d", "M3 tap")
    part.extrude_toward(s, "arm_y0 + arm_t - arm_boss_y + 2 * cut_over", kCut, "M3 adjust screw hole", "y", True)
    yoke_common_tab(part)


def spring_pad(part):
    s = part.sketch_y("pad_face", "nose")
    part.circle(s, "pip_x", "pip_z", "nose_d", "nose")
    part.extrude_toward(s, "skirt_y0 + flange_gap - pad_face", kJoin, "nose", "y", True)
    s = part.sketch_y("skirt_y0 + flange_gap", "flange")
    part.circle(s, "pip_x", "pip_z", "flange_d", "flange")
    part.extrude_toward(s, "flange_h", kJoin, "flange", "y", True)
    s = part.sketch_x("pip_x - slot_half_len", "pin hole")
    part.circle(s, "skirt_y0 + pin_y", "pip_z", "pin_d", "pin")
    part.extrude_toward(s, "2 * slot_half_len", kCut, "M2 pin hole", "x", True)


def tip_stripper(part):
    s = part.sketch_y("st_base_t", "base plate")
    rect(part, s, "0 mm", "st_w", "0 mm", "st_d", "base")
    part.extrude_toward(s, "st_base_t", kJoin, "base plate", "y", False)
    for nm, x0, x1 in (("side wall 1", "st_side_x", "st_side_x + st_side_w"),
                       ("side wall 2", "st_w - st_side_x - st_side_w", "st_w - st_side_x")):
        s = part.sketch_y("st_base_t", nm)
        rect(part, s, x0, x1, "st_front", "st_d - st_back", nm)
        part.extrude_toward(s, "st_h - st_base_t", kJoin, nm, "y", True)
    s = part.sketch_y("st_h - st_plate_t", "top plate")
    rect(part, s, "st_side_x", "st_w - st_side_x", "st_front", "st_d - st_back", "top plate")
    part.extrude_toward(s, "st_plate_t", kJoin, "top plate", "y", True)
    s = part.sketch_y("st_base_t", "back wall")
    rect(part, s, "st_side_x", "st_w - st_side_x", "st_d - st_back - st_back_t", "st_d - st_back", "back wall")
    part.extrude_toward(s, "st_h - st_base_t", kJoin, "back wall", "y", True)
    xs = ("st_x1", "st_x2", "st_x3")
    s = part.sketch_y("st_h - st_plate_t - cut_over", "entry holes")
    for k, x in enumerate(xs, 1):
        part.circle(s, x, "st_entry_z", "st_entry_d", f"entry {k}")
    part.extrude_toward(s, "st_plate_t + 2 * cut_over", kCut, "entry holes", "y", True)
    s = part.sketch_y("st_h - st_plate_t - cut_over", "stripper slots")
    for k, x in enumerate(xs, 1):
        rect(part, s, f"{x} - st_slot_w / 2", f"{x} + st_slot_w / 2", "st_entry_z - st_slot_len", "st_entry_z", f"slot {k}")
    part.extrude_toward(s, "st_plate_t + 2 * cut_over", kCut, "stripper slots", "y", True)


def envelope(part):
    s = part.sketch_y("body_bottom", "body")
    body_loop(part, s)
    part.extrude_toward(s, "body_len", kJoin, "body (hook cut off)", "y", True)
    s = part.sketch_y("body_top", "plunger stem")
    part.circle(s, "pip_x", "pip_z", "plunger_d", "stem")
    part.extrude_toward(s, "knob_y0 - body_top", kJoin, "plunger stem", "y", True)
    s = part.sketch_y("knob_y0", "knob")
    part.circle(s, "pip_x", "pip_z", "knob_d", "knob")
    part.extrude_toward(s, "knob_h", kJoin, "plunger knob", "y", True)
    y1 = "cone_end + cone_len"
    y2 = f"{y1} + sleeve_len"
    prof = [("pip_x", "cone_end"), ("pip_x + cone_d1 / 2", "cone_end"), ("pip_x + cone_d0 / 2", y1),
            ("pip_x + sleeve_d1 / 2", y1), ("pip_x + sleeve_d0 / 2", y2), ("pip_x + collar_d / 2", "body_bottom - collar_len"),
            ("pip_x + collar_d / 2", "body_bottom"), ("pip_x", "body_bottom")]
    s = part.sketch_z("pip_z", "lower part section")
    part.polygon(s, prof, "lower part")
    ax = part.axis_y_at("pip_x", "pip_z", "pipette axis")
    f = part.feats.RevolveFeatures.AddFull(s.profile(), ax, kJoin)
    f.Name = "collar, ejector sleeve, tip cone"
    s = part.sketch_y("ej_top - ej_h", "ejector button")
    rect(part, s, "pip_x - ej_hw", "pip_x + ej_hw", "pip_z + ej_d0", "pip_z + ej_d1", "button")
    part.extrude_toward(s, "ej_h", kJoin, "ejector button", "y", True)


def switch(part):
    s = part.sketch_x("wall_x1", "switch body")
    rect(part, s, "sw_y0", "sw_y0 + sw_h", "sw_z0", "sw_z0 + sw_l", "body")
    part.extrude_toward(s, "sw_w", kJoin, "switch body", "x", True)
    s = part.sketch_x("wall_x1 + lever_x0", "lever")
    rect(part, s, "sw_y0 - lever_drop", "sw_y0", "sw_z0 + lever_inset", "sw_z0 + sw_l - lever_inset", "lever")
    part.extrude_toward(s, "lever_x1 - lever_x0", kJoin, "lever", "x", True)


BUILDERS = {"frame": frame, "yoke": yoke, "yoke eject": yoke_eject, "spring pad": spring_pad,
            "tip stripper": tip_stripper, "envelope": envelope, "switch": switch}


def build_parts(app, ps) -> dict:
    reps = {}
    for key, fn in BUILDERS.items():
        part = NativePart(app, ps, DESC[key], linked=FILES["params"])
        try:
            fn(part)
            part.set_appearance(COLOURS[key])
            if key == "envelope":
                part.cd.BOMStructure = 51972                   # reference
            rep = part.report()
            part.save(FILES[key], step=False)
            export_step(app, part.doc, NATIVE / f"{FILES[key].stem}.stp")
        finally:
            part.close()
        reps[key] = rep
        print(f"  {FILES[key].name}: {rep['features']} features, {rep.get('volume_mm3')} mm3, health "
              f"{rep['health'] or 'ok'}, not parametric {len(rep['not_parametric'])}, warnings {rep['warnings']}")
        for u in rep["not_parametric"]:
            print("     not parametric:", u)
    return reps


# ----------------------------------------------------------------------------- flex test
def volume(doc):
    b = doc.ComponentDefinition.SurfaceBodies
    return round(sum(b.Item(i).Volume(0.0001) for i in range(1, b.Count + 1)) * 1000, 2)


def lowest_y(doc):
    return round(doc.ComponentDefinition.RangeBox.MinPoint.Y * 10, 3)


def health(doc):
    bad = []
    for coll in (doc.ComponentDefinition.Features, doc.ComponentDefinition.Sketches):
        for f in coll:
            hs = safe(lambda: f.HealthStatus)
            if hs is not None and hs != 11778:
                bad.append(f"{safe(lambda: f.Name, '?')}: {hs}")
    return bad


def flex(app, ps) -> dict:
    """Change FLEX values in the parameter part (in memory), update every part, check health and
    that the volume changed; revert and check the volume is back."""
    master = app.Documents.Open(str(FILES["params"]), False)
    docs = {k: app.Documents.Open(str(FILES[k]), False) for k in BUILDERS}
    base = {k: volume(d) for k, d in docs.items()}
    base_y = {k: lowest_y(d) for k, d in docs.items()}
    up = master.ComponentDefinition.Parameters.UserParameters
    res = {"changed": {}, "parts": {}}
    try:
        for n, dv in FLEX.items():
            up.Item(n).Expression = f"{ps[n] + dv:.6g} mm"
            res["changed"][n] = f"{ps[n]:.6g} -> {ps[n] + dv:.6g} mm"
        master.Update()
        for k, d in docs.items():
            d.Update()
            got = d.ComponentDefinition.Parameters.Item("datum_y").Value * 10
            res["parts"][k] = {"linked datum_y": round(got, 3), "volume": volume(d), "base volume": base[k],
                               "lowest y": f"{base_y[k]} -> {lowest_y(d)}", "health": health(d)}
    finally:
        for n in FLEX:
            up.Item(n).Expression = f"{ps[n]:.6g} mm"
        master.Update()
    for k, d in docs.items():
        d.Update()
        res["parts"][k]["volume after revert"] = volume(d)
        res["parts"][k]["health after revert"] = health(d)
    for d in docs.values():
        d.Close(True)
    master.Close(True)
    return res


# ----------------------------------------------------------------------------- assemblies
def interference(app, cd, a, b):
    s1 = app.TransientObjects.CreateObjectCollection()
    s2 = app.TransientObjects.CreateObjectCollection()
    s1.Add(a)
    s2.Add(b)
    try:
        res = cd.AnalyzeInterference(s1, s2)
    except Exception as exc:
        return None, [str(exc)[:60]]
    vol, boxes = 0.0, []
    for r in res:
        vol += safe(lambda: r.Volume, 0.0)
        rb = safe(lambda: r.InterferenceBody.RangeBox)
        if rb is not None:
            boxes.append([round(v * 10, 2) for v in (rb.MinPoint.X, rb.MinPoint.Y, rb.MinPoint.Z,
                                                      rb.MaxPoint.X, rb.MaxPoint.Y, rb.MaxPoint.Z)])
    return round(vol * 1000, 3), boxes


class Asm:
    def __init__(self, app, path: Path, description: str):
        self.app, self.path = app, path
        close_if_open(app, path)
        self.doc = app.Documents.Add(12291, app.FileManager.GetTemplateFile(12291), False)
        self.cd = self.doc.ComponentDefinition
        props = self.doc.PropertySets.Item("Design Tracking Properties")
        props.Item("Part Number").Value = path.stem
        props.Item("Description").Value = description
        self.cd.Parameters.DerivedParameterTables.Add(str(FILES["params"]))
        self.up = self.cd.Parameters.UserParameters
        self.occ, self.target, self.problems = {}, {}, []

    def param(self, name, expr, comment):
        prm = self.up.AddByExpression(name, expr, "mm")
        prm.Comment = comment
        return prm

    def add(self, role, path, origin, cols, reference=False, visible=True):
        occ = self.cd.Occurrences.Add(str(path), matrix(self.app, origin, cols))
        tag(occ, role)
        if reference:
            occ.BOMStructure = 51972
        if not visible:
            occ.Visible = False
        self.occ[role], self.target[role] = occ, (origin, cols)
        return occ

    def at_target(self, role, tol=1e-3) -> bool:
        origin, cols = self.target[role]
        t = self.occ[role].Transformation
        if any(abs(getattr(t.Translation, c) * 10 - origin[i]) > tol for i, c in enumerate("XYZ")):
            return False
        for j in range(3):
            for i in range(3):
                if abs(t.Cell(i + 1, j + 1) - cols[j][i]) > 1e-6:
                    return False
        return True

    def frame_feature(self, kind, name):
        f = self.occ["frame"]
        coll = f.Definition.WorkPlanes if kind == "plane" else f.Definition.WorkAxes
        return f.CreateGeometryProxy(coll.Item(name))

    def tie(self, role, mine, other, offset="0 mm", kind="plane", verify=True):
        """Constrain the occurrence's own work plane/axis `mine` (index or name) to `other` (a
        proxy) - flush or mate by the normals at the target pose, offset expression with the sign
        that keeps the occurrence at its target (`verify=False` for a constraint that leaves it
        free to slide, e.g. an axis mate before its plane)."""
        occ = self.occ[role]
        coll = occ.Definition.WorkPlanes if kind == "plane" else occ.Definition.WorkAxes
        a = occ.CreateGeometryProxy(coll.Item(mine))
        add = self.cd.Constraints.AddMateConstraint
        if kind == "plane":
            na, nb = a.Plane.Normal, other.Plane.Normal
            if na.X * nb.X + na.Y * nb.Y + na.Z * nb.Z > 0:
                add = self.cd.Constraints.AddFlushConstraint
        c = add(a, other, 0.0)
        c.Name = f"{role}: {safe(lambda: coll.Item(mine).Name, mine)}"
        if not verify:
            c.Offset.Expression = offset
            return c
        for expr in (offset, f"-({offset})"):
            c.Offset.Expression = expr
            self.doc.Update()
            if self.at_target(role):
                return c
        self.problems.append(f"{c.Name}: constraint moves it off its pose (offset {offset})")
        return c

    def planes(self, role, offsets=("0 mm", "0 mm", "0 mm"), others=None):
        """The occurrence's YZ / XZ / XY planes onto the frame's YZ / XZ / XY (or `others`)."""
        others = others or [self.occ["frame"].CreateGeometryProxy(self.occ["frame"].Definition.WorkPlanes.Item(i))
                            for i in (1, 2, 3)]
        for i, (off, oth) in enumerate(zip(offsets, others), 1):
            self.tie(role, i, oth, off)

    def check(self, pairs, known=None, label=""):
        out = []
        for a, b in pairs:
            if a in self.occ and b in self.occ:
                v, boxes = interference(self.app, self.cd, self.occ[a], self.occ[b])
                note = (known or {}).get((a, b), "")
                out.append({"pair": f"{a} / {b}", "at": label, "mm3": v, "boxes": boxes[:3], "note": note})
                flag = "ok " if v == 0 else ("known" if note else ("n/a" if v is None else "CHECK"))
                print(f"    [{flag}] {label} {a} / {b}: {v} mm3 {note}")
        return out

    def save(self):
        if self.path.exists():
            self.path.unlink()
        self.doc.SaveAs(str(self.path), False)
        return [safe(lambda: d.FullFileName) for d in self.doc.File.ReferencedFileDescriptors if d.ReferenceMissing]


PAIRS = [("frame", "pipette envelope"), ("frame", "actuator"), ("frame", "home switch"), ("frame", "tool board"),
         ("tool board", "pipette envelope"), ("tool board", "actuator"), ("tool board", "home switch"),
         ("yoke", "tool board"), ("yoke eject", "tool board"),
         ("frame", "insert 1"), ("frame", "insert 2"), ("frame", "insert 3"), ("frame", "OT-2 mount bar (reference model)"),
         ("yoke", "actuator"), ("yoke", "pipette envelope"), ("yoke", "frame"), ("yoke", "home switch"),
         ("yoke eject", "actuator"), ("yoke eject", "pipette envelope"), ("yoke eject", "frame"),
         ("yoke eject", "home switch"), ("spring pad", "yoke eject"), ("spring pad", "pipette envelope"),
         ("actuator", "OT-2 mount bar (reference model)"), ("pipette envelope", "OT-2 mount bar (reference model)")]
MOVING = [("yoke", "frame"), ("yoke", "home switch"), ("yoke", "actuator"), ("yoke", "tool board"), ("yoke eject", "frame"),
          ("yoke eject", "home switch"), ("yoke eject", "actuator"), ("spring pad", "frame"), ("spring pad", "yoke eject")]
KNOWN = {("frame", f"insert {k}"): "heat-set insert knurl in its 4.1 mm hole (by design)" for k in (1, 2, 3)}
KNOWN[("yoke eject", "tool board")] = "eject-variant arm runs through the board's place (needs another board place)"


def build_assembly(app, ps, path, eject) -> dict:
    a = Asm(app, path, "ONiLAB 0.1-2.5 ul pipette as OT-2 tool (P20 GEN2 emulation), adapter frame; "
            "parameters linked from " + FILES["params"].name)
    try:
        a.param("travel", "1 mm", "plunger travel below the robot's 'top', real mm (0 top, u_bottom first stop, "
                "u_blowout, u_max); drives the yoke and the spring pad")
        a.param("bar_dx", "-60.5 mm", "OT-2 reference model -> adapter frame (its mount-bar screw holes on the inserts)")
        a.param("bar_dy", "-441.5 mm", "")
        a.param("bar_dz", "-83.445 mm", "")
        a.param("devcore_dz", "23.34 mm", "ONiLAB-Dev-Core axis offset in its own part frame")
        a.doc.Update()
        x, z, travel = ps.pip_x, ps.pip_z, 1.0
        fr = a.add("frame", FILES["frame"], (0, 0, 0), I)
        fr.Grounded = True
        if eject:
            a.add("yoke eject", FILES["yoke eject"], (0, -travel, 0), I)
            a.planes("yoke eject", ("0 mm", "travel", "0 mm"))
            a.add("spring pad", FILES["spring pad"], (0, -travel, 0), I)
            a.planes("spring pad", ("0 mm", "min(travel; u_blowout)", "0 mm"))
        else:
            a.add("yoke", FILES["yoke"], (0, -travel, 0), I)
            a.planes("yoke", ("0 mm", "travel", "0 mm"))
        # YPF part: axis +x (front face x = 0, shaft toward -x), connector on -y -> axis -y, connector -z
        a.add("actuator", CadNew().find(YPF + ".ipt"), (x, ps.mot_front, z), ((0, 1, 0), (0, 0, 1), (1, 0, 0)))
        a.tie("actuator", 1, a.frame_feature("plane", "WP motor seat"))
        a.tie("actuator", 2, a.frame_feature("plane", "WP pipette z"))
        a.tie("actuator", 3, a.frame_feature("plane", "WP pipette x"))
        a.add("home switch", FILES["switch"], (0, 0, 0), I)
        a.planes("home switch")
        # KiCad board, exported by pipette_tool.py already in the tool frame
        a.add("tool board", FILES["tool board"], (0, 0, 0), I)
        a.planes("tool board")
        # Ruthex part: axis +y from the lead-in (y = 0) to the flush end (y = insert_len)
        ins = ((ps.ins_top_y, ps.ins_z), (ps.ins_top_y, -ps.ins_z), (ps.ins_bot_y, -ps.ins_z))
        for k, (y, zz) in enumerate(ins, 1):
            role = f"insert {k}"
            a.add(role, RUTHEX, (ps.mount_x + ps.insert_len, y, zz), ((0, 1, 0), (-1, 0, 0), (0, 0, 1)))
            a.tie(role, 2, a.frame_feature("axis", f"WA insert {k}"), kind="axis", verify=False)
            a.tie(role, 2, a.frame_feature("plane", "WP mount face"), "insert_len")
        a.add("pipette envelope", FILES["envelope"], (0, 0, 0), I, reference=True)
        a.planes("pipette envelope")
        a.add("OT-2 mount bar (reference model)", BAR_REF, (-60.5, -441.5, -83.445), I, reference=True)
        a.planes("OT-2 mount bar (reference model)", ("bar_dx", "bar_dy", "bar_dz"))
        a.add("gripper adapter (same frame)", ADAPTER, (0, 0, 0), I, reference=True, visible=False)
        a.planes("gripper adapter (same frame)")
        # Dev-Core: axis at x -23.34 in its file, body bottom y 0, nose +x -> on the tool axis, nose +z
        a.add("ONiLAB-Dev-Core (with hook)", DEVCORE, (x, ps.body_bottom, z + 23.34), ((0, 0, 1), (0, 1, 0), (-1, 0, 0)),
              reference=True, visible=False)
        f = a.occ["frame"]
        fp = [f.CreateGeometryProxy(f.Definition.WorkPlanes.Item(i)) for i in (1, 2, 3)]
        a.tie("ONiLAB-Dev-Core (with hook)", 1, fp[2], "pip_z + devcore_dz")
        a.tie("ONiLAB-Dev-Core (with hook)", 2, fp[1], "body_bottom")
        a.tie("ONiLAB-Dev-Core (with hook)", 3, fp[0], "pip_x")
        # travel: back to 'top', check, then sweep
        a.up.Item("travel").Expression = "0 mm"
        a.doc.Update()
        yk = "yoke eject" if eject else "yoke"
        checks = a.check(PAIRS, KNOWN, "travel 0")
        sweep = []
        for expr, val in (("-sw_trigger", -ps.sw_trigger), ("u_blowout", ps.u_blowout), ("u_max", ps.u_max)):
            a.up.Item("travel").Expression = expr
            a.doc.Update()
            got = -a.occ[yk].Transformation.Translation.Y * 10
            sweep.append({"travel": expr, "value": round(val, 3), "yoke moved": round(got, 3)})
            if abs(got - val) > 1e-3:
                a.problems.append(f"travel {expr}: yoke moved {got:.3f} instead of {val:.3f}")
            if val < 0:
                v, _ = interference(app, a.cd, a.occ[yk], a.occ["home switch"])
                sweep[-1]["tab lifts the switch lever (mm3)"] = v
                print(f"    [{'ok ' if v else 'CHECK'}] travel {expr}: tab in the lever {v} mm3 (trips the switch)")
            else:
                checks += a.check(MOVING, KNOWN, f"travel {expr}")
        a.up.Item("travel").Expression = "0 mm"
        a.doc.Update()
        missing = a.save()
        n_cons = a.cd.Constraints.Count
        return {"assembly": str(path), "constraints": n_cons, "constraint problems": a.problems, "sweep": sweep,
                "interference": checks, "missing_refs": missing}
    finally:
        a.doc.Close(True)


def main():
    ps = pipette_params.from_build_json(OUT / f"{NAME} - build.json")
    for f in (BAR_REF, ADAPTER, RUTHEX, DEVCORE):
        if not f.exists():
            raise SystemExit(f"missing {f}")
    NATIVE.mkdir(parents=True, exist_ok=True)
    report = {"parameters": FILES["params"].name}
    with session() as app:
        open_ot2 = [(d.DisplayName, bool(d.Dirty)) for d in app.Documents if "OT2-Pipette" in safe(lambda: d.DisplayName, "")]
        print("OT2-Pipette documents open in the session:", open_ot2)
        for nm in ("", " - eject variant"):
            close_if_open(app, INV / f"{NAME}{nm}.iam")
        for k in FILES:
            close_if_open(app, FILES[k])
        bad = write_parameter_part(app, ps, FILES["params"],
                                   "OT2-Pipette parameters (from pipette_tool.P + stack); parts and assemblies link it")
        print(f"{FILES['params'].name}: {len(ps.items)} parameters, refused: {bad or 'none'}")
        if bad:
            raise SystemExit("fix the parameter names first")
        report["parts"] = build_parts(app, ps)
        board = import_step(app, OUT / "ref" / "REF - tool board (KiCad).step", FILES["tool board"], force=True)
        if board is None or board.suffix != ".ipt":
            raise SystemExit(f"tool board import gave {board} - expected a single part")
        doc = app.Documents.Open(str(board), False)
        try:
            props = doc.PropertySets.Item("Design Tracking Properties")
            props.Item("Part Number").Value = board.stem
            props.Item("Description").Value = ("OT2-Pipette tool board (KiCad 10, electronics/pcb), XIAO ESP32-S3 on its pins; "
                                               "imported from KiCad - edit it there")
            doc.Save()
        finally:
            doc.Close(True)
        print("flex test")
        report["flex"] = flex(app, ps)
        for k, r in report["flex"]["parts"].items():
            ok = (r["linked datum_y"] != round(ps.datum_y, 3) and not r["health"] and not r["health after revert"]
                  and abs(r["volume after revert"] - r["base volume"]) < 0.05)
            print(f"  [{'ok ' if ok else 'CHECK'}] {k}: volume {r['base volume']} -> {r['volume']} -> "
                  f"{r['volume after revert']} mm3, lowest y {r['lowest y']}, health {r['health'] or 'ok'}")
        for nm, eject in (("", False), (" - eject variant", True)):
            print(f"{NAME}{nm}.iam")
            report[nm or "main"] = build_assembly(app, ps, INV / f"{NAME}{nm}.iam", eject)
    (INV / f"{NAME} - inventor build.json").write_text(json.dumps(report, indent=1), encoding="utf-8")
    for k in ("main", " - eject variant"):
        r = report[k]
        print(k, f"{r['constraints']} constraints, problems: {r['constraint problems'] or 'none'}, "
              f"missing references: {r['missing_refs'] or 'none'}")
    sys.stdout.flush()


if __name__ == "__main__":
    main()
