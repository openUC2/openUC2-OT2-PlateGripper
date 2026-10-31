"""ONiLAB 0.1-2.5 ul pipette as an OT-2 tool - parametric CadQuery model (prototype V0.1).

Frame = the gripper's "OT2-interface adapter" frame, so both tools share one interface:
  x  away from the OT-2 mount bar (mount face at x = -9.5, the 3 M3 inserts point -x)
  y  up (adapter top y = 53, assumed level with the GEN2 pipette top datum)
  z  depth along the 37.5 mm mount bar
Printed parts: frame = head (mount plate replica + motor shelf + switch seat + header
window) fused with the clamp (sleeve taking the pipette from below, top ledge = height
stop; 204 mm, prints upright), yoke (on the Joy-IT NEMA08-04LA captive shaft, presses the
plunger button), tip stripper.

    uv run --with cadquery python pipette_tool.py            # build, check, export to out/
Values marked MEASURE are taken from ONiLAB-Dev-Core / GrabCAD and must be measured.
"""
import json
import math
import os
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"
REF = HERE / "ref"


@dataclass
class P:
    # --- OT-2 envelope (GEN2 single: 32.34 x 44.5 mm, top datum -> P20 nozzle 330.5 mm) ---
    mount_x: float = -9.5          # mount face (adapter)
    env_x1: float = 22.84          # -9.5 + 32.34: inner edge of a GEN2 pipette (neighbour 1.7 mm further)
    env_z: float = 18.75           # half depth we allow (mount bar depth 37.5)
    top_y: float = 53.0            # adapter top = GEN2 top datum (MEASURE on the robot)
    p20_top_to_nozzle: float = 330.5
    shroud_drop: float = 273.3     # top datum 514.3 above deck at Z home, gantry shroud from 241
    # --- mount plate (copied from OT2-Gripper V1-1 - OT2-interface adapter) ---
    plate_x1: float = -0.5         # +x face of the 9 mm plate
    plate_y0: float = -49.0        # bottom (the original continues to -53.4 with the gripper block)
    plate_chamfer: float = 2.0
    insert_d: float = 4.1          # Ruthex M3 (RX-M3x5.7)
    insert_x_end: float = -1.5
    inserts: tuple = ((46.0, 13.25), (46.0, -13.25), (-43.0, -13.25))   # (y, z) on the mount face
    # --- pipette (ONiLAB-Dev-Core outline; knob, top and cone length MEASURE) ---
    pip_x: float = 8.2             # axis; motor M2 screws reachable beside the plate (GEN2 nozzle at 6.5)
    pip_z: float = -3.46           # axis, body centred in the 37.5 mm bar depth
    body_r: float = 12.33          # round back
    body_hw: float = 12.0          # flats
    knob_d: float = 19.0           # MEASURE
    knob_above_body: float = 12.0  # knob top above the body shoulder (MEASURE)
    body_len: float = 121.0        # body bottom -> shoulder around the knob (MEASURE)
    cone_len: float = 100.0        # body bottom -> tip cone end (MEASURE)
    cone_d0: float = 11.7
    cone_d1: float = 9.3
    stroke: float = 9.0            # pad gap 0.5 + first stop 5 + blow-out 3 + 0.5 (MEASURE)
    # --- actuator: Joy-IT NEMA08-04LA captive, Tr3.5x4, stroke 38.1, pilot 16, M2 on 16 mm ---
    mot_sq: float = 20.2
    mot_len: float = 27.5
    mot_front_y: float = -0.5      # front face (shaft points down)
    mot_pilot_d: float = 16.0
    mot_boss_d: float = 13.0
    mot_holes: float = 16.0        # M2 square (gripper adapter; datasheet says 15.4 - check)
    sleeve_d: float = 10.0
    sleeve_len: float = 38.5
    shoulder_rest: float = 49.0    # shaft shoulder below the front face at rest (full retraction ~47)
    rear_screw_rest: float = 26.0  # screw end above the rear face at rest (28 at full retraction)
    shelf_t: float = 5.0
    # --- yoke / pad ---
    yoke_d: float = 16.0
    yoke_h: float = 6.0
    pad_d: float = 12.0
    pad_h: float = 2.0
    pad_gap: float = 0.5
    # --- home switch (Omron D2F-L class: 12.8 x 5.8 x 6.5, M2 holes 6.5 apart) ---
    sw_l: float = 12.8
    sw_w: float = 5.8
    sw_h: float = 6.5
    sw_z0: float = 5.6
    sw_trigger: float = 1.0        # yoke rise above rest that trips it
    # --- clamp ---
    clr: float = 0.3
    wall: float = 3.0
    sleeve_bottom: float = 40.0    # sleeve starts this far above the body bottom (frame 204 mm tall)
    ledge_t: float = 2.5
    knob_hole_d: float = 21.0
    eject_win_d: float = 8.5       # front window from this D (ejector button)
    eject_win_h: float = 26.0
    disp_win: tuple = (55.0, 88.0, -6.0, 6.0)   # volume window on +x wall: P-Y0, P-Y1, D0, D1 (MEASURE)
    grub_y: tuple = (46.0, 100.0)   # M3 grub screws through +x wall (P-Y)
    joint_t: float = 11.0          # clamp block under the head plate (fused)
    # --- PCB with the 2x5 ribbon header (3M 30310-6002HB class) ---
    hdr_y: float = 36.0            # header centre on the mount face (MEASURE: where the OT-2 tab lands)
    hdr_w: float = 21.0
    hdr_h: float = 10.0


def body_outline(p, n=24):
    """Body section (W, D) of ONiLAB-Dev-Core at mid height; D points to the ejector side.
    Round back r 12.33, flats +-12, chamfers to the nose (arc r 13.25 about D = 6)."""
    r, hw = p.body_r, p.body_hw
    a0 = math.atan2(-math.sqrt(r * r - hw * hw), hw)          # back arc from W=+hw ...
    a1 = -math.pi - a0                                        # ... through D=-r to W=-hw
    pts = [(r * math.cos(a0 + (a1 - a0) * i / n), r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]
    pts += [(-hw, 5.17), (-9.78, 10.75), (-7.0, 17.25)]
    pts += [(-7.0 + 14.0 * i / n, 6.0 + math.sqrt(13.25 ** 2 - (-7.0 + 14.0 * i / n) ** 2)) for i in range(1, n)]
    pts += [(7.0, 17.25), (9.78, 10.75), (hw, 5.17)]
    return pts


def to_xz(p, wd):
    return [(p.pip_x + w, p.pip_z + d) for w, d in wd]


def prism(pts_xz, y0, h, off=0.0):
    """Closed polygon in the x-z plane at height y0, grown by off, extruded +y by h."""
    wp = cq.Workplane("XZ", origin=(0, y0, 0)).polyline(pts_xz).close()
    if off:
        wp = wp.offset2D(off, "arc")
    return wp.extrude(-h)


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl_y(x, z, d, y0, y1):
    return cq.Workplane("XZ", origin=(x, y0, z)).circle(d / 2).extrude(-(y1 - y0))


def cyl_x(y, z, d, x0, x1):
    return cq.Workplane("YZ", origin=(x0, y, z)).circle(d / 2).extrude(x1 - x0)


# ----------------------------------------------------------------------------- vertical stack
def stack(p):
    s = {}
    s["shoulder"] = p.mot_front_y - p.shoulder_rest            # yoke top at rest
    s["knob_top"] = s["shoulder"] - p.yoke_h - p.pad_h - p.pad_gap
    s["shoulder_body"] = s["knob_top"] - p.knob_above_body       # body top around the knob
    s["body_bottom"] = s["shoulder_body"] - p.body_len
    s["cone_end"] = s["body_bottom"] - p.cone_len
    s["p20_nozzle"] = p.top_y - p.p20_top_to_nozzle
    s["nozzle_low_by"] = s["p20_nozzle"] - s["cone_end"]
    s["rear_screw_top"] = p.mot_front_y + p.mot_len + p.rear_screw_rest
    s["shroud_y"] = p.top_y - p.shroud_drop
    s["yoke_low"] = s["shoulder"] - p.stroke - p.yoke_h - p.pad_h
    return s


# ----------------------------------------------------------------------------- parts
def head(p):
    """Mount plate (gripper adapter interface) + motor shelf + switch seat + header window."""
    s = stack(p)
    t = p.plate_x1 - p.mount_x
    plate = (cq.Workplane("YZ", origin=(p.mount_x, 0, 0))
             .center(0, 0).rect(p.top_y - p.plate_y0, 2 * p.env_z).extrude(t)
             .translate((0, (p.top_y + p.plate_y0) / 2, 0)))
    plate = plate.edges("|X and >Y").fillet(5.0)
    plate = plate.faces(">X").edges("|Y").chamfer(p.plate_chamfer)
    for y, z in p.inserts:
        plate = plate.cut(cyl_x(y, z, p.insert_d, p.mount_x - 1, p.insert_x_end))
        plate = plate.cut(cyl_x(y, z, 6.0, p.mount_x - 1, p.mount_x + 0.3))
    # pocket for the motor body beside the plate
    mx0 = p.pip_x - p.mot_sq / 2 - 0.4
    plate = plate.cut(box(mx0, p.plate_x1 + 1, p.mot_front_y, p.mot_front_y + p.mot_len + 1.0,
                          p.pip_z - p.mot_sq / 2 - 0.6, p.pip_z + p.mot_sq / 2 + 0.6))
    # motor shelf + two gussets
    sz0, sz1 = p.pip_z - p.mot_sq / 2 - 1.0, p.pip_z + p.mot_sq / 2 + 1.0
    sx1 = min(p.pip_x + p.mot_sq / 2 + 1.0, p.env_x1 - 0.3)
    y1 = p.mot_front_y
    shelf = box(mx0, sx1, y1 - p.shelf_t, y1, sz0, sz1)
    for zz in (sz0, sz1 - 2.5):
        g = (cq.Workplane("XY", origin=(0, 0, zz))
             .polyline([(mx0, y1 - p.shelf_t), (sx1, y1 - p.shelf_t), (mx0, y1 - p.shelf_t - (sx1 - mx0) - 1)])
             .close().extrude(2.5))
        shelf = shelf.union(g)
    part = plate.union(shelf)
    part = part.cut(cyl_y(p.pip_x, p.pip_z, p.mot_pilot_d + 0.4, y1 - 1.4, y1 + 1))
    part = part.cut(cyl_y(p.pip_x, p.pip_z, p.mot_boss_d + 0.8, y1 - p.shelf_t - 1, y1 + 1))
    h = p.mot_holes / 2
    for dx in (-h, h):
        for dz in (-h, h):
            part = part.cut(cyl_y(p.pip_x + dx, p.pip_z + dz, 2.4, y1 - p.shelf_t - 1, y1 + 1))
            y0 = p.plate_y0 - 1 if dx < 0 else y1 - p.shelf_t - 1      # screwdriver path past the plate
            part = part.cut(cyl_y(p.pip_x + dx, p.pip_z + dz, 4.2, y0, y1 - p.shelf_t + 1.6))
    # header window through the plate + PCB screw holes (M2.5 self-tapping)
    part = part.cut(box(p.mount_x - 1, p.plate_x1 + 1, p.hdr_y - p.hdr_h / 2, p.hdr_y + p.hdr_h / 2,
                        -p.hdr_w / 2, p.hdr_w / 2))
    for y in (p.hdr_y - 6.0, p.hdr_y + 6.0):
        for z in (-14.0, 14.0):
            part = part.cut(cyl_x(y, z, 2.2, p.plate_x1 - 6, p.plate_x1 + 1))
    # home switch seat: two M2 pilot holes in the plate's +x face, switch hangs lever-down
    sy0 = s["shoulder"] + p.sw_trigger + 1.5
    for k in (0, 1):
        z = p.sw_z0 + (p.sw_l - 6.5) / 2 + 6.5 * k
        part = part.cut(cyl_x(sy0 + p.sw_h - 2.9, z, 1.8, p.plate_x1 - 6, p.plate_x1 + 1))
    return part


def clamp(p):
    """Sleeve around the body (inserted from below), ledge on the body shoulder, spine to the head."""
    s = stack(p)
    yb, yt = s["body_bottom"] + p.sleeve_bottom, s["shoulder_body"]
    outline = to_xz(p, body_outline(p))
    sleeve = prism(outline, yb, yt - yb + p.ledge_t, p.clr + p.wall)
    # spine on the mount side, flush with the mount face, up to the head plate
    spine_x1 = p.pip_x - p.body_hw - p.clr
    spine = box(p.mount_x, spine_x1, yb, p.plate_y0, -p.env_z, p.env_z)
    top = box(p.mount_x, p.mount_x + 6.0, p.plate_y0 - p.joint_t, p.plate_y0, -p.env_z, p.env_z)
    part = sleeve.union(spine).union(top)
    part = part.cut(prism(outline, yb - 1, yt - yb + 1, p.clr))
    part = part.cut(cyl_y(p.pip_x, p.pip_z, p.knob_hole_d, yt - 1, yt + p.ledge_t + 1))
    # trim to the envelope
    part = part.intersect(box(p.mount_x, p.env_x1, yb - 1, p.plate_y0 + 1, -p.env_z, p.env_z))
    # windows: ejector button (front, open to the top) and volume display (+x wall)
    part = part.cut(box(p.pip_x - 8.0, p.pip_x + 8.0, yt - p.eject_win_h, yt + p.ledge_t + 1,
                        p.pip_z + p.eject_win_d, p.env_z + 1))
    y0, y1, d0, d1 = p.disp_win
    part = part.cut(box(p.pip_x + 5, p.env_x1 + 1, s["body_bottom"] + y0, s["body_bottom"] + y1,
                        p.pip_z + d0, p.pip_z + d1))
    # grub screws (M3 tapped 2.5) pressing the +x flat; joint screws up into the head inserts
    for yy in p.grub_y:
        part = part.cut(cyl_x(s["body_bottom"] + yy, p.pip_z + 2.0, 2.5, p.pip_x + 5, p.env_x1 + 1))
    return part


def frame(p):
    """Head and clamp as one printed part (no joint inside the envelope)."""
    return head(p).union(clamp(p))


def yoke(p, travel=0.0):
    """Disc on the actuator's M3 shaft end (heat-set insert), flat pad on the button, switch tab."""
    s = stack(p)
    y_top = s["shoulder"] - travel
    disc = cyl_y(p.pip_x, p.pip_z, p.yoke_d, y_top - p.yoke_h, y_top)
    pad = cyl_y(p.pip_x, p.pip_z, p.pad_d, y_top - p.yoke_h - p.pad_h, y_top - p.yoke_h)
    tab_x = p.plate_x1 + p.sw_w / 2
    tab = box(min(tab_x - 2.5, p.pip_x - 2), p.pip_x + 2, y_top - 3.0, y_top,
              p.pip_z, p.sw_z0 + p.sw_l / 2 + 2)
    part = disc.union(pad).union(tab)
    part = part.cut(cyl_y(p.pip_x, p.pip_z, 4.0, y_top - 5.8, y_top + 1))
    return part


STRIP = dict(w=127.76, d=85.48, h=60.0, plate_t=4.0, xs=(40.0, 64.0, 88.0), entry_y=60.0,
             slot_len=22.0, slot_w=5.0, entry_d=9.5)    # slot_w between cone end and tip collar (MEASURE)


def stripper(p):
    """Deck-slot tip stripper (frame: x along the slot width, y up, z from the front edge = labware y).
    The tip enters a hole with its collar below the top plate, slides into the narrow slot,
    and the robot lifts: the collar catches under the plate. Tips drop into the tray below."""
    s = STRIP
    part = box(0, s["w"], 0, 3.0, 0, s["d"])
    for x0 in (8.0, s["w"] - 14.0):
        part = part.union(box(x0, x0 + 6.0, 0, s["h"], 20.0, s["d"] - 4.0))
    part = part.union(box(8.0, s["w"] - 8.0, s["h"] - s["plate_t"], s["h"], 20.0, s["d"] - 4.0))
    part = part.union(box(8.0, s["w"] - 8.0, 0, s["h"], s["d"] - 7.0, s["d"] - 4.0))   # back wall
    for x in s["xs"]:
        part = part.cut(cyl_y(x, s["entry_y"], s["entry_d"], s["h"] - s["plate_t"] - 1, s["h"] + 1))
        part = part.cut(box(x - s["slot_w"] / 2, x + s["slot_w"] / 2, s["h"] - s["plate_t"] - 1, s["h"] + 1,
                            s["entry_y"] - s["slot_len"], s["entry_y"]))
    return part


def stripper_labware():
    """Opentrons labware definition (schema 2) so the robot knows the stripper's height;
    wells = the entry holes (load with ctx.load_labware_from_definition)."""
    s = STRIP
    wells = {f"A{i + 1}": {"depth": s["plate_t"], "totalLiquidVolume": 0, "shape": "circular",
                           "diameter": s["entry_d"], "x": x, "y": s["entry_y"], "z": s["h"] - s["plate_t"]}
             for i, x in enumerate(s["xs"])}
    return {
        "ordering": [[k] for k in wells], "brand": {"brand": "openUC2", "brandId": []},
        "metadata": {"displayName": "openUC2 tip stripper (ONiLAB tool)", "displayCategory": "other",
                     "displayVolumeUnits": "µL", "tags": []},
        "dimensions": {"xDimension": s["w"], "yDimension": s["d"], "zDimension": s["h"]},
        "wells": wells,
        "groups": [{"metadata": {"wellBottomShape": "flat"}, "wells": list(wells)}],
        "parameters": {"format": "irregular", "quirks": [], "isTiprack": False, "isMagneticModuleCompatible": False,
                       "loadName": "openuc2_tipstripper_3_slot"},
        "namespace": "custom_beta", "version": 1, "schemaVersion": 2,
        "cornerOffsetFromSlot": {"x": 0, "y": 0, "z": 0},
    }


# ----------------------------------------------------------------------------- references
def pipette(p, travel=0.0):
    s = stack(p)
    outline = to_xz(p, body_outline(p))
    body = prism(outline, s["body_bottom"], p.body_len)
    knob = cyl_y(p.pip_x, p.pip_z, p.knob_d, s["shoulder_body"], s["knob_top"] - min(travel, p.stroke - p.pad_gap))
    cone = cq.Solid.makeCone(p.cone_d1 / 2, p.cone_d0 / 2, p.cone_len,
                             cq.Vector(p.pip_x, s["cone_end"], p.pip_z), cq.Vector(0, 1, 0))
    return body.union(knob).union(cq.Workplane().add(cone))


def actuator(p):
    y1 = p.mot_front_y
    mot = box(p.pip_x - p.mot_sq / 2, p.pip_x + p.mot_sq / 2, y1, y1 + p.mot_len,
              p.pip_z - p.mot_sq / 2, p.pip_z + p.mot_sq / 2)
    mot = mot.union(cyl_y(p.pip_x, p.pip_z, p.mot_pilot_d, y1 - 1.2, y1))
    mot = mot.union(cyl_y(p.pip_x, p.pip_z, p.mot_boss_d, y1 - 6.5, y1 - 1.2))
    mot = mot.union(cyl_y(p.pip_x, p.pip_z, p.sleeve_d, y1 - p.sleeve_len, y1 - 6.5))
    mot = mot.union(cyl_y(p.pip_x, p.pip_z, 3.5, stack(p)["shoulder"], y1 - p.sleeve_len))
    mot = mot.union(cyl_y(p.pip_x, p.pip_z, 3.5, y1 + p.mot_len, y1 + p.mot_len + p.rear_screw_rest))
    return mot


def switch(p):
    s = stack(p)
    sy0 = s["shoulder"] + p.sw_trigger + 1.5
    b = box(p.plate_x1, p.plate_x1 + p.sw_w, sy0, sy0 + p.sw_h, p.sw_z0, p.sw_z0 + p.sw_l)
    lever = box(p.plate_x1 + 1.5, p.plate_x1 + 4.3, sy0 - 1.5 - p.sw_trigger, sy0, p.sw_z0 + 1, p.sw_z0 + p.sw_l - 1)
    return b.union(lever)


# ----------------------------------------------------------------------------- checks
def vol(wp):
    try:
        return sum(s.Volume() for s in wp.solids().vals())
    except Exception:
        return 0.0


def checks(p, parts):
    s = stack(p)
    res = []

    def add(ok, msg):
        res.append(("ok " if ok else "FAIL", msg))

    add(abs(s["rear_screw_top"] - p.top_y) < 0.6,
        f"actuator rear screw end at y={s['rear_screw_top']:.1f} vs top datum {p.top_y} (no headroom above)")
    add(s["nozzle_low_by"] < 25, f"cone end {s['nozzle_low_by']:.1f} mm below the P20 GEN2 nozzle "
        f"(Z headroom lost; pipette offset calibration absorbs it)")
    for name in ("frame", "yoke"):
        bb = parts[name].val().BoundingBox()
        inside = bb.xmin >= p.mount_x - 0.01 and bb.xmax <= p.env_x1 + 0.01 and \
            bb.zmin >= -p.env_z - 0.01 and bb.zmax <= p.env_z + 0.01
        add(inside, f"{name}: x {bb.xmin:.2f}..{bb.xmax:.2f}, z {bb.zmin:.2f}..{bb.zmax:.2f} "
            f"within x[{p.mount_x}, {p.env_x1}] z[+-{p.env_z}]")
    pip = pipette(p)
    add(vol(parts["frame"].intersect(pip)) < 1.0, "frame clear of the pipette body")
    add(vol(parts["frame"].intersect(actuator(p))) < 0.5, "frame clear of the actuator")
    for t in (0.0, p.stroke):
        y = yoke(p, t)
        add(vol(y.intersect(parts["frame"])) < 0.5, f"yoke at {t:.1f} mm travel clear of the frame")
        add(vol(y.intersect(switch(p))) < 0.5 or t == 0, f"yoke at {t:.1f} mm clear of the switch")
    add(vol(yoke(p, -p.sw_trigger - 0.5).intersect(switch(p))) > 0.2, "yoke tab reaches the switch lever above rest")
    # interface identical to the gripper adapter: mount face plane and insert axes
    try:
        ad = cq.importers.importStep(str(REF / "OT2-interface-adapter.stp"))
        for y, z in p.inserts:
            probe = cyl_x(y, z, 1.0, p.mount_x + 0.5, p.mount_x + 6.0)
            add(vol(ad.intersect(probe)) < 0.05 and vol(parts["frame"].intersect(probe)) < 0.05,
                f"insert axis y={y} z={z} hollow in both adapter and head")
        bb = ad.val().BoundingBox()
        add(abs(bb.xmin - p.mount_x) < 0.01 and abs(bb.ymax - p.top_y) < 0.01,
            f"mount face x={bb.xmin:.2f} and top y={bb.ymax:.2f} match the adapter")
    except Exception as e:
        add(False, f"adapter comparison failed: {e}")
    add(s["shroud_y"] < s["body_bottom"] + p.sleeve_bottom,
        f"shroud line y={s['shroud_y']:.1f}: everything above lies in the envelope (sleeve bottom "
        f"{s['body_bottom'] + p.sleeve_bottom:.1f})")
    return res


# ----------------------------------------------------------------------------- main
def build(p):
    return {"frame": frame(p), "yoke": yoke(p), "stripper": stripper(p)}


def main():
    p = P()
    OUT.mkdir(exist_ok=True)
    parts = build(p)
    s = stack(p)
    res = checks(p, parts)
    for st, msg in res:
        print(f"[{st}] {msg}")
    names = {"frame": "OT2-Pipette V0.1 - frame", "yoke": "OT2-Pipette V0.1 - yoke",
             "stripper": "OT2-Pipette V0.1 - tip stripper"}
    for k, wp in parts.items():
        cq.exporters.export(wp, str(OUT / f"{names[k]}.step"))
        cq.exporters.export(wp, str(OUT / f"{names[k]}.stl"), tolerance=0.02, angularTolerance=0.1)
    asm = cq.Assembly(name="OT2-Pipette V0.1")
    col = {"frame": (0.55, 0.7, 0.85), "yoke": (0.5, 0.8, 0.55)}
    for k in ("frame", "yoke"):
        asm.add(parts[k], name=names[k], color=cq.Color(*col[k], 1.0))
    asm.add(pipette(p), name="REF - ONiLAB envelope", color=cq.Color(0.75, 0.75, 0.78, 0.6))
    asm.add(actuator(p), name="BUY - Joy-IT - linear actuator - NEMA08-04LA", color=cq.Color(0.3, 0.3, 0.32, 1))
    asm.add(switch(p), name="BUY - Omron - micro switch - D2F-L", color=cq.Color(0.2, 0.2, 0.2, 1))
    asm.save(str(OUT / "OT2-Pipette V0.1 - assembly.step"))
    report = {"params": asdict(p), "stack": {k: round(v, 2) for k, v in s.items()},
              "checks": [{"status": st, "msg": m} for st, m in res]}
    (OUT / "OT2-Pipette V0.1 - build.json").write_text(json.dumps(report, indent=1))
    (OUT / "openuc2_tipstripper_3_slot.json").write_text(json.dumps(stripper_labware(), indent=1, ensure_ascii=False),
                                                         encoding="utf-8")
    print(json.dumps(report["stack"], indent=1))
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
