"""ONiLAB 0.1-2.5 ul pipette as an OT-2 tool - parametric CadQuery model (prototype V0.3).

Frame = the gripper's "OT2-interface adapter" frame, so both tools share one interface:
  x  away from the OT-2 mount bar (mount face at x = -9.5, the 3 M3 inserts point -x)
  y  up (adapter top y = 53; GEN2 top datum estimated at y = 168, see datum_y)
  z  depth along the 37.5 mm mount bar
Printed parts: frame (mount wall with the 3 inserts, sleeve taking the pipette from below,
ledge on the body top, header window, motor shelf), yoke (on the YPF 20K-2AB-12.7 captive
shaft, presses the plunger button), tip stripper. Optional eject variant: yoke with a
spring pad and an arm that pushes the ejector button past blow-out (needs ~17 mm stroke).

    uv run --with cadquery python pipette_tool.py            # build, check, export to out/
Values marked MEASURE are estimates (product photos, ONiLAB-Dev-Core, OT-2 manual photos).
Every number lives in P: pipette_params.py turns P + stack() into the Inventor parameter table
of the native parts (build_inventor.py), so both models follow the same values.
"""
import json
import math
import os
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import cadquery as cq

import pipette_params

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"
REF = HERE / "ref"
NAME = "OT2-Pipette V0.3"


@dataclass
class P:
    # --- OT-2 (GEN2 single: 32.34 x 44.5 mm, top datum -> P20 nozzle 330.5 mm) ---
    mount_x: float = -9.5          # mount face (adapter)
    env_x1: float = 22.84          # -9.5 + 32.34: inner edge of a GEN2 pipette (neighbour 1.7 mm further)
    env_z: float = 18.75           # half depth we allow (mount bar depth 37.5)
    adapter_top: float = 53.0      # top of OT2-Gripper V1-1 - OT2-interface adapter
    datum_y: float = 168.0         # GEN2 top datum (MEASURE: photos + OT-2 reference model give 166-171)
    bar_top: float = 123.65        # top of the mount bar (OT-2 reference model, top screws at y = 46)
    p20_top_to_nozzle: float = 330.5
    shroud_drop: float = 273.3     # top datum 514.3 above deck at Z home, gantry shroud from 241
    nozzle_offset: float = -5.0    # cone end above (+) the P20 GEN2 nozzle; -5 keeps headroom above the motor
    # --- mount wall (insert pattern copied from OT2-Gripper V1-1 - OT2-interface adapter) ---
    plate_y0: float = -49.0        # bottom of the wall (bottom insert at -43)
    wall_x1: float = -2.2          # +x face of the wall (motor side face at -1.8)
    insert_d: float = 4.1          # Ruthex M3 (RX-M3x5.7)
    insert_x_end: float = -2.8
    insert_len: float = 5.7        # insert length (assembly: lead-in end inside the wall)
    insert_cb_d: float = 6.0       # shallow recess around each insert on the mount face
    insert_cb_h: float = 0.3
    inserts: tuple = ((46.0, 13.25), (46.0, -13.25), (-43.0, -13.25))   # (y, z) on the mount face
    wall_fillet: float = 4.0       # top edges of the mount wall
    cut_over: float = 1.0          # cutters run this far past the faces they open
    # --- pipette: ONiLAB = DLAB TopPette 0.1-2.5 ul; upper part ONiLAB-Dev-Core, rest product photos ---
    pip_x: float = 8.2             # axis (GEN2 nozzle at 6.5)
    pip_z: float = -3.46           # axis; ejector button points +z
    body_r: float = 12.33          # round back
    body_hw: float = 12.0          # flats
    body_len: float = 121.0        # body bottom -> top face around the plunger (Dev-Core)
    body_flat_d: float = 5.17      # flats end at this D (toward the ejector side)
    body_ch_w: float = 9.78        # chamfer corner (W, D) between flat and nose
    body_ch_d: float = 10.75
    nose_hw: float = 7.0           # nose arc: ends at W = +-7, radius 13.25 about D = 6
    nose_r: float = 13.25
    nose_c: float = 6.0
    knob_d: float = 16.6           # photo (MEASURE)
    knob_h: float = 7.0            # photo (MEASURE)
    knob_above_body: float = 18.5  # knob top above the body top face, at rest (photo, MEASURE)
    plunger_d: float = 9.9         # white plunger stem (photo)
    collar_d: float = 18.1         # blue collar under the body (photo)
    collar_len: float = 12.6
    taper_len: float = 7.2
    sleeve_d0: float = 9.4         # blue ejector sleeve, top / bottom (photo)
    sleeve_d1: float = 8.6
    sleeve_len: float = 62.6
    cone_d0: float = 4.7           # black tip cone, top / end (photo)
    cone_d1: float = 2.4
    cone_len: float = 19.1
    ej_d0: float = 12.3            # ejector button: from D (Dev-Core cut-out above 100 mm) ...
    ej_d1: float = 21.5            # ... to D (photo, MEASURE)
    ej_hw: float = 5.0             # half width (MEASURE)
    ej_top_below: float = 2.5      # button top below the body top face (photo, MEASURE)
    ej_h: float = 14.5
    disp_y: tuple = (33.0, 56.0)   # volume window above the body bottom, on the -x flat (photo)
    # --- strokes, real mm (MEASURE) ---
    pad_gap: float = 0.5           # pad above the button at the robot's 'top'
    s1: float = 5.0                # to the first stop at 2.5 ul
    s2: float = 3.0                # first stop -> blow-out stop
    eject_gap: float = 0.5         # blow-out -> eject screw touches the ejector button
    eject_stroke: float = 7.0      # ejector button travel that releases the tip
    eject_over: float = 0.5
    # --- actuator: YPF Motors 20K-2AB-12.7 captive, 1 mm lead (CAD-new BUY part) ---
    mot_sq: float = 20.0
    mot_len: float = 37.35         # closed rear: nothing sticks out at the back
    mot_pilot_d: float = 16.0
    mot_pilot_h: float = 1.5
    mot_boss_d: float = 13.0
    mot_boss_h: float = 5.6
    mot_nut_d: float = 9.0         # anti-rotation housing
    mot_nut_h: float = 14.8
    mot_holes: float = 15.4        # 4 x M2, 2.5 deep, on a 15.4 mm square
    shaft_d: float = 3.5
    thread_d: float = 2.5          # M2.5 x 5 on the shaft end (MEASURE)
    thread_len: float = 5.0
    mot_stroke: float = 12.7
    shaft_out: float = 15.8        # shaft shoulder below the front face, fully retracted
    conn_y: tuple = (29.65, 34.65) # connector housing, from the front face (on one side, here -z)
    conn_out: float = 2.8
    home_margin: float = 1.2       # robot 'top' below the retracted hard stop
    end_margin: float = 0.3        # never closer to the extended hard stop
    shelf_t: float = 4.0
    shelf_back: float = 0.5        # shelf reaches this far into the wall
    shelf_side: float = 1.0        # shelf beyond the motor square (+x) ...
    env_margin: float = 0.3        # ... but stays this far inside the GEN2 outline
    gusset_t: float = 2.5          # two gussets at the ends of the bar depth
    pilot_clr: float = 0.4         # motor pilot recess: diameter and depth clearance
    boss_clr: float = 0.8          # motor boss hole: diameter clearance
    m2_clear_d: float = 2.4        # 4 x M2 through the shelf
    driver_d: float = 4.4          # screwdriver channels for the M2 screws, through the ledge
    # --- yoke ---
    yoke_d: float = 18.0
    yoke_total: float = 28.0       # shaft shoulder -> pad face (room for the eject variant's spring pad)
    disc_h: float = 7.0
    insert_m25: float = 3.6        # Ruthex M2.5 insert for the shaft thread
    insert_m25_depth: float = 6.0
    pad_d: float = 13.0
    tab_x0: float = 0.0            # switch tab: from x = 0 to the axis, 3 mm high, toward -z
    tab_h: float = 3.0
    tab_z_in: float = 1.0          # tab ends this far inside the switch's -z end
    skirt_id: float = 14.6         # eject variant
    slot_y0: float = 1.3           # pin slots in the skirt: start above the skirt bottom
    slot_hw: float = 1.2
    slot_half_len: float = 10.0    # slots cut right through the skirt
    pin_y: float = 2.5             # pad pin axis above the skirt bottom
    slot_end: float = 2.2          # slot runs past the pin at full squeeze
    flange_d: float = 14.0
    flange_h: float = 3.0
    flange_gap: float = 1.0        # flange starts this far above the skirt bottom
    pin_d: float = 1.6             # M2 pin hole through the pad (tapped)
    nose_d: float = 12.0
    nose_out: float = 3.0
    arm_w: float = 6.0
    arm_t: float = 5.0
    arm_y: float = 4.0             # arm bar above the skirt bottom
    arm_r: float = 16.5            # eject screw axis, D from the pipette axis
    boss_d: float = 7.0
    boss_h: float = 6.0
    screw_out: float = 3.0         # M3 adjust screw below the boss
    m3_tap_d: float = 2.5
    # --- home switch (Omron D2F-L class: 12.8 x 5.8 x 6.5, M2 holes 6.5 apart), lever down ---
    sw_l: float = 12.8
    sw_w: float = 5.8
    sw_h: float = 6.5
    sw_z0: float = -16.0
    sw_trigger: float = 0.6        # yoke rise above 'top' that trips it
    sw_pitch: float = 6.5          # M2 holes
    sw_hole_down: float = 2.9      # hole axis below the switch top
    sw_pilot_d: float = 1.8        # M2 self-tapping into the wall
    lever_drop: float = 1.5        # lever below the switch body
    lever_clear: float = 0.2       # lever end above the tab at 'top'
    lever_x0: float = 1.5          # lever from the wall face, x
    lever_x1: float = 4.3
    lever_inset: float = 1.0       # lever ends inside the body ends, z
    hole_depth: float = 6.0        # self-tapping pilot holes into the wall face
    # --- clamp ---
    clr: float = 0.3
    wall: float = 3.0
    sleeve_bottom: float = 60.0    # sleeve starts this far above the body bottom (volume window stays free)
    ledge_t: float = 2.5
    knob_hole_d: float = 18.2       # knob 16.6 passes; leaves ~3 mm of ledge on the body top
    body_cut_below: float = 30.0   # body clearance cut starts this far below the body
    ej_slot_clr: float = 0.6       # ejector-button insertion slot: side clearance ...
    ej_slot_z: float = 0.5         # ... and starts this far inside the button's D
    eject_win_d: float = 8.5       # front window from this D (ejector button, arm)
    eject_win_h: float = 28.0
    eject_win_hw: float = 8.0
    disp_hw: float = 6.0           # volume window half height (z)
    grub_y: tuple = (70.0, 108.0)  # M3 grub screws through the mount-side wall (above the body bottom)
    grub_d: float = 2.5            # tapped M3
    grub_into: float = 1.0         # hole ends this far past the -x flat
    # --- identity PCB with the 2x5 header (3M 30310-6002HB class), header facing the mount ---
    hdr_y: float = 76.0            # MEASURE: where the robot's plug lands (photos: ~64-67); PCB clears the ledge
    hdr_z: float = 13.0            # MEASURE: side (+/-) to be checked on the robot
    hdr_l: float = 20.9
    hdr_w: float = 9.5
    hdr_body_l: float = 20.3       # header body (placeholder)
    hdr_body_w: float = 8.9
    hdr_in: float = 0.5            # header front inside the mount face
    pcb_t: float = 1.6
    pcb_margin: float = 6.0        # board beyond the header ends, y
    pcb_hw: float = 5.5            # board half width, z
    pcb_hole_d: float = 2.2        # 2 x M2.5 self-tapping
    pcb_hole_off: float = 3.5      # holes beyond the window ends
    # --- deck-slot tip stripper (own frame: x along the slot width, y up, z from the front edge) ---
    st_w: float = 127.76           # SBS footprint
    st_d: float = 85.48
    st_h: float = 60.0
    st_base_t: float = 3.0
    st_plate_t: float = 4.0
    st_side_x: float = 8.0         # side walls (and top plate ends) this far in from x = 0 / w
    st_side_w: float = 6.0
    st_front: float = 20.0         # walls and top plate from this z ...
    st_back: float = 4.0           # ... to this far before the back edge
    st_back_t: float = 3.0         # back wall
    st_x1: float = 40.0            # first entry hole, then st_pitch apart (3 holes)
    st_pitch: float = 24.0
    st_entry_z: float = 60.0
    st_entry_d: float = 9.5
    st_slot_len: float = 22.0
    st_slot_w: float = 4.4         # > cone at the plate, < 10 ul tip rim (MEASURE)


def body_outline(p, n=24):
    """Body section (W, D) of ONiLAB-Dev-Core at mid height; D points to the ejector side.
    Round back r 12.33, flats +-12, chamfers to the nose (arc r 13.25 about D = 6)."""
    r, hw = p.body_r, p.body_hw
    a0 = math.atan2(-math.sqrt(r * r - hw * hw), hw)          # back arc from W=+hw ...
    a1 = -math.pi - a0                                        # ... through D=-r to W=-hw
    pts = [(r * math.cos(a0 + (a1 - a0) * i / n), r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]
    nw, nd = p.nose_hw, p.nose_c + math.sqrt(p.nose_r ** 2 - p.nose_hw ** 2)
    pts += [(-hw, p.body_flat_d), (-p.body_ch_w, p.body_ch_d), (-nw, nd)]
    pts += [(-nw + 2 * nw * i / n, p.nose_c + math.sqrt(p.nose_r ** 2 - (-nw + 2 * nw * i / n) ** 2)) for i in range(1, n)]
    pts += [(nw, nd), (p.body_ch_w, p.body_ch_d), (hw, p.body_flat_d)]
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


def frustum(x, z, d0, d1, y0, y1):
    """Cone from diameter d0 at y0 to d1 at y1 (y1 > y0)."""
    return cq.Workplane().add(cq.Solid.makeCone(d0 / 2, d1 / 2, y1 - y0, cq.Vector(x, y0, z), cq.Vector(0, 1, 0)))


# ----------------------------------------------------------------------------- vertical stack
def stack(p):
    """Heights in the frame (y) and plunger travel u (real mm down from the robot's 'top')."""
    s = {}
    s["p20_nozzle"] = p.datum_y - p.p20_top_to_nozzle
    s["cone_end"] = s["p20_nozzle"] + p.nozzle_offset
    s["lower_len"] = p.collar_len + p.taper_len + p.sleeve_len + p.cone_len
    s["body_bottom"] = s["cone_end"] + s["lower_len"]
    s["body_top"] = s["body_bottom"] + p.body_len
    s["knob_top"] = s["body_top"] + p.knob_above_body
    s["ej_top"] = s["body_top"] - p.ej_top_below
    s["top"] = s["knob_top"] + p.pad_gap + p.yoke_total            # shaft shoulder at the robot's 'top'
    s["retracted"] = s["top"] + p.home_margin
    s["mot_front"] = s["retracted"] + p.shaft_out
    s["mot_rear"] = s["mot_front"] + p.mot_len
    s["headroom"] = p.datum_y - s["mot_rear"]
    s["shroud_y"] = p.datum_y - p.shroud_drop
    s["sleeve_y0"] = s["body_bottom"] + p.sleeve_bottom
    # travel budget (real mm below 'top')
    s["u_max"] = p.mot_stroke - p.home_margin - p.end_margin
    s["u_bottom"] = p.pad_gap + p.s1
    s["u_blowout"] = s["u_bottom"] + p.s2
    s["u_eject0"] = s["u_blowout"] + p.eject_gap
    s["u_drop"] = s["u_eject0"] + p.eject_stroke + p.eject_over
    s["stroke_for_eject"] = s["u_drop"] + p.home_margin + p.end_margin
    # part datums (the native Inventor parts use the same names, see pipette_params.py)
    s["shelf_x1"] = min(p.pip_x + p.mot_sq / 2 + p.shelf_side, p.env_x1 - p.env_margin)
    s["gusset_h"] = s["shelf_x1"] - p.wall_x1
    s["sw_y0"] = s["top"] + p.lever_clear + p.lever_drop        # switch body bottom
    s["pad_face"] = s["top"] - p.yoke_total                      # rigid pad face / relaxed spring pad face
    s["skirt_y0"] = s["pad_face"] + p.nose_out                   # eject variant: skirt bottom
    s["arm_y0"] = s["skirt_y0"] + p.arm_y
    s["slot_y1"] = s["skirt_y0"] + p.pin_y + p.eject_stroke + p.eject_over + p.slot_end
    s["eject_tip_y"] = s["ej_top"] + s["u_eject0"]               # adjust-screw tip at 'top'
    s["arm_boss_y"] = s["eject_tip_y"] + p.screw_out
    return s


# ----------------------------------------------------------------------------- parts
def frame(p):
    """One printed part: mount wall (3 inserts), sleeve + ledge, header window, motor shelf."""
    s = stack(p)
    outline = to_xz(p, body_outline(p))
    y_top = s["mot_rear"]
    yb, yt = s["sleeve_y0"], s["body_top"]
    o = p.cut_over
    wall = box(p.mount_x, p.wall_x1, p.plate_y0, y_top, -p.env_z, p.env_z)
    wall = wall.edges("|X and >Y").fillet(p.wall_fillet)
    sleeve = prism(outline, yb, yt - yb + p.ledge_t, p.clr + p.wall)
    # motor shelf + two gussets at the far ends of the bar depth (clear of yoke, tab, arm, switch)
    y1 = s["mot_front"]
    sx1, x0 = s["shelf_x1"], p.wall_x1 - p.shelf_back
    shelf = box(x0, sx1, y1 - p.shelf_t, y1, -p.env_z, p.env_z)
    for zz in (-p.env_z, p.env_z - p.gusset_t):
        g = (cq.Workplane("XY", origin=(0, 0, zz))
             .polyline([(x0, y1 - p.shelf_t), (sx1, y1 - p.shelf_t), (x0, y1 - p.shelf_t - s["gusset_h"])])
             .close().extrude(p.gusset_t))
        shelf = shelf.union(g)
    part = wall.union(sleeve).union(shelf)
    # pipette: body clearance (from below, through wall and sleeve), knob hole in the ledge
    part = part.cut(prism(outline, s["body_bottom"] - p.body_cut_below, yt - s["body_bottom"] + p.body_cut_below, p.clr))
    part = part.cut(cyl_y(p.pip_x, p.pip_z, p.knob_hole_d, yt - o, yt + p.ledge_t + o))
    # ejector button: insertion slot along the sleeve, window at the top (open upward for the arm)
    part = part.cut(box(p.pip_x - p.ej_hw - p.ej_slot_clr, p.pip_x + p.ej_hw + p.ej_slot_clr, yb - o, yt,
                        p.pip_z + p.ej_d0 - p.ej_slot_z, p.env_z + o))
    part = part.cut(box(p.pip_x - p.eject_win_hw, p.pip_x + p.eject_win_hw, yt - p.eject_win_h, yt + p.ledge_t + o,
                        p.pip_z + p.eject_win_d, p.env_z + o))
    # ledge relief between the two front screw channels (no loose ledge slivers between knob hole,
    # channels and window)
    h = p.mot_holes / 2
    part = part.cut(box(p.pip_x - h, p.pip_x + h, yt - o, yt + p.ledge_t + o, p.pip_z, p.pip_z + p.eject_win_d))
    # volume window through the mount wall (readable off the robot)
    part = part.cut(box(p.mount_x - o, p.pip_x - p.body_hw, s["body_bottom"] + p.disp_y[0],
                        s["body_bottom"] + p.disp_y[1], p.pip_z - p.disp_hw, p.pip_z + p.disp_hw))
    # inserts in the mount face
    for y, z in p.inserts:
        part = part.cut(cyl_x(y, z, p.insert_d, p.mount_x - o, p.insert_x_end))
        part = part.cut(cyl_x(y, z, p.insert_cb_d, p.mount_x - o, p.mount_x + p.insert_cb_h))
    # grub screws (M3, tapped 2.5) from the mount face onto the -x flat
    for yy in p.grub_y:
        part = part.cut(cyl_x(s["body_bottom"] + yy, p.pip_z, p.grub_d, p.mount_x - o, p.pip_x - p.body_hw + p.grub_into))
    # motor: pilot recess, boss hole, 4 x M2 through, screwdriver channels down through the ledge
    part = part.cut(cyl_y(p.pip_x, p.pip_z, p.mot_pilot_d + p.pilot_clr, y1 - p.mot_pilot_h - p.pilot_clr, y1 + o))
    part = part.cut(cyl_y(p.pip_x, p.pip_z, p.mot_boss_d + p.boss_clr, y1 - p.shelf_t - o, y1 + o))
    for dx in (-h, h):
        for dz in (-h, h):
            part = part.cut(cyl_y(p.pip_x + dx, p.pip_z + dz, p.m2_clear_d, y1 - p.shelf_t - o, y1 + o))
            part = part.cut(cyl_y(p.pip_x + dx, p.pip_z + dz, p.driver_d, yt - o, y1 - p.shelf_t + o))
    # identity PCB: header window through the wall, 2 x M2.5 self-tapping holes beside it
    part = part.cut(box(p.mount_x - o, p.wall_x1 + o, p.hdr_y - p.hdr_l / 2, p.hdr_y + p.hdr_l / 2,
                        p.hdr_z - p.hdr_w / 2, p.hdr_z + p.hdr_w / 2))
    for y in (p.hdr_y - p.hdr_l / 2 - p.pcb_hole_off, p.hdr_y + p.hdr_l / 2 + p.pcb_hole_off):
        part = part.cut(cyl_x(y, p.hdr_z, p.pcb_hole_d, p.wall_x1 - p.hole_depth, p.wall_x1 + o))
    # home switch seat: two M2 pilot holes in the wall face, switch hangs lever-down
    for z in switch_hole_z(p):
        part = part.cut(cyl_x(s["sw_y0"] + p.sw_h - p.sw_hole_down, z, p.sw_pilot_d, p.wall_x1 - p.hole_depth,
                              p.wall_x1 + o))
    # trim to the GEN2 envelope
    return part.intersect(box(p.mount_x, p.env_x1, p.plate_y0 - o, y_top + o, -p.env_z, p.env_z))


def switch_hole_z(p):
    z0 = p.sw_z0 + (p.sw_l - p.sw_pitch) / 2
    return z0, z0 + p.sw_pitch


def switch_y0(p):
    """Bottom of the switch body; the lever hangs 1.5 below it and ends 0.2 above the tab at 'top'."""
    return stack(p)["sw_y0"]


def yoke(p, travel=0.0, eject=False):
    """Part on the actuator's M2.5 shaft end (heat-set insert) with the switch tab.
    Rigid: a post with the pad face. Eject variant: skirt with two slots for the spring pad's
    pins, and an arm carrying an M3 adjust screw above the ejector button."""
    s = stack(p)
    yt = s["top"] - travel
    o = p.cut_over
    part = cyl_y(p.pip_x, p.pip_z, p.yoke_d, yt - p.disc_h, yt)
    if not eject:
        part = part.union(cyl_y(p.pip_x, p.pip_z, p.pad_d, s["pad_face"] - travel, yt - p.disc_h))
    else:
        sb = s["skirt_y0"] - travel                              # skirt bottom
        skirt = cyl_y(p.pip_x, p.pip_z, p.yoke_d, sb, yt - p.disc_h)
        skirt = skirt.cut(cyl_y(p.pip_x, p.pip_z, p.skirt_id, sb - o, yt - p.disc_h))
        skirt = skirt.cut(box(p.pip_x - p.slot_half_len, p.pip_x + p.slot_half_len, sb + p.slot_y0,
                              s["slot_y1"] - travel, p.pip_z - p.slot_hw, p.pip_z + p.slot_hw))
        part = part.union(skirt)
        ym = s["arm_y0"] - travel                                # arm bar
        z0, z1 = p.pip_z + p.skirt_id / 2, p.pip_z + p.arm_r + p.arm_t / 2
        part = part.union(box(p.pip_x - p.arm_w / 2, p.pip_x + p.arm_w / 2, ym, ym + p.arm_t, z0, z1))
        yb0 = s["arm_boss_y"] - travel                           # boss bottom, adjust screw sticks out below
        part = part.union(box(p.pip_x - p.arm_w / 2, p.pip_x + p.arm_w / 2, yb0, ym + p.arm_t,
                              p.pip_z + p.arm_r - p.arm_t / 2, z1))
        part = part.union(cyl_y(p.pip_x, p.pip_z + p.arm_r, p.boss_d, yb0, yb0 + p.boss_h))
        part = part.cut(cyl_y(p.pip_x, p.pip_z + p.arm_r, p.m3_tap_d, yb0 - o, ym + p.arm_t + o))   # M3 tapped
    # switch tab toward -z, under the lever
    part = part.union(box(p.tab_x0, p.pip_x, yt - p.tab_h, yt, p.sw_z0 + p.tab_z_in, p.pip_z))
    part = part.cut(cyl_y(p.pip_x, p.pip_z, p.insert_m25, yt - p.insert_m25_depth, yt + o))
    return part


def pad(p, travel=0.0, squeeze=0.0):
    """Eject variant: spring-loaded pad (flange inside the skirt, nose on the plunger button),
    retained by two M2 screws (pins) that ride in the skirt slots."""
    s = stack(p)
    yp = s["pad_face"] - travel + squeeze
    sb = s["skirt_y0"] - travel + squeeze
    part = cyl_y(p.pip_x, p.pip_z, p.nose_d, yp, sb + p.flange_gap)
    part = part.union(cyl_y(p.pip_x, p.pip_z, p.flange_d, sb + p.flange_gap, sb + p.flange_gap + p.flange_h))
    return part.cut(cyl_x(sb + p.pin_y, p.pip_z, p.pin_d, p.pip_x - p.slot_half_len, p.pip_x + p.slot_half_len))


def strip_xs(p):
    return tuple(p.st_x1 + p.st_pitch * i for i in range(3))


def stripper(p):
    """Deck-slot tip stripper (frame: x along the slot width, y up, z from the front edge = labware y).
    The tip enters a hole with its rim below the top plate, slides into the narrow slot,
    and the robot lifts: the rim catches under the plate. Tips drop into the tray below."""
    o, w, d, h = p.cut_over, p.st_w, p.st_d, p.st_h
    part = box(0, w, 0, p.st_base_t, 0, d)
    for x0 in (p.st_side_x, w - p.st_side_x - p.st_side_w):
        part = part.union(box(x0, x0 + p.st_side_w, 0, h, p.st_front, d - p.st_back))
    part = part.union(box(p.st_side_x, w - p.st_side_x, h - p.st_plate_t, h, p.st_front, d - p.st_back))
    part = part.union(box(p.st_side_x, w - p.st_side_x, 0, h, d - p.st_back - p.st_back_t, d - p.st_back))   # back wall
    for x in strip_xs(p):
        part = part.cut(cyl_y(x, p.st_entry_z, p.st_entry_d, h - p.st_plate_t - o, h + o))
        part = part.cut(box(x - p.st_slot_w / 2, x + p.st_slot_w / 2, h - p.st_plate_t - o, h + o,
                            p.st_entry_z - p.st_slot_len, p.st_entry_z))
    return part


def stripper_labware(p):
    """Opentrons labware definition (schema 2) so the robot knows the stripper's height;
    wells = the entry holes (load with ctx.load_labware_from_definition)."""
    wells = {f"A{i + 1}": {"depth": p.st_plate_t, "totalLiquidVolume": 0, "shape": "circular",
                           "diameter": p.st_entry_d, "x": x, "y": p.st_entry_z, "z": p.st_h - p.st_plate_t}
             for i, x in enumerate(strip_xs(p))}
    return {
        "ordering": [[k] for k in wells], "brand": {"brand": "openUC2", "brandId": []},
        "metadata": {"displayName": "openUC2 tip stripper (ONiLAB tool)", "displayCategory": "other",
                     "displayVolumeUnits": "µL", "tags": []},
        "dimensions": {"xDimension": p.st_w, "yDimension": p.st_d, "zDimension": p.st_h},
        "wells": wells,
        "groups": [{"metadata": {"wellBottomShape": "flat"}, "wells": list(wells)}],
        "parameters": {"format": "irregular", "quirks": [], "isTiprack": False, "isMagneticModuleCompatible": False,
                       "loadName": "openuc2_tipstripper_3_slot"},
        "namespace": "custom_beta", "version": 1, "schemaVersion": 2,
        "cornerOffsetFromSlot": {"x": 0, "y": 0, "z": 0},
    }


# ----------------------------------------------------------------------------- references
def pipette(p, push=0.0, eject=0.0):
    """ONiLAB envelope (hook cut off): body, plunger stem + knob pushed down by `push`,
    ejector button pushed down by `eject`, lower part from the photos."""
    s = stack(p)
    outline = to_xz(p, body_outline(p))
    x, z = p.pip_x, p.pip_z
    body = prism(outline, s["body_bottom"], p.body_len)
    kt = s["knob_top"] - push
    stem = cyl_y(x, z, p.plunger_d, s["body_top"], kt - p.knob_h)
    knob = cyl_y(x, z, p.knob_d, kt - p.knob_h, kt)
    y = s["body_bottom"]
    lower = cyl_y(x, z, p.collar_d, y - p.collar_len, y)
    y -= p.collar_len
    lower = lower.union(frustum(x, z, p.sleeve_d0, p.collar_d, y - p.taper_len, y))
    y -= p.taper_len
    lower = lower.union(frustum(x, z, p.sleeve_d1, p.sleeve_d0, y - p.sleeve_len, y))
    y -= p.sleeve_len
    lower = lower.union(frustum(x, z, p.cone_d1, p.cone_d0, y - p.cone_len, y))
    et = s["ej_top"] - eject
    button = box(x - p.ej_hw, x + p.ej_hw, et - p.ej_h, et, z + p.ej_d0, z + p.ej_d1)
    return body.union(stem).union(knob).union(lower).union(button)


def actuator(p, travel=0.0):
    """YPF 20K-2AB-12.7: body, pilot, boss, anti-rotation housing, shaft (+ M2.5 end), connector (-z)."""
    s = stack(p)
    y1, x, z = s["mot_front"], p.pip_x, p.pip_z
    h = p.mot_sq / 2
    mot = box(x - h, x + h, y1, y1 + p.mot_len, z - h, z + h)
    mot = mot.union(cyl_y(x, z, p.mot_pilot_d, y1 - p.mot_pilot_h, y1))
    mot = mot.union(cyl_y(x, z, p.mot_boss_d, y1 - p.mot_boss_h, y1 - p.mot_pilot_h))
    mot = mot.union(cyl_y(x, z, p.mot_nut_d, y1 - p.mot_nut_h, y1 - p.mot_boss_h))
    sh = s["top"] - travel
    mot = mot.union(cyl_y(x, z, p.shaft_d, sh, y1 - p.mot_nut_h))
    mot = mot.union(cyl_y(x, z, p.thread_d, sh - p.thread_len, sh))
    c0, c1 = p.conn_y
    mot = mot.union(box(x - 6.1, x + 6.1, y1 + c0, y1 + c1, z - h - p.conn_out, z - h))
    return mot


def switch(p):
    sy0 = switch_y0(p)
    b = box(p.wall_x1, p.wall_x1 + p.sw_w, sy0, sy0 + p.sw_h, p.sw_z0, p.sw_z0 + p.sw_l)
    lever = box(p.wall_x1 + p.lever_x0, p.wall_x1 + p.lever_x1, sy0 - p.lever_drop, sy0,
                p.sw_z0 + p.lever_inset, p.sw_z0 + p.sw_l - p.lever_inset)
    return b.union(lever)


def identity_pcb(p):
    """Perfboard on the wall face with the shrouded 2x5 header through the window (facing -x)."""
    b = box(p.wall_x1, p.wall_x1 + p.pcb_t, p.hdr_y - p.hdr_l / 2 - p.pcb_margin, p.hdr_y + p.hdr_l / 2 + p.pcb_margin,
            p.hdr_z - p.pcb_hw, min(p.hdr_z + p.pcb_hw, p.env_z))
    hdr = box(p.mount_x + p.hdr_in, p.wall_x1, p.hdr_y - p.hdr_body_l / 2, p.hdr_y + p.hdr_body_l / 2,
              p.hdr_z - p.hdr_body_w / 2, p.hdr_z + p.hdr_body_w / 2)
    return b.union(hdr)


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

    add(s["headroom"] >= 3.0, f"motor rear y={s['mot_rear']:.1f}, {s['headroom']:.1f} mm below the GEN2 top datum "
        f"y={p.datum_y} (estimate)")
    add(abs(s["cone_end"] - s["p20_nozzle"] - p.nozzle_offset) < 0.01,
        f"cone end y={s['cone_end']:.1f}, {-p.nozzle_offset:.1f} mm below the P20 GEN2 nozzle y={s['p20_nozzle']:.1f}")
    add(s["u_blowout"] + 0.2 <= s["u_max"], f"pipetting stroke: blow-out at {s['u_blowout']:.1f} mm of "
        f"{s['u_max']:.1f} mm usable ({p.mot_stroke} stroke - margins)")
    fits = s["u_drop"] <= s["u_max"]
    res.append(("ok " if fits else "info", f"plunger ejection needs {s['u_drop']:.1f} mm travel = actuator stroke "
                f">= {s['stroke_for_eject']:.1f} mm; {p.mot_stroke} mm stroke -> "
                f"{'eject by over-travel' if fits else 'use the tip stripper (or a longer-stroke actuator)'}"))
    for name in ("frame", "yoke", "yoke_eject"):
        bb = parts[name].val().BoundingBox()
        inside = bb.xmin >= p.mount_x - 0.01 and bb.xmax <= p.env_x1 + 0.01 and \
            bb.zmin >= -p.env_z - 0.01 and bb.zmax <= p.env_z + 0.01
        add(inside, f"{name}: x {bb.xmin:.2f}..{bb.xmax:.2f}, z {bb.zmin:.2f}..{bb.zmax:.2f} "
            f"within x[{p.mount_x}, {p.env_x1}] z[+-{p.env_z}]")
    act = actuator(p)
    bb = act.val().BoundingBox()
    add(bb.zmin >= -p.env_z and bb.xmax <= p.env_x1, f"actuator incl. connector housing x<= {bb.xmax:.2f}, "
        f"z>= {bb.zmin:.2f} (plug: right-angle or soldered leads)")
    fr = parts["frame"]
    add(all(len(parts[k].solids().vals()) == 1 for k in ("frame", "yoke", "yoke_eject", "pad")),
        "frame, yokes and pad are one solid each")
    add(vol(fr.intersect(pipette(p))) < 1.0, "frame clear of the pipette at rest")
    add(vol(fr.intersect(pipette(p, push=s["u_blowout"] - p.pad_gap, eject=p.eject_stroke))) < 1.0,
        "frame clear of the pipette with the button at blow-out and the ejector pressed")
    add(vol(fr.intersect(act)) < 0.5, "frame clear of the actuator")
    add(vol(fr.intersect(switch(p))) < 0.5, "frame clear of the switch")
    add(vol(fr.intersect(identity_pcb(p))) < 0.5, "frame clear of the identity board and header")
    for t in (-p.sw_trigger, 0.0, s["u_blowout"], s["u_max"]):
        for ej in (False, True):
            y = yoke(p, t, eject=ej)
            nm = "eject yoke" if ej else "yoke"
            add(vol(y.intersect(fr)) < 0.5, f"{nm} at {t:+.1f} mm clear of the frame")
            if t >= 0:
                add(vol(y.intersect(switch(p))) < 0.5, f"{nm} at {t:+.1f} mm clear of the switch")
    add(vol(yoke(p, -p.sw_trigger).intersect(switch(p))) > 0.2, "yoke tab lifts the switch lever above 'top'")
    add(vol(yoke(p, 0.0).intersect(pipette(p))) < 0.1 and vol(yoke(p, 0.0, True).intersect(pipette(p))) < 0.1,
        "yokes clear of the pipette at 'top' (pad 0.5 mm above the knob)")
    add(vol(yoke(p, s["u_blowout"], True).intersect(pipette(p, push=s["u_blowout"] - p.pad_gap))) < 0.1,
        "eject arm still clear of the ejector button at blow-out")
    for t in (0.0, s["u_max"]):
        add(vol(pad(p, t, squeeze=max(0.0, t - s["u_blowout"])).intersect(fr)) < 0.1, f"spring pad at {t:.1f} mm clear of the frame")
    # interface identical to the gripper adapter: mount face plane and insert axes
    try:
        ad = cq.importers.importStep(str(REF / "OT2-interface-adapter.stp"))
        for y, z in p.inserts:
            probe = cyl_x(y, z, 1.0, p.mount_x + 0.5, p.mount_x + 6.0)
            add(vol(ad.intersect(probe)) < 0.05 and vol(fr.intersect(probe)) < 0.05,
                f"insert axis y={y} z={z} hollow in both adapter and frame")
        bb = ad.val().BoundingBox()
        add(abs(bb.xmin - p.mount_x) < 0.01 and abs(bb.ymax - p.adapter_top) < 0.01,
            f"mount face x={bb.xmin:.2f} and adapter top y={bb.ymax:.2f} match the adapter")
    except Exception as e:
        add(False, f"adapter comparison failed: {e}")
    bb = fr.val().BoundingBox()
    add(s["shroud_y"] < bb.ymin, f"shroud line y={s['shroud_y']:.1f} below the frame (y>={bb.ymin:.1f}); "
        f"pipette above it stays inside the GEN2 outline")
    pb = pipette(p).intersect(box(-50, 50, s["shroud_y"], 400, -50, 50)).val().BoundingBox()
    add(pb.xmin >= p.mount_x and pb.xmax <= p.env_x1 and pb.zmin >= -p.env_z and pb.zmax <= p.env_z,
        f"pipette above the shroud line: x {pb.xmin:.2f}..{pb.xmax:.2f}, z {pb.zmin:.2f}..{pb.zmax:.2f}")
    return res


# ----------------------------------------------------------------------------- main
def build(p):
    return {"frame": frame(p), "yoke": yoke(p), "yoke_eject": yoke(p, eject=True), "pad": pad(p),
            "stripper": stripper(p)}


def main():
    p = P()
    OUT.mkdir(exist_ok=True)
    parts = build(p)
    s = stack(p)
    res = checks(p, parts)
    bad = pipette_params.check(p, stack)
    res.append(("ok " if not bad else "FAIL", "Inventor parameter table (pipette_params.py) reproduces stack()"
                + ("" if not bad else f": {bad}")))
    for st, msg in res:
        print(f"[{st}] {msg}")
    names = {"frame": f"{NAME} - frame", "yoke": f"{NAME} - yoke", "yoke_eject": f"{NAME} - yoke eject variant",
             "pad": f"{NAME} - spring pad", "stripper": f"{NAME} - tip stripper"}
    for k, wp in parts.items():
        cq.exporters.export(wp, str(OUT / f"{names[k]}.step"))
        cq.exporters.export(wp, str(OUT / f"{names[k]}.stl"), tolerance=0.02, angularTolerance=0.1)
    for variant in ("", " eject"):
        asm = cq.Assembly(name=NAME + variant)
        asm.add(parts["frame"], name=names["frame"], color=cq.Color(0.55, 0.7, 0.85, 1.0))
        if variant:
            asm.add(parts["yoke_eject"], name=names["yoke_eject"], color=cq.Color(0.5, 0.8, 0.55, 1.0))
            asm.add(parts["pad"], name=names["pad"], color=cq.Color(0.9, 0.75, 0.3, 1.0))
        else:
            asm.add(parts["yoke"], name=names["yoke"], color=cq.Color(0.5, 0.8, 0.55, 1.0))
        asm.add(pipette(p), name="REF - ONiLAB envelope", color=cq.Color(0.75, 0.75, 0.78, 0.6))
        asm.add(actuator(p), name="BUY - YPF Motors - linear actuator - 20K-2AB-12.7", color=cq.Color(0.3, 0.3, 0.32, 1))
        asm.add(switch(p), name="BUY - Omron - micro switch - D2F-L", color=cq.Color(0.2, 0.2, 0.2, 1))
        asm.add(identity_pcb(p), name="REF - identity board + 2x5 header", color=cq.Color(0.1, 0.45, 0.2, 1))
        asm.save(str(OUT / f"{NAME} - assembly{variant}.step"))
    (OUT / "ref").mkdir(exist_ok=True)                     # reference solids for the Inventor assembly
    for nm, wp in (("REF - ONiLAB envelope", pipette(p)), ("REF - identity board + 2x5 header", identity_pcb(p)),
                   ("BUY - Omron - micro switch - D2F-L", switch(p))):
        cq.exporters.export(wp, str(OUT / "ref" / f"{nm}.step"))
    report ={"params": asdict(p), "stack": {k: round(v, 2) for k, v in s.items()},
              "checks": [{"status": st, "msg": m} for st, m in res]}
    (OUT / f"{NAME} - build.json").write_text(json.dumps(report, indent=1))
    (OUT / "openuc2_tipstripper_3_slot.json").write_text(json.dumps(stripper_labware(p), indent=1, ensure_ascii=False),
                                                         encoding="utf-8")
    print(json.dumps(report["stack"], indent=1))
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
