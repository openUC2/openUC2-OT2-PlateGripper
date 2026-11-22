"""ONiLAB 0.1-2.5 ul pipette as an OT-2 tool - parametric CadQuery model (prototype V0.6).

Frame = the gripper's "OT2-interface adapter" frame, so both tools share one interface:
  x  away from the OT-2 mount bar (mount face at x = -9.5, the 3 M3 inserts point -x)
  y  up (adapter top y = 53; GEN2 top datum estimated at y = 172, see datum_y)
  z  depth along the 37.5 mm mount bar (+z = robot front on the LEFT mount)
Printed parts: frame (mount wall with the 3 inserts, sleeve taking the pipette from below,
ledge on the body top, header window, motor shelf), pipette seat (fork under the body),
yoke (on the YPF 20K-2AB-12.7 captive shaft, presses the plunger button), tip stripper.
V0.6: the pipette is turned 180 deg about its axis, so its ejector button faces the robot rear
(-z), where a GEN2 reaches 7 mm past the mount bar. Optional eject variant (needs a 19.05 mm
stroke): yoke with a spring pad on two pins and an arm to the rear that runs on a 4 mm steel
guide rod and pushes the ejector button past blow-out; the tool board keeps the +z side.

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
PCB_STEP = HERE.parent / "electronics" / "pcb" / "out" / "ot2_pipette_board.step"
NAME = "OT2-Pipette V0.6"


@dataclass
class P:
    # --- OT-2 (GEN2 single: 32.34 x 44.5 mm, top datum -> P20 nozzle 330.5 mm) ---
    mount_x: float = -9.5          # mount face (adapter)
    env_x1: float = 22.84          # -9.5 + 32.34: inner edge of a GEN2 pipette (neighbour 1.7 mm further)
    env_z: float = 18.75           # front limit: the mount bar's front face (a GEN2 is flush with it)
    env_zr: float = 25.5           # rear limit: a GEN2 reaches z = -25.84 (44.5 deep), 7 mm past the bar
    adapter_top: float = 53.0      # top of OT2-Gripper V1-1 - OT2-interface adapter
    datum_y: float = 172.0         # GEN2 top datum: 126 +-2 above the top screws (OT-2 reference CAD + API offsets,
                                   # Jubilee photo; V0.2-V0.3 used 168) - MEASURE
    bar_top: float = 123.65        # top of the mount bar (OT-2 reference model, top screws at y = 46)
    p20_top_to_nozzle: float = 330.5
    shroud_drop: float = 273.3     # top datum 514.3 above deck at Z home, gantry shroud from 241
    nozzle_offset: float = -9.0    # cone end above (+) the P20 GEN2 nozzle (same pipette height as V0.3 with datum 168)
    # --- mount wall (insert pattern copied from OT2-Gripper V1-1 - OT2-interface adapter) ---
    wall_x1: float = -2.2          # +x face of the wall (motor side face at -1.8); bottom = body bottom
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
    pip_z: float = -3.46           # axis; ejector button points -z (V0.6: turned 180 deg; V0.2-V0.5 +z)
    body_r: float = 12.33          # round back
    body_hw: float = 12.0          # flats
    body_len: float = 121.0        # body bottom -> top face around the plunger (Dev-Core)
    body_flat_d: float = 5.17      # flats end at this D (toward the ejector side)
    body_ch_w: float = 9.78        # chamfer corner (W, D) between flat and nose
    body_ch_d: float = 10.75
    nose_hw: float = 7.0           # nose arc: ends at W = +-7, radius 13.25 about D = 6
    nose_r: float = 13.25
    nose_c: float = 6.0
    nose_top: float = 100.0        # the nose ends here above the body bottom; above it the body stops at D = ej_d0
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
    # (volume display: on the flat that now faces +x, 33-56 mm above the body bottom - readable from the side)
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
    mot_stroke_ej: float = 19.05   # eject variant: same family, 19.05 mm stroke ...
    mot_len_ej: float = 43.7       # ... body ~6.35 mm longer, same seat and shaft end (ask YPF - MEASURE)
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
    yoke_total: float = 32.0       # shaft shoulder -> pad face (eject variant: pad + its springs fit in between)
    disc_h: float = 7.0
    insert_m25: float = 3.6        # Ruthex M2.5 insert for the shaft thread
    insert_m25_depth: float = 6.0
    pad_d: float = 13.0            # rigid yoke: post down to the pad face
    tab_x0: float = 0.0            # switch tab: from x = 0 to the axis, 3 mm high, toward -z
    tab_h: float = 3.0
    tab_z_in: float = 1.0          # tab ends this far inside the switch's -z end
    # --- eject variant: spring pad on 2 pins below the disc, rear arm on a steel guide rod ---
    pad_l: float = 18.0            # spring pad plate on the knob: along x ...
    pad_w: float = 9.0             # ... along z
    pad_t: float = 5.0
    pin_r: float = 5.5             # 2 x M2 x 28 (DIN 912) at pip_x +- pin_r: head in the disc, thread in the pad
    pin_clear_d: float = 2.3
    pin_cb_d: float = 4.2          # head counterbores in the disc top (heads flush at rest)
    pin_cb_h: float = 2.2
    pad_ins_d: float = 3.2         # M2 heat-set inserts in the pad (through holes)
    spring_od: float = 6.0         # 2 springs on the pins: wire 0.6, OD 6, free ~30, ~0.55 N/mm, preload ~6 N each ...
    spring_ls: float = 10.2        # ... solid length; together their preload must exceed the blow-out force (MEASURE)
    arm_d0: float = 5.5            # rear arm block: front face this far behind the axis (pad half width + 1)
    arm_x0: float = 5.0
    arm_top: float = 5.0           # block top below the yoke top (clear of the -z gusset)
    arm_h: float = 14.0
    post_d: float = 8.0            # push post above the ejector button centre
    post_ins_d: float = 4.1        # M3 heat-set insert at the post bottom: M3 x 6 set screw, flat point ...
    post_ins_h: float = 5.7
    post_bore_d: float = 3.2       # ... set from above with a 1.5 mm hex key through the post
    screw_out: float = 3.0         # set screw tip below the post
    rod_x: float = 18.5            # guide rod: 4 mm steel (h8), beside the ejector button, behind the bar
    rod_z: float = -22.0
    rod_d: float = 4.0
    rod_len: float = 75.0          # top flush with the motor shelf top
    bush_od: float = 6.8           # printed bushing on the arm (or a pressed igus/bronze sleeve)
    bush_clr: float = 0.25
    bush_len: float = 25.0         # from the block top down; stays above the ledge at full eject travel
    col_hw: float = 3.5            # rod column on the sleeve: from rod_x - col_hw, rear to the GEN2 outline ...
    col_y0: float = 42.0
    col_y1: float = 57.5           # ... up to the ledge top, below the bushing at the eject variant's full stroke
    col_front: float = 5.0         # ... front face this far in front of the rod axis
    boss_hw: float = 4.5           # rod boss at the rear of the motor shelf
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
    eject_win_d: float = 12.0      # window over the ejector button for the push post, from this D ...
    eject_win_h: float = 28.0
    eject_win_hw: float = 5.0
    ledge_relief_d: float = 8.5    # ledge cut between the two rear motor-screw channels, to this D
    grub_y: tuple = (70.0, 108.0)  # M3 grub screws through the mount-side wall (above the body bottom)
    grub_d: float = 2.5            # tapped M3
    grub_into: float = 1.0         # hole ends this far past the -x flat
    # --- robot side of the interface, LEFT mount (OT-2 reference CAD + Opentrons photos; MEASURE) ---
    hdr_y: float = 68.5            # robot's 2x5 plug centre: 22.5 above the top screws ...
    hdr_z: float = 9.3             # ... 9.3 toward the robot front (right mount: at the rear, z +16.5 mirrored)
    hdr_l: float = 21.2            # window for the board's right-angle header body (20.36 x 8.9) + clearance
    hdr_w: float = 9.7
    plug_pocket_l: float = 29.0    # pocket in the mount face around the window, for the robot's plug holder
    plug_pocket_w: float = 16.6
    plug_pocket_d: float = 2.5
    relief_y0: float = 84.0        # top-block relief: a GEN2 is 5.5 mm recessed 38-82 mm above the top screws
    relief_y1: float = 128.0
    relief_z0: float = -5.0        # relief from here to +z; the switch screws (z <= -6) keep the full wall
    relief_d: float = 5.5
    rib_z0: float = 6.3            # rib inside the relieved wall, between the yoke disc and the board
    rib_z1: float = 13.2
    rib_x1: float = 0.0
    # --- tool board (electronics/pcb, layout.FRAME): vertical, component side toward the pipette axis ---
    brd_x_ref: float = 21.8        # tool-frame x, y of KiCad (0, 0)
    brd_y_ref: float = 112.0
    brd_z_back: float = 13.75      # component side; the right-angle header body hangs 8.9 below it
    brd_t: float = 1.6
    brd_hole_x: tuple = (17.8, 4.2)  # 2 x M2 self-tapping into tabs under the motor shelf
    brd_hole_y: float = 111.8
    brd_tab_w: float = 6.0
    brd_tab_below: float = 3.0     # tabs reach this far below the board holes
    brd_pilot_d: float = 1.8
    brd_back_clr: float = 1.8      # room behind the board for the soldered XIAO pins (cuts the +z gusset there)
    brd_xiao_y: tuple = (88.5, 109.5)  # y range of the XIAO pins behind the board
    # --- pipette seat: printed fork under the body bottom, takes plunger and eject forces ---
    seat_t: float = 5.0
    seat_hole_d: float = 18.7      # around the blue collar (18.1); the fork opens toward +x
    seat_x1: float = 20.5
    seat_z0: float = -23.0
    seat_z1: float = 16.5
    seat_screw_x: float = -5.85    # 2 x M3 x 10 up into Ruthex inserts in the wall's bottom face
    seat_screw_z: tuple = (13.5, -16.0)
    seat_screw_d: float = 3.4
    seat_ins_d: float = 4.1
    seat_ins_h: float = 5.7
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
    """(W, D) about the pipette axis -> frame (x, z). D points to the ejector, which faces -z since
    V0.6 (the body is turned 180 deg; it is symmetric in W). The list is reversed to keep the loop's
    sense after the z flip (offset2D grows a counter-clockwise loop outward)."""
    return [(p.pip_x + w, p.pip_z - d) for w, d in wd][::-1]


def zd(p, d):
    """Frame z of a point D in front of the pipette axis toward the ejector (-z)."""
    return p.pip_z - d


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


def cyl_z(x, y, d, z0, z1):
    return cq.Workplane("XY", origin=(x, y, z0)).circle(d / 2).extrude(z1 - z0)


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
    s["u_max_ej"] = p.mot_stroke_ej - p.home_margin - p.end_margin   # eject variant (19.05 mm actuator)
    s["mot_rear_ej"] = s["mot_front"] + p.mot_len_ej
    s["headroom_ej"] = p.datum_y - s["mot_rear_ej"]
    # part datums (the native Inventor parts use the same names, see pipette_params.py)
    s["shelf_x1"] = min(p.pip_x + p.mot_sq / 2 + p.shelf_side, p.env_x1 - p.env_margin)
    s["gusset_h"] = s["shelf_x1"] - p.wall_x1
    s["sw_y0"] = s["top"] + p.lever_clear + p.lever_drop        # switch body bottom
    s["pad_face"] = s["top"] - p.yoke_total                      # rigid pad face / relaxed spring pad face
    s["spring_gap"] = s["top"] - p.disc_h - s["pad_face"] - p.pad_t   # eject: disc bottom -> pad top, relaxed
    s["arm_y1"] = s["top"] - p.arm_top                           # eject: arm block top
    s["eject_tip_y"] = s["ej_top"] + s["u_eject0"]               # set-screw tip at 'top'
    s["arm_boss_y"] = s["eject_tip_y"] + p.screw_out             # push post bottom
    s["rod_y0"] = s["mot_front"] - p.rod_len                     # guide rod bottom
    return s


# ----------------------------------------------------------------------------- parts
def frame(p):
    """One printed part: mount wall (3 inserts), sleeve + ledge, header window, motor shelf."""
    s = stack(p)
    outline = to_xz(p, body_outline(p))
    y_top = s["mot_rear"]
    yb, yt = s["sleeve_y0"], s["body_top"]
    o = p.cut_over
    wall = box(p.mount_x, p.wall_x1, s["body_bottom"], y_top, -p.env_z, p.env_z)
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
    # guide rod for the eject variant: column on the sleeve (bottom end), boss at the rear of the shelf (top end)
    col = box(p.rod_x - p.col_hw, p.env_x1 - p.env_margin, p.col_y0, p.col_y1, -p.env_zr - o, p.rod_z + p.col_front)
    boss = box(p.rod_x - p.boss_hw, p.env_x1 - p.env_margin, y1 - p.shelf_t, y1, -p.env_zr - o, -p.env_z + o)
    # rib on the inner side of the relieved wall, between the yoke disc and the board
    rib = box(p.wall_x1 - o, p.rib_x1, p.relief_y0, y1 - p.shelf_t, p.rib_z0, p.rib_z1)
    part = wall.union(sleeve).union(shelf).union(col).union(boss).union(rib)
    # pipette: body clearance (from below, through wall and sleeve), knob hole in the ledge
    part = part.cut(prism(outline, s["body_bottom"] - p.body_cut_below, yt - s["body_bottom"] + p.body_cut_below, p.clr))
    part = part.cut(cyl_y(p.pip_x, p.pip_z, p.knob_hole_d, yt - o, yt + p.ledge_t + o))
    # ejector button (rear): insertion slot along the sleeve, window over it for the push post
    part = part.cut(box(p.pip_x - p.ej_hw - p.ej_slot_clr, p.pip_x + p.ej_hw + p.ej_slot_clr, yb - o, yt,
                        -p.env_zr - o, zd(p, p.ej_d0 - p.ej_slot_z)))
    part = part.cut(box(p.pip_x - p.eject_win_hw, p.pip_x + p.eject_win_hw, yt - p.eject_win_h, yt + p.ledge_t + o,
                        -p.env_zr - o, zd(p, p.eject_win_d)))
    # ledge relief between the two rear screw channels (no loose ledge slivers between knob hole,
    # channels and window)
    h = p.mot_holes / 2
    part = part.cut(box(p.pip_x - h, p.pip_x + h, yt - o, yt + p.ledge_t + o, zd(p, p.ledge_relief_d), p.pip_z))
    # guide rod hole: column, then up through the shelf boss
    part = part.cut(cyl_y(p.rod_x, p.rod_z, p.rod_d, s["rod_y0"] - 0.5, y1 + o))
    # inserts for the pipette seat in the wall's bottom face
    for z in p.seat_screw_z:
        part = part.cut(cyl_y(p.seat_screw_x, z, p.seat_ins_d, s["body_bottom"] - o, s["body_bottom"] + p.seat_ins_h))
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
    # robot plug: pocket in the mount face for its holder, window for the board's right-angle header
    part = part.cut(box(p.mount_x - o, p.mount_x + p.plug_pocket_d, p.hdr_y - p.plug_pocket_l / 2,
                        p.hdr_y + p.plug_pocket_l / 2, p.hdr_z - p.plug_pocket_w / 2, p.hdr_z + p.plug_pocket_w / 2))
    part = part.cut(box(p.mount_x - o, p.wall_x1 + o, p.hdr_y - p.hdr_l / 2, p.hdr_y + p.hdr_l / 2,
                        p.hdr_z - p.hdr_w / 2, p.hdr_z + p.hdr_w / 2))
    # the mount's top block (plug holder, round part) sits in a relief, as on a GEN2
    part = part.cut(box(p.mount_x - o, p.mount_x + p.relief_d, p.relief_y0, p.relief_y1, p.relief_z0, p.env_z + o))
    # behind the board: room for the soldered XIAO pins (in the +z gusset)
    part = part.cut(box(p.brd_x_ref - 22.0, p.brd_x_ref + o, p.brd_xiao_y[0], p.brd_xiao_y[1],
                        p.brd_z_back + p.brd_t, p.brd_z_back + p.brd_t + p.brd_back_clr))
    # board tabs under the motor shelf, M2 pilot holes from the board side
    y_sh = s["mot_front"] - p.shelf_t
    for x in p.brd_hole_x:
        part = part.union(box(x - p.brd_tab_w / 2, x + p.brd_tab_w / 2, p.brd_hole_y - p.brd_tab_below, y_sh + 0.5,
                              p.brd_z_back + p.brd_t, p.env_z))
        part = part.cut(cyl_z(x, p.brd_hole_y, p.brd_pilot_d, p.brd_z_back + p.brd_t - o, p.env_z + o))
    # home switch seat: two M2 pilot holes in the wall face, switch hangs lever-down
    for z in switch_hole_z(p):
        part = part.cut(cyl_x(s["sw_y0"] + p.sw_h - p.sw_hole_down, z, p.sw_pilot_d, p.wall_x1 - p.hole_depth,
                              p.wall_x1 + o))
    # trim to the GEN2 envelope (+x: the neighbouring pipette, -z: a GEN2's rear face)
    return part.intersect(box(p.mount_x, p.env_x1, s["body_bottom"] - o, y_top + o, -p.env_zr, p.env_z))


def switch_hole_z(p):
    z0 = p.sw_z0 + (p.sw_l - p.sw_pitch) / 2
    return z0, z0 + p.sw_pitch


def switch_y0(p):
    """Bottom of the switch body; the lever hangs 1.5 below it and ends 0.2 above the tab at 'top'."""
    return stack(p)["sw_y0"]


def post_z(p):
    """Push post axis: over the middle of the ejector button."""
    return zd(p, (p.ej_d0 + p.ej_d1) / 2)


def yoke(p, travel=0.0, eject=False):
    """Part on the actuator's M2.5 shaft end (heat-set insert) with the switch tab.
    Rigid: a post with the pad face. Eject variant: the disc carries the spring pad on two M2
    pins (springs between disc and pad), and a block to the rear with a bushing on the guide rod
    and a push post (M3 set screw) over the ejector button. The rod takes the tilt from the
    off-axis eject force, so the actuator's shaft only sees axial load."""
    s = stack(p)
    yt = s["top"] - travel
    o = p.cut_over
    part = cyl_y(p.pip_x, p.pip_z, p.yoke_d, yt - p.disc_h, yt)
    if not eject:
        part = part.union(cyl_y(p.pip_x, p.pip_z, p.pad_d, s["pad_face"] - travel, yt - p.disc_h))
    else:
        ya1 = s["arm_y1"] - travel                               # arm block top
        ya0 = ya1 - p.arm_h
        part = part.union(box(p.arm_x0, p.rod_x, ya0, ya1, p.rod_z - p.bush_od / 2, zd(p, p.arm_d0)))
        part = part.union(cyl_y(p.rod_x, p.rod_z, p.bush_od, ya1 - p.bush_len, ya1))
        pz, yb = post_z(p), s["arm_boss_y"] - travel             # post bottom; the set screw sticks out below
        part = part.union(cyl_y(p.pip_x, pz, p.post_d, yb, ya0 + o))
        part = part.cut(cyl_y(p.rod_x, p.rod_z, p.rod_d + 2 * p.bush_clr, ya1 - p.bush_len - o, ya1 + o))
        part = part.cut(cyl_y(p.pip_x, pz, p.post_ins_d, yb - o, yb + p.post_ins_h))
        part = part.cut(cyl_y(p.pip_x, pz, p.post_bore_d, yb, ya1 + o))
    # switch tab toward -z, under the lever
    part = part.union(box(p.tab_x0, p.pip_x, yt - p.tab_h, yt, p.sw_z0 + p.tab_z_in, p.pip_z))
    part = part.cut(cyl_y(p.pip_x, p.pip_z, p.insert_m25, yt - p.insert_m25_depth, yt + o))
    if eject:
        for dx in (-p.pin_r, p.pin_r):                           # pins: slide in the disc, heads flush on top
            part = part.cut(cyl_y(p.pip_x + dx, p.pip_z, p.pin_clear_d, yt - p.disc_h - o, yt + o))
            part = part.cut(cyl_y(p.pip_x + dx, p.pip_z, p.pin_cb_d, yt - p.pin_cb_h, yt + o))
    return part


def pad(p, travel=0.0, squeeze=0.0):
    """Eject variant: spring pad on the plunger knob, hangs on two M2 pins (heat-set inserts);
    two springs between it and the disc hold it down until past blow-out."""
    s = stack(p)
    yp = s["pad_face"] - travel + squeeze
    part = box(p.pip_x - p.pad_l / 2, p.pip_x + p.pad_l / 2, yp, yp + p.pad_t, p.pip_z - p.pad_w / 2, p.pip_z + p.pad_w / 2)
    for dx in (-p.pin_r, p.pin_r):
        part = part.cut(cyl_y(p.pip_x + dx, p.pip_z, p.pad_ins_d, yp - p.cut_over, yp + p.pad_t + p.cut_over))
    return part


def seat(p):
    """Pipette seat: fork under the body bottom (around the collar, open toward +x), screwed up
    into the wall's bottom face. Slid in after the pipette; it takes the plunger and eject forces
    that two grub screws held before."""
    s = stack(p)
    o, yb = p.cut_over, s["body_bottom"]
    part = box(p.mount_x, p.seat_x1, yb - p.seat_t, yb, p.seat_z0, p.seat_z1)
    part = part.cut(cyl_y(p.pip_x, p.pip_z, p.seat_hole_d, yb - p.seat_t - o, yb + o))
    part = part.cut(box(p.pip_x, p.seat_x1 + o, yb - p.seat_t - o, yb + o, p.pip_z - p.seat_hole_d / 2,
                        p.pip_z + p.seat_hole_d / 2))
    for z in p.seat_screw_z:
        part = part.cut(cyl_y(p.seat_screw_x, z, p.seat_screw_d, yb - p.seat_t - o, yb + o))
    return part


def guide_rod(p):
    s = stack(p)
    return cyl_y(p.rod_x, p.rod_z, p.rod_d, s["rod_y0"], s["mot_front"])


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
    # above nose_top the body ends at D = ej_d0 (Dev-Core): the ejector button sits in that cut-out
    body = body.cut(box(x - 20, x + 20, s["body_bottom"] + p.nose_top, s["body_top"] + 1, z - 40, zd(p, p.ej_d0)))
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
    button = box(x - p.ej_hw, x + p.ej_hw, et - p.ej_h, et, zd(p, p.ej_d1), zd(p, p.ej_d0))
    return body.union(stem).union(knob).union(lower).union(button)


def actuator(p, travel=0.0, length=None):
    """YPF 20K-2AB-12.7 (or the eject variant's longer body): body, pilot, boss, anti-rotation
    housing, shaft (+ M2.5 end), connector (-z)."""
    s = stack(p)
    y1, x, z = s["mot_front"], p.pip_x, p.pip_z
    h = p.mot_sq / 2
    mot = box(x - h, x + h, y1, y1 + (length or p.mot_len), z - h, z + h)
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


def tool_board_fused(p):
    """The tool board as ONE solid for Inventor (a multi-solid STEP comes in as an assembly): every
    part is moved 0.15 mm into the board (KiCad exports the laminate ~0.09 mm thinner than the parts sit), then fused."""
    sols = tool_board(p).val().Solids()
    board = max(sols, key=lambda x: (x.BoundingBox().xlen * x.BoundingBox().ylen))
    parts = [x.translate((0, 0, 0.15)) for x in sols if x is not board]
    fused = board.fuse(*parts).clean()
    return cq.Workplane().add(fused)


def tool_board(p):
    """The KiCad tool board (electronics/pcb/out/ot2_pipette_board.step, components included) in the
    tool frame: KiCad STEP (x, y, z) -> (brd_x_ref - x, brd_y_ref + y, brd_z_back + brd_t - z), i.e.
    turned 180 deg about y, component side toward -z (the pipette axis)."""
    shape = cq.importers.importStep(str(PCB_STEP)).val()
    shape = shape.rotate((0, 0, 0), (0, 1, 0), 180).translate((p.brd_x_ref, p.brd_y_ref, p.brd_z_back + p.brd_t))
    return cq.Workplane().add(shape)


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
    add(s["headroom_ej"] >= 3.0, f"eject variant: {p.mot_stroke_ej} mm actuator rear y={s['mot_rear_ej']:.1f}, "
        f"{s['headroom_ej']:.1f} mm below the datum")
    add(abs(s["cone_end"] - s["p20_nozzle"] - p.nozzle_offset) < 0.01,
        f"cone end y={s['cone_end']:.1f}, {-p.nozzle_offset:.1f} mm below the P20 GEN2 nozzle y={s['p20_nozzle']:.1f}")
    add(s["u_blowout"] + 0.2 <= s["u_max"], f"pipetting stroke: blow-out at {s['u_blowout']:.1f} mm of "
        f"{s['u_max']:.1f} mm usable ({p.mot_stroke} stroke - margins)")
    add(s["u_drop"] <= s["u_max_ej"], f"plunger ejection needs {s['u_drop']:.1f} mm travel (stroke >= "
        f"{s['stroke_for_eject']:.1f} mm): the eject variant's {p.mot_stroke_ej} mm actuator gives {s['u_max_ej']:.2f}; "
        f"with {p.mot_stroke} mm use the tip stripper")
    q = s["u_max_ej"] - s["u_blowout"]
    add(s["spring_gap"] - q >= p.spring_ls + 0.3, f"eject springs: {s['spring_gap']:.1f} mm between disc and pad, "
        f"{s['spring_gap'] - q:.2f} left at full squeeze ({q:.2f}) >= solid length {p.spring_ls} + 0.3")
    seat_p = parts["seat"]
    for name in ("frame", "yoke", "yoke_eject", "pad", "seat"):
        bb = parts[name].val().BoundingBox()
        inside = bb.xmin >= p.mount_x - 0.01 and bb.xmax <= p.env_x1 + 0.01 and \
            bb.zmin >= -p.env_zr - 0.01 and bb.zmax <= p.env_z + 0.01
        add(inside, f"{name}: x {bb.xmin:.2f}..{bb.xmax:.2f}, z {bb.zmin:.2f}..{bb.zmax:.2f} "
            f"within x[{p.mount_x}, {p.env_x1}] z[{-p.env_zr}, {p.env_z}] (GEN2 outline)")
    act = actuator(p)
    bb = act.val().BoundingBox()
    add(bb.zmin >= -p.env_z and bb.xmax <= p.env_x1, f"actuator incl. connector housing x<= {bb.xmax:.2f}, "
        f"z>= {bb.zmin:.2f} (plug: right-angle or soldered leads)")
    fr = parts["frame"]
    add(all(len(parts[k].solids().vals()) == 1 for k in ("frame", "yoke", "yoke_eject", "pad", "seat")),
        "frame, yokes, pad and seat are one solid each")
    add(vol(fr.intersect(pipette(p))) < 1.0, "frame clear of the pipette at rest")
    add(vol(fr.intersect(pipette(p, push=s["u_blowout"] - p.pad_gap, eject=p.eject_stroke + p.eject_over))) < 1.0,
        "frame clear of the pipette with the button at blow-out and the ejector pressed")
    add(vol(fr.intersect(act)) < 0.5, "frame clear of the actuator")
    add(vol(fr.intersect(switch(p))) < 0.5, "frame clear of the switch")
    rod = guide_rod(p)
    add(vol(fr.intersect(rod)) < 0.05, "guide rod sits in its hole (column and shelf boss)")
    add(vol(seat_p.intersect(fr)) < 0.1 and vol(seat_p.intersect(pipette(p))) < 0.1,
        "seat clear of frame and pipette (rests under the wall and the body bottom)")
    bb = seat_p.val().BoundingBox()
    add(abs(bb.ymax - s["body_bottom"]) < 0.01, f"seat top y={bb.ymax:.2f} = body bottom: the pipette rests on it")
    brd = tool_board(p)
    bb = brd.val().BoundingBox()
    add(bb.xmin >= p.mount_x - 0.01 and bb.zmax <= p.env_z + 0.01 and bb.xmax <= p.env_x1 + 0.01,
        f"tool board incl. parts: x {bb.xmin:.2f}..{bb.xmax:.2f}, y {bb.ymin:.1f}..{bb.ymax:.1f}, z {bb.zmin:.2f}..{bb.zmax:.2f} "
        f"inside the envelope; header face {bb.xmin - p.mount_x:.2f} mm inside the mount face")
    add(vol(fr.intersect(brd)) < 0.5, f"frame clear of the tool board (overlap {vol(fr.intersect(brd)):.2f} mm3)")
    add(vol(brd.intersect(pipette(p))) < 0.1 and vol(brd.intersect(pipette(p, push=s["u_blowout"] - p.pad_gap))) < 0.1,
        "tool board clear of the pipette at rest and at blow-out")
    add(vol(brd.intersect(switch(p))) < 0.1 and vol(brd.intersect(actuator(p))) < 0.1, "tool board clear of switch and actuator")
    travels = {False: (-p.sw_trigger, 0.0, s["u_blowout"], s["u_max"]),
               True: (-p.sw_trigger, 0.0, s["u_blowout"], s["u_eject0"], s["u_drop"], s["u_max_ej"])}
    for ej in (False, True):
        nm = "eject yoke" if ej else "rigid yoke"
        worst = max(vol(brd.intersect(yoke(p, t, eject=ej))) for t in travels[ej])
        add(worst < 0.1, f"tool board clear of the {nm} over its whole travel (worst {worst:.2f} mm3)")
    sw = switch(p)
    for ej in (False, True):
        nm = "eject yoke" if ej else "yoke"
        length = p.mot_len_ej if ej else None
        for t in travels[ej]:
            y = yoke(p, t, eject=ej)
            bad = [k for k, v in (("frame", vol(y.intersect(fr))),
                                  ("switch", vol(y.intersect(sw)) if t >= 0 else 0.0),
                                  ("actuator", vol(y.intersect(actuator(p, t, length)))),
                                  ("guide rod", vol(y.intersect(rod)) if ej else 0.0)) if v >= 0.1]
            add(not bad, f"{nm} at {t:+.2f} mm clear of frame, switch, actuator" + (", guide rod" if ej else "")
                + (f" - hits {bad}" if bad else ""))
    add(vol(yoke(p, -p.sw_trigger).intersect(sw)) > 0.2 and vol(yoke(p, -p.sw_trigger, True).intersect(sw)) > 0.2,
        "both yokes' tab lifts the switch lever above 'top'")

    def pip_at(t):
        """Pipette as the yoke at travel t leaves it: knob pushed to blow-out at most, ejector by the post."""
        push = min(max(t - p.pad_gap, 0.0), s["u_blowout"] - p.pad_gap)
        return pipette(p, push=push, eject=max(t - s["u_eject0"], 0.0))

    for t in travels[True]:
        if t < 0:
            continue
        pp, sq = pip_at(t), max(0.0, t - s["u_blowout"])
        pd = pad(p, t, squeeze=sq)
        rig = vol(yoke(p, t).intersect(pp)) if t <= s["u_blowout"] else 0.0     # the rigid yoke stops at blow-out
        bad = [k for k, v in (("rigid yoke vs pipette", rig), ("eject yoke vs pipette", vol(yoke(p, t, True).intersect(pp))),
                              ("pad vs pipette", vol(pd.intersect(pp))), ("pad vs eject yoke", vol(pd.intersect(yoke(p, t, True)))),
                              ("pad vs frame", vol(pd.intersect(fr))), ("pad vs board", vol(pd.intersect(brd)))) if v >= 0.1]
        add(not bad, f"at {t:.2f} mm (pad squeezed {sq:.2f}): yokes, pad, pipette, frame, board clear"
            + (f" - {bad}" if bad else ""))
    tip = s["eject_tip_y"] - s["u_drop"]
    add(abs(tip - (s["ej_top"] - p.eject_stroke - p.eject_over)) < 0.01,
        f"set-screw tip at u_drop y={tip:.2f}: ejector pushed {p.eject_stroke + p.eject_over:.1f} mm")
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
    y0 = seat_p.val().BoundingBox().ymin
    add(s["shroud_y"] < y0, f"shroud line y={s['shroud_y']:.1f} below frame and seat (y>={y0:.1f}); "
        f"pipette above it stays inside the GEN2 outline")
    pb = pipette(p).intersect(box(-50, 50, s["shroud_y"], 400, -50, 50)).val().BoundingBox()
    add(pb.xmin >= p.mount_x and pb.xmax <= p.env_x1 and pb.zmin >= -p.env_zr and pb.zmax <= p.env_z,
        f"pipette above the shroud line: x {pb.xmin:.2f}..{pb.xmax:.2f}, z {pb.zmin:.2f}..{pb.zmax:.2f}")
    return res


# ----------------------------------------------------------------------------- main
def build(p):
    return {"frame": frame(p), "seat": seat(p), "yoke": yoke(p), "yoke_eject": yoke(p, eject=True), "pad": pad(p),
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
    names = {"frame": f"{NAME} - frame", "seat": f"{NAME} - pipette seat", "yoke": f"{NAME} - yoke",
             "yoke_eject": f"{NAME} - yoke eject variant", "pad": f"{NAME} - spring pad",
             "stripper": f"{NAME} - tip stripper"}
    for k, wp in parts.items():
        cq.exporters.export(wp, str(OUT / f"{names[k]}.step"))
        cq.exporters.export(wp, str(OUT / f"{names[k]}.stl"), tolerance=0.02, angularTolerance=0.1)
    for variant in ("", " eject"):
        asm = cq.Assembly(name=NAME + variant)
        asm.add(parts["frame"], name=names["frame"], color=cq.Color(0.55, 0.7, 0.85, 1.0))
        asm.add(parts["seat"], name=names["seat"], color=cq.Color(0.55, 0.7, 0.85, 1.0))
        if variant:
            asm.add(parts["yoke_eject"], name=names["yoke_eject"], color=cq.Color(0.5, 0.8, 0.55, 1.0))
            asm.add(parts["pad"], name=names["pad"], color=cq.Color(0.9, 0.75, 0.3, 1.0))
            asm.add(guide_rod(p), name="BUY - steel rod 4 h8 x 75", color=cq.Color(0.8, 0.8, 0.82, 1))
            asm.add(actuator(p, length=p.mot_len_ej), name="BUY - YPF Motors - linear actuator - 20K-2AB-19.05 (estimate)",
                    color=cq.Color(0.3, 0.3, 0.32, 1))
        else:
            asm.add(parts["yoke"], name=names["yoke"], color=cq.Color(0.5, 0.8, 0.55, 1.0))
            asm.add(actuator(p), name="BUY - YPF Motors - linear actuator - 20K-2AB-12.7", color=cq.Color(0.3, 0.3, 0.32, 1))
        asm.add(pipette(p), name="REF - ONiLAB envelope", color=cq.Color(0.75, 0.75, 0.78, 0.6))
        asm.add(switch(p), name="BUY - Omron - micro switch - D2F-L", color=cq.Color(0.2, 0.2, 0.2, 1))
        asm.add(tool_board(p), name="REF - tool board (KiCad)", color=cq.Color(0.1, 0.45, 0.2, 1))
        asm.save(str(OUT / f"{NAME} - assembly{variant}.step"))
    (OUT / "ref").mkdir(exist_ok=True)                     # reference solids for the Inventor assembly
    for nm, wp in (("REF - ONiLAB envelope", pipette(p)), ("REF - tool board (KiCad)", tool_board_fused(p)),
                   ("BUY - Omron - micro switch - D2F-L", switch(p)), ("BUY - steel rod 4 h8 x 75", guide_rod(p))):
        cq.exporters.export(wp, str(OUT / "ref" / f"{nm}.step"))
    report = {"params": asdict(p), "stack": {k: round(v, 2) for k, v in s.items()},
              "checks": [{"status": st, "msg": m} for st, m in res]}
    (OUT / f"{NAME} - build.json").write_text(json.dumps(report, indent=1))
    (OUT / "openuc2_tipstripper_3_slot.json").write_text(json.dumps(stripper_labware(p), indent=1, ensure_ascii=False),
                                                         encoding="utf-8")
    print(json.dumps(report["stack"], indent=1))
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
