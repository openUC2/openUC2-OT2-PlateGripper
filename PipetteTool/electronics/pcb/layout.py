"""Placement of the OT2-Pipette tool board (KiCad mm: X right, Y down, origin top left).

Where it sits (tool frame of cad/pipette_tool.py: x away from the mount, y up, z along the bar,
+z = robot front on the LEFT mount): vertical, in a plane parallel to x-y on the +z side of the
yoke, copper from z = 13.75 to 15.35, components on the side facing the pipette axis (-z).
KiCad is viewed from the component side, i.e. from -z: KiCad X = 21.8 - x, KiCad Y = 112 - y.

- J1, right-angle 2x5 box header: its mating face sits 0.5 mm inside the mount face (x = -9.0)
  at y 68.5, z 9.3, where the robot's ribbon plug lands on the left mount (22.5 mm above the
  top screws, 9.3 mm toward the front: Opentrons OT-2 reference CAD + photos). Its key slot faces
  the board's component side (-z, toward the pipette axis) - check against the robot's plug.
- U1, XIAO ESP32-S3, USB-C toward +x (the open side of the tool), hanging 6 mm below the board on
  its own pin headers: clear of the yoke disc (z <= 5.54) by 2 mm. A female socket stack (5-8.5 mm
  more) does not fit inside the GEN2 outline here; the DIP pads take sockets for bench use.
- Motor (J2) and switch (J3) plugs at the bottom edge, the shifters and passives between.
  The XIAO's spare pins D1/D2 (AUX1/AUX2) stay on its own pads (no J4 on this board).
- Two M2 holes at the top edge onto tabs that hang from the motor shelf.
"""
X_REF, Y_REF = 21.8, 112.0          # tool-frame x and y of KiCad (0, 0)
Z_COPPER = (13.75, 15.35)           # board back (component side) and front, tool frame z
W, Y0, Y1 = 21.5, -2.0, 54.0
W_LOW, Y_STEP = 23.5, 28.0       # below y = 84 the wall face is at x = -2.2: the board reaches 0.5 mm from it
OUTLINE = [(0.0, Y0), (W, Y0), (W, Y_STEP), (W_LOW, Y_STEP), (W_LOW, Y1), (0.0, Y1)]
# ref: (x, y, rotation deg, side)
PLACE = {
    "J1": (17.52, 38.42, 0.0, "F"),     # pins 1/3/5/7/9 at frame x 4.28, 2/4/.. at 1.74; face at x -9.0
    "U1": (11.0, 13.0, 90.0, "F"),      # USB toward KiCad X 0 = tool +x
    "J2": (2.6, 50.0, 0.0, "F"),        # JST XH 4P motor
    "J3": (2.6, 43.0, 0.0, "F"),        # JST XH 2P switch
    "Q1": (3.0, 24.6, 0.0, "F"),
    "Q2": (7.8, 24.6, 0.0, "F"),
    "Q3": (12.6, 24.6, 0.0, "F"),
    "R1": (3.0, 28.2, 0.0, "F"),
    "R2": (7.8, 28.2, 0.0, "F"),
    "R3": (12.6, 28.2, 0.0, "F"),
    "R4": (3.0, 30.6, 0.0, "F"),        # DNP
    "R5": (7.8, 30.6, 0.0, "F"),        # DNP
    "C3": (12.6, 30.6, 0.0, "F"),
    "R6": (3.0, 33.0, 0.0, "F"),
    "R7": (7.8, 33.0, 0.0, "F"),
    "D1": (12.9, 33.2, 0.0, "F"),
    "U3": (3.6, 37.4, 0.0, "F"),
    "C2": (8.6, 37.2, 0.0, "F"),
    "C4": (12.8, 37.0, 0.0, "F"),       # DNP
    "U2": (11.4, 42.6, 90.0, "F"),       # DNP (24C02, option A)
    "C1": (16.6, 29.0, 90.0, "F"),
}
HOLES = [(4.0, 0.2, 2.2), (17.6, 0.2, 2.2)]      # x, y, drill (M2 self-tapping into the shelf tabs)
SILK = [("OT2-Pipette tool board V0.5", 10.5, 52.9)]
FRAME = {"x_ref": X_REF, "y_ref": Y_REF, "z_back": Z_COPPER[0], "z_front": Z_COPPER[1]}
