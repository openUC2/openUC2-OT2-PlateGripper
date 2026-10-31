"""The ONiLAB 0.1-2.5 ul tool as seen by the OT-2: it identifies as a P20 Single-Channel GEN2
(EEPROM model "p20_single_v2.2"); the robot's own plunger driver moves our linear stepper.

This module holds the numbers that link both worlds (robot "command" millimetres and
microlitres vs. real yoke travel and real volume) and the conversions. Plain Python, no
Opentrons import, so it runs on a PC, in Jupyter on the robot, or pasted into a protocol.
Values marked MEASURE are estimates until measured on the real pipette.
"""
from bisect import bisect_left

# --- robot side (Opentrons v8.8.2 sources) -------------------------------------------------
EMULATED_MODEL = "p20_single_v2.2"
ROBOT_STEPS_PER_MM = 3200.0      # GEN2 single plunger axis (model_constants.MOUNT_CONFIG_LOOKUP_TABLE)
ROBOT_MICROSTEPS = 32            # A5984 driver, microstepping on (M52/M54)
HOMED_PLUNGER = 19.0             # plunger coordinate after homing (smoothie constants HOMED_POSITION)
OVERRIDE_RANGE = (-20.0, 30.0)   # allowed top/bottom/blowout/drop (model_constants._MIN_MAX_LOOKUP)
P20_MAX_VOLUME = 20.0            # liquid_class.max_volume used for plunger speed
# P20 GEN2 ul/mm table for the default tips (liquid/single_channel/p20/default/2_2.json, key "1"):
# rows (max volume of the piece, slope, intercept); ul/mm = slope * V + intercept
P20_UL_PER_MM = [
    (0.8679, 0.1607, 0.5953), (1.0259, 0.1345, 0.5032), (1.1863, 0.1113, 0.527),
    (1.3419, 0.0766, 0.5682), (1.4882, 0.0377, 0.6204), (1.6413, 0.0482, 0.6047),
    (1.7935, 0.039, 0.6199), (1.9447, 0.0313, 0.6336), (2.0929, 0.021, 0.6538),
    (2.4644, 0.0174, 0.6612), (2.8527, 0.0233, 0.6466), (3.2241, 0.0089, 0.6878),
    (3.6095, 0.0141, 0.671), (3.9964, 0.0122, 0.6779), (4.3696, 0.0044, 0.7088),
    (4.7483, 0.0059, 0.7026), (5.1447, 0.0112, 0.6772), (5.9135, 0.0055, 0.7066),
    (7.4416, 0.0033, 0.7199), (8.9724, 0.0023, 0.7269), (10.5091, 0.0019, 0.7305),
    (12.0515, 0.0017, 0.7331), (13.6004, 0.0015, 0.7349), (15.142, 0.001, 0.7422),
    (16.679, 0.0007, 0.7469), (18.221, 0.0007, 0.7466), (19.7746, 0.0009, 0.7434),
    (21.3288, 0.0008, 0.7455),
]

# --- our actuator: YPF Motors 20K-2AB-12.7 captive, 1 mm lead (same numbers as cad/pipette_tool.py) ---
MOTOR_STEPS_PER_REV = 200        # 1.8 deg
LEAD_MM = 1.0                    # (Joy-IT NEMA08-04LA of V0.1: 4.0)
MOTOR_STROKE = 12.7              # mm
HOME_MARGIN = 1.2                # mm, robot 'top' below the retracted hard stop
END_MARGIN = 0.3                 # mm, never closer to the extended hard stop
SW_TRIGGER = 0.6                 # mm, the yoke trips the home switch this far above 'top'

# --- the pipette and the yoke (MEASURE) ---------------------------------------------------
PAD_GAP = 0.5        # mm, pad above the plunger button at "top"
S1 = 5.0             # mm, button travel to the first stop with the volume set to 2.5 ul
S2 = 3.0             # mm, extra travel from first stop to blow-out stop
EJECT_BY_PLUNGER = False  # needs an actuator stroke >= 18 mm (eject variant yoke); 12.7 mm -> tip stripper
EJECT_GAP = 0.5      # mm, from blow-out until the arm screw touches the ejector button
EJECT_STROKE = 7.0   # mm, ejector button travel that releases the tip
EJECT_OVER = 0.5     # mm, extra push at the end of the eject
UL_PER_MM = 2.5 / S1 # real ul per mm of button travel at the 2.5 ul setting (MEASURE gravimetrically)

# --- currents for the robot's plunger driver (A, peak) ------------------------------------
# YPF rating not known yet (MEASURE coil resistance; 20 mm hybrids are typically 0.5 A).
# P20 GEN2 idle 0.3 A and homing 1.0 A are NOT overridable.
PLUNGER_CURRENT = 0.5            # run; P20 GEN2 default 1.0
DROP_TIP_CURRENT = 0.6           # only matters when ejecting by over-travel
DROP_TIP_SPEED = 5.0             # command mm/s
PICK_UP_CURRENT = 0.15           # Z current while pressing onto a tip (P20 GEN2 default 0.1)


def real_per_cmd_mm(lead=LEAD_MM):
    """Real yoke travel (mm) per robot plunger millimetre."""
    return ROBOT_STEPS_PER_MM / (MOTOR_STEPS_PER_REV * ROBOT_MICROSTEPS) * lead


def ul_per_mm_p20(v):
    """The robot's ul/mm for a commanded total volume v (piecewise, as in ul_per_mm.py)."""
    for vmax, slope, intercept in P20_UL_PER_MM:
        if v <= vmax:
            return slope * v + intercept
    raise ValueError(f"{v} ul beyond the P20 table")


def cmd_mm_above_bottom(v_cmd):
    """Plunger height above 'bottom' (command mm) that the robot uses for total volume v_cmd."""
    return 0.0 if v_cmd <= 0 else v_cmd / ul_per_mm_p20(v_cmd)


def travel(eject=EJECT_BY_PLUNGER):
    """Real yoke travel (mm below 'top') at bottom / blowout / dropTip, and what the actuator allows."""
    u = {"bottom": PAD_GAP + S1, "blowout": PAD_GAP + S1 + S2}
    u["dropTip"] = u["blowout"] + (EJECT_GAP + EJECT_STROKE + EJECT_OVER if eject else 0.0)
    return u, MOTOR_STROKE - HOME_MARGIN - END_MARGIN


def plunger_positions(lead=LEAD_MM):
    """top/bottom/blowout/dropTip in command mm (positive = up), checked against the API range
    and the actuator stroke. The robot sets the plunger to HOMED_PLUNGER where the switch trips."""
    r = real_per_cmd_mm(lead)
    top = HOMED_PLUNGER - SW_TRIGGER / r
    u, u_max = travel()
    pos = {"top": top, **{k: top - v / r for k, v in u.items()}}
    lo, hi = OVERRIDE_RANGE
    bad = {k: v for k, v in pos.items() if not lo <= v <= hi}
    if bad:
        raise ValueError(f"positions outside {OVERRIDE_RANGE}: {bad} - change LEAD_MM or the strokes")
    if u["dropTip"] > u_max:
        raise ValueError(f"dropTip needs {u['dropTip']:.1f} mm of travel, the actuator gives {u_max:.1f} mm")
    return {k: round(v, 3) for k, v in pos.items()}


class VolumeMap:
    """Real ul <-> commanded ul. Theory first; replace by gravimetric points with set_points()."""

    def __init__(self, lead=LEAD_MM, ul_per_mm=UL_PER_MM):
        self.r = real_per_cmd_mm(lead)
        self.k = ul_per_mm
        self.points = None          # [(v_cmd, v_real)] sorted, from a calibration run
        plunger_positions(lead)                         # raises if the strokes do not fit
        self.cmd_stroke = S1 / self.r                   # command mm above 'bottom' the button follows

    def real_from_cmd(self, v_cmd):
        if self.points:
            return _interp(self.points, v_cmd)
        return self.k * self.r * cmd_mm_above_bottom(v_cmd)

    def cmd_from_real(self, v_real):
        """Commanded total volume that puts v_real ul into the tip (bisection, monotonic)."""
        if v_real <= 0:
            return 0.0
        lo, hi = 0.0, P20_MAX_VOLUME
        if self.real_from_cmd(hi) < v_real:
            raise ValueError(f"{v_real} ul exceeds the tool's stroke")
        for _ in range(60):
            mid = (lo + hi) / 2
            if self.real_from_cmd(mid) < v_real:
                lo = mid
            else:
                hi = mid
        v = (lo + hi) / 2
        if cmd_mm_above_bottom(v) > self.cmd_stroke + 0.01:      # P20 table has 6 um steps
            raise ValueError(f"{v_real} ul needs more than the first-stop stroke")
        return v

    def max_real(self):
        lo, hi = 0.0, P20_MAX_VOLUME
        for _ in range(60):
            mid = (lo + hi) / 2
            if cmd_mm_above_bottom(mid) <= self.cmd_stroke:
                lo = mid
            else:
                hi = mid
        return self.real_from_cmd(lo)

    def cmd_flow(self, real_ul_per_s):
        """Commanded flow rate (ul/s) for a real flow rate; the robot divides by ul/mm at 20 ul."""
        return real_ul_per_s * ul_per_mm_p20(P20_MAX_VOLUME) / (self.r * self.k)

    def set_points(self, pairs):
        """pairs: [(v_cmd, v_real_measured)], e.g. from weighing 10 dispenses per point."""
        self.points = sorted(pairs)


def _interp(points, x):
    xs = [p[0] for p in points]
    i = min(max(bisect_left(xs, x), 1), len(points) - 1)
    (x0, y0), (x1, y1) = points[i - 1], points[i]
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def overrides():
    """Fields for PATCH /settings/pipettes/{id} on the robot (see configure_pipette.py)."""
    f = dict(plunger_positions())
    f.update(plungerCurrent=PLUNGER_CURRENT, dropTipCurrent=DROP_TIP_CURRENT,
             dropTipSpeed=DROP_TIP_SPEED, pickUpCurrent=PICK_UP_CURRENT)
    return f


if __name__ == "__main__":
    m = VolumeMap()
    print(f"real mm per command mm: {m.r:.3f}")
    print("overrides:", overrides())
    print(f"max real volume at the first stop: {m.max_real():.3f} ul")
    for v in (0.1, 0.25, 0.5, 1.0, 1.25, 2.0, 2.5):
        print(f"  real {v:5.2f} ul -> aspirate({m.cmd_from_real(v):.4f}) on the P20")
    print(f"real 1 ul/s -> flow_rate {m.cmd_flow(1.0):.3f} ul/s")
