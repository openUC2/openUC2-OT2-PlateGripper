"""Example OT-2 protocol for the ONiLAB 0.1-2.5 ul tool (emulates a P20 Single-Channel GEN2).

Volumes and flow rates in this protocol are REAL microlitres; the small wrapper converts them
to the P20 volumes the robot expects. The conversion is a copy of ot2/onilab_tool.py so this
file stays self-contained for upload in the Opentrons App - keep both in sync.
"""
from opentrons import protocol_api

metadata = {"protocolName": "ONiLAB 2.5 ul tool - demo", "author": "openUC2"}
requirements = {"robotType": "OT-2", "apiLevel": "2.20"}

# ---- tool model (copy of onilab_tool.py; edit the MEASURE values after calibration) -------
LEAD_MM, S1, UL_PER_MM = 1.0, 5.0, 0.5            # YPF 20K lead, first-stop travel, real ul/mm
R = 3200.0 / (200 * 32) * LEAD_MM                  # real mm per robot plunger mm
P20 = [(0.8679, 0.1607, 0.5953), (1.0259, 0.1345, 0.5032), (1.1863, 0.1113, 0.527),
       (1.3419, 0.0766, 0.5682), (1.4882, 0.0377, 0.6204), (1.6413, 0.0482, 0.6047),
       (1.7935, 0.039, 0.6199), (1.9447, 0.0313, 0.6336), (2.0929, 0.021, 0.6538),
       (2.4644, 0.0174, 0.6612), (2.8527, 0.0233, 0.6466), (3.2241, 0.0089, 0.6878),
       (3.6095, 0.0141, 0.671), (3.9964, 0.0122, 0.6779), (4.3696, 0.0044, 0.7088),
       (4.7483, 0.0059, 0.7026), (5.1447, 0.0112, 0.6772), (5.9135, 0.0055, 0.7066),
       (7.4416, 0.0033, 0.7199), (8.9724, 0.0023, 0.7269), (10.5091, 0.0019, 0.7305),
       (12.0515, 0.0017, 0.7331), (13.6004, 0.0015, 0.7349), (15.142, 0.001, 0.7422),
       (16.679, 0.0007, 0.7469), (18.221, 0.0007, 0.7466), (19.7746, 0.0009, 0.7434),
       (21.3288, 0.0008, 0.7455)]


def _ul_per_mm(v):
    return next(s * v + i for vmax, s, i in P20 if v <= vmax)


def _real(v_cmd):
    return 0.0 if v_cmd <= 0 else UL_PER_MM * R * v_cmd / _ul_per_mm(v_cmd)


def _cmd(v_real):
    lo, hi = 0.0, 20.0
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if _real(mid) < v_real else (lo, mid)
    if (lo / _ul_per_mm(lo) if lo else 0) > S1 / R + 0.01:
        raise ValueError(f"{v_real} ul exceeds the first-stop stroke")
    return (lo + hi) / 2


class OnilabPipette:
    """Real-volume front end for the P20 GEN2 instrument that is physically the ONiLAB tool."""

    def __init__(self, pipette, aspirate_ul_s=0.5, dispense_ul_s=1.0):
        self.p = pipette
        self.set_flow(aspirate_ul_s, dispense_ul_s)

    def set_flow(self, aspirate_ul_s, dispense_ul_s, blow_out_ul_s=None):
        k = _ul_per_mm(20.0) / (R * UL_PER_MM)
        self.p.flow_rate.aspirate = aspirate_ul_s * k
        self.p.flow_rate.dispense = dispense_ul_s * k
        self.p.flow_rate.blow_out = (blow_out_ul_s or dispense_ul_s) * k

    @property
    def volume(self):
        return _real(self.p.current_volume)

    def aspirate(self, ul, location=None):
        self.p.aspirate(_cmd(self.volume + ul) - self.p.current_volume, location)
        return self

    def dispense(self, ul=None, location=None):
        if ul is None:
            self.p.dispense(None, location)
        else:
            self.p.dispense(self.p.current_volume - _cmd(max(self.volume - ul, 0.0)), location)
        return self

    def strip_tip(self, hole, tip_len=31.0, slot_len=22.0, lift=25.0):
        """Take the tip off in the printed stripper: collar below the plate, slide into the slot,
        lift. Then a drop_tip() at the same spot clears the robot's tip state (no plunger
        over-travel with the 12.7 mm actuator: dropTip = blowout)."""
        from opentrons.types import Point
        below = hole.top(-hole.depth - tip_len - 2.0)       # tip collar 2 mm under the plate
        self.p.move_to(hole.top(5))
        self.p.move_to(below, speed=20)
        self.p.move_to(below.move(Point(y=-slot_len)), speed=10)
        self.p.move_to(below.move(Point(y=-slot_len, z=lift)), speed=10)
        self.p.drop_tip(hole.top(30), home_after=True)

    def __getattr__(self, name):           # pick_up_tip, drop_tip, blow_out, touch_tip, ...
        return getattr(self.p, name)


STRIPPER = None    # paste PipetteTool/cad/out/openuc2_tipstripper_3_slot.json here as a dict


def run(ctx: protocol_api.ProtocolContext):
    tips = ctx.load_labware("opentrons_96_tiprack_10ul", 1)        # standard 10 ul tips: use your rack's definition
    plate = ctx.load_labware("corning_96_wellplate_360ul_flat", 2)
    stripper = ctx.load_labware_from_definition(STRIPPER, 9) if STRIPPER else None
    p20 = ctx.load_instrument("p20_single_gen2", "left", tip_racks=[tips])
    pip = OnilabPipette(p20, aspirate_ul_s=0.5, dispense_ul_s=1.0)

    pip.pick_up_tip()
    for i, vol in enumerate([0.25, 0.5, 1.0, 2.0]):
        pip.aspirate(vol, plate["A1"].bottom(1))
        ctx.delay(seconds=1)                  # let the air cushion settle at sub-ul volumes
        pip.dispense(vol, plate.columns()[1][i].bottom(0.5))
        pip.blow_out(plate.columns()[1][i].bottom(0.5))
        pip.touch_tip(plate.columns()[1][i])
    if stripper:
        pip.strip_tip(stripper["A1"])
    else:
        pip.drop_tip()                        # 12.7 mm actuator cannot eject: remove the tip by hand when paused
        ctx.pause("Remove the tip from the ONiLAB tool, then resume.")
