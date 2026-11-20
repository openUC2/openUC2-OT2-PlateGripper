"""Inventor parameter table of the OT2-Pipette tool, built from pipette_tool.P (COM-free).

Base parameters = the fields of P (tuples split into named values), comments taken from the
inline comments in pipette_tool.py. Derived parameters = stack() as expressions, so Inventor
recomputes the heights itself when a base value changes in the parameter part. `check()`
evaluates the expressions here and compares them with stack(): both models must agree.

pipette_tool.py runs `check()` and stores the values in "<NAME> - build.json"; build_inventor.py
(pyinventor env, no CadQuery) rebuilds the table from that file with `from_build_json()`.

    uv run --with cadquery python pipette_params.py     # table + consistency check
"""
import json
import re
import sys
from dataclasses import asdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from cadlib.params import D, P, ParamSet  # noqa: E402

RENAME = {"s1": "stroke1", "s2": "stroke2"}          # names Inventor would read as units or that clash
TUPLES = {
    "inserts": None,                                   # special: (top y, +-z) pair and the bottom one
    "disp_y": ("disp_y0", "disp_y1"),
    "grub_y": ("grub_y1", "grub_y2"),
    "conn_y": ("conn_y0", "conn_y1"),
    "brd_hole_x": ("brd_hole_x1", "brd_hole_x2"),
    "brd_xiao_y": ("brd_xiao_y0", "brd_xiao_y1"),
}
# stack() key -> (Inventor name, expression); the order is the evaluation order
DERIVED = [
    ("p20_nozzle", "p20_nozzle", "datum_y - p20_top_to_nozzle", "P20 GEN2 nozzle"),
    ("cone_end", "cone_end", "p20_nozzle + nozzle_offset", "end of the ONiLAB tip cone"),
    ("lower_len", "lower_len", "collar_len + taper_len + sleeve_len + cone_len", "body bottom -> cone end"),
    ("body_bottom", "body_bottom", "cone_end + lower_len", "ONiLAB body bottom"),
    ("body_top", "body_top", "body_bottom + body_len", "ONiLAB body top face (ledge sits on it)"),
    ("knob_top", "knob_top", "body_top + knob_above_body", "plunger knob top at rest"),
    ("ej_top", "ej_top", "body_top - ej_top_below", "ejector button top at rest"),
    ("top", "top_y", "knob_top + pad_gap + yoke_total", "shaft shoulder (yoke top) at the robot's 'top'"),
    ("retracted", "retracted", "top_y + home_margin", "shaft shoulder fully retracted"),
    ("mot_front", "mot_front", "retracted + shaft_out", "actuator front face = motor seat on the shelf"),
    ("mot_rear", "mot_rear", "mot_front + mot_len", "actuator rear = top of the frame"),
    ("headroom", "headroom", "datum_y - mot_rear", "actuator rear below the GEN2 top datum"),
    ("shroud_y", "shroud_y", "datum_y - shroud_drop", "gantry shroud line"),
    ("sleeve_y0", "sleeve_y0", "body_bottom + sleeve_bottom", "sleeve bottom"),
    ("u_max", "u_max", "mot_stroke - home_margin - end_margin", "usable travel below 'top'"),
    ("u_bottom", "u_bottom", "pad_gap + stroke1", "travel to the first stop"),
    ("u_blowout", "u_blowout", "u_bottom + stroke2", "travel to the blow-out stop"),
    ("u_eject0", "u_eject0", "u_blowout + eject_gap", "eject screw touches the ejector button"),
    ("u_drop", "u_drop", "u_eject0 + eject_stroke + eject_over", "travel that releases the tip"),
    ("stroke_for_eject", "stroke_for_eject", "u_drop + home_margin + end_margin", "actuator stroke for plunger ejection"),
    ("shelf_x1", "shelf_x1", "min(pip_x + mot_sq / 2 + shelf_side; env_x1 - env_margin)", "shelf +x end"),
    ("gusset_h", "gusset_h", "shelf_x1 - wall_x1", "gusset height below the shelf"),
    ("sw_y0", "sw_y0", "top_y + lever_clear + lever_drop", "switch body bottom"),
    ("pad_face", "pad_face", "top_y - yoke_total", "pad face (rigid yoke) / relaxed spring pad face"),
    ("skirt_y0", "skirt_y0", "pad_face + nose_out", "eject yoke: skirt bottom"),
    ("arm_y0", "arm_y0", "skirt_y0 + arm_y", "eject yoke: arm bar bottom"),
    ("slot_y1", "slot_y1", "skirt_y0 + pin_y + eject_stroke + eject_over + slot_end", "eject yoke: pin slot top"),
    ("eject_tip_y", "eject_tip_y", "ej_top + u_eject0", "adjust-screw tip at 'top'"),
    ("arm_boss_y", "arm_boss_y", "eject_tip_y + screw_out", "adjust-screw boss bottom"),
]
EXTRA = [   # convenience values used by several parts
    D("wall_out", "clr + wall", "mm", "sleeve outline offset from the body outline"),
    D("knob_y0", "knob_top - knob_h", "mm", "knob bottom at rest"),
    D("brd_z_front", "brd_z_back + brd_t", "mm", "tool board face toward +z (tabs)"),
    D("shelf_y0", "mot_front - shelf_t", "mm", "underside of the motor shelf"),
    D("st_x2", "st_x1 + st_pitch", "mm", "stripper entry hole 2"),
    D("st_x3", "st_x1 + 2 * st_pitch", "mm", "stripper entry hole 3"),
]


def comments(src: Path) -> dict:
    """Inline comments of the fields of P in pipette_tool.py; a comment line continues to the
    fields that follow without their own."""
    out = {}
    for line in src.read_text(encoding="utf-8").splitlines():
        m = re.match(r"\s+(\w+): \w+ = [^#]*#\s*(.+)$", line)
        if m:
            out[m.group(1)] = m.group(2).strip()
    return out


def parameters(values: dict) -> list:
    """`values`: the fields of pipette_tool.P by name (asdict(P()) or build.json "params")."""
    com = comments(HERE / "pipette_tool.py")
    out = []
    for name, v in values.items():
        if isinstance(v, (tuple, list)):
            if name == "inserts":
                (y0, z0), (y1, z1), (y2, z2) = v
                if not (y0 == y1 and z0 == -z1 and z2 == z1):
                    raise ValueError(f"insert pattern {v} is not (top y, +-z), (bottom y, -z)")
                out += [P("ins_top_y", y0, "mm", "M3 inserts on the mount face: top pair height"),
                        P("ins_bot_y", y2, "mm", "bottom insert height (at -z)"),
                        P("ins_z", z0, "mm", "inserts at +-z")]
            else:
                for n, x in zip(TUPLES[name], v):
                    out.append(P(n, x, "mm", com.get(name, "")))
            continue
        out.append(P(RENAME.get(name, name), v, "mm", com.get(name, "")))
    out += [D(n, e, "mm", c) for _, n, e, c in DERIVED]
    return out + EXTRA


def make(values: dict) -> ParamSet:
    return ParamSet(parameters(values))


def from_build_json(path: Path) -> ParamSet:
    return make(json.loads(Path(path).read_text(encoding="utf-8"))["params"])


def check(p, stack) -> list[str]:
    """Differences between the Inventor expressions and stack(p) (empty = consistent)."""
    ps, s = make(asdict(p)), stack(p)
    bad = [f"{n}: {ps[n]:.4f} vs stack {s[k]:.4f}" for k, n, _, _ in DERIVED if abs(ps[n] - s[k]) > 1e-6]
    for name, v in asdict(p).items():
        if not isinstance(v, (tuple, list)) and abs(ps[RENAME.get(name, name)] - v) > 1e-9:
            bad.append(f"{name} differs")
    return bad


if __name__ == "__main__":
    import pipette_tool as t
    ps = make(asdict(t.P()))
    for row in ps.table():
        print(f"{row['name']:<18} {row['value']:>10.3f}  {row['expr']:<55} {row['comment'][:60]}")
    bad = check(t.P(), t.stack)
    print(f"{len(ps.items)} parameters;", "consistent with stack()" if not bad else f"MISMATCH: {bad}")
