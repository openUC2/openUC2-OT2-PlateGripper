"""Native Inventor parts vs the CadQuery model: the STEP of every part from build_inventor.py
(out/native/) against the CadQuery export of the same part, by volume, bounding box and
point sampling (no booleans between the two kernels).

    uv run --with cadquery python check_native.py [N points per part, default 30000]

A point counts as a mismatch when one model has it inside and the other not; the mismatch
volume is that share of the sampled box. The CadQuery outline is a polyline (24 segments per
arc), the native one true arcs, so a few tens of mm3 along the sleeve are expected.
"""
import json
import os
import sys
from pathlib import Path

import cadquery as cq
import numpy as np
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_IN

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"
NATIVE = OUT / "native"
NAME = "OT2-Pipette V0.6"
PAIRS = {
    "frame": (f"{NAME} - frame.step", f"{NAME} - frame.stp"),
    "seat": (f"{NAME} - pipette seat.step", f"{NAME} - pipette seat.stp"),
    "yoke": (f"{NAME} - yoke.step", f"{NAME} - yoke.stp"),
    "yoke eject": (f"{NAME} - yoke eject variant.step", f"{NAME} - yoke eject variant.stp"),
    "spring pad": (f"{NAME} - spring pad.step", f"{NAME} - spring pad.stp"),
    "tip stripper": (f"{NAME} - tip stripper.step", f"{NAME} - tip stripper.stp"),
    "envelope": ("ref/REF - ONiLAB envelope.step", f"REF - {NAME} - ONiLAB envelope.stp"),
    "switch": ("ref/BUY - Omron - micro switch - D2F-L.step", f"BUY - Omron - micro switch - D2F-L - {NAME} placeholder.stp"),
    "rod": ("ref/BUY - steel rod 4 h8 x 75.step", f"BUY - steel rod - 4 h8 x 75 - {NAME} placeholder.stp"),
}
TOL_REL = 0.005          # 0.5 % of the part volume
TOL_ABS = 5.0            # mm3


def load(path):
    shape = cq.importers.importStep(str(path)).val()
    return shape, sum(s.Volume() for s in shape.Solids()), shape.BoundingBox()


def inside(shape, pts):
    cl = BRepClass3d_SolidClassifier(shape.wrapped)
    out = np.zeros(len(pts), bool)
    for i, (x, y, z) in enumerate(pts):
        cl.Perform(gp_Pnt(x, y, z), 1e-6)
        out[i] = cl.State() == TopAbs_IN
    return out


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 30000
    rng = np.random.default_rng(1)
    report, ok_all = {}, True
    for key, (cq_file, inv_file) in PAIRS.items():
        a, va, ba = load(OUT / cq_file)
        b, vb, bb = load(NATIVE / inv_file)
        lo = np.array([min(ba.xmin, bb.xmin), min(ba.ymin, bb.ymin), min(ba.zmin, bb.zmin)])
        hi = np.array([max(ba.xmax, bb.xmax), max(ba.ymax, bb.ymax), max(ba.zmax, bb.zmax)])
        pts = lo + rng.random((n, 3)) * (hi - lo)
        ia, ib = inside(a, pts), inside(b, pts)
        mis = ia != ib
        vbox = float(np.prod(hi - lo))
        vmis = mis.sum() / n * vbox
        dbox = max(abs(ba.xmin - bb.xmin), abs(ba.ymin - bb.ymin), abs(ba.zmin - bb.zmin),
                   abs(ba.xmax - bb.xmax), abs(ba.ymax - bb.ymax), abs(ba.zmax - bb.zmax))
        ok = abs(va - vb) <= max(TOL_ABS, TOL_REL * va) and vmis <= max(TOL_ABS, TOL_REL * va) + vbox / n * 3 and dbox < 0.05
        ok_all &= ok
        where = None
        if mis.any():
            m = pts[mis]
            where = [round(float(v), 1) for v in (*m.min(0), *m.max(0))]
        report[key] = {"cadquery_mm3": round(va, 1), "native_mm3": round(vb, 1), "delta_mm3": round(vb - va, 1),
                       "bbox_delta_mm": round(dbox, 3), "points": n, "mismatch_points": int(mis.sum()),
                       "mismatch_mm3": round(vmis, 1), "mismatch_region": where, "ok": bool(ok)}
        print(f"[{'ok ' if ok else 'FAIL'}] {key}: CadQuery {va:.1f} / native {vb:.1f} mm3 (d {vb - va:+.1f}), "
              f"bbox d {dbox:.3f} mm, {mis.sum()} of {n} points differ (~{vmis:.1f} mm3) {where or ''}")
    (NATIVE / "check_native.json").write_text(json.dumps(report, indent=1), encoding="utf-8")
    print("all native parts match the CadQuery model" if ok_all else "MISMATCH - see above")
    sys.stdout.flush()
    os._exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
