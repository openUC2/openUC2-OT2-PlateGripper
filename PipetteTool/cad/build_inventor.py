"""Inventor assemblies of the OT2-Pipette tool from the CadQuery exports (run pipette_tool.py first).

    C:\\Users\\benir\\miniforge3\\envs\\pyinventor\\python.exe build_inventor.py
Imports the printed parts and reference solids (STEP, all in the gripper-adapter frame) as
parts into ../INVENTOR, places them with the CAD-new YPF actuator (in place, never copied),
three Ruthex inserts and, as reference, the OT-2 mount bar model, the gripper adapter and the
ONiLAB-Dev-Core (both hidden); checks interference; saves "OT2-Pipette V0.2.iam" and
"OT2-Pipette V0.2 - eject variant.iam". Works in the open session with invisible documents.
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, r"C:\Users\benir\Documents\openuc2-frame-opm")
from cadlib.inventor_helpers import (Builder, CadNew, appearance, close_if_open, find_open, import_step,  # noqa: E402
                                     kAssemblyDocumentObject, kReferenceBOMStructure, safe, session,
                                     set_appearance)

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"
REPO = HERE.parents[1]
INV = HERE.parent / "INVENTOR"
NAME = "OT2-Pipette V0.2"
I = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
YPF = "BUY - Stepping motor - 20K-2AB-12.7 - YPF Motors"
BAR_REF = REPO / "INVENTOR" / "00_OT-2 Reference_Pipettte.ipt"
ADAPTER = REPO / "INVENTOR" / "OT2-Gripper V1-1 - OT2-interface adapter.ipt"
RUTHEX = REPO / "INVENTOR" / "Threaded Insert-Ruthex-M3.ipt"
DEVCORE = REPO / "PipetteGrabCad" / "ONiLAB-Dev-Core.ipt"
BAR_SHIFT = (-60.5, -441.5, -83.445)       # reference model -> adapter frame (bar screw holes on the inserts)

PARTS = {   # key: (STEP, ipt stem, description)
    "frame": (OUT / f"{NAME} - frame.step", f"{NAME} - frame",
              "printed frame: mount wall with 3 inserts, sleeve + ledge, header window, motor shelf (PETG/ASA)"),
    "yoke": (OUT / f"{NAME} - yoke.step", f"{NAME} - yoke", "printed yoke on the actuator shaft (M2.5 insert), rigid pad"),
    "yoke eject": (OUT / f"{NAME} - yoke eject variant.step", f"{NAME} - yoke eject variant",
                   "printed yoke, eject variant: skirt for the spring pad, arm with M3 adjust screw"),
    "spring pad": (OUT / f"{NAME} - spring pad.step", f"{NAME} - spring pad", "printed spring pad, eject variant"),
    "tip stripper": (OUT / f"{NAME} - tip stripper.step", f"{NAME} - tip stripper", "printed deck-slot tip stripper"),
    "envelope": (OUT / "ref" / "REF - ONiLAB envelope.step", "REF - ONiLAB envelope",
                 "ONiLAB 0.1-2.5 ul envelope: Dev-Core body (hook cut), photo lower part, ejector button"),
    "board": (OUT / "ref" / "REF - identity board + 2x5 header.step", "REF - identity board + 2x5 header",
              "perfboard with 24C02 and the shrouded 2x5 header (placeholder)"),
    "switch": (OUT / "ref" / "BUY - Omron - micro switch - D2F-L.step", "BUY - Omron - micro switch - D2F-L",
               "home switch placeholder (body + lever)"),
}
COLOURS = {"printed": ["Light Steel Blue", "Sky Blue Medium"], "yoke": ["Green", "Smooth - Green", "Green - Medium"],
           "pad": ["Yellow", "Smooth - Yellow", "Orange"], "pcb": ["Green - Dark", "Dark Green", "Green"],
           "ref": ["Clear", "Glass - Clear", "Clear - Light", "Glass"], "black": ["Black", "Smooth - Black"]}


def stack():
    rep = json.loads((OUT / f"{NAME} - build.json").read_text())
    return rep["params"], rep["stack"]


def import_parts(app) -> dict:
    """STEP -> .ipt (re-imported when the STEP is newer), Part Number / Description / BOM set."""
    files = {}
    for key, (step, stem, desc) in PARTS.items():
        ipt = import_step(app, step, INV / f"{stem}.ipt")
        if ipt is None:
            raise SystemExit(f"{step.name} missing - run pipette_tool.py first")
        doc = find_open(app, ipt) or app.Documents.Open(str(ipt), False)
        props = doc.PropertySets.Item("Design Tracking Properties")
        changed = False
        for k, v in (("Part Number", stem), ("Description", desc)):
            if safe(lambda: props.Item(k).Value) != v:
                props.Item(k).Value = v
                changed = True
        if stem.startswith("REF - ") and safe(lambda: doc.ComponentDefinition.BOMStructure) != kReferenceBOMStructure:
            doc.ComponentDefinition.BOMStructure = kReferenceBOMStructure
            changed = True
        if changed:
            doc.Save()
        files[key] = ipt
    return files


def rows(p, s, files, eject: bool):
    """(role, file, origin_mm, columns, reference, visible, colour) in the adapter frame."""
    ypf = CadNew().find(YPF + ".ipt")
    r = [("frame", files["frame"], (0, 0, 0), I, False, True, "printed")]
    if eject:
        r += [("yoke eject", files["yoke eject"], (0, 0, 0), I, False, True, "yoke"),
              ("spring pad", files["spring pad"], (0, 0, 0), I, False, True, "pad")]
    else:
        r += [("yoke", files["yoke"], (0, 0, 0), I, False, True, "yoke")]
    # YPF part: axis +x (front face x = 0, shaft toward -x), connector on -y  ->  axis -y, connector -z
    r += [("actuator", ypf, (p["pip_x"], s["mot_front"], p["pip_z"]), ((0, 1, 0), (0, 0, 1), (1, 0, 0)),
           False, True, None),
          ("home switch", files["switch"], (0, 0, 0), I, False, True, "black"),
          ("identity board", files["board"], (0, 0, 0), I, False, True, "pcb")]
    # Ruthex part: axis +y from the lead-in (y = 0) to the flush end (y = 5.7) -> flush end on the mount face
    for k, (y, z) in enumerate(p["inserts"], 1):
        r.append((f"insert {k}", RUTHEX, (p["mount_x"] + 5.7, y, z), ((0, 1, 0), (-1, 0, 0), (0, 0, 1)),
                  False, True, None))
    r += [("pipette envelope", files["envelope"], (0, 0, 0), I, True, True, "ref"),
          ("OT-2 mount bar (reference model)", BAR_REF, BAR_SHIFT, I, True, True, None),
          ("gripper adapter (same frame)", ADAPTER, (0, 0, 0), I, True, False, None),
          # Dev-Core: axis at x -23.34, body bottom y 0, nose +x, flats on z  ->  axis on the tool, nose +z
          ("ONiLAB-Dev-Core (with hook)", DEVCORE, (p["pip_x"], s["body_bottom"], p["pip_z"] + 23.34),
           ((0, 0, 1), (0, 1, 0), (-1, 0, 0)), True, False, None)]
    return r


PAIRS = [("frame", "pipette envelope"), ("frame", "actuator"), ("frame", "home switch"), ("frame", "identity board"),
         ("frame", "insert 1"), ("frame", "insert 2"), ("frame", "insert 3"), ("frame", "OT-2 mount bar (reference model)"),
         ("yoke", "actuator"), ("yoke", "pipette envelope"), ("yoke", "frame"), ("yoke", "home switch"),
         ("yoke eject", "actuator"), ("yoke eject", "pipette envelope"), ("yoke eject", "frame"),
         ("spring pad", "yoke eject"), ("spring pad", "pipette envelope"),
         ("actuator", "OT-2 mount bar (reference model)"), ("pipette envelope", "OT-2 mount bar (reference model)")]
KNOWN = {("frame", "insert 1"): "heat-set insert knurl in its 4.1 mm hole (by design)",
         ("frame", "insert 2"): "heat-set insert knurl in its 4.1 mm hole (by design)",
         ("frame", "insert 3"): "heat-set insert knurl in its 4.1 mm hole (by design)"}


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


def build(app, path: Path, rws, assets) -> dict:
    close_if_open(app, path)
    doc = app.Documents.Add(kAssemblyDocumentObject, app.FileManager.GetTemplateFile(kAssemblyDocumentObject), False)
    try:
        props = doc.PropertySets.Item("Design Tracking Properties")
        props.Item("Part Number").Value = path.stem
        props.Item("Description").Value = "ONiLAB 0.1-2.5 ul pipette as OT-2 tool (P20 GEN2 emulation), adapter frame"
        b = Builder(app, doc)
        occ = {}
        for role, f, org, cols, ref, vis, col in rws:
            occ[role] = b.place(role, Path(f), org, cols, ground=True, reference=ref)
            if not vis:
                occ[role].Visible = False
            if col and assets.get(col) is not None:
                set_appearance(doc, occ[role], assets[col])
        doc.Update()
        checks = []
        for a, c in PAIRS:
            if a in occ and c in occ:
                v, boxes = interference(app, doc.ComponentDefinition, occ[a], occ[c])
                note = KNOWN.get((a, c), "")
                checks.append({"pair": f"{a} / {c}", "mm3": v, "boxes": boxes[:3], "note": note})
                flag = "ok " if v == 0 else ("known" if note else ("n/a" if v is None else "CHECK"))
                print(f"  [{flag}] {a} / {c}: {v} mm3 {note}")
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.exists():
            path.unlink()
        doc.SaveAs(str(path), False)
        missing = [safe(lambda: d.FullFileName) for d in doc.File.ReferencedFileDescriptors if d.ReferenceMissing]
        return {"assembly": str(path), "occurrences": b.placed, "interference": checks, "missing_refs": missing}
    finally:
        doc.Close(True)


def main():
    p, s = stack()
    for f in (BAR_REF, ADAPTER, RUTHEX, DEVCORE):
        if not f.exists():
            raise SystemExit(f"missing {f}")
    report = {}
    with session() as app:
        for nm in ("", " - eject variant"):
            close_if_open(app, INV / f"{NAME}{nm}.iam")
        files = import_parts(app)
        assets = {k: appearance(app, v) for k, v in COLOURS.items()}
        print("  appearances:", {k: safe(lambda: a.DisplayName) if a is not None else None for k, a in assets.items()})
        for nm, eject in (("", False), (" - eject variant", True)):
            print(f"{NAME}{nm}.iam")
            report[nm or "main"] = build(app, INV / f"{NAME}{nm}.iam", rows(p, s, files, eject), assets)
    (INV / f"{NAME} - inventor build.json").write_text(json.dumps(report, indent=1), encoding="utf-8")
    for k, r in report.items():
        print(k, "missing references:", r["missing_refs"] or "none")
    sys.stdout.flush()


if __name__ == "__main__":
    main()
