"""Build the tool board end to end and write the JLCPCB files (plain Python, calls KiCad 10).

    python make.py              # place -> route -> finish -> DRC -> fab files -> STEP -> renders

out/ot2_pipette_board.kicad_pcb    the board (open it in KiCad 10 to review or edit)
out/drc.json                       DRC report (must show 0 violations, 0 unconnected)
out/jlcpcb/gerbers.zip             Gerbers + Excellon drill, upload to jlcpcb.com
out/jlcpcb/bom.csv, cpl.csv        JLCPCB assembly: Comment, Designator, Footprint, LCSC / position file
out/ot2_pipette_board.step         board with components, for the Inventor assembly
out/render_top.png, render_bottom.png
"""
import csv
import json
import os
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"
KICAD = Path(os.environ.get("KICAD10", r"C:\Program Files\KiCad\10.0\bin"))
PY = str(KICAD / "python.exe")
CLI = str(KICAD / "kicad-cli.exe")
BOARD = OUT / "ot2_pipette_board.kicad_pcb"


def run(args, **kw):
    print(">", " ".join(str(a) for a in args)[:160], flush=True)
    r = subprocess.run([str(a) for a in args], cwd=HERE, capture_output=True, text=True, **kw)
    if r.returncode:
        print(r.stdout[-2000:], r.stderr[-2000:])
        raise SystemExit(f"failed: {args[0]}")
    return r.stdout


def main():
    sys.path.insert(0, str(HERE))
    from netlist import PARTS
    run([PY, "build_pcb.py", "place"])
    run(["uv", "run", "--python", "3.12", "--with", "numpy", "--with", "scipy", "python", "-u", "route.py",
         OUT / "pads.json", OUT / "routes.json"])
    routes = json.loads((OUT / "routes.json").read_text())
    run([PY, "build_pcb.py", "finish"])
    run([CLI, "pcb", "drc", "--format", "json", "--severity-error", "-o", OUT / "drc.json", BOARD])
    drc = json.loads((OUT / "drc.json").read_text())
    viol = [v for v in drc.get("violations", []) if v.get("severity") == "error"]
    unc = drc.get("unconnected_items", [])
    print(f"DRC: {len(viol)} errors, {len(unc)} unconnected")
    for v in viol[:15]:
        print("   ", v.get("type"), "-", v.get("description"), [i.get("description") for i in v.get("items", [])][:2])
    fab = OUT / "jlcpcb"
    if fab.exists():
        shutil.rmtree(fab)
    (fab / "gerber").mkdir(parents=True)
    run([CLI, "pcb", "export", "gerbers", "--layers", "F.Cu,B.Cu,F.Paste,B.Paste,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts",
         "--subtract-soldermask", "-o", str(fab / "gerber") + os.sep, BOARD])
    run([CLI, "pcb", "export", "drill", "--format", "excellon", "--excellon-separate-th", "--generate-map",
         "--map-format", "gerberx2", "-o", str(fab / "gerber") + os.sep, BOARD])
    with zipfile.ZipFile(fab / "gerbers.zip", "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted((fab / "gerber").iterdir()):
            z.write(f, f.name)
    run([CLI, "pcb", "export", "pos", "--format", "csv", "--units", "mm", "--side", "both", "--exclude-dnp",
         "-o", fab / "pos_raw.csv", BOARD])
    # JLCPCB CPL: Designator, Mid X, Mid Y, Layer, Rotation
    rows = list(csv.DictReader(open(fab / "pos_raw.csv", newline="")))
    with open(fab / "cpl.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for r in rows:
            w.writerow([r["Ref"], f"{float(r['PosX']):.3f}mm", f"{float(r['PosY']):.3f}mm",
                        "Top" if r["Side"].lower().startswith("top") else "Bottom", f"{float(r['Rot']):.1f}"])
    groups = {}
    for ref, (lib, value, lcsc, nets, populate, note) in PARTS.items():
        if populate and lcsc:
            groups.setdefault((value, lib.split(":")[-1], lcsc), []).append(ref)
    with open(fab / "bom.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
        for (value, fp, lcsc), refs in sorted(groups.items()):
            w.writerow([value, ",".join(sorted(refs)), fp, lcsc])
    run([CLI, "pcb", "export", "step", "--subst-models", "--force", "-o", OUT / "ot2_pipette_board.step", BOARD])
    for side in ("top", "bottom"):
        run([CLI, "pcb", "render", "--side", side, "--quality", "high", "-w", "1400", "-h", "800",
             "-o", OUT / f"render_{side}.png", BOARD])
    run([CLI, "pcb", "render", "--rotate", "-45,0,35", "--quality", "high", "-w", "1400", "-h", "900", "--zoom", "0.9",
         "-o", OUT / "render_iso.png", BOARD])
    summary = {"routing_failed": routes["failed"], "drc_errors": len(viol), "unconnected": len(unc),
               "segments": len(routes["tracks"]), "vias": len(routes["vias"])}
    (OUT / "summary.json").write_text(json.dumps(summary, indent=1))
    print(summary)


if __name__ == "__main__":
    main()
