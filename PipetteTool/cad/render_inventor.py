"""Reopen the native assemblies from disk (references, constraint health, linked parameters) and
render them off-screen from Inventor into docs/pipette_tool_inventor.png.

    C:\\Users\\benir\\miniforge3\\envs\\pyinventor\\python.exe render_inventor.py
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from cadlib.inventor_helpers import close_if_open, safe, session  # noqa: E402

INV = HERE.parent / "INVENTOR"
DOCS = HERE.parent / "docs"
NAME = "OT2-Pipette V0.3"
HIDE = ("pipette envelope", "OT-2 mount bar", "gripper adapter", "ONiLAB-Dev-Core")


def role(occ):
    return safe(lambda: occ.AttributeSets.Item("uc2gen").Item("role").Value, "")


def render(app, doc, png, eye, target, up, extent):
    tg = app.TransientGeometry
    doc.ObjectVisibility.AllWorkFeatures = False
    cam = app.TransientObjects.CreateCamera()
    cam.SceneObject = doc.ComponentDefinition
    cam.Perspective = False
    cam.Eye = tg.CreatePoint(*[v / 10 for v in eye])
    cam.Target = tg.CreatePoint(*[v / 10 for v in target])
    cam.UpVector = tg.CreateUnitVector(*up)
    cam.SetExtents(extent[0] / 10, extent[1] / 10)
    cam.ApplyWithoutTransition()
    cam.SaveAsBitmap(str(png), 900, 1500)


def main():
    out = {}
    with session() as app:
        for nm in ("", " - eject variant"):
            path = INV / f"{NAME}{nm}.iam"
            close_if_open(app, path)
            doc = app.Documents.Open(str(path), False)
            try:
                cd = doc.ComponentDefinition
                missing = [d.FullFileName for d in doc.File.ReferencedFileDescriptors if d.ReferenceMissing]
                bad = [c.Name for c in cd.Constraints if safe(lambda: c.HealthStatus) not in (11778, None)]
                linked = [t.ReferencedDocumentDescriptor.DisplayName for t in cd.Parameters.DerivedParameterTables]
                travel = cd.Parameters.UserParameters.Item("travel").Expression
                print(f"{path.name}: {cd.Occurrences.Count} occurrences, {cd.Constraints.Count} constraints, "
                      f"unhealthy {bad or 'none'}, missing {missing or 'none'}, linked {linked}, travel {travel}")
                out[nm] = (missing, bad)
                for occ in cd.Occurrences:
                    if any(h in role(occ) for h in HIDE[1:]):
                        occ.Visible = False
                # three-quarter view from +x, +z, a little above; the pipette envelope stays (clear)
                png = DOCS / f"pipette_tool_inventor{'_eject' if nm else ''}.png"
                render(app, doc, png, eye=(260, 160, 230), target=(6, 40, 0), up=(0, 1, 0), extent=(150, 250))
                print("  ->", png.name)
            finally:
                doc.Close(True)
    sys.stdout.flush()


if __name__ == "__main__":
    main()
