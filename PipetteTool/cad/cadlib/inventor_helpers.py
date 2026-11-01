"""Inventor COM helpers shared by the openUC2 module builders (identical copies in
openuc2-opmsimulator and openuc2-openraman, so each repo stands alone - keep them in sync).

Needs the miniforge `pyinventor` env (pywin32) and a running Inventor; the API works in cm.

CAD-new names: builders place the openUC2-CAD-new files themselves (`CadNew.find`; that
workspace is a library path of the FRAME project), never copies, and keep Inventor's own
occurrence names ("ASS - 2000 - CUB - V04:3").  `Builder.place` tags every occurrence with an
attribute (set "uc2gen", key "role"), so a re-run finds it again without renaming it.
`occ_frame`/`turn_frame` re-place a CAD-new module's parts turned inside a cube. Generated
parts get CAD-new-style names with provisional numbers (`cadnew_name`); rays and beam solids
are "REF - ..." parts kept out of the BOM.  `session()` restores SilentOperation and closes
only the documents the script itself loaded.
"""
from __future__ import annotations

import os
import re
from contextlib import contextmanager
from pathlib import Path

import win32com.client

kPartDocumentObject = 12290
kAssemblyDocumentObject = 12291
kStringType = 14595                      # ValueTypeEnum
kReferenceBOMStructure = 51972           # BOMStructureEnum (read from RxInventor.tlb)
kPurchasedBOMStructure = 51973
kIsoTopRightViewOrientation = 10759
R_IDENT = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
ROLE_SET, ROLE_KEY = "uc2gen", "role"
CADNEW_WORKSPACE = Path(os.environ.get("UC2_CADNEW_WORKSPACE",
                                       r"C:\Users\benir\Documents\openUC2-CAD-new\workspace"))
_OLD_VERSION = re.compile(r"\.\d{4}\.(ipt|iam)$", re.IGNORECASE)    # Inventor's "x.0001.ipt" backups
_CADNEW_NAME = re.compile(r"^(PRT|ASS|SUB|MAS|BUY|STD|OPM) - (\d{3,4}) - ")


def safe(fn, default=None):
    try:
        return fn()
    except Exception:
        return default


# --------------------------------------------------------------------------- CAD-new names
class CadNew:
    """File index of the CAD-new workspace (old versions skipped).  `find` takes an exact file
    name ("PRT - 1004 - PUZ11 - V04 - A.ipt") or a number prefix ("PRT - 1004")."""

    def __init__(self, root: Path = CADNEW_WORKSPACE):
        self.root = Path(root)
        if not self.root.is_dir():
            raise FileNotFoundError(f"CAD-new workspace not found: {self.root} (set UC2_CADNEW_WORKSPACE)")
        self.files: dict[str, list[Path]] = {}
        for dirpath, dirnames, filenames in os.walk(self.root):
            dirnames[:] = [d for d in dirnames if d.lower() != "oldversions" and not d.startswith("_")]
            for f in filenames:
                if f.lower().endswith((".ipt", ".iam")) and not _OLD_VERSION.search(f):
                    self.files.setdefault(f.lower(), []).append(Path(dirpath) / f)

    def find(self, name: str) -> Path:
        hits = self.files.get(name.lower())
        if not hits:
            pre = name.lower().rstrip() + " - "
            hits = [p for k, ps in self.files.items() if k.startswith(pre) for p in ps]
            if len({p.name.lower() for p in hits}) > 1:
                raise LookupError(f"{name!r} is ambiguous in CAD-new: {sorted({p.name for p in hits})}")
        if not hits:
            raise FileNotFoundError(f"{name!r} is not in the CAD-new workspace {self.root}")
        return min(hits, key=lambda p: (len(p.parts), str(p)))     # the copy nearest the top

    def numbers_used(self, prefix: str) -> set[int]:
        out = set()
        for ps in self.files.values():
            m = _CADNEW_NAME.match(ps[0].name)
            if m and m.group(1) == prefix:
                out.add(int(m.group(2)))
        return out


def cadnew_name(prefix: str, number: int, abbrev: str, rev: str = "V04") -> str:
    """'PRT - 9101 - OPMDETHOL - V04': the CAD-new pattern <prefix> - <number> - <ABBREV> - <rev>."""
    return f"{prefix} - {number:04d} - {abbrev} - {rev}"


def check_provisional(lib: CadNew, names) -> list[str]:
    """Generated names whose number CAD-new already uses (they must be renumbered)."""
    clash = []
    for n in names:
        m = _CADNEW_NAME.match(n)
        if m and int(m.group(2)) in lib.numbers_used(m.group(1)):
            clash.append(n)
    return clash


# --------------------------------------------------------------------------- session
@contextmanager
def session():
    """Inventor with SilentOperation on.  On exit the previous value is restored and every
    document this script loaded is closed again, unless it is visible, dirty or still
    referenced; documents that were open before are never touched."""
    app = win32com.client.GetActiveObject("Inventor.Application")
    silent = bool(app.SilentOperation)
    before = {(safe(lambda: d.FullFileName) or "").lower() for d in app.Documents}
    app.SilentOperation = True
    try:
        yield app
    finally:
        try:
            n = close_loaded_since(app, before)
            if n:
                print(f"  (closed {n} documents this script loaded)")
        finally:
            try:
                app.SilentOperation = silent
            except Exception:
                pass


def close_loaded_since(app, before: set[str]) -> int:
    visible = {(safe(lambda: d.FullFileName) or "").lower() for d in app.Documents.VisibleDocuments}
    closed = 0
    for _ in range(5):                                   # assemblies first, then the parts they held
        for d in list(app.Documents):
            try:
                fn = d.FullFileName.lower()
                if fn in before or fn in visible or d.Dirty or d.ReferencingDocuments.Count > 0:
                    continue
                d.Close(True)
                closed += 1
            except Exception:
                pass
    return closed


def find_open(app, path: Path):
    for d in app.Documents:
        if (safe(lambda: d.FullFileName) or "").lower() == str(path).lower():
            return d
    return None


def close_if_open(app, path: Path) -> bool:
    """Close one of our own output files before it is rewritten; refuses unsaved changes.
    Returns True when the document was visible (so the caller can reopen it)."""
    d = find_open(app, path)
    if d is None:
        return False
    if d.Dirty:
        raise SystemExit(f"{path.name} is open in Inventor with unsaved changes - save or close it first")
    was_visible = any((safe(lambda: v.FullFileName) or "").lower() == str(path).lower()
                      for v in app.Documents.VisibleDocuments)
    if d.ReferencingDocuments.Count > 0:
        raise SystemExit(f"{path.name} is used by another open document - close that first")
    d.Close(True)
    return was_visible


# --------------------------------------------------------------------------- placing
def matrix(app, origin_mm, cols=R_IDENT):
    """Inventor matrix from column vectors (images of the part's x, y, z) and a translation in mm."""
    tg = app.TransientGeometry
    m = tg.CreateMatrix()
    c1, c2, c3 = cols
    m.PutMatrixData([c1[0], c2[0], c3[0], origin_mm[0] / 10.0,
                     c1[1], c2[1], c3[1], origin_mm[1] / 10.0,
                     c1[2], c2[2], c3[2], origin_mm[2] / 10.0,
                     0.0, 0.0, 0.0, 1.0])
    return m


def occ_frame(occ):
    """(origin_mm, cols) of an occurrence's transformation (in its assembly's frame)."""
    m = occ.Transformation
    c = [[m.Cell(i, j) for j in (1, 2, 3, 4)] for i in (1, 2, 3)]
    cols = tuple(tuple(c[i][j] for i in range(3)) for j in range(3))
    return (c[0][3] * 10.0, c[1][3] * 10.0, c[2][3] * 10.0), cols


def turn_frame(frame, R, shift=(0.0, 0.0, 0.0)):
    """Apply p -> R p + shift (R given as columns) to an (origin, cols) frame."""
    o, cols = frame

    def rot(v):
        return tuple(R[0][i] * v[0] + R[1][i] * v[1] + R[2][i] * v[2] for i in range(3))
    return tuple(a + b for a, b in zip(rot(o), shift)), tuple(rot(c) for c in cols)


def occurrence_file(occ) -> tuple[str, str]:
    """(path, model state) of an occurrence's document; the state is '' for the default."""
    full = safe(lambda: occ.ReferencedDocumentDescriptor.FullDocumentName) or ""
    if "<" in full and full.endswith(">"):
        path, state = full[:-1].split("<", 1)
        return path, state
    return full, ""


def role_of(occ) -> str | None:
    try:
        if occ.AttributeSets.NameIsUsed(ROLE_SET):
            return occ.AttributeSets.Item(ROLE_SET).Item(ROLE_KEY).Value
    except Exception:
        pass
    return None


def tag(occ, role: str) -> None:
    sets = occ.AttributeSets
    aset = sets.Item(ROLE_SET) if sets.NameIsUsed(ROLE_SET) else sets.Add(ROLE_SET)
    if aset.NameIsUsed(ROLE_KEY):
        aset.Item(ROLE_KEY).Value = role
    else:
        aset.Add(ROLE_KEY, kStringType, role)


class Builder:
    """Places occurrences by explicit transforms under their default names, keyed by a role
    attribute: re-posing keeps an occurrence, swapping its file replaces it."""

    def __init__(self, app, asm_doc):
        self.app = app
        self.doc = asm_doc
        self.cd = asm_doc.ComponentDefinition
        self.existing: dict[str, object] = {}
        self.untagged = 0
        for occ in self.cd.Occurrences:
            r = role_of(occ)
            if r:
                self.existing[r] = occ
            else:
                self.untagged += 1
        self.touched: set[str] = set()
        self.placed: list[dict] = []

    def place(self, role: str, path: Path, origin_mm, cols=R_IDENT, ground=True, reference=False, state: str = ""):
        m = matrix(self.app, origin_mm, cols)
        occ = self.existing.get(role)
        if occ is not None:
            ref = (safe(lambda: occ.ReferencedDocumentDescriptor.FullDocumentName) or "").split("<")[0]
            if ref.lower() != str(path).lower():
                occ.Delete()
                occ = None
                self.existing.pop(role, None)
        if occ is not None:
            occ.Grounded = False
            occ.Transformation = m
        else:
            occ = None
            if state:                                    # the module's own model state of that part
                occ = safe(lambda: self.cd.Occurrences.Add(f"{path}<{state}>", m))
            if occ is None:
                occ = self.cd.Occurrences.Add(str(path), m)
            tag(occ, role)
            self.existing[role] = occ
        if reference:
            try:
                occ.BOMStructure = kReferenceBOMStructure
            except Exception:
                pass
        if ground:
            occ.Grounded = True
        self.touched.add(role)
        self.placed.append({"role": role, "occurrence": safe(lambda: occ.Name, ""), "file": str(path),
                            "reference": bool(reference)})
        return occ

    def prune(self) -> int:
        """Delete tagged occurrences of an earlier build that this build did not place
        (untagged ones - added by hand - stay)."""
        n = 0
        for role, occ in list(self.existing.items()):
            if role not in self.touched:
                try:
                    occ.Delete()
                    n += 1
                except Exception:
                    pass
                self.existing.pop(role, None)
        return n


def mark_purchased(doc, name: str) -> None:
    """A part saved under a BUY name gets the purchased BOM structure, like the CAD-new BUY parts."""
    if name.startswith("BUY - "):
        try:
            doc.ComponentDefinition.BOMStructure = kPurchasedBOMStructure
        except Exception:
            pass


def import_step(app, step: Path, ipt: Path, force: bool = False) -> Path | None:
    """Open a STEP in Inventor and save it as a part. A multi-body STEP comes in as an assembly
    and is kept as .iam. The assembly that references the result must be closed meanwhile."""
    if not step.exists():
        print(f"  [miss]  {step.name} not generated yet")
        return None
    for cand in (ipt, ipt.with_suffix(".iam")):
        if cand.exists() and not force and cand.stat().st_mtime >= step.stat().st_mtime:
            return cand
    doc = app.Documents.Open(str(step), False)
    try:
        if doc.DocumentType == kAssemblyDocumentObject:
            ipt = ipt.with_suffix(".iam")
        for cand in (ipt.with_suffix(".ipt"), ipt.with_suffix(".iam")):
            d = find_open(app, cand)
            if d is not None and d is not doc:
                if d.Dirty or d.ReferencingDocuments.Count > 0:
                    raise SystemExit(f"{cand.name} is open and in use - close it in Inventor first")
                d.Close(True)
            if cand.exists():
                cand.unlink()
        ipt.parent.mkdir(parents=True, exist_ok=True)
        mark_purchased(doc, ipt.stem)
        doc.SaveAs(str(ipt), False)
        print(f"  [import] {step.name} -> {ipt.name}")
    finally:
        doc.Close(True)
    return ipt


def convert_part(app, src: Path, dst: Path) -> Path | None:
    """Open a foreign part (SolidWorks .sldprt, STEP) and save it as an Inventor part."""
    if dst.exists():
        return dst
    if not src.exists():
        print(f"  [miss]  {src}")
        return None
    dst.parent.mkdir(parents=True, exist_ok=True)
    doc = app.Documents.Open(str(src), False)
    try:
        mark_purchased(doc, dst.stem)
        doc.SaveAs(str(dst), False)
        print(f"  [convert] {src.name} -> {dst.name}")
    finally:
        doc.Close(True)
    return dst


def appearance(app, names):
    """First appearance asset of the Autodesk libraries whose display name is in `names`."""
    for want in names:
        try:
            for lib in app.AssetLibraries:
                if "Appearance" not in (safe(lambda: lib.DisplayName) or ""):
                    continue
                for a in lib.AppearanceAssets:
                    if (safe(lambda: a.DisplayName) or "") == want:
                        return a
        except Exception:
            pass
    return None


def set_appearance(doc, occ, asset) -> bool:
    """Give an occurrence a library appearance.  A library asset can be assigned only once per
    document - Inventor copies it in under a new internal name - so later occurrences must take
    the document's copy, which is found by its display name."""
    if asset is None:
        return False
    want = safe(lambda: asset.DisplayName)
    try:
        for a in doc.Assets:
            if safe(lambda: a.DisplayName) == want:
                occ.Appearance = a
                return True
    except Exception:
        pass
    try:
        occ.Appearance = asset
        return True
    except Exception:
        try:
            occ.Appearance = asset.CopyTo(doc)
            return True
        except Exception:
            return False
