"""Make a copy of Robot.SLDASM where both arms are revolute about the shoulder Y-axis,
so the user can swing them (punch) by dragging in SolidWorks. Originals untouched.

Hinge per arm = Concentric(body shoulder Y-axis, arm Y-axis) + Coincident(arm Top, R_Top+-116).
Body axis = Front Plane n R_Rightm129_1 (Y line at x=-129.1, z=0; passes through both shoulders).
Arm axis  = arm Right Plane n arm Front Plane (Y line through the arm origin).
"""
from __future__ import annotations
import asyncio, sys
import numpy as np
sys.path.insert(0, r"D:\AI prodject\robo-boxer\scripts")
import build_joint as bj
from build_joint import mc, t, top_component

SRC = bj.PROJ / "cad" / "body" / "Robot.SLDASM"
DST = bj.PROJ / "cad" / "body" / "Robot_Boxing.SLDASM"
TITLE = "Robot_Boxing"
SHOT = bj.PROJ / "cad" / "body" / "screens"


def doc():
    return mc.typed(bj.model(), "IModelDoc2")


def ext():
    return mc.typed(doc().Extension, "IModelDocExtension")


def sel(name, kind, append=False, mark=None):
    if mark is None:
        mark = 1 if kind == "PLANE" else 0
    ok = ext().SelectByID2(name, kind, 0, 0, 0, append, mark, None, 0)
    if not ok:
        raise RuntimeError(f"cannot select {kind} {name!r}")


def mate_names():
    out = []
    f = F(bj.model().FirstFeature())
    while f is not None:
        if f.GetTypeName2() == "MateGroup":
            s = F(f.GetFirstSubFeature())
            while s is not None:
                out.append(s.Name)
                s = F(s.GetNextSubFeature())
        f = F(f.GetNextFeature())
    return out


def try_mate(pairs, mtype, aligns=(2, 0, 1), flips=(False, True)):
    """Try a mate (one entity pair, or a list of candidate pairs) across alignments;
    keep the first that returns err==0, delete any stray from a failed attempt."""
    asm = mc.typed(bj.model(), "IAssemblyDoc")
    if isinstance(pairs[0], str):
        pairs = [pairs]
    for (e1, k1, e2, k2) in pairs:
        for al in aligns:
            for fl in flips:
                before = set(mate_names())
                doc().ClearSelection2(True)
                try:
                    sel(e1, k1, False, mark=1)
                    sel(e2, k2, True, mark=1)
                except RuntimeError as ex:
                    print(f"   select fail {ex}")
                    break  # bad entity, try next candidate pair
                nsel = mc.typed(doc().SelectionManager, "ISelectionMgr").GetSelectedObjectCount2(-1)
                res = asm.AddMate5(mc.MATE_TYPES[mtype], al, fl, 0, 0, 0, 1.0, 1.0, 0, 0, 0, False, False, 0)
                feat, err = res if isinstance(res, tuple) else (res, None)
                doc().ClearSelection2(True)
                doc().EditRebuild3()
                added = set(mate_names()) - before
                print(f"   try {mtype} {e1}|{e2} nsel={nsel} align={al} flip={fl} -> err={err} feat={feat is not None} added={sorted(added)}")
                if added:  # a mate feature was created; accept it (err=1 on a plane pair is a warning, not a refusal)
                    print(f"   mate {mtype} {e1}|{e2} OK (align={al} flip={fl}) err={err} +{sorted(added)}")
                    return True
    raise RuntimeError(f"mate {mtype} failed for all of {pairs}")


def F(f):
    return mc.typed(f, "IFeature") if f is not None else None


def refaxis_names():
    names = []
    f = F(bj.model().FirstFeature())
    while f is not None:
        if f.GetTypeName2() == "RefAxis":
            names.append(f.Name)
        f = F(f.GetNextFeature())
    return names


def del_mate(name):
    doc().ClearSelection2(True)
    if ext().SelectByID2(name, "MATE", 0, 0, 0, False, 0, None, 0):
        doc().EditDelete()
        print(f"  deleted mate {name}")
    else:
        print(f"  !! mate {name} not found")


def make_axis(p1, p2, label):
    """Reference axis at the intersection of two planes; returns its feature name."""
    before = set(refaxis_names())
    doc().ClearSelection2(True)
    sel(p1, "PLANE", False)
    sel(p2, "PLANE", True)
    ok = bj.model().InsertAxis2(True)
    doc().ClearSelection2(True)
    new = [n for n in refaxis_names() if n not in before]
    if not ok or not new:
        raise RuntimeError(f"InsertAxis2 failed for {label}: ok={ok} new={new}")
    print(f"  axis {label} = {new[-1]!r} (ok={ok})")
    return new[-1]


async def main():
    async with bj.sw_session() as (m, a):
        bj.mcp, bj.adapter = m, a
        SHOT.mkdir(parents=True, exist_ok=True)
        await t("open_model", {"file_path": str(SRC)}, why="open original", n=40)
        await t("save_as", {"file_path": str(DST), "overwrite": True}, why="work on a copy", n=60)
        print("title now:", doc().GetTitle())

        for nm in ("Coincident4", "Coincident5", "Coincident6",
                   "Coincident7", "Coincident8", "Coincident9"):
            del_mate(nm)
        doc().EditRebuild3()

        # Revolute about the shoulder Y-axis built from plane/point mates (axis mates no-op here):
        #   Top coincident        -> locks Y-translation + tilt about X and Z
        #   origin point on R_Right(x=-129.1) -> locks X-translation
        #   origin point on Front (z=0)       -> locks Z-translation
        # left free: rotation about Y = the punch swing.
        for arm, rtop in (("Arm-1", "R_Topp116"), ("Arm-2", "R_Topm116")):
            try_mate((rtop, "PLANE", f"Top Plane@{arm}@{TITLE}", "PLANE"), "coincident")
            pt = [(f"Point1@Origin@{arm}@{TITLE}", "EXTSKETCHPOINT"),
                  (f"Point1@Origin@{arm}@{TITLE}", "DATUMPOINT")]
            for plane, why in (("R_Rightm129_1", "lock X"), ("Front Plane", "lock Z")):
                try:
                    try_mate([(plane, "PLANE", nm, kd) for nm, kd in pt], "coincident")
                    print(f"   {arm} {why}: pinned")
                except RuntimeError as e:
                    print(f"   {arm} {why}: NOT pinned ({str(e)[:50]}) -> arm will also slide")
        doc().EditRebuild3()

        print("  mates now:", mate_names())
        for v in ("isometric", "front", "right"):
            await t("export_image", {"file_path": str(SHOT / f"boxing_{v}.png"), "format_type": "png",
                                     "view_orientation": v}, why=f"check {v}", n=40)
        await t("save_as", {"file_path": str(DST), "overwrite": True}, why="save Robot_Boxing", n=60)
        print("SAVED", DST)


if __name__ == "__main__":
    asyncio.run(main())
