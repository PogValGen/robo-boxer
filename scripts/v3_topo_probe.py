"""Does the SOLIDWORKS Simulation Topology study work on this PC? (2026-11-14, user: "just check the tool")
Opens a COPY of X3_Thigh, fixes the hip holes, presses on the web, meshes, runs the default topology study.
    python v3_topo_probe.py
"""
import shutil
import sys
import time
from pathlib import Path

import pythoncom
import win32com.client
from win32com.client import VARIANT

sys.path.insert(0, r"D:\AI prodject\solidworks-mcp\SolidworksMCP-python\src")
from solidworks_mcp.tools._model_com import typed  # noqa: E402

V3 = Path(r"D:\AI prodject\robo-boxer\cad\v3")
SRC = V3 / "parts" / "X3_Thigh.SLDPRT"
DST = V3 / "topo" / "Topo_Thigh.SLDPRT"
COSW = r"C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\Simulation\cosworks.dll"


NULL = VARIANT(pythoncom.VT_DISPATCH, None)
UNIT_PA = 0


def byref():
    return VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)


def val(o, n):
    v = getattr(o, n)
    if hasattr(v, "_oleobj_") or isinstance(v, (int, float, str, bool, tuple)) or v is None:
        return v                                             # already a value / object (dynamic dispatch property)
    try:
        return v()
    except Exception:  # noqa: BLE001
        return v


def main():
    pythoncom.CoInitialize()
    app = win32com.client.GetActiveObject("SldWorks.Application")
    sw = typed(app, "ISldWorks")
    try:
        app.CloseDoc(DST.name)                               # a copy left open by the previous probe run
    except Exception:  # noqa: BLE001
        pass
    shutil.copyfile(SRC, DST)
    addin = None
    for attempt in range(3):
        r = app.LoadAddIn(COSW)
        print("LoadAddIn ->", r, "(0 ok, 2 already loaded)")
        for pid in ("CosmosWorks.CosmosWorks", "SldWorks.Simulation"):
            try:
                addin = app.GetAddInObject(pid)
            except Exception as e:  # noqa: BLE001
                print("  GetAddInObject", pid, "ERR", str(e)[:80])
            if addin is not None:
                print("add-in object:", pid)
                break
        if addin is not None:
            break
        # after another process used the add-in and exited, it stays half-loaded (-1 / no object): reload it
        print("  reload: UnloadAddIn ->", app.UnloadAddIn(COSW))
        time.sleep(3)
    if addin is None:
        print("RESULT: Simulation add-in object not available")
        return 1
    cw = val(addin, "CosmosWorks")
    res = app.OpenDoc6(str(DST), 1, 1, "", 0, 0)             # early-bound: returns (doc, errors, warnings)
    doc = res[0] if isinstance(res, tuple) else res
    print("opened", DST.name, "->", doc is not None, res[1:] if isinstance(res, tuple) else "")
    part = typed(doc, "IPartDoc")
    faces_fix, face_load = [], None
    for body in part.GetBodies2(0, True) or []:
        for f in typed(body, "IBody2").GetFaces() or []:
            f = typed(f, "IFace2")
            s = typed(f.GetSurface(), "ISurface")
            if s.IsCylinder():
                p = list(s.CylinderParams)
                x, r = p[0] * 1000, p[6] * 1000
                if abs(x) < 0.5 and r < 4.0:
                    faces_fix.append(f)
            elif s.IsPlane():
                p = list(s.PlaneParams)                      # normal(3), root(3)
                box = [v * 1000 for v in f.GetBox()]
                if abs(abs(p[0]) - 1) < 1e-6 and abs(box[0] - 21.0) < 0.01 and abs(box[3] - 21.0) < 0.01:
                    face_load = f
    print("fixture faces:", len(faces_fix), "load face:", face_load is not None)
    cwdoc = val(cw, "ActiveDoc")
    sm = val(cwdoc, "StudyManager")
    err = byref()
    study = sm.CreateNewStudy3("Topo_probe", 13, 0, err)
    print("CreateNewStudy3(topology) ->", "OK" if study is not None else "None", "err", err.value)
    if study is None:
        print("RESULT: topology study was NOT created")
        return 2
    lbc = val(study, "LoadsAndRestraintsManager")
    arr = VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_VARIANT, [f._oleobj_ for f in faces_fix])
    err = byref()
    r = lbc.AddRestraint(0, arr, NULL, err)
    print("AddRestraint(fixed) ->", r is not None, "err", err.value)
    arr = VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_VARIANT, [face_load._oleobj_])
    err = byref()
    p = lbc.AddPressure(0, arr, NULL, err)
    print("AddPressure ->", p is not None, "err", err.value)
    if p is not None:
        val(p, "PressureBeginEdit")
        p.Unit = UNIT_PA
        p.Value = 2.0e5                                      # 0.2 MPa
        print("pressure unit/value read back:", val(p, "Unit"), val(p, "Value"))
        print("pressure EndEdit ->", val(p, "PressureEndEdit"))
    t0 = time.time()
    print("CreateMesh ->", study.CreateMesh(0, 1.5, 0.075), f"{time.time() - t0:.0f} s")
    t0 = time.time()
    rc = val(study, "RunAnalysis")
    print("RunAnalysis ->", rc, f"{time.time() - t0:.0f} s")
    try:
        res = val(study, "Results")
        print("results object:", res is not None)
    except Exception as ex:  # noqa: BLE001
        print("results ERR", str(ex)[:100])
    typed(doc, "IModelDoc2").ShowNamedView2("*Isometric", 7)
    typed(doc, "IModelDoc2").ViewZoomtofit2()
    out = str(V3 / "topo" / "Topo_Thigh_result.png")
    print("image ->", typed(doc, "IModelDoc2").SaveAs3(out, 0, 0))
    print("RESULT: topology study", "RAN" if rc == 0 else f"FAILED rc={rc}")
    print("document left open for viewing: Topo_Thigh (close without saving when done)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
