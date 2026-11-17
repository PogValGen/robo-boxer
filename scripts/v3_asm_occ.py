"""Robot-boxer v3 assembly WITHOUT SolidWorks (2026-11-17): the same layout()/posed() as v3_asm.py, meshes from the
build123d STL set, pairwise interference volumes with manifold3d, pictures with pyvista.

    python v3_asm_occ.py                    -> zero pose: <root>/interference.json, Robot3.glb, screens/Robot3_*.png
    python v3_asm_occ.py pose L_knee=60 ... -> one pose (same report, suffix _pose)

Run with D:\\AI prodject\\_shared\\cadenv\\Scripts\\python.exe. Reading the report: "X3_Pin x cheek" <= 3.2 mm3 is the
D6 press fit (normal), everything else above 0.01 mm3 is a defect (v3_asm.real()).
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np
import trimesh
import manifold3d as m3d

import v3_asm as A
from v3cad_occ import STL, SHOTS, V3

MG_STL = V3 / "stl" / "MG90S.STL"


def mg90s_mesh():
    """the GrabCAD STEP of the MG90S -> one STL (cached)"""
    if not MG_STL.exists():
        from build123d import import_step, export_stl
        export_stl(import_step(str(A.MG90S)), str(MG_STL), tolerance=0.01, angular_tolerance=0.2)
    return MG_STL


_cache: dict = {}


def mesh_of(path):
    p = Path(path)
    stl = mg90s_mesh() if p.stem.startswith("MG90S") else STL / f"{p.stem}.STL"
    if stl not in _cache:
        m = trimesh.load(stl, force="mesh")
        m.merge_vertices()
        _cache[stl] = m
    return _cache[stl]


def placed(item, angles):
    ax, o = A.posed(item, angles) if angles else ([tuple(a) for a in item["axes"]], tuple(item["origin"]))
    T = np.eye(4)
    T[:3, :3] = np.array(ax).T            # columns = images of the component X, Y, Z in the robot frame
    T[:3, 3] = o
    m = mesh_of(item["file"]).copy()
    m.apply_transform(T)
    return m


def to_manifold(m):
    mesh = m3d.Mesh(vert_properties=np.asarray(m.vertices, dtype=np.float32),
                    tri_verts=np.asarray(m.faces, dtype=np.uint32))
    return m3d.Manifold(mesh)


def vol(man):
    v = man.volume
    return v() if callable(v) else v


def run(angles=None, tag=""):
    t0 = time.perf_counter()
    items = A.layout()
    names, meshes, mans = [], [], []
    count: dict = {}
    for it in items:
        stem = Path(it["file"]).stem
        count[stem] = count.get(stem, 0) + 1
        names.append(f"{stem}-{count[stem]}")
        m = placed(it, angles)
        meshes.append(m)
        mans.append(to_manifold(m))
    print(f"  placed {len(items)} components in {time.perf_counter() - t0:.1f} s", flush=True)
    bad_mesh = [names[i] for i, mm in enumerate(mans) if mm.status() != m3d.Error.NoError]
    if bad_mesh:
        print("  WARN not manifold:", bad_mesh, flush=True)
    bbs = [m.bounds for m in meshes]
    pairs = {}
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            a, b = bbs[i], bbs[j]
            if np.any(a[1] < b[0] - 0.01) or np.any(b[1] < a[0] - 0.01):
                continue
            v = vol(mans[i] ^ mans[j])
            if v > 1e-4:
                pairs[tuple(sorted((names[i], names[j])))] = round(float(v), 3)
    # MG90S x horn <= 4.7 mm3: the GrabCAD shaft ends at s 19.9, the MEASURED one at 19.65 (user 2026-11-15) and the
    # horn is built to the measurement -> a 0.25 mm layer of the model shaft (pi * 2.425^2 * 0.25 = 4.6). Checked
    # 2026-11-17: the overlap lies exactly in s 19.65..19.9. Not a defect of the robot.
    bad = {k: v for k, v in A.real(pairs).items() if not ("MG90S" in k and "X3_Horn" in k and v <= 4.7)}
    rep = {"pose": angles or {}, "components": len(items), "pairs": {f"{a} x {b}": v for (a, b), v in pairs.items()},
           "unexpected": bad, "seconds": round(time.perf_counter() - t0, 1)}
    (V3 / f"interference{tag}.json").write_text(json.dumps(rep, indent=1), encoding="utf-8")
    print(f"  interference: {len(pairs)} pairs, unexpected {len(bad)}: {bad}", flush=True)
    scene = trimesh.Scene()
    for n, m in zip(names, meshes):
        scene.add_geometry(m, node_name=n)
    scene.export(str(V3 / f"Robot3{tag}.glb"))
    shots(meshes, names, tag)
    print(f"  done in {time.perf_counter() - t0:.1f} s", flush=True)
    return rep


COLORS = {"MG90S": "#2a5db0", "X3_Lid": "#2a5db0", "X3_Horn": "#f0f0f0", "X3_Battery": "#444444", "X3_Board": "#2f7d3a"}


def shots(meshes, names, tag=""):
    import pyvista as pv
    for view, d in (("isometric", (-1, 1, 1)), ("front", (0, 0, 1)), ("right", (0, -1, 0)), ("back", (0, 0, -1))):
        p = pv.Plotter(off_screen=True, window_size=(900, 1100))
        p.set_background("white")
        for n, m in zip(names, meshes):
            col = next((c for k, c in COLORS.items() if n.startswith(k)), "#c8a040")
            p.add_mesh(pv.wrap(m), color=col, smooth_shading=False)
        c = np.mean([m.bounds.mean(axis=0) for m in meshes], axis=0)
        p.camera.focal_point = c
        p.camera.position = tuple(c + np.array(d) * 600)
        p.camera.up = (-1, 0, 0)
        p.enable_parallel_projection()
        p.reset_camera()
        p.add_text(f"Robot3 {view}{tag}", font_size=10, color="black")
        p.screenshot(str(SHOTS / f"Robot3{tag}_{view}.png"))
        p.close()


if __name__ == "__main__":
    a = sys.argv[1:]
    if a and a[0] == "pose":
        ang = {k: float(v) for k, v in (x.split("=") for x in a[1:])}
        run(ang, "_pose")
    else:
        run()
