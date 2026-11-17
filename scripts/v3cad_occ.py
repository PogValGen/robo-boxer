"""v3 modelling toolkit WITHOUT SolidWorks (2026-11-17): the same API as v3cad.py on build123d / OpenCascade.

    from v3cad_occ import R, C, G, prism, new_part, save, session, PARTS, V3

Prism = shapes extruded along a model axis between the planes axis = c0 and axis = c1, robot frame (x down, y left,
z forward, mm). Shape coordinates in cyclic order: axis x -> (y, z); axis y -> (z, x); axis z -> (x, y).
Overlapping shapes are fused (no "groups" trick needed as in SolidWorks).

Self-checks kept from v3cad (each caught a real bug in SolidWorks): the volume must change in the right direction,
the first boss must equal its profile x depth, the centroid of the added / removed volume must lie inside the prism,
the saved part must be ONE solid. Output: STEP (parts/), STL (stl/), PNG (screens/, pyvista off-screen).
Run with D:\\AI prodject\\_shared\\cadenv\\Scripts\\python.exe (build123d, pyvista).
"""
from __future__ import annotations

import contextlib
import os
import time
from pathlib import Path

from build123d import (Plane, Pos, Rectangle, Circle, Polygon, extrude, export_step, export_stl, CenterOf)

PROJ = Path(__file__).resolve().parents[1]
# output root = cad/v3 (the same stl/ the user prints from). The pilot (2026-11-17) wrote to cad/v3/occ and matched the
# SolidWorks set 28/28; since the corner pins (17.11) the .SLDPRT files are OUTDATED - STEP in parts/ is the source.
V3 = Path(os.environ.get("V3CAD_ROOT", PROJ / "cad" / "v3"))
PARTS, SHOTS, STL = V3 / "parts", V3 / "screens", V3 / "stl"
for _d in (PARTS, SHOTS, STL):
    _d.mkdir(parents=True, exist_ok=True)

OTHER = {"x": (1, 2), "y": (2, 0), "z": (0, 1)}
AX_I = {"x": 0, "y": 1, "z": 2}
UNIT = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
_n = {"feat": 0, "t0": 0.0, "name": "", "part": None}


def R(a0, b0, a1, b1):
    return ("rect", float(a0), float(b0), float(a1), float(b1))


def C(a, b, r):
    return ("circle", float(a), float(b), float(r))


def G(pts):
    return ("poly", [(float(a), float(b)) for a, b in pts])


def _plane(axis, c):
    """sketch plane at axis = c: local X = first shape coordinate, local Y = second, normal = +axis"""
    ia, _ = OTHER[axis]
    o = [0.0, 0.0, 0.0]
    o[AX_I[axis]] = c
    return Plane(origin=tuple(o), x_dir=UNIT[ia], z_dir=UNIT[AX_I[axis]])


def _sketch(s):
    if s[0] == "rect":
        a0, b0, a1, b1 = s[1:]
        return Pos((a0 + a1) / 2, (b0 + b1) / 2) * Rectangle(abs(a1 - a0), abs(b1 - b0))
    if s[0] == "circle":
        return Pos(s[1], s[2]) * Circle(s[3])
    return Polygon(*s[1], align=None)


def _bb(s):
    if s[0] == "rect":
        return min(s[1], s[3]), min(s[2], s[4]), max(s[1], s[3]), max(s[2], s[4])
    if s[0] == "circle":
        return s[1] - s[3], s[2] - s[3], s[1] + s[3], s[2] + s[3]
    a = [p[0] for p in s[1]]
    b = [p[1] for p in s[1]]
    return min(a), min(b), max(a), max(b)


def _mass(p):
    if p is None:
        return 0.0, [0.0, 0.0, 0.0]
    c = p.center(CenterOf.MASS)
    return p.volume, [c.X, c.Y, c.Z]


async def prism(axis, c0, c1, shapes, cut=False, why="", expect=None):
    c0, c1 = sorted((float(c0), float(c1)))
    depth = c1 - c0
    assert depth > 1e-6, (why, c0, c1)
    pl = _plane(axis, c0)
    tool = None
    for s in shapes:
        # dir explicit: build123d extrudes along the FACE normal, and a clockwise polygon has it reversed
        # (2026-11-17: the torso shoulder rail went to -x; caught by the centroid check)
        sol = extrude(pl * _sketch(s), amount=depth, dir=pl.z_dir)
        tool = sol if tool is None else tool + sol
    full = tool.volume
    part = _n["part"]
    v0, m0 = _mass(part)
    if cut:
        assert part is not None, f"{why}: cut on an empty part"
        part = part - tool
    else:
        part = tool if part is None else part + tool
    _n["part"] = part
    _n["feat"] += 1
    v1, m1 = _mass(part)
    dv = v1 - v0
    ok = (dv < -1e-3) if cut else (dv > 1e-3)
    if cut and expect is not None:
        ok = ok and -dv > 0.5 * expect
    if not cut and _n["feat"] == 1:
        ok = ok and abs(dv - full) < 0.01 * full + 0.5
    where = ""
    if abs(dv) > 1.0:
        cen = [(v1 * m1[i] - v0 * m0[i]) / dv for i in range(3)]
        tol = 0.1 + 0.002 * max(v0, v1) / abs(dv)
        bbs = [_bb(s) for s in shapes]
        lim = {AX_I[axis]: (c0, c1), OTHER[axis][0]: (min(b[0] for b in bbs), max(b[2] for b in bbs)),
               OTHER[axis][1]: (min(b[1] for b in bbs), max(b[3] for b in bbs))}
        out = [f"{'xyz'[i]} {cen[i]:.2f} not in {lim[i][0]:.2f}..{lim[i][1]:.2f}" for i in range(3)
               if not lim[i][0] - tol <= cen[i] <= lim[i][1] + tol]
        if out:
            where = "   <-- WRONG PLACE: centroid " + ", ".join(out)
    print(f"  {'cut ' if cut else 'boss'} {axis} {c0:8.2f}..{c1:8.2f}  dV {dv:+10.1f} (profile {full:9.1f})  {why}"
          f"{where if where else ('' if ok else '   <-- NOTHING CHANGED')}", flush=True)
    assert not where, f"{why}: {where}"
    assert ok, f"{why}: volume change {dv:.2f}"
    return dv


async def new_part(name):
    _n.update(feat=0, t0=time.perf_counter(), name=name, part=None)
    print(f"== {name}", flush=True)


# isometric from (-1, 1, 1): -x = robot UP, look from above-left-front
VIEWS = {"isometric": (-1, 1, 1), "front": (0, 0, 1), "top": (-1, 0, 0), "right": (0, 1, 0), "back": (0, 0, -1)}


def shot(stl_path, png_path, view="isometric"):
    """off-screen picture of an STL in the robot frame: screen up = -x (robot up), view from the given direction"""
    import pyvista as pv
    mesh = pv.read(str(stl_path))
    p = pv.Plotter(off_screen=True, window_size=(900, 700))
    p.set_background("white")
    p.add_mesh(mesh, color="#c8a040", show_edges=False, smooth_shading=False)
    p.add_mesh(mesh.extract_feature_edges(30), color="black", line_width=1)
    d = VIEWS[view]
    c = mesh.center
    r = mesh.length * 1.6
    p.camera.focal_point = c
    p.camera.position = (c[0] + d[0] * r, c[1] + d[1] * r, c[2] + d[2] * r)
    p.camera.up = (-1, 0, 0) if view != "top" else (0, 0, 1)
    p.enable_parallel_projection()
    p.reset_camera()
    p.add_text(f"{Path(stl_path).stem}  {view}", font_size=10, color="black")
    p.screenshot(str(png_path))
    p.close()


async def save(path, material="PLA", shots=("isometric",), stl=True):
    part = _n["part"]
    name = Path(path).stem
    nb = len(part.solids())
    assert nb == 1, f"{name}: {nb} solid bodies - a piece of the part is not attached"
    bb = part.bounding_box()
    lo, hi = [bb.min.X, bb.min.Y, bb.min.Z], [bb.max.X, bb.max.Y, bb.max.Z]
    export_step(part, str(PARTS / f"{name}.step"))
    stl_path = STL / f"{name}.STL"
    export_stl(part, str(stl_path), tolerance=0.005, angular_tolerance=0.1)
    # export_stl fails SILENTLY on a non-ASCII path (2026-11-17, scratchpad under C:\Users\Валерий): check the file
    assert stl_path.exists() and stl_path.stat().st_mtime > _n["t0"] - 1 + time.time() - time.perf_counter(), \
        f"STL not written: {stl_path}"
    if shots:
        for v in shots:
            shot(stl_path, SHOTS / f"{name}_{v}.png", v)
    vol = part.volume
    print(f"   saved {name}: bbox {[round(a, 2) for a in lo]} .. {[round(a, 2) for a in hi]}, "
          f"V {vol:.0f} mm3 = {vol * 1.24e-3:.1f} g solid PLA, {_n['feat']} features, "
          f"{time.perf_counter() - _n['t0']:.1f} s", flush=True)
    return {"min": lo, "max": hi, "vol_mm3": vol}


@contextlib.asynccontextmanager
async def session(recalib=False):
    yield None, None
