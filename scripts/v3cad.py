"""v3 modelling toolkit (2026-11-14): axis-aligned prisms in MODEL coordinates through the solidworks-mcp tools.

Every feature = sketch on one of the THREE BASE PLANES + mid-plane boss / cut whose start condition is
"Offset from the sketch plane" (`start_offset`, added to the MCP create_extrusion / create_cut_extrude). No reference
plane per feature (the first version made one per feature - the user: "why so many planes?"), no extrusion-direction
guessing. Parts are modelled directly in the ROBOT frame (x down, y left, z forward, mm) at the zero pose.

    async with session():
        await new_part("X3_Block")
        await prism("y", 6.8, 41.2, [R(-8, -17, 8, 17)])                 # boss: box
        await prism("y", 10, 38, [R(-6.5, -11.6, 6.5, 11.6)], cut=True)  # pocket
        await save(path)

Shapes are given in the two model coordinates other than the prism axis, in cyclic order:
    axis x -> (y, z);  axis y -> (z, x);  axis z -> (x, y)
R(a0, b0, a1, b1) rectangle, C(a, b, r) circle, G([(a, b), ...]) closed polygon.

Self-checks (each one caught a real bug on 2026-11-14):
  * the sketch is drawn with SketchManager.AddToDB (no inference snapping: a 1..3 rectangle became 0..3);
  * calib() measures the sketch (u, v) <-> model mapping and the sign of the start offset per base plane, for boss and
    for cut, on scratch parts (cached in cad/v3/v3cad_calib.json);
  * after EVERY feature the centroid of the added / removed volume must lie inside the requested prism
    (catches mirrored sketches and a feature on the wrong side of the plane).
"""
from __future__ import annotations

import contextlib
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import build_joint as base  # noqa: E402
from build_joint import mc, sw_session, open_docs  # noqa: E402

PROJ = base.PROJ
V3 = PROJ / "cad" / "v3"
PARTS, SHOTS, STL = V3 / "parts", V3 / "screens", V3 / "stl"
for _d in (PARTS, SHOTS, STL):
    _d.mkdir(parents=True, exist_ok=True)
CALIB = V3 / "v3cad_calib.json"
CALIB_VER = 2
BASE_PLANE = {"x": "Right", "y": "Top", "z": "Front"}
OTHER = {"x": (1, 2), "y": (2, 0), "z": (0, 1)}          # model axes (a, b) of the shape coordinates
AX_I = {"x": 0, "y": 1, "z": 2}
VERBOSE = False
_calib: dict = {}
_n = {"feat": 0, "t0": 0.0, "name": "", "mass": (0.0, [0.0, 0.0, 0.0])}


def R(a0, b0, a1, b1):
    return ("rect", float(a0), float(b0), float(a1), float(b1))


def C(a, b, r):
    return ("circle", float(a), float(b), float(r))


def G(pts):
    return ("poly", [(float(a), float(b)) for a, b in pts])


async def call(name, args=None, wrap=True):
    payload = {"input_data": args} if (wrap and args is not None) else (args or {})
    r = base._unwrap(await base.mcp.call_tool(name, payload))
    st = r.get("status") if isinstance(r, dict) else "?"
    if VERBOSE or st not in ("success", "partial_success"):
        print(f"   {name} [{st}] {json.dumps(r, ensure_ascii=False, default=str)[:300]}")
    if st not in ("success", "partial_success"):
        raise RuntimeError(f"{name} failed: {str(r)[:400]}")
    return r if isinstance(r, dict) else {"raw": r}


def mass():
    """(volume mm3, centre of mass mm) of the active part"""
    m = mc.mass_summary(base.model()) or {}
    return float(m.get("volume_mm3", 0.0)), [float(v) for v in (m.get("center_of_mass_mm") or [0, 0, 0])]


def vol():
    return base.vol()


def bbox():
    return base.box()


def _snap(on):
    """sketch inference off while drawing: entities near the origin / other geometry snapped to it (1..3 -> 0..3)"""
    try:
        base.model().SketchManager.AddToDB = not on
    except Exception as e:                                   # noqa: BLE001
        print("   WARN AddToDB:", e, flush=True)


def _uv(axis, a, b):
    """model shape coordinates (a, b) -> sketch (u, v) on the base plane of this axis."""
    m = _calib.get(axis) or {"swap": False, "su": 1, "sv": 1}
    u, v = (b, a) if m["swap"] else (a, b)
    return m["su"] * u, m["sv"] * v


def _draw(axis, shapes):
    out = []
    for s in shapes:
        if s[0] == "rect":
            (u0, v0), (u1, v1) = _uv(axis, s[1], s[2]), _uv(axis, s[3], s[4])
            out.append(("add_rectangle", {"x1": min(u0, u1), "y1": min(v0, v1), "x2": max(u0, u1), "y2": max(v0, v1)}))
        elif s[0] == "circle":
            u, v = _uv(axis, s[1], s[2])
            out.append(("add_circle", {"center_x": u, "center_y": v, "radius": s[3]}))
        else:
            p = [_uv(axis, a, b) for a, b in s[1]]
            for (u0, v0), (u1, v1) in zip(p, p[1:] + p[:1]):
                out.append(("add_line", {"x1": u0, "y1": v0, "x2": u1, "y2": v1}))
    return out


def _area(shapes):
    a = 0.0
    for s in shapes:
        if s[0] == "rect":
            a += abs(s[3] - s[1]) * abs(s[4] - s[2])
        elif s[0] == "circle":
            a += 3.14159265 * s[3] ** 2
        else:
            p = s[1]
            a += abs(sum(p[i][0] * p[(i + 1) % len(p)][1] - p[(i + 1) % len(p)][0] * p[i][1] for i in range(len(p)))) / 2
    return a


def _bb(s):
    if s[0] == "rect":
        return min(s[1], s[3]), min(s[2], s[4]), max(s[1], s[3]), max(s[2], s[4])
    if s[0] == "circle":
        return s[1] - s[3], s[2] - s[3], s[1] + s[3], s[2] + s[3]
    return (min(p[0] for p in s[1]), min(p[1] for p in s[1]), max(p[0] for p in s[1]), max(p[1] for p in s[1]))


def _groups(shapes):
    """split into sketches whose contours do not touch each other (SW refuses intersecting contours in one sketch)"""
    out = []
    for s in shapes:
        b = _bb(s)
        for g in out:
            if all(b[2] < q[0] - 0.01 or q[2] < b[0] - 0.01 or b[3] < q[1] - 0.01 or q[3] < b[1] - 0.01
                   for q in map(_bb, g)):
                g.append(s)
                break
        else:
            out.append([s])
    return out


async def prism(axis, c0, c1, shapes, cut=False, why="", expect=None):
    """Boss (or cut) of `shapes` between the planes axis = c0 and axis = c1. Returns the volume change (mm3).
    Overlapping shapes go into separate features (one sketch each group); the check is on the total change."""
    gr = _groups(shapes)
    if len(gr) > 1:
        dv = 0.0
        for i, g in enumerate(gr):
            dv += await _prism(axis, c0, c1, g, cut, f"{why} [{i + 1}/{len(gr)}]", None, strict=(i == 0))
        assert (dv < -1e-3) if cut else (dv > 1e-3), f"{why}: volume change {dv:.2f}"
        return dv
    return await _prism(axis, c0, c1, shapes, cut, why, expect)


async def _feature(axis, mid, depth, shapes, cut, off_sign=None):
    """one sketch on the base plane + mid-plane boss / cut starting `mid` away from it"""
    sk = base.find_name(await call("create_sketch", {"plane": BASE_PLANE[axis]}), "Sketch") or "Sketch"
    no_snap = all(sh[0] != "poly" for sh in shapes)      # polygon corners must merge: keep the inference for them
    if no_snap:
        _snap(False)
    try:
        for kind, args in _draw(axis, shapes):
            await call(kind, args)
    finally:
        if no_snap:
            _snap(True)
    await call("exit_sketch", None, wrap=False)
    if off_sign is None:
        off_sign = (_calib.get(axis) or {}).get("cut_off" if cut else "boss_off", 1)
    off = round(off_sign * mid, 6)
    if cut:
        await call("create_cut_extrude", {"depth": depth, "end_condition": "MidPlane", "start_offset": off})
    else:
        await call("create_extrusion", {"sketch_name": sk, "depth": depth, "both_directions": True, "start_offset": off})


async def _prism(axis, c0, c1, shapes, cut=False, why="", expect=None, strict=True):
    c0, c1 = sorted((float(c0), float(c1)))
    if cut and abs(c0 + c1) < 1e-6:
        c0 -= 0.02          # a MidPlane cut that starts ON the base plane fails (FeatureCut returns nothing)
    mid, depth = (c0 + c1) / 2, c1 - c0
    assert depth > 1e-6, (why, c0, c1)
    v0, m0 = _n["mass"] if _n["feat"] else (0.0, [0.0, 0.0, 0.0])
    await _feature(axis, mid, depth, shapes, cut)
    _n["feat"] += 1
    v1, m1 = mass()
    _n["mass"] = (v1, m1)
    dv = v1 - v0
    full = _area(shapes) * depth
    ok = (dv < -1e-3) if cut else (dv > 1e-3)
    if cut and expect is not None:
        ok = ok and -dv > 0.5 * expect
    if not cut and _n["feat"] == 1 and len(shapes) == 1:
        ok = ok and abs(dv - full) < 0.01 * full + 0.5       # first boss must equal its profile (snap / mapping guard)
    where = ""
    if abs(dv) > 1.0:
        # centroid of the added / removed volume must be inside the requested prism
        cen = [(v1 * m1[i] - v0 * m0[i]) / dv for i in range(3)]
        tol = 0.1 + 0.002 * max(v0, v1) / abs(dv)
        bbs = [_bb(s) for s in shapes]
        lim = {AX_I[axis]: (c0, c1), OTHER[axis][0]: (min(b[0] for b in bbs), max(b[2] for b in bbs)),
               OTHER[axis][1]: (min(b[1] for b in bbs), max(b[3] for b in bbs))}
        out = [f"{'xyz'[i]} {cen[i]:.2f} not in {lim[i][0]:.2f}..{lim[i][1]:.2f}" for i in range(3)
               if not lim[i][0] - tol <= cen[i] <= lim[i][1] + tol]
        if out:
            ok = False
            where = "   <-- WRONG PLACE: centroid " + ", ".join(out)
    print(f"  {'cut ' if cut else 'boss'} {axis} {c0:8.2f}..{c1:8.2f}  dV {dv:+10.1f} (profile {full:9.1f})  {why}"
          f"{where if where else ('' if ok else '   <-- NOTHING CHANGED')}", flush=True)
    assert not where, f"{why}: {where}"
    assert ok or not strict, f"{why}: volume change {dv:.2f}"
    return dv


async def close():
    """close the active document without saving; the MCP close_model sometimes fails once ('str' is not callable)"""
    for i in range(3):
        try:
            return await call("close_model", {"save": False})
        except RuntimeError as e:
            print(f"   WARN close_model retry {i + 1}: {str(e)[:80]}", flush=True)
            time.sleep(1.0)
    raise RuntimeError("close_model failed 3 times")


async def new_part(name):
    _n.update(feat=0, t0=time.perf_counter(), name=name, mass=(0.0, [0.0, 0.0, 0.0]))
    await call("create_part", {"name": name})
    print(f"== {name}", flush=True)


async def shot(path, view="isometric"):
    await call("export_image", {"file_path": str(path), "format_type": "png", "view_orientation": view})


async def save(path, material="ABS", shots=("isometric",), stl=True):
    await call("set_material", {"material": material})
    nb = mc.geometry_report(base.model(), "bounding_box").get("body_count")
    assert nb == 1, f"{Path(path).stem}: {nb} solid bodies - a piece of the part is not attached"
    lo, hi = bbox()
    await call("save_as", {"file_path": str(path), "overwrite": True})
    if stl:
        await call("export_stl", {"file_path": str(STL / (Path(path).stem + ".STL")), "format_type": "stl"})
    for v in shots:
        await shot(SHOTS / f"{Path(path).stem}_{v}.png", v)
    print(f"   saved {Path(path).name}: bbox {[round(a, 2) for a in lo]} .. {[round(a, 2) for a in hi]}, "
          f"V {vol():.0f} mm3 = {vol() * 1.24e-3 * 1.0:.1f} g solid PLA, {_n['feat']} features, "
          f"{time.perf_counter() - _n['t0']:.0f} s", flush=True)
    info = {"min": lo, "max": hi, "vol_mm3": vol()}
    await close()
    return info


async def calib():
    """Per base plane: the sketch (u, v) mapping and the sign of the start offset for a boss and for a cut."""
    global _calib
    _calib = {}
    res = {"ver": CALIB_VER}
    for axis in "xyz":
        ia, ib = OTHER[axis]
        k = AX_I[axis]
        # ---- boss: rectangle u 30..32, v 50..60 drawn with the identity mapping, started +6 from the base plane
        await new_part("calib")
        _calib[axis] = {"swap": False, "su": 1, "sv": 1}
        await _feature(axis, 6.0, 2.0, [R(30, 50, 32, 60)], cut=False, off_sign=1)
        assert abs(vol() - 40.0) < 0.1, (axis, "calib boss volume", vol())
        lo, hi = bbox()
        assert abs(abs(lo[k] + hi[k]) / 2 - 6.0) < 0.01 and abs(hi[k] - lo[k] - 2.0) < 0.01, (axis, "offset", lo, hi)
        boss_off = 1 if lo[k] > 0 else -1
        ea, eb = (lo[ia], hi[ia]), (lo[ib], hi[ib])
        swap = abs((ea[1] - ea[0]) - 10.0) < 0.01          # a got the long side -> a = v
        eu, ev = (eb, ea) if swap else (ea, eb)
        m = {"swap": swap, "su": 1 if eu[0] > 0 else -1, "sv": 1 if ev[0] > 0 else -1, "boss_off": boss_off}
        await close()
        # ---- cut: block +-40 x +-40 x +-20, a slab cut 9..21 started +15 from the base plane trims one end
        await new_part("calib")
        await _feature(axis, 0.0, 40.0, [R(-40, -40, 40, 40)], cut=False, off_sign=1)
        await _feature(axis, 15.0, 12.0, [R(-50, -50, 50, 50)], cut=True, off_sign=1)
        lo, hi = bbox()
        assert abs(hi[k] - lo[k] - 29.0) < 0.01, (axis, "calib cut", lo, hi)
        m["cut_off"] = 1 if abs(hi[k] - 9.0) < 0.01 else -1
        await close()
        res[axis] = m
        print(f"   calib {axis}: {m}", flush=True)
    _calib = res
    CALIB.write_text(json.dumps(res, indent=1))
    return res


@contextlib.asynccontextmanager
async def session(recalib=False):
    global _calib
    async with sw_session() as (m, a):
        base.mcp, base.adapter = m, a
        print("open docs at start:", open_docs(a), flush=True)
        old = json.loads(CALIB.read_text()) if CALIB.exists() else {}
        if old.get("ver") == CALIB_VER and not recalib:
            _calib = old
        else:
            await calib()
        yield m, a


async def selftest():
    """12 combinations axis x side x (first / second feature) for boss, 6 for cut: bbox against the expectation."""
    bad = []
    for axis in "xyz":
        k, (ia, ib) = AX_I[axis], OTHER[axis]
        for sgn in (1, -1):
            c0, c1 = (10, 12) if sgn > 0 else (-12, -10)
            for second in (False, True):
                name = f"{axis}{'+' if sgn > 0 else '-'}{'_2nd' if second else '_1st'}"
                await new_part(name)
                e_lo, e_hi = [1e9] * 3, [-1e9] * 3

                def acc(ax, p0, p1, a0, b0, a1, b1):
                    for i, lo, hi in ((AX_I[ax], p0, p1), (OTHER[ax][0], a0, a1), (OTHER[ax][1], b0, b1)):
                        e_lo[i], e_hi[i] = min(e_lo[i], lo), max(e_hi[i], hi)
                if second:
                    o = "y" if axis != "y" else "z"
                    await prism(o, 40, 42, [R(-2, -3, 2, 3)], why="base")
                    acc(o, 40, 42, -2, -3, 2, 3)
                await prism(axis, c0, c1, [R(-24, 10, -8, 20)], why=name)
                acc(axis, c0, c1, -24, 10, -8, 20)
                lo, hi = bbox()
                ok = all(abs(lo[i] - e_lo[i]) < 0.01 and abs(hi[i] - e_hi[i]) < 0.01 for i in range(3))
                print(f"   {name}: {'OK' if ok else 'MISMATCH'} {[round(v, 2) for v in lo]} {[round(v, 2) for v in hi]}", flush=True)
                if not ok:
                    bad.append(name)
                await close()
            # cut: block, then a blind pocket on the `sgn` side and a through slot across the base plane
            name = f"{axis}{'+' if sgn > 0 else '-'}_cut"
            await new_part(name)
            await prism(axis, -20, 20, [R(-30, -30, 30, 30)], why="block")
            dv = await prism(axis, c0, c1, [R(-24, 10, -8, 20)], cut=True, why=name)       # centroid check inside
            dv0 = await prism(axis, -3, 3, [R(5, -25, 9, -15)], cut=True, why=name + " across the base plane")
            ok = abs(dv + 320.0) < 0.5 and abs(dv0 + 240.0 + 0.8) < 1.0
            print(f"   {name}: {'OK' if ok else 'MISMATCH'} dV {dv:.1f} / {dv0:.1f}", flush=True)
            if not ok:
                bad.append(name)
            await close()
    print("SELFTEST BAD:", bad, flush=True)
    return bad


if __name__ == "__main__":
    import asyncio

    async def _test():
        async with session(recalib=True):
            await selftest()

    asyncio.run(_test())
