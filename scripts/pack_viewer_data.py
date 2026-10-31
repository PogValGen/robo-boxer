"""Pack viewer data (make_viewer.py .json) into a SMALL blob to inline in an artifact page.

Session limitation (2026-10-31): a session started from C:\\WINDOWS\\system32 cannot attach
local files to an artifact, so the model must travel inside the page text. To keep it small:
welded + decimated meshes, vertices quantized to int16 over the part box, uint16 indices,
instances rounded; everything in one gzip, base64.

Usage: python pack_viewer_data.py <robot_3d.json> <out.b64> [budget_triangles_per_part]
"""

from __future__ import annotations

import base64
import gzip
import json
import struct
import sys

import fast_simplification
import numpy as np


def stl_tris(raw: bytes) -> np.ndarray:
    n = struct.unpack("<I", raw[80:84])[0]
    rec = np.frombuffer(raw[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rec["v"].reshape(-1, 3, 3).astype(np.float64)


def weld(tris: np.ndarray, tol: float = 1e-3):
    pts = tris.reshape(-1, 3)
    key = np.round(pts / tol).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    verts = np.zeros((len(uniq), 3))
    verts[inv.ravel()] = pts
    faces = inv.reshape(-1, 3)
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])]
    return verts, faces


def box(x0, y0, z0, x1, y1, z1):
    v = np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)], float)
    f = [[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]]
    return v, np.array(f)


def cyl(axis: int, c, r: float, a0: float, a1: float, n: int = 16, r_in: float = 0.0):
    """Cylinder (or tube when r_in > 0) along axis from a0 to a1, centre c (2 coords of the other axes)."""
    o = [i for i in range(3) if i != axis]
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    rings = []
    for a in (a0, a1):
        for rr in ((r, r_in) if r_in else (r,)):
            pts = np.zeros((n, 3)); pts[:, axis] = a
            pts[:, o[0]] = c[0] + rr * np.cos(ang); pts[:, o[1]] = c[1] + rr * np.sin(ang)
            rings.append(pts)
    v = np.vstack(rings)
    f = []
    def band(i0, i1):
        for k in range(n):
            a, b = i0 + k, i0 + (k + 1) % n
            f.extend([[a, b, i1 + (k + 1) % n], [a, i1 + (k + 1) % n, i1 + k]])
    if r_in:
        band(0, 2 * n); band(n, 3 * n); band(0, n); band(2 * n, 3 * n)
    else:
        band(0, n)
        cv = len(v); v = np.vstack([v, [[*(np.r_[a0, c[0], c[1]] if axis == 0 else [0, 0, 0])]]])  # placeholder
        v = v[:-1]
        for base in (0, n):  # caps as fans
            cc = np.zeros(3); cc[axis] = a0 if base == 0 else a1; cc[o[0]], cc[o[1]] = c
            v = np.vstack([v, cc]); ci = len(v) - 1
            for k in range(n):
                f.append([ci, base + k, base + (k + 1) % n])
    return v, np.array(f)


def merge(*meshes):
    vs, fs, off = [], [], 0
    for v, f in meshes:
        vs.append(v); fs.append(f + off); off += len(v)
    return np.vstack(vs), np.vstack(fs)


def proxy(stem: str, lo, hi):
    """Simple stand-ins for purchased parts (same box): MG90S, bearings (axis = part x)."""
    if "MG90S_grabcad" in stem:  # GrabCAD frame: shaft Y at x=5.5; body y -11.9..11.89, tabs 5.15..7.65 (holes x=+-13.75)
        teeth = 20  # splined shaft: star prism
        ang = np.linspace(0, 2 * np.pi, 2 * teeth, endpoint=False)
        rr = np.where(np.arange(2 * teeth) % 2 == 0, 2.42, 2.15)
        ring = lambda y: np.c_[5.5 + rr * np.cos(ang), np.full(2 * teeth, y), rr * np.sin(ang)]
        sv = np.vstack([ring(16.03), ring(19.9), [[5.5, 16.03, 0]], [[5.5, 19.9, 0]]])
        n = 2 * teeth; sf = []
        for k in range(n):
            a, b = k, (k + 1) % n
            sf += [[a, b, n + b], [a, n + b, n + a], [2 * n, b, a], [2 * n + 1, n + a, n + b]]
        tab = lambda x0, x1: box(x0, 5.15, -6.1, x1, 7.65, 6.1)
        return merge(box(-11.4, -11.9, -6.1, 11.4, 11.89, 6.1), tab(-16.1, -15.0), tab(-12.5, -11.4), box(-15.0, 5.15, -6.1, -12.5, 7.65, -1.0),
                     box(-15.0, 5.15, 1.0, -12.5, 7.65, 6.1), tab(11.4, 12.5), tab(15.0, 16.1), box(12.5, 5.15, -6.1, 15.0, 7.65, -1.0),
                     box(12.5, 5.15, 1.0, 15.0, 7.65, 6.1), cyl(1, (5.5, 0), 6.0, 11.89, 16.03, 28), cyl(1, (-1.5, 0), 3.0, 11.89, 15.0, 20),
                     (sv, np.array(sf)))
    if "Bearing" in stem:  # exact-size ring (outer D, bore ~D/2.2), axis = part x; balls are hidden in the cup anyway
        r = (hi[1] - lo[1]) / 2
        return cyl(0, (0, 0), r, lo[0], hi[0], 28, r_in=r * 0.46)
    return None


PROXY = True  # MG90S and bearings as exact-size stand-ins (GrabCAD splines / Toolbox balls cost ~8k / 2k triangles)


def main(src: str, out: str, budget: int = 700) -> None:
    d = json.load(open(src, encoding="utf-8"))
    parts, blobs = [], []
    keys = list(d["meshes"])
    for k in keys:
        m = d["meshes"][k]
        tris = stl_tris(gzip.decompress(base64.b64decode(m["gz"]))) + np.array(m["off"])
        v, f = weld(tris)
        px = proxy(m["stem"], tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)) if PROXY else None
        tgt = budget
        if px is not None:
            v, f = px
        elif len(f) > 4000:  # only heavy purchased meshes (MG90S splines, bearing balls); printed parts stay exact
            want = 3000 if "MG90S" in m["stem"] else 300
            v, f = fast_simplification.simplify(v.astype(np.float32), f.astype(np.int32), target_reduction=1 - want / len(f), agg=4)
        lo, hi = v.min(0), v.max(0)
        sc = np.maximum(hi - lo, 1e-6) / 65535.0
        q = np.round((v - lo) / sc).astype(np.uint16)
        assert len(v) < 65536
        parts.append({"stem": m["stem"], "color": m["color"], "lo": [round(float(x), 3) for x in lo],
                      "sc": [float(f"{x:.6g}") for x in sc], "nv": int(len(v)), "nf": int(len(f))})
        blobs.append(q.tobytes() + f.astype(np.uint16).tobytes())
        print(f"  {m['stem']}: {len(tris)} -> {len(f)} tris, {len(v)} verts")
    idx = {k: i for i, k in enumerate(keys)}
    inst = [[idx[i["p"]]] + [round(x, 4) for x in i["m"][:9]] + [round(x * 1000, 2) for x in i["m"][9:12]] for i in d["inst"]]
    head = json.dumps({"title": d["title"], "parts": parts, "inst": inst}, separators=(",", ":")).encode()
    payload = struct.pack("<I", len(head)) + head + b"".join(blobs)
    b64 = base64.b64encode(gzip.compress(payload, 9)).decode()
    open(out, "w").write(b64)
    print(f"payload {len(payload) / 1024:.0f} KB -> gzip+b64 {len(b64) / 1024:.0f} KB")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], int(sys.argv[3]) if len(sys.argv) > 3 else 700)
