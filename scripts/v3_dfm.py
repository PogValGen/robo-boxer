"""FDM printability check of the v3 parts by their STL (2026-11-14, user: "all parts must be adapted to printing").

For the print orientation of every part (table PRINT in v3_parts: the model direction that points UP from the bed):
  * base      - area lying on the bed (mm2) and its share of the footprint;
  * overhangs - downward faces flatter than 45 deg that are NOT on the bed, grouped by height: area and the size of
                the group in the two horizontal directions. The smaller size is the bridge span if the ceiling is
                held on both sides; a ceiling held on one side only is a cantilever and needs a redesign;
  * size      - the box in the print frame against the 125 mm bed of the Snapmaker Original.

    python v3_dfm.py [part ...]        -> table + cad/v3/dfm_report.json   (no SolidWorks needed)
"""
from __future__ import annotations

import json
import struct
import sys
from pathlib import Path

import numpy as np

V3 = Path(__file__).resolve().parents[1] / "cad" / "v3"
STL = V3 / "stl"
BED = 120.0
MAX_BRIDGE = 15.0
AX = {"x": 0, "y": 1, "z": 2}


def read_stl(path):
    b = path.read_bytes()
    n = struct.unpack("<I", b[80:84])[0]
    if 84 + 50 * n == len(b):
        a = np.frombuffer(b, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n, offset=84)
        return a["v"].astype(float)
    tri, cur = [], []
    for line in b.decode("ascii", "ignore").splitlines():
        p = line.split()
        if p[:1] == ["vertex"]:
            cur.append([float(v) for v in p[1:4]])
            if len(cur) == 3:
                tri.append(cur)
                cur = []
    return np.array(tri, dtype=float)


def check(name, up, tris=None):
    tris = read_stl(STL / f"X3_{name}.STL") if tris is None else tris
    k, sgn = AX[up[1]], (1.0 if up[0] == "+" else -1.0)
    h = sgn * tris[:, :, k]
    h = h - h.min()
    e1, e2 = tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    cr = np.cross(e1, e2)
    area = np.linalg.norm(cr, axis=1) / 2
    nz = sgn * cr[:, k] / np.maximum(2 * area, 1e-12)            # normal component along "up"
    hc = h.mean(axis=1)
    hor = [i for i in range(3) if i != k]
    down = nz < -0.707
    base = down & (hc < 0.05)
    over = down & (hc >= 0.05) & (area > 1e-6)
    ext = tris.reshape(-1, 3)
    size = [float(np.ptp(ext[:, i])) for i in hor] + [float(h.max())]
    foot = size[0] * size[1]
    # connected patches of overhanging facets (shared vertices): each patch is one ceiling / one flank
    idx = np.nonzero(over)[0]
    parent = {int(i): int(i) for i in idx}

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    seen = {}
    for i in idx:
        for v in tris[i]:
            key = tuple(np.round(v, 2))
            j = seen.setdefault(key, int(i))
            ra, rb = find(int(i)), find(j)
            if ra != rb:
                parent[ra] = rb
    comp = {}
    for i in idx:
        comp.setdefault(find(int(i)), []).append(int(i))
    groups = []
    for members in comp.values():
        pts = tris[members].reshape(-1, 3)
        d = [float(np.ptp(pts[:, i])) for i in hor]
        a = float(area[members].sum())
        if a < 0.5:
            continue
        groups.append({"h": round(float(hc[members].mean()), 1), "area": round(a, 1),
                       "size": [round(d[0], 1), round(d[1], 1)], "slope45": bool(nz[members].max() > -0.75),
                       "at": [round(float(pts[:, i].mean()), 1) for i in range(3)]})
    groups.sort(key=lambda g: g["h"])
    worst = max((min(g["size"]) for g in groups if not g["slope45"]), default=0.0)    # 45 deg flanks print unsupported
    return {"part": name, "up": up, "size": [round(v, 1) for v in size], "fits_bed": max(size[:2]) <= BED and size[2] <= BED,
            "base_mm2": round(float(area[base].sum()), 1), "base_share": round(float(area[base].sum()) / max(foot, 1e-9), 2),
            "overhang_mm2": round(float(area[over].sum()), 1), "worst_span": round(worst, 1), "groups": groups}


def main(names):
    import v3_parts as vp
    todo = names or list(vp.PRINT)
    rep = {}
    print(f"{'part':14} up  {'size, mm':>20} {'base mm2':>9} {'overh mm2':>10} {'span':>6}  verdict")
    for n in todo:
        f = STL / f"X3_{n}.STL"
        if not f.exists():
            print(f"{n:14} -- no STL")
            continue
        r = check(n, vp.PRINT[n])
        rep[n] = r
        verdict = "ok" if r["worst_span"] <= MAX_BRIDGE and r["fits_bed"] else "CHECK"
        if r["overhang_mm2"] < 1:
            verdict = "ok, no overhangs"
        sz = " x ".join(f"{v:.0f}" for v in r["size"])
        print(f"{n:14} {r['up']}  {sz:>20} {r['base_mm2']:9.0f} {r['overhang_mm2']:10.0f} {r['worst_span']:6.1f}  {verdict}")
        for g in r["groups"]:
            if min(g["size"]) > MAX_BRIDGE and not g["slope45"]:
                print(f"{'':18}h {g['h']:5.1f}: {g['area']:7.1f} mm2, {g['size'][0]} x {g['size'][1]} at {g['at']}")
    (V3 / "dfm_report.json").write_text(json.dumps(rep, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main(sys.argv[1:])
