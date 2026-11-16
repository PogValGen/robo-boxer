"""Sweep (clash) check of a joint on the real STL meshes, numpy only.

Static set (stator side): housing + bridge + previous lever (and its horn) placed LINK_DX behind.
Moving set (rotor side): lever + horn + next housing + next bridge (LINK_DX ahead), turned about the
servo axis (Y through x = AX, z = 0) from -range to +range.
Static parts are voxelised (0.2 mm, inside = ray parity along z); moving parts are covered with
surface points (~6 per mm^2). A point inside static material = clash; within one voxel (0.2 mm) =
near-touch. Counts at angle 0 are the designed fits (bushings, pegs) and are subtracted.

Usage: python sweep_check.py [v2] [range_deg]
"""
from __future__ import annotations

import math
import struct
import sys
from pathlib import Path

import numpy as np

PROJ = Path(__file__).resolve().parents[1]
AX = 5.5
VER = {"v2": dict(dir=PROJ / "cad" / "v2" / "joint" / "stl", housing="J2_Housing", bridge="J2_Bridge",
                  lever=("J2_LeverTop", "J2_LeverBot"), bbox_json=PROJ / "cad" / "v2" / "joint" / "bboxes.json",
                  horn=PROJ / "cad" / "joint" / "stl" / "MG90S_Horn.STL", link_dx=45.3,
                  # SW writes STL moved into the positive octant -> place each mesh by its model bbox min
                  # (printed by build_joint_v2 finish(): "bbox [...]")
                  bbox_min={"J2_Housing": (-24.1, -18.7, -8.5), "J2_Bridge": (-21.3, 19.1, -6.5),
                            "J2_Lever": (-1.5, -19.0, -7.0)})}


def load_stl(p: Path) -> np.ndarray:
    b = p.read_bytes()
    if b[:5].lower() == b"solid" and b"facet" in b[:400]:
        v = [list(map(float, ln.split()[1:4])) for ln in b.decode(errors="ignore").splitlines() if ln.strip().startswith("vertex")]
        return np.array(v).reshape(-1, 3, 3)
    n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3, 3).astype(float)


def unit_mm(tris: np.ndarray) -> np.ndarray:
    return tris * 1000.0 if np.abs(tris).max() < 1.0 else tris   # SW STL may be in metres


class Vox:
    def __init__(self, meshes, h=0.2, pad=1.0):
        allv = np.vstack([m.reshape(-1, 3) for m in meshes])
        self.lo = allv.min(0) - pad; self.h = h
        self.n = np.ceil((allv.max(0) + pad - self.lo) / h).astype(int) + 1
        self.occ = np.zeros(self.n, bool)
        for m in meshes:
            self.occ |= self._fill(m)

    def _fill(self, tris):
        nx, ny, nz = self.n
        cross = {}
        xs = self.lo[0] + (np.arange(nx) + 0.5) * self.h; ys = self.lo[1] + (np.arange(ny) + 0.5) * self.h
        for t in tris:
            a, b, c = t
            x0, x1 = min(a[0], b[0], c[0]), max(a[0], b[0], c[0]); y0, y1 = min(a[1], b[1], c[1]), max(a[1], b[1], c[1])
            i0, i1 = np.searchsorted(xs, x0), np.searchsorted(xs, x1); j0, j1 = np.searchsorted(ys, y0), np.searchsorted(ys, y1)
            if i0 >= i1 or j0 >= j1:
                continue
            X, Y = np.meshgrid(xs[i0:i1], ys[j0:j1], indexing="ij")
            d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
            if abs(d) < 1e-12:
                continue
            l1 = ((b[1] - c[1]) * (X - c[0]) + (c[0] - b[0]) * (Y - c[1])) / d
            l2 = ((c[1] - a[1]) * (X - c[0]) + (a[0] - c[0]) * (Y - c[1])) / d
            l3 = 1 - l1 - l2
            ins = (l1 >= 0) & (l2 >= 0) & (l3 >= 0)
            Z = l1 * a[2] + l2 * b[2] + l3 * c[2]
            for ii, jj, z in zip(*np.nonzero(ins), Z[ins]):
                cross.setdefault((i0 + ii, j0 + jj), []).append(z)
        occ = np.zeros(self.n, bool)
        for (i, j), zl in cross.items():
            zl = sorted(zl)
            for z0, z1 in zip(zl[0::2], zl[1::2]):
                k0 = int(math.ceil((z0 - self.lo[2]) / self.h - 0.5)); k1 = int(math.floor((z1 - self.lo[2]) / self.h - 0.5))
                if k1 >= k0:
                    occ[i, j, max(k0, 0):min(k1, nz - 1) + 1] = True
        return occ

    def hits(self, pts, grow=0):
        idx = np.floor((pts - self.lo) / self.h).astype(int)
        ok = np.all((idx >= 0) & (idx < self.n), axis=1)
        idx = idx[ok]
        occ = self.occ
        if grow:
            g = occ.copy()
            for ax in range(3):
                for s in (-1, 1):
                    g |= np.roll(occ, s, axis=ax)
            occ = g
        return int(occ[idx[:, 0], idx[:, 1], idx[:, 2]].sum())


def surface_points(tris, per_mm2=6.0, seed=0):
    rng = np.random.default_rng(seed)
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    n = np.maximum(1, np.round(area * per_mm2).astype(int))
    k = np.repeat(np.arange(len(tris)), n)
    u, v = rng.random(len(k)), rng.random(len(k))
    f = u + v > 1; u[f], v[f] = 1 - u[f], 1 - v[f]
    return a[k] + u[:, None] * (b[k] - a[k]) + v[:, None] * (c[k] - a[k])


def rot_y(pts, deg):
    th = math.radians(deg); c, s = math.cos(th), math.sin(th)
    x, z = pts[:, 0] - AX, pts[:, 2]
    return np.column_stack([AX + c * x + s * z, pts[:, 1], -s * x + c * z])


def main(ver="v2", rng_deg=110.0):
    cfg = VER[ver]; d = cfg["dir"]; dx = np.array([cfg["link_dx"], 0, 0])
    import json as _json
    bmin = {k: v["min"] for k, v in _json.loads(Path(cfg["bbox_json"]).read_text()).items()} if "bbox_json" in cfg else cfg["bbox_min"]

    def ld(n):
        if isinstance(n, (tuple, list)):
            return np.vstack([ld(x) for x in n])
        m = unit_mm(load_stl(d / f"{n}.STL"))
        return m - m.reshape(-1, 3).min(0) + np.array(bmin[n])
    H, B, L = ld(cfg["housing"]), ld(cfg["bridge"]), ld(cfg["lever"])
    horn = unit_mm(load_stl(cfg["horn"])) if Path(cfg["horn"]).exists() else np.zeros((0, 3, 3))
    static = [H, B, L - dx] + ([horn - dx] if len(horn) else [])
    moving = [L, H + dx, B + dx] + ([horn] if len(horn) else [])
    names = ["lever", "next housing", "next bridge", "horn"][:len(moving)]
    print(f"[1/3] voxelising static set ({sum(len(m) for m in static)} triangles, 0.2 mm) ...", flush=True)
    vox = Vox(static)
    print(f"      grid {vox.n.tolist()}, occupied {vox.occ.sum()} voxels")
    pts = [surface_points(m, seed=i) for i, m in enumerate(moving)]
    # rotary bushing pairs (spigot D6 in hole D6.2, r 3.0/3.1) turn in themselves: not a sweep question
    pts = [p[np.hypot(p[:, 0] - AX, p[:, 2]) > 3.4] for p in pts]
    print(f"[2/3] moving points: {[len(p) for p in pts]}")
    base = [(vox.hits(p), vox.hits(p, grow=1)) for p in pts]
    print(f"      at 0 deg (designed fits): {dict(zip(names, base))}")
    worst = {n: (0, 0, 0.0) for n in names}
    angles = np.arange(-rng_deg, rng_deg + 0.1, 5.0)
    print(f"[3/3] sweep {angles[0]:.0f}..{angles[-1]:.0f} deg, {len(angles)} steps")
    for k, a in enumerate(angles):
        row = []
        for n, p, (b0, g0) in zip(names, pts, base):
            q = rot_y(p, a)
            hi, gi = vox.hits(q) - b0, vox.hits(q, grow=1) - g0
            row.append(f"{n}:{hi:+d}/{gi:+d}")
            if hi > worst[n][0] or (hi == worst[n][0] and gi > worst[n][1]):
                worst[n] = (hi, gi, float(a))
        if k % 4 == 0 or any(int(r.split(":")[1].split("/")[0]) > 0 for r in row):
            print(f"  {a:+6.0f} deg  " + "  ".join(row))
    print("\nworst extra (clash points / near-touch points, angle):")
    ok = True
    for n, (hi, gi, a) in worst.items():
        flag = "CLASH" if hi > 5 else ("near" if gi > 20 else "ok")
        ok &= flag != "CLASH"
        print(f"  {n:<14} {hi:5d} / {gi:5d} at {a:+.0f} deg  {flag}")
    print("RESULT:", "no clashes in +-%.0f deg" % rng_deg if ok else "CLASHES FOUND")
    return ok


if __name__ == "__main__":
    a = sys.argv[1:]
    main(a[0] if a else "v2", float(a[1]) if len(a) > 1 else 110.0)
