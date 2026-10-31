"""Measure rectangular pockets (servo seats) in an STL: pairs of facing inner walls.

Walls of a cavity face each other: the wall at the smaller coordinate has its
outward normal pointing +axis (into the void), the other one -axis. For every
axis we pair such planes whose projections overlap and report the gap.

Usage: python stl_pockets.py <file.stl> [<file.stl> ...]
"""

from __future__ import annotations

import struct
import sys
from collections import defaultdict

import numpy as np


def load(path: str) -> np.ndarray:
    data = open(path, "rb").read()
    if data[:5].lower() == b"solid" and b"facet" in data[:400]:
        tris = []
        for line in data.decode("ascii", "replace").splitlines():
            line = line.strip()
            if line.startswith("vertex"):
                tris.append([float(v) for v in line.split()[1:4]])
        return np.array(tris, float).reshape(-1, 3, 3)
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rec["v"].reshape(-1, 3, 3).astype(float)


def planes(tris: np.ndarray, axis: int) -> dict[tuple[int, float], dict]:
    """Axis-aligned planar patches: (sign, coord) -> area and bounding box of the other two axes."""
    e1, e2 = tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    nrm = np.cross(e1, e2)
    area = np.linalg.norm(nrm, axis=1) / 2
    unit = nrm / np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    out: dict = defaultdict(lambda: {"area": 0.0, "lo": np.full(3, 1e9), "hi": np.full(3, -1e9)})
    for t, u, a in zip(tris, unit, area):
        if abs(abs(u[axis]) - 1) > 1e-3 or a < 1e-6:
            continue
        key = (int(np.sign(u[axis])), round(float(t[:, axis].mean()), 2))
        p = out[key]
        p["area"] += a
        p["lo"] = np.minimum(p["lo"], t.min(axis=0))
        p["hi"] = np.maximum(p["hi"], t.max(axis=0))
    return out


def pockets(path: str) -> None:
    tris = load(path)
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    print(f"\n{path}\n  triangles {len(tris)}, bbox {np.round(lo, 2)} .. {np.round(hi, 2)} size {np.round(hi - lo, 2)}")
    for axis, name in enumerate("XYZ"):
        pl = planes(tris, axis)
        plus = sorted((c, p) for (s, c), p in pl.items() if s > 0)
        minus = sorted((c, p) for (s, c), p in pl.items() if s < 0)
        found = []
        for c1, p1 in plus:
            for c2, p2 in minus:
                if c2 <= c1 + 0.3:
                    continue
                other = [i for i in range(3) if i != axis]
                overlap = all(min(p1["hi"][i], p2["hi"][i]) - max(p1["lo"][i], p2["lo"][i]) > 1.0 for i in other)
                if overlap and p1["area"] > 4 and p2["area"] > 4:
                    found.append((round(c2 - c1, 2), round(c1, 2), round(c2, 2), round(min(p1["area"], p2["area"]), 1)))
                    break  # nearest facing wall
        found.sort()
        print(f"  {name}: inner gaps (gap, from, to, wall area mm2):", found[:12])


if __name__ == "__main__":
    for f in sys.argv[1:]:
        pockets(f)
