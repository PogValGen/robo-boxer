"""Quasi-static sagittal (2D) check of a symmetric whole-body motion on a flat floor (2026-11-13).
The body (all part points projected on the robot x-z plane) rests on an edge of its 2D convex hull; when the COM
leaves the edge the body rolls over the edge's end to the next edge. Tracks the resting edge by continuity.
theta = gravity direction in the robot frame: 0 = standing, +90 = on the chest, -90 = on the back, 180 = head down.

    python path2d.py <scenario> chest|back [--param k=v ...] [--dt 0.05]
Prints theta, what touches the floor, COM margin (mm inside the support edge) along the scenario's keyframes.
"""
import math, sys
import numpy as np
from scipy.spatial import ConvexHull
import wb_fk

B = wb_fk.Body(cell=4.0)


def edges(q):
    P, idx = B.points(q); c = B.com(q)
    p2 = np.c_[P[:, 0], P[:, 2]]; c2 = np.array([c[0], c[2]])
    h = ConvexHull(p2); v = h.vertices; n = len(v); E = []
    for i in range(n):
        a, b = p2[v[i]], p2[v[(i + 1) % n]]; d = b - a; L = np.linalg.norm(d)
        if L < 1e-9:
            continue
        nrm = np.array([d[1], -d[0]]) / L                       # outward (hull is CCW)
        s = (c2 - a) @ d / L                                     # COM projection along the edge, 0..L = stable
        E.append(dict(th=math.degrees(math.atan2(nrm[1], nrm[0])), s=s, L=L, hc=float((a - c2) @ nrm),
                      pa=B.parts[idx[v[i]]]["name"], pb=B.parts[idx[v[(i + 1) % n]]]["name"]))
    return E


def dang(a, b):
    return (a - b + 180) % 360 - 180


def settle(q, th):
    E = edges(q); n = len(E)
    i = min(range(n), key=lambda k: abs(dang(E[k]["th"], th)))
    for _ in range(n):
        e = E[i]
        if -0.5 <= e["s"] <= e["L"] + 0.5:
            return e
        i = (i + 1) % n if e["s"] > e["L"] else (i - 1) % n     # roll over the end the COM went past
    return E[i]


def run(cmds, dur, th0, dt=0.05, verbose=True):
    ts = np.arange(0, dur + 1e-9, dt); th = th0; out = []
    for t in ts:
        q = {r: float(np.interp(t, [k[0] for k in ks], [k[1] for k in ks])) for r, ks in cmds.items()}
        e = settle(q, th); th = e["th"]
        out.append((t, th, e))
    if verbose:
        last = None
        for t, th, e in out:
            key = (round(th / 5), e["pa"], e["pb"])
            if key != last or abs(t * 2 - round(t * 2)) < 1e-6:
                print(f"{t:5.2f}  th {th:+7.1f}  margin {min(e['s'], e['L'] - e['s']):6.1f} of {e['L']:6.1f}  "
                      f"COM h {e['hc']:6.1f}  on {e['pa']} .. {e['pb']}")
            last = key
    return out


if __name__ == "__main__":
    import gaits
    a = sys.argv[1:]
    par = {}
    dt = 0.05
    i = 2
    while i < len(a):
        if a[i] == "--param":
            k, v = a[i + 1].split("="); par[k] = float(v); i += 2
        elif a[i] == "--dt":
            dt = float(a[i + 1]); i += 2
        else:
            i += 1
    cmds, dur = gaits.scenario(a[0], fixed=False, steps=None, **par)
    run(cmds, dur, {"chest": 90.0, "back": -90.0, "up": 0.0}[a[1]], dt)
