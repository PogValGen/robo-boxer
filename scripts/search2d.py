"""Dijkstra over symmetric poses (A shoulder, H hip, K knee, N ankle; step STEP deg) with the quasi-static resting
orientation from path2d: find a keyframe path from lying (chest/back) to standing.   (2026-11-13)
    python search2d.py chest|back [step=30] [maxjump=35]
Cost = 1 per joint step + jump penalty (sudden roll). Writes search2d_<lie>.json with the path."""
import heapq, itertools, json, sys, time
import numpy as np
import path2d
from path2d import dang

lie = sys.argv[1]; STEP = int(sys.argv[2]) if len(sys.argv) > 2 else 30; MAXJ = float(sys.argv[3]) if len(sys.argv) > 3 else 35
NAMES = ["A", "H", "K", "N"]
ROLE = {"A": "arm{}_shoulder", "H": "leg{}_hippitch", "K": "leg{}_knee", "N": "leg{}_ankle"}
path2d.B = path2d.wb_fk.Body(cell=8.0)

def Q(p):
    return {ROLE[n].format(s): float(v) for n, v in zip(NAMES, p) for s in "AB"}

def move(p, th, i, d):
    """one joint i by d*STEP in 3 substeps; returns (new pose, new th, biggest jump, min margin at the end)"""
    jump = 0.0
    for k in (1, 2, 3):
        pp = list(p); pp[i] = p[i] + d * STEP * k / 3
        e = path2d.settle(Q(pp), th); jump = max(jump, abs(dang(e["th"], th))); th = e["th"]
    return tuple(pp), th, jump, min(e["s"], e["L"] - e["s"])

th0 = {"chest": 90.0, "back": -90.0}[lie]
e0 = path2d.settle(Q((0, 0, 0, 0)), th0)
start = ((0, 0, 0, 0), round(e0["th"] / 5))
dist = {start: 0.0}; prev = {}; thv = {start: e0["th"]}
pq = [(0.0, start)]; t0 = time.time(); n = 0; goal = None
while pq:
    c, s = heapq.heappop(pq)
    if c > dist.get(s, 1e9):
        continue
    p, _ = s; th = thv[s]; n += 1
    if n % 200 == 0:
        print(f"  {n} states, cost {c:.1f}, queue {len(pq)}, {time.time() - t0:.0f} s", flush=True)
    if all(v == 0 for v in p) and abs(dang(th, 0)) < 12:
        goal = s; break
    for i, d in itertools.product(range(4), (-1, 1)):
        if abs(p[i] + d * STEP) > 90:
            continue
        p2, th2, jump, mar = move(p, th, i, d)
        if jump > MAXJ:
            continue
        s2 = (tuple(int(round(v)) for v in p2), round(th2 / 5))
        c2 = c + 1 + jump / 10 + (2 if mar < 4 else 0)
        if c2 < dist.get(s2, 1e9):
            dist[s2] = c2; prev[s2] = s; thv[s2] = th2; heapq.heappush(pq, (c2, s2))
print(f"{n} states expanded in {time.time() - t0:.0f} s; goal {'FOUND' if goal else 'not found'}")
if goal:
    path = [goal]
    while path[-1] in prev:
        path.append(prev[path[-1]])
    path.reverse()
    for s in path:
        print("  ", dict(zip(NAMES, s[0])), f"th {thv[s]:+.0f}")
    json.dump([[list(s[0]), thv[s]] for s in path], open(f"search2d_{lie}.json", "w"))
else:
    # report the states closest to standing
    best = sorted(dist, key=lambda s: abs(dang(thv[s], 0)))[:10]
    for s in best:
        print("  near:", dict(zip(NAMES, s[0])), f"th {thv[s]:+.0f} cost {dist[s]:.1f}")
