"""Heights above the floor (mm) of pelvis / head / feet / fists over time for a --lie run (2026-11-13).
    python hts.py <label> chest|back|up [t1 t2 ...]"""
import json, re, sys, numpy as np
from pathlib import Path
P = Path(__file__).resolve().parent.parent / "cad" / "v2" / "physics"
Q = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.0]])
lab, lie = sys.argv[1], sys.argv[2]
g = {"chest": np.array([0, 0, 1.0]), "back": np.array([0, 0, -1.0]), "up": np.array([0, -1.0, 0])}[lie]
r = json.loads((P / f"servo_{lab}_result.json").read_text(encoding="utf-8"))
log = (P.parent / "logs" / f"q{lab}.log").read_text(encoding="utf-8", errors="ignore")
link = {m.group(1): m.group(2) for m in re.finditer(r"servo S_(\w+) <- motor '\w+' on (\S+)", log)}
pts = {"pelvis": ("Pelvis-1", (0, 0, 0)), "head": ("Torso_Mount-1", (-140, 0, 0)),
       "footA": (link["legA_ankleroll"], (117, 28, 0)), "footB": (link["legB_ankleroll"], (117, -28, 0)),
       "toeA": (link["legA_ankleroll"], (117, 28, 40)), "heelA": (link["legA_ankleroll"], (117, 28, -30)),
       "fistA": (link["armA_elbow"], (15, 60, 0)), "fistB": (link["armB_elbow"], (15, -60, 0))}
ps = r["poses"]
def M(t, n):
    m = ps[t][n]; return np.array(m[:9]).reshape(3, 3), np.array(m[9:12]) * 1000
loc = {}
for k, (n, p) in pts.items():
    R0, T0 = M("0", n); loc[k] = (np.array(p, float) @ Q - T0) @ R0.T
def W(t, k):
    R, T = M(t, pts[k][0]); return loc[k] @ R + T
ts = sys.argv[3:] or [f"{x:g}" for x in np.arange(0, float(list(ps)[-1]) + 1e-9, 0.25)]
# floor level = lowest of all tracked points at t=0 minus nothing (relative heights)
print("    t  " + " ".join(f"{k:>7}" for k in pts) + "   fwd(head-pelvis along body)")
for t in ts:
    if t in ps:
        print(f"{t:>5}  " + " ".join(f"{-(W(t, k) @ g):7.1f}" for k in pts))
