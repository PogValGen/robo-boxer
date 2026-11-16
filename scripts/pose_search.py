"""Pick shoulder/elbow angles (sign + value) for a real boxing guard and jab with the same FK as
the physics model (axes from the servos, build_anim.build_links). Robot frame: x DOWN, z FORWARD.
Guard: fist in front of the chest (z > 0) at face height; jab: fist straight forward at shoulder height."""
from __future__ import annotations
import itertools, json, math, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import build_anim as BA  # noqa: E402

d = json.loads(BA.SRC.read_text(encoding="utf-8"))
links, classify, order, units = BA.build_links(d)
roleOrder, seen = [], set()
def add(r):
    if r in seen: return
    if links[r]["parent"]: add(links[r]["parent"])
    seen.add(r); roleOrder.append(r)
for r in links: add(r)

def rot(axis, ang):
    a = np.array(axis, float); a /= np.linalg.norm(a); x, y, z = a; c, s = math.cos(ang), math.sin(ang); C = 1 - c
    return np.array([[c+x*x*C, x*y*C-z*s, x*z*C+y*s], [y*x*C+z*s, c+y*y*C, y*z*C-x*s], [z*x*C-y*s, z*y*C+x*s, c+z*z*C]])

def fk(ang):
    L = {}
    for r in roleOrder:
        l = links[r]
        if l["parent"] is None: L[r] = np.eye(4); continue
        P = L[l["parent"]]; ax = P[:3, :3] @ np.array(l["axis"]); pv = (P @ np.append(l["pivot"], 1))[:3]
        M = np.eye(4); M[:3, :3] = rot(ax, math.radians(ang.get(r, 0.0))); M[:3, 3] = pv - M[:3, :3] @ pv
        L[r] = M @ P
    return L

fist = {n: np.array(it["m"][9:12]) * 1000 for it in d["inst"] for n in [it["name"]] if "Fist" in n}
fA = [n for n in fist if n.startswith("Arm-1")][0]
def fistpos(ang, side="A"):
    n = fA if side == "A" else [k for k in fist if k.startswith("Arm-2")][0]
    L = fk(ang); return (L[classify(n)] @ np.append(fist[n], 1))[:3]

sh = np.array(links["armA_shoulder"]["pivot"]); print("shoulder A pivot", sh.round(0), " fist A neutral", fist[fA].round(0))
best = []
for s_sh, s_el in itertools.product((1, -1), (1, -1)):
    for S in (60, 75, 90):
        for E in (70, 90, 110):
            g = fistpos({"armA_shoulder": s_sh * S, "armA_elbow": s_el * E})
            score = (g[2] > 20) * 1000 - abs(g[0] - (sh[0] - 40)) - 0.3 * abs(g[1] - sh[1])   # in front, ~40 mm above shoulder
            best.append((score, s_sh, s_el, S, E, g))
best.sort(key=lambda r: -r[0])
print("\nGUARD candidates (score, sign_sh, sign_el, sh, el, fist x/y/z):")
for r in best[:5]: print("  ", round(r[0]), r[1], r[2], r[3], r[4], r[5].round(0))
s_sh, s_el = best[0][1], best[0][2]
print("\nJAB with the same signs:")
for S, E in ((80, 0), (90, 0), (95, 5), (100, 0)):
    j = fistpos({"armA_shoulder": s_sh * S, "armA_elbow": s_el * E})
    print(f"   sh={s_sh*S:+} el={s_el*E:+} fist={j.round(0)}  forward z={j[2]:.0f}  height dx from shoulder={j[0]-sh[0]:.0f}")
print("\nsigns: shoulder", s_sh, "elbow", s_el)
# robot frame x down, z forward -> +y = LEFT. Right arm = Arm-2 (side B, y=-116).
print("\n== right arm (B) and waist twist ==")
for sB in (1, -1):
    p = fistpos({"armB_shoulder": sB * 90}, "B"); print(f"  armB_shoulder={sB*90:+}: fist {p.round(0)}")
sB = 1 if fistpos({"armB_shoulder": 90}, "B")[2] > 0 else -1
for w in (20, -20):
    p0 = fistpos({"armB_shoulder": sB * 90}, "B"); p1 = fistpos({"armB_shoulder": sB * 90, "torso": w}, "B")
    print(f"  waist {w:+}: right fist z {p0[2]:.0f} -> {p1[2]:.0f}")
print("RESULT: right shoulder sign", sB, "| waist sign that drives the right fist forward:",
      1 if fistpos({"armB_shoulder": sB * 90, "torso": 20}, "B")[2] > fistpos({"armB_shoulder": sB * 90}, "B")[2] else -1)
