"""Whole-body kinematics of robo-boxer v2 for motion design without SolidWorks (2026-11-13).

Same model as leg_fk / servo_sim: phys_spec.json (robot frame mm: x down, y left = side A, z forward),
joint angle = rotor vs stator about the spec axis (right hand, column convention), masses from
massprops.json (SW), surface points from docs/viewer/robot_v2_3d.json meshes.

    B = Body();  P, idx = B.points(q);  c = B.com(q)
    B.rest(q, down)   -> how the robot lies on a flat floor: stable convex-hull facet nearest to the
                         gravity guess `down` (unit, robot frame), contact parts, COM height, margin
Quasi-static get-up check: call rest() for every key pose; the facet must stay stable and the parts
touching the floor must be the intended ones (fists/feet...).
"""
from __future__ import annotations
import base64, gzip, json, math, os, sys
from pathlib import Path
import numpy as np

os.environ.setdefault("ROBO_V", "2")
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
PROJ = HERE.parent
PHYS = PROJ / "cad" / "v2" / "physics"


def rot(axis, deg):
    a = np.array(axis, float); a /= np.linalg.norm(a); t = math.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


class Body:
    def __init__(self, cell=3.0):
        from pack_viewer_data import stl_tris
        from render_phys import cluster
        s = json.loads((PHYS / "phys_spec.json").read_text(encoding="utf-8"))
        self.spec = s
        self.J = {j["rotor"]: j for j in s["joints"]}            # link -> joint that moves it
        self.roles = [j["role"] for j in s["joints"]]
        mp = {p["name"]: p for p in json.loads((PHYS / "massprops.json").read_text(encoding="utf-8"))["parts"]}
        view = json.loads((PROJ / "docs" / "viewer" / "robot_v2_3d.json").read_text(encoding="utf-8"))
        path_of = {it["name"]: it["p"] for it in view["inst"]}
        cnt, cache = {}, {}
        self.parts = []
        for p in s["parts"]:
            st = p["stem"]; cnt[st] = cnt.get(st, 0) + 1; nm = f"{st}-{cnt[st]}"
            R = np.array(p["m"][:9]).reshape(3, 3); T = np.array(p["m"][9:12]) * 1000
            q = mp.get(nm) or mp.get(nm.replace("MG90S_grabcad", "MG90S_phys"))   # physics uses the 13.4 g copy
            m = q["m"] if q else 0.0
            cm = (np.array(q["cm"]) * 1000 @ R + T) if q else T
            src = path_of[p["name"]]
            if src not in cache:
                me = view["meshes"][src]
                tris = stl_tris(gzip.decompress(base64.b64decode(me["gz"]))) + np.array(me["off"])
                v, _ = cluster(tris, cell)
                cache[src] = v
            self.parts.append(dict(name=nm, stem=st, link=p["link"], m=m, cm=cm, pts=cache[src] @ R + T))
        self.mass = sum(p["m"] for p in self.parts)

    def link_tf(self, q):
        """q: {role: deg} -> {link: (R, t)}: current point = R @ initial + t (base frame)."""
        out = {"base": (np.eye(3), np.zeros(3))}

        def tf(L):
            if L in out:
                return out[L]
            j = self.J[L]
            Rs, ts = tf(j["stator"])
            Rj = rot(j["axis"], q.get(j["role"], 0.0)); piv = np.array(j["pivot_m"]) * 1000
            R = Rs @ Rj; t = Rs @ (piv - Rj @ piv) + ts
            out[L] = (R, t)
            return out[L]
        for p in self.parts:
            tf(p["link"])
        return out

    def points(self, q):
        T = self.link_tf(q)
        P, idx = [], []
        for i, p in enumerate(self.parts):
            R, t = T[p["link"]]
            P.append(p["pts"] @ R.T + t); idx.append(np.full(len(p["pts"]), i))
        return np.concatenate(P), np.concatenate(idx)

    def com(self, q):
        T = self.link_tf(q)
        acc = np.zeros(3)
        for p in self.parts:
            R, t = T[p["link"]]
            acc += p["m"] * (R @ p["cm"] + t)
        return acc / self.mass

    def where(self, q, stem):
        """centroids of the parts with this stem (current pose)."""
        T = self.link_tf(q)
        out = []
        for p in self.parts:
            if p["stem"] == stem:
                R, t = T[p["link"]]
                out.append((p["name"], (p["pts"] @ R.T + t).mean(0)))
        return out

    def rest(self, q, down, tol=2.0, max_turn=None):
        """Stable resting facet on a flat floor nearest to gravity guess `down` (robot frame unit vector).
        Returns dict(n=floor normal pointing into the floor (= gravity dir), parts touching (within tol mm),
        margin = distance (mm) of the COM projection inside the support polygon, h = COM height)."""
        from scipy.spatial import ConvexHull
        P, idx = self.points(q); c = self.com(q)
        hull = ConvexHull(P)
        best = None
        down = np.array(down, float); down /= np.linalg.norm(down)
        for eq in np.unique(np.round(hull.equations, 6), axis=0):
            n = eq[:3]; off = eq[3]                       # n.x + off <= 0 inside; n outward
            d = -(P @ n + off)                             # height above this facet plane (>=0)
            sup = d < tol
            S = P[sup]
            if len(S) < 3:
                continue
            u = np.cross(n, [1, 0, 0] if abs(n[0]) < 0.9 else [0, 1, 0]); u /= np.linalg.norm(u); w = np.cross(n, u)
            S2 = np.c_[S @ u, S @ w]; c2 = np.array([c @ u, c @ w])
            try:
                h2 = ConvexHull(S2)
            except Exception:
                continue
            m = min(-(e[:2] @ c2 + e[2]) for e in h2.equations)   # >0 inside
            if m <= 0:
                continue
            score = n @ down
            if max_turn is not None and score < math.cos(math.radians(max_turn)):
                continue                                   # slow motion: the body cannot jump to a far facet
            if best is None or score > best[0]:
                parts = sorted({self.parts[i]["name"] for i in idx[sup]})
                best = (score, dict(n=n, margin=float(m), h=float(-(c @ n + off)), parts=parts, tilt=math.degrees(math.acos(min(1, score)))))
        return best[1] if best else None


if __name__ == "__main__":
    B = Body()
    print(f"mass {B.mass * 1000:.0f} g, COM {np.round(B.com({}), 1)}")
    for k in ("Fist_TPU", "Leg_Foot", "Pelvis", "Torso_Mount", "Shoulder_Yoke"):
        for n, c in B.where({}, k):
            print(f"  {n:16s} {np.round(c, 1)}")
    for nm, dn in (("standing", [1, 0, 0]), ("back", [0, 0, -1]), ("chest", [0, 0, 1])):
        r = B.rest({}, dn)
        print(nm, "-> tilt", round(r["tilt"], 1), "margin", round(r["margin"], 1), "h", round(r["h"], 1), r["parts"][:8])
