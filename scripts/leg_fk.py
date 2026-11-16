"""Forward kinematics on phys_spec.json (robot frame: x down, y left(+), z forward) for gait design.
Joint angle = rotor vs stator about the spec axis (right hand), as servo_sim/check_phys measure it."""
from __future__ import annotations
import json, math, os
from pathlib import Path
import numpy as np

PROJ = Path(__file__).resolve().parents[1]
VER = os.environ.get("ROBO_V", "1")
V2 = VER == "2"
SPEC = PROJ / "cad" / {"2": "v2/physics", "3": "v3/physics"}.get(VER, "physics") / "phys_spec.json"


def rot(axis, deg):
    a = np.array(axis, float); a /= np.linalg.norm(a); t = math.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K      # column convention


class Leg:
    def __init__(self, side, spec=None):
        s = spec or json.loads(SPEC.read_text(encoding="utf-8"))
        J = {j["role"]: j for j in s["joints"]}
        names = [k for k in ("hiproll", "hippitch", "knee", "ankle", "ankleroll") if f"leg{side}_{k}" in J]
        self.names = names
        self.j = [J[f"leg{side}_{k}"] for k in names]
        if "sole0" in s:                        # v3: sole centre given in the spec (robot frame, mm)
            self.sole0 = np.array(s["sole0"][side], float)
            return
        foot = [p for p in s["parts"] if p["stem"] == "Leg_Foot"
                and p["link"] == f"leg{side}_{names[-1]}"][0]
        sole_local_x = 32.0 if V2 else 41.5    # foot part frame: sole face (WEB_X + SOLE_T)
        R0 = np.array(foot["m"][:9]).reshape(3, 3)
        self.sole0 = np.array([sole_local_x, 0, 0]) @ R0 + np.array(foot["m"][9:12]) * 1000

    def pose(self, ang):
        """ang = [hiproll, hippitch, knee, ankle(, ankleroll)] deg -> (sole centre mm, foot rotation) vs the pelvis."""
        ang = list(ang) + [0.0] * (len(self.j) - len(ang))
        R = np.eye(3); p = self.sole0.copy()
        for j, a in reversed(list(zip(self.j, ang))):     # product of exponentials: distal first
            Rj = rot(j["axis"], a); piv = np.array(j["pivot_m"]) * 1000
            p = Rj @ (p - piv) + piv; R = Rj @ R
        return p, R


if __name__ == "__main__":
    for side in ("A", "B"):
        L = Leg(side); p0, _ = L.pose([0, 0, 0, 0])
        print(f"leg {side}: sole0 {p0.round(1)}")
        for name, ang in (("hiproll+10", [10, 0, 0, 0]), ("hippitch+10", [0, 10, 0, 0]), ("knee+20", [0, 0, 20, 0]),
                          ("crouch h-10 k+20 a-10", [0, -10, 20, -10]), ("crouch h+10 k-20 a+10", [0, 10, -20, 10]),
                          ("ankleroll+10", [0, 0, 0, 0, 10]), ("roll+10 & ankleroll-10", [10, 0, 0, 0, -10])):
            p, R = L.pose(ang)
            tilt = math.degrees(math.acos(max(-1, min(1, R[0, 0]))))
            print(f"   {name:<24} d(sole) = {(p - p0).round(1)}  foot tilt {tilt:.1f} deg")


def ik(leg: "Leg", target_dp, guess=None, iters=60):
    """Multi-start wrapper around _ik1: previous solution, zero, symmetric crouch guesses; best residual wins."""
    n = len(leg.j)
    starts = ([guess] if guess is not None else []) + ([] if VER == "3" else [[0.0] * n])
    for c in (15.0, 30.0):
        # v3: human knee first (thigh forward = hip pitch -, knee +); from the straight leg the solver fell into
        # the bird-like branch (hip +33, knee -66), which the v3 knee is not designed for
        for sg in ((-1, 1) if VER == "3" else (1, -1)):
            g = [0.0] * n; g[1], g[2], g[3] = sg * c, -2 * sg * c, sg * c
            starts.append(g)
    best = None
    for g in starts:
        q, e = _ik1(leg, target_dp, g, iters)
        if best is None or e < best[1]:
            best = (q, e)
        if e < 1e-3 and (guess is None or np.abs(q - np.asarray(guess)).max() < 60):
            break
    return best


def _ik1(leg: "Leg", target_dp, guess=None, iters=40):
    """Joint angles (deg, 5) putting the sole centre at sole0 + target_dp (mm, pelvis frame: x down, y left,
    z forward) with the foot flat (foot roll = pitch = 0; yaw is not controllable without a hip yaw joint).
    Damped Newton on [position (3), foot x-axis tilt (2)]."""
    n = len(leg.j)
    q = np.array(guess if guess is not None else [0.0] * n, float)
    p0, _ = leg.pose([0.0] * n)
    tgt = p0 + np.asarray(target_dp, float)

    def res(qq):
        p, R = leg.pose(qq)
        down = R[:, 0]                       # foot local x (down) in the pelvis frame -> must stay (1, 0, 0)
        return np.concatenate([(p - tgt), 100.0 * down[1:3]])   # 100 mm per rad of tilt

    for _ in range(iters):
        r = res(q)
        if np.linalg.norm(r) < 1e-3:
            break
        J = np.zeros((len(r), n)); h = 1e-3
        for i in range(n):
            dq = np.zeros(n); dq[i] = h
            J[:, i] = (res(q + dq) - r) / h
        q -= np.linalg.lstsq(J.T @ J + 1e-4 * np.eye(n), J.T @ r, rcond=None)[0]
        q = np.clip(q, -100.0, 100.0)                     # servo travel +-90 (+ margin): no wild branches
    return q, float(np.linalg.norm(res(q)))
