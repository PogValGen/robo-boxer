"""Whole-robot test poses for `v3_asm.py poses <json>` (interference in SolidWorks, both sides at once).
Leg poses come from the numeric IK (leg_fk, flat feet), so they are the poses the gait really uses.

    set ROBO_V=3
    python v3_poses.py            -> cad/v3/poses_set.json
    python v3_poses.py gait dyn   -> cad/v3/poses_gait.json: keyframes of a gaits scenario sampled every 0.1 s
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

os.environ.setdefault("ROBO_V", "3")
import leg_fk  # noqa: E402

V3 = Path(__file__).resolve().parents[1] / "cad" / "v3"
LEGS = {s: leg_fk.Leg(s) for s in "AB"}


def leg(side, dx=0.0, dy=0.0, dz=0.0, yaw=0.0):
    q, err = leg_fk.ik(LEGS[side], [dx, dy, dz])
    assert err < 0.5, (side, dx, dy, dz, err)
    out = {f"leg{side}_{n}": round(float(a), 2) for n, a in zip(LEGS[side].names, q)}
    out[f"leg{side}_hipyaw"] = yaw
    return out


def arms(sh=0.0, roll=0.0, el=0.0, shB=None, rollB=None, elB=None):
    return {"armA_shoulder": sh, "armA_shoulderroll": roll, "armA_elbow": el,
            "armB_shoulder": sh if shB is None else shB, "armB_shoulderroll": roll if rollB is None else rollB,
            "armB_elbow": el if elB is None else elB}


def both(a, b, arm):
    return {**a, **b, **arm}


def poses():
    guard = arms(-60, 0, -60)
    P = []
    add = lambda n, d: P.append([n, d])                                       # noqa: E731
    c = 10.0                                                                  # walking crouch, mm
    add("crouch10_guard", both(leg("A", -c), leg("B", -c), guard))
    for nm, dy in (("L", -26.0), ("R", 26.0)):                                # body over the left / right foot
        add(f"sway{nm}_guard", both(leg("A", -c, dy), leg("B", -c, dy), guard))
        add(f"sway{nm}_arms_down", both(leg("A", -c, dy), leg("B", -c, dy), arms()))
    add("swayL_liftR15_fwd30", both(leg("A", -c, -26), leg("B", -c - 15, -26, 30), guard))
    add("swayL_liftR15_back30", both(leg("A", -c, -26), leg("B", -c - 15, -26, -30), guard))
    add("swayR_liftL15_fwd30", both(leg("A", -c - 15, 26, 30), leg("B", -c, 26), guard))
    add("split_A+30_B-30", both(leg("A", -c, 0, 30), leg("B", -c, 0, -30), guard))
    add("split_A-30_B+30", both(leg("A", -c, 0, -30), leg("B", -c, 0, 30), guard))
    add("sidestep_A+20", both(leg("A", -c, 20), leg("B", -c), guard))
    add("crouch30", both(leg("A", -30), leg("B", -30), guard))
    add("crouch20", both(leg("A", -20), leg("B", -20), guard))        # knee 95 deg at crouch 30: the deepest squat
    # turning: feet together and in a wider stance (each foot 12 mm out)
    for y in (5.0, 10.0):
        add(f"yaw_both+{y:.0f}", both(leg("A", -c, yaw=y), leg("B", -c, yaw=y), guard))
        add(f"yaw_both-{y:.0f}", both(leg("A", -c, yaw=-y), leg("B", -c, yaw=-y), guard))
    for y in (10.0, 20.0, 30.0):
        add(f"wide12_yaw_both+{y:.0f}", both(leg("A", -c, 12, yaw=y), leg("B", -c, -12, yaw=y), guard))
        add(f"wide12_yaw_both-{y:.0f}", both(leg("A", -c, 12, yaw=-y), leg("B", -c, -12, yaw=-y), guard))
        add(f"wide12_turn{y:.0f}", both(leg("A", -c, 12, yaw=y), leg("B", -c, -12, yaw=-y), guard))
    # arms
    st = both(leg("A", -c), leg("B", -c), {})
    add("arms_down", {**st, **arms()})
    add("jab_B", {**st, **arms(-60, 0, -60, shB=-90, elB=0)})
    add("arms_fwd90", {**st, **arms(-90)})
    add("arms_up180", {**st, **arms(-180)})
    add("arms_back15", {**st, **arms(15)})
    add("arms_side60", {**st, **arms(0, 60)})
    add("arms_fwd90_in20", {**st, **arms(-90, -20)})
    add("guard_elbow90", {**st, **arms(-60, 0, -90)})
    add("hook_B", {**st, **arms(-60, 0, -60, shB=-90, rollB=45, elB=-90)})
    return P


def gait(name, dt=0.1, **par):
    import gaits
    cmds, dur = gaits.scenario(name, **par)
    P = []
    n = int(dur / dt) + 1
    for i in range(n):
        t = i * dt
        P.append([f"{name}_t{t:05.2f}", {r: round(gaits.eval_cmd(k, t), 2) for r, k in cmds.items()}])
    return P


if __name__ == "__main__":
    a = sys.argv[1:]
    if a[:1] == ["gait"]:
        par = {k: float(v) for k, v in (x.split("=") for x in a[2:])}
        P = gait(a[1], **par)
        out = V3 / "poses_gait.json"
    else:
        P = poses()
        out = V3 / "poses_set.json"
    out.write_text(json.dumps(P, indent=1), encoding="utf-8")
    print(f"{len(P)} poses -> {out}")
    for n, d in P[:60]:
        print(f"  {n:26s}", " ".join(f"{k.split('_')[0][-1]}{k.split('_')[1][:5]}={v:.0f}" for k, v in d.items() if abs(v) > 0.5))
