"""Motor profiles (deg) for the SOLIDWORKS Motion study, one rotary motor per servo.

Scenario "punch": standing (pelvis fixed), arms up to guard, right (side A) jab with a waist
twist, back to guard. Angles = the animation (build_anim.py) values, verified with kin_sim.py
(fist goes forward +z). Motion STEP() = smooth cubic ramp. Angles relative to the start pose.
`reverse` per joint: set after comparing achieved vs commanded angles (check_phys.py).

Usage: python gen_phys_motion.py [punch] [out.json]
"""
from __future__ import annotations
import json, math, re, sys
from pathlib import Path

PROJ = Path(__file__).resolve().parents[1]
REV_FILE = PROJ / "cad" / "physics" / "motor_reverse.json"   # role -> bool, written by check_phys.py
# Angles in the physics model's own sense (servo +Y), found with scripts/pose_search.py:
#   robot frame x DOWN, z FORWARD -> +y is LEFT: Arm-1 = side A = LEFT, Arm-2 = side B = RIGHT.
#   shoulder NEGATIVE swings the arm forward (-90 = horizontal, fist z=+106 at shoulder height);
#   the "elbow" servo carries the fist at its own lever (no forearm) -> it only spins the fist, held at 0;
#   waist +20 drives the right fist forward (+33 mm).
G = {"shoulder": -60.0, "elbow": 0.0, "roll": 0.0}    # guard: both fists in front of the chest
P = {"shoulder": -95.0, "waist": 20.0}                 # right jab: arm horizontal forward + body twist
PUNCH_SIDE = "B"                                       # right arm
T_UP, T_HOLD, T_OUT, T_BACK, T_END = 0.8, 1.2, 1.45, 1.9, 2.4


def step(t0, t1, dh):
    return f"STEP(TIME,{t0},0,{t1},{dh:.3f}D)" if abs(dh) > 1e-9 else ""


def expr(*terms):
    t = [x for x in terms if x]
    return " + ".join(t) if t else "0"


def punch(kind, side):
    hit = side == PUNCH_SIDE
    if kind == "shoulder":
        g = G["shoulder"]
        return expr(step(0, T_UP, g), step(T_HOLD, T_OUT, P["shoulder"] - g) if hit else "",
                    step(T_OUT, T_BACK, g - P["shoulder"]) if hit else "")
    if kind in ("elbow", "shoulderroll"):
        return "0"
    if kind == "waist":
        return expr(step(T_HOLD, T_OUT, P["waist"]), step(T_OUT, T_BACK, -P["waist"]))
    return "0"   # legs hold the stance


def main(scn="punch", out=None):
    spec = json.loads((PROJ / "cad" / "physics" / "phys_spec.json").read_text(encoding="utf-8"))
    rev = json.loads(REV_FILE.read_text()) if REV_FILE.exists() else {}
    motors = []
    for j in spec["joints"]:
        e = punch(j["kind"], j["side"])
        flip = bool(rev.get(j["role"], False))
        if flip and e != "0":   # ReverseDirection is ignored on some faces -> flip the profile sign instead
            e = re.sub(r",(-?[\d.]+)D\)", lambda m: f",{-float(m.group(1)):.3f}D)", e)
        motors.append({"role": j["role"], "expr": e, "reverse": False, "flipped": flip})
        print(f"  {j['role']:<20} rev={motors[-1]['reverse']!s:<5} {e}")
    m = {"scenario": scn, "duration": T_END, "fps": 100, "motors": motors,
         "times": [round(0.01 * i, 2) for i in range(int(round(T_END / 0.01)) + 1)]}   # 10 ms: accelerations
    out = out or str(PROJ / "cad" / "physics" / f"motion_{scn}.json")
    Path(out).write_text(json.dumps(m, indent=1), encoding="utf-8")
    print("->", out)


if __name__ == "__main__":
    main(*(sys.argv[1:] or ["punch"]))
