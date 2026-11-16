"""v3 'Gorilla' -> spec for the SOLIDWORKS Motion model (tools/physics/build_physics.exe build), same schema and role
names as v2 (gen_phys_spec.py), so leg_fk / gaits / servo_sim work with ROBO_V=3.

    python v3_phys_spec.py            -> cad/v3/physics/phys_spec.json (+ physics copies of the parts, masses not set)
    python v3_phys_spec.py mass       -> set the printed masses on the physics copies (needs SolidWorks + massprops.json)

Links are named after the joint that moves them: legA_hipyaw (yoke), legA_hiproll (hip block), legA_hippitch (thigh),
legA_knee (shank), legA_ankle (ankle block), legA_ankleroll (foot), armA_shoulder, armA_shoulderroll, armA_elbow.
A = left (+y), B = right. Joint = Concentric(pin D6 of the fork link, pin hole D6.3 of the servo link) +
Coincident(base plane normal to the axis in both links: all parts are modelled in the robot frame).
Angle sense: right-hand rotation of the rotor about `axis` = the servo shaft direction on the left side and the
opposite of it on the right side (same angle = mirrored motion). v3_asm angles are about the shaft: right = -role.
"""
from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import numpy as np

import v3_asm as A
import v3_parts as vp

PHYS = A.V3 / "physics"
PPARTS = PHYS / "parts"
OUT = PHYS / "phys_spec.json"
SERVO_PHYS = A.base.PROJ / "cad" / "physics" / "MG90S_phys.SLDPRT"       # GrabCAD copy with the datasheet mass 13.4 g
ROLE = {"hip_yaw": "hipyaw", "hip_roll": "hiproll", "hip_pitch": "hippitch", "knee": "knee", "ankle_pitch": "ankle",
        "ankle_roll": "ankleroll", "shoulder": "shoulder", "abduction": "shoulderroll", "elbow": "elbow"}
LINK_JOINT = {"yaw": "hip_yaw", "hipblock": "hip_roll", "thigh": "hip_pitch", "shank": "knee", "ankle": "ankle_pitch",
              "foot": "ankle_roll", "shoulder": "shoulder", "upperarm": "abduction", "forearm": "elbow"}
LEG = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")
PLANE = {0: "Right Plane", 1: "Top Plane", 2: "Front Plane"}            # base plane normal to the robot axis
MASS_G = {"X3_Battery": 104.0, "X3_Board": 25.0, "X3_Horn": 0.6}        # purchased (battery / board: verify)
PRINT_FACTOR = 0.55 * 1.24 / 1.04       # printed PLA / solid ABS of the model (3 perimeters + 30 %); the user prints PLA
#                                         (2026-11-15; was 0.55 = ABS). Rerun `v3_phys_spec.py mass` + `servo_sim.py build` after a change.
TPU = ("X3_Fist",)


def limb(side, joint):
    return ("leg" if joint in LEG else "arm") + ("A" if side == "L" else "B")


def link_name(link):
    if link == "base":
        return "base"
    side, k = link.split(":")
    j = LINK_JOINT[k]
    return f"{limb(side, j)}_{ROLE[j]}"


def phys_path(p):
    p = Path(p)
    if p.stem == "MG90S_grabcad":
        return str(SERVO_PHYS)
    return str(PPARTS / p.name)


def main():
    PPARTS.mkdir(parents=True, exist_ok=True)
    items = A.layout()
    srv = A.servos()
    parts = []
    for it in items:
        src = Path(it["file"])
        dst = Path(phys_path(src))
        if src.stem != "MG90S_grabcad" and (not dst.exists() or dst.stat().st_mtime < src.stat().st_mtime):
            shutil.copy2(src, dst)                           # geometry changed -> fresh copy (mass must be set again)
        m = [float(c) for a in it["axes"] for c in a] + [v / 1000.0 for v in it["origin"]] + [1.0, 0.0, 0.0, 0.0]
        parts.append({"name": it["label"], "stem": dst.stem, "path": str(dst), "m": m, "link": link_name(it["link"])})
    idx = {p["name"]: i for i, p in enumerate(parts)}
    # anchor = the printed structural part of the link
    pref = ["X3_Torso", "X3_Yoke", "X3_Block", "X3_Thigh", "X3_Shank", "X3_Foot", "X3_Shoulder", "X3_UpperArm", "X3_Forearm"]
    rank = lambda i: next((k for k, s in enumerate(pref) if parts[i]["stem"].startswith(s)), 99)   # noqa: E731
    anchors = {}
    for i, p in enumerate(parts):
        if p["link"] not in anchors or rank(i) < rank(anchors[p["link"]]):
            anchors[p["link"]] = i
    assert all(rank(i) < 99 for i in anchors.values()), {L: parts[i]["stem"] for L, i in anchors.items()}
    joints = []
    for name, s in srv.items():
        side, j = name[0], name[2:]
        role = f"{limb(side, j)}_{ROLE[j]}"
        lk = lambda k: "base" if k == "base" else link_name(f"{side}:{k}")      # noqa: E731
        fork, serv = lk(A.FORK_LINK[j]), lk(A.SERVO_LINK[j])
        # the DISTAL link (named after the joint) is the rotor: for hip roll and ankle pitch that is the link with
        # the servo, the fork belongs to the proximal one
        rotor, stator = (fork, serv) if fork == role else (serv, fork)
        assert rotor == role, (fork, serv, role)
        # the part with the servo pocket (pin hole): the link anchor, except the shoulder servo - it sits in the pod
        pin, blk = idx[f"{name} pin"], (idx[f"{side} shoulder pod"] if j == "shoulder" else anchors[serv])
        # pin shaft r 3.0 (s -18.7..-13.0), block hole r 3.15 (s -15.4..-12.8)
        p_pin = [v / 1000.0 for v in s.p(vp.AXL + vp.PIN_D / 2, vp.S_BOT - 1.0, 0.0)]
        p_hole = [v / 1000.0 for v in s.p(vp.AXL + vp.PIN_HOLE / 2, vp.S_BOT + 1.5, 0.0)]
        lever, p_l, other, p_o = (pin, p_pin, blk, p_hole) if fork == role else (blk, p_hole, pin, p_pin)
        ax = int(np.argmax(np.abs(s.sv)))
        joints.append({"role": role, "kind": ROLE[j], "side": "A" if side == "L" else "B", "rotor": rotor, "stator": stator,
                       # lever = the component of the ROTOR link that carries the motor face (p_spigot)
                       "lever": lever, "housing": anchors[stator], "bridge": other,
                       "plane_a": anchors[rotor], "plane_b": anchors[stator], "plane_name": PLANE[ax],
                       "p_spigot": p_l, "p_cup": p_o,
                       # right side: axis = -shaft, so the SAME angle gives the MIRRORED motion on both sides
                       "axis": [float(v) * (1 if side == "L" else -1) for v in s.sv],
                       "pivot_m": [v / 1000.0 for v in s.pt]})
    sole = {sd: [vp.X_SOLE, sg * vp.YL, 4.5] for sd, sg in (("A", 1), ("B", -1))}          # sole plate z -38..47
    spec = {"parts": parts, "anchors": anchors, "joints": joints, "fixed_link": "base", "sole0": sole,
            "foot_stem": "X3_Foot"}
    OUT.write_text(json.dumps(spec, indent=1), encoding="utf-8")
    from collections import Counter
    print(f"parts={len(parts)} links={len(anchors)} joints={len(joints)} -> {OUT}")
    print("parts per link:", dict(Counter(p["link"] for p in parts)))
    print("anchors:", {L: parts[i]["stem"] for L, i in anchors.items()})


def set_masses():
    """printed mass = PRINT_FACTOR x solid ABS on the physics copies; purchased parts by MASS_G"""
    import subprocess
    exe = A.base.PROJ / "tools" / "physics" / "build_physics.exe"
    mp = json.loads((PHYS / "massprops_solid.json").read_text(encoding="utf-8"))["parts"]
    solid = {Path(p["path"]).stem: p["m"] * 1000 for p in mp}
    tot = 0.0
    for f in sorted(PPARTS.glob("*.SLDPRT")):
        st = f.stem
        g = MASS_G.get(st) or solid[st] * PRINT_FACTOR * (1.2 / 1.04 if st in TPU else 1.0)
        out = subprocess.run([str(exe), "massfix", str(f), f"{g:.2f}"], capture_output=True, text=True).stdout
        line = [l for l in out.splitlines() if "mass" in l][-1:]
        print(f"  {st:<16} {g:7.2f} g  {line}", flush=True)
        tot += g
    print(f"set {len(list(PPARTS.glob('*.SLDPRT')))} part files")


if __name__ == "__main__":
    set_masses() if sys.argv[1:] == ["mass"] else main()
