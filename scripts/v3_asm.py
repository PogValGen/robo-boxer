"""Robot-boxer v3 'Gorilla' assembly (2026-11-14): every part is modelled in the robot frame at the zero pose, so the
assembly is pure placement - no mates. Servo-frame parts (MG90S, lid, pin) are placed by the frame of their servo,
horn + cap by the horn frame of the joint (X = arm direction, Y = shaft axis).

The robot frame (x down, y left, z forward) is turned upright for SolidWorks: p_SW = (y, -x, z).

    python v3_asm.py             -> cad/v3/Robot3.SLDASM + manifest + screenshots + interference report (zero pose)
    python v3_asm.py range [joint ...]
                                 -> turn every LEFT joint through its test angles IN SOLIDWORKS and run the
                                    interference check at each pose: cad/v3/range_report.json (resumable)
    python v3_asm.py pose name=deg ...   -> one pose: interference + screenshot cad/v3/screens/Robot3_pose.png
"""
from __future__ import annotations

import asyncio
import json
import math
import re
import sys
import time

import v3_parts as vp
from v3_parts import Srv, AXL, S_B0, XR, YR, ZR0, YS, X_ANK
import os
if os.environ.get("V3CAD", "occ") == "sw":          # SolidWorks backend: only on explicit request since 2026-11-17
    from v3cad import PARTS, SHOTS, V3, base, call, mc, session
    MG90S = base.PROJ / "cad" / "purchased" / "MG90S_grabcad.SLDPRT"
else:                                                # no SolidWorks: layout()/posed() are used by v3_asm_occ.py
    from v3cad_occ import PARTS, SHOTS, V3, PROJ
    MG90S = PROJ / "cad" / "purchased" / "MG90S_grabcad.STEP"

ASM = V3 / "Robot3.SLDASM"
MANIFEST = V3 / "Robot3_manifest.json"
RANGE = V3 / "range_report.json"
IDENT = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
ROT_X180 = ((1, 0, 0), (0, -1, 0), (0, 0, -1))
ROT_Z180 = ((-1, 0, 0), (0, -1, 0), (0, 0, 1))

# kinematics: the joints between the torso (base) and each link, in order
CHAIN = {"base": [], "yaw": ["hip_yaw"], "hipblock": ["hip_yaw", "hip_roll"],
         "thigh": ["hip_yaw", "hip_roll", "hip_pitch"], "shank": ["hip_yaw", "hip_roll", "hip_pitch", "knee"],
         "ankle": ["hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch"],
         "foot": ["hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll"],
         "shoulder": ["shoulder"], "upperarm": ["shoulder", "abduction"], "forearm": ["shoulder", "abduction", "elbow"]}
SERVO_LINK = {"hip_yaw": "base", "hip_roll": "hipblock", "hip_pitch": "hipblock", "knee": "shank",
              "ankle_pitch": "ankle", "ankle_roll": "ankle", "shoulder": "base", "abduction": "shoulder",
              "elbow": "upperarm"}                                             # the link that holds the servo + lid
FORK_LINK = {"hip_yaw": "yaw", "hip_roll": "yaw", "hip_pitch": "thigh", "knee": "thigh", "ankle_pitch": "shank",
             "ankle_roll": "foot", "shoulder": "shoulder", "abduction": "upperarm", "elbow": "forearm"}   # pin, horn, cap
HORN_DIR = {"hip_yaw": (0, 0, -1), "hip_roll": (-1, 0, 0), "hip_pitch": (1, 0, 0), "knee": (-1, 0, 0),
            "ankle_pitch": (-1, 0, 0), "ankle_roll": (1, 0, 0), "shoulder": (1, 0, 0), "abduction": (1, 0, 0),
            "elbow": (1, 0, 0)}                                                # = the `toward` of the cheek pockets
TEST = {"hip_yaw": (5, 10, 15, 30, 45), "hip_roll": (3, 6, 10, 15, 20, 27, 35), "hip_pitch": (30, 45, 60, 90, 100, 110, 125),
        "knee": (30, 60, 90, 100, 110, 125), "ankle_pitch": (20, 45, 55, 70), "ankle_roll": (10, 12, 15, 20, 30),
        "shoulder": (15, 30, 45, 90, 135, 180), "abduction": (30, 60, 70, 75, 80, 90), "elbow": (45, 90, 110, 120, 130)}
LEG_JOINTS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")
# leg joints are swept with both arms held forward: a hanging fist sits 6.5 mm from the thigh and masked the leg
# limits in the first sweep (hip roll +10 "hit" was the fist)
ARMS_FWD = {"L_shoulder": -90.0, "R_shoulder": 90.0}


def G(v):
    """robot frame -> SW assembly frame (upright)"""
    return (v[1], -v[0], v[2])


def mirror(s: Srv) -> Srv:
    def m(a):
        return (("-" if a[0] == "+" else "+") + "y") if a[1] == "y" else a
    return Srv((s.pt[0], -s.pt[1], s.pt[2]), m(s.S), m(s.L))


def servos():
    left = {
        "hip_yaw": vp.srv_yaw(), "hip_roll": vp.srv_hip_roll(), "hip_pitch": vp.srv_hip_pitch(), "knee": vp.srv_knee(),
        "ankle_pitch": Srv((X_ANK, YS, 0.0), "+y", "+z"), "ankle_roll": Srv((X_ANK + XR, YR, ZR0), "+z", "-y"),
        "shoulder": vp.srv_shoulder(), "abduction": vp.srv_abd(), "elbow": vp.srv_elbow(),
    }
    out = {}
    for k, s in left.items():
        out[f"L_{k}"] = s
        out[f"R_{k}"] = mirror(s)
    return out


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def layout():
    """items: file, axes (images of the component X, Y, Z in the robot frame), origin mm, link ('base' or 'L:thigh')"""
    P = lambda n: PARTS / f"X3_{n}.SLDPRT"                                    # noqa: E731
    it = []

    def add(path, ax, o, link, label):
        it.append({"file": str(path), "axes": [list(a) for a in ax], "origin": list(o), "link": link, "label": label})
    add(P("Torso"), IDENT, (0, 0, 0), "base", "torso")
    zf = vp.BAY_Z1 + vp.BAY_W
    for i, (x, y) in enumerate((x, y) for x in vp.PCB_X for y in (-vp.PCB_Y, vp.PCB_Y)):
        add(P("Standoff"), IDENT, (x, y, zf), "base", f"board standoff {i + 1}")
    add(P("Battery"), IDENT, (0, 0, 0), "base", "battery 2x18650 (mock-up)")
    add(P("Board"), IDENT, (0, 0, 0), "base", "controller board (mock-up)")
    for side, o in (("L", "R"), ("R", "L")):
        ln = lambda k: f"{side}:{k}"                                            # noqa: E731
        add(P(f"Yoke_{side}"), IDENT, (0, 0, 0), ln("yaw"), f"{side} yoke")
        add(P(f"Block_{side}"), IDENT, (0, 0, 0), ln("hipblock"), f"{side} hip block")
        add(P("Thigh"), IDENT if side == "L" else ROT_X180, (0, 0, 0), ln("thigh"), f"{side} thigh")
        add(P(f"Shank_{side}"), IDENT, (0, 0, 0), ln("shank"), f"{side} shank")
        add(P(f"Block_{o}"), ROT_Z180, (X_ANK, 0, 0), ln("ankle"), f"{side} ankle block (= {o} block turned 180 about z)")
        add(P(f"Foot_{side}"), IDENT, (0, 0, 0), ln("foot"), f"{side} foot")
        add(P(f"Pod_{side}"), IDENT, (0, 0, 0), "base", f"{side} shoulder pod")
        add(P(f"Shoulder_{side}"), IDENT, (0, 0, 0), ln("shoulder"), f"{side} shoulder")
        add(P(f"UpperArm_{side}"), IDENT, (0, 0, 0), ln("upperarm"), f"{side} upper arm")
        add(P(f"Forearm_{side}"), IDENT, (0, 0, 0), ln("forearm"), f"{side} forearm")
        add(P("Fist"), IDENT, (0, 0 if side == "L" else -2 * vp.FIST_Y, 0), ln("forearm"), f"{side} fist")
    for name, s in servos().items():
        side, j = name[0], name[2:]
        lk = lambda k: k if k == "base" else f"{side}:{k}"                      # noqa: E731
        ax = (s.lv, s.sv, s.tv)
        add(MG90S, ax, s.o, lk(SERVO_LINK[j]), f"{name} MG90S")
        add(P("Lid"), ax, s.o, lk(SERVO_LINK[j]), f"{name} lid")
        add(P("Pin"), ax, s.p(AXL, S_B0, 0.0), lk(FORK_LINK[j]), f"{name} pin")
        d = HORN_DIR[j]
        hax = (d, tuple(s.sv), cross(d, s.sv))                                   # X = arm, Y = shaft, Z = X x Y
        add(P("Horn"), hax, s.pt, lk(FORK_LINK[j]), f"{name} horn")
        add(P("Cap"), hax, s.pt, lk(FORK_LINK[j]), f"{name} cap")
    return it


# ----------------------------------------------------------------------------- forward kinematics
def rot(v, d, a):
    """vector v turned by a (rad) about the unit direction d"""
    c, s = math.cos(a), math.sin(a)
    k = cross(d, v)
    dv = sum(d[i] * v[i] for i in range(3))
    return tuple(v[i] * c + k[i] * s + d[i] * dv * (1 - c) for i in range(3))


def posed(item, angles, srv=None):
    """axes and origin of an item for the joint angles {'L_knee': deg, ...}"""
    srv = srv or servos()
    ax, o = [tuple(a) for a in item["axes"]], tuple(item["origin"])
    if item["link"] == "base":
        return ax, o
    side, link = item["link"].split(":")
    for j in reversed(CHAIN[link]):                       # innermost joint first
        a = math.radians(angles.get(f"{side}_{j}", 0.0))
        if not a:
            continue
        s = srv[f"{side}_{j}"]
        ax = [rot(v, s.sv, a) for v in ax]
        r = rot(tuple(o[i] - s.pt[i] for i in range(3)), s.sv, a)
        o = tuple(s.pt[i] + r[i] for i in range(3))
    return ax, o


def sw_array(ax, o):
    return [float(c) for a in ax for c in G(a)] + [v / 1000.0 for v in G(o)] + [1.0, 0.0, 0.0, 0.0]


# ----------------------------------------------------------------------------- SolidWorks
def components():
    ad = mc.typed(base.model(), "IAssemblyDoc")
    return {mc.typed(c, "IComponent2").Name2: mc.typed(c, "IComponent2") for c in ad.GetComponents(True) or []}


def apply_pose(items, angles, comps, only_side=None):
    import pythoncom
    from win32com.client import VARIANT
    mu = mc.typed(base.adapter.swApp.GetMathUtility(), "IMathUtility")
    srv = servos()
    for it in items:
        if it["link"] == "base" or (only_side and not it["link"].startswith(only_side)):
            continue
        ax, o = posed(it, angles, srv)
        comps[it["name"]].Transform2 = mu.CreateTransform(VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, sw_array(ax, o)))
    mc.typed(base.model(), "IModelDoc2").EditRebuild3()


MEM_LIMIT_MB = 4500


def sw_mem_mb():
    """working set of SLDWORKS, MB (tasklist)"""
    import subprocess
    try:
        out = subprocess.run(["tasklist", "/FI", "IMAGENAME eq SLDWORKS.exe", "/FO", "CSV", "/NH"],
                             capture_output=True, text=True).stdout
        return max(int(re.sub(r"\D", "", ln.split('","')[-1])) // 1024 for ln in out.splitlines() if "SLDWORKS" in ln)
    except Exception:                                        # noqa: BLE001
        return 0


def mem_ok():
    """2026-11-15: 80 poses in one SolidWorks session grew it to 12.5 GB and Windows ran out of memory (user: 'low
    memory'). The pose loops are resumable: stop here, restart SolidWorks, run the same command again."""
    m = sw_mem_mb()
    if m > MEM_LIMIT_MB:
        print(f"  MEMORY STOP: SLDWORKS {m} MB > {MEM_LIMIT_MB} - restart SolidWorks and rerun (resumes)", flush=True)
        return False
    return True


def strip(n):
    return re.sub(r"-\d+$", "", n)


async def interference():
    r = await call("check_interference", {})
    out = {}
    for it in r.get("interferences", []):
        a, b = sorted(it["components"])
        out[(a, b)] = float(it["volume_mm3"])
    return out


def real(pairs, baseline=None):
    """interferences that are not the expected ones: pin press fits (<= 3.2 mm3), touches (0), the zero-pose set"""
    bad = {}
    for (a, b), v in pairs.items():
        if v < 0.01:
            continue
        if ("X3_Pin" in a or "X3_Pin" in b) and v <= 3.2:
            continue
        if baseline is not None and (a, b) in baseline and v <= baseline[(a, b)] + 0.3:
            continue
        bad[f"{a} x {b}"] = round(v, 2)
    return bad


async def build():
    items = layout()
    await call("create_assembly", {"name": "Robot3"})
    for i, it in enumerate(items):
        rot9 = [float(c) for a in it["axes"] for c in G(a)]
        x, y, z = G(it["origin"])
        r = await call("insert_component", {"file_path": it["file"], "x": x, "y": y, "z": z, "rotation": rot9})
        it["name"] = r["component"]["name"]
        if (i + 1) % 10 == 0 or i + 1 == len(items):
            print(f"  [{i + 1}/{len(items)}] {it['label']}: {it['name']}", flush=True)
    MANIFEST.write_text(json.dumps(items, indent=1), encoding="utf-8")
    asm = base.model()
    ad = mc.typed(asm, "IAssemblyDoc")
    n = 0
    for c in ad.GetComponents(True) or []:
        c = mc.typed(c, "IComponent2")
        if not c.IsFixed():
            c.Select4(False, None, False)
            ad.FixComponent()
            n += 1
    mc.typed(asm, "IModelDoc2").ClearSelection2(True)
    mc.typed(asm, "IModelDoc2").EditRebuild3()
    print(f"  fixed {n} components", flush=True)
    pairs = await interference()
    (V3 / "interference.json").write_text(json.dumps({f"{a} x {b}": v for (a, b), v in pairs.items()}, indent=1), encoding="utf-8")
    print(f"  interference: {len(pairs)} pairs, unexpected: {real(pairs)}", flush=True)
    await call("save_as", {"file_path": str(ASM), "overwrite": True})
    for v in ("isometric", "front", "right", "back"):
        try:
            await call("export_image", {"file_path": str(SHOTS / f"Robot3_{v}.png"), "format_type": "png",
                                        "view_orientation": v})
        except RuntimeError as e:
            print("   WARN shot", v, str(e)[:100], flush=True)
    await call("save_as", {"file_path": str(ASM), "overwrite": True})
    await call("close_model", {"save": False})


async def open_asm():
    await call("open_model", {"file_path": str(ASM)})
    items = json.loads(MANIFEST.read_text(encoding="utf-8"))
    comps = components()
    missing = [it["name"] for it in items if it["name"] not in comps]
    assert not missing, f"components not found: {missing[:5]}"
    return items, comps


async def joint_range(joints):
    """one LEFT joint at a time, both directions; stops a direction at the first collision"""
    rep = json.loads(RANGE.read_text(encoding="utf-8")) if RANGE.exists() else {}
    items, comps = await open_asm()
    try:
        t0 = time.perf_counter()
        baseline = await interference()
        print(f"  zero pose: {len(baseline)} pairs, unexpected {real(baseline)}  ({time.perf_counter() - t0:.0f} s)", flush=True)
        for j in joints:
            if not mem_ok():
                break
            pose0 = dict(ARMS_FWD) if j in LEG_JOINTS else {}
            apply_pose(items, pose0, comps)
            for sgn in (1, -1):
                for a in TEST[j]:
                    key = f"{j} {sgn * a:+d}"
                    if key in rep:
                        if rep[key]:
                            break
                        continue
                    t0 = time.perf_counter()
                    apply_pose(items, {**pose0, f"L_{j}": sgn * a}, comps, only_side="L")
                    bad = real(await interference(), baseline)
                    rep[key] = bad
                    RANGE.write_text(json.dumps(rep, indent=1), encoding="utf-8")
                    print(f"  {key:22s} {'free' if not bad else 'HIT ' + json.dumps(bad)[:200]}  ({time.perf_counter() - t0:.0f} s)", flush=True)
                    if bad:
                        break
            apply_pose(items, {}, comps)
    finally:
        await call("close_model", {"save": False})


async def one_pose(angles):
    items, comps = await open_asm()
    try:
        baseline = await interference()
        apply_pose(items, angles, comps)
        bad = real(await interference(), baseline)
        print("  pose", angles, "->", "free" if not bad else bad, flush=True)
        await call("export_image", {"file_path": str(SHOTS / "Robot3_pose.png"), "format_type": "png",
                                    "view_orientation": "isometric"})
    finally:
        await call("close_model", {"save": False})


ROLE_JOINT = {"hipyaw": "hip_yaw", "hiproll": "hip_roll", "hippitch": "hip_pitch", "knee": "knee", "ankle": "ankle_pitch",
              "ankleroll": "ankle_roll", "shoulder": "shoulder", "shoulderroll": "abduction", "elbow": "elbow"}


def from_roles(cmd):
    """{'legA_knee': deg, ...} (physics roles: same angle = mirrored motion) -> v3_asm angles about the servo shafts"""
    out = {}
    for role, deg in cmd.items():
        limb, kind = role.split("_")
        if kind not in ROLE_JOINT:
            continue
        side = "L" if limb.endswith("A") else "R"
        out[f"{side}_{ROLE_JOINT[kind]}"] = deg if side == "L" else -deg
    return out


async def pose_list(path):
    """whole-robot poses [[label, {role: deg}], ...] (both sides at once) -> interference at each one, in SolidWorks;
    cad/v3/poses_report.json (resumable). Catches leg-leg, arm-torso, foot-foot hits that one-joint sweeps cannot."""
    poses = json.loads(open(path, encoding="utf-8").read())
    out_f = V3 / "poses_report.json"
    rep = json.loads(out_f.read_text(encoding="utf-8")) if out_f.exists() else {}
    items, comps = await open_asm()
    try:
        baseline = await interference()
        for i, (label, cmd) in enumerate(poses):
            if label in rep:
                continue
            if not mem_ok():
                break
            t0 = time.perf_counter()
            apply_pose(items, from_roles(cmd), comps)
            bad = real(await interference(), baseline)
            rep[label] = bad
            out_f.write_text(json.dumps(rep, indent=1), encoding="utf-8")
            print(f"  [{i + 1}/{len(poses)}] {label:28s} {'free' if not bad else 'HIT ' + json.dumps(bad)[:220]}"
                  f"  ({time.perf_counter() - t0:.0f} s)", flush=True)
            if "--shots" in sys.argv:
                await call("export_image", {"file_path": str(SHOTS / f"pose_{label}.png"), "format_type": "png",
                                            "view_orientation": "isometric"})
    finally:
        await call("close_model", {"save": False})


async def main(argv):
    async with session():
        if argv[:1] == ["range"]:
            await joint_range(argv[1:] or list(TEST))
        elif argv[:1] == ["poses"]:
            await pose_list(argv[1])
        elif argv[:1] == ["pose"]:
            await one_pose({k: float(v) for k, v in (a.split("=") for a in argv[1:])})
        else:
            await build()


if __name__ == "__main__":
    if "list" in sys.argv:
        for it in layout():
            print(it["label"], it["link"], [round(v, 2) for v in it["origin"]])
    else:
        asyncio.run(main(sys.argv[1:]))
