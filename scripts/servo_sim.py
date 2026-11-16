"""robo-boxer in SOLIDWORKS Motion with datasheet servo models (MCP recipe solidworks_mcp.recipes.motion).

Every one of the 15 joints is a MG90S model, not an ideal motor: torque limited to stall
(0.177 N*m @4.8 V), torque falls with speed (no-load 600 deg/s), travel +-90 deg, dead band 1 us
(TowerPro datasheet; travel and regulator band are not published -> verify). The servo loop runs
inside the solver at `fps` (200 Hz, SW2022 maximum). Optional floor with foot contacts and a free pelvis = real
standing/walking; otherwise the pelvis is fixed.

Angles: rotor relative to stator about the servo axis of phys_spec.json (servo local +Y), the same
measure check_phys.rel_angle uses. `sign` per joint (cad/physics/servo_sign.json) maps it to the
torque face axis; `calib` finds it.

Usage (venv solidworks-mcp):
  python servo_sim.py calib                      # fixed pelvis, +15 deg on every joint -> servo_sign.json
  python servo_sim.py punch [--fixed] [--v6]     # guard + right jab with waist twist
  python servo_sim.py walk  [--fixed] [--v6] [--steps N]
  python servo_sim.py <scn> --save               # keep the study in Robot_Physics.SLDASM (Play in SW)
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

PROJ = Path(__file__).resolve().parents[1]
import os  # noqa: E402
VER = os.environ.get("ROBO_V", "1")
V2, V3 = VER == "2", VER == "3"
PHYS = PROJ / "cad" / {"2": "v2/physics", "3": "v3/physics"}.get(VER, "physics")
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, r"D:\AI prodject\solidworks-mcp\SolidworksMCP-python\src")
from check_phys import rel_angle  # noqa: E402
from solidworks_mcp.recipes import motion as RM  # noqa: E402
import gaits  # noqa: E402

ASM = PHYS / "Robot_Servo.SLDASM"         # upright copy: SW +Y up, robot faces +Z (Front view)
SPEC_UP = PHYS / "phys_spec_up.json"
# robot frame (x down, y left, z forward) -> SW frame: p_sw = p_r @ Q  (x_r -> -Y, y_r -> +X, z_r -> +Z)
Q = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
DOWN, SIDE, FWD = Q[0], Q[1], Q[2]
BUILD_EXE = PROJ / "tools" / "physics" / "build_physics.exe"
SIGN_FILE = PHYS / "servo_sign.json"
FLOOR = PHYS / "Floor.SLDPRT"
def sole_x() -> float:
    """Lowest point of the feet (robot x = down, m) from the viewer json the spec was made from."""
    import base64, gzip
    if V3:                                         # v3: the sole centre is written into the spec
        return json.loads((PHYS / "phys_spec.json").read_text(encoding="utf-8"))["sole0"]["A"][0] / 1000.0
    d = json.loads(gaits_src().read_text(encoding="utf-8"))
    import build_anim as BA
    xs = []
    for it in d["inst"]:
        if BA.stem_of(it["name"]) == "Leg_Foot":
            mp = d["meshes"][it["p"]]
            v = BA.stl_tris(gzip.decompress(base64.b64decode(mp["gz"]))).reshape(-1, 3) + np.array(mp["off"])
            R, T = np.array(it["m"][:9]).reshape(3, 3), np.array(it["m"][9:12]) * 1000
            xs.append((v @ R + T)[:, 0].max())
    return max(xs) / 1000.0


def gaits_src():
    import build_anim as BA
    return BA.SRC


SOLE_X = None            # set in main() (v1: 0.1525)
FLOOR_GAP = 0.0005


def build_upright():
    """phys_spec.json (robot frame) -> phys_spec_up.json (SW frame) -> Robot_Servo.SLDASM."""
    import subprocess
    sp = json.loads((PHYS / "phys_spec.json").read_text(encoding="utf-8"))   # gen_phys_spec.py output
    for p in sp["parts"]:
        m = p["m"]; R = np.array(m[:9]).reshape(3, 3) @ Q; T = np.array(m[9:12]) @ Q
        p["m"] = [*R.ravel().tolist(), *T.tolist(), *m[12:16]]
    for j in sp["joints"]:
        for k in ("p_spigot", "p_cup", "axis", "pivot_m"):
            j[k] = (np.array(j[k]) @ Q).tolist()
    SPEC_UP.write_text(json.dumps(sp, indent=1), encoding="utf-8")
    if ASM.exists():
        ASM.unlink()
    r = subprocess.run([str(BUILD_EXE), "build", str(SPEC_UP), str(ASM)], capture_output=True, text=True)
    print(r.stdout[-1500:])
    if r.returncode:
        raise SystemExit("build failed")
    print(RM.mass_props(ASM, MASS_SRC)["mass_kg"] * 1000, "g total (massprops)")


def comp_names(spec):
    """spec part index -> SW component name (insertion order = SW numbering per file stem)."""
    cnt, out = {}, []
    for p in spec["parts"]:
        st = Path(p["path"]).stem
        cnt[st] = cnt.get(st, 0) + 1
        out.append(f"{st}-{cnt[st]}")
    return out


MASS_SRC = PHYS / "massprops.json"   # SW mass properties per component (sw_motion massprops, made by `build`)


def joint_inertias(spec):
    """Inertia (kg*m^2) of everything distal to each joint about its servo axis, initial pose."""
    parts = spec["parts"]
    if not MASS_SRC.exists():
        RM.mass_props(ASM, MASS_SRC)
    byname = {p["name"]: p for p in json.loads(MASS_SRC.read_text(encoding="utf-8"))["parts"]}
    mp = {i: byname[n] for i, n in enumerate(comp_names(spec)) if n in byname}
    child = {}
    for j in spec["joints"]:
        child.setdefault(j["stator"], []).append(j["rotor"])

    def sub(L):
        out = [L]
        for c in child.get(L, []):
            out += sub(c)
        return out
    out = {}
    for j in spec["joints"]:
        links = set(sub(j["rotor"])); a = np.array(j["axis"]) / np.linalg.norm(j["axis"]); piv = np.array(j["pivot_m"])
        I = 0.0
        for i, p in enumerate(parts):
            if p["link"] not in links or i not in mp:
                continue
            R = np.array(p["m"][:9]).reshape(3, 3); T = np.array(p["m"][9:12])
            Ip = np.array(mp[i]["I"]).reshape(3, 3) * (13.4 / 9.0 if "MG90S" in p["stem"] else 1.0)
            cw = np.array(mp[i]["cm"]) @ R + T; Iw = R.T @ Ip @ R
            r = cw - piv; r_perp = r - (r @ a) * a
            I += a @ Iw @ a + mp[i]["m"] * (r_perp @ r_perp)
        out[j["role"]] = I
    return out


CONTACT = {"stiffness": 1e7, "exponent": 1.5, "damping": 2000, "penetration": 1e-4, "mu_s": 0.5, "mu_d": 0.4}
SOLVER = {"max_step": 1e-3, "precise_contact": True}


FLOOR_BIG = PHYS / "Floor_600.SLDPRT"
# parts that can touch the floor when the robot lies / gets up (one contact PAIR per part: robot parts must not
# collide with each other, a single contact group would make every pair of them a contact)
LIE_CONTACT_STEMS = ("Leg_Foot", "Fist_TPU", "Pelvis", "Torso_Mount", "Shoulder_Yoke", "Controller_RTrobot32",
                     "Battery_2x18650", "J2_Housing")
LIE = None               # None | "back" | "chest" (set in main)


def lie_setup(spec, names):
    """Robot starts lying: gravity along SW -Z (back) / +Z (chest), floor plane under the back / chest."""
    import wb_fk
    P, _ = wb_fk.Body(cell=4.0).points({})                       # robot frame mm, z forward (= SW z)
    gap = FLOOR_GAP
    if not FLOOR_BIG.exists():
        print(RM.make_box(FLOOR_BIG, 0.01, 0.6, 0.6))
    if LIE == "back":
        z0 = P[:, 2].min() / 1000 - gap; R = [[0, 0, -1], [0, 1, 0], [1, 0, 0]]; grav = {"axis": "Z", "reverse": True}
    else:
        z0 = P[:, 2].max() / 1000 + gap; R = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]; grav = {"axis": "Z", "reverse": False}
    # floor centred under the robot's length (SW y = -robot x)
    yc = -(P[:, 0].min() + P[:, 0].max()) / 2000
    insert = [{"path": str(FLOOR_BIG), "m": [*np.array(R, float).ravel().tolist(), 0.0, yc, z0, 1, 0, 0, 0]}]
    parts = [names[i] for i, p in enumerate(spec["parts"]) if p["stem"] in LIE_CONTACT_STEMS]
    contacts = [{"name": f"C_{n}", "components": [n, "Floor_600-1"], **CONTACT} for n in parts]
    return insert, grav, contacts


def build_spec(spec, scn, cmds, duration, fixed, preset, save, fps=200):
    names = comp_names(spec)
    anchors = {L: names[i] for L, i in spec["anchors"].items()}
    sign = json.loads(SIGN_FILE.read_text()) if SIGN_FILE.exists() else {}
    servos = []
    inert = joint_inertias(spec)
    for j in spec["joints"]:
        r = j["role"]
        servos.append(RM.servo(
            f"S_{r}", {"component": names[j["lever"]], "point": j["p_spigot"]},
            anchors[j["stator"]],
            cmds.get(r, [[0.0, 0.0]]), preset, sign=sign.get(r, 1), inertia_kgm2=inert[r]))
    s = {"assembly": str(ASM), "study": f"Servo_{scn}", "save": save, "duration": duration, "fps": fps,
         "sample_dt": 0.01, "gravity": {"axis": "Y", "reverse": True},
         "solver": dict(SOLVER), "servos": servos,
         "sample_components": sorted(set(anchors.values()))}
    if not fixed:
        if not FLOOR.exists():
            print(RM.make_box(FLOOR, 0.01, 0.4, 0.4))
        s["insert"] = [{"path": str(FLOOR), "m": [*Q.ravel().tolist(), *((SOLE_X + FLOOR_GAP) * DOWN).tolist(), 1, 0, 0, 0]}]
        s["unfix"] = [anchors["base"]]
        s["fix"] = ["Floor-1"]
        feet = [names[i] for i, p in enumerate(spec["parts"]) if p["stem"].startswith(spec.get("foot_stem", "Leg_Foot"))]
        s["contacts"] = [{"name": "C_feet_floor", "components": feet + ["Floor-1"], **CONTACT}]
        s["sample_components"] = sorted(set(anchors.values()) | {"Floor-1"})
        if LIE:
            s["insert"], s["gravity"], s["contacts"] = lie_setup(spec, names)
            s["fix"] = ["Floor_600-1"]
            s["sample_components"] = sorted(set(anchors.values()) | {"Floor_600-1"})
    return s, anchors


def analyse(spec, anchors, res, cmds, label):
    J = {j["role"]: j for j in spec["joints"]}
    poses = res["poses"]; ts = sorted(poses, key=float)
    P = {t: {L: poses[t][c] for L, c in anchors.items()} for t in ts}
    t0 = ts[0]
    out = {"label": label, "joints": {}, "base": {}}
    print(f"\n== {label}: solver {res['seconds']:.0f} s, servo steps {res['control_steps']}")
    print(f"  {'joint':<20} {'peak N*m':>8} {'%stall':>6} {'sat s':>6} {'max err':>7} {'end cmd':>7} {'end got':>7}")
    for r, j in J.items():
        sv = res["servos"][f"S_{r}"]; rows = np.array(sv["rows"])
        got = rel_angle(P[t0][j["rotor"]], P[t0][j["stator"]], P[ts[-1]][j["rotor"]], P[ts[-1]][j["stator"]], j["axis"])
        cmd_end = gaits.eval_cmd(cmds.get(r, [[0, 0]]), float(ts[-1]))
        err = float(np.max(np.abs(rows[:, 1] - rows[:, 2]))) if len(rows) else 0.0
        pct = 100 * sv["peak_nm"] / sv["stall_nm"]
        out["joints"][r] = {"peak_nm": sv["peak_nm"], "pct_stall": pct, "sat_s": sv["sat_s"], "max_err_deg": err,
                            "end_cmd": cmd_end, "end_got": got}
        flag = "!! stall" if pct >= 99.9 else ("! >80%" if pct > 80 else "")
        print(f"  {r:<20} {sv['peak_nm']:8.3f} {pct:6.0f} {sv['sat_s']:6.2f} {err:7.1f} {cmd_end:7.1f} {got:7.1f} {flag}")
    b = np.array([P[t]["base"] for t in ts])
    pos = b[:, 9:12] * 1000
    R0 = np.array(b[0][:9]).reshape(3, 3)
    tilt = [math.degrees(math.acos(max(-1, min(1, (DOWN @ R0.T @ np.array(m[:9]).reshape(3, 3)) @ DOWN)))) for m in b]  # body down vs world down
    def yaw_of(m):
        z0, z = FWD, FWD @ R0.T @ np.array(m[:9]).reshape(3, 3)   # body forward, now (row-vector frames)
        return math.degrees(math.atan2(np.cross(z0, z) @ DOWN, z0 @ z))
    yaw = [yaw_of(m) for m in b]
    dd = pos[-1] - pos[0]
    d = np.array([dd @ DOWN, dd @ SIDE, dd @ FWD])
    drop = (pos - pos[0]) @ DOWN
    fell = max(tilt) > 40 or drop.max() > 50
    out["base"] = {"dx_down_mm": float(d[0]), "dy_side_mm": float(d[1]), "dz_fwd_mm": float(d[2]),
                   "max_tilt_deg": float(max(tilt)), "end_yaw_deg": float(yaw[-1]), "fell": bool(fell),
                   "traj": [[float(t), float((p - pos[0]) @ FWD), float((p - pos[0]) @ SIDE), float((p - pos[0]) @ DOWN), float(a)]
                            for t, p, a in zip(ts, pos, tilt)]}
    print(f"  pelvis: forward {d[2]:+.1f} mm, side {d[1]:+.1f} mm, down {d[0]:+.1f} mm, max tilt {max(tilt):.1f} deg, "
          f"yaw {yaw[-1]:+.1f} deg -> {'УПАЛ' if fell else 'стоит'}")
    # orientation vs gravity at every sample: upright = body down (robot +x) . gravity (1 = standing)
    g = {None: DOWN, "back": np.array([0, 0, -1.0]), "chest": np.array([0, 0, 1.0])}[LIE]
    up = [float((DOWN @ R0.T @ np.array(m[:9]).reshape(3, 3)) @ g) for m in b]
    hgt = [float(-(p @ g)) for p in pos]                          # pelvis origin height along -g (mm, floor-relative offset)
    out["base"]["upright"] = [[float(t), u, h - hgt[0]] for t, u, h in zip(ts, up, hgt)]
    out["base"]["upright_end"] = up[-1]
    print("  upright (1 = standing) / pelvis rise mm, every 0.5 s:")
    for t, u, h in zip(ts, up, hgt):
        if abs(float(t) * 2 - round(float(t) * 2)) < 1e-6:
            print(f"    {float(t):5.1f}  {u:+.2f}  {h - hgt[0]:+7.1f}")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario")
    ap.add_argument("--fixed", action="store_true")
    ap.add_argument("--v6", action="store_true", help="MG90S at 6 V (datasheet 2.2 kgf*cm @6.6 V, 0.08 s/60 @6 V)")
    ap.add_argument("--save", action="store_true")
    ap.add_argument("--reuse", action="store_true", help="analyse existing servo_<scn>_result.json, no solver run")
    ap.add_argument("--lie", choices=["back", "chest"], help="start lying on the back / chest (get-up scenarios)")
    ap.add_argument("--steps", type=int, default=4)
    ap.add_argument("--tag", default="")
    ap.add_argument("--fps", type=int, default=200)
    ap.add_argument("--contact", action="append", default=[], help="contact k=v (stiffness, damping, penetration, exponent, mu_s, mu_d)")
    ap.add_argument("--solver", action="append", default=[], help="solver k=v (max_step, error_tolerance, precise_contact=1)")
    ap.add_argument("--param", action="append", default=[], help="gait parameter k=v")
    a = ap.parse_args()
    global SOLE_X, LIE
    LIE = a.lie
    SOLE_X = sole_x()
    print(f"[0/3] v{VER}: sole at robot x = {SOLE_X * 1000:.1f} mm, physics dir {PHYS}")
    if a.scenario == "build" or not ASM.exists() or not SPEC_UP.exists():
        build_upright()
        if a.scenario == "build":
            return
    spec = json.loads(SPEC_UP.read_text(encoding="utf-8"))
    preset = "MG90S@6V" if a.v6 else "MG90S@4.8V"
    par = {k: float(v) for k, v in (p.split("=") for p in a.param)}
    CONTACT.update({k: float(v) for k, v in (p.split("=") for p in a.contact)})
    SOLVER.update({k: (bool(int(v)) if k == "precise_contact" else float(v)) for k, v in (p.split("=") for p in a.solver)})
    fixed = a.fixed or a.scenario == "calib"
    cmds, dur = gaits.scenario(a.scenario, fixed=fixed, steps=a.steps, **par)
    scn = a.scenario + ("_fixed" if fixed and a.scenario != "calib" else "") + ("_6v" if a.v6 else "") + a.tag
    s, anchors = build_spec(spec, scn, cmds, dur, fixed, preset, a.save, a.fps)
    sp = PHYS / f"servo_{scn}.json"
    sp.write_text(json.dumps(s, indent=1), encoding="utf-8")
    print(f"[1/3] spec {sp.name}: {len(s['servos'])} servos ({preset}), {'fixed pelvis' if fixed else 'floor + free pelvis'}, {dur} s")
    print("[2/3] SOLIDWORKS Motion ...", flush=True)
    rp = PHYS / f"servo_{scn}_result.json"
    if a.reuse and rp.exists():   # analyse an earlier solver result (e.g. run crashed only while closing docs)
        res = json.loads(rp.read_text(encoding="utf-8")); res["log"] = f"(reused {rp.name})"; res["result_path"] = str(rp)
    else:
        res = RM.run_study(sp, rp)
    print("\n".join(l for l in res["log"].splitlines() if not l.startswith("  S_")))
    print("[3/3] analysis")
    out = analyse(spec, anchors, res, cmds, scn)
    out["contact"] = dict(CONTACT); out["solver"] = dict(SOLVER)
    noise = {r: float(np.std(np.diff([x[4] for x in res["servos"][f"S_{r}"]["rows"]]))) for r in ("legA_ankle", "legA_knee", "legA_hippitch")}
    out["torque_step_noise"] = noise
    print("  torque noise (std of 10 ms differences):", {k: round(v, 4) for k, v in noise.items()})
    out["params"] = par; out["preset"] = preset
    # summary first: a COM failure while saving the view must not lose the analysis (2026-11-12)
    (PHYS / f"servo_{scn}_summary.json").write_text(json.dumps(out, indent=1), encoding="utf-8")
    if a.save:
        try:
            print(RM.set_view(ASM, "*Isometric", 0, PHYS / "screens" / f"servo_{scn}.bmp", res["study"], save=True))
        except RuntimeError as e:
            msg = str(e)
            print(("  WARN: view saved, but closing docs failed (restart SW): " if "saved with view (err=0)" in msg
                   else "  WARN: save view failed: ") + msg.splitlines()[-1])
    if a.scenario == "calib":
        sign = json.loads(SIGN_FILE.read_text()) if SIGN_FILE.exists() else {}
        for r, jr in out["joints"].items():
            if abs(jr["end_got"]) < 3:
                print(f"  !! {r}: moved {jr['end_got']:.1f} deg — servo not acting"); continue
            if np.sign(jr["end_got"]) != np.sign(jr["end_cmd"]):
                sign[r] = -sign.get(r, 1); print(f"  flip {r}")
            else:
                sign.setdefault(r, 1)
        SIGN_FILE.write_text(json.dumps(sign, indent=1)); print("signs ->", SIGN_FILE)


if __name__ == "__main__":
    main()
