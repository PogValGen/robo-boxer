"""Analyse a SOLIDWORKS Motion run (tools/physics/build_physics.exe run ...).

1) achieved joint angle (rotor link vs stator link, about the servo axis) vs commanded profile
   -> sign check; writes cad/physics/motor_reverse.json for joints that turned the wrong way;
2) motor torque per servo vs TowerPro MG90S stall torque (docs/web/2026-11-01_towerpro_mg90s_specs.md):
   1.8 kgf*cm = 0.1765 N*m @4.8 V, 2.2 kgf*cm = 0.2157 N*m @6.6 V.

Usage: python check_phys.py <motion_x.json>
"""
from __future__ import annotations
import json, math, re, sys
from pathlib import Path
import numpy as np

PROJ = Path(__file__).resolve().parents[1]
STALL_48, STALL_66 = 1.8 * 0.0980665, 2.2 * 0.0980665   # N*m


def step(t, t0, h0, t1, h1):
    if t <= t0: return h0
    if t >= t1: return h1
    u = (t - t0) / (t1 - t0)
    return h0 + (h1 - h0) * u * u * (3 - 2 * u)


def eval_expr(expr, t):
    if expr.strip() == "0": return 0.0
    tot = 0.0
    for a, b, c, h in re.findall(r"STEP\(TIME,([\d.]+),([\d.]+),([\d.]+),(-?[\d.]+)D\)", expr):
        tot += step(t, float(a), float(b), float(c), float(h))
    return tot


def R(m):  # SW row-vector rotation
    return np.array(m[:9]).reshape(3, 3)


def rel_angle(rot0, sta0, rot1, sta1, axis):
    """angle (deg) of rotor relative to stator between t0 and t1, about the servo axis (world at t0)."""
    A0 = R(rot0) @ R(sta0).T                   # rotor-local -> stator-local (row-vector convention)
    A1 = R(rot1) @ R(sta1).T
    D = A0.T @ A1                              # stator-local(t0) -> stator-local(t1): rotation IN THE STATOR FRAME
    ang = math.degrees(math.acos(max(-1, min(1, (np.trace(D) - 1) / 2))))
    if ang < 1e-6: return 0.0
    k = np.array([D[1, 2] - D[2, 1], D[2, 0] - D[0, 2], D[0, 1] - D[1, 0]])  # rotation axis (row-vec convention)
    a_loc = np.array(axis) @ R(sta0).T         # servo axis (world, t0) in the STATOR frame (fixed to it)
    return ang * float(np.sign(k @ a_loc) or 1.0)


G_VEC = np.array([9.80665, 0.0, 0.0])          # robot +x = down (same as the Motion gravity)
SERVO_I_SCALE = 13.4 / 9.0                     # mass overridden 9.0 -> 13.4 g, inertia from geometry: scale


def inverse_dynamics(res, spec, mot):
    """Joint torque about each servo axis needed to follow the Motion trajectories (Newton-Euler,
    rigid links). Returns {role: (t[], tau_Nm[])}."""
    poses = res["poses"]; times = np.array(sorted((float(t) for t in poses)))
    keys = [k for k in sorted(poses, key=float)]
    parts = spec["parts"]; anchors = spec["anchors"]
    # --- link inertial data at t0 (world frame, SI)
    links = {}
    for pm in res["parts"]:
        p = parts[pm["i"]]; L = pm["link"]
        R = np.array(p["m"][:9]).reshape(3, 3); T = np.array(p["m"][9:12])
        Ip = np.array(pm["I"]).reshape(3, 3) * (SERVO_I_SCALE if "MG90S" in p["stem"] else 1.0)
        cm_w = np.array(pm["cm"]) @ R + T
        Iw = R.T @ Ip @ R                       # column-convention rotation Q = R^T
        d = links.setdefault(L, {"m": 0.0, "s": np.zeros(3), "parts": []})
        d["m"] += pm["m"]; d["s"] += pm["m"] * cm_w; d["parts"].append((pm["m"], cm_w, Iw))
    for L, d in links.items():
        c = d["s"] / d["m"]; I = np.zeros((3, 3))
        for m, cw, Iw in d["parts"]:
            r = cw - c; I += Iw + m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
        d["cm0"], d["I0"] = c, I
    # --- per link: rotation (column conv) and CM over time, relative to t0 through its anchor
    A0 = {L: poses[keys[0]][L] for L in links}
    def pose(L, k):
        a, a0 = poses[k][L], A0[L]
        Ra, Ta = np.array(a[:9]).reshape(3, 3), np.array(a[9:12])
        Ra0, Ta0 = np.array(a0[:9]).reshape(3, 3), np.array(a0[9:12])
        M = np.linalg.inv(Ra0) @ Ra             # row conv: v_t = v_0 @ M
        return M.T, (links[L]["cm0"] - Ta0) @ M + Ta, (lambda v: (np.asarray(v) - Ta0) @ M + Ta)
    dt = np.gradient(times)
    kin = {}
    for L in links:
        Q = np.array([pose(L, k)[0] for k in keys]); C = np.array([pose(L, k)[1] for k in keys])
        dQ = np.gradient(Q, times, axis=0)
        W = np.array([[ (dQ[i] @ Q[i].T)[2, 1], (dQ[i] @ Q[i].T)[0, 2], (dQ[i] @ Q[i].T)[1, 0]] for i in range(len(keys))])
        Al = np.gradient(W, times, axis=0)
        Acm = np.gradient(np.gradient(C, times, axis=0), times, axis=0)
        kin[L] = (Q, C, W, Al, Acm)
    # --- subtree of every joint (rotor link and all its descendants)
    child = {}
    for j in spec["joints"]:
        child.setdefault(j["stator"], []).append(j["rotor"])
    def subtree(L):
        out = [L]
        for c in child.get(L, []): out += subtree(c)
        return out
    out = {}
    for j in spec["joints"]:
        sub = subtree(j["rotor"]); tau = np.zeros(len(keys))
        for i, k in enumerate(keys):
            Qs, _, f = pose(j["stator"], k)
            axis = Qs @ np.array(j["axis"]); piv = f(np.array(j["pivot_m"]))
            M = np.zeros(3)
            for L in sub:
                Q, C, W, Al, Acm = kin[L]
                m = links[L]["m"]; I = Q[i] @ links[L]["I0"] @ Q[i].T
                M += np.cross(C[i] - piv, m * (Acm[i] - G_VEC)) + I @ Al[i] + np.cross(W[i], I @ W[i])
            tau[i] = M @ axis
        out[j["role"]] = (times, tau)
    return out


def main(motion_path):
    mp = Path(motion_path)
    mot = json.loads(mp.read_text(encoding="utf-8"))
    res = json.loads(mp.with_name(mp.stem + "_result.json").read_text(encoding="utf-8"))
    spec = json.loads((mp.parent / "phys_spec.json").read_text(encoding="utf-8"))
    J = {j["role"]: j for j in spec["joints"]}
    poses = res["poses"]; times = sorted(poses, key=float)
    print(f"calc={res['calc']} mass={res['mass_g']:.0f} g")
    flips = {}
    print("\n== sign check: achieved vs commanded angle at the time of max |command| ==")
    for mo in mot["motors"]:
        role = mo["role"]; j = J[role]
        sgn = -1.0 if mo.get("flipped") else 1.0          # intended angle = expression with the flip undone
        cmd = [sgn * eval_expr(mo["expr"], float(t)) for t in times]
        k = int(np.argmax(np.abs(cmd)))
        if abs(cmd[k]) < 1: print(f"  {role:<20} hold 0 (no check)"); continue
        t0, tk = times[0], times[k]
        got = rel_angle(poses[t0][j["rotor"]], poses[t0][j["stator"]], poses[tk][j["rotor"]], poses[tk][j["stator"]], j["axis"])
        same = np.sign(got) == np.sign(cmd[k]) and abs(abs(got) - abs(cmd[k])) < 3
        flips[role] = bool(mo.get("flipped")) if same else (not mo.get("flipped"))
        print(f"  {role:<20} t={float(tk):.2f} cmd={cmd[k]:7.1f} got={got:7.1f} {'ok' if same else '!! FLIP'}")
    print("\n== servo torque by inverse dynamics (Motion trajectories + SW masses) vs MG90S stall 0.177 N*m @4.8V ==")
    tser = inverse_dynamics(res, spec, mot)
    rows = []
    for role, (t, tau) in tser.items():
        a = np.abs(tau); k = int(np.argmax(a))
        rows.append((role, a.max(), t[k], float(np.mean(a))))
    csv = mp.with_name(mp.stem + "_torque.csv")
    roles = list(tser)
    with open(csv, "w", encoding="utf-8") as fh:
        fh.write("t," + ",".join(roles) + "\n")
        for i, tt in enumerate(tser[roles[0]][0]):
            fh.write(f"{tt:.3f}," + ",".join(f"{tser[r][1][i]:.5f}" for r in roles) + "\n")
    print("  torque series ->", csv)
    for role, mx, tk, avg in sorted(rows, key=lambda r: -r[1]):
        pct = 100 * mx / STALL_48
        flag = "!! ПРЕВЫШЕН stall @4.8V" if mx > STALL_48 else ("!  >50% stall" if pct > 50 else "ok")
        print(f"  {role:<20} max {mx:.4f} N*m ({mx / 0.0980665:5.2f} kgf*cm) at t={tk:4.2f}s  mean {avg:.4f}  = {pct:5.1f}% stall@4.8V  {flag}")
    res["_summary"] = [{"role": r, "max_Nm": m, "t": tk, "pct_stall_4v8": 100 * m / STALL_48} for r, m, tk, _ in rows]
    mp.with_name(mp.stem + "_summary.json").write_text(json.dumps(res["_summary"], indent=1), encoding="utf-8")
    return flips


if __name__ == "__main__":
    f = main(sys.argv[1])
    out = PROJ / "cad" / "physics" / "motor_reverse_suggested.json"
    out.write_text(json.dumps(f, indent=1)); print("\nsuggested reverse flags ->", out)
