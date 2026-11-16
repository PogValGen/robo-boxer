"""Joint command keyframes for servo_sim.py: {role: [[t, deg], ...]} — cubic (STEP-like) between keys.

Angle sense = rotor vs stator about the servo axis (phys_spec.json), as found with pose_search /
the ideal-motor punch study: shoulder -90 = arm horizontal forward, waist +20 drives the right
fist (side B) forward. Leg senses: see gait notes in walk().
"""
from __future__ import annotations

ARMS = ("A", "B")
LEGS = ("A", "B")


def eval_cmd(keys, t):
    if t <= keys[0][0]:
        return keys[0][1]
    for (t0, v0), (t1, v1) in zip(keys, keys[1:]):
        if t < t1:
            u = (t - t0) / (t1 - t0)
            return v0 + (v1 - v0) * u * u * (3 - 2 * u)
    return keys[-1][1]


class K(dict):
    """keyframe builder: k.at(t, role, deg) appends, keeping the previous value until t."""
    def at(self, t, role, deg):
        self.setdefault(role, [[0.0, 0.0]])
        if abs(self[role][-1][0] - t) < 1e-9:
            self[role][-1][1] = deg
        else:
            self[role].append([t, deg])

    def hold(self, t, role):
        self.at(t, role, self.get(role, [[0, 0]])[-1][1])


def calib(**_):
    k = K()
    roles = [f"arm{s}_{j}" for s in ARMS for j in ("shoulder", "shoulderroll", "elbow")] + ["torso"] + \
            [f"leg{s}_{j}" for s in LEGS for j in ("hipyaw", "hiproll", "hippitch", "knee", "ankle", "ankleroll")]
    for r in roles:                                # roles a robot version does not have are ignored by servo_sim
        k.at(0.1, r, 0); k.at(0.4, r, 15.0)
    return k, 0.6


def guard_and_jab(k, t0, side="B", guard=-60.0, jab=-95.0, waist=20.0, t_out=0.25, t_back=0.45, hold=0.4):
    """arms up to guard (0.8 s), jab with the `side` arm + waist twist, back to guard."""
    for s in ARMS:
        k.at(t0, f"arm{s}_shoulder", 0); k.at(t0 + 0.8, f"arm{s}_shoulder", guard)
    t1 = t0 + 0.8 + hold
    r = f"arm{side}_shoulder"
    k.hold(t1, r); k.at(t1 + t_out, r, jab); k.at(t1 + t_out + t_back, r, guard)
    w = waist if side == "B" else -waist
    k.at(t1, "torso", 0); k.at(t1 + t_out, "torso", w); k.at(t1 + t_out + t_back, "torso", 0)
    return t1 + t_out + t_back


def punch(fixed=False, t_out=0.25, **_):
    k = K()
    t0 = 0.0 if fixed else 0.3                    # free robot: settle on the floor first
    t = guard_and_jab(k, t0, "B", t_out=t_out)
    return k, round(t + 0.5, 3)


def stand(fixed=False, **_):
    """arms to guard, then stand still: baseline for static torques and contact noise."""
    k = K()
    for s in ARMS:
        k.at(0.3, f"arm{s}_shoulder", 0); k.at(1.1, f"arm{s}_shoulder", -60.0)
    return k, 2.0


def leg_cmd(k, t, side, roll, pitch, lift, crouch):
    """Leg keyframe at t: hip roll `roll`, leg swing `pitch` (+ = foot back), extra knee lift `lift`
    (deg of hip/ankle flexion) on top of the crouch; foot kept parallel: ankle = -(hip + knee)."""
    h = -crouch + pitch - lift
    kn = 2 * crouch + 2 * lift
    k.at(t, f"leg{side}_hiproll", roll)
    k.at(t, f"leg{side}_hippitch", h)
    k.at(t, f"leg{side}_knee", kn)
    k.at(t, f"leg{side}_ankle", -(h + kn))


def walk(fixed=False, steps=4, T=1.2, phi=12.0, beta=8.0, lift=14.0, crouch=15.0, arms=-30.0, **_):
    """Rocking walk for legs without ankle roll (leg_fk: hip roll + -> body toward side A; hip pitch +
    -> foot back). Per step: sway onto the stance leg (both hip rolls), lift + swing the free leg
    forward while the stance leg pushes back, put it down, sway back to the middle."""
    k = K()
    t = 0.0 if fixed else 0.3
    state = {"A": 0.0, "B": 0.0}                       # current swing pitch of each leg
    for s in LEGS:
        leg_cmd(k, t, s, 0, 0, 0, 0)
    for s in ARMS:
        k.at(t, f"arm{s}_shoulder", 0)
    t += 0.6
    for s in LEGS:
        leg_cmd(k, t, s, 0, 0, 0, crouch)
    for s in ARMS:
        k.at(t, f"arm{s}_shoulder", arms)
    t += 0.3
    for n in range(int(steps)):
        stance, swing = ("A", "B") if n % 2 == 0 else ("B", "A")
        r = phi if stance == "A" else -phi
        q = T / 4
        for s in LEGS:                                   # 1) sway onto the stance leg
            leg_cmd(k, t + q, s, r, state[s], 0, crouch)
        state[swing], state[stance] = -beta, beta        # swing goes forward, stance pushes back
        leg_cmd(k, t + 2 * q, swing, r, -beta / 2 if n == 0 else 0.0, lift, crouch)  # 2) lift, half way
        leg_cmd(k, t + 2 * q, stance, r, 0.0 if n else beta / 2, 0, crouch)
        leg_cmd(k, t + 3 * q, swing, r, state[swing], 0, crouch)      # 3) put down in front
        leg_cmd(k, t + 3 * q, stance, r, state[stance], 0, crouch)
        for s in LEGS:                                   # 4) sway back to the middle
            leg_cmd(k, t + 4 * q, s, 0, state[s], 0, crouch)
        t += T
    for s in LEGS:                                       # feet together, stand
        leg_cmd(k, t + 0.6, s, 0, 0, 0, crouch)
    return k, round(t + 1.4, 3)


# ----------------------------------------------------------------------------------------------
# v2 legs with ankle roll: footstep planner + numeric IK (leg_fk.ik). World frame = robot frame at
# start: y left (side A = +y), z forward; heights: crouch / lift in mm (feet go UP = -x in the pelvis frame).
NOMINAL_Y = {"A": 50.0, "B": -50.0}


def com_offset():
    """COM of the whole robot relative to the pelvis origin, robot frame (mm), from SW mass properties."""
    import json, os
    from pathlib import Path
    import numpy as np
    proj = Path(__file__).resolve().parents[1]
    phys = proj / "cad" / {"2": "v2/physics", "3": "v3/physics"}.get(os.environ.get("ROBO_V", "1"), "physics")
    spec = json.loads((phys / "phys_spec.json").read_text(encoding="utf-8"))
    mp = {p["name"]: p for p in json.loads((phys / "massprops.json").read_text(encoding="utf-8"))["parts"]}
    cnt, tot, acc = {}, 0.0, np.zeros(3)
    for p in spec["parts"]:
        st = Path(p["path"]).stem; cnt[st] = cnt.get(st, 0) + 1
        # stem comes from the path (MG90S_phys already); alias kept as a guard for specs that name MG90S_grabcad
        q = mp.get(f"{st}-{cnt[st]}") or mp.get(f"{st}-{cnt[st]}".replace("MG90S_grabcad", "MG90S_phys"))
        if not q:
            continue
        R = np.array(p["m"][:9]).reshape(3, 3); T = np.array(p["m"][9:12])
        c = np.array(q["cm"]) @ R + T
        tot += q["m"]; acc += q["m"] * c
    return acc / tot * 1000.0


class Planner:
    """Quasi-static stepping: the COM is kept over the stance foot (or between the feet) at every key."""

    def __init__(self, t0=0.3, crouch=10.0, arms=-20.0, gamma=0.8):
        import leg_fk
        self.legs = {s: leg_fk.Leg(s) for s in LEGS}
        for s in LEGS:                                  # stance width from the model (v2: +-25)
            NOMINAL_Y[s] = float(self.legs[s].sole0[1])
        self.k = K(); self.t = t0
        self.com = com_offset()                 # robot frame (x down, y, z)
        self.crouch, self.gamma = crouch, gamma
        self.feet = {s: [0.0, 0.0] for s in LEGS}      # world foot (z, y offset from nominal)
        self.body = [0.0, 0.0]                         # world COM (z, y)
        self.lift = {s: 0.0 for s in LEGS}
        self.q = {s: None for s in LEGS}
        self.worst = 0.0
        for s in LEGS:
            for j in self.legs[s].names:
                self.k.at(t0, f"leg{s}_{j}", 0.0)
        for a in ARMS:
            self.k.at(t0, f"arm{a}_shoulder", 0.0)
        self.t += 0.8
        for a in ARMS:
            self.k.at(self.t, f"arm{a}_shoulder", arms)
        self._key()

    def _key(self):
        import numpy as np
        for s in LEGS:
            # foot relative to the pelvis: foot - (COM - com_offset)
            dz = self.feet[s][0] - self.body[0] + self.com[2]
            dy = (NOMINAL_Y[s] + self.feet[s][1]) - self.body[1] + self.com[1] - NOMINAL_Y[s]
            dx = -(self.crouch + self.lift[s])
            q, err = __import__("leg_fk").ik(self.legs[s], [dx, dy, dz], self.q[s])
            self.worst = max(self.worst, err)
            if err > 0.5:
                raise ValueError(f"IK: leg {s} cannot reach dp=({dx:.1f},{dy:.1f},{dz:.1f}) mm at t={self.t:.2f} "
                                 f"(residual {err:.1f}); more crouch or less shift")
            self.q[s] = q
            for j, a in zip(self.legs[s].names, q):
                self.k.at(self.t, f"leg{s}_{j}", float(a))

    def move(self, dt, body=None, foot=None, lift=None):
        """advance dt: body=(z, y) COM target; foot=(side, (z, y)); lift=(side, mm)."""
        self.t += dt
        if body is not None:
            self.body = list(body)
        if foot is not None:
            self.feet[foot[0]] = list(foot[1])
        if lift is not None:
            self.lift[lift[0]] = lift[1]
        self._key()

    def over(self, s):          # COM target over foot s
        return (self.feet[s][0], self.gamma * NOMINAL_Y[s] + self.feet[s][1])

    def middle(self):
        return ((self.feet["A"][0] + self.feet["B"][0]) / 2,
                (self.feet["A"][1] + self.feet["B"][1]) / 2)

    def step(self, swing, to, lift=12.0, ts=0.6, tl=0.25, tw=0.45):
        stance = "A" if swing == "B" else "B"
        self.move(ts, body=self.over(stance))
        self.move(tl, lift=(swing, lift))
        self.move(tw, foot=(swing, to))
        self.move(tl, lift=(swing, 0.0))

    def done(self, hold=1.0):
        self.move(0.6, body=self.middle())
        return dict(self.k), round(self.t + hold, 3)


def walk2(fixed=False, steps=4, S=20.0, lift=10.0, crouch=8.0, arms=-20.0, ts=1.0, tw=0.45, tl=0.25, direction=1.0, gamma=0.7, **_):
    """Walk forward (direction +1) or backward (-1): footholds B +S, A +2S, ... last step closes the feet."""
    P = Planner(crouch=crouch, arms=arms, gamma=gamma)
    S = S * (1 if direction >= 0 else -1)
    for n in range(int(steps)):
        sw = "B" if n % 2 == 0 else "A"
        other = "A" if sw == "B" else "B"
        to_z = P.feet[other][0] + (S if n < steps - 1 else 0.0) if n else S
        P.step(sw, (to_z, P.feet[sw][1]), lift=lift, ts=ts, tl=tl, tw=tw)
    k, d = P.done()
    print(f"    walk2: IK worst residual {P.worst:.3f}, COM offset {P.com.round(1)}")
    return k, d


def side(fixed=False, steps=2, S=15.0, lift=10.0, crouch=8.0, arms=-20.0, ts=1.0, tw=0.45, direction=1.0, gamma=0.7, **_):
    """Side steps toward +y (side A) for direction +1, toward B for -1: lead foot out, then the trail foot."""
    P = Planner(crouch=crouch, arms=arms, gamma=gamma)
    lead, trail = ("A", "B") if direction >= 0 else ("B", "A")
    dy = S if direction >= 0 else -S
    for _n in range(int(steps)):
        P.step(lead, (P.feet[lead][0], P.feet[lead][1] + dy), lift=lift, ts=ts, tw=tw)
        P.step(trail, (P.feet[trail][0], P.feet[trail][1] + dy), lift=lift, ts=ts, tw=tw)
    k, d = P.done()
    print(f"    side: IK worst residual {P.worst:.3f}, COM offset {P.com.round(1)}")
    return k, d


# ----------------------------------------------------------------------------------------------
# Dynamic walking (2026-11-12): linear inverted pendulum + ZMP preview control (Kajita 2003).
# The ZMP (not the COM) is put on the stance foot; the COM sways less and never stops, so the
# step can be several times faster than the quasi-static Planner.step. Units: mm, s.
G_MM = 9810.0


def preview_gains(T, zc, N, Qe=1.0, R=1e-6):
    import numpy as np
    A = np.array([[1, T, T * T / 2], [0, 1, T], [0, 0, 1]])
    B = np.array([[T ** 3 / 6], [T * T / 2], [T]])
    C = np.array([[1, 0, -zc / G_MM]])
    A1 = np.block([[np.eye(1), C @ A], [np.zeros((3, 1)), A]])
    B1 = np.vstack([C @ B, B])
    Q = np.diag([Qe, 0, 0, 0])
    P = Q.copy()
    for _ in range(20000):                                   # discrete Riccati by iteration
        S = R + B1.T @ P @ B1
        Pn = Q + A1.T @ P @ A1 - A1.T @ P @ B1 @ np.linalg.solve(S, B1.T @ P @ A1)
        if np.max(np.abs(Pn - P)) < 1e-10 * max(1.0, np.max(np.abs(P))):
            P = Pn; break
        P = Pn
    S = R + B1.T @ P @ B1
    K = np.linalg.solve(S, B1.T @ P @ A1)
    Gi, Gx = K[0, 0], K[0, 1:]
    Ac = A1 - B1 @ K
    X = -Ac.T @ P @ np.array([[1.0], [0], [0], [0]])
    Gd = np.zeros(N); Gd[0] = -Gi
    for j in range(1, N):
        Gd[j] = np.linalg.solve(S, B1.T @ X)[0, 0]
        X = Ac.T @ X
    return A, B, C, Gi, Gx, Gd


def lipm_com(pref, T, zc, preview=1.6, R=1e-6, sub=1):
    """COM trajectory (one axis) whose LIPM ZMP tracks pref[k]; returns (com, zmp).
    sub > 1: run the controller at T/sub (pref linearly resampled), return every sub-th sample."""
    import numpy as np
    if sub > 1:
        n = len(pref); tt = np.arange((n - 1) * sub + 1) / sub
        c, p = lipm_com(np.interp(tt, np.arange(n), pref), T / sub, zc, preview, R, 1)
        return c[::sub], p[::sub]
    N = int(round(preview / T))
    A, B, C, Gi, Gx, Gd = preview_gains(T, zc, N, R=R)
    pad = np.concatenate([pref, np.full(N, pref[-1])])
    x = np.array([pref[0], 0.0, 0.0]); e = 0.0
    com, zmp = np.zeros(len(pref)), np.zeros(len(pref))
    for k in range(len(pref)):
        p = float((C @ x)[0]); zmp[k] = p; com[k] = x[0]
        e += p - pref[k]
        u = -Gi * e - Gx @ x - Gd @ pad[k:k + N]
        x = A @ x + B[:, 0] * u
    return com, zmp


def dyn(fixed=False, steps=4, S=25.0, lift=10.0, crouch=8.0, arms=-20.0, tss=0.6, tds=0.2, zy=0.9,
        direction=1.0, dt=0.02, settle=0.8, R=1e-8, sub=4, **_):
    """Dynamic forward/backward walk: ZMP on the stance foot (lateral = zy * foot y) during single
    support (tss), moved linearly foot-to-foot in double support (tds); COM from ZMP preview control;
    swing foot: lift 0.5(1-cos 2pi u), forward smoothstep. Footholds as walk2."""
    import numpy as np
    P = Planner(crouch=crouch, arms=arms, gamma=1.0)
    zc = float(P.legs["A"].sole0[0]) - float(P.com[0]) - crouch      # COM height over the sole, mm
    S = S * (1 if direction >= 0 else -1)
    feet = {s: [0.0, 0.0] for s in LEGS}
    Y = lambda s: zy * (NOMINAL_Y[s] + feet[s][1])
    # phases: (kind, duration, data); ZMP reference built with the planned footholds
    ph = [("hold", settle, None)]
    plan = []
    for n in range(int(steps)):
        sw = "B" if n % 2 == 0 else "A"
        other = "A" if sw == "B" else "B"
        to_z = feet[other][0] + (S if n < steps - 1 else 0.0) if n else S
        plan.append((sw, other, feet[sw][0], to_z))
        feet[sw][0] = to_z
    feet = {s: [0.0, 0.0] for s in LEGS}
    t_ref, zr, yr, events = [], [], [], []
    def add(dur, zf, yf):                    # linear ZMP segment from current to (zf, yf) over dur
        z0 = zr[-1] if zr else 0.0; y0 = yr[-1] if yr else 0.0
        n = max(1, int(round(dur / dt)))
        for i in range(1, n + 1):
            zr.append(z0 + (zf - z0) * i / n); yr.append(y0 + (yf - y0) * i / n)
    zr.append(0.0); yr.append(0.0)
    add(settle, 0.0, 0.0)
    for sw, st, z_from, z_to in plan:
        add(tds, feet[st][0], Y(st))                     # double support: ZMP to the stance foot
        k0 = len(zr) - 1
        add(tss, feet[st][0], Y(st))                     # single support: ZMP held on it
        events.append((k0, len(zr) - 1, sw, z_from, z_to))
        feet[sw][0] = z_to
    add(tds + 0.2, (feet["A"][0] + feet["B"][0]) / 2, 0.0)   # back to the middle
    add(1.0, zr[-1], 0.0)
    zr, yr = np.array(zr), np.array(yr)
    cz, pz = lipm_com(zr, dt, zc, R=R, sub=int(sub))
    cy, py = lipm_com(yr, dt, zc, R=R, sub=int(sub))
    # what matters: ZMP inside the STANCE foot during single support (double support has both feet)
    ss = np.zeros(len(zr), bool)
    for k0, k1, *_ in events:
        ss[k0:k1 + 1] = True
    err = max(np.max(np.abs(pz - zr)[ss]), np.max(np.abs(py - yr)[ss]))
    # keyframes
    swing = np.full(len(zr), None, dtype=object); lift_v = np.zeros(len(zr)); fz = {s: np.zeros(len(zr)) for s in LEGS}
    # per-sample foot positions
    pos = {s: 0.0 for s in LEGS}; ei = 0
    for k in range(len(zr)):
        for s in LEGS:
            fz[s][k] = pos[s]
        while ei < len(events) and k >= events[ei][1]:
            _, _, sw, _, z_to = events[ei]; pos[sw] = z_to; fz[sw][k] = z_to; ei += 1
        for k0, k1, sw, z_from, z_to in events:
            if k0 <= k <= k1:
                u = (k - k0) / (k1 - k0)
                fz[sw][k] = z_from + (z_to - z_from) * u * u * (3 - 2 * u)
                lift_v[k] = lift * 0.5 * (1 - np.cos(2 * np.pi * u)); swing[k] = sw
    t0 = P.t
    for k in range(1, len(zr)):
        P.t = t0 + k * dt
        P.body = [float(cz[k]), float(cy[k])]
        for s in LEGS:
            P.feet[s] = [float(fz[s][k]), 0.0]
            P.lift[s] = float(lift_v[k]) if swing[k] == s else 0.0
        P._key()
    sway = float(np.max(np.abs(cy)))
    print(f"    dyn: zc {zc:.1f} mm, step {tss + tds:.2f} s x {abs(S):.0f} mm = {abs(S) / (tss + tds):.0f} mm/s, "
          f"COM sway +-{sway:.1f} mm (feet +-{abs(NOMINAL_Y['A']):.1f}), ZMP err in single support {err:.2f} mm, IK worst {P.worst:.3f}")
    return dict(P.k), round(P.t + 0.5, 3)


# ----------------------------------------------------------------------------------------------
# Get-up from lying (servo_sim --lie back|chest), 2026-11-13. Found with wb_fk/squat_path.py: with the hips
# bent 90 deg (torso horizontal) the COM is already over the flat soles (12.7 mm), and it stays within 13 mm
# all the way while the hips straighten -> "table" pose, lift the arms, straighten the hips.
ALL_ROLES = [f"arm{s}_{j}" for s in ARMS for j in ("shoulder", "shoulderroll", "elbow")] + ["torso"] + \
            [f"leg{s}_{j}" for s in LEGS for j in ("hiproll", "hippitch", "knee", "ankle", "ankleroll")]


def keyseq(seq):
    """seq = [(t, {role: deg}), ...]; every key holds all roles (no early ramps)."""
    k = K(); cur = {r: 0.0 for r in ALL_ROLES}
    for t, q in seq:
        cur.update(q)
        for r in ALL_ROLES:
            k.at(t, r, cur[r])
    return k


def lie(fixed=False, dur=1.5, **_):
    """lie still (check of the lying setup)."""
    return keyseq([(0.1, {})]), dur


def getup(side=1.0, fixed=False, arm=88.0, hip=88.0, t_arm=0.8, t_hip=1.0, t_lift=0.6, t_up=1.5, crouch=0.0, **_):
    """side=-1: lying on the chest (arms/hips '-' = toward the floor), side=+1: on the back."""
    s = 1.0 if side >= 0 else -1.0
    A = lambda v: {"armA_shoulder": v, "armB_shoulder": v}
    H = lambda h, k=0.0, a=0.0: {f"leg{x}_{j}": v for x in LEGS for j, v in (("hippitch", h), ("knee", k), ("ankle", a))}
    t = 0.5
    seq = [(t, {})]
    t += t_arm; seq.append((t, A(s * arm)))                       # arms to the floor: chest/back up
    t += t_hip; seq.append((t, H(s * hip)))                       # legs down to the floor, soles flat: 'table'
    t += 0.3; seq.append((t, {}))
    t += t_lift; seq.append((t, A(0.0)))                          # arms off the floor: standing bent over
    t += t_up; seq.append((t, H(crouch, -2 * crouch, crouch)))    # straighten the hips
    t += 0.3; seq.append((t, A(-20.0)))
    return keyseq(seq), round(t + 1.0, 3)


def getup_chest(**kw):
    return getup(side=-1, **kw)


def getup_back(**kw):
    return getup(side=1, **kw)


def _sym(A=None, H=None, K=None, N=None):
    """both arms / both legs: shoulder A, hip pitch H, knee K, ankle N (None = keep)."""
    q = {}
    for x in LEGS:
        for j, v in (("hippitch", H), ("knee", K), ("ankle", N)):
            if v is not None:
                q[f"leg{x}_{j}"] = float(v)
    for x in "AB":
        if A is not None:
            q[f"arm{x}_shoulder"] = float(A)
    return q


def getup_chest2(fixed=False, t_arm=0.8, t_fold=1.0, t_shift=0.5, t_knee=1.5, t_up=1.5, n1=45.0, a1=-45.0, **_):
    """From the chest via a headstand (13.11, SW run v1 showed: arms '-88' lie on the floor and turn the body about the
    shoulders, head down, legs up). Then: fold hips -90 / knees -90 -> the legs come down as a bracket and stand on the
    soles next to the head (shank+foot 117 mm > torso 87 mm, head hangs free); ankle +n1 puts the COM over the soles
    (wb_fk: dz +10 mm); unfold the knees with the ankle following (45 -> 45 -> 30 -> 15); straighten the hips."""
    t = 0.5; seq = [(t, {})]
    t += t_arm; seq.append((t, _sym(A=-88)))                          # headstand on head + arms
    t += 0.3; seq.append((t, {}))
    t += t_fold; seq.append((t, _sym(H=-90, K=-90)))                  # bracket: soles land
    t += t_shift; seq.append((t, _sym(N=n1, A=a1)))                   # COM over the soles
    t += t_knee / 3; seq.append((t, _sym(K=-60, N=n1)))
    t += t_knee / 3; seq.append((t, _sym(K=-30, N=30)))
    t += t_knee / 3; seq.append((t, _sym(K=0, N=15)))                 # 'table': legs straight, torso horizontal
    t += t_up; seq.append((t, _sym(H=0, N=0, A=-20)))                 # straighten the hips
    return keyseq(seq), round(t + 1.0, 3)


def _bracket_tail(seq, t, t_head=0.6, t_knee=1.5, t_up=1.5):
    """Common end of both get-ups: the robot stands on its soles as a 'bracket' folded BACK (hips +90, knees +90,
    ankle +30: soles flat while the head still touches the floor, COM 11 mm from the sole centre - wb_fk) with the arms
    at +88 as a counterweight. Ankle -> 0 lifts the head; knees unfold with the ankle keeping the COM over the soles
    (|dz| < 17 mm); arms come down; hips straighten."""
    t += t_head; seq.append((t, _sym(N=0)))                              # head off the floor, torso upside down on the feet
    t += t_knee / 3; seq.append((t, _sym(K=60, N=-10)))
    t += t_knee / 6; seq.append((t, _sym(K=45, N=-20, A=66)))
    t += t_knee / 6; seq.append((t, _sym(K=30, N=-30, A=44)))
    t += t_knee / 6; seq.append((t, _sym(K=15, N=-20, A=22)))
    t += t_knee / 6; seq.append((t, _sym(K=0, N=-10, A=0)))              # legs straight, torso horizontal (face up)
    t += 0.3; seq.append((t, {}))
    t += t_up / 2; seq.append((t, _sym(H=45, N=-10, A=-10)))
    t += t_up / 2; seq.append((t, _sym(H=0, N=0, A=-20)))                # standing
    return t


def getup_back2(fixed=False, t_arm=0.8, t_fold=1.0, n_land=30.0, **_):
    """From the back: arms +88 lie on the floor and turn the body about the shoulders head down (SW run v1), the legs
    fold back between the arms and land on the soles; then _bracket_tail."""
    t = 0.5; seq = [(t, {})]
    t += t_arm; seq.append((t, _sym(A=88)))
    t += 0.3; seq.append((t, {}))
    t += t_fold; seq.append((t, _sym(H=90, K=90, N=n_land)))
    t += 0.4; seq.append((t, {}))
    t = _bracket_tail(seq, t)
    return keyseq(seq), round(t + 1.0, 3)


def getup_chest3(fixed=False, t_arm=0.8, t_fold=1.0, n_land=30.0, t_swing=0.8, **_):
    """From the chest: headstand on the arms (-88), the legs fold BACK (away from the arms) and land on the soles while
    the arms give way (-55); the arms swing over to +88 (COM from the heel edge to the middle of the sole); then
    _bracket_tail. (The bracket folded forward, v2, has the COM at the toe edge and never left the head.)"""
    t = 0.5; seq = [(t, {})]
    t += t_arm; seq.append((t, _sym(A=-88)))
    t += 0.3; seq.append((t, {}))
    t += t_fold; seq.append((t, _sym(H=90, K=90, N=n_land, A=-55)))
    t += 0.3; seq.append((t, {}))
    t += t_swing; seq.append((t, _sym(A=88)))
    t += 0.3; seq.append((t, {}))
    t = _bracket_tail(seq, t)
    return keyseq(seq), round(t + 1.0, 3)


def punch_step(fixed=False, S=30.0, tss=0.5, tds=0.15, guard=-60.0, jab=-88.0, waist=20.0, t_out=0.2, t_back=0.4,
               lead=-0.05, **_):
    """Step forward with the right foot (B) using the dynamic gait, right jab + waist twist as the foot lands
    (lead = s before landing), then the left foot closes up. Arms in guard the whole time."""
    k, dur = dyn(steps=2, S=S, tss=tss, tds=tds, arms=guard, settle=0.8)
    t_land = 1.1 + 0.8 + tds + tss                                  # Planner start 1.1 + settle + first step
    t1 = t_land + lead
    kk = K(); kk.update({r: [list(x) for x in v] for r, v in k.items()})
    kk.at(t1, "armB_shoulder", guard); kk.at(t1 + t_out, "armB_shoulder", jab); kk.at(t1 + t_out + t_back, "armB_shoulder", guard)
    kk.at(t1, "torso", 0.0); kk.at(t1 + t_out, "torso", waist); kk.at(t1 + t_out + t_back, "torso", 0.0)
    print(f"    punch_step: foot lands {t_land:.2f} s, jab {t1:.2f}..{t1 + t_out:.2f} s")
    return dict(kk), max(dur, round(t1 + t_out + t_back + 1.0, 3))


def kick(fixed=False, crouch=10.0, gamma=0.95, ts=1.2, lift_c=45.0, z_c=10.0, lift_k=60.0, z_k=85.0, t_c=0.4,
         t_k=0.18, t_hold=0.15, t_r=0.35, arms=-60.0, **_):
    """Front kick with the right leg (B) at an opponent's belly: weight over the left foot (quasi-static),
    chamber (knee up: foot lift_c mm up, z_c forward), snap out (lift_k up, z_k forward) in t_k s, back to the
    chamber, foot down at its place, weight back to the middle."""
    P = Planner(crouch=crouch, arms=arms, gamma=gamma)
    P.move(ts, body=P.over("A"))
    P.move(0.25, lift=("B", 8.0))
    P.move(t_c, lift=("B", lift_c), foot=("B", (z_c, 0.0)))
    t_snap = P.t
    P.move(t_k, lift=("B", lift_k), foot=("B", (z_k, 0.0)))
    P.move(t_hold)
    P.move(t_r, lift=("B", lift_c), foot=("B", (z_c, 0.0)))
    P.move(0.35, lift=("B", 8.0), foot=("B", (0.0, 0.0)))
    P.move(0.25, lift=("B", 0.0))
    k, d = P.done()
    print(f"    kick: snap {t_snap:.2f}..{t_snap + t_k:.2f} s, foot {lift_k:.0f} mm up / {z_k:.0f} mm forward, IK worst {P.worst:.3f}")
    return k, d


def kick2(fixed=False, crouch=10.0, gamma=0.95, ts=1.2, hip_c=None, knee_c=None, hip_k=-80.0, t_c=0.2, t_k=0.15,
          t_hold=0.2, t_r=0.35, arms=-60.0, comp=1.0, lift0=25.0, **_):
    """Front kick with the right leg (B), joint-space (toes may point): weight over the left foot (Planner),
    chamber (hip hip_c, knee knee_c), snap (knee -> 0, hip hip_k: straight leg at the belly), back to the chamber,
    foot down. The stance ankle (A) leans the body back so the COM stays over the left sole (wb_fk)."""
    import numpy as np
    import wb_fk, leg_fk
    P = Planner(crouch=crouch, arms=arms, gamma=gamma)
    P.move(ts, body=P.over("A"))
    P.move(0.35, lift=("B", lift0))     # v1 (8 mm): the toes hit the floor in the chamber and pushed the robot over
    names = P.legs["A"].names
    qA = dict(zip(names, P.q["A"])); qB = dict(zip(names, P.q["B"]))
    B = wb_fk.Body(cell=6.0); LA = leg_fk.Leg("A")

    def com_off(q):            # COM along the stance foot's forward axis, relative to the sole centre (mm)
        T = B.link_tf(q); R, t = T["legA_ankleroll"]; p = R @ LA.sole0 + t
        return (B.com(q) - p) @ (R @ np.array([0, 0, 1.0]))

    def pose(hb, kb, da):
        q = {f"legA_{n}": v for n, v in qA.items()}
        q.update({f"legB_{n}": v for n, v in qB.items()})
        q["legB_hippitch"], q["legB_knee"], q["legB_ankle"] = hb, kb, 0.0
        q["legA_ankle"] = qA["ankle"] + da
        for s in ARMS:
            q[f"arm{s}_shoulder"] = arms
        return q

    def balance(hb, kb):       # stance-ankle offset that puts the COM over the sole (bisection on da)
        lo, hi = -40.0, 40.0
        f = lambda da: com_off(pose(hb, kb, da))
        if f(lo) * f(hi) > 0:
            return 0.0
        for _ in range(30):
            m = (lo + hi) / 2
            lo, hi = (m, hi) if f(lo) * f(m) > 0 else (lo, m)
        return comp * (lo + hi) / 2

    # 13.11 (SW runs v1/v2): the legs of this robot bend 'bird-like' (crouch = hip +31, knee -46); a human-like chamber
    # (hip -70, knee +80) passes through the straight leg and stamps the foot into the floor (-5 mm) -> robot pushed
    # over sideways. Default: no separate chamber, the snap starts from the lifted pose (clearance stays > 25 mm).
    if hip_c is None:
        hip_c, knee_c = qB["hippitch"], qB["knee"]
    t0 = P.t
    base = {f"legA_{n}": v for n, v in qA.items()}; base.update({f"legB_{n}": v for n, v in qB.items()})
    keys = [(0.0, base)]
    def add(dt, hb, kb):
        q = pose(hb, kb, balance(hb, kb)); keys.append((keys[-1][0] + dt, q)); return q
    add(t_c, hip_c, knee_c)                 # chamber
    qk = add(t_k, hip_k, 0.0)               # snap
    add(t_hold, hip_k, 0.0)
    add(t_r, hip_c, knee_c)                 # re-chamber
    add(0.35, qB["hippitch"], qB["knee"])   # foot back down (ankle restored below)
    keys[-1][1]["legB_ankle"] = qB["ankle"]; keys[-1][1]["legA_ankle"] = qA["ankle"]
    for dt, q in keys[1:]:
        for r, v in q.items():
            if r.startswith("leg"):
                P.k.at(t0 + dt, r, float(v))
    P.t = t0 + keys[-1][0]
    # weight back to the middle, both feet down
    P.q["A"] = [qA[n] for n in names]; P.q["B"] = [qB[n] for n in names]
    P.move(0.25, lift=("B", 0.0))
    k, d = P.done()
    T = B.link_tf(qk); fb = [c for n, c in B.where(qk, "Leg_Foot") if n == "Leg_Foot-2"][0]
    print(f"    kick2: snap {t0 + t_c:.2f}..{t0 + t_c + t_k:.2f} s, kick foot at robot x {fb[0]:.0f} (sole {LA.sole0[0]:.0f}) z {fb[2]:+.0f} mm, "
          f"stance-ankle lean {qk['legA_ankle'] - qA['ankle']:+.1f} deg")
    return k, d


SCENARIOS = {"calib": calib, "punch": punch, "stand": stand, "walk": walk, "walk2": walk2, "side": side, "dyn": dyn,
             "lie": lie, "getup_chest": getup_chest, "getup_back": getup_back, "punch_step": punch_step, "kick": kick,
             "kick2": kick2, "getup_chest2": getup_chest2, "getup_back2": getup_back2,
             "getup_chest3": getup_chest3}


def scenario(name, **kw):
    k, dur = SCENARIOS[name](**kw)
    return dict(k), dur
