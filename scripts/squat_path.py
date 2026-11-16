"""Find leg poses (hippitch h, knee k, ankle a; both legs equal) where the sole is flat on the floor for a given
torso pitch and the COM projects onto the sole: path from 'table' (torso horizontal) to a standing squat.
Torso pitch = rotation of the pelvis about the pitch axis relative to the floor; the foot is the reference.
"""
import itertools, math
import numpy as np
import wb_fk

B = wb_fk.Body(cell=5.0)
J = {j["role"]: j for j in B.spec["joints"]}


def foot_frame(q):
    """sole centre + rotation of foot A in the pelvis frame (column convention)."""
    T = B.link_tf(q)
    R, t = T["legA_ankleroll"]
    import leg_fk
    L = leg_fk.Leg("A")
    return R @ L.sole0 + t, R


def eval_pose(h, k, a, arms):
    q = {}
    for s in "AB":
        q[f"leg{s}_hippitch"] = h; q[f"leg{s}_knee"] = k; q[f"leg{s}_ankle"] = a
        q[f"arm{s}_shoulder"] = arms
    p, R = foot_frame(q)
    c = B.com(q)
    n = R @ np.array([1.0, 0, 0])            # sole normal (robot x of the foot) = gravity when standing on it
    tilt = math.degrees(math.acos(max(-1, min(1, n @ np.array([1.0, 0, 0])))))
    sgn = np.sign(np.cross([1.0, 0, 0], n) @ np.array([0, 1.0, 0])) or 1
    d = c - p
    dz = d @ (R @ np.array([0, 0, 1.0]))     # COM offset along the foot's forward axis (mm)
    h_com = d @ n * -1                        # COM height above the sole
    return tilt * sgn, dz, h_com


if __name__ == "__main__":
    print("torso tilt vs floor (deg, + = leaning ...), COM offset along foot (mm, |.|<30 ok), COM height")
    rows = []
    for h, k in itertools.product(range(-90, 91, 10), range(-90, 91, 10)):
        for arms in (0,):
            # ankle chosen so the sole is flat when the pelvis has the tilt implied by h, k: scan a too
            for a in range(-90, 91, 10):
                tilt, dz, hc = eval_pose(h, k, a, arms)
                rows.append((h, k, a, tilt, dz, hc))
    rows = np.array(rows)
    # for each torso tilt bucket, the pose with COM closest over the sole and highest COM
    for T in range(-90, 91, 15):
        sel = rows[(np.abs(rows[:, 3] - T) < 4) & (np.abs(rows[:, 4]) < 15)]
        if len(sel):
            b = sel[np.argmax(sel[:, 5])]
            print(f"tilt {T:+4d}: h {b[0]:+4.0f} k {b[1]:+4.0f} a {b[2]:+4.0f} -> COM dz {b[4]:+5.1f} height {b[5]:6.1f}  ({len(sel)} poses)")
        else:
            print(f"tilt {T:+4d}: none")
