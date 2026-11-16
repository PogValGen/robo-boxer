"""Construction version switch for the robot builders (build_leg / build_pelvis / build_arm).

ROBO_V=1 (default): joint v1 (bearings, M2 screws), output cad/leg, cad/body, cad/arm — unchanged.
ROBO_V=2: joint v2 (build_joint_v2: bushings, press pegs, no screws), output cad/v2/...
Import this BEFORE `import build_joint` / `from build_joint import ...`: in v2 it patches the joint
constants of build_joint so the builders pick the v2 unit.
"""
from __future__ import annotations

import os

V2 = os.environ.get("ROBO_V", "1") == "2"

import build_joint as bj  # noqa: E402

if V2:
    import build_joint_v2 as j2  # noqa: E402  (also redirects bj.OUT/SHOTS/STL/LOG to cad/v2/joint)
    for k in ("ASM", "HX0", "LINK_DX", "LINK_YZ", "WEB_R0", "WEB_T"):
        setattr(bj, k, getattr(j2, k))
    CAD = bj.PROJ / "cad" / "v2"
    Y_MIN, Y_MAX = j2.LV_BOT0, j2.BR_Y0 + j2.BR_T     # unit extent along its own axis (y)
    HZ = j2.HZ                                        # unit half width
    PEG_L = j2.LINK_PEG_L                             # link pegs stick out of the housing -x face
    LINK_D = j2.LINK_D
    FACE_Y1 = j2.BR_Y0                                # top of the housing -x face
    xboss = j2.xboss
else:
    CAD = bj.PROJ / "cad"
    Y_MIN, Y_MAX = bj.LV_BOT0, bj.BR_Y0 + bj.BR_T
    HZ = bj.HZ
    PEG_L = 0.0
    LINK_D = 0.0
    FACE_Y1 = bj.BR_Y0 - 1.2
    xboss = None
X_MIN = bj.HX0 - PEG_L                                # unit bbox along x (pegs included)


def tag() -> str:
    return "v2" if V2 else "v1"


def _added_center(m0, m1, axis):
    v0, v1 = m0["volume_mm3"], m1["volume_mm3"]
    if v1 - v0 < 1:
        return None
    return (v1 * m1["center_of_mass_mm"][axis] - v0 * m0["center_of_mass_mm"][axis]) / (v1 - v0)


async def zboss(z0: float, draw, depth: float, outward: int, why: str) -> None:
    """Boss from a Front-type plane (sketch (u, v) = (x, y)) placed OUTSIDE at z0 + outward*depth,
    extruded back to z0; checked by the centre of the ADDED volume (the part may extend past the pegs),
    retried reversed on a new sketch."""
    zs = z0 + outward * depth
    r = await bj.t("create_reference_plane", {"base_plane": "Front", "offset": abs(zs), "flip": zs < 0,
                                              "name": f"Z{zs:+.2f}".replace(".", "_")}, why=f"plane z={zs}", n=60)
    want = (z0 + zs) / 2
    for rev in (False, True):
        m0 = bj.mc.mass_summary(bj.model())
        sk = await bj.sketch(r["plane"]["name"], draw)
        await bj.t("create_extrusion", {"sketch_name": sk, "depth": depth, "reverse_direction": rev}, why=why + (" (rev)" if rev else ""), n=80)
        m1 = bj.mc.mass_summary(bj.model())
        c = _added_center(m0, m1, 2)
        # COM is reported rounded: the error of the added-volume centre grows with V_total / V_added
        dv = max(m1["volume_mm3"] - m0["volume_mm3"], 1e-6)
        tol = min(0.3 + 0.01 * m1["volume_mm3"] / dv, depth / 2 - 0.05)
        print(f"    added volume centre z = {c} (want {want:.2f} +- {tol:.2f})")
        if c is not None and abs(c - want) < tol:
            return
        bj.mc.typed(bj.model(), "IModelDoc2").EditUndo2(1)
    raise AssertionError((why, "pegs not placed"))
