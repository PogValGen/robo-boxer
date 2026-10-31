"""Arms + shoulder yoke + whole robot for robo-boxer.

Arm frame = joint frame of S1 (same axes as the robot: x down, y lateral, z forward):
S1 shoulder pitch (axis y) -> S2 shoulder roll (unit turned 90 deg about x, axis z)
-> E elbow pitch (axis y) -> TPU fist on the elbow lever. Joints 55.5 apart along x,
linked like the leg (lever web -> next housing -x face, square 4 x M2).
The yoke (6 mm plate) sits on the torso shelf (4 x M2) and carries both S1 housings
under its ends (4 x M2 each, screws from above).

Usage: python build_arm.py [fist] [yoke] [arm] [robot]   (default: all)
"""

from __future__ import annotations

import asyncio
import sys
import time

import build_joint as bj
import build_leg as bl
import build_pelvis as bp
from build_joint import HX0, LINK_DX, LINK_YZ, cut, mc, rect, t

OUT = bj.PROJ / "cad" / "arm"
OUT.mkdir(parents=True, exist_ok=True)
FIST, YOKE = OUT / "Fist_TPU.SLDPRT", bj.PROJ / "cad" / "body" / "Shoulder_Yoke.SLDPRT"
ARM, ROBOT = OUT / "Arm.SLDASM", bj.PROJ / "cad" / "body" / "Robot.SLDASM"
WEB_X = bl.WEB_X                                  # 33.5
SHELF_TOP = bp.SH_X1 - bp.SH_T                    # -151.1 (robot x)
YK_T = 6.0
YK_X0 = SHELF_TOP - YK_T                          # yoke top
ARM_Y = 116.0                                     # shoulder axis lateral: arm (y +-28.3..31.3) clears the legs (|y| <= 81.3) by 6 mm
S1_X0 = SHELF_TOP - HX0                           # S1 unit origin: its -x face under the yoke at SHELF_TOP
FIST_L, FIST_H = 30.0, 16.0


async def turned_joint(name_hint: str, x0: float, base_comp: str) -> str:
    """Insert the joint unit at x0 turned 90 deg about x (axis -> z); try plane alignments until
    the box is right (x0-22..x0+33.5 along x, the 18 mm width along y)."""
    r = await t("insert_component", {"file_path": str(bj.ASM), "x": x0}, why=f"{name_hint}: joint turned 90", n=60)
    comp = r["component"]["name"]
    r = await t("create_reference_plane", {"base_plane": "Right", "offset": abs(x0), "flip": x0 < 0,
                                           "name": f"T_{name_hint}"}, why=f"plane x={x0}", n=60)
    px = r["plane"]["name"]
    doc = mc.typed(bj.model(), "IModelDoc2")
    for al in ("closest", "aligned", "anti_aligned"):
        made = []
        for ref, pl, a in ((px, "Right", "aligned"), (f"{base_comp}/Front Plane", "Top", al),
                           (f"{base_comp}/Top Plane", "Front", "closest")):
            rr = await t("add_mate", {"entity1": ref, "entity2": f"{comp}/{pl} Plane", "mate_type": "coincident",
                                      "align": a, "keep_position": False}, why=f"{comp} {pl}", n=60)
            made.append(rr["mate"]["name"])
        b = [round(v * 1000, 1) for v in bj.top_component(comp).GetBox(False, False)]
        print(f"    {comp} ({al}) box {b}")
        if abs(b[0] - (x0 + HX0)) < 0.3 and abs(b[3] - (x0 + WEB_X)) < 0.3 and abs(b[1] + 9) < 0.3 and abs(b[4] - 9) < 0.3:
            return comp
        for m in made:
            doc.Extension.SelectByID2(m, "MATE", 0, 0, 0, False, 0, None, 0)
            doc.EditDelete()
    raise AssertionError(f"{comp}: orientation not found")


async def part_fist() -> None:
    bj.STL = OUT / "stl"
    bj.STL.mkdir(exist_ok=True)
    await t("create_part", {"name": "Fist_TPU"}, why="TPU fist: absorbs the hit (ТЗ), bolts to the elbow lever web")
    p0 = await bl.xplane(WEB_X, "X_WEB")
    sk = await bj.sketch(p0, rect(-FIST_H, -FIST_H, FIST_H, FIST_H))
    await t("create_extrusion", {"sketch_name": sk, "depth": FIST_L}, why="fist 30 x 32 x 32", n=60)
    lo, hi = bj.box()
    if hi[0] < WEB_X + FIST_L - 0.01:
        mc.typed(bj.model(), "IModelDoc2").EditUndo2(1)
        await t("create_extrusion", {"sketch_name": sk, "depth": FIST_L, "reverse_direction": True}, why="fist (rev)", n=60)
    await t("add_fillet", {"radius": 6.0, "edge_names": []}, why="round all edges R6 (no hard corners)", n=60)
    await cut(p0, [d for y, z in LINK_YZ for d in bj.circle(z, y, 0.8)], 6.0, 4 * 3.1416 * 0.64 * 6 * 0.9,
              "4 x pilot D1.6 x6 for M2 screws from inside the elbow lever")
    await bj.finish(FIST, "ABS", "fist_iso.png")  # print in TPU; no TPU in the SW material DB


async def part_yoke() -> None:
    bj.STL = bj.PROJ / "cad" / "body" / "stl"
    await t("create_part", {"name": "Shoulder_Yoke"}, why="shoulder yoke: on the torso shelf, carries both S1 housings")
    await bp.boss("Right", YK_X0, rect(-26.0, -(ARM_Y + 12.0), 26.0, ARM_Y + 12.0), YK_T, 0, "yoke 184 x 52 x 6")
    pts = [[YK_X0, s * ARM_Y + dy, dz] for s in (1, -1) for dy, dz in LINK_YZ]
    pts += [[YK_X0, y, z] for y, z in bp.SHELF_YZ]
    await t("create_hole_wizard", {"kind": "clearance", "size": "M2", "direction": [1, 0, 0], "points": pts},
            why="Hole Wizard 12 x M2: 8 to the S1 housings, 4 to the shelf (all from above)", n=120)
    for s in (1, -1):  # lightening windows between shelf and shoulders
        await cut(await bp.plane("Right", YK_X0, f"P_win{s:+d}".replace("+", "p").replace("-", "m")),
                  rect(-18.0, min(s * 40.0, s * 100.0), 18.0, max(s * 40.0, s * 100.0)), YK_T, 0.6 * 60 * 36 * YK_T,
                  f"window y{s * 52:+g}")
    await bj.finish(YOKE, "ABS", "yoke_iso.png")


async def arm_asm() -> None:
    await t("create_assembly", {"name": "Arm"}, why="arm: S1 -> S2 (turned) -> E + fist")
    r = await t("insert_component", {"file_path": str(bj.ASM)}, why="S1 shoulder pitch", n=60)
    s1 = r["component"]["name"]
    await t("add_mate", {"entity1": s1, "mate_type": "fix"}, why="S1 = reference")
    await turned_joint("S2", LINK_DX, s1)
    for path, x, why in ((bj.ASM, 2 * LINK_DX, "E elbow"), (FIST, 2 * LINK_DX, "fist on the elbow lever")):
        r = await t("insert_component", {"file_path": str(path), "x": x}, why=why, n=60)
        c = r["component"]["name"]
        rr = await t("create_reference_plane", {"base_plane": "Right", "offset": x, "name": f"A_{c}".replace("-", "_")},
                     why=f"plane x={x}", n=60)
        for ref, pl in ((rr["plane"]["name"], "Right"), (f"{s1}/Top Plane", "Top"), (f"{s1}/Front Plane", "Front")):
            await t("add_mate", {"entity1": ref, "entity2": f"{c}/{pl} Plane", "mate_type": "coincident", "align": "aligned"},
                    why=f"{c} {pl}", n=60)
    await t("save_as", {"file_path": str(ARM), "overwrite": True}, why="save arm", n=60)
    await t("check_interference", {}, why="arm interference", n=300)
    await t("close_model", {"save": False}, why="close", n=30)


async def robot_asm() -> None:
    await t("create_assembly", {"name": "Robot"}, why="robot: lower body + yoke + 2 arms")
    r = await t("insert_component", {"file_path": str(bp.ASM)}, why="lower body", n=60)
    low = r["component"]["name"]
    await t("add_mate", {"entity1": low, "mate_type": "fix"}, why="lower body = reference")
    planes: dict[tuple[str, float], str] = {}

    async def ap(base: str, v: float) -> str:
        if v == 0:
            return f"{low}/{base} Plane"
        if (base, v) not in planes:
            planes[(base, v)] = await bp.plane(base, v, f"R_{base}{v:+g}".replace(".", "_").replace("-", "m").replace("+", "p"))
        return planes[(base, v)]

    for path, xyz, why in ((YOKE, (0, 0, 0), "shoulder yoke"), (ARM, (S1_X0, ARM_Y, 0), "right arm"),
                           (ARM, (S1_X0, -ARM_Y, 0), "left arm")):
        r = await t("insert_component", {"file_path": str(path), "x": xyz[0], "y": xyz[1], "z": xyz[2]}, why=why, n=60)
        c = r["component"]["name"]
        for base, v in (("Right", xyz[0]), ("Top", xyz[1]), ("Front", xyz[2])):
            await t("add_mate", {"entity1": await ap(base, v), "entity2": f"{c}/{base} Plane", "mate_type": "coincident",
                                 "align": "aligned"}, why=f"{c} {base}", n=60)
        print(f"    {c} box {[round(v * 1000, 1) for v in bj.top_component(c).GetBox(False, False)]}")
    await t("save_as", {"file_path": str(ROBOT), "overwrite": True}, why="save robot", n=60)
    for v in ("isometric", "front", "right"):
        await t("export_image", {"file_path": str(bj.SHOTS / f"robot_{v}.png"), "format_type": "png",
                                 "view_orientation": v}, why=f"robot {v}", n=40)
    await t("check_interference", {}, why="robot interference (servo tab contacts only)", n=400)
    await t("close_model", {"save": False}, why="close", n=30)


async def main(phases: list[str]) -> None:
    t0 = time.perf_counter()
    async with bj.sw_session() as (m, a):
        bj.mcp, bj.adapter = m, a
        if "fist" in phases:
            await part_fist()
        if "yoke" in phases:
            await part_yoke()
        if "arm" in phases:
            await arm_asm()
        if "robot" in phases:
            await robot_asm()
    print(f"total {time.perf_counter() - t0:.0f} s")


if __name__ == "__main__":
    asyncio.run(main(sys.argv[1:] or ["fist", "yoke", "arm", "robot"]))
