"""Leg for robo-boxer: hip roll (turned 90 deg) -> hip pitch -> knee -> ankle (4 joint units) + foot.

Each joint unit = cad/joint/Joint_MG90S.SLDASM (build_joint.py). The lever web of joint i
bolts to the -x face of joint i+1 (4 x M2 socket screws from inside the U, before the lever
is slid onto its own servo), so joints sit LINK_DX = 55.5 mm apart along x. The foot bolts
to the ankle lever web the same way. Leg frame = joint frame: pitch axes along Y,
the leg hangs along +x (robot down), z = fore-aft.

Usage: python build_leg.py [foot] [asm]   (default: both). Needs build_joint.py output.
"""

from __future__ import annotations

import asyncio
import sys
import time

import build_joint as bj
from build_joint import AX, LINK_DX, LINK_YZ, WEB_R0, WEB_T, cut, mc, rect, t

LEG = bj.PROJ / "cad" / "leg"
LEG.mkdir(parents=True, exist_ok=True)
FOOT = LEG / "Leg_Foot.SLDPRT"
ASM = LEG / "Leg_Pitch.SLDASM"
WEB_X = AX + WEB_R0 + WEB_T            # 33.5: lever web outer face (joint frame)
SOLE_T, FOOT_HY, FOOT_HZ = 8.0, 26.0, 40.0
# steel weight insert (ТЗ: weighted feet): plate 60 x 40 x 3 glued in a pocket from below
W_HZ, W_HY, W_T = 30.0, 20.0, 3.0


async def xplane(x: float, name: str) -> str:
    r = await t("create_reference_plane", {"base_plane": "Right", "offset": abs(x), "flip": x < 0, "name": name},
                why=f"plane x={x}", n=60)
    return r["plane"]["name"]


async def part_foot() -> None:
    bj.STL = LEG / "stl"
    bj.STL.mkdir(exist_ok=True)
    await t("create_part", {"name": "Leg_Foot"}, why="foot: sole with a steel weight pocket, bolts to the ankle lever")
    p0 = await xplane(WEB_X, "X_WEB")
    sk = await bj.sketch(p0, rect(-FOOT_HZ, -FOOT_HY, FOOT_HZ, FOOT_HY))  # Right-type sketch: (u, v) = (+-Z, Y)
    await t("create_extrusion", {"sketch_name": sk, "depth": SOLE_T}, why="sole 80 x 52 x 8", n=60)
    lo, hi = bj.box()
    if hi[0] < WEB_X + SOLE_T - 0.01:
        mc.typed(bj.model(), "IModelDoc2").EditUndo2(1)
        await t("create_extrusion", {"sketch_name": sk, "depth": SOLE_T, "reverse_direction": True}, why="sole (rev)", n=60)
        lo, hi = bj.box()
    assert abs(lo[0] - WEB_X) < 0.01 and abs(hi[0] - WEB_X - SOLE_T) < 0.01, (lo, hi)
    await cut(p0, [d for y, z in LINK_YZ for d in bj.circle(z, y, 0.8)], 5.0, 4 * 3.1416 * 0.64 * 5 * 0.9,
              "4 x pilot D1.6 x5 for the M2 screws from the ankle lever")
    p1 = await xplane(WEB_X + SOLE_T, "X_SOLE")
    await cut(p1, rect(-W_HZ - 0.2, -W_HY - 0.2, W_HZ + 0.2, W_HY + 0.2), W_T + 0.2,
              (2 * W_HZ + 0.4) * (2 * W_HY + 0.4) * (W_T + 0.2), "pocket for the steel weight 60 x 40 x 3 (+0.2)")
    await t("add_chamfer", {"distance": 1.0, "edge_names": [f"{(WEB_X + SOLE_T) / 1000},{FOOT_HY / 1000},0",
                                                           f"{(WEB_X + SOLE_T) / 1000},{-FOOT_HY / 1000},0"]},
            why="sole edge chamfers 1 mm", n=60)
    await bj.finish(FOOT, "ABS", "foot_iso.png")


async def assembly() -> None:
    await t("create_assembly", {"name": "Leg_Pitch"}, why="leg: hip pitch, knee, ankle + foot")
    names = []
    for i in range(3):
        r = await t("insert_component", {"file_path": str(bj.ASM), "x": i * LINK_DX}, why=f"joint {i + 1}", n=80)
        names.append(r["component"]["name"])
    r = await t("insert_component", {"file_path": str(FOOT), "x": 2 * LINK_DX}, why="foot on the ankle lever", n=80)
    names.append(r["component"]["name"])
    await t("add_mate", {"entity1": names[0], "mate_type": "fix"}, why="hip pitch joint = reference")
    # Hip roll H1: same unit turned 90 deg about x (its axis = leg z, fore-aft); its lever web
    # bolts to the -x face of H2, so its origin is LINK_DX above H2.
    r = await t("insert_component", {"file_path": str(bj.ASM), "x": -LINK_DX}, why="hip roll joint H1", n=80)
    h1 = r["component"]["name"]
    r = await t("create_reference_plane", {"base_plane": "Right", "offset": LINK_DX, "flip": True, "name": "LEG_X_H1"},
                why="H1 plane", n=60)
    for ref, pl, al in ((r["plane"]["name"], "Right", "aligned"), (f"{names[0]}/Front Plane", "Top", "closest"),
                        (f"{names[0]}/Top Plane", "Front", "closest")):
        await t("add_mate", {"entity1": ref, "entity2": f"{h1}/{pl} Plane", "mate_type": "coincident", "align": al,
                             "keep_position": False}, why=f"H1 {pl} -> {ref.split('/')[-1]}", n=80)
    c = bj.top_component(h1)
    hb = [round(v * 1000, 1) for v in c.GetBox(False, False)]
    print(f"    {h1} box {hb}")
    # turned: x stays -77.5..-22, the joint's 18 mm width now along leg y, its height along z
    assert abs(hb[0] + 77.5) < 0.2 and abs(hb[3] + 22.0) < 0.2 and abs(hb[1] + 9) < 0.2 and abs(hb[4] - 9) < 0.2, hb
    planes: dict[float, str] = {}
    for comp, dx in zip(names[1:], (LINK_DX, 2 * LINK_DX, 2 * LINK_DX)):
        if dx not in planes:
            r = await t("create_reference_plane", {"base_plane": "Right", "offset": dx, "name": f"LEG_X{dx:g}".replace(".", "_")},
                        why=f"leg plane x={dx}", n=60)
            planes[dx] = r["plane"]["name"]
        for ref, pl in ((planes[dx], "Right"), (f"{names[0]}/Top Plane", "Top"), (f"{names[0]}/Front Plane", "Front")):
            await t("add_mate", {"entity1": ref, "entity2": f"{comp}/{pl} Plane", "mate_type": "coincident",
                                 "align": "aligned"}, why=f"{comp} {pl}", n=80)
    asm = mc.typed(bj.model(), "IAssemblyDoc")
    for n in names:
        c = bj.top_component(n)
        print(f"    {n} box {[round(v * 1000, 1) for v in c.GetBox(False, False)]}")
    await t("save_as", {"file_path": str(ASM), "overwrite": True}, why="save leg", n=60)
    for v in ("isometric", "front", "top", "right"):
        await t("export_image", {"file_path": str(bj.SHOTS / f"leg_{v}.png"), "format_type": "png",
                                 "view_orientation": v}, why=f"leg {v}", n=40)
    await t("check_interference", {}, why="leg interference (only tab contacts expected)", n=1200)
    await t("calculate_mass_properties", {"units": "metric"}, why="leg mass (solid ABS)", n=300)
    await t("close_model", {"save": False}, why="close", n=40)


async def main(phases: list[str]) -> None:
    t0 = time.perf_counter()
    async with bj.sw_session() as (m, a):
        bj.mcp, bj.adapter = m, a
        if "foot" in phases:
            await part_foot()
        if "asm" in phases:
            await assembly()
    print(f"total {time.perf_counter() - t0:.0f} s")


if __name__ == "__main__":
    asyncio.run(main(sys.argv[1:] or ["foot", "asm"]))
