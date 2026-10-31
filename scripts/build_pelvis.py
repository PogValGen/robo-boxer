"""Pelvis + waist yaw W + torso mount + both legs: robot lower body.

Robot frame = leg frame (build_leg.py): x DOWN, y lateral, z fore-aft (forward +z).
- Legs at y = +-LEG_Y; their hip-roll housings (H1) face up at x = H1_TOP and bolt to the
  pelvis plate with 4 x M2 each (screws from above, through the plate).
- W = the same joint unit, axis vertical: unit x -> robot +z, unit y -> robot -x (up),
  unit z -> robot -y. Its housing -x face bolts to the spine upright at the back of the
  pelvis (screws from behind); its lever web carries the torso mount (L: plate + shelf).
Sketch axes: Front-type (u, v) = (x, y); Top-type (x, -z); Right-type (-z, y).

Usage: python build_pelvis.py [pelvis] [mount] [asm]   (default: all)
"""

from __future__ import annotations

import asyncio
import sys
import time

import build_joint as bj
import build_leg as bl
from build_joint import AX, HX0, LINK_DX, LINK_YZ, WEB_R0, WEB_T, cut, mc, rect, t

OUT = bj.PROJ / "cad" / "body"
OUT.mkdir(parents=True, exist_ok=True)
PELVIS, MOUNT, ASM = OUT / "Pelvis.SLDPRT", OUT / "Torso_Mount.SLDPRT", OUT / "Robot_Lower.SLDASM"
LEG_Y = 50.0                         # leg centre lines -> stance ~100 mm (concept)
H1_TOP = -LINK_DX + HX0              # -77.5: top (-x) face of the hip-roll housings
PL_T, PL_HY, PL_HZ = 6.0, 62.0, 26.0
PL_X0 = H1_TOP - PL_T                # -83.5: pelvis top
# W unit placement (unit frame -> robot): axis on (y=0, z=0)
W_Z0 = -AX                           # unit origin z: unit x = AX lands on z = 0
W_X0 = PL_X0 - 28.3 - 2.0            # unit lowest point (lever bottom cheek, unit y -28.3) 2 mm above the pelvis
SP_Z1 = W_Z0 + HX0                   # -27.5: W housing -x face = front of the spine upright
SP_T, SP_HY = 6.0, 12.0
SP_X_TOP = W_X0 - 26.0               # upright top (covers the housing face up to unit y 24.8)
WEB_Z = W_Z0 + AX + WEB_R0 + WEB_T   # 28.0: W lever web front face
MT_T, SH_T = 4.0, 4.0
SH_X1 = W_X0 - 31.3 - 2.0            # shelf underside: 2 mm above the W bridge top (unit y 31.3)
SHELF_YZ = [(y, z) for y in (-30.0, 30.0) for z in (-15.0, 15.0)]   # shoulder yoke screws


def w_to_robot(ux: float, uy: float, uz: float) -> tuple[float, float, float]:
    """W unit frame -> robot frame (x, y, z)."""
    return (W_X0 - uy, -uz, W_Z0 + ux)


async def plane(base: str, value: float, name: str) -> str:
    r = await t("create_reference_plane", {"base_plane": base, "offset": abs(value), "flip": value < 0, "name": name},
                why=f"{base} {value:+g}", n=60)
    return r["plane"]["name"]


async def boss(base: str, value: float, draw, depth: float, axis: int, why: str) -> None:
    """Extrude from the plane `base`=value towards +axis by depth; undo+reverse if it went the other way."""
    sk = await bj.sketch(await plane(base, value, f"P_{why[:10]}".replace(" ", "_")), draw)
    await t("create_extrusion", {"sketch_name": sk, "depth": depth}, why=why, n=60)
    lo, hi = bj.box()
    if hi[axis] < value + depth - 0.01:
        mc.typed(bj.model(), "IModelDoc2").EditUndo2(1)
        await t("create_extrusion", {"sketch_name": sk, "depth": depth, "reverse_direction": True}, why=why + " (rev)", n=60)
        lo, hi = bj.box()
    assert hi[axis] >= value + depth - 0.01, (why, lo, hi)


async def part_pelvis() -> None:
    await t("create_part", {"name": "Pelvis"}, why="pelvis plate + spine upright for the waist joint")
    # plate x[PL_X0, H1_TOP]: Right-type plane at x = PL_X0, extrude +x
    await boss("Right", PL_X0, rect(-PL_HZ, -PL_HY, PL_HZ, PL_HY), PL_T, 0, "pelvis plate 124x52x6")
    # spine upright: Front-type plane z = SP_Z1 - SP_T, extrude +z up to the W housing face
    await boss("Front", SP_Z1 - SP_T, rect(SP_X_TOP, -SP_HY, PL_X0 + 0.01, SP_HY), SP_T, 2, "spine upright")
    # 2 gussets behind the upright (Top-type sketch: u = x, v = -z)
    for y0 in (SP_HY - 3.0, -SP_HY):
        g = [(PL_X0, -(SP_Z1 - SP_T)), (PL_X0, -(SP_Z1 - SP_T - 16.0)), (PL_X0 - 26.0, -(SP_Z1 - SP_T))]
        await boss("Top", y0, [("add_line", {"x1": a[0], "y1": a[1], "x2": b[0], "y2": b[1]})
                               for a, b in zip(g, g[1:] + g[:1])], 3.0, 1, f"gusset y{y0:+g}")
    pts = [[PL_X0, s * LEG_Y + dy, dz] for s in (1, -1) for dy, dz in LINK_YZ]
    await t("create_hole_wizard", {"kind": "clearance", "size": "M2", "direction": [1, 0, 0], "points": pts},
            why="Hole Wizard 8 x M2: pelvis -> hip-roll housings (screws from above)", n=120)
    # W housing face pattern: unit (y, z) on its -x face -> robot (x, y)
    pts = [[w_to_robot(HX0, uy, uz)[0], w_to_robot(HX0, uy, uz)[1], SP_Z1 - SP_T] for uy, uz in LINK_YZ]
    await t("create_hole_wizard", {"kind": "clearance", "size": "M2", "direction": [0, 0, 1], "points": pts},
            why="Hole Wizard 4 x M2: spine -> W housing (screws from behind)", n=120)
    await bj.finish(PELVIS, "ABS", "pelvis_iso.png")


async def part_mount() -> None:
    await t("create_part", {"name": "Torso_Mount"}, why="torso mount: plate on the W lever web + shelf over the unit")
    x_lo = W_X0 + 28.3                         # web bottom (unit y -28.3) in robot x
    # vertical plate on the web front face z[WEB_Z, WEB_Z + MT_T]
    await boss("Front", WEB_Z, rect(SH_X1 - SH_T, -10.0, x_lo, 10.0), MT_T, 2, "plate on the W lever web")
    # shelf x[SH_X1 - SH_T, SH_X1]: Right-type plane at x = SH_X1 - SH_T, extrude +x
    await boss("Right", SH_X1 - SH_T, rect(-(WEB_Z + MT_T), -45.0, 35.0, 45.0), SH_T, 0, "torso shelf 90 x 67")
    draw = [d for uy, uz in LINK_YZ for d in bj.circle(w_to_robot(0, uy, uz)[0], w_to_robot(0, uy, uz)[1], 0.8)]
    await cut(await plane("Front", WEB_Z, "P_web"), draw, 4.0, 4 * 3.1416 * 0.64 * 4 * 0.9,
              "4 x pilot D1.6 x4 for M2 screws from inside the W lever")
    # shoulder yoke (build_arm.py) sits on the shelf: 4 x pilot from the shelf top (Right-type: u=-z, v=y)
    await cut(await plane("Right", SH_X1 - SH_T, "P_shelf_top"), [d for y, z in SHELF_YZ for d in bj.circle(-z, y, 0.8)],
              3.5, 4 * 3.1416 * 0.64 * 3.5 * 0.9, "4 x pilot D1.6 for the shoulder yoke screws")
    await bj.finish(MOUNT, "ABS", "torso_mount_iso.png")


async def assembly() -> None:
    await t("create_assembly", {"name": "Robot_Lower"}, why="lower body: pelvis, 2 legs, waist W, torso mount")
    r = await t("insert_component", {"file_path": str(PELVIS)}, why="pelvis", n=60)
    pel = r["component"]["name"]
    await t("add_mate", {"entity1": pel, "mate_type": "fix"}, why="pelvis = reference")
    planes: dict[tuple[str, float], str] = {}

    async def ap(base: str, v: float) -> str:
        if v == 0:
            return f"{pel}/{base} Plane"
        if (base, v) not in planes:
            planes[(base, v)] = await plane(base, v, f"A_{base}{v:+g}".replace(".", "_").replace("-", "m").replace("+", "p"))
        return planes[(base, v)]

    for s in (1, -1):
        r = await t("insert_component", {"file_path": str(bl.ASM), "y": s * LEG_Y}, why=f"leg y={s * LEG_Y:+g}", n=60)
        leg = r["component"]["name"]
        for ref, pl in ((await ap("Right", 0), "Right"), (await ap("Top", s * LEG_Y), "Top"), (await ap("Front", 0), "Front")):
            await t("add_mate", {"entity1": ref, "entity2": f"{leg}/{pl} Plane", "mate_type": "coincident", "align": "aligned"},
                    why=f"{leg} {pl}", n=60)
    r = await t("insert_component", {"file_path": str(bj.ASM), "x": W_X0, "z": W_Z0}, why="waist joint W", n=60)
    w = r["component"]["name"]
    # unit x -> robot z, unit y -> robot -x, unit z -> robot -y. Which plane alignments give
    # that is not predictable (the third mate decides the handedness), so try the combinations
    # and keep the one whose box matches; wrong mates are deleted again.
    want = [W_X0 - 31.3, -9.0, W_Z0 + HX0, W_X0 + 28.3, 9.0, W_Z0 + AX + WEB_R0 + WEB_T]
    refs = [(await ap("Front", W_Z0), "Right"), (await ap("Right", W_X0), "Top"), (await ap("Top", 0), "Front")]
    doc = mc.typed(bj.model(), "IModelDoc2")
    for combo in [(a1, a3) for a1 in ("aligned", "anti_aligned") for a3 in ("aligned", "anti_aligned")]:
        made = []
        for (ref, pl), al in zip(refs, (combo[0], "closest", combo[1])):
            r = await t("add_mate", {"entity1": ref, "entity2": f"{w}/{pl} Plane", "mate_type": "coincident", "align": al,
                                     "keep_position": False}, why=f"W {pl} {al}", n=60)
            made.append(r["mate"]["name"])
        wb = [round(v * 1000, 1) for v in bj.top_component(w).GetBox(False, False)]
        print(f"    {w} {combo} box {wb} want {[round(v, 1) for v in want]}")
        if all(abs(a - b) < 0.3 for a, b in zip(wb, want)):
            break
        for m in made:
            doc.Extension.SelectByID2(m, "MATE", 0, 0, 0, False, 0, None, 0)
            doc.EditDelete()
    else:
        raise AssertionError(("W orientation not found", wb, want))
    r = await t("insert_component", {"file_path": str(MOUNT)}, why="torso mount (robot frame)", n=60)
    mt = r["component"]["name"]
    for pl in ("Right", "Top", "Front"):
        await t("add_mate", {"entity1": f"{pel}/{pl} Plane", "entity2": f"{mt}/{pl} Plane", "mate_type": "coincident",
                             "align": "aligned"}, why=f"mount {pl}", n=60)
    await t("save_as", {"file_path": str(ASM), "overwrite": True}, why="save lower body", n=60)
    for v in ("isometric", "front", "right", "top"):
        await t("export_image", {"file_path": str(bj.SHOTS / f"lower_{v}.png"), "format_type": "png",
                                 "view_orientation": v}, why=f"lower body {v}", n=40)
    await t("check_interference", {}, why="interference (only servo tab contacts expected)", n=2500)
    await t("close_model", {"save": False}, why="close", n=40)


async def main(phases: list[str]) -> None:
    t0 = time.perf_counter()
    bj.STL = OUT / "stl"
    bj.STL.mkdir(exist_ok=True)
    async with bj.sw_session() as (m, a):
        bj.mcp, bj.adapter = m, a
        if "pelvis" in phases:
            await part_pelvis()
        if "mount" in phases:
            await part_mount()
        if "asm" in phases:
            await assembly()
    print(f"total {time.perf_counter() - t0:.0f} s")


if __name__ == "__main__":
    asyncio.run(main(sys.argv[1:] or ["pelvis", "mount", "asm"]))
