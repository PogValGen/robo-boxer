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

import robo_version as RV  # ROBO_V=2 -> joint v2, output cad/v2
import build_joint as bj
import build_leg as bl
from build_joint import AX, HX0, LINK_DX, LINK_YZ, WEB_R0, WEB_T, cut, mc, rect, t

OUT = RV.CAD / "body"
OUT.mkdir(parents=True, exist_ok=True)
PELVIS, MOUNT, ASM = OUT / "Pelvis.SLDPRT", OUT / "Torso_Mount.SLDPRT", OUT / "Robot_Lower.SLDASM"
BATT, CTRL = OUT / "Battery_2x18650.SLDPRT", OUT / "Controller_RTrobot32.SLDPRT"
# leg centre lines: v1 +-50 (stance ~100); v2 +-25 (2026-11-07): the stance hip roll holds the upper body on a
# lever = half the hip width -> 0.33 N*m at 50 mm, 0.16 N*m at 25 mm (MG90S 0.216 @6 V)
# v2 rev (2026-11-07, user): hip-roll servos H1 raised above the pelvis line beside the waist (legs 51 mm shorter,
# COM lower); legs at +-28: H1 inner edge 19.5 clears the torso mount / W lever sweep up to ~35 deg of waist yaw
# (punch uses 20); hip-roll torque ~0.18 N*m (~84 % @6 V, printed masses)
LEG_Y = 28.0 if RV.V2 else 50.0
H1_TOP = -LINK_DX + HX0              # top (-x) face of the hip-roll housings (v1 -77.5)
PL_T, PL_HY, PL_HZ = 6.0, (41.0 if RV.V2 else 62.0), 26.0
PL_X0 = H1_TOP - PL_T                # -83.5: pelvis top
# W unit placement (unit frame -> robot): axis on (y=0, z=0)
W_Z0 = -AX                           # unit origin z: unit x = AX lands on z = 0
W_X0 = PL_X0 + RV.Y_MIN - 2.0        # unit lowest point (lever bottom cheek, unit y Y_MIN) 2 mm above the pelvis
SP_Z1 = W_Z0 + HX0                   # -27.5: W housing -x face = front of the spine upright
SP_T, SP_HY = 6.0, 12.0
SP_X_TOP = W_X0 - (RV.FACE_Y1 + 1.2)  # upright top (covers the housing -x face)
WEB_Z = W_Z0 + AX + WEB_R0 + WEB_T   # 28.0: W lever web front face
MT_T, SH_T = 4.0, 4.0
SH_X1 = W_X0 - RV.Y_MAX - 2.0        # shelf underside: 2 mm above the W bridge top (unit y Y_MAX)
SHELF_YZ = [(y, z) for y in (-30.0, 30.0) for z in (-15.0, 15.0)]   # shoulder yoke screws
# raised H1 (v2): its -x face 5 mm below the shelf underside (3 mm cradle plate + 2 mm to the rotating shelf)
H1F = SH_X1 + 5.0 if RV.V2 else H1_TOP        # robot x of the H1 housing -x face
LEG_DX = H1F - H1_TOP                          # legs shifted up (v2: -51.1 mm)
CR_T, CR_HY, CR_Z0, CR_Z1 = 3.0, 10.0, -31.0, 10.0   # cradle plate thickness, half width (y), z range
CR_WALL_Z1 = -28.5                             # back wall front face (H1 bridge at z -27.1)
MT_HY = 7.0 if RV.V2 else 10.0                 # torso mount plate half width (v2 narrower: waist sweep)
DECK_Z1 = -28.0                                # v2 rear deck front edge: behind the H1 webs / H2 housings (z >= -27.1)
# --- v2 electronics (2026-11-07) ----------------------------------------------------------------
# Battery: 2 Li-ion cells, marking not given by the user -> 18650 assumed (verify); a common 2 x 18650
# side-by-side holder is ~77 x 41 x 20 mm (verify against the real one). Tray under the pelvis between the
# hip-roll housings (only stators there; H1 pivots are 6 mm below the tray floor), opens to the front,
# held by a strap (no screws).
# v2 (hips +-25): no room between the hip-roll housings any more -> the pack stands upright behind the legs,
# hanging from the rear deck: 78 (y) x 42 (x, down) x 21 (z) inside; open at +y (slides in sideways), strap.
BT_IN_Y, BT_IN_X, BT_IN_Z = 78.0, 42.0, 21.0   # interior: length (y), height (x), thickness (z)
BT_W = 2.0
BT_ZF = -30.0                                  # interior front face (z): 2.9 mm behind the H1 bridge (z -27.1)
BT_ZB = BT_ZF - BT_IN_Z                        # interior back face
BT_FLOOR = H1_TOP + BT_IN_X                    # floor top (robot x)
BATT_G = 2 * 46.0 + 12.0                       # 2 cells (typ. 18650 ~46 g) + holder (verify)
# Controller RTrobot Sync 32 ch: board 45 x 64, 4 holes on 39 x 58 (manual hw3 p.32,
# docs/images/2026-11-07_rtrobot32_dimensions_p32.png); hole D and board mass not published (verify).
# Vertical back panel behind the spine: below the torso shelf, outside its yaw sweep.
BP_Z1 = SP_Z1 - SP_T - 16.0                    # gusset back = panel front face (z)
BP_T = 2.0
BP_Z0 = BP_Z1 - BP_T
BP_X_TOP = SH_X1 + 4.0                         # 4 mm below the shelf underside
BP_HY = 41.0
BD_X0 = (PL_X0 + BP_X_TOP) / 2                 # board centre (robot x)
BD_HOLES = [(BD_X0 + dx, dy) for dx in (-19.5, 19.5) for dy in (-29.0, 29.0)]   # (x, y), 39 along x, 58 along y
SO_D, SO_H, PIN_D, PIN_L = 6.0, 5.0, 2.8, 2.5   # standoffs + pins into the board holes (hole D verify)
BD_T = 1.6
CTRL_G = 25.0                                  # board + headers (verify: not in the manual)


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
    if RV.V2:   # raised hips: the H2 housings (45.8 mm along y) cross the pelvis level -> no plate under the legs,
        # only a rear deck behind them (z <= DECK_Z1) carrying spine, H1 cradles, controller panel, battery tray
        await boss("Right", PL_X0, rect(-min(BP_Z0, BT_ZB - BT_W), -BP_HY, -DECK_Z1, BP_HY), PL_T, 0, "rear deck")
    else:
        await boss("Right", PL_X0, rect(-PL_HZ, -PL_HY, PL_HZ, PL_HY), PL_T, 0, "pelvis plate 124x52x6")
    # spine upright: Front-type plane z = SP_Z1 - SP_T, extrude +z up to the W housing face
    await boss("Front", SP_Z1 - SP_T, rect(SP_X_TOP, -SP_HY, PL_X0 + 0.01, SP_HY), SP_T, 2, "spine upright")
    # 2 gussets behind the upright (Top-type sketch: u = x, v = -z)
    for y0 in (SP_HY - 3.0, -SP_HY):
        g = [(PL_X0, -(SP_Z1 - SP_T)), (PL_X0, -(SP_Z1 - SP_T - 16.0)), (PL_X0 - 26.0, -(SP_Z1 - SP_T))]
        await boss("Top", y0, [("add_line", {"x1": a[0], "y1": a[1], "x2": b[0], "y2": b[1]})
                               for a, b in zip(g, g[1:] + g[:1])], 3.0, 1, f"gusset y{y0:+g}")
    if RV.V2:   # raised H1: L-cradles (top plate with peg holes + back wall to the rear deck); notches for H2
        for sg in (1, -1):
            y0, y1 = sg * LEG_Y - CR_HY, sg * LEG_Y + CR_HY
            await boss("Right", H1F - CR_T, rect(-CR_Z1, y0, -CR_Z0, y1), CR_T, 0, f"H1 cradle plate y{sg * LEG_Y:+g}")
            await boss("Front", CR_Z0, rect(H1F - CR_T, y0, PL_X0 + 0.01, y1), CR_WALL_Z1 - CR_Z0, 2,
                       f"H1 cradle back wall y{sg * LEG_Y:+g}")
        await cut(await plane("Right", H1F, "P_h1"), [d for s in (1, -1) for dy, dz in LINK_YZ
                                                      for d in bj.circle(dz, s * LEG_Y + dy, RV.LINK_D / 2)],
                  RV.PEG_L + 0.2, 8 * 3.1416 * 2.25 * RV.PEG_L * 0.9, "8 holes D3 for the hip-roll housing pegs")

        await cut(await plane("Front", SP_Z1, "P_spine"), [d for uy, uz in LINK_YZ
                                                            for d in bj.circle(w_to_robot(HX0, uy, uz)[0], w_to_robot(HX0, uy, uz)[1], RV.LINK_D / 2)],
                  RV.PEG_L + 0.2, 4 * 3.1416 * 2.25 * RV.PEG_L * 0.9, "4 holes D3 for the W housing pegs")
    else:
        pts = [[PL_X0, s * LEG_Y + dy, dz] for s in (1, -1) for dy, dz in LINK_YZ]
        await t("create_hole_wizard", {"kind": "clearance", "size": "M2", "direction": [1, 0, 0], "points": pts},
                why="Hole Wizard 8 x M2: pelvis -> hip-roll housings (screws from above)", n=120)
        # W housing face pattern: unit (y, z) on its -x face -> robot (x, y)
        pts = [[w_to_robot(HX0, uy, uz)[0], w_to_robot(HX0, uy, uz)[1], SP_Z1 - SP_T] for uy, uz in LINK_YZ]
        await t("create_hole_wizard", {"kind": "clearance", "size": "M2", "direction": [0, 0, 1], "points": pts},
                why="Hole Wizard 4 x M2: spine -> W housing (screws from behind)", n=120)
    if RV.V2:
        # rear deck (pelvis plate extended back) + vertical controller panel + standoffs with pins
        # (rear deck already made as the base feature)
        await boss("Front", BP_Z0, rect(BP_X_TOP, -BP_HY, PL_X0 + 0.01, BP_HY), BP_T, 2, "controller panel 2 mm")
        await RV.zboss(BP_Z0, [d for x, y in BD_HOLES for d in bj.circle(x, y, SO_D / 2)], SO_H, -1,
                       "4 standoffs D6 x5 (39 x 58)")
        await RV.zboss(BP_Z0 - SO_H, [d for x, y in BD_HOLES for d in bj.circle(x, y, PIN_D / 2)], PIN_L, -1,
                       "4 pins D2.8 into the board holes (no screws; D verify)")
        # battery tray behind the legs, hanging from the deck (Right-type sketch u = -z, v = y); open at +y
        hy, w = BT_IN_Y / 2, BT_W
        await boss("Right", H1_TOP, rect(-BT_ZF, -hy - w, -(BT_ZF + w), hy + w), BT_IN_X, 0, "battery tray front wall")
        await boss("Right", H1_TOP, rect(-(BT_ZB - w), -hy - w, -BT_ZB, hy + w), BT_IN_X, 0, "battery tray back wall")
        await boss("Right", H1_TOP, rect(-BT_ZB, -hy - w, -BT_ZF, -hy), BT_IN_X, 0, "battery tray end wall -y")
        await boss("Right", BT_FLOOR, rect(-(BT_ZB - w), -hy - w, -(BT_ZF + w), hy + w), BT_W, 0, "battery tray floor")
    await bj.finish(PELVIS, "ABS", "pelvis_iso.png")


async def part_electronics() -> None:
    """Envelope dummies (robot frame) for clearance and the physics mass: battery pack and controller."""
    await t("create_part", {"name": "Battery_2x18650"}, why="battery envelope 2 x 18650 + holder (verify)")
    await boss("Right", H1_TOP + 0.3, rect(-(BT_ZF - 0.3), -(BT_IN_Y / 2 - 0.5), -(BT_ZB + 0.3), BT_IN_Y / 2 - 0.5),
               BT_IN_X - 1.0, 0, "pack 77 x 41 x 20.4 (upright)")
    await bj.finish(BATT, "ABS", "battery_iso.png")
    await t("create_part", {"name": "Controller_RTrobot32"}, why="RTrobot 32 ch board envelope 45 x 64 x 1.6")
    await boss("Front", BP_Z0 - SO_H - BD_T, rect(BD_X0 - 22.5, -32.0, BD_X0 + 22.5, 32.0), BD_T, 2, "board 45 x 64")
    await cut(await plane("Front", BP_Z0 - SO_H - BD_T, "P_bd"), [d for x, y in BD_HOLES for d in bj.circle(x, y, 1.5)],
              BD_T, 4 * 3.1416 * 2.25 * BD_T * 0.8, "4 holes (D3 assumed)")
    await bj.finish(CTRL, "ABS", "controller_iso.png")


async def part_mount() -> None:
    await t("create_part", {"name": "Torso_Mount"}, why="torso mount: plate on the W lever web + shelf over the unit")
    x_lo = W_X0 - RV.Y_MIN                     # web bottom (unit y Y_MIN) in robot x
    # vertical plate on the web front face z[WEB_Z, WEB_Z + MT_T]
    await boss("Front", WEB_Z, rect(SH_X1 - SH_T, -MT_HY, x_lo, MT_HY), MT_T, 2, "plate on the W lever web")
    # shelf x[SH_X1 - SH_T, SH_X1]: Right-type plane at x = SH_X1 - SH_T, extrude +x
    await boss("Right", SH_X1 - SH_T, rect(-(WEB_Z + MT_T), -45.0, 35.0, 45.0), SH_T, 0, "torso shelf 90 x 67")
    if RV.V2:
        draw = [d for uy, uz in LINK_YZ for d in bj.circle(w_to_robot(0, uy, uz)[0], w_to_robot(0, uy, uz)[1], RV.LINK_D / 2)]
        await RV.zboss(WEB_Z, draw, RV.PEG_L, -1, "4 press pegs D3 into the W lever web (no screws)")
        await RV.xboss(SH_X1 - SH_T, [d for y, z in SHELF_YZ for d in bj.circle(-z, y, RV.LINK_D / 2)], RV.PEG_L, -1,
                       "4 pegs D3 up from the shelf into the shoulder yoke")
    else:
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
        r = await t("insert_component", {"file_path": str(bl.ASM), "x": LEG_DX, "y": s * LEG_Y}, why=f"leg y={s * LEG_Y:+g}", n=60)
        leg = r["component"]["name"]
        for ref, pl in ((await ap("Right", LEG_DX), "Right"), (await ap("Top", s * LEG_Y), "Top"), (await ap("Front", 0), "Front")):
            await t("add_mate", {"entity1": ref, "entity2": f"{leg}/{pl} Plane", "mate_type": "coincident", "align": "aligned"},
                    why=f"{leg} {pl}", n=60)
    r = await t("insert_component", {"file_path": str(bj.ASM), "x": W_X0, "z": W_Z0}, why="waist joint W", n=60)
    w = r["component"]["name"]
    # unit x -> robot z, unit y -> robot -x, unit z -> robot -y. Which plane alignments give
    # that is not predictable (the third mate decides the handedness), so try the combinations
    # and keep the one whose box matches; wrong mates are deleted again.
    want = [W_X0 - RV.Y_MAX, -RV.HZ, W_Z0 + RV.X_MIN, W_X0 - RV.Y_MIN, RV.HZ, W_Z0 + AX + WEB_R0 + WEB_T]
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
    for path, why in ((MOUNT, "torso mount"), *(((BATT, "battery pack"), (CTRL, "controller")) if RV.V2 else ())):
        r = await t("insert_component", {"file_path": str(path)}, why=f"{why} (robot frame)", n=60)
        mt = r["component"]["name"]
        for pl in ("Right", "Top", "Front"):
            await t("add_mate", {"entity1": f"{pel}/{pl} Plane", "entity2": f"{mt}/{pl} Plane", "mate_type": "coincident",
                                 "align": "aligned"}, why=f"{why} {pl}", n=60)
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
        if RV.V2 and ("electronics" in phases or "pelvis" in phases):
            await part_electronics()
        if "asm" in phases:
            await assembly()
    print(f"total {time.perf_counter() - t0:.0f} s")


if __name__ == "__main__":
    asyncio.run(main(sys.argv[1:] or ["pelvis", "mount", "asm"]))
