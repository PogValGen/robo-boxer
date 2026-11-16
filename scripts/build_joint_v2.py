"""Joint v2 (2026-11-06): simplified, compact, no bearings, no screws.

User brief: "максимально без крепежных винтов, подшипники пока исключи — втулки (на одной детали
втулка, на другой отверстие), лишние пустоты удалить (между статором и ротором), просчитать, чтобы
не задевали при движении, максимально компактно; v1 не удалять".

Frame = MG90S GrabCAD model (as v1): shaft axis Y at x=5.5, z=0; tabs along X.
  Housing (stator): MG90S pocket (user-fitted 23.2 x 13.0), servo rests on its tabs; cable groove /
      channel / -x tunnel as v1; post at -x for the bridge; bottom spigot D6 coaxial with the shaft
      (plain bushing instead of 623ZZ + pin); 4 link pegs D3 on the -x face (press, instead of 4 x M2).
  Lever (rotor, U) in TWO halves (rev b: no open contours at the axis): bottom half with a CLOSED
      bushing hole D6.2 (pushed UP onto the housing spigot), top half with a CLOSED horn pocket (dropped
      1.6 onto the horn) and the top spigot D6 into the bridge; halves meet on the web (tongue/groove at
      y = 0) and are tied by the next link's 4 pegs (2 holes D3 in each half). Web R15.5..18.5
      (v1 R25..28: computed sweep, free +-120 deg).
  Bridge: hole D6.2 over the lever spigot, 2 press pegs D2.5 into the post.
  Servo held without screws: tabs on the housing top (down), horn in the lever recess (up);
  horn centre screw not needed (lever held axially by the bridge and the housing).
Link pitch 45.3 mm (v1 55.5).

Usage: python build_joint_v2.py [parts | housing bridge levertop leverbot horn] [asm]
Assembly order: servo + horn on the spline -> bottom half up onto the housing spigot -> top half down onto
the horn and the tongue -> bridge down onto the top spigot, 2 pegs into the post -> next link on the 4 pegs.
"""
from __future__ import annotations

import asyncio
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import build_joint as base  # noqa: E402  (helpers; module globals redirected to v2 below)
from build_joint import (P, S, PK, H, AX, mc, t, sketch, slot, rect, circle, boss_up, front_boss, cut,  # noqa: E402
                         yplane, box, vol, open_docs, sw_session)

PROJ = base.PROJ
OUT = PROJ / "cad" / "v2" / "joint"
SHOTS, STL = OUT / "screens", OUT / "stl"
for d in (OUT, SHOTS, STL):
    d.mkdir(parents=True, exist_ok=True)
base.OUT, base.SHOTS, base.STL, base.LOG = OUT, SHOTS, STL, OUT / "build_log.json"
SERVO = base.SERVO
HOUSING, BRIDGE, HORN = (OUT / f"{n}.SLDPRT" for n in ("J2_Housing", "J2_Bridge", "MG90S_Horn"))
# lever in two halves (2026-11-06, user: "открытый контур на оси — ненадёжно"): both axis holes and the
# horn pocket are CLOSED; the halves meet on the web (tongue/groove at y = SPLIT_Y) and are tied by the
# next link's 4 pegs (2 holes in each half)
LEVER_TOP, LEVER_BOT = OUT / "J2_LeverTop.SLDPRT", OUT / "J2_LeverBot.SLDPRT"
BBOX_JSON = OUT / "bboxes.json"
ASM = OUT / "Joint2_MG90S.SLDASM"

GAP = 0.3                                # running clearance between stator and rotor faces
# --- housing
TOP = S["tab_bottom_y"]                  # 5.15: tabs rest here
FLOOR = S["bottom_y"] - PK["floor_gap"]  # -13.7 (1.8 cable channel under the servo)
BOT = FLOOR - 2.0                        # -15.7
HX1, HZ = 16.5, 8.5                      # +x end (tab support), half width (pocket 13 + 2 walls)
POST_X0, POST_X1 = -21.3, -16.3          # post 5 thick behind the -x tab
HX0 = POST_X0
SPIG_D, SPIG_HOLE = 6.0, 6.2             # plain bushings
SPIG_L = 3.0
# --- lever
LV_TOP0 = H["plate_top_y"] - H["plate_t"]   # 19.3: cheek bottom = horn plate bottom (plate in a recess)
LV_TOP1 = LV_TOP0 + 3.5                     # 22.8
LV_BOT1 = BOT - GAP                         # -16.0
LV_BOT0 = LV_BOT1 - 3.0                     # -19.0
CHEEK_HW = 7.0
WEB_R0, WEB_T = 15.5, 3.0                   # web radius (sweep +-110 deg, 1 mm clearance)
X_END = AX + WEB_R0 + WEB_T                 # 24.0
SPLIT_Y = 0.0                               # web split between the halves (link pegs at y +-4)
TONGUE = (22.0, 23.0, 2.0, 6.5)             # x0, x1, height, half length (z) on the bottom half web
GROOVE = (21.9, 23.1, 2.2, 6.7)             # +0.1 per side in the top half web
# --- bridge
BR_Y0 = LV_TOP1 + GAP                       # 23.1 = post top
BR_T = 4.0
BR_HW = 6.5
SPIG_TOP_Y1 = BR_Y0 + BR_T - 0.6            # lever spigot top (inside the bridge hole)
BR_PEGS = [(-18.8, 4.5), (-18.8, -4.5)]     # (x, z) D2.5 press pegs into the post
BR_PEG_D, BR_PEG_L = 2.5, 4.0
# --- link (lever web <-> next housing -x face); square -> also fits a joint turned 90 deg
LINK_YZ = [(y, z) for y in (-4.0, 4.0) for z in (-4.0, 4.0)]
LINK_D, LINK_PEG_L = 3.0, 2.8
LINK_DX = X_END - HX0                       # 45.3


async def xboss(x0: float, draw, depth: float, outward: int, why: str) -> None:
    """Boss on a plane x = x0 (Right-type sketch: (u, v) = (+-Z, Y)), `outward` = -1 grows toward -x."""
    # sketch on the OUTER plane and extrude back to the face (an extrusion from the face plane went
    # into the body and the reversed retry was refused, 2026-11-06)
    xs = x0 + outward * depth
    r = await t("create_reference_plane", {"base_plane": "Right", "offset": abs(xs), "flip": xs < 0,
                                           "name": f"X{xs:+.2f}".replace(".", "_")}, why=f"plane x={xs}", n=60)
    sk = await sketch(r["plane"]["name"], draw)
    v0 = vol()
    lo0, hi0 = box()
    await t("create_extrusion", {"sketch_name": sk, "depth": depth}, why=why, n=80)
    lo, hi = box()
    good = lambda lo, hi: abs(lo[0] - xs) < 0.05 if outward < 0 else abs(hi[0] - xs) < 0.05  # noqa: E731
    if not good(lo, hi) or vol() - v0 < 1:
        print(f"    went the wrong way ({lo[0]}..{hi[0]}), redo reversed")
        mc.typed(base.model(), "IModelDoc2").EditUndo2(1)
        sk = await sketch(r["plane"]["name"], draw)   # undo eats the sketch: new one for the retry
        await t("create_extrusion", {"sketch_name": sk, "depth": depth, "reverse_direction": True}, why=why + " (rev)", n=80)
        lo, hi = box()
    assert good(lo, hi), (why, lo, hi)


async def boss_down(y1: float, draw, depth: float, why: str) -> None:
    """Boss hanging DOWN from y1 by depth (sketch on y1 - depth, extruded up)."""
    await boss_up(y1 - depth, draw, depth, why)


# ----------------------------------------------------------------------------- parts
async def part_housing() -> None:
    await t("create_part", {"name": "J2_Housing"}, why="v2 housing: pocket, post, spigot D6, link pegs")
    await front_boss(rect(HX0, BOT, HX1, TOP), 2 * HZ, "block")
    await front_boss(rect(POST_X0, TOP, POST_X1, BR_Y0), 2 * HZ, "post for the bridge")
    L, W = PK["length"], PK["width"]
    await cut(await yplane(TOP), rect(-L / 2, -W / 2, L / 2, W / 2), TOP - FLOOR, L * W * (TOP - FLOOR),
              f"pocket {L}x{W} (user-fitted)")
    c = P["cable"]
    await cut(await yplane(FLOOR), rect(L / 2, -W / 2, L / 2 + c["groove_depth"], W / 2), S["bottom_y"] + c["exit_h"] - FLOOR,
              60, "cable groove in the +x end wall (shaft side)")
    await cut("Front", rect(HX0, FLOOR, -L / 2, FLOOR + c["h"]), None, 300, "cable exit tunnel (through Z) at the -x end")
    g = [(POST_X1, TOP), (-11.0, TOP), (POST_X1, TOP + 9.85)]
    await front_boss([("add_line", {"x1": a[0], "y1": a[1], "x2": b[0], "y2": b[1]}) for a, b in zip(g, g[1:] + g[:1])],
                     2 * HZ, "post gusset")
    await cut(await yplane(TOP + 9.85), rect(POST_X1, -6.5, -10.9, 6.5), 9.85, 250,
              "split the gusset into 2 side ribs (clear the servo tab)")
    await boss_down(BOT, circle(AX, 0, SPIG_D / 2), SPIG_L, "bottom spigot D6 (plain bushing in the lever fork)")
    for x, z in BR_PEGS:
        v0 = vol()
        await t("tutorial_simple_hole", {"plane": await yplane(BR_Y0), "center_x": x, "center_y": -z,
                                        "diameter": BR_PEG_D, "depth": BR_PEG_L + 0.3}, why=f"post hole D{BR_PEG_D} ({x},{z})", n=60)
        assert v0 - vol() > 10, "post hole not cut"
    await xboss(HX0, [d for y, z in LINK_YZ for d in circle(z, y, LINK_D / 2)], LINK_PEG_L, -1,
                "4 link pegs D3 x2.8 on the -x face (press into the previous lever web)")
    save_bbox("J2_Housing")
    await base.finish(HOUSING, "ABS", "housing_iso.png")


async def part_bridge() -> None:
    await t("create_part", {"name": "J2_Bridge"}, why="v2 bridge: bushing hole over the lever spigot")
    await boss_up(BR_Y0, slot(AX, 0, HX0, BR_HW), BR_T, "plate R6.5 around the axis to the post")
    await cut(await yplane(BR_Y0), circle(AX, 0, SPIG_HOLE / 2), BR_T, 3.1416 * 9.6 * BR_T * 0.9, "bushing hole D6.2")
    await boss_down(BR_Y0, [d for x, z in BR_PEGS for d in circle(x, -z, BR_PEG_D / 2)], BR_PEG_L,
                    "2 press pegs D2.5 into the post")
    save_bbox("J2_Bridge")
    await base.finish(BRIDGE, "ABS", "bridge_iso.png")


def save_bbox(name: str) -> None:
    lo, hi = box()
    d = json.loads(BBOX_JSON.read_text()) if BBOX_JSON.exists() else {}
    d[name] = {"min": lo, "max": hi}
    BBOX_JSON.write_text(json.dumps(d, indent=1))


async def part_lever_bot() -> None:
    await t("create_part", {"name": "J2_LeverBot"}, why="v2 lever, bottom half: closed bushing hole on the housing spigot")
    await boss_up(LV_BOT0, slot(AX, 0, X_END, CHEEK_HW), LV_BOT1 - LV_BOT0, "bottom cheek")
    await front_boss(rect(AX + WEB_R0, LV_BOT0, X_END, SPLIT_Y), 2 * CHEEK_HW, "lower web R15.5..18.5")
    x0, x1, h, hz = TONGUE
    await boss_up(SPLIT_Y, rect(x0, -hz, x1, hz), h, "tongue into the top half (alignment)")
    await cut(await yplane(LV_BOT0), circle(AX, 0, SPIG_HOLE / 2), LV_BOT1 - LV_BOT0, 3.1416 * 9.6 * 3 * 0.9,
              "CLOSED bushing hole D6.2 for the housing spigot")
    r = await t("create_reference_plane", {"base_plane": "Right", "offset": X_END, "flip": False, "name": "X_LINK"},
                why="web outer face", n=60)
    await cut(r["plane"]["name"], [d for y, z in LINK_YZ if y < SPLIT_Y for d in circle(z, y, LINK_D / 2)], WEB_T,
              2 * 3.1416 * 2.25 * WEB_T * 0.9, "2 holes D3 (next link pegs tie both halves)")
    await t("add_fillet", {"radius": 1.5, "edge_names": [f"{(AX + WEB_R0) / 1000},{LV_BOT1 / 1000},0"]},
            why="inner web/cheek fillet R1.5", n=80)
    save_bbox("J2_LeverBot")
    await base.finish(LEVER_BOT, "ABS", "lever_bot_iso.png")


async def part_lever_top() -> None:
    await t("create_part", {"name": "J2_LeverTop"}, why="v2 lever, top half: closed horn pocket, spigot into the bridge")
    await boss_up(LV_TOP0, slot(AX, 0, X_END, CHEEK_HW), LV_TOP1 - LV_TOP0, "top cheek (on the horn)")
    await front_boss(rect(AX + WEB_R0, SPLIT_Y, X_END, LV_TOP1), 2 * CHEEK_HW, "upper web R15.5..18.5")
    await boss_up(LV_TOP1, circle(AX, 0, SPIG_D / 2), SPIG_TOP_Y1 - LV_TOP1, "top spigot D6 into the bridge")
    pt = H["plate_t"]
    hw = H["arm_w"] / 2 + 0.2
    hub = H["hub_d"] / 2 + 0.2
    await cut(await yplane(LV_TOP0), circle(AX, 0, hub), pt, 3.1416 * hub * hub * pt * 0.9, "CLOSED horn pocket: hub D7.4")
    await cut(await yplane(LV_TOP0), slot(AX, 0, AX + H["arm_r"] + 0.2, hw), pt, 2 * hw * H["arm_r"] * pt * 0.5,
              "CLOSED horn pocket: arm +0.2 (torque by form fit, the lever drops 1.6 onto the horn)")
    x0, x1, h, hz = GROOVE
    await cut(await yplane(SPLIT_Y), rect(x0, -hz, x1, hz), h, (x1 - x0) * 2 * hz * h * 0.8, "groove for the tongue")
    r = await t("create_reference_plane", {"base_plane": "Right", "offset": X_END, "flip": False, "name": "X_LINK"},
                why="web outer face", n=60)
    await cut(r["plane"]["name"], [d for y, z in LINK_YZ if y > SPLIT_Y for d in circle(z, y, LINK_D / 2)], WEB_T,
              2 * 3.1416 * 2.25 * WEB_T * 0.9, "2 holes D3 (next link pegs tie both halves)")
    await t("add_fillet", {"radius": 1.5, "edge_names": [f"{(AX + WEB_R0) / 1000},{LV_TOP0 / 1000},0"]},
            why="inner web/cheek fillet R1.5", n=80)
    save_bbox("J2_LeverTop")
    await base.finish(LEVER_TOP, "ABS", "lever_top_iso.png")


async def part_horn() -> None:
    base.HORN = HORN
    await base.part_horn()


# ----------------------------------------------------------------------------- assembly
async def assembly() -> None:
    await t("create_assembly", {"name": "Joint2_MG90S"}, why="v2 joint assembly")
    for p in (SERVO, HOUSING, BRIDGE, HORN, LEVER_BOT, LEVER_TOP):
        await t("insert_component", {"file_path": str(p)}, why=f"{p.stem} at origin (frame = servo frame)", n=80)
    await t("add_mate", {"entity1": "MG90S_grabcad-1", "mate_type": "fix"}, why="servo = reference")
    for comp in ("J2_Housing-1", "J2_Bridge-1", "MG90S_Horn-1", "J2_LeverBot-1", "J2_LeverTop-1"):
        for pl in ("Right", "Top", "Front"):
            await t("add_mate", {"entity1": f"MG90S_grabcad-1/{pl} Plane", "entity2": f"{comp}/{pl} Plane",
                                 "mate_type": "coincident", "align": "aligned"}, why=f"{comp} {pl}", n=80)
    await t("save_as", {"file_path": str(ASM), "overwrite": True}, why="save assembly", n=80)
    for v in ("isometric", "front", "top", "right"):
        await t("export_image", {"file_path": str(SHOTS / f"asm_{v}.png"), "format_type": "png",
                                 "view_orientation": v}, why=f"assembly {v}", n=40)
    await t("check_interference", {}, why="interference (expect only coincident touches)", n=1500)
    await t("calculate_mass_properties", {"units": "metric"}, why="unit mass", n=300)
    await t("close_model", {"save": False}, why="close", n=40)


async def main(phases: list[str]) -> None:
    t0 = time.perf_counter()
    async with sw_session() as (m, a):
        base.mcp, base.adapter = m, a
        print("open docs at start:", open_docs(a))
        parts = {"housing": part_housing, "bridge": part_bridge, "levertop": part_lever_top, "leverbot": part_lever_bot,
                 "horn": part_horn}
        todo = list(parts.values()) if "parts" in phases else [parts[k] for k in phases if k in parts]
        for f in todo:
            ts = time.perf_counter()
            await f()
            print(f"=== {f.__name__}: {time.perf_counter() - ts:.0f} s")
        if "asm" in phases:
            await assembly()
    print(f"total {time.perf_counter() - t0:.0f} s; LINK_DX = {LINK_DX}")


if __name__ == "__main__":
    asyncio.run(main(sys.argv[1:] or ["parts", "asm"]))
