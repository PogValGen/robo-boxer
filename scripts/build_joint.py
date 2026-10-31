"""Joint unloading unit (ТЗ разд. 4) for robo-boxer, built through solidworks-mcp.

Frame = MG90S GrabCAD model: shaft axis Y up at x=5.5, z=0; tabs along X.
Load path: lever -> spigot D5 -> 685ZZ in the bridge -> housing post;
lever bottom cheek -> pin ISO 2338 3x12 -> 623ZZ in the housing boss.
Assembly (v0.3): lever + horn slide in sideways 3.6 above the shaft, drop onto the spline;
the lever is held axially between 685 and 623 (no horn centre screw needed).
Link: the lever web bolts to the -x face of the next housing (4 x M2, LINK_YZ).
The servo shaft only gets torque through the horn (keyed in a recess of the lever).

Usage: python build_joint.py [parts | housing bridge lever horn toolbox] [asm]   (default: parts asm)
Rebuild only what changed: e.g. `build_joint.py lever` (~25 s) instead of all parts (~2.5 min).
The leg (build_leg.py) uses the joint as ONE sub-assembly file, so a change reaches all 4 joints.
Parameters: cad/mg90s_params.json (verify=true -> to be measured).
"""

from __future__ import annotations

import asyncio
import json
import sys
import time
from pathlib import Path

MCP_ROOT = Path(r"D:\AI prodject\grok\projects\solidworks-mcp\SolidworksMCP-python")
sys.path.insert(0, str(MCP_ROOT / "scripts"))
sys.path.insert(0, str(MCP_ROOT / "src"))
from audit_harness import _unwrap, open_docs, sw_session  # noqa: E402
from solidworks_mcp.tools import _model_com as mc  # noqa: E402

PROJ = Path(__file__).resolve().parents[1]
P = json.loads((PROJ / "cad" / "mg90s_params.json").read_text(encoding="utf-8"))
OUT = PROJ / "cad" / "joint"
SHOTS = OUT / "screens"
STL = OUT / "stl"
for d in (OUT, SHOTS, STL):
    d.mkdir(parents=True, exist_ok=True)
SERVO = PROJ / "cad" / "purchased" / "MG90S_grabcad.SLDPRT"
HOUSING, BRIDGE, LEVER, HORN = (OUT / f"{n}.SLDPRT" for n in ("Joint_Housing", "Joint_Bridge", "Joint_Lever", "MG90S_Horn"))
B685, B623, PIN = OUT / "Bearing_685ZZ.SLDPRT", OUT / "Bearing_623ZZ.SLDPRT", OUT / "Pin_ISO2338_3x12.SLDPRT"
ASM = OUT / "Joint_MG90S.SLDASM"
LOG = OUT / "build_log.json"

S, PK, H = P["servo"], P["pocket"], P["horn"]
AX = S["shaft_x"]                       # joint axis x (z = 0)
# --- housing
TOP = S["tab_bottom_y"]                 # tabs rest on the housing top
FLOOR = S["bottom_y"] - PK["floor_gap"]  # pocket floor
BOT = FLOOR - 2.0                       # floor 2 mm
HX0, HX1, HZ = -22.0, 16.5, 9.0         # housing x range (post at -22..-17), half width
POST_X1 = -17.0
# --- 623ZZ under the floor (bearing 3x10x4), 685ZZ in the bridge (5x11x5)
B623_Y0 = BOT - 4.5                     # boss bottom; cup 4 deep from it
B623_C = B623_Y0 + 2.0
# --- lever / horn
CHEEK_T_TOP, CHEEK_T_BOT, GAP = 5.0, 4.0, 0.5
LV_TOP0 = H["plate_top_y"] - H["plate_t"]          # cheek bottom = horn plate bottom (plate keyed in a recess)
LV_TOP1 = LV_TOP0 + CHEEK_T_TOP
# v0.3: room to lift the lever + horn over the shaft top while sliding it in sideways
LIFT = S["shaft_top_y"] - H["hub_bottom_y"] + 0.1
LV_BOT1 = B623_Y0 - GAP - LIFT
PIN_L = 12.0
LV_BOT0 = LV_BOT1 - CHEEK_T_BOT
SPIG_Y1 = LV_TOP1 + GAP + 5.0                       # spigot top = 685 top
WEB_R0, WEB_T, CHEEK_HW = 25.0, 3.0, 7.0
# --- bridge
BR_Y0 = LV_TOP1 + GAP
BR_T = 6.5
B685_C = BR_Y0 + 2.5
POST_SCREW = [(-19.5, 4.5), (-19.5, -4.5)]
# link pattern (y, z) on the lever web and on the -x face of the next housing: a square, so it also
# fits a joint turned 90 deg about x (hip roll on hip pitch); clears the cable tunnel (y < -9.7)
LINK_YZ = [(y, z) for y in (-4.5, 4.5) for z in (-4.5, 4.5)]
LINK_DX = (AX + WEB_R0 + WEB_T) - HX0   # 55.5: axis-to-axis of two linked joints

log: list[dict] = []
mcp = adapter = None


async def t(name: str, args: dict | None = None, *, wrap: bool = True, why: str = "", n: int = 200) -> dict:
    payload = {"input_data": args} if (wrap and args is not None) else (args or {})
    t0 = time.perf_counter()
    try:
        r = _unwrap(await mcp.call_tool(name, payload))
    except Exception as e:  # noqa: BLE001
        r = {"status": "exception", "message": str(e)}
    st = r.get("status") if isinstance(r, dict) else "?"
    log.append({"tool": name, "status": st, "why": why, "s": round(time.perf_counter() - t0, 2)})
    LOG.write_text(json.dumps(log, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"{'OK ' if st in ('success', 'partial_success') else '!! '}{name} [{st}] {why}\n    "
          f"{json.dumps(r, ensure_ascii=False, default=str)[:n]}")
    if st not in ("success", "partial_success"):
        raise RuntimeError(f"{name} failed: {r}")
    return r if isinstance(r, dict) else {"raw": r}


def top_component(name: str):
    """Top-level component by name: GetComponentByName also matches nested ones
    (a leg's Joint_MG90S-1 was returned instead of the waist joint, 2026-10-31)."""
    for c in mc.typed(model(), "IAssemblyDoc").GetComponents(True) or []:
        c = mc.typed(c, "IComponent2")
        if c.Name2 == name:
            return c
    raise KeyError(name)


def model():
    return adapter.currentModel


def vol() -> float:
    return (mc.mass_summary(model()) or {}).get("volume_mm3", 0.0)


def box() -> tuple[list[float], list[float]]:
    b = mc.geometry_report(model(), "bounding_box")["part_bounding_box"]
    return b["min_mm"], b["max_mm"]


def find_name(r, prefix: str) -> str | None:
    s = json.dumps(r, default=str)
    import re
    m = re.findall(rf'"({prefix}\d+)"', s)
    return m[-1] if m else None


_planes: dict[float, str] = {}


async def yplane(y: float) -> str:
    """Plane parallel to Top at height y (cached per part)."""
    if y == 0:
        return "Top"
    if y not in _planes:
        r = await t("create_reference_plane", {"base_plane": "Top", "offset": abs(y), "flip": y < 0,
                                               "name": f"Y{y:+.2f}".replace(".", "_")}, why=f"plane y={y}", n=80)
        _planes[y] = r["plane"]["name"]
    return _planes[y]


async def sketch(plane: str, draw) -> str:
    r = await t("create_sketch", {"plane": plane}, why=f"sketch on {plane}", n=80)
    for kind, args in draw:
        await t(kind, args, why=kind, n=60)
    await t("exit_sketch", None, wrap=False, n=40)
    return find_name(r, "Sketch") or "Sketch"


def slot(cx, cz, x_end, hw):
    """Outline on a Top-type plane: half circle R=hw around (cx, cz) + rectangle to x_end.
    Sketch (u, v) on a Top-parallel plane = model (X, -Z); CreateArc runs counter-clockwise,
    so the arc starts at +hw when the rectangle goes to +x (arc on the -x side) and vice versa."""
    v = -cz
    a, b = ((v + hw, v - hw) if x_end > cx else (v - hw, v + hw))
    return [("add_arc", {"center_x": cx, "center_y": v, "start_x": cx, "start_y": a, "end_x": cx, "end_y": b}),
            ("add_line", {"x1": cx, "y1": b, "x2": x_end, "y2": b}),
            ("add_line", {"x1": x_end, "y1": b, "x2": x_end, "y2": a}),
            ("add_line", {"x1": x_end, "y1": a, "x2": cx, "y2": a})]


def rect(x0, y0, x1, y1):
    return [("add_rectangle", {"x1": x0, "y1": y0, "x2": x1, "y2": y1})]


def circle(x, y, r):
    return [("add_circle", {"center_x": x, "center_y": y, "radius": r})]


async def boss_up(y0: float, draw, depth: float, why: str) -> None:
    """Extrude from the plane y0 UP by depth; fixed by undo+reverse if SW went down."""
    sk = await sketch(await yplane(y0), draw)
    lo0, hi0 = box() if vol() > 0 else ([1e9] * 3, [-1e9] * 3)
    await t("create_extrusion", {"sketch_name": sk, "depth": depth}, why=why, n=80)
    lo, hi = box()
    if hi[1] < y0 + depth - 0.01 or lo[1] < min(lo0[1], y0) - 0.01:
        print(f"    went down ({lo[1]}..{hi[1]}), redo reversed")
        mc.typed(model(), "IModelDoc2").EditUndo2(1)
        await t("create_extrusion", {"sketch_name": sk, "depth": depth, "reverse_direction": True}, why=why + " (rev)", n=80)
        lo, hi = box()
        assert hi[1] >= y0 + depth - 0.01 and lo[1] >= min(lo0[1], y0) - 0.01, (why, lo, hi)


async def front_boss(draw, width: float, why: str) -> None:
    sk = await sketch("Front", draw)
    await t("create_extrusion", {"sketch_name": sk, "depth": width, "both_directions": True}, why=why, n=80)


async def cut(plane: str, draw, depth: float | None, expect_mm3: float, why: str) -> None:
    """Blind cut (or through all); checks the removed volume, retries reversed."""
    v0 = vol()
    await sketch(plane, draw)
    args = {"depth": depth or 100, "end_condition": "Blind" if depth else "ThroughAllBoth"}
    await t("create_cut_extrude", args, why=why, n=80)
    dv = v0 - vol()
    if dv < 0.6 * expect_mm3:
        print(f"    removed {dv:.1f} of ~{expect_mm3:.1f} mm3, redo reversed")
        mc.typed(model(), "IModelDoc2").EditUndo2(1)
        await t("create_cut_extrude", dict(args, reverse_direction=True), why=why + " (rev)", n=80)
        dv = v0 - vol()
    print(f"    removed {dv:.1f} mm3 (expected ~{expect_mm3:.1f})")
    assert dv > 0.6 * expect_mm3, (why, dv, expect_mm3)


async def finish(path: Path, material: str, shot: str) -> None:
    await t("set_material", {"material": material}, why=material, n=80)
    lo, hi = box()
    print(f"    {path.stem}: bbox {lo} .. {hi}, V={vol():.0f} mm3")
    await t("save_as", {"file_path": str(path), "overwrite": True}, why="save", n=80)
    if "Horn" not in path.stem:
        stl = STL / (path.stem + ".STL")
        await t("export_stl", {"file_path": str(stl), "format_type": "stl"}, why="STL for printing", n=80)
        # The STL must hold the whole part: a leftover selection once exported 10 triangles.
        import stl_pockets
        v = stl_pockets.load(str(stl)).reshape(-1, 3)
        size = [round(float(a), 2) for a in v.max(0) - v.min(0)]
        want = [round(b - a, 2) for a, b in zip(lo, hi)]
        print(f"    STL {stl.name}: {len(v) // 3} triangles, size {size} (model {want})")
        assert all(abs(a - b) < 0.05 for a, b in zip(size, want)), (stl.name, size, want)
    await t("export_image", {"file_path": str(SHOTS / shot), "format_type": "png", "view_orientation": "isometric"},
            why="screenshot", n=60)
    await t("close_model", {"save": False}, why="close", n=40)
    _planes.clear()


# ----------------------------------------------------------------------------- parts
async def part_housing() -> None:
    await t("create_part", {"name": "Joint_Housing"}, why="housing with MG90S pocket, post, 623ZZ boss")
    await front_boss(rect(HX0, BOT, HX1, TOP), 2 * HZ, "block")
    await front_boss(rect(HX0, TOP, POST_X1, BR_Y0), 2 * HZ, "post for the bridge")
    await boss_up(B623_Y0, circle(AX, 0, 7.5), BOT - B623_Y0, "boss D15 for 623ZZ under the floor")
    L, W = PK["length"], PK["width"]
    await cut(await yplane(TOP), rect(-L / 2, -W / 2, L / 2, W / 2), TOP - FLOOR, L * W * (TOP - FLOOR),
              f"pocket {L}x{W} (user-fitted, Голень.stl)")
    await cut(await yplane(B623_Y0), circle(AX, 0, 5.0), 4.0, 3.1416 * 25 * 4, "cup D10 x4 for 623ZZ (press, D nominal)")
    c = P["cable"]
    # Cable (photo 2026-10-31): leaves the case bottom at the SHAFT end (+x). Route: groove in the +x
    # end wall -> flat channel under the servo (floor gap 1.8) -> tunnel out at -x, the sector the
    # lever never sweeps (it turns about +-140 deg around +x).
    m0 = mc.mass_summary(model())
    await cut(await yplane(FLOOR), rect(L / 2, -W / 2, L / 2 + c["groove_depth"], W / 2), S["bottom_y"] + c["exit_h"] - FLOOR,
              60, "cable groove in the +x end wall (shaft side)")
    m1 = mc.mass_summary(model())
    cy = (m0["volume_mm3"] * m0["center_of_mass_mm"][1] - m1["volume_mm3"] * m1["center_of_mass_mm"][1]) / (
        m0["volume_mm3"] - m1["volume_mm3"])
    print(f"    groove removed at y~{cy:.2f} (must be above the floor {FLOOR})")
    if cy < FLOOR:  # cut went down into the floor/boss: undo, redo the other way
        mc.typed(model(), "IModelDoc2").EditUndo2(1)
        await t("create_cut_extrude", {"depth": S["bottom_y"] + c["exit_h"] - FLOOR, "end_condition": "Blind",
                                       "reverse_direction": True}, why="cable groove (rev)", n=80)
    await cut("Front", rect(HX0, FLOOR, -L / 2, FLOOR + c["h"]), None, 300,
              "cable exit tunnel (through Z) at the -x end")
    for x in (-S["hole_x"], S["hole_x"]):
        v0 = vol()
        await t("tutorial_simple_hole", {"plane": await yplane(TOP), "center_x": x, "center_y": 0,
                                        "diameter": P["screws"]["tab_pilot_d"], "depth": 8}, why=f"tab screw pilot x={x}", n=60)
        assert v0 - vol() > 5, "tab pilot not cut"
    for x, z in POST_SCREW:
        await t("tutorial_simple_hole", {"plane": await yplane(BR_Y0), "center_x": x, "center_y": -z,
                                        "diameter": 1.6, "depth": 10}, why=f"bridge screw pilot ({x},{z})", n=60)
    # v0.2 (2026-10-31): ribs + lightening per ТЗ (walls 2-2.5, ribs)
    g = [(-17.0, TOP), (-11.0, TOP), (-17.0, TOP + 9.85)]
    await front_boss([("add_line", {"x1": a[0], "y1": a[1], "x2": b[0], "y2": b[1]}) for a, b in zip(g, g[1:] + g[:1])],
                     2 * HZ, "post gusset (bridge load -> housing)")
    await cut(await yplane(TOP + 9.85), rect(-17.0, -6.5, -10.9, 6.5), 9.85, 300,
              "split the gusset into 2 side ribs 2.5 (clear the servo tabs)")
    r = await t("create_reference_plane", {"base_plane": "Right", "offset": -HX0, "flip": True, "name": "X_LINK"},
                why="-x face plane (link to the previous lever)", n=60)
    draw = [d for y, z in LINK_YZ for d in circle(z, y, 0.8)]  # Right-type sketch: (u, v) = (+-Z, Y), pattern symmetric in Z
    await cut(r["plane"]["name"], draw, 6.0, 4 * 3.1416 * 0.64 * 6 * 0.9, "4 x pilot D1.6 x6 for M2 link screws")
    await finish(HOUSING, "ABS", "housing_iso.png")


async def part_bridge() -> None:
    await t("create_part", {"name": "Joint_Bridge"}, why="bridge: cup for 685ZZ coaxial with the servo shaft")
    await boss_up(BR_Y0, slot(AX, 0, HX0, HZ), BR_T, "plate R9 around the axis to the post")
    await cut(await yplane(BR_Y0), circle(AX, 0, 5.5), 5.0, 3.1416 * 30.25 * 5, "cup D11 x5 for 685ZZ (press)")
    await cut(await yplane(BR_Y0 + 5.0), circle(AX, 0, 4.0), 1.5, 3.1416 * 16 * 1.5, "shoulder hole D8 (inner ring free)")
    r = await t("create_hole_wizard", {"kind": "clearance", "size": "M2",
                                      "points": [[x, BR_Y0 + BR_T, z] for x, z in POST_SCREW]},
                why="Hole Wizard ISO M2 clearance to the post", n=300)
    await cut(await yplane(BR_Y0 + BR_T), rect(-16.5, -6.5, -1.5, 6.5), 4.0, 700,
              "top pocket: channel section, side ribs 2.5, floor 2.5")
    await finish(BRIDGE, "ABS", "bridge_iso.png")


async def part_lever() -> None:
    await t("create_part", {"name": "Joint_Lever"}, why="U-lever: horn cheek + spigot D5, pin cheek, web")
    x_end = AX + WEB_R0 + WEB_T
    await boss_up(LV_TOP0, slot(AX, 0, x_end, CHEEK_HW), CHEEK_T_TOP, "top cheek (on the horn)")
    await boss_up(LV_BOT0, slot(AX, 0, x_end, CHEEK_HW), CHEEK_T_BOT, "bottom cheek (pin)")
    await front_boss(rect(AX + WEB_R0, LV_BOT0, x_end, LV_TOP1), 2 * CHEEK_HW, "web R25..28 (clears housing corners)")
    await boss_up(LV_TOP1, circle(AX, 0, 2.5), SPIG_Y1 - LV_TOP1, "spigot D5 into 685ZZ")
    hw = H["arm_w"] / 2 + 0.2
    await cut(await yplane(LV_TOP0), circle(AX, 0, H["hub_d"] / 2 + 0.2), H["plate_t"], 40, "horn hub recess D7.4")
    await cut(await yplane(LV_TOP0), slot(AX, 0, AX + H["arm_r"] + 0.2, hw), H["plate_t"], 1.6 * hw * 2 * H["arm_r"] * 0.6,
              "horn arm recess +0.2 (torque by form fit)")
    await cut(await yplane(H["plate_top_y"]), circle(AX, 0, H["center_screw_head_d"] / 2 + 0.2),
              H["center_screw_head_h"], 15, "relief for the horn centre screw head")
    await cut(await yplane(LV_BOT0), circle(AX, 0, 1.5), CHEEK_T_BOT, 3.1416 * 2.25 * 4 * 0.9, "pin hole D3 (press)")
    for r in H["screw_r"]:  # one call per hole: the 2nd ray of a blind pair found no face (2026-10-31)
        await t("create_hole_wizard", {"kind": "clearance", "size": "M2", "depth": CHEEK_T_TOP,
                                       "points": [[AX + r, LV_TOP1, 0]]},
                why=f"Hole Wizard M2 r={r}: screw into the horn arm", n=200)
    await t("create_hole_wizard", {"kind": "clearance", "size": "M2", "direction": [-1, 0, 0],
                                   "points": [[x_end, y, z] for y, z in LINK_YZ]},
            why="Hole Wizard M2 x4: next link on the web", n=200)
    await t("add_fillet", {"radius": 2.0, "edge_names": [f"{(AX + WEB_R0) / 1000},{LV_TOP0 / 1000},0",
                                                         f"{(AX + WEB_R0) / 1000},{LV_BOT1 / 1000},0"]},
            why="inner web/cheek fillets R2 (ribs against bending)", n=80)
    await cut(await yplane(LV_BOT0), rect(12.0, -3.5, 27.0, 3.5), CHEEK_T_BOT, 380, "window in the bottom cheek")
    await cut(await yplane(LV_TOP1), rect(21.0, -4.5, 29.0, 4.5), 2.5, 160, "pocket in the top cheek")
    await finish(LEVER, "ABS", "lever_iso.png")


async def part_horn() -> None:
    await t("create_part", {"name": "MG90S_Horn"}, why="horn stand-in (verify): hub + single arm")
    hw = H["arm_w"] / 2
    await boss_up(LV_TOP0, slot(AX, 0, AX + H["arm_r"], hw), H["plate_t"], "arm plate")
    await boss_up(H["hub_bottom_y"], circle(AX, 0, H["hub_d"] / 2), LV_TOP0 - H["hub_bottom_y"], "hub")
    await cut(await yplane(H["hub_bottom_y"]), circle(AX, 0, S["shaft_d"] / 2), S["shaft_top_y"] - H["hub_bottom_y"],
              3.1416 * 5.9 * 3.5 * 0.9, "spline bore on the shaft")
    await cut(await yplane(S["shaft_top_y"]), circle(AX, 0, 1.1), H["plate_top_y"] - S["shaft_top_y"], 4, "centre screw hole")
    for r in H["screw_r"]:
        await cut(await yplane(LV_TOP0), circle(AX + r, 0, H["screw_hole_d"] / 2), H["plate_t"], 1.0, f"arm hole r={r}")
    await finish(HORN, "Nylon 6/10", "horn_iso.png")


async def toolbox() -> None:
    for std, size, length, path in (("ISO 15", "685", None, B685), ("ISO 15", "623", None, B623),
                                    ("ISO 2338", "3", PIN_L, PIN)):
        args = {"standard": std, "size": size, "save_path": str(path)}
        if length:
            args["length"] = length
        await t("create_toolbox_part", args, why=f"Toolbox {std} {size}", n=200)


# ----------------------------------------------------------------------------- assembly
async def assembly() -> None:
    await t("create_assembly", {"name": "Joint_MG90S"}, why="joint assembly")
    for p in (SERVO, HOUSING, BRIDGE, HORN, LEVER):
        await t("insert_component", {"file_path": str(p)}, why=f"{p.stem} at origin (frame = servo frame)", n=80)
    await t("add_mate", {"entity1": "MG90S_grabcad-1", "mate_type": "fix"}, why="servo = reference")
    for comp in ("Joint_Housing-1", "Joint_Bridge-1", "MG90S_Horn-1", "Joint_Lever-1"):
        for pl in ("Right", "Top", "Front"):
            await t("add_mate", {"entity1": f"MG90S_grabcad-1/{pl} Plane", "entity2": f"{comp}/{pl} Plane",
                                 "mate_type": "coincident", "align": "aligned"}, why=f"{comp} {pl}", n=80)
    planes: dict[tuple[str, float], str] = {}

    async def plane(axis: str, v: float) -> str:
        if v == 0:
            return f"MG90S_grabcad-1/{dict(x='Right', y='Top', z='Front')[axis]} Plane"
        if (axis, v) not in planes:
            base = dict(x="Right", y="Top", z="Front")[axis]
            r = await t("create_reference_plane", {"base_plane": base, "offset": abs(v), "flip": v < 0,
                                                   "name": f"ASM_{axis.upper()}{v:+.2f}".replace(".", "_")},
                        why=f"asm plane {axis}={v}", n=80)
            planes[(axis, v)] = r["plane"]["name"]
        return planes[(axis, v)]

    # Toolbox parts: axis = part X. Bearings centred on their origin, pin origin at its end.
    for comp, y, want in (("Bearing_685ZZ-1", B685_C, (BR_Y0, BR_Y0 + 5)),
                          ("Bearing_623ZZ-1", B623_C, (B623_Y0, B623_Y0 + 4)),
                          ("Pin_ISO2338_3x12-1", LV_BOT0, (LV_BOT0, LV_BOT0 + PIN_L))):
        path = {"Bearing_685ZZ-1": B685, "Bearing_623ZZ-1": B623}.get(comp, PIN)
        await t("insert_component", {"file_path": str(path), "x": AX, "y": y, "z": 0}, why=comp, n=80)
        axial = "aligned" if comp.startswith("Pin") else "anti_aligned"  # pin: origin at its end, must point up
        for ent2, ref, al in ((f"{comp}/Right Plane", await plane("y", y), axial),
                              (f"{comp}/Top Plane", await plane("x", AX), "closest"),
                              (f"{comp}/Front Plane", await plane("z", 0), "closest")):
            await t("add_mate", {"entity1": ref, "entity2": ent2, "mate_type": "coincident", "align": al,
                                 "keep_position": False}, why=f"{comp}: {ent2.split('/')[1]}", n=80)
        c = top_component(comp)
        b = [round(v * 1000, 2) for v in c.GetBox(False, False)]
        print(f"    {comp} box {b}")
        if abs(b[1] - want[0]) > 0.3 or abs(b[4] - want[1]) > 0.3:
            # pin pointing down: flip the axial mate alignment
            raise AssertionError((comp, b, want))
    await t("save_as", {"file_path": str(ASM), "overwrite": True}, why="save assembly", n=80)
    for v in ("isometric", "front", "top", "right"):
        await t("export_image", {"file_path": str(SHOTS / f"asm_{v}.png"), "format_type": "png",
                                 "view_orientation": v}, why=f"assembly {v}", n=40)
    r = await t("check_interference", {}, why="interference (press fits = coincident, expect none)", n=1500)
    await t("calculate_mass_properties", {"units": "metric"}, why="unit mass", n=300)
    await t("close_model", {"save": False}, why="close", n=40)


async def main(phases: list[str]) -> None:
    global mcp, adapter
    t0 = time.perf_counter()
    async with sw_session() as (mcp, adapter):
        print("open docs at start:", open_docs(adapter))
        parts = {"housing": part_housing, "bridge": part_bridge, "lever": part_lever, "horn": part_horn, "toolbox": toolbox}
        todo = list(parts.values()) if "parts" in phases else [parts[k] for k in phases if k in parts]
        if todo:
            for f in todo:
                ts = time.perf_counter()
                await f()
                print(f"=== {f.__name__}: {time.perf_counter() - ts:.0f} s")
        if "asm" in phases:
            await assembly()
    print(f"total {time.perf_counter() - t0:.0f} s")


if __name__ == "__main__":
    asyncio.run(main(sys.argv[1:] or ["parts", "asm"]))
