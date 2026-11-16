"""Spec for the SOLIDWORKS Motion physics model (consumed by tools/physics/build_physics.exe).

Flat assembly: every printed part / servo / horn inserted at its world pose (from
docs/viewer/robot_3d.json), parts of one kinematic link locked together, one revolute per
servo: Concentric(lever spigot D5, bridge cup D11) + Coincident(lever Top Plane, housing Top
Plane). Links/axes come from build_anim.build_links (servo local +Y = joint axis).
Self-check: the face pick points must lie on the servo axis (spigot r=2.5, cup r=5.5).

Usage: python gen_phys_spec.py <out.json>
"""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import build_anim as BA  # noqa: E402

SERVO_PHYS = str(BA.PROJ / "cad" / "physics" / "MG90S_phys.SLDPRT")  # copy of the GrabCAD servo, mass override
DROP = {"Bearing_685ZZ", "Bearing_623ZZ", "Pin_ISO2338_3x12"}   # ~1-2 g each, no effect on dynamics
if BA.V2:   # v2: lever spigot D6 (y 22.8..26.5) in the bridge bushing hole D6.2 (y 23.1..27.1)
    SPIGOT_PT, CUP_PT, R_SPIG, R_CUP = [5.5 + 3.0, 22.95, 0.0], [5.5 + 3.1, 26.8, 0.0], 3.0, 3.1   # exposed bands: spigot between cheek and bridge, hole above the spigot top
    LEVER, HOUSING, BRIDGE = "J2_LeverTop", "J2_Housing", "J2_Bridge"   # top half carries the spigot
    OUT_DEF = BA.PROJ / "cad" / "v2" / "physics" / "phys_spec.json"
else:
    SPIGOT_PT = [5.5 + 2.5, 27.0, 0.0]      # lever spigot D5, y 24.3..29.8 (build_joint LV_TOP1..SPIG_Y1)
    CUP_PT = [5.5 + 5.5, 27.3, 0.0]         # bridge cup D11, y 24.8..29.8 (BR_Y0..BR_Y0+5)
    R_SPIG, R_CUP = 2.5, 5.5
    LEVER, HOUSING, BRIDGE = "Joint_Lever", "Joint_Housing", "Joint_Bridge"
    OUT_DEF = BA.PROJ / "cad" / "physics" / "phys_spec.json"


def wpt(m, p):
    R = np.array(m[:9]).reshape(3, 3); T = np.array(m[9:12]) * 1000.0
    return np.array(p) @ R + T


def dist_to_axis(p, piv, ax):
    v = np.array(p) - np.array(piv); a = np.array(ax) / np.linalg.norm(ax)
    return float(np.linalg.norm(v - (v @ a) * a))


def main(out):
    d = json.loads(BA.SRC.read_text(encoding="utf-8"))
    links, classify, order, units = BA.build_links(d)
    parts = []
    for it in d["inst"]:
        stem = BA.stem_of(it["name"])
        if stem in DROP:
            continue
        path = SERVO_PHYS if stem == "MG90S_grabcad" else it["p"]   # datasheet mass 13.4 g (TowerPro)
        parts.append({"name": it["name"], "stem": stem, "path": path,
                      "m": it["m"][:12] + [1.0, 0.0, 0.0, 0.0], "link": classify(it["name"])})
    idx = {p["name"]: i for i, p in enumerate(parts)}
    # anchor of each link: its largest/most central part -> prefer housing/lever/pelvis/foot/fist
    pref = ["Pelvis", "Torso_Mount", LEVER, HOUSING, "Leg_Foot", "Fist_TPU"]
    rank = lambda i: pref.index(parts[i]["stem"]) if parts[i]["stem"] in pref else 99  # noqa: E731
    anchors = {}
    for i, p in enumerate(parts):
        if p["link"] not in anchors or rank(i) < rank(anchors[p["link"]]):
            anchors[p["link"]] = i
    joints = []
    bad = 0
    for limb, us in order.items():
        for pos, u in enumerate(us):
            role = [r for r, L in links.items() if L.get("pivot") and np.allclose(L["pivot"], wpt(units[u], BA.SHAFT), atol=0.05)][0]
            lever = idx[f"{u}/{LEVER}-1"]; housing = idx[f"{u}/{HOUSING}-1"]; bridge = idx[f"{u}/{BRIDGE}-1"]
            ps = wpt(parts[lever]["m"], SPIGOT_PT); pc = wpt(parts[bridge]["m"], CUP_PT)
            L = links[role]
            rs, rc = dist_to_axis(ps, L["pivot"], L["axis"]), dist_to_axis(pc, L["pivot"], L["axis"])
            ok = abs(rs - R_SPIG) < 0.05 and abs(rc - R_CUP) < 0.05
            bad += not ok
            print(f"  {role:<20} spigot r={rs:5.2f} cup r={rc:5.2f} {'ok' if ok else '!! OFF AXIS'}")
            joints.append({"role": role, "kind": L["kind"], "side": L["side"],
                           "rotor": parts[lever]["link"], "stator": parts[housing]["link"],
                           "lever": lever, "housing": housing, "bridge": bridge,
                           "p_spigot": (ps / 1000).tolist(), "p_cup": (pc / 1000).tolist(),
                           "axis": L["axis"], "pivot_m": (np.array(L["pivot"]) / 1000).tolist()})
    assert bad == 0, f"{bad} joints with face points off the servo axis"
    spec = {"parts": parts, "anchors": anchors, "joints": joints, "fixed_link": "base"}
    Path(out).write_text(json.dumps(spec, indent=1), encoding="utf-8")
    from collections import Counter
    print(f"parts={len(parts)} links={len(anchors)} joints={len(joints)} -> {out}")
    print("parts per link:", dict(Counter(p['link'] for p in parts)))
    print("anchors:", {L: parts[i]['stem'] for L, i in anchors.items()})


if __name__ == "__main__":
    OUT_DEF.parent.mkdir(parents=True, exist_ok=True)
    main(sys.argv[1] if len(sys.argv) > 1 else str(OUT_DEF))
