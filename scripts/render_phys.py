"""Render the SOLIDWORKS Motion result: link trajectories from the solved study
(<motion>_result.json, 10 ms), part geometry from the SolidWorks model (docs/viewer/robot_3d.json),
own camera (robot upright, 3/4 view, same scale for all frames). Writes f_XXXX.png for make_phys_video.

Why not SolidWorks screenshots: SaveAs(png) of an assembly gave a stale picture for every time step,
SaveBMP followed the motion but ignored zoom/orientation calls (robot lying, cut) - 2026-11-01.
Usage: python render_phys.py <motion_x.json> <out_dir> [fps]
"""
from __future__ import annotations
import base64, gzip, json, sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

sys.path.insert(0, str(Path(__file__).parent))
from pack_viewer_data import stl_tris, proxy  # noqa: E402

# same palette as make_viewer.py (not imported: it pulls in the whole SolidWorks stack)
COLORS = [("MG90S", "#2b2f36"), ("Fist", "#d94f2b"), ("Yoke", "#5b6f8f"), ("Horn", "#f2f2f2"), ("Housing", "#7f8fa6"),
          ("Bridge", "#b8c4d6"), ("Lever", "#ef8a2f"), ("Foot", "#4f9d69"), ("Pelvis", "#5b6f8f"), ("Torso", "#c75b5b")]


def color(stem):
    return next((c for k, c in COLORS if k.lower() in stem.lower()), "#aaaaaa")

PROJ = Path(__file__).resolve().parents[1]


def cluster(tris, cell):
    """vertex clustering decimation (robust on STL soups)."""
    pts = tris.reshape(-1, 3); key = np.round(pts / cell).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    v = np.zeros((len(uniq), 3)); n = np.zeros(len(uniq))
    np.add.at(v, inv.ravel(), pts); np.add.at(n, inv.ravel(), 1); v /= n[:, None]
    f = inv.reshape(-1, 3); f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
    return v, f


def R(a): return np.array(a[:9]).reshape(3, 3)


def main(motion, out_dir, fps=25.0):
    mp = Path(motion)
    res = json.loads(mp.with_name(mp.stem + "_result.json").read_text(encoding="utf-8"))
    spec = json.loads((mp.parent / "phys_spec.json").read_text(encoding="utf-8"))
    view = json.loads((PROJ / "docs" / "viewer" / "robot_3d.json").read_text(encoding="utf-8"))
    path_of = {it["name"]: it["p"] for it in view["inst"]}
    poses = res["poses"]; keys = sorted(poses, key=float); kt = np.array([float(k) for k in keys])
    # geometry per part in its own frame (mm), coarse
    cache, parts = {}, []
    for p in spec["parts"]:
        src = path_of[p["name"]]
        if src not in cache:
            m = view["meshes"][src]
            tris = stl_tris(gzip.decompress(base64.b64decode(m["gz"]))) + np.array(m["off"])
            px = proxy(m["stem"], tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0))
            cache[src] = (px if px is not None else cluster(tris, 1.6), color(m["stem"]))
        (v, f), col = cache[src]
        w0 = v @ R(p["m"]) + np.array(p["m"][9:12]) * 1000.0            # world at t0, mm
        parts.append((w0, f, np.array(matplotlib.colors.to_rgb(col)), p["link"]))
    A0 = {L: poses[keys[0]][L] for L in spec["anchors"]}
    light = np.array([0.35, -0.55, 0.75]); light /= np.linalg.norm(light)
    to_plot = lambda P: np.c_[P[:, 2], P[:, 1], -P[:, 0]]                 # robot (x down, z fwd) -> plot (z up)

    def frame_tris(k):
        allv, allc = [], []
        for w0, f, col, L in parts:
            a, a0 = poses[k][L], A0[L]
            M = np.linalg.inv(R(a0)) @ R(a)
            w = (w0 - np.array(a0[9:12]) * 1000) @ M + np.array(a[9:12]) * 1000
            P = to_plot(w); T = P[f]
            n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); n /= np.linalg.norm(n, axis=1)[:, None] + 1e-12
            shade = 0.35 + 0.65 * np.abs(n @ light)
            allv.append(T); allc.append(np.clip(col[None, :] * shade[:, None], 0, 1))
        return np.concatenate(allv), np.concatenate(allc)

    n = int(round(float(keys[-1]) * fps))
    times = [i / fps for i in range(n + 1)]
    ks = [keys[int(np.argmin(np.abs(kt - t)))] for t in times]
    # common limits over all frames
    lo, hi = np.full(3, 1e9), np.full(3, -1e9)
    for k in ks[:: max(1, len(ks) // 12)] + [ks[-1]]:
        T, _ = frame_tris(k); lo = np.minimum(lo, T.reshape(-1, 3).min(0)); hi = np.maximum(hi, T.reshape(-1, 3).max(0))
    c = (lo + hi) / 2; r = (hi - lo).max() / 2 * 1.02
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    for old in out.glob("*.png"): old.unlink()
    for old in out.glob("*.bmp"): old.unlink()
    for i, k in enumerate(ks):
        T, C = frame_tris(k)
        fig = plt.figure(figsize=(6.6, 9.0), dpi=110); ax = fig.add_subplot(111, projection="3d")
        ax.add_collection3d(Poly3DCollection(T, facecolors=C, edgecolors=(0.15, 0.15, 0.15, 0.25), linewidths=0.15))
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        bg = (0.933, 0.945, 0.961)
        ax.set_box_aspect((1, 1, 1), zoom=1.45); ax.view_init(elev=14, azim=-38); ax.set_axis_off()
        ax.set_facecolor(bg); fig.patch.set_facecolor(bg)
        fig.subplots_adjust(0, 0, 1, 1)
        fig.savefig(out / f"f_{i:04d}.png", facecolor=(0.933, 0.945, 0.961)); plt.close(fig)
    print(f"{len(ks)} frames -> {out}  (tris/frame {len(T)})")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 25.0)
