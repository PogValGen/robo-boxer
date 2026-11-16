"""Video of a servo_sim.py (v2) SOLIDWORKS Motion result: link poses from cad/v2/physics/servo_<label>_result.json
(10 ms), part placement from phys_spec_up.json (same upright SW frame: x side, y up, z forward, metres),
meshes from docs/viewer/robot_v2_3d.json. Overlay: time, pelvis travel, hip-roll load (% stall, median 25 ms).

Usage (venv solidworks-mcp, ROBO_V=2 not needed):
    python render_servo.py <label> [out.mp4] [fps] [--follow]
    label = walk2_6v_back, dyn_6v_d31 ...      -> docs/anim/<date>_<label>.mp4
2026-11-12: written for the dynamic gait video (render_phys.py is the v1 / build_physics version).
"""
from __future__ import annotations
import base64, gzip, json, os, shutil, subprocess, sys, time
from datetime import date
from multiprocessing import Pool
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
PROJ = Path(__file__).resolve().parents[1]
PHYS = PROJ / "cad" / "v2" / "physics"
G = {}


def R(a):
    return np.array(a[:9]).reshape(3, 3)


def load(label):
    from pack_viewer_data import stl_tris, proxy
    from render_phys import cluster, color
    import matplotlib
    res = json.loads((PHYS / f"servo_{label}_result.json").read_text(encoding="utf-8"))
    spec = json.loads((PHYS / "phys_spec_up.json").read_text(encoding="utf-8"))
    view = json.loads((PROJ / "docs" / "viewer" / "robot_v2_3d.json").read_text(encoding="utf-8"))
    path_of = {it["name"]: it["p"] for it in view["inst"]}
    cnt, inst = {}, []
    for p in spec["parts"]:                                   # SW instance names = stem-N in insertion order
        cnt[p["stem"]] = cnt.get(p["stem"], 0) + 1; inst.append(f"{p['stem']}-{cnt[p['stem']]}")
    anchor = {L: inst[i] for L, i in spec["anchors"].items()}
    cache, parts = {}, []
    for p in spec["parts"]:
        src = path_of[p["name"]]
        if src not in cache:
            m = view["meshes"][src]
            tris = stl_tris(gzip.decompress(base64.b64decode(m["gz"]))) + np.array(m["off"])
            px = proxy(m["stem"], tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0))
            cache[src] = (px if px is not None else cluster(tris, 1.6), color(m["stem"]))
        (v, f), col = cache[src]
        w0 = v @ R(p["m"]) + np.array(p["m"][9:12]) * 1000.0
        parts.append((w0, f, np.array(matplotlib.colors.to_rgb(col)), anchor[p["link"]]))
    poses = res["poses"]; keys = sorted(poses, key=float)
    tq = {}
    for r in ("legA_hiproll", "legB_hiproll"):
        rows = np.array(res["servos"][f"S_{r}"]["rows"]); stall = res["servos"][f"S_{r}"].get("stall", 0.2157)
        tq[r] = (rows[:, 0], np.abs(rows[:, 4]) / stall * 100.0)
    return dict(poses=poses, keys=keys, parts=parts, tq=tq, base=anchor["base"])


def tris_at(k):
    poses, A0 = G["poses"], G["poses"][G["keys"][0]]
    light = np.array([0.45, 0.75, -0.5]); light /= np.linalg.norm(light)
    allv, allc = [], []
    for w0, f, col, comp in G["parts"]:
        a, a0 = poses[k][comp], A0[comp]
        M = np.linalg.inv(R(a0)) @ R(a)
        w = (w0 - np.array(a0[9:12]) * 1000) @ M + np.array(a[9:12]) * 1000
        lie = os.environ.get("RS_LIE", "")
        if lie == "chest":                                    # gravity SW +Z: up = -z, head along +X
            P = np.c_[w[:, 1], w[:, 0], -w[:, 2]]
        elif lie == "back":                                   # gravity SW -Z: up = +z
            P = np.c_[-w[:, 1], w[:, 0], w[:, 2]]
        else:
            P = np.c_[w[:, 2], w[:, 0], w[:, 1]]              # plot: X forward, Y side, Z up
        T = P[f]
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); n /= np.linalg.norm(n, axis=1)[:, None] + 1e-12
        allv.append(T); allc.append(np.clip(col[None, :] * (0.35 + 0.65 * np.abs(n @ light))[:, None], 0, 1))
    return np.concatenate(allv), np.concatenate(allc)


def init(label, lims, follow):
    G.update(load(label)); G["lims"] = lims; G["follow"] = follow; G["label"] = label


def render(job):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    i, k, t, out = job
    T, C = tris_at(k)
    c, r, floor = G["lims"]
    c = c.copy()
    b = G["poses"][k][G["base"]]; b0 = G["poses"][G["keys"][0]][G["base"]]
    travel = (b[11] - b0[11]) * 1000, (b[9] - b0[9]) * 1000
    if G["follow"]:
        c[0] += travel[0]
    bg = (0.933, 0.945, 0.961)
    fig = plt.figure(figsize=(7.2, 8.0), dpi=110); fig.patch.set_facecolor(bg)
    ax = fig.add_axes([0, 0.1, 1, 0.84], projection="3d")
    g = np.arange(-200, 201, 20.0)                            # floor grid, 20 mm
    for x in g:
        ax.plot([c[0] + x, c[0] + x], [c[1] - 200, c[1] + 200], [floor, floor], color=(0.7, 0.72, 0.76), lw=0.4)
        ax.plot([c[0] - 200, c[0] + 200], [c[1] + x, c[1] + x], [floor, floor], color=(0.7, 0.72, 0.76), lw=0.4)
    ax.add_collection3d(Poly3DCollection(T, facecolors=C, edgecolors=(0.15, 0.15, 0.15, 0.22), linewidths=0.12))
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1), zoom=1.35); ax.view_init(elev=10, azim=-70 if os.environ.get('RS_LIE') else -35); ax.set_axis_off(); ax.set_facecolor(bg)
    fig.text(0.04, 0.955, f"{G['label']}   t = {t:5.2f} s", fontsize=13, family="DejaVu Sans Mono")
    fig.text(0.04, 0.925, f"таз: вперёд {travel[0]:+6.1f} мм, вбок {travel[1]:+6.1f} мм", fontsize=10.5, family="DejaVu Sans")
    for j, (r_, name) in enumerate((("legA_hiproll", "крен бедра A"), ("legB_hiproll", "крен бедра B"))):
        ts, pc = G["tq"][r_]; lo = np.searchsorted(ts, t - 0.0125); hi = max(lo + 1, np.searchsorted(ts, t + 0.0125))
        v = float(np.median(pc[lo:hi])) if lo < len(pc) else 0.0
        y = 0.055 - j * 0.035
        fig.text(0.04, y, f"{name} {v:4.0f}%", fontsize=10, family="DejaVu Sans")
        ax2 = fig.add_axes([0.33, y - 0.004, 0.6, 0.022]); ax2.set_xlim(0, 150); ax2.set_ylim(0, 1); ax2.axis("off")
        ax2.barh(0.5, 150, height=1, color=(0.85, 0.86, 0.88))
        ax2.barh(0.5, min(v, 150), height=1, color="#4f9d69" if v < 80 else "#e0a030" if v < 100 else "#d94f2b")
        ax2.axvline(100, color="k", lw=0.8)
    fig.savefig(out / f"f_{i:04d}.png", facecolor=bg); plt.close(fig)
    return i


def ffmpeg():
    for p in (shutil.which("ffmpeg"), r"C:\ffmpeg\bin\ffmpeg.exe", r"C:\camstream\ffmpeg\bin\ffmpeg.exe",
              r"D:\AI prodject\camstream\ffmpeg\bin\ffmpeg.exe"):
        if p and Path(p).exists():
            return p
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        raise SystemExit("ffmpeg not found")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    follow = "--follow" in sys.argv
    for a_ in sys.argv[1:]:                                    # --lie=chest|back: show a lying run with the real floor down
        if a_.startswith("--lie="):
            os.environ["RS_LIE"] = a_.split("=", 1)[1]
    label = args[0]
    out_mp4 = Path(args[1]) if len(args) > 1 else PROJ / "docs" / "anim" / f"{date.today()}_{label}.mp4"
    fps = float(args[2]) if len(args) > 2 else 25.0
    t_start = time.time()
    G.update(load(label))
    keys = G["keys"]; kt = np.array([float(k) for k in keys])
    times = [i / fps for i in range(int(kt[-1] * fps) + 1)]
    ks = [keys[int(np.argmin(np.abs(kt - t)))] for t in times]
    lo, hi = np.full(3, 1e9), np.full(3, -1e9)
    for k in ks[:: max(1, len(ks) // 12)] + [ks[-1]]:
        T, _ = tris_at(k); lo = np.minimum(lo, T.reshape(-1, 3).min(0)); hi = np.maximum(hi, T.reshape(-1, 3).max(0))
    floor = lo[2]
    if follow:                                                 # window = robot at t0, camera moves with the pelvis
        T, _ = tris_at(ks[0]); lo, hi = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0)
    c = (lo + hi) / 2; r = (hi - lo).max() / 2 * 1.05
    frames = PROJ / "cad" / "v2" / "physics" / "frames" / label
    frames.mkdir(parents=True, exist_ok=True)
    for old in frames.glob("f_*.png"):
        old.unlink()
    jobs = [(i, k, t, frames) for i, (k, t) in enumerate(zip(ks, times))]
    n = len(jobs); done = 0
    with Pool(min(6, os.cpu_count() or 2), initializer=init, initargs=(label, (c, r, floor), follow)) as pool:
        for _ in pool.imap_unordered(render, jobs, chunksize=4):
            done += 1
            if done % 25 == 0 or done == n:
                bar = "█" * int(10 * done / n) + "░" * (10 - int(10 * done / n))
                print(f"[{bar}] {100 * done // n}% — кадр {done}/{n}, {time.time() - t_start:.0f} с", flush=True)
    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    cmd = [ffmpeg(), "-y", "-loglevel", "error", "-framerate", str(fps), "-i", str(frames / "f_%04d.png"),
           "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23",
           "-movflags", "+faststart", str(out_mp4)]
    subprocess.run(cmd, check=True)
    print(f"video {out_mp4} ({out_mp4.stat().st_size // 1024} KB, {n} frames, {time.time() - t_start:.0f} s)")


if __name__ == "__main__":
    main()
