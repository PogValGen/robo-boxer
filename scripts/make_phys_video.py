"""Stitch SolidWorks-rendered Motion frames into an MP4 with a torque overlay.

Frames: tools/physics/build_physics.exe render ... (f_0000.png ...), torques: <motion>_torque.csv
(scripts/check_phys.py, inverse dynamics), limit: TowerPro MG90S stall 0.177 N*m @4.8 V.
Usage: python make_phys_video.py <frames_dir> <motion_x.json> <out.mp4> [fps]
"""
from __future__ import annotations
import csv, json, subprocess, sys, shutil
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

STALL = 1.8 * 0.0980665
FONT = "C:/Windows/Fonts/arial.ttf"
SHOW = [("torso", "Пояс (скручивание)"), ("armB_shoulder", "Правое плечо (удар)"),
        ("armA_shoulder", "Левое плечо (гард)"), ("legB_hippitch", "Правое бедро")]
PHASES = [(0.8, "подъём рук в гард"), (1.2, "гард"), (1.45, "УДАР правой"), (1.9, "возврат"), (99, "гард")]


def prepare(frames):
    """Frames -> canvas with room for the torque panel. PNG frames from render_phys.py are used as is;
    (legacy) SolidWorks BMP frames are rotated upright and trimmed."""
    src = sorted(Path(frames).glob("f_*.bmp"))
    if not src:
        out = []
        for f in sorted(Path(frames).glob("f_*.png")):
            c = Image.open(f).convert("RGB"); s = 900 / c.height
            c = c.resize((int(c.width * s), 900), Image.LANCZOS)
            canvas = Image.new("RGB", (c.width + 620, 960), (238, 241, 245)); canvas.paste(c, (620, 60))
            p = f.with_name("c_" + f.name); canvas.save(p); out.append(p)
        return out
    def clean(im):   # window border is pure black: paint it as background so it is not cropped in
        px = im.load(); w, h = im.size
        for x in range(w):
            for y in (0, 1, 2, h - 3, h - 2, h - 1):
                if sum(px[x, y]) < 30: px[x, y] = (238, 241, 245)
        for y in range(h):
            for x in list(range(0, 6)) + list(range(w - 6, w)):
                if sum(px[x, y]) < 30: px[x, y] = (238, 241, 245)
        return im
    ims = [clean(Image.open(f).convert("RGB")).rotate(-90, expand=True, fillcolor=(238, 241, 245)) for f in src]
    # NO brightness bbox: light-grey printed parts match the background and were cut off (2026-11-01).
    # The SW view is already fitted (closed loop in build_physics render); trim only solid black bands.
    import numpy as np
    a = np.asarray(ims[0].convert("L"))
    cols = np.where((a < 15).mean(axis=0) < 0.9)[0]; rows = np.where((a < 15).mean(axis=1) < 0.9)[0]
    box = (int(cols[0]), int(rows[0]), int(cols[-1]) + 1, int(rows[-1]) + 1)
    out = []
    for f, im in zip(src, ims):
        c = im.crop(box); s = 900 / c.height
        c = c.resize((int(c.width * s), 900), Image.LANCZOS)
        canvas = Image.new("RGB", (max(c.width + 620, 1280), 900 + 60), (238, 241, 245))
        canvas.paste(c, (620, 60))
        p = f.with_suffix(".png"); canvas.save(p); out.append(p)
    return out


def main(frames, motion, out, fps=25.0):
    fr = prepare(frames)
    mp = Path(motion)
    rows = list(csv.DictReader(open(mp.with_name(mp.stem + "_torque.csv"), encoding="utf-8")))
    T = [float(r["t"]) for r in rows]
    f_big, f_mid, f_sm = (ImageFont.truetype(FONT, s) for s in (30, 22, 18))
    tmp = Path(frames) / "ov"; tmp.mkdir(exist_ok=True)
    peak = {k: max(abs(float(r[k])) for r in rows) for k, _ in SHOW}
    for i, f in enumerate(fr):
        t = i / fps
        k = min(range(len(T)), key=lambda j: abs(T[j] - t))
        im = Image.open(f).convert("RGB")
        W, H = im.size
        d = ImageDraw.Draw(im)
        d.rectangle([0, 0, W, 54], fill=(255, 255, 255))
        d.text((16, 12), "SOLIDWORKS Motion: удар правой, стоя (таз закреплён), гравитация", font=f_mid, fill=(20, 20, 20))
        ph = next(n for tt, n in PHASES if t < tt)
        d.text((W - 360, 12), f"t = {t:4.2f} c   {ph}", font=f_mid, fill=(199, 60, 40) if "УДАР" in ph else (20, 20, 20))
        # torque bars
        x0, bw = 16, 260
        y0 = H - 60 * len(SHOW) + 10            # first bar top; label sits 18 px above each bar
        d.rectangle([0, y0 - 60, 600, H], fill=(255, 255, 255))
        d.text((x0, y0 - 56), "Момент серво / паспорт MG90S 0,177 Н·м (4,8 В)", font=f_sm, fill=(40, 40, 40))
        for j, (key, label) in enumerate(SHOW):
            v = abs(float(rows[k][key])); y = y0 + 60 * j
            frac = min(v / STALL, 1.2)
            col = (46, 160, 67) if frac < 0.5 else ((230, 150, 30) if frac < 0.9 else (210, 40, 40))
            d.rectangle([x0, y, x0 + bw, y + 24], outline=(150, 150, 150))
            d.rectangle([x0, y, x0 + int(bw * frac / 1.2), y + 24], fill=col)
            xl = x0 + int(bw / 1.2); d.line([xl, y - 2, xl, y + 26], fill=(0, 0, 0), width=2)
            d.text((x0, y - 18), f"{label}: {v:.3f} Н·м ({100 * v / STALL:3.0f}%), пик {100 * peak[key] / STALL:3.0f}%",
                   font=f_sm, fill=(20, 20, 20))
        im.save(tmp / f"f_{i:04d}.png")
    ff = shutil.which("ffmpeg") or "ffmpeg"
    subprocess.run([ff, "-y", "-loglevel", "error", "-framerate", str(fps), "-i", str(tmp / "f_%04d.png"),
                    "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", str(out)], check=True)
    print(f"{len(fr)} frames -> {out} ({Path(out).stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4]) if len(sys.argv) > 4 else 25.0)
