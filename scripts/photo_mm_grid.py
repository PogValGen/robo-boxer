import sys, numpy as np
from PIL import Image, ImageDraw
src, out, sx, sy = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
boxes = [tuple(map(int, b.split(","))) for b in sys.argv[5:]]
im = Image.open(src).convert("RGB")
for k, (x0, y0, x1, y1) in enumerate(boxes):
    c = im.crop((x0, y0, x1, y1)); d = ImageDraw.Draw(c)
    nx, ny = int((x1 - x0) / sx) + 1, int((y1 - y0) / sy) + 1
    for i in range(nx):
        x = i * sx
        d.line([(x, 0), (x, c.height)], fill=(0, 255, 0) if i % 5 else (255, 255, 0), width=1 if i % 5 else 2)
        if i % 5 == 0: d.text((x + 3, 2), str(i), fill=(255, 255, 0))
    for j in range(ny):
        y = j * sy
        d.line([(0, y), (c.width, y)], fill=(0, 255, 0) if j % 5 else (255, 255, 0), width=1 if j % 5 else 2)
        if j % 5 == 0: d.text((2, y + 3), str(j), fill=(255, 255, 0))
    c.save(f"{out}_{k}.png"); print(k, c.size)
