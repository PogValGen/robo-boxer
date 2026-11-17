"""Interactive 3D window of the v3 assembly on this PC (no SolidWorks): mouse = rotate, wheel = zoom, shift = pan.

    python v3_view.py [L_knee=60 ...]      (cadenv python)

Colours: printed PLA ochre, MG90S + lid blue, horn white, pin grey, battery dark, board green.
"""
import sys

import numpy as np
import pyvista as pv

import v3_asm as A
from v3_asm_occ import placed
from pathlib import Path

COL = [("MG90S", "#2a5db0"), ("X3_Lid", "#2a5db0"), ("X3_Horn", "#f0f0ee"), ("X3_Pin", "#8a8f94"),
       ("X3_Battery", "#3c3f42"), ("X3_Board", "#2f7d3a")]

ang = {k: float(v) for k, v in (x.split("=") for x in sys.argv[1:])} or None
p = pv.Plotter(window_size=(1100, 1000), title="Gorilla v3 - robo-boxer (no SolidWorks)")
p.set_background("#e3e7e1")
for it in A.layout():
    stem = Path(it["file"]).stem
    m = placed(it, ang)
    col = next((c for k, c in COL if stem.startswith(k)), "#c8a040")
    p.add_mesh(pv.wrap(m), color=col, smooth_shading=False)
p.add_axes()
p.camera.up = (-1, 0, 0)                       # robot x points down
c = np.array(p.bounds).reshape(3, 2).mean(axis=1)
p.camera.focal_point = c
p.camera.position = tuple(c + np.array([-0.6, 1, 1.2]) * 700)
p.reset_camera()
p.add_text("mouse: rotate | wheel: zoom | shift+mouse: pan", font_size=9)
p.show()
