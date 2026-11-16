"""Set the SolidWorks mass of printed parts to the printed estimate (default 55 % of solid ABS: 3 perimeters +
30 % infill; TPU x1.2/1.04). Solid mass taken from a massprops json (sw_motion massprops) of an assembly in
which these parts are NOT overridden yet (just rebuilt).
Usage: python printed_mass.py <massprops.json> <part.SLDPRT> [...]   (factor via PRINT_FACTOR env)"""
import json, os, subprocess, sys
EXE = r"D:\AI prodject\robo-boxer\tools\physics\build_physics.exe".replace("\\\\", "\\")
F = float(os.environ.get("PRINT_FACTOR", "0.55"))
mp = {p["path"].lower(): p["m"] * 1000 for p in json.load(open(sys.argv[1], encoding="utf-8"))["parts"]}
for part in sys.argv[2:]:
    solid = mp.get(part.replace("\\", "/").lower())
    if solid is None:
        print(part, ": not in", sys.argv[1]); continue
    g = solid * F * (1.2 / 1.04 if "TPU" in part else 1.0)
    out = subprocess.run([EXE, "massfix", part, f"{g:.2f}"], capture_output=True, text=True).stdout.strip().splitlines()
    print(part, "|", [l for l in out if "mass" in l][-1:])
