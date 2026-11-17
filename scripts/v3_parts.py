"""Robot-boxer v3 'Gorilla' parts (2026-11-14). Concept: docs/notes/2026-11-14_v3_concept.md.

All parts are modelled in the ROBOT frame at the zero pose (x down, y left, z forward, mm; origin on the hip pitch axis,
centre), left side; `sy = -1` builds the mirrored right-side part. Universal small parts (lid, pin, cap) are modelled
in the SERVO frame (l = SW X, s = SW Y = shaft axis, t = SW Z) like the MG90S model itself.

    python v3_parts.py lid pin cap block thigh shank foot ...   (names below; 'all' = everything)
"""
from __future__ import annotations

import asyncio
import json
import sys

import os
if os.environ.get("V3CAD", "occ") == "sw":          # old SolidWorks backend (only on explicit request since 2026-11-17)
    from v3cad import R, C, G, prism, new_part, save, session, PARTS, V3
else:                                                # build123d / OpenCascade, no SolidWorks (cadenv python)
    from v3cad_occ import R, C, G, prism, new_part, save, session, PARTS, V3

# ----------------------------------------------------------------------------- servo block (servo frame l, s, t)
BL, BT = 11.6, 6.5            # pocket half length / half thickness (user-fitted 23.2 x 13.0)
EL, ET = 16.4, 8.1            # block envelope half length (tabs +0.3) / half thickness (wall 1.6)
S_FLOOR, S_BOT = -12.1, -15.4  # pocket floor, outer bottom face (3.3 wall with the pin hole)
S_RIM, S_TABTOP = 5.15, 7.85  # tabs rest on the rim; tab cavity top in the lid
S_CASE, S_LID = 16.2, 19.0    # lid inner top, lid outer top (horn plate starts at 19.3)
AXL = 5.5                     # shaft offset from the body centre along l
GAP = 0.3                     # running clearance
ARM_T_A, ARM_T_B = 4.2, 3.0   # fork cheek thickness: horn side (+1.0 on 2026-11-16, user: cap plug 0.75 -> 1.75), pin side
S_A0, S_A1 = S_LID + GAP, S_LID + GAP + ARM_T_A      # 19.3 .. 23.5 horn-side cheek
S_B1, S_B0 = S_BOT - GAP, S_BOT - GAP - ARM_T_B      # -15.7 .. -18.7 pin-side cheek
PIN_D, PIN_HOLE, PIN_PRESS, PIN_DEPTH = 6.0, 6.3, 5.9, 2.6
PEG_L, PEG_D, PEG_HOLE, PEG_DEPTH = 13.75, 2.8, 3.0, 3.0      # OLD printed peg lid (2026-11-14): pegs broke, see below
# Lid <-> block = 2 FILAMENT PINS per servo (user 2026-11-16, "пины из прутка"): PLA filament D1.75 cut to FPIN_LEN, pressed
# into a blind hole in the lid from below, through the servo tab hole (model D3.0 -> +-0.6 slack, so the unmeasured tab
# pitch no longer matters) into a free hole in the block. Fibres along the pin: no layer split in bending like the
# printed standing pegs. Lid and block holes come from the same model -> they always agree.
FPIN_D = 1.75
FPIN_LID_D, FPIN_LID_DEPTH = 1.85, 8.0       # press (vertical printed holes come out ~0.1 under: trial print decides)
FPIN_BLK_D, FPIN_BLK_DEPTH = 2.0, 6.0        # free; s -0.85 .. 5.15, clear of the cable slot (s -12.1 .. -6.1)
# + 4 CORNER filament pins per servo (user 2026-11-17: "два пина мало, делай 4 по углам, а для сервы оставь то что было"):
# the 2 tab pins stay (they locate the servo), 4 more hold lid <-> block at the corners. The MG90S tabs are full width
# (t +-6.12) up to the ends (l +-16.12), so a corner pin must sit OUTSIDE the tab: t 7.3 (0.3 + pin radius clear of the
# tab). The 1.6 side wall cannot take a D2 hole there -> 4 corner lugs widen the block and the lid to t +-9.7 over
# l 12.4..16.4 only. Same pin length: lid hole from the lid bottom (through the tab layer) 10.7 deep, block hole 6.0.
# 17.11 v1 (lugs SIDEWAYS, t +-9.7): the 2 servos of the hip block sit back to back in t with no gap -> lid x lid
# 157 mm3, lid x block 204 mm3. v2: pins BEYOND THE TAB ENDS (l +-17.4, 0.4 clear of the tab end 16.12), t +-4.6,
# the lugs lengthen the block and the lid from l 16.4 to 19.6 (1.2 wall to the end), width unchanged.
# v2 is worse: lid x yoke 222, lid x shank 456 mm3 + 0.05 mm walls. Neither layout fits every joint -> "off" (the
# validated 16.11 design: 2 tab pins) until the user picks a way (fork reported 2026-11-17).
CPIN_MODE = "off"
if CPIN_MODE == "off":
    CPIN_L = CPIN_T = None
    LUG = ((0.0, 0.0), 0.0)
elif CPIN_MODE == "t":
    CPIN_L, CPIN_T = 14.2, 7.3
    LUG = ((11.6, 16.4), 9.7)          # lug l range, half width t
else:
    CPIN_L, CPIN_T = 17.4, 4.6
    LUG = ((16.4, 19.6), ET)
LUG_L0, LUG_L1, LUG_T = LUG[0][0], LUG[0][1], LUG[1]
FPIN_LEN = round(FPIN_LID_DEPTH + (S_TABTOP - S_RIM) + FPIN_BLK_DEPTH - 0.7, 1)   # 16.0: 0.7 short of the bottoms
# Stock single-arm horn, MEASURED 2026-11-15 (user, caliper): overall 21.4, hub D6.8 (shaft side), tip D3.85,
# plate 2.0 thick, height 4.7, spline cup 2.7 deep. Wedge angle from the photo on 5 mm grid paper (clean edge,
# mirrored): width 6.2 at r 3.9 -> 3.85 at the tip. The old "typical" pocket (r 15.2, constant width 5.4) did not fit.
HORN_AX = 21.4 - 6.8 / 2                             # 18.0 axis -> arm tip
HORN_TIP_R = 3.85 / 2                                # tip rounding, centre at r 16.075
HORN_ALPHA = 5.4                                     # wedge half angle, deg (photo, +-0.5)
HORN_T, HORN_H = 2.0, 4.7                            # plate thickness, overall height (hub 2.7 below the plate)
HORN_CLR = 0.15                                      # pocket clearance per side (trial print decides)
# The user TRIMS the horns (2026-11-15): cut between the 4th (r 10.8) and the 5th (r 12.8) hole from the hub, i.e. at
# r ~ 11.8 +-0.5. The pocket is longer than the stub on purpose (a hand cut is not exact; the torque goes through the
# wedge flanks) and stays short of the sole plate of the foot (its top is 13.8 from the ankle roll axis).
HORN_CUT = 11.8                                      # nominal length of the trimmed arm from the shaft axis
HORN_R = 12.7                                        # pocket reach (flat end); 13.0 left a 1.3 mm wall in the foot
HUB_R = 6.8 / 2 + HORN_CLR                           # 3.55
SHAFT_TOP = S_RIM + 14.5                             # 19.65 shaft end: tab underside -> shaft end 14.5 MEASURED by the
#                                                      user 2026-11-15 (the GrabCAD model had 14.75 -> 19.9)
HORN_S0, HORN_S1 = SHAFT_TOP, SHAFT_TOP + HORN_T     # 19.65 .. 21.65 horn plate (the cup bottoms on the shaft end)


def horn_hw(u, clr=0.0):
    """half width of the horn arm (+clr) at distance u from the shaft axis"""
    import math
    al = math.radians(HORN_ALPHA)
    return (HORN_TIP_R + clr) / math.cos(al) + (HORN_AX - HORN_TIP_R - u) * math.tan(al)


HORN_HW = horn_hw(0.0, HORN_CLR)                     # 3.60 widest half width of the pocket (at the hub)


def horn_poly(a, b, da, db, clr=HORN_CLR, n=8):
    """wedge outline of the horn arm (+clr) from the axis point (a, b) along the unit direction (da, db)"""
    import math
    al = math.radians(HORN_ALPHA)
    uc, rc = HORN_AX - HORN_TIP_R, HORN_TIP_R + clr
    if HORN_R < uc:                                        # trimmed horn: wedge with a flat end at the pocket reach
        e = HORN_R - (HORN_CLR - clr)
        loc = [(0.0, -horn_hw(0.0, clr)), (e, -horn_hw(e, clr)), (e, horn_hw(e, clr)), (0.0, horn_hw(0.0, clr))]
        return G([(a + da * u - db * v, b + db * u + da * v) for u, v in loc])
    loc = [(0.0, -horn_hw(0.0, clr))]
    loc += [(uc + rc * math.sin(t), -rc * math.cos(t)) for t in
            (al + (math.pi - 2 * al) * i / n for i in range(n + 1))]
    loc.append((0.0, horn_hw(0.0, clr)))
    return G([(a + da * u - db * v, b + db * u + da * v) for u, v in loc])
CAP_TIP = 13.5                # cap flange reach along the horn arm (shorter than the pocket: see part_cap)
DISC = 6.5                    # cheek end radius around an axis
STRIP = 6.5                   # cheek strip half width = DISC: a stadium with straight edges (lies flat on the bed)

# ----------------------------------------------------------------------------- layout (left side, robot frame)
YL = 24.0                     # leg centre line
YS = 22.2                     # s = 0 plane of the pitch servos (block y 6.8 .. 41.2)
Y_IN, Y_OUT = YS + S_BOT, YS + S_LID                 # 6.8, 41.2
YA0, YA1 = YS + S_A0, YS + S_A1                      # 41.5 .. 44.7 outer cheek (horn)
YB0, YB1 = YS + S_B0, YS + S_B1                      # 3.5 .. 6.5 inner cheek (pin)
XR = 16.2                     # roll axis above (hip) / below (ankle) the pitch axis
YR = 18.5                     # roll axis y
ZR0 = -7.3                    # s = 0 plane of the roll servo (block z -22.7 .. 11.7)
X_KNEE, X_ANK = 46.0, 92.0
X_SOLE = 125.0
AXES = "xyz"


def sgn(v):
    return 1 if v >= 0 else -1


MIN_WALL = 1.6                # 4 perimeters of a 0.4 nozzle
THIN: list = []               # thin walls found by Part._wall (part, why, mm)
_cur = {"name": ""}


def _outline(s, step=0.25):
    """points along the outline of a shape"""
    import math
    if s[0] == "circle":
        n = max(24, int(2 * math.pi * s[3] / step))
        return [(s[1] + s[3] * math.cos(2 * math.pi * i / n), s[2] + s[3] * math.sin(2 * math.pi * i / n)) for i in range(n)]
    if s[0] == "rect":
        a0, a1, b0, b1 = min(s[1], s[3]), max(s[1], s[3]), min(s[2], s[4]), max(s[2], s[4])
        pts = [(a0, b0), (a1, b0), (a1, b1), (a0, b1)]
    else:
        pts = list(s[1])
    out = []
    for (pa, pb), (qa, qb) in zip(pts, pts[1:] + pts[:1]):
        n = max(1, int(math.hypot(qa - pa, qb - pb) / step))
        out += [(pa + (qa - pa) * i / n, pb + (qb - pb) * i / n) for i in range(n)]
    return out


def _inside(s, p, m=0.0):
    """point inside the shape shrunk by m"""
    if s[0] == "circle":
        return (p[0] - s[1]) ** 2 + (p[1] - s[2]) ** 2 < (s[3] - m) ** 2
    if s[0] == "rect":
        return (min(s[1], s[3]) + m < p[0] < max(s[1], s[3]) - m) and (min(s[2], s[4]) + m < p[1] < max(s[2], s[4]) - m)
    pts, c = s[1], False
    for (pa, pb), (qa, qb) in zip(pts, pts[1:] + pts[:1]):
        if (pb > p[1]) != (qb > p[1]) and p[0] < (qa - pa) * (p[1] - pb) / (qb - pb) + pa:
            c = not c
    return c


class Part:
    """prisms with the y mirror folded in + the thin-wall rule: after a cut, the wall left between the cut and the
    outer edge of the bosses of the same layer must be >= MIN_WALL (2026-11-14: a horn pocket left a 1.3 mm wall
    in the shank plate and only the user noticed it)"""

    def __init__(self, sy=1):
        self.sy = sy
        self.layers = []                                   # (axis, c0, c1, shapes) of the bosses

    def _wall(self, axis, c0, c1, shapes, why, min_wall=MIN_WALL):
        import math
        c0, c1 = sorted((c0, c1))
        bos = [sh for ax, a0, a1, shs in self.layers if ax == axis and min(a1, c1) - max(a0, c0) > 0.2 for sh in shs]
        if not bos:
            return
        edge = [p for b in bos for p in _outline(b) if not any(_inside(o, p, 0.02) for o in bos if o is not b)]
        for sh in shapes:
            allp = _outline(sh)
            cut = [p for p in allp if any(_inside(b, p, 0.02) for b in bos)]
            outp = [p for p in allp if not any(_inside(b, p, -0.02) for b in bos)]
            if not cut or not edge or len(cut) < 0.5 * len(allp):
                continue                                    # the cut leaves the material on purpose (open slot)
            # an open mouth of the cut is not a wall: skip the outline next to the points that left the material
            cut = [p for p in cut if all(math.hypot(p[0] - o[0], p[1] - o[1]) > 2.5 for o in outp)]
            if not cut:
                continue
            d = min(math.hypot(p[0] - q[0], p[1] - q[1]) for p in cut for q in edge)
            if d < min_wall - 0.01:
                THIN.append((_cur["name"], why, round(d, 2)))
                print(f"   THIN WALL {d:.2f} mm < {min_wall}: {why} ({axis} {c0:.1f}..{c1:.1f})", flush=True)

    def _m(self, axis, shapes):
        if self.sy == 1 or axis == "y":
            # axis y: shapes are (z, x) - no y inside
            return shapes
        out = []
        iy = 0 if axis == "x" else 1          # axis x: (y, z) -> y is a; axis z: (x, y) -> y is b
        for s in shapes:
            if s[0] == "rect":
                v = list(s[1:])
                v[iy], v[iy + 2] = -v[iy], -v[iy + 2]
                out.append(R(*v))
            elif s[0] == "circle":
                v = list(s[1:])
                v[iy] = -v[iy]
                out.append(C(*v))
            else:
                out.append(G([((-a, b) if iy == 0 else (a, -b)) for a, b in s[1]]))
        return out

    async def prism(self, axis, c0, c1, shapes, cut=False, why="", min_wall=MIN_WALL):
        if cut:
            self._wall(axis, c0, c1, shapes, why, min_wall)
        else:
            self.layers.append((axis, *sorted((c0, c1)), list(shapes)))
        if axis == "y":
            c0, c1 = self.sy * c0, self.sy * c1
        return await prism(axis, c0, c1, self._m(axis, shapes), cut=cut, why=why)

    async def box(self, x0, x1, y0, y1, z0, z1, cut=False, why="", ax=None):
        d = {"x": abs(x1 - x0), "y": abs(y1 - y0), "z": abs(z1 - z0)}
        ax = ax or min(d, key=d.get)
        if ax == "x":
            return await self.prism("x", x0, x1, [R(y0, z0, y1, z1)], cut, why)
        if ax == "y":
            return await self.prism("y", y0, y1, [R(z0, x0, z1, x1)], cut, why)
        return await self.prism("z", z0, z1, [R(x0, y0, x1, y1)], cut, why)

    async def cyl(self, axis, c0, c1, p, q, r, cut=False, why="", min_wall=MIN_WALL):
        """cylinder along axis; (p, q) = the two other coordinates in cyclic order (x: y,z; y: z,x; z: x,y)"""
        return await self.prism(axis, c0, c1, [C(p, q, r)], cut, why, min_wall)


class Srv:
    """A servo placed axis-aligned in the robot frame: pt = point of the shaft axis at s = 0, S / L = signed axes
    ('+y', '-x', ...) of the shaft direction and of the +l direction (the shaft sits at l = +5.5; the tail is -l)."""

    def __init__(self, pt, S, L):
        self.S, self.L = S, L
        self.sv, self.lv = self._vec(S), self._vec(L)
        self.tv = [self.lv[1] * self.sv[2] - self.lv[2] * self.sv[1], self.lv[2] * self.sv[0] - self.lv[0] * self.sv[2],
                   self.lv[0] * self.sv[1] - self.lv[1] * self.sv[0]]          # t = l x s
        self.o = [pt[i] - AXL * self.lv[i] for i in range(3)]                   # servo frame origin (body centre line)
        self.pt = list(pt)

    @staticmethod
    def _vec(a):
        v = [0.0, 0.0, 0.0]
        v[AXES.index(a[1])] = 1.0 if a[0] == "+" else -1.0
        return v

    def p(self, l, s, t):
        return [self.o[i] + l * self.lv[i] + s * self.sv[i] + t * self.tv[i] for i in range(3)]

    def rng(self, l0, l1, s0, s1, t0, t1):
        a, b = self.p(l0, s0, t0), self.p(l1, s1, t1)
        return [v for i in range(3) for v in sorted((a[i], b[i]))]             # x0, x1, y0, y1, z0, z1

    def matrix(self):
        """SW component transform data (rotation rows = images of the servo X, Y, Z axes; translation in m)."""
        return [*self.lv, *self.sv, *self.tv, *[v / 1000.0 for v in self.o], 1.0, 0.0, 0.0, 0.0]

    # ---- features (P = Part; y mirror is applied by P)
    async def body(self, P, why="servo block body"):
        await P.box(*self.rng(-EL, EL, S_BOT, S_RIM, -ET, ET), why=why, ax=self.S[1])
        for l0, l1 in (((LUG_L0, LUG_L1), (-LUG_L1, -LUG_L0)) if CPIN_L else ()):
            await P.box(*self.rng(l0, l1, S_BOT, S_RIM, -LUG_T, LUG_T), why="corner pin lugs", ax=self.S[1])

    async def pocket(self, P, cable=True, sag=(0.0, 0.0)):
        """sag = extra clearance on the -t / +t wall: the one that is printed as a long bridge sags"""
        ax = self.S[1]
        await P.box(*self.rng(-BL, BL, S_FLOOR, S_RIM, -BT - sag[0], BT + sag[1]), cut=True,
                    why="servo pocket 23.2 x 13.0", ax=ax)
        for l in (-PEG_L, PEG_L):
            # filament pin hole D2.0 (2026-11-16): 1.65 to the block end, 1.15 to the pocket - exception to MIN_WALL:
            # the wall is 3 perimeters around a vertical hole and only sees the pin's side load, which the servo tab
            # (in the pocket) shares. The old D3 hole needed a keyway (0.65 sliver); D2 does not.
            await self.hole(P, l, S_RIM - FPIN_BLK_DEPTH, S_RIM, FPIN_BLK_D / 2, "filament pin hole D2.0 (free)",
                            min_wall=1.1)
        for l in ((-CPIN_L, CPIN_L) if CPIN_L else ()):
            for t in (-CPIN_T, CPIN_T):
                # 1.2 to the block end, 1.4 to the lug side: same 3-perimeter exception as the tab pin holes
                await self.hole(P, l, S_RIM - FPIN_BLK_DEPTH, S_RIM, FPIN_BLK_D / 2, "corner filament pin hole D2.0",
                                min_wall=1.1, t=t)
        await self.hole(P, AXL, S_BOT, S_BOT + PIN_DEPTH, PIN_HOLE / 2, "pin hole D6.3 (plain bushing)")
        if cable:
            await P.box(*self.rng(BL, EL, S_FLOOR, S_FLOOR + 6.0, -2.2, 2.2), cut=True, why="cable slot (shaft end)",
                        ax=self.L[1])

    async def hole(self, P, l, s0, s1, r, why, cut=True, min_wall=MIN_WALL, t=0.0):
        ax = self.S[1]
        a, b = self.p(l, s0, t), self.p(l, s1, t)
        i = AXES.index(ax)
        pq = [(1, 2), (2, 0), (0, 1)][i]
        await P.cyl(ax, a[i], b[i], a[pq[0]], a[pq[1]], r, cut=cut, why=why, min_wall=min_wall)

    def lid_rng(self):
        return self.rng(-EL, EL, S_RIM, S_LID, -ET, ET)


def cheek_shapes(c_axis_pt2, toward, length, horn=False, pin=False, disc=DISC, strip=STRIP):
    """2D outline pieces of a fork cheek in the plane perpendicular to the joint axis (coordinates = the two plane
    axes in cyclic order): disc around the axis + strip of half width `strip` running `length` along `toward`
    (a unit direction in the plane, axis-aligned). Returns (boss shapes, cut shapes)."""
    a, b = c_axis_pt2
    da, db = toward
    boss = [C(a, b, disc)]
    if length > 0:
        if da:
            boss.append(R(a, b - strip, a + da * length, b + strip))
        else:
            boss.append(R(a - strip, b, a + strip, b + db * length))
    cuts = []
    if horn:
        cuts.append(C(a, b, HUB_R))
        cuts.append(horn_poly(a, b, da, db))               # wedge pocket = the measured horn + HORN_CLR
    if pin:
        cuts.append(C(a, b, PIN_PRESS / 2))
    return boss, cuts


# ============================================================================= universal small parts (servo frame)
async def part_lid():
    await new_part("X3_Lid")
    await prism("y", S_RIM, S_LID, [R(-ET, -EL, ET, EL)], why="lid block s 5.15..19.0")                 # (z, x) = (t, l)
    if CPIN_L:
        await prism("y", S_RIM, S_LID, [R(-LUG_T, LUG_L0, LUG_T, LUG_L1), R(-LUG_T, -LUG_L1, LUG_T, -LUG_L0)],
                    why="corner pin lugs")
    await prism("y", S_RIM, S_TABTOP, [R(-BT, -EL, BT, EL)], cut=True, why="tab cavity (open ends)")
    await prism("y", S_TABTOP, S_CASE, [R(-BT, -BL - 0.1, BT, BL + 0.1)], cut=True, why="upper case cavity")
    await prism("y", S_CASE, S_LID, [C(0, AXL, 3.8)], cut=True, why="horn hub hole D7.6")
    # 2 blind holes from the tab cavity up for the filament pins (pressed in before the lid goes on); 1.15 mm wall to
    # the upper case cavity - same exception as in the block, 1.7 to the lid end
    await prism("y", S_TABTOP - 0.02, S_TABTOP + FPIN_LID_DEPTH,
                [C(0, -PEG_L, FPIN_LID_D / 2), C(0, PEG_L, FPIN_LID_D / 2)], cut=True,
                why=f"2 filament pin holes D{FPIN_LID_D} x {FPIN_LID_DEPTH} (press), pin D{FPIN_D} x {FPIN_LEN}")
    # 4 corner pins: from the lid bottom through the tab layer, the same pin length as the tab pins
    cdep = (S_TABTOP - S_RIM) + FPIN_LID_DEPTH
    if CPIN_L:
        await prism("y", S_RIM - 0.02, S_RIM + cdep,
                [C(t, l, FPIN_LID_D / 2) for l in (-CPIN_L, CPIN_L) for t in (-CPIN_T, CPIN_T)], cut=True,
                why=f"4 corner filament pin holes D{FPIN_LID_D} x {cdep:.1f} (press)")
    return await save(PARTS / "X3_Lid.SLDPRT", shots=("isometric",))


async def part_pin():
    await new_part("X3_Pin")
    L = ARM_T_B + GAP + PIN_DEPTH - 0.2                                                               # 5.7
    await prism("y", 0, L, [C(0, 0, PIN_D / 2)], why="pin D6: press in the cheek, free in the block")
    await prism("y", -1.0, 0, [C(0, 0, 5.0)], why="head D10 x 1")
    return await save(PARTS / "X3_Pin.SLDPRT", shots=("isometric",))


async def part_cap():
    await new_part("X3_Cap")
    s0, s1 = HORN_S1 + 0.1, S_A1                                                                      # plug above the horn plate
    if s1 - s0 < 1.0:
        # OPEN DESIGN ISSUE (2026-11-15): the real horn plate is 2.0 thick and ends at s 21.9 -> only 0.5 mm of the
        # 3.2 mm cheek is left for the press plug. Decide: thicker horn cheek (ARM_T_A) or cap held by the horn screw.
        THIN.append(("X3_Cap", "press plug depth above the horn plate", round(s1 - s0, 2)))
        print(f"   OPEN ISSUE: cap plug only {s1 - s0:.2f} mm deep (horn plate s {HORN_S0}..{HORN_S1}, cheek to {s1})", flush=True)
    out = [C(0, 0, HUB_R - 0.05), horn_poly(0, 0, 0, 1, clr=HORN_CLR - 0.05)]                         # (z, x): horn along +x
    await prism("y", s0, s1, out, why="plug = horn outline (press), holds the horn down")
    # the flange overhangs the pocket on the sides and around the hub only: past the arm tip it would run into the
    # sole plate of the foot (assembly 2026-11-14: 19.5 mm3 with a tip overhang; foot top is 13.8 from the roll axis)
    await prism("y", s1, s1 + 0.8, [C(0, 0, HUB_R + 1.5), R(-HORN_HW - 1.5, 0, HORN_HW + 1.5, CAP_TIP)], why="flange 0.8")
    await prism("y", s0, s1 + 0.8, [C(0, 0, 1.3)], cut=True, why="hole for the horn screw / pry point")
    return await save(PARTS / "X3_Cap.SLDPRT", shots=("isometric",))


async def part_horntest():
    """fit coupon (2026-11-16): ONE horn-side fork cheek exactly as in every joint - ARM_T_A thick, disc + 20 mm strip,
    through wedge pocket for the TRIMMED horn (HORN_CUT) + hub hole. Printed with X3_Cap. Checks: the trimmed horn
    drops into the pocket without play along the wedge flanks, the cap plug presses in and holds the horn plate."""
    await new_part("X3_HornTest")
    b, c = cheek_shapes((0.0, 0.0), (0, 1), 20.0, horn=True)                                          # (z, x): horn along +x
    await prism("y", S_A0, S_A1, b, why=f"cheek s {S_A0}..{S_A1} (ARM_T_A {ARM_T_A})")
    await prism("y", S_A0, S_A1, c, cut=True, why="hub hole + wedge pocket (through, like the joints)")
    return await save(PARTS / "X3_HornTest.SLDPRT", shots=("isometric",))


# ============================================================================= X block (hip L; mirrored R)
def srv_hip_pitch(sy=1):
    return Srv((0.0, YS, 0.0), "+y", "+z")            # mirrored by Part (sy) for the geometry


def srv_hip_roll():
    return Srv((-XR, YR, ZR0), "+z", "-y")


async def part_block(sy=1):
    name = "X3_Block_L" if sy == 1 else "X3_Block_R"
    await new_part(name)
    P = Part(sy)
    sp, sr = srv_hip_pitch(), srv_hip_roll()
    await sp.body(P, "pitch tier")
    await sr.body(P, "roll tier (above)")
    x0, x1, y0, _, z0, z1 = sr.rng(-EL, EL, S_BOT, S_RIM, -ET, ET)
    await P.box(x0, x1, Y_IN, y0 + 0.2, z0, z1, why="roll tier down to the print base (y 6.8)", ax="y")
    await sp.pocket(P)
    await sr.pocket(P)
    # lightening + look: chamfer-like window on the free upper back corner is left for the styling pass
    return await save(PARTS / f"{name}.SLDPRT", shots=("isometric", "front", "right"))


# ============================================================================= thigh (H fork, symmetric: one part)
async def part_thigh():
    await new_part("X3_Thigh")
    P = Part(1)
    # cheeks in the x-z plane; prism axis y -> shape coordinates (z, x)
    for y0, y1, horn, pin in ((YA0, YA1, True, False), (YB0, YB1, False, True)):
        b1, c1 = cheek_shapes((0.0, 0.0), (0, 1), X_KNEE, horn=horn, pin=pin)          # hip end, strip down to the knee
        b2, c2 = cheek_shapes((0.0, X_KNEE), (0, -1), 0, horn=horn, pin=pin)           # knee end disc
        await P.prism("y", y0, y1, b1 + b2, why=f"cheek y {y0}..{y1}")
        await P.prism("y", y0, y1, c1 + c2, cut=True, why="horn pockets" if horn else "pin holes D5.9")
    await P.box(16.0, 21.0, YB0, YA1, -STRIP, STRIP, why="web 5 mm (clears the hip block up to 125 deg flexion)", ax="x")
    await P.box(17.5, 19.5, YB1 + 4, YA0 - 4, -STRIP, STRIP, cut=True, why="lightening slot", ax="z")
    return await save(PARTS / "X3_Thigh.SLDPRT", shots=("isometric", "front"))


# ============================================================================= shank (knee block + ankle fork)
def srv_knee():
    return Srv((X_KNEE, YS, 0.0), "+y", "-x")         # tail down the shank


async def part_shank(sy=1):
    name = "X3_Shank_L" if sy == 1 else "X3_Shank_R"
    await new_part(name)
    P = Part(sy)
    sk = srv_knee()
    await sk.body(P, "knee servo block (tail down)")
    x_att = X_KNEE + 11.0                          # plates start below the thigh cheek discs (R8 + clearance)
    zp0, zp1 = -4.0, ET                            # plates in the front half (the thigh strips fold behind them),
    #                                                front edge flush with the knee block = the print base
    # inner plate: K block face -> ankle disc (pin side)
    b, c = cheek_shapes((0.0, X_ANK), (0, -1), 0, pin=True)
    # the plates sit 2 mm forward (knee flexion), but next to the ankle the thigh never comes: full width there,
    # otherwise the horn pocket leaves a 1.3 mm wall (user 2026-11-14: "the cut-out is off centre")
    wide = R(-STRIP, X_ANK - 20.0, zp1, X_ANK - 4.0)
    sq = R(0.0, X_ANK - 4.0, zp1, X_ANK + DISC)        # square front corner of the ankle end: lies on the bed
    await P.prism("y", YB0, YB1, [R(zp0, x_att, zp1, X_ANK - 4.0), wide, sq] + b, why="inner plate + ankle disc")
    await P.prism("y", YB0, YB1, c, cut=True, why="ankle pin hole D5.9")
    # cross beam under the knee lid, outer plate (horn side)
    xb0, xb1 = sk.rng(-EL, EL, 0, 0, 0, 0)[1] + GAP, sk.rng(-EL, EL, 0, 0, 0, 0)[1] + GAP + 4.0
    await P.box(xb0, xb1, YB0, YA1, zp0, zp1, why="cross beam under the knee block", ax="x")
    # the beam keeps 0.3 to the removable lid, but the block BODY must be one piece with it (it was held by a face
    # contact only): neck from the block end into the beam over the body height
    await P.box(xb0 - GAP - 1.0, xb0 + 1.0, Y_IN, YS + S_RIM, zp0, zp1, why="neck: knee block -> cross beam", ax="x")
    b, c = cheek_shapes((0.0, X_ANK), (0, -1), 0, horn=True)
    await P.prism("y", YA0, YA1, [R(zp0, xb0, zp1, X_ANK - 4.0), wide, sq] + b, why="outer plate + ankle disc")
    await P.prism("y", YA0, YA1, c, cut=True, why="ankle horn pocket")
    await sk.pocket(P)                             # printed front-down: the -z wall is a 23 mm bridge (0.37 mm fit
    #                                                clearance takes the sag; an extra relief left a 1.3 mm wall)
    return await save(PARTS / f"{name}.SLDPRT", shots=("isometric", "front"))


# ============================================================================= foot
X_AR = X_ANK + XR                                  # ankle roll axis x = 108.2
FOOT_TOP = 122.0


async def part_foot(sy=1):
    name = "X3_Foot_L" if sy == 1 else "X3_Foot_R"
    await new_part(name)
    P = Part(sy)
    await P.box(FOOT_TOP, X_SOLE, 2.0, 46.0, -38.0, 47.0, why="sole plate 85 x 44", ax="x")
    za0, za1 = ZR0 + S_A0, ZR0 + S_A1              # 12.0 .. 15.2 front cheek (horn)
    zb0, zb1 = ZR0 + S_B0, ZR0 + S_B1              # -26.0 .. -23.0 back cheek (pin)
    for z0, z1, horn, pin in ((za0, za1, True, False), (zb0, zb1, False, True)):
        b, c = cheek_shapes((X_AR, YR), (1, 0), FOOT_TOP - X_AR + 0.5, horn=horn, pin=pin)   # axis z -> (x, y)
        await P.prism("z", z0, z1, b, why=f"roll cheek z {z0:.1f}..{z1:.1f}")
        await P.prism("z", z0, z1, c, cut=True, why="horn pocket" if horn else "pin hole")
    return await save(PARTS / f"{name}.SLDPRT", shots=("isometric",))


# ============================================================================= yoke (yaw C-bracket + hip roll fork)
X_YAW0 = -56.3                                     # s = 0 plane of the yaw servo (shaft down): block x -71.7 .. -37.3
XYA0, XYA1 = X_YAW0 + S_A0, X_YAW0 + S_A1          # -37.0 .. -33.8 lower cheek (horn)
XYB0, XYB1 = X_YAW0 + S_B0, X_YAW0 + S_B1          # -75.0 .. -72.0 upper cheek (pin)
Z_SP0, Z_SP1 = -27.4, -24.0                        # spine behind the yaw block (block corner sweeps r 23.35)


def srv_yaw():
    return Srv((X_YAW0, YL, 0.0), "+x", "+z")      # body y 15.9 .. 32.1, z -21.9 .. 10.9, tail to the back


async def part_yoke(sy=1):
    name = "X3_Yoke_L" if sy == 1 else "X3_Yoke_R"
    await new_part(name)
    P = Part(sy)
    za0, za1 = ZR0 + S_A0, ZR0 + S_A1              # 12.0 .. 15.2 front roll cheek (horn)
    zb0, zb1 = ZR0 + S_B0, ZR0 + S_B1              # -26.0 .. -23.0 back roll cheek (pin)
    # lower plate = horn cheek of the yaw servo; axis x -> shapes (y, z)
    await P.prism("x", XYA0, XYA1, [R(YR - STRIP, Z_SP0, YL + DISC, za1)], why="lower plate (yaw horn cheek)")
    _, c = cheek_shapes((YL, 0.0), (0, -1), 0, horn=True)
    await P.prism("x", XYA0, XYA1, c, cut=True, why="yaw horn pocket (arm to the back)")
    # hip roll fork hanging from the plate; axis z -> shapes (x, y)
    for z0, z1, horn, pin in ((za0, za1, True, False), (zb0, zb1, False, True)):
        b, c = cheek_shapes((-XR, YR), (-1, 0), -XR - XYA0, horn=horn, pin=pin)
        await P.prism("z", z0, z1, b, why=f"roll cheek z {z0:.1f}..{z1:.1f}")
        await P.prism("z", z0, z1, c, cut=True, why="roll horn pocket" if horn else "roll pin hole D5.9")
    # spine behind the yaw block + upper cheek with the pin hole
    yb = YR - STRIP                                # 12.0: the print base, every wall of the yoke starts here
    await P.box(XYB0, XYA1, yb, YL + STRIP, Z_SP0, Z_SP1, why="spine behind the yaw block", ax="z")
    await P.prism("x", XYB0, XYB1, [C(YL, 0.0, DISC), R(yb, Z_SP0, YL + STRIP, 0.0)], why="upper cheek (yaw pin)")
    await P.prism("x", XYB0, XYB1, [C(YL, 0.0, PIN_PRESS / 2)], cut=True, why="yaw pin hole D5.9")
    return await save(PARTS / f"{name}.SLDPRT", shots=("isometric", "front", "right"))


# ============================================================================= upper body layout (left side)
X_SH, Y_SH0 = -85.0, 61.2                          # shoulder pitch axis x; s = 0 plane (block y 45.8 .. 80.2, shaft outward)
X_AB, Y_AB, Z_AB0 = -59.4, 64.5, STRIP - S_RIM     # abduction axis (along z); rim z 6.5 = cheek edge = the print base
X_EL, Y_EL0 = -19.2, 64.5 + STRIP - S_RIM          # elbow axis x; rim y 71.0 = abduction cheek edge = the print base
BAY_X0, BAY_X1 = -100.1, -77.0                     # battery bay 2x18650 holder (78 x 42 x 21.6 inside) on top of the yaw blocks
BAY_Y, BAY_Z0, BAY_Z1, BAY_W = 39.0, -31.0, 11.0, 1.6
RAIL = [(40.0, -25.0), (45.5, -25.0), (45.5, -11.2), (40.0, -5.7)]   # rail block (y, z): 45 deg underside (printed front-down)
X_RIM_YAW = X_YAW0 + S_RIM                         # -51.15 lower edge of the yaw blocks
# pod tongue (y, z): neck 4 wide, 45 deg flare to 8; its lower flank is at z -21.9 = rear face of the pod = print base
DOVE = [(46.3, -19.9), (46.3, -15.9), (43.8, -15.9), (41.8, -13.9), (41.8, -21.9), (43.8, -19.9)]
DOVE_SLOT = [(45.6, -20.1), (45.6, -15.7), (43.75, -15.7), (41.6, -13.55), (41.6, -22.25), (43.75, -20.1)]   # 0.2 clearance
PCB_X, PCB_Y = (-94.5, -55.5), 29.0                # RTrobot 32ch board 45 x 64, holes 39 x 58 (manual hw3 p.32)
FIST_Y = 68.0


def srv_shoulder():
    return Srv((X_SH, Y_SH0, 0.0), "+y", "+z")     # body x -93.1..-76.9, z -21.9..10.9 (tail to the back)


def srv_abd():
    return Srv((X_AB, Y_AB, Z_AB0), "+z", "+y")    # body y 42.6..75.4, x -67.5..-51.3, z -15.05..5.5


def srv_elbow():
    return Srv((X_EL, Y_EL0, 0.0), "+y", "+x")     # body x -41.1..-8.3 (tail up), z +-8.1


async def part_torso():
    await new_part("X3_Torso")
    L, Rt = Part(1), Part(-1)
    sy = srv_yaw()
    # printed FRONT-DOWN (z = 12.6 on the bed): floor and side walls are vertical; no rear wall (it would be a
    # cantilevered ceiling) - the battery is closed by the clip-on rear cover (armour)
    await L.box(BAY_X0, BAY_X1, -BAY_Y - BAY_W, BAY_Y + BAY_W, BAY_Z0, BAY_Z1 + BAY_W, why="battery bay shell", ax="x")
    await L.box(X_YAW0 + S_BOT, X_RIM_YAW, -16.5, 16.5, -4.0, 10.9, why="core between the yaw blocks", ax="z")
    # the yoke upper cheek (x -75..-72, y from 12) sweeps r <= 17 around the yaw axis (y 24) at yaw +-45 deg:
    # the neck must stay inside |y| < 24 - 17.5 (assembly check 2026-11-14: 36 mm3 overlap with a 30 mm neck)
    await L.box(BAY_X1 - 1.0, X_YAW0 + S_BOT, -6.5, 6.5, -2.0, 10.9, why="core neck (clears the yoke upper cheeks)", ax="z")
    await L.box(BAY_X1 - 1.0, X_RIM_YAW, -(YL + ET), YL + ET, 10.5, BAY_Z1 + BAY_W, why="front plate", ax="z")
    for P in (L, Rt):
        await sy.body(P, "yaw servo block")
        await P.prism("x", BAY_X0, BAY_X1 + 1.6, [G(RAIL)], why="shoulder rail block (45 deg underside)")
    zf = BAY_Z1 + BAY_W
    pts = [(x, y) for x in PCB_X for y in (-PCB_Y, PCB_Y)]
    # ---- cuts
    await L.box(BAY_X0, BAY_X1 - 1.5, -BAY_Y, BAY_Y, BAY_Z0 - 1.0, BAY_Z1, cut=True, why="battery bay 78 x 42 x 21.6, open to the rear", ax="x")
    await L.prism("z", 10.4, zf, [C(x, y, 1.45) for x, y in pts], cut=True, why="4 holes D2.9 for the board standoffs (separate parts)")
    for P in (L, Rt):
        await P.prism("x", BAY_X0, BAY_X1 + 0.2, [G(DOVE_SLOT)], cut=True, why="dovetail slot for the shoulder pod (from the top)")
        await sy.pocket(P)
        await P.box(X_YAW0 + S_FLOOR, X_YAW0 + S_FLOOR + 6.0, YL - 2.2, YL + 2.2, 10.4, zf, cut=True,
                    why="cable window through the front plate", ax="z")
        await P.prism("x", BAY_X1 - 1.5, BAY_X1, [C(YL, 0.0, 5.5)], cut=True, why="access hole for the yaw pin (bay floor)")
    return await save(PARTS / "X3_Torso.SLDPRT", shots=("isometric", "front", "top"))


async def part_pod(sy=1):
    name = "X3_Pod_L" if sy == 1 else "X3_Pod_R"
    await new_part(name)
    P = Part(sy)
    ss = srv_shoulder()
    await ss.body(P, "shoulder servo block")
    x0, x1 = ss.rng(-EL, EL, 0, 0, -ET, ET)[0:2]
    await P.prism("x", x0, x1, [G(DOVE)], why="dovetail tongue (slides into the torso from the top)")
    await ss.pocket(P)
    return await save(PARTS / f"{name}.SLDPRT", shots=("isometric",))


async def part_shoulder(sy=1):
    name = "X3_Shoulder_L" if sy == 1 else "X3_Shoulder_R"
    await new_part(name)
    P = Part(sy)
    sa = srv_abd()
    xb0, xb1, yb0, yb1 = sa.rng(-EL, EL, S_BOT, S_RIM, -ET, ET)[0:4]              # -67.5, -51.3, 42.6, 75.4
    ya0, ya1, yp0, yp1 = Y_SH0 + S_A0, Y_SH0 + S_A1, Y_SH0 + S_B0, Y_SH0 + S_B1   # 80.5, 83.7, 42.5, 45.5
    ln = (xb0 + 1.5) - X_SH                                                       # strips run 1.5 into the block top
    await sa.body(P, "abduction servo block")
    await P.box(xb0, xb0 + 4.5, yb1 - 2.4, ya1, -STRIP, STRIP, why="beam: outer cheek -> block", ax="x")
    for y0, y1, horn, pin in ((ya0, ya1, True, False), (yp0, yp1, False, True)):
        b, c = cheek_shapes((0.0, X_SH), (0, 1), ln, horn=horn, pin=pin)          # axis y -> (z, x)
        await P.prism("y", y0, y1, b, why=f"shoulder cheek y {y0:.1f}..{y1:.1f}")
        await P.prism("y", y0, y1, c, cut=True, why="horn pocket" if horn else "pin hole D5.9")
    await sa.pocket(P)
    return await save(PARTS / f"{name}.SLDPRT", shots=("isometric", "front"))


async def part_upperarm(sy=1):
    name = "X3_UpperArm_L" if sy == 1 else "X3_UpperArm_R"
    await new_part(name)
    P = Part(sy)
    se = srv_elbow()
    xe0 = se.rng(-EL, EL, 0, 0, 0, 0)[0]                                          # -41.1 block top
    za0, za1, zb0, zb1 = Z_AB0 + S_A0, Z_AB0 + S_A1, Z_AB0 + S_B0, Z_AB0 + S_B1   # 19.65..22.85, -18.35..-15.35
    await se.body(P, "elbow servo block (tail up)")
    await P.box(xe0 - 3.5, xe0 + 1.1, Y_AB - STRIP, Y_AB + STRIP, zb0, za1, why="bridge plate on the block top", ax="x")
    for z0, z1, horn, pin in ((za0, za1, True, False), (zb0, zb1, False, True)):
        b, c = cheek_shapes((X_AB, Y_AB), (1, 0), xe0 - X_AB, horn=horn, pin=pin)  # axis z -> (x, y)
        await P.prism("z", z0, z1, b, why=f"abduction cheek z {z0:.2f}..{z1:.2f}")
        await P.prism("z", z0, z1, c, cut=True, why="horn pocket" if horn else "pin hole D5.9")
    await se.pocket(P)
    return await save(PARTS / f"{name}.SLDPRT", shots=("isometric", "front"))


async def part_forearm(sy=1):
    name = "X3_Forearm_L" if sy == 1 else "X3_Forearm_R"
    await new_part(name)
    P = Part(sy)
    ya0, ya1, yp0, yp1 = Y_EL0 + S_A0, Y_EL0 + S_A1, Y_EL0 + S_B0, Y_EL0 + S_B1   # 85.5, 88.7, 47.5, 50.5
    xw0, xw1 = X_EL + 14.2, X_EL + 18.2                                           # web below the block corners (r 13.6)
    await P.box(xw0, xw1, yp0, ya1, -STRIP, STRIP, why="web under the elbow block", ax="x")
    for y0, y1, horn, pin in ((ya0, ya1, True, False), (yp0, yp1, False, True)):
        b, c = cheek_shapes((0.0, X_EL), (0, 1), xw1 - X_EL, horn=horn, pin=pin)
        await P.prism("y", y0, y1, b, why=f"elbow cheek y {y0:.1f}..{y1:.1f}")
        await P.prism("y", y0, y1, c, cut=True, why="horn pocket" if horn else "pin hole D5.9")
    await P.box(xw1 - 1.0, 36.0, FIST_Y - 6.0, FIST_Y + 6.0, -STRIP, STRIP, why="stem", ax="x")
    await P.box(32.0, 36.0, FIST_Y - 9.0, FIST_Y + 9.0, -STRIP, STRIP, why="barb flange for the TPU fist (barbs in y only: flat part)", ax="x")
    return await save(PARTS / f"{name}.SLDPRT", shots=("isometric", "front"))


async def part_fist():
    await new_part("X3_Fist")
    P = Part(1)
    await P.box(22.0, 56.0, FIST_Y - 16.0, FIST_Y + 16.0, -16.0, 16.0, why="fist 34 x 32 x 32 (TPU)", ax="x")
    await P.box(22.0, 32.0, FIST_Y - 6.2, FIST_Y + 6.2, -STRIP - 0.2, STRIP + 0.2, cut=True, why="stem slot", ax="x")
    await P.box(32.0, 36.4, FIST_Y - 9.2, FIST_Y + 9.2, -STRIP - 0.2, STRIP + 0.2, cut=True, why="barb cavity", ax="x")
    return await save(PARTS / "X3_Fist.SLDPRT", shots=("isometric",))


async def part_standoff():
    await new_part("X3_Standoff")
    # printed with the flat top face on the bed (peg up); the board is held by a self-tapping screw from the front
    await prism("z", 0.0, 5.0, [C(0, 0, 3.0)], why="board standoff D6 x 5")
    await prism("z", -2.0, 0.0, [C(0, 0, 1.4)], why="peg D2.8 into the torso front wall")
    await prism("z", 1.0, 5.0, [C(0, 0, 0.9)], cut=True, why="pilot hole D1.8 for the board screw (board hole size: verify)")
    return await save(PARTS / "X3_Standoff.SLDPRT", shots=("isometric",))


async def part_testblock():
    """fit coupon for the first print (2026-11-15): ONE servo block exactly as in every joint - pocket 23.2 x 13.0,
    tab rim, 2 peg holes D3 with keyways, pin hole D6.3, cable slot. Shaft along +z = pocket up on the bed.
    Checked with it: servo fit, tab seat, lid pegs through the servo tab holes, pin in the bushing hole."""
    await new_part("X3_TestBlock")
    P = Part(1)
    s = Srv((AXL, 0.0, 0.0), "+z", "+x")
    await s.body(P, "servo block (the universal module)")
    await s.pocket(P)
    return await save(PARTS / "X3_TestBlock.SLDPRT", shots=("isometric",))


BATT = (BAY_X0 + 0.6, BAY_X0 + 20.6, -38.5, 38.5, BAY_Z0 + 0.5, BAY_Z0 + 41.5)   # holder 2x18650 ~77 x 41 x 20 (verify)
BOARD_Z = BAY_Z1 + BAY_W + 5.0                                                     # board back face on the standoffs


async def part_battery():
    """mock-up of the purchased 2x18650 holder with cells (mass in the physics copy: 104 g, verify)"""
    await new_part("X3_Battery")
    await prism("x", BATT[0], BATT[1], [R(BATT[2], BATT[4], BATT[3], BATT[5])], why="2x18650 holder 20 x 77 x 41")
    return await save(PARTS / "X3_Battery.SLDPRT", shots=("isometric",), stl=True)


async def part_board():
    """mock-up of the RTrobot 32ch controller: board 45 x 64 x 1.6 (manual hw3 p.32) + 10 mm component envelope"""
    await new_part("X3_Board")
    xc = (PCB_X[0] + PCB_X[1]) / 2
    await prism("z", BOARD_Z, BOARD_Z + 1.6, [R(xc - 22.5, -32.0, xc + 22.5, 32.0)], why="board 45 x 64 x 1.6")
    await prism("z", BOARD_Z + 1.6, BOARD_Z + 11.6, [R(xc - 17.0, -27.0, xc + 17.0, 27.0)], why="component envelope 10 mm")
    return await save(PARTS / "X3_Board.SLDPRT", shots=("isometric",), stl=True)


async def part_horn():
    """purchased single-arm horn of the MG90S, MEASURED 2026-11-15 and TRIMMED by the user to HORN_CUT from the axis,
    in the horn frame: X = arm, Y = shaft axis (s), origin on the shaft axis at s = 0 - like X3_Cap"""
    await new_part("X3_Horn")
    s_hub = HORN_S1 - HORN_H                                                       # 16.95: cup 2.7 deep below the plate
    await prism("y", s_hub, HORN_S1, [C(0, 0, 6.8 / 2)], why="hub D6.8, overall height 4.7")
    e = HORN_CUT
    arm = G([(-horn_hw(0.0), 0.0), (-horn_hw(e), e), (horn_hw(e), e), (horn_hw(0.0), 0.0)])          # (z, x): along +x
    await prism("y", HORN_S0, HORN_S1, [arm], why="trimmed wedge arm to r 11.8, plate 2.0")
    await prism("y", s_hub, HORN_S0, [C(0, 0, 2.45)], cut=True, why="spline cup D4.9, 2.7 deep")
    return await save(PARTS / "X3_Horn.SLDPRT", material="ABS", shots=("isometric",), stl=True)


# print orientation: the model direction that points UP from the bed (checked by v3_dfm.py)
PRINT = {"Lid": "-y", "Pin": "+y", "Cap": "-y", "HornTest": "+y", "Standoff": "-z",
         "Block_L": "+y", "Block_R": "-y", "Thigh": "+z", "Shank_L": "-z", "Shank_R": "-z",
         "Foot_L": "-x", "Foot_R": "-x", "Yoke_L": "+y", "Yoke_R": "-y", "Torso": "-z",
         "Pod_L": "+z", "Pod_R": "+z", "Shoulder_L": "-z", "Shoulder_R": "-z",
         "UpperArm_L": "-y", "UpperArm_R": "+y", "Forearm_L": "+z", "Forearm_R": "+z", "Fist": "+x"}


BUILD = {"lid": part_lid, "pin": part_pin, "cap": part_cap, "yoke": lambda: part_yoke(1), "yoke_r": lambda: part_yoke(-1),
         "block": lambda: part_block(1), "block_r": lambda: part_block(-1),
         "thigh": part_thigh, "shank": lambda: part_shank(1), "shank_r": lambda: part_shank(-1),
         "foot": lambda: part_foot(1), "foot_r": lambda: part_foot(-1),
         "torso": part_torso, "pod": lambda: part_pod(1), "pod_r": lambda: part_pod(-1),
         "shoulder": lambda: part_shoulder(1), "shoulder_r": lambda: part_shoulder(-1),
         "upperarm": lambda: part_upperarm(1), "upperarm_r": lambda: part_upperarm(-1),
         "forearm": lambda: part_forearm(1), "forearm_r": lambda: part_forearm(-1), "fist": part_fist,
         "standoff": part_standoff, "horn": part_horn, "battery": part_battery, "board": part_board,
         "testblock": part_testblock, "horntest": part_horntest}
PURCHASED = ("horn", "battery", "board")           # not printed: no wall / DFM rules


async def main(names):
    info_f = V3 / "parts_info.json"
    info = json.loads(info_f.read_text()) if info_f.exists() else {}
    async with session():
        for n in names:
            _cur["name"] = n
            info[n] = await BUILD[n]()
            info_f.write_text(json.dumps(info, indent=1))
    if THIN:
        print("THIN WALLS:", THIN, flush=True)


async def dry(names):
    """design-rule pass without SolidWorks: run the part functions with the CAD calls stubbed"""
    async def _prism(*a, **k):
        return 0.0

    async def _none(*a, **k):
        return {}
    g = globals()
    g["prism"], g["new_part"], g["save"] = _prism, _none, _none
    for n in names:
        _cur["name"] = n
        await BUILD[n]()
    print(f"dry run: {len(names)} parts, thin walls: {len(THIN)}")
    for t in THIN:
        print("  ", t)


if __name__ == "__main__":
    a = sys.argv[1:]
    is_dry = "--dry" in a
    a = [x for x in a if x != "--dry"]
    names = list(BUILD) if a in ([], ["all"]) else a
    asyncio.run(dry(names) if is_dry else main(names))
