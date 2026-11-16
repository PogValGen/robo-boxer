"""Quasi-static design of get-up / kick motions with wb_fk (no SolidWorks), 2026-11-13.

check(keys, down0): keys = [(t, {role: deg}), ...] (unlisted roles = 0 / previous); interpolates joint-space
in steps, at each step the robot settles on the stable hull facet nearest to the previous floor normal
(slow-motion assumption). Prints contact parts, COM margin and the tilt of the robot's long axis.
Final success = only Leg_Foot parts touch and robot x (down) ~ gravity.
"""
from __future__ import annotations
import math
import numpy as np
import wb_fk

B = None


def body():
    global B
    if B is None:
        B = wb_fk.Body(cell=4.0)
    return B


def full(q, prev=None):
    out = dict(prev or {})
    out.update(q)
    return out


def check(keys, down0, steps=15, verbose=True):
    b = body()
    down = np.array(down0, float)
    prev = {}
    log = []
    q_last = {}
    for ki, (t, q) in enumerate(keys):
        q = full(q, q_last)
        for s in range(1, steps + 1) if ki else [steps]:
            u = s / steps
            qi = {r: q_last.get(r, 0.0) + (q.get(r, 0.0) - q_last.get(r, 0.0)) * u for r in set(q) | set(q_last)} if ki else q
            r = b.rest(qi, down, max_turn=35)
            if r is None:                                  # tips over: take the nearest stable facet anywhere
                r = b.rest(qi, down)
                if r is None:
                    log.append((t, "NO STABLE FACET")); return False, log
                if verbose:
                    print(f"     (tips over at t~{t}, step {s}/{steps})")
            down = r["n"]
        q_last = q
        up_dot = float(np.array([1.0, 0, 0]) @ down)            # robot x vs gravity: 1 = upright
        short = sorted({p.rsplit("-", 1)[0] + "-" + p.rsplit("-", 1)[1] for p in r["parts"]})
        log.append((t, f"margin {r['margin']:5.1f}  h {r['h']:5.1f}  upright {up_dot:+.2f}  on {short}"))
        if verbose:
            print(f"  t={t:5.2f}  {log[-1][1]}")
    ok = all(p.startswith("Leg_Foot") for p in r["parts"]) and up_dot > 0.95
    return ok, log


if __name__ == "__main__":
    print("probe: lying on chest, arms forward to the floor")
    check([(0, {}), (1, {"armA_shoulder": -90, "armB_shoulder": -90})], [0, 0, 1])
