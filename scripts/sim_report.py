"""Robust report for a servo_sim run: torque percentiles (filtered 25 ms median), windows of overload,
pelvis trajectory. Usage: python sim_report.py <label> [<label> ...]
  label = part of file name after 'servo_', e.g. walk2_6v_g80  ->  cad/v2/physics/servo_walk2_6v_g80_{result,summary}.json
Peak alone lies (contact-solver bursts > 25 ms); judge by p95/p99 and the time windows of >150% stall."""
import json
import os
import sys
from pathlib import Path

import numpy as np

VER = os.environ.get("ROBO_V", "2")               # same default as sim_queue.py; v3: ROBO_V=3
PHYS = Path(__file__).resolve().parents[1] / "cad" / f"v{VER}" / "physics"


def report(label):
    r = json.load(open(PHYS / f"servo_{label}_result.json"))
    s = json.load(open(PHYS / f"servo_{label}_summary.json"))
    print(f"== {label}  params {s.get('params')}")
    worst = []
    for name, sv in r["servos"].items():
        rows = np.array(sv["rows"]); st = sv["stall_nm"]
        tf = np.abs(rows[:, 4]); t = rows[:, 0]
        over = t[tf > 1.5 * st]
        p95, p99 = np.percentile(tf, 95) / st * 100, np.percentile(tf, 99) / st * 100
        worst.append((p95, name[2:]))
        win = f"{over.min():5.2f}..{over.max():5.2f} s, {len(over) * 0.01:.2f} s total" if len(over) else "-"
        print(f"  {name[2:]:<18} p95 {p95:5.0f}%  p99 {p99:7.0f}%  >150%: {win}")
    b = s["base"]
    tr = np.array(b["traj"])
    print(f"  pelvis end: fwd {b['dz_fwd_mm']:+.1f}  side {b['dy_side_mm']:+.1f}  down {b['dx_down_mm']:+.1f} mm, "
          f"tilt max {b['max_tilt_deg']:.1f}, yaw {b['end_yaw_deg']:+.1f} -> {'FELL' if b['fell'] else 'standing'}")
    print("  t, fwd, side, down, tilt  (every 0.5 s):")
    for row in tr[::50]:
        print("   ", " ".join(f"{v:7.1f}" for v in row))
    worst.sort(reverse=True)
    print("  worst p95:", ", ".join(f"{n} {p:.0f}%" for p, n in worst[:4]))


if __name__ == "__main__":
    for lab in sys.argv[1:] or ["walk2_6v_g80"]:
        report(lab)
