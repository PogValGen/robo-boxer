"""Unattended queue of servo_sim.py runs (2026-11-13): no Claude turn between runs.

For every line of the queue file (servo_sim.py arguments, e.g. `dyn --v6 --tag _d46 --param tss=0.5 --param S=30`):
  1. close own SW docs + exit SW (sw_docs.py exit), start a fresh SW (audit_harness.start_solidworks)
     -> memory/handles stay low (SW grows to 1.5 GB / 290k handles per run)
  2. run servo_sim.py (ROBO_V=2), log -> cad/v2/logs/q_<tag>.log
  3. sim_report.py summary -> one row in cad/v2/logs/queue_results.md (+ .csv)
A failed run is recorded and the queue goes on. Runs whose summary.json already exists are skipped (--force reruns).
Lines starting with # are comments.

Usage (venv solidworks-mcp, from scripts/):
    python sim_queue.py queue.txt [--force] [--keep]     (--keep: do not restart SW before the first run)
"""
from __future__ import annotations
import csv, json, os, subprocess, sys, time
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJ = HERE.parent
VER = os.environ.get("ROBO_V", "2")              # v3: set ROBO_V=3 before the queue (2026-11-15)
PHYS = PROJ / "cad" / f"v{VER}" / "physics"
LOGS = PROJ / "cad" / f"v{VER}" / "logs"
PY = sys.executable
HARNESS = Path(r"D:\AI prodject\solidworks-mcp\SolidworksMCP-python\scripts")
ENV = {**os.environ, "ROBO_V": VER, "PYTHONIOENCODING": "utf-8"}


def label_of(args):
    """servo_sim label: scenario [+_fixed] [+_6v] + tag (same rule as servo_sim.main)."""
    a = args.split(); scn = a[0]
    tag = a[a.index("--tag") + 1] if "--tag" in a else ""
    fixed = "--fixed" in a or scn == "calib"
    return scn + ("_fixed" if fixed and scn != "calib" else "") + ("_6v" if "--v6" in a else "") + tag


def sw_answers(timeout=120):
    """True if SW answers COM (sw_docs.py lists documents) within timeout s."""
    try:
        r = subprocess.run([PY, str(HERE / "sw_docs.py")], cwd=HERE, env=ENV, capture_output=True, text=True, timeout=timeout)
        return "open docs" in (r.stdout or "")
    except subprocess.TimeoutExpired:
        return False


def sw_running():
    r = subprocess.run(["tasklist", "/FI", "IMAGENAME eq SLDWORKS.exe", "/NH"], capture_output=True, text=True)
    return "SLDWORKS.exe" in r.stdout


def dismiss_dialogs():
    """Press OK on hidden startup dialogs (sw_dialogs.py); returns its report lines."""
    try:
        r = subprocess.run([PY, str(HERE / "sw_dialogs.py"), "dismiss"], cwd=HERE, env=ENV, capture_output=True, text=True, timeout=90)
        return [l for l in r.stdout.splitlines() if l.startswith("dialog:")]
    except subprocess.TimeoutExpired:
        return []


def restart_sw():
    """Exit SW GRACEFULLY (wait up to 90 s for the process to end - killing it right after ExitApp corrupts the toolbar
    settings and the next start hangs on a hidden 'Toolbar information is inconsistent' dialog), kill only if it is
    still there, start a fresh one, press OK on startup dialogs and wait until SW answers COM."""
    try:
        subprocess.run([PY, str(HERE / "sw_docs.py"), "exit"], cwd=HERE, env=ENV, capture_output=True, timeout=120)
    except subprocess.TimeoutExpired:
        pass
    t0 = time.time()
    while sw_running() and time.time() - t0 < 90:
        time.sleep(3)
    killed = sw_running()
    if killed:
        subprocess.run(["taskkill", "/F", "/IM", "SLDWORKS.exe"], capture_output=True)
        time.sleep(5)
    t0 = time.time()
    exe = subprocess.run([PY, "-c", "import audit_harness as h; print(h.SLDWORKS_EXE)"], cwd=HARNESS, env=ENV,
                         capture_output=True, text=True).stdout.strip().splitlines()[-1]
    subprocess.Popen([exe])
    notes = []
    if "SLDEXITAPP" in subprocess.run(["tasklist", "/FI", "IMAGENAME eq SLDEXITAPP.exe", "/NH"], capture_output=True, text=True).stdout:
        subprocess.run(["taskkill", "/F", "/IM", "SLDEXITAPP.exe"], capture_output=True)   # "SOLIDWORKS Error Report" of the previous instance
        notes.append("closed SOLIDWORKS Error Report (crash on exit of the previous SW)")
    while time.time() - t0 < 900:
        time.sleep(10)
        notes += dismiss_dialogs()
        if sw_answers(40):
            return f"ready in {time.time() - t0:.0f} s" + (" (killed)" if killed else "") + ("; " + "; ".join(notes) if notes else "")
    raise RuntimeError("SW does not answer 15 min after start; " + "; ".join(notes))


def metrics(label):
    s = json.loads((PHYS / f"servo_{label}_summary.json").read_text(encoding="utf-8"))
    r = json.loads((PHYS / f"servo_{label}_result.json").read_text(encoding="utf-8"))
    import numpy as np
    row = {"label": label, "solver_s": round(r.get("seconds", 0)), "params": json.dumps(s.get("params", {}), ensure_ascii=False)}
    b = s.get("base", {})
    row.update(fwd=round(b.get("dz_fwd_mm", 0), 1), side=round(b.get("dy_side_mm", 0), 1),
               tilt=round(b.get("max_tilt_deg", 0), 1), fell=b.get("fell"))
    if "upright_end" in b:                                   # get-up runs: 1 = standing at the end
        row["upright"] = round(b["upright_end"], 2)
        row["rise"] = round(b["upright"][-1][2], 1)
    worst = []
    for name, sv in r["servos"].items():
        rows = np.array(sv["rows"]); stall = sv.get("stall_nm", 0.2157)
        p = np.abs(rows[:, 4]) / stall * 100
        worst.append((float(np.percentile(p, 95)), float(np.percentile(p, 99)), name[2:]))
    worst.sort(reverse=True)
    row["worst_p95"] = f"{worst[0][2]} {worst[0][0]:.0f}%"
    row["worst_p99"] = f"{max(w[1] for w in worst):.0f}%"
    return row


def append(row):
    md = LOGS / "queue_results.md"; cv = LOGS / "queue_results.csv"
    cols = ["time", "label", "status", "fwd", "side", "tilt", "fell", "upright", "rise", "worst_p95", "worst_p99", "solver_s", "params"]
    new = not md.exists()
    with md.open("a", encoding="utf-8") as f:
        if new:
            f.write("| " + " | ".join(cols) + " |\n|" + "---|" * len(cols) + "\n")
        f.write("| " + " | ".join(str(row.get(c, "")) for c in cols) + " |\n")
    new = not cv.exists()
    with cv.open("a", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, cols, extrasaction="ignore")
        if new:
            w.writeheader()
        w.writerow(row)


def main():
    qf = Path(sys.argv[1]); force = "--force" in sys.argv
    keep = "--keep" in sys.argv; first = None
    jobs = [l.strip() for l in qf.read_text(encoding="utf-8").splitlines() if l.strip() and not l.strip().startswith("#")]
    t0 = time.time()
    for i, args in enumerate(jobs, 1):
        label = label_of(args)
        bar = "█" * int(10 * (i - 1) / len(jobs)) + "░" * (10 - int(10 * (i - 1) / len(jobs)))
        print(f"[{bar}] {100 * (i - 1) // len(jobs)}% — прогон {i}/{len(jobs)}: {label}  ({time.time() - t0:.0f} с)", flush=True)
        if not force and (PHYS / f"servo_{label}_summary.json").exists():
            print("   уже есть summary -> пропуск", flush=True); continue
        first = i if first is None else first
        row = {"time": datetime.now().strftime("%m-%d %H:%M"), "label": label}
        try:
            if keep and i == first and sw_answers(60):          # --keep: first run on the SW that is already up
                print("   SW: уже запущен, без перезапуска", flush=True)
            else:
                print("   SW:", restart_sw(), flush=True)
            log = LOGS / f"q{label}.log"
            t1 = time.time()
            with log.open("w", encoding="utf-8") as f:
                rc = subprocess.run([PY, "-u", str(HERE / "servo_sim.py"), *args.split()], cwd=HERE, env=ENV,
                                    stdout=f, stderr=subprocess.STDOUT, timeout=3600).returncode
            if (PHYS / f"servo_{label}_summary.json").exists():
                row.update(metrics(label)); row["status"] = "ok" if rc == 0 else f"ok(rc={rc})"
            else:
                row["status"] = f"FAIL rc={rc}, см. {log.name}"
            print(f"   {row.get('status')}  fwd {row.get('fwd')}  side {row.get('side')}  fell {row.get('fell')}  "
                  f"worst {row.get('worst_p95')} / p99 {row.get('worst_p99')}  ({time.time() - t1:.0f} с)", flush=True)
        except Exception as e:                                  # one bad run must not stop the queue
            row["status"] = f"FAIL {type(e).__name__}: {str(e)[:120]}"
            print("  ", row["status"], flush=True)
        append(row)
    print(f"[██████████] 100% — {len(jobs)} прогонов за {(time.time() - t0) / 60:.1f} мин -> {LOGS / 'queue_results.md'}", flush=True)


if __name__ == "__main__":
    main()
