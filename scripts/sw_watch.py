"""Watchdog for every SolidWorks job (2026-11-15, user: "why don't you track SolidWorks messages yourself?").

    python sw_watch.py [--mem 6000] [--minutes 9] -- <command> [args...]   run the job under the watchdog
    python sw_watch.py [--mem 6000] [--minutes 9]                          watch only (a job started elsewhere)

Every 5 s while the job runs:
  * SLDEXITAPP ("SOLIDWORKS Error Report") or SLDWORKS gone  -> the job is stopped, the report window closed, rc 3
  * SLDWORKS private memory above --mem MB                    -> the job is stopped BEFORE Windows runs out, rc 4
  * modal dialogs of SLDWORKS (every 15 s)                    -> text logged, the safe button pressed (sw_dialogs)
  * SLDWORKS window not responding for 120 s                  -> logged (Motion solver is busy but still answers)
The last line is always `WATCH: <verdict>` so the caller reads one line. Events also go to cad/v3/logs/sw_watch.log.
"""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

import psutil

LOG = Path(__file__).resolve().parents[1] / "cad" / "v3" / "logs" / "sw_watch.log"


def say(msg):
    line = f"{time.strftime('%H:%M:%S')} {msg}"
    print("  [watch] " + line, flush=True)
    with open(LOG, "a", encoding="utf-8") as f:
        f.write(line + "\n")


def procs(name):
    return [p for p in psutil.process_iter(["name"]) if (p.info["name"] or "").lower() == name]


def sw_mem():
    m = 0
    for p in procs("sldworks.exe"):
        try:
            mi = p.memory_info()
            m = max(m, getattr(mi, "private", mi.rss) // 2 ** 20)
        except psutil.Error:
            pass
    return m


def kill_tree(p):
    try:
        for c in psutil.Process(p.pid).children(recursive=True):
            c.kill()
        p.kill()
    except (psutil.Error, OSError):
        pass


def main():
    a = sys.argv[1:]
    cmd = a[a.index("--") + 1:] if "--" in a else None
    opt = a[:a.index("--")] if "--" in a else a
    mem = int(opt[opt.index("--mem") + 1]) if "--mem" in opt else 6000
    minutes = float(opt[opt.index("--minutes") + 1]) if "--minutes" in opt else 9.0
    job = subprocess.Popen(cmd) if cmd else None
    t0, last_dlg, peak, verdict, rc = time.time(), 0.0, 0, None, 0
    seen_sw = bool(procs("sldworks.exe"))
    reports, gone_t = 0, None
    while True:
        if job is not None and job.poll() is not None:
            verdict, rc = f"job finished rc={job.returncode}", job.returncode
            break
        if job is None and time.time() - t0 > minutes * 60:
            verdict = "watch window over, no events"
            break
        if procs("sldexitapp.exe"):
            # appears after a crash AND after a restart of a bloated instance: close it, count it, keep watching -
            # a real crash shows up as SLDWORKS missing (below) or as the job failing on COM
            reports += 1
            say("'SOLIDWORKS Error Report' window (SLDEXITAPP) - closed")
            for p in procs("sldexitapp.exe"):
                try:
                    p.kill()
                except psutil.Error:
                    pass
        sw = procs("sldworks.exe")
        seen_sw = seen_sw or bool(sw)
        gone_t = None if sw else (gone_t or time.time())
        # a queue restarts SolidWorks between runs (first watchdog version stopped the queue on that restart)
        if seen_sw and gone_t and time.time() - gone_t > 150:
            say("SLDWORKS process is gone for 150 s")
            verdict, rc = "SOLIDWORKS EXITED", 3
            break
        m = sw_mem()
        peak = max(peak, m)
        if m > mem:
            say(f"SLDWORKS private memory {m} MB > {mem} MB - stopping the job before Windows runs out")
            verdict, rc = f"MEMORY LIMIT ({m} MB)", 4
            break
        if time.time() - last_dlg > 15:
            last_dlg = time.time()
            try:
                import sw_dialogs
                for d, text, btns in sw_dialogs.dialogs():
                    say(f"dialog: {text[:200]!r} buttons {[b.window_text() for b in btns]}")
                if sw_dialogs.dialogs():
                    say(f"dismissed {sw_dialogs.dismiss()}")
            except Exception as e:                                # noqa: BLE001
                say(f"dialog check failed: {str(e)[:120]}")
        time.sleep(5)
    if rc in (3, 4) and job is not None:
        kill_tree(job)
    print(f"WATCH: {verdict}; SLDWORKS peak {peak} MB, error reports closed {reports}, {time.time() - t0:.0f} s", flush=True)
    return rc


if __name__ == "__main__":
    sys.exit(main())
