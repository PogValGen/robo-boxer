"""SolidWorks memory guard for the physics runs: start SW if it is not running; restart it
gracefully (ExitApp, never kill) when it is idle (0 documents) and over the harness limits.
Usage: python sw_guard.py [pre|post]
"""
import sys
from pathlib import Path

sys.path.insert(0, r"D:\AI prodject\solidworks-mcp\SolidworksMCP-python\scripts")
from audit_harness import start_solidworks, sw_health_guard  # noqa: E402

import psutil  # noqa: E402

if __name__ == "__main__":
    running = any((p.info["name"] or "").lower() == "sldworks.exe" for p in psutil.process_iter(["name"]))
    phase = sys.argv[1] if len(sys.argv) > 1 else "pre"
    if not running:
        print("[guard] SolidWorks not running -", start_solidworks() if phase == "pre" else "left closed")
    else:
        print("[guard]", sw_health_guard())
    vm = psutil.virtual_memory()
    print(f"[guard] RAM free {vm.available / 2**20:.0f} MB of {vm.total / 2**20:.0f} MB")
