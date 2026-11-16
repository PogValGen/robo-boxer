"""Startup / modal dialogs of SOLIDWORKS that block COM (2026-11-13).
    python sw_dialogs.py            - list dialogs (#32770) of every SLDWORKS.exe with their text and buttons
    python sw_dialogs.py dismiss    - press the default-safe button: OK / 'No, disable ...' (add-in problem) / Close
A hidden dialog behind the splash keeps SW from answering COM for minutes ('3DEXPERIENCE Marketplace problems last
time', 'Toolbar information is inconsistent'); the text is only visible through UI Automation.
Used by sim_queue.restart_sw while it waits for SW."""
import sys, time
import psutil
import comtypes.client  # noqa: F401
from pywinauto import Desktop

PREFER = ("no, disable", "ok", "close", "закрыть", "нет")


def dialogs():
    pids = {p.pid for p in psutil.process_iter(["name"]) if (p.info["name"] or "").lower() == "sldworks.exe"}
    out = []
    for w in Desktop(backend="uia").windows():
        try:
            if w.process_id() not in pids:
                continue
            for d in [w] + w.descendants(control_type="Window"):
                if d.class_name() != "#32770":
                    continue
                kids = d.descendants()
                text = " | ".join(k.window_text() for k in kids if k.element_info.control_type == "Text" and k.window_text())
                btns = [k for k in kids if k.element_info.control_type in ("Button", "Pane") and k.window_text()]
                out.append((d, text, btns))
        except Exception:  # noqa: BLE001 - window vanished
            pass
    return out


def dismiss():
    n = 0
    for d, text, btns in dialogs():
        names = [b.window_text() for b in btns]
        pick = next((b for p in PREFER for b in btns if b.window_text().lower().replace("&", "").startswith(p)), None)
        print(f"dialog: {text[:160]!r} buttons {names} -> {pick.window_text() if pick else 'NO SAFE BUTTON'}", flush=True)
        if pick:
            try:
                pick.click_input() if not hasattr(pick, "invoke") else pick.invoke()
            except Exception:  # noqa: BLE001
                d.set_focus(); d.type_keys("{ENTER}")
            n += 1; time.sleep(1)
    return n


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "dismiss":
        print("dismissed", dismiss())
    else:
        for d, text, btns in dialogs():
            print(f"dialog: {text!r} buttons {[b.window_text() for b in btns]}")
        print("done")
