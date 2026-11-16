"""List open SW docs; with 'exit' close own (robo-boxer) docs without saving and ExitApp if nothing else is open."""
import sys
import win32com.client


def val(o, name):
    try:
        return getattr(o, name)()
    except Exception:
        return getattr(o, name)


app = win32com.client.GetActiveObject("SldWorks.Application")
docs = []
for d in (val(app, "GetDocuments") or []):
    docs.append((val(d, "GetPathName"), val(d, "GetTitle")))
print("open docs:", len(docs))
for p, t in docs:
    print("  ", t, "|", p)
if len(sys.argv) > 1 and sys.argv[1] == "exit":
    own = [x for x in docs if "robo-boxer" in (x[0] or "") or not x[0]]
    if len(own) != len(docs):
        print("foreign docs open -> not exiting"); sys.exit(1)
    for p, t in own:
        app.CloseDoc(t)
    app.ExitApp()
    print("ExitApp sent")
