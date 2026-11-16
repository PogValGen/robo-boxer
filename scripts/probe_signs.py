"""Print where fists/feet/COM go for +-60 deg on each joint (sign conventions for motion design)."""
import numpy as np
import wb_fk

B = wb_fk.Body()


def show(q, tag):
    f = {n: np.round(c, 0) for n, c in B.where(q, "Fist_TPU")}
    ft = {n: np.round(c, 0) for n, c in B.where(q, "Leg_Foot")}
    print(f"{tag:28s} fist1 {f['Fist_TPU-1']} fist2 {f['Fist_TPU-2']} foot1 {ft['Leg_Foot-1']} foot2 {ft['Leg_Foot-2']} COM {np.round(B.com(q), 0)}")


show({}, "zero")
for r in ["armA_shoulder", "armA_shoulderroll", "armA_elbow", "torso", "legA_hiproll", "legA_hippitch",
          "legA_knee", "legA_ankle", "legA_ankleroll"]:
    for a in (-60, 60):
        show({r: a}, f"{r} {a:+d}")
print([(j["role"], j["stator"]) for j in B.spec["joints"]])
print("fist links", [(p["name"], p["link"]) for p in B.parts if p["stem"] == "Fist_TPU"])
print("foot links", [(p["name"], p["link"]) for p in B.parts if p["stem"] == "Leg_Foot"])
