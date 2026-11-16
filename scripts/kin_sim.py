"""Numeric FK check of the walk+punch animation (same model + angle formulas as the HTML).
Prints world positions of fist/foot/stator to confirm the motion directions are correct."""
from __future__ import annotations
import json, math, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, r"C:\Users\40E4~1\AppData\Local\Temp\claude\C--WINDOWS-system32\f6af84ef-89e2-4e17-8f15-b33bc6f6b67e\scratchpad")
import build_anim as BA

d = json.loads(BA.SRC.read_text(encoding="utf-8"))
links, classify, order, units = BA.build_links(d)

# topo order (parent first)
roleOrder, seen = [], set()
def add(r):
    if r in seen: return
    p = links[r]["parent"]
    if p: add(p)
    seen.add(r); roleOrder.append(r)
for r in links: add(r)

P = 1.15; T_WALK = P*3; T_SET = 0.5; T_PNCH = 1.4; T_END = 0.7
LOOP = T_WALK+T_SET+T_PNCH+T_END

def ease(a,b,u):
    u=max(0,min(1,u)); u=u*u*(3-2*u); return a+(b-a)*u

def gait(kind,side,t):
    ph=2*math.pi*t/P+(math.pi if side=='B' else 0); s=math.sin(ph); lift=max(0,math.sin(ph))
    if kind=='hippitch': return 0.50*s
    if kind=='knee':     return -1.35*lift
    if kind=='ankle':    return -0.50*s+0.55*lift
    if kind=='hiproll':  return 0.05*(1 if side=='A' else -1)*math.sin(2*math.pi*t/P)
    return 0.0

def bodyMat(t):
    w=1.0 if (t%LOOP)<T_WALK else 0.0
    up=-4*abs(math.sin(2*math.pi*t/P))*w; sway=7*math.sin(2*math.pi*t/P)*w
    M=np.eye(4); M[:3,3]=[up,sway,0]; return M

def guardArm(kind,side):
    if kind=='shoulder': return 1.15
    if kind=='shoulderroll': return (1 if side=='A' else -1)*0.25
    if kind=='elbow': return 1.5
    return 0.0

def angles(role,t):
    L=links[role]; kind=L.get("kind")
    if not kind: return 0.0
    side=L["side"]; t=t%LOOP
    if t<T_WALK:
        if kind=='waist': return 0.12*math.sin(2*math.pi*t/P)
        if kind.startswith('shoulder') or kind=='elbow':
            if kind=='shoulder': return 0.9+0.15*math.sin(2*math.pi*t/P+(math.pi if side=='A' else 0))
            if kind=='elbow': return 1.2
            return (1 if side=='A' else -1)*0.2
        return gait(kind,side,t)
    if t<T_WALK+T_SET:
        u1=(t-T_WALK)/T_SET
        if kind=='waist': return ease(0.12*math.sin(2*math.pi*T_WALK/P),0,u1)
        if kind.startswith('shoulder') or kind=='elbow':
            base=0.9 if kind=='shoulder' else (1.2 if kind=='elbow' else (0.2 if side=='A' else -0.2))
            return ease(base,guardArm(kind,side),u1)
        return ease(gait(kind,side,T_WALK),0,u1)
    tp=t-T_WALK-T_SET
    if tp<T_PNCH:
        u=tp/T_PNCH; strike=math.sin(math.pi*u)
        if side=='A':
            if kind=='shoulder': return ease(guardArm('shoulder','A'),0.15,strike)
            if kind=='elbow': return ease(guardArm('elbow','A'),0.15,strike)
            if kind=='shoulderroll': return guardArm('shoulderroll','A')
        else:
            if kind.startswith('shoulder') or kind=='elbow': return guardArm(kind,'B')
        if kind=='waist': return 0.35*strike
        return 0.0
    u2=(tp-T_PNCH)/T_END
    if kind=='waist': return 0.0
    if kind.startswith('shoulder') or kind=='elbow': return guardArm(kind,side)
    return 0.0

def rot_axis(axis,ang):
    a=np.array(axis,float); a=a/np.linalg.norm(a); x,y,z=a; c=math.cos(ang); s=math.sin(ang); C=1-c
    return np.array([[c+x*x*C, x*y*C-z*s, x*z*C+y*s],
                     [y*x*C+z*s, c+y*y*C, y*z*C-x*s],
                     [z*x*C-y*s, z*y*C+x*s, c+z*z*C]])

def about(axis,pivot,ang):
    M=np.eye(4); M[:3,:3]=rot_axis(axis,ang)
    piv=np.array(pivot,float)
    M[:3,3]=piv-M[:3,:3]@piv
    return M

def fk(t):
    Lm={}
    for r in roleOrder:
        L=links[r]
        if L["parent"] is None: Lm[r]=bodyMat(t); continue
        par=Lm[L["parent"]]
        ax=par[:3,:3]@np.array(L["axis"],float); ax=ax/np.linalg.norm(ax)
        pv=(par@np.array(list(L["pivot"])+[1.0]))[:3]
        Lm[r]=about(ax,pv,angles(r,t))@par
    return Lm

# track points: part name -> (link, neutral origin mm)
pt={}
for it in d["inst"]:
    nm=it["name"]; stem=nm.split("/")[-1].rsplit("-",1)[0]
    org=np.array(it["m"][9:12])*1000.0
    pt[nm]=(classify(nm),org)

def wp(name,Lm):
    link,org=pt[name]
    return (Lm[link]@np.append(org,1))[:3]

# find key parts
fistA=[n for n in pt if n.startswith("Arm-1/Fist")][0]
footA=[n for n in pt if "Leg_Pitch" in n and n.endswith("Leg_Foot-1") and "Leg_Pitch-1" in n][0]
footB=[n for n in pt if n.endswith("Leg_Foot-1") and "Leg_Pitch-2" in n][0]
shoA_housing=[n for n in pt if n.startswith("Arm-1/Joint_MG90S-1/Joint_Housing")][0]
shoulderA_piv=np.array(links["armA_shoulder"]["pivot"],float)

print("fistA part:",fistA,"link",pt[fistA][0])
print("footA part:",footA,"link",pt[footA][0])

def rep(label,t):
    Lm=fk(t)
    f=wp(fistA,Lm); fa=wp(footA,Lm); fb=wp(footB,Lm); sh=wp(shoA_housing,Lm)
    armlen=np.linalg.norm(f-shoulderA_piv)
    print(f"[{label} t={t:.2f}] fistA=({f[0]:6.0f},{f[1]:6.0f},{f[2]:6.0f}) armlen={armlen:5.0f} | "
          f"footA=({fa[0]:6.0f},{fa[1]:5.0f},{fa[2]:6.0f}) footB_z={fb[2]:6.0f} | shoHousing=({sh[0]:5.0f},{sh[1]:5.0f},{sh[2]:5.0f})")

print("\n-- neutral --"); rep("neutral",1e9)  # huge t%LOOP ~ ... use 0 instead
rep("t0",0.0)
print("\n-- walk (two antiphase phases) --")
rep("walkA",0.10); rep("walkB",0.10+P/2)
print("\n-- guard & punch --")
rep("guard",T_WALK+T_SET)
rep("punch_peak",T_WALK+T_SET+T_PNCH*0.50)  # strike=1 here
rep("punch_end",T_WALK+T_SET+T_PNCH)
print("\n-- walk foot lift sweep (leg A), x=down (smaller=higher) --")
for tt in [round(0.02+ i*P/8,3) for i in range(9)]:
    Lm=fk(tt); fa=wp(footA,Lm)
    print(f"  t={tt:.2f} footA x={fa[0]:6.1f} z={fa[2]:6.1f}")
print("\nshoulderA pivot (neutral):",shoulderA_piv.round(0))
