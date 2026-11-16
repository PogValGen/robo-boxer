"""Build an animated 3D page (three.js) of the robot WALKING + PUNCHING with correct
forward kinematics: each servo's stator stays on its parent link, only the lever+horn and
everything distal rotate about that servo's output axis (derived from the MG90S transform).

Input: robo-boxer/docs/viewer/robot_3d.json (make_viewer output: meshes + instances w/ full names).
Output: <out.html> self-contained (three r128 from CDN, meshes inline, FK+animation in JS).
Usage: python build_anim.py <out.html>
"""
from __future__ import annotations
import base64, gzip, json, struct, sys
from collections import defaultdict
from pathlib import Path
import numpy as np

PROJ = Path(r"D:\AI prodject\robo-boxer")
sys.path.insert(0, str(PROJ / "scripts"))
from pack_viewer_data import stl_tris, weld, proxy  # noqa: E402

import os  # noqa: E402
V2 = os.environ.get("ROBO_V", "1") == "2"
SRC = PROJ / "docs" / "viewer" / ("robot_v2_3d.json" if V2 else "robot_3d.json")
ROTOR = {"Joint_Lever", "J2_Lever", "J2_LeverTop", "J2_LeverBot", "MG90S_Horn"}
DROP = {"Bearing_685ZZ", "Bearing_623ZZ", "Pin_ISO2338_3x12"}  # hidden internals
SHAFT = [5.5, 16.0, 0.0]  # servo output-shaft point (local), axis = servo local +Y


def rt(m):
    return np.array(m[:9]).reshape(3, 3), np.array(m[9:12]) * 1000.0  # R (row-vec), T mm


def wpt(m, p):
    R, T = rt(m); return np.array(p) @ R + T


def wdir(m, d):
    R, _ = rt(m); v = np.array(d) @ R; return v / np.linalg.norm(v)


def stem_of(name):
    return name.split("/")[-1].rsplit("-", 1)[0]


def unit_of(name):
    segs = name.split("/")
    for i, s in enumerate(segs):
        if s.startswith(("Joint_MG90S-", "Joint2_MG90S-")):
            return "/".join(segs[:i]), "/".join(segs[:i + 1])
    return "/".join(segs[:-1]), None


def build_links(d):
    """Return (links, classify, order, units) from the assembly json."""
    inst = d["inst"]
    # --- group units, find each unit's servo transform, order per limb by x ---
    units = {}  # unit path -> servo m
    for it in inst:
        if stem_of(it["name"]) == "MG90S_grabcad":
            _, u = unit_of(it["name"])
            units[u] = it["m"]
    bylimb = defaultdict(list)  # limb -> [(x, unit)]
    for u, m in units.items():
        limb = u.rsplit("/", 1)[0]
        bylimb[limb].append((wpt(m, SHAFT)[0], u))
    order = {}  # limb -> [unit proximal..distal]
    for limb, lst in bylimb.items():
        order[limb] = [u for _, u in sorted(lst)]

    def limb_kind(limb):
        if "Leg_Pitch" in limb:
            return "leg"
        if limb.startswith("Arm-"):
            return "arm"
        return "waist"

    def side_tag(limb, m):
        y = wpt(m, SHAFT)[1]
        return "A" if y >= 0 else "B"

    # tag each limb -> side, build joint role names
    CHAIN = {"leg": ["hiproll", "hippitch", "knee", "ankle", "ankleroll"],   # v2: 5th = ankle roll
             "arm": ["shoulder", "shoulderroll", "elbow"], "waist": ["waist"]}
    limb_side = {}
    for limb, us in order.items():
        limb_side[limb] = side_tag(limb, units[us[0]])

    def link_role(limb, pos):
        k = limb_kind(limb)
        if k == "waist":
            return "torso"
        return f"{k}{limb_side[limb]}_{CHAIN[k][pos]}"

    # --- link tree: role -> {parent, kind, side, axis, pivot} ---
    links = {"base": {"parent": None}}
    for limb, us in order.items():
        k = limb_kind(limb)
        for pos, u in enumerate(us):
            role = link_role(limb, pos)
            axis = wdir(units[u], [0, 1, 0]).tolist()
            piv = wpt(units[u], SHAFT).tolist()
            if k == "waist":
                parent = "base"
            elif pos == 0:
                parent = "torso" if k == "arm" else "base"
            else:
                parent = link_role(limb, pos - 1)
            links[role] = {"parent": parent, "kind": CHAIN[k][pos], "side": limb_side[limb],
                           "axis": [round(a, 5) for a in axis], "pivot": [round(p, 2) for p in piv]}

    # --- classify each instance to a link role ---
    def classify(name):
        segs = name.split("/"); stem = stem_of(name)
        limb, u = unit_of(name)
        if u is None:  # loose part
            if stem == "Pelvis":
                return "base"
            if stem in ("Torso_Mount", "Shoulder_Yoke"):
                return "torso"
            if stem == "Leg_Foot":
                return link_role(limb, len(order[limb]) - 1)  # ankle rotor
            if stem == "Fist_TPU":
                return link_role(limb, len(order[limb]) - 1)  # elbow rotor
            return "base"
        pos = order[limb].index(u)
        k = limb_kind(limb)
        if stem in ROTOR:
            return link_role(limb, pos)
        if pos == 0:  # stator of first unit -> parent body
            return "torso" if k == "arm" else "base"
        return link_role(limb, pos - 1)

    return links, classify, order, units


def main(out_html: str):
    d = json.loads(SRC.read_text(encoding="utf-8"))
    inst = d["inst"]
    meshes = d["meshes"]
    links, classify, order, units = build_links(d)

    # --- pack geometry (unique parts kept, servos as proxy, internals dropped) ---
    keep_paths = sorted({it["p"] for it in inst if stem_of(it["name"]) not in DROP})
    pidx = {p: i for i, p in enumerate(keep_paths)}
    parts, blobs = [], []
    for p in keep_paths:
        mp = meshes[p]
        tris = stl_tris(gzip.decompress(base64.b64decode(mp["gz"]))) + np.array(mp["off"])
        px = proxy(mp["stem"], tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0))
        if px is not None:
            v, f = px
        else:
            v, f = weld(tris, tol=0.8)  # printed parts: light weld
        lo, hi = v.min(0), v.max(0)
        sc = np.maximum(hi - lo, 1e-6) / 65535.0
        q = np.round((v - lo) / sc).astype(np.uint16)
        assert len(v) < 65536, (mp["stem"], len(v))
        parts.append({"color": mp["color"], "lo": [round(float(x), 2) for x in lo],
                      "sc": [float(f"{x:.5g}") for x in sc], "nv": int(len(v)), "nf": int(len(f))})
        blobs.append(q.tobytes() + f.astype(np.uint16).tobytes())

    # --- instances: [partIdx, m(12), linkRole] ---
    insts = []
    for it in inst:
        if stem_of(it["name"]) in DROP:
            continue
        insts.append({"p": pidx[it["p"]], "m": [round(x, 6) for x in it["m"]], "L": classify(it["name"])})

    head = json.dumps({"parts": parts, "inst": insts, "links": links}, separators=(",", ":")).encode()
    payload = struct.pack("<I", len(head)) + head + b"".join(blobs)
    b64 = base64.b64encode(gzip.compress(payload, 9)).decode()
    html = HTML.replace("__B64__", b64)
    Path(out_html).write_text(html, encoding="utf-8")
    nL = len(links)
    print(f"parts={len(parts)} inst={len(insts)} links={nL} payload={len(payload)/1024:.0f}KB b64={len(b64)/1024:.0f}KB")
    print("link roles:", sorted(links))
    for r, L in sorted(links.items()):
        if L["parent"]:
            print(f"  {r:<18} parent={L['parent']:<16} kind={L.get('kind'):<12} axis={L.get('axis')} pivot={L.get('pivot')}")


HTML = r"""<!DOCTYPE html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Робот-боксёр — ходьба и удар</title>
<style>
html,body{margin:0;height:100%;background:#eef1f5;font:14px system-ui,sans-serif;color:#1d232a;overflow:hidden}
#bar{position:fixed;left:0;right:0;top:0;padding:8px 12px;background:rgba(255,255,255,.92);border-bottom:1px solid #d9dee3;z-index:2}
#bar b{font-size:15px}#bar small{color:#5d6873}
#msg{position:fixed;top:48%;left:0;right:0;text-align:center;color:#5d6873;z-index:1}
#ui{position:fixed;left:8px;right:8px;bottom:8px;display:flex;gap:6px;align-items:center;z-index:2;flex-wrap:wrap}
button{font:600 13px system-ui;padding:9px 12px;border-radius:8px;border:1px solid #c4ccd4;background:#fff;color:#1d232a}
#ph{font-weight:600;color:#c75b5b}
</style>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
</head><body>
<div id="bar"><b>Робот-боксёр MG90S</b> <small>· симуляция: ходьба → удар · <span id="ph">—</span></small></div>
<div id="msg">Загрузка…</div>
<div id="ui"><button id="play">⏸ Пауза</button><button id="viso">3/4</button><button id="vfront">Спереди</button><button id="vside">Сбоку</button><button id="slow">½×</button></div>
<script type="text/plain" id="model">__B64__</script>
<script>
async function unpack(){
  const b64=document.getElementById('model').textContent.trim();
  const bin=atob(b64),u8=new Uint8Array(bin.length);for(let i=0;i<bin.length;i++)u8[i]=bin.charCodeAt(i);
  const buf=await new Response(new Blob([u8]).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer();
  const hl=new DataView(buf).getUint32(0,true);
  const head=JSON.parse(new TextDecoder().decode(new Uint8Array(buf,4,hl)));
  let o=4+hl;
  head.geoms=head.parts.map(p=>{
    const q=new Uint16Array(buf.slice(o,o+p.nv*6));o+=p.nv*6;
    const idx=new Uint16Array(buf.slice(o,o+p.nf*6));o+=p.nf*6;
    const pos=new Float32Array(p.nv*3);
    for(let i=0;i<p.nv*3;i++)pos[i]=p.lo[i%3]+q[i]*p.sc[i%3];
    let g=new THREE.BufferGeometry();
    g.setAttribute('position',new THREE.BufferAttribute(pos,3));g.setIndex(new THREE.BufferAttribute(idx,1));
    g=g.toNonIndexed();g.computeVertexNormals();return g;});
  if(o!==buf.byteLength)throw new Error('данные обрезаны '+o+'/'+buf.byteLength);
  return head;
}
// base matrix (SW row-vector p*R+T, T in metres) -> THREE Matrix4
function baseMat(a){const M=new THREE.Matrix4();
  M.set(a[0],a[3],a[6],a[9]*1000, a[1],a[4],a[7],a[10]*1000, a[2],a[5],a[8],a[11]*1000, 0,0,0,1);return M;}
function aboutAxis(axis,pivot,ang){ // world rotation `ang` about line (axis@pivot)
  const q=new THREE.Quaternion().setFromAxisAngle(axis,ang);
  const R=new THREE.Matrix4().makeRotationFromQuaternion(q);
  const T1=new THREE.Matrix4().makeTranslation(pivot.x,pivot.y,pivot.z);
  const T0=new THREE.Matrix4().makeTranslation(-pivot.x,-pivot.y,-pivot.z);
  return T1.multiply(R).multiply(T0);}

let PAUSE=false,RATE=1;
async function start(){
  const msg=document.getElementById('msg');
  try{
    if(!window.THREE||!THREE.OrbitControls)throw new Error('three.js не загрузился');
    const H=await unpack();
    const scene=new THREE.Scene();scene.background=new THREE.Color(0xeef1f5);
    const camera=new THREE.PerspectiveCamera(42,innerWidth/innerHeight,1,20000);
    const renderer=new THREE.WebGLRenderer({antialias:true});
    renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.setSize(innerWidth,innerHeight);
    document.body.appendChild(renderer.domElement);
    scene.add(new THREE.HemisphereLight(0xffffff,0x8899aa,0.9));
    const sun=new THREE.DirectionalLight(0xffffff,0.6);sun.position.set(300,600,500);scene.add(sun);
    const root=new THREE.Group();root.rotation.z=-Math.PI/2;scene.add(root); // model x down -> screen up
    // link neutral data
    const links=H.links;
    for(const r in links){const L=links[r];
      if(L.axis){L._ax=new THREE.Vector3(L.axis[0],L.axis[1],L.axis[2]);L._pv=new THREE.Vector3(L.pivot[0],L.pivot[1],L.pivot[2]);}
    }
    // order links parent-first
    const roleOrder=[];{const seen={};function add(r){if(seen[r])return;const p=links[r].parent;if(p)add(p);seen[r]=1;roleOrder.push(r);}for(const r in links)add(r);}
    // meshes
    const mats=H.parts.map(p=>new THREE.MeshStandardMaterial({color:p.color,metalness:0.05,roughness:0.65}));
    const items=[];
    for(const it of H.inst){const mesh=new THREE.Mesh(H.geoms[it.p],mats[it.p]);mesh.matrixAutoUpdate=false;
      root.add(mesh);items.push({mesh,base:baseMat(it.m),L:it.L});}
    // fit camera
    root.updateMatrixWorld(true);
    const box=new THREE.Box3().setFromObject(root);const sph=box.getBoundingSphere(new THREE.Sphere());
    const controls=new THREE.OrbitControls(camera,renderer.domElement);controls.enableDamping=true;
    function view(dir){const vf=camera.fov*Math.PI/180,hf=2*Math.atan(Math.tan(vf/2)*camera.aspect);
      const dd=sph.radius/Math.sin(Math.min(vf,hf)/2)*1.05;
      camera.position.copy(sph.center).add(new THREE.Vector3(...dir).normalize().multiplyScalar(dd));
      controls.target.copy(sph.center);controls.update();}
    const V={iso:[0.6,0.25,0.9],front:[0,0.02,1],side:[1,0.02,0.02]};let cur=V.iso;view(cur);
    document.getElementById('viso').onclick=()=>view(cur=V.iso);
    document.getElementById('vfront').onclick=()=>view(cur=V.front);
    document.getElementById('vside').onclick=()=>view(cur=V.side);
    document.getElementById('play').onclick=e=>{PAUSE=!PAUSE;e.target.textContent=PAUSE?'▶ Играть':'⏸ Пауза';};
    document.getElementById('slow').onclick=e=>{RATE=RATE===1?0.5:(RATE===0.5?0.25:1);e.target.textContent=(RATE===1?'½×':RATE===0.5?'¼×':'1×');};
    addEventListener('resize',()=>{camera.aspect=innerWidth/innerHeight;camera.updateProjectionMatrix();renderer.setSize(innerWidth,innerHeight);view(cur);});
    msg.remove();

    // ---------- animation: joint angle (rad) by kind/side at time t (s) ----------
    const P=1.15;                 // gait period
    const phEl=document.getElementById('ph');
    function ease(a,b,u){u=Math.max(0,Math.min(1,u));u=u*u*(3-2*u);return a+(b-a)*u;}
    function gait(kind,side,t){   // walk in place, legs A/B antiphase
      const ph=2*Math.PI*t/P+(side==='B'?Math.PI:0),s=Math.sin(ph);
      const lift=Math.max(0,Math.sin(ph));        // 0..1 during swing
      if(kind==='hippitch') return 0.50*s;        // swing fwd/back
      if(kind==='knee')     return -1.35*lift;    // bend (flex) to lift the foot in swing
      if(kind==='ankle')    return -0.50*s+0.55*lift;
      if(kind==='hiproll')  return 0.05*(side==='A'?1:-1)*Math.sin(2*Math.PI*t/P);
      return 0;
    }
    function bodyMat(t){ // gentle bob (up = model -x) + lateral sway during walking
      const w=(t%LOOP)<T_WALK?1:0;
      const up=-4*Math.abs(Math.sin(2*Math.PI*t/P))*w, sway=7*Math.sin(2*Math.PI*t/P)*w;
      return new THREE.Matrix4().makeTranslation(up,sway,0);
    }
    function guardArm(kind,side){ // boxing guard: forearms up, fists near face
      if(kind==='shoulder')     return 1.15;      // upper arm forward/up from hanging
      if(kind==='shoulderroll') return (side==='A'?1:-1)*0.25;
      if(kind==='elbow')        return 1.5;        // bent elbow
      return 0;
    }
    // timeline: walk -> settle -> punch(right=A) -> hold -> loop
    const T_WALK=P*3, T_SET=0.5, T_PNCH=1.4, T_END=0.7;
    const LOOP=T_WALK+T_SET+T_PNCH+T_END;
    function angles(role,t){
      const L=links[role];if(!L.kind)return 0;const kind=L.kind,side=L.side;
      t=((t%LOOP)+LOOP)%LOOP;
      if(t<T_WALK){ phEl.textContent='ходьба';
        if(kind==='waist')return 0.12*Math.sin(2*Math.PI*t/P);
        if(kind.startsWith('shoulder')||kind==='elbow'){ // arms swing a little while walking
          if(kind==='shoulder')return 0.9+0.15*Math.sin(2*Math.PI*t/P+(side==='A'?Math.PI:0));
          if(kind==='elbow')return 1.2; return (side==='A'?1:-1)*0.2;}
        return gait(kind,side,t);
      }
      const u1=(t-T_WALK)/T_SET;
      if(t<T_WALK+T_SET){ phEl.textContent='стойка';
        if(kind==='waist')return ease(0.12*Math.sin(2*Math.PI*T_WALK/P),0,u1);
        if(kind.startsWith('shoulder')||kind==='elbow')return ease(kind==='shoulder'?0.9:(kind==='elbow'?1.2:(side==='A'?0.2:-0.2)),guardArm(kind,side),u1);
        return ease(gait(kind,side,T_WALK),0,u1); // legs to stance
      }
      const tp=t-T_WALK-T_SET;
      if(tp<T_PNCH){ phEl.textContent='удар правой';
        const u=tp/T_PNCH, strike=Math.sin(Math.PI*u); // 0 -> 1 (extended) -> 0 over the window
        if(side==='A'){ // punching arm (right)
          if(kind==='shoulder')return ease(guardArm('shoulder','A'),0.15,strike)  // drop/extend forward
                                    + 0;
          if(kind==='elbow')   return ease(guardArm('elbow','A'),0.15,strike);    // straighten
          if(kind==='shoulderroll')return guardArm('shoulderroll','A');
        } else { // guard arm stays
          if(kind.startsWith('shoulder')||kind==='elbow')return guardArm(kind,'B');
        }
        if(kind==='waist')return 0.35*strike;      // twist into the punch
        return 0;
      }
      phEl.textContent='возврат';
      const u2=(tp-T_PNCH)/T_END;
      if(kind==='waist')return ease(0.35*0,0,u2);
      if(kind.startsWith('shoulder')||kind==='elbow')return guardArm(kind,side);
      return 0;
    }

    const Lm={};  // role -> Matrix4 world
    const tmp=new THREE.Matrix4();
    function fk(t){
      for(const r of roleOrder){const L=links[r];
        if(L.parent===null){Lm[r]=bodyMat(t);continue;} // base: body bob/sway while walking
        const par=Lm[L.parent];
        const ax=L._ax.clone().transformDirection(par).normalize();
        const pv=L._pv.clone().applyMatrix4(par);
        const J=aboutAxis(ax,pv,angles(r,t));
        Lm[r]=J.multiply(par);
      }
      for(const it of items){tmp.multiplyMatrices(Lm[it.L],it.base);it.mesh.matrix.copy(tmp);}
    }
    let t0=performance.now()/1000,tv=0;
    const th=location.hash.match(/t=([0-9.]+)/);if(th){PAUSE=true;tv=parseFloat(th[1]);}
    const vv=location.hash.match(/v=(iso|front|side)/);if(vv)view(cur=V[vv[1]]);
    if(th){ // static single-frame render (for screenshots): no rAF loop
      fk(tv);controls.update();renderer.render(scene,camera);
      requestAnimationFrame(()=>{fk(tv);renderer.render(scene,camera);document.title='READY';});
    } else {
      (function loop(){requestAnimationFrame(loop);
        const now=performance.now()/1000;if(!PAUSE)tv+=(now-t0)*RATE;t0=now;
        fk(tv);controls.update();renderer.render(scene,camera);})();
    }
  }catch(e){msg.textContent='Ошибка: '+e.message;console.error(e);}
}
document.addEventListener('DOMContentLoaded',start);
</script>
</body></html>
"""

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else str(PROJ / "docs" / "anim" / "robot_anim.html"))
