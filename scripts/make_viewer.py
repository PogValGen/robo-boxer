"""Interactive 3D viewer (single HTML, works offline on Android) of a SolidWorks assembly.

Walks every leaf part of the assembly, exports each UNIQUE part once to STL (coarse) and
writes an HTML page with three.js (inlined) that places the meshes with the SolidWorks
component transforms. One-finger rotate, pinch zoom, two-finger pan.

Usage: python make_viewer.py [assembly.SLDASM] [out.html]
       default: cad/body/Robot_Lower.SLDASM -> docs/viewer/robot_lower_3d.html
Vendor JS (three r128 + OrbitControls) is read from VENDOR (downloaded once).
"""

from __future__ import annotations

import asyncio
import base64
import gzip
import html as html_lib
import json
import sys
from pathlib import Path

import build_joint as bj
import stl_pockets
from build_joint import mc, t

VENDOR = Path(r"C:\Users\40E4~1\AppData\Local\Temp\claude\C--WINDOWS-system32\f6af84ef-89e2-4e17-8f15-b33bc6f6b67e\scratchpad\vendor_three")
STL_DIR = bj.PROJ / "cad" / "viewer_stl"
COLORS = [("MG90S_grabcad", "#2b2f36"), ("Fist", "#d94f2b"), ("Yoke", "#5b6f8f"), ("Horn", "#f2f2f2"), ("Bearing", "#9fb3c8"), ("Pin", "#c9d3dc"),
          ("Housing", "#7f8fa6"), ("Bridge", "#b8c4d6"), ("Lever", "#ef8a2f"), ("Foot", "#4f9d69"),
          ("Pelvis", "#5b6f8f"), ("Torso", "#c75b5b")]


def color(stem: str) -> str:
    return next((c for k, c in COLORS if k.lower() in stem.lower()), "#aaaaaa")


async def main(asm_path: Path, out_html: Path) -> None:
    STL_DIR.mkdir(parents=True, exist_ok=True)
    out_html.parent.mkdir(parents=True, exist_ok=True)
    async with bj.sw_session() as (m, a):
        bj.mcp, bj.adapter = m, a
        await t("open_model", {"file_path": str(asm_path)}, why="assembly to view", n=60)
        asm = mc.typed(bj.model(), "IAssemblyDoc")
        inst = []
        for c in asm.GetComponents(False) or []:
            c = mc.typed(c, "IComponent2")
            path = c.GetPathName()
            if not path.lower().endswith(".sldprt") or c.IsSuppressed() or c.IsHidden(True):
                continue
            arr = list(mc.typed(c.Transform2, "IMathTransform").ArrayData)
            box = [v * 1000 for v in c.GetBox(False, False)]
            inst.append({"path": path, "m": arr[:12], "box": box, "name": c.Name2})
        print(f"    {len(inst)} part instances, {len({i['path'] for i in inst})} unique parts")
        await t("close_model", {"save": False}, why="close assembly", n=40)
        meshes = {}
        for path in sorted({i["path"] for i in inst}):
            stem = Path(path).stem
            stl = STL_DIR / (stem + ".STL")
            await t("open_model", {"file_path": path}, why=f"open {stem}", n=40)
            lo = mc.geometry_report(bj.model(), "bounding_box")["part_bounding_box"]["min_mm"]
            await t("export_stl", {"file_path": str(stl), "format_type": "stl", "resolution": "coarse"},
                    why=f"STL {stem}", n=40)
            await t("close_model", {"save": False}, why="close", n=30)
            tri = stl_pockets.load(str(stl))
            # STL may be shifted to positive space: offset back to the part frame
            off = [round(a_ - b_, 4) for a_, b_ in zip(lo, tri.reshape(-1, 3).min(0))]
            meshes[path] = {"stem": stem, "b64": base64.b64encode(stl.read_bytes()).decode(), "off": off,
                            "color": color(stem), "n": len(tri)}
            print(f"    {stem}: {len(tri)} triangles, offset {off}")
    # self-check the transform convention on every instance: part box corners -> assembly box
    import itertools

    import numpy as np
    bad = 0
    for i in inst:
        tri = stl_pockets.load(str(STL_DIR / (Path(i["path"]).stem + ".STL"))).reshape(-1, 3) + np.array(meshes[i["path"]]["off"])
        r = np.array(i["m"][:9]).reshape(3, 3)
        tr = np.array(i["m"][9:12]) * 1000
        pts = tri @ r + tr                        # SW: row vector * R + T
        got = np.concatenate([pts.min(0), pts.max(0)])
        if np.abs(got - np.array(i["box"])).max() > 1.0:
            bad += 1
            if bad < 4:
                print("    transform mismatch", i["name"], got.round(1), np.round(i["box"], 1))
    print(f"    transform check: {len(inst) - bad}/{len(inst)} instances match the SW boxes")
    data = {"title": asm_path.stem, "meshes": {
        k: {"gz": base64.b64encode(gzip.compress((STL_DIR / (v["stem"] + ".STL")).read_bytes(), 9)).decode(),
            "off": v["off"], "color": v["color"], "stem": v["stem"]} for k, v in meshes.items()},
        "inst": [{"p": i["path"], "m": i["m"], "name": i["name"]} for i in inst]}
    out_html.with_suffix(".json").write_text(json.dumps(data), encoding="utf-8")
    write_html(data, out_html)


def write_html(data: dict, out_html: Path) -> None:
    """Page for the claude.ai artifact viewer (works in the Android app): three r128 from jsDelivr,
    meshes gzip+base64 in <script type=text/plain> blocks at the END, parsed after DOMContentLoaded
    (relay-matrix recipe: parsing earlier read a half-loaded block)."""
    legend = sorted({(v["stem"], v["color"]) for v in data["meshes"].values()})
    meta = {"title": data["title"], "inst": data["inst"], "legend": legend,
            "meshes": {k: {kk: v[kk] for kk in ("off", "color", "stem")} for k, v in data["meshes"].items()}}
    blobs = "\n".join(f'<script type="text/plain" class="mesh" data-p="{html_lib.escape(k)}">{v["gz"]}</script>'
                       for k, v in data["meshes"].items())
    page = HTML.replace("__TITLE__", data["title"]).replace("__META__", json.dumps(meta)).replace("__BLOBS__", blobs)
    out_html.write_text(page, encoding="utf-8")
    print(f"    wrote {out_html} ({out_html.stat().st_size / 1e6:.2f} MB)")


HTML = r"""<!DOCTYPE html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">
<title>__TITLE__ — 3D</title>
<style>
html,body{margin:0;height:100%;background:#eef1f5;font:14px system-ui,sans-serif;color:#1d232a;overflow:hidden}
#bar{position:fixed;left:0;right:0;top:0;padding:8px 12px;background:rgba(255,255,255,.9);border-bottom:1px solid #d9dee3;z-index:2}
#bar b{font-size:15px}#bar small{color:#5d6873}
#msg{position:fixed;top:50%;left:0;right:0;text-align:center;color:#5d6873;z-index:1}
#legend{position:fixed;left:8px;bottom:8px;background:rgba(255,255,255,.9);border:1px solid #d9dee3;border-radius:8px;padding:6px 8px;font-size:12px;z-index:2;max-width:62%}
#legend span{display:inline-flex;align-items:center;gap:4px;margin:2px 8px 2px 0}
#legend i{width:11px;height:11px;border-radius:2px;display:inline-block}
#btns{position:fixed;right:8px;bottom:8px;display:flex;flex-direction:column;gap:6px;z-index:2}
button{font:13px system-ui;padding:9px 11px;border-radius:8px;border:1px solid #c4ccd4;background:#fff}
</style>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
</head><body>
<div id="bar"><b>__TITLE__</b> <small>· палец — вращать, два — масштаб/сдвиг</small></div>
<div id="msg">Загрузка модели…</div>
<div id="legend"></div>
<div id="btns"><button id="reset">Вид</button><button id="servo">Сервы</button></div>
<script>const META = __META__;</script>
<script>
async function gunzip(b64) {
  const bin = atob(b64.trim()), u8 = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
  const ds = new Blob([u8]).stream().pipeThrough(new DecompressionStream('gzip'));
  return await new Response(ds).arrayBuffer();
}
function stlGeometry(buf, off) {
  const dv = new DataView(buf), n = dv.getUint32(80, true);
  if (84 + n * 50 > buf.byteLength) throw new Error('STL обрезан: ' + buf.byteLength + ' из ' + (84 + n * 50));
  const pos = new Float32Array(n * 9);
  for (let i = 0; i < n; i++) { const o = 84 + i * 50 + 12;
    for (let k = 0; k < 9; k++) pos[i * 9 + k] = dv.getFloat32(o + k * 4, true) + off[k % 3]; }
  const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  g.computeVertexNormals(); return g;
}
async function start() {
  const msg = document.getElementById('msg');
  try {
    if (!window.THREE || !THREE.OrbitControls) throw new Error('three.js не загрузился');
    const scene = new THREE.Scene(); scene.background = new THREE.Color(0xeef1f5);
    const camera = new THREE.PerspectiveCamera(40, innerWidth / innerHeight, 1, 10000);
    const renderer = new THREE.WebGLRenderer({antialias: true});
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); renderer.setSize(innerWidth, innerHeight);
    document.body.appendChild(renderer.domElement);
    scene.add(new THREE.HemisphereLight(0xffffff, 0x8899aa, 0.85));
    const sun = new THREE.DirectionalLight(0xffffff, 0.65); sun.position.set(300, 500, 400); scene.add(sun);
    const geoms = {};
    for (const el of document.querySelectorAll('script.mesh')) {
      const p = el.dataset.p; geoms[p] = stlGeometry(await gunzip(el.textContent), META.meshes[p].off);
    }
    const robot = new THREE.Group(); robot.rotation.z = -Math.PI / 2;   // model x points down -> screen up
    scene.add(robot); const servos = [];
    for (const it of META.inst) {
      const m = META.meshes[it.p], a = it.m;
      const mat = new THREE.MeshStandardMaterial({color: m.color, metalness: /Bearing|Pin/.test(m.stem) ? 0.6 : 0.05, roughness: 0.6});
      const mesh = new THREE.Mesh(geoms[it.p], mat); mesh.matrixAutoUpdate = false;
      // SolidWorks transform: p' = p * R + T (row vector), T in metres
      mesh.matrix.set(a[0], a[3], a[6], a[9] * 1000, a[1], a[4], a[7], a[10] * 1000, a[2], a[5], a[8], a[11] * 1000, 0, 0, 0, 1);
      robot.add(mesh); if (m.stem.includes('MG90S_grabcad')) servos.push(mesh);
    }
    robot.updateMatrixWorld(true);
    const sphere = new THREE.Box3().setFromObject(robot).getBoundingSphere(new THREE.Sphere());
    const controls = new THREE.OrbitControls(camera, renderer.domElement); controls.enableDamping = true;
    function home() {
      const vfov = camera.fov * Math.PI / 180, hfov = 2 * Math.atan(Math.tan(vfov / 2) * camera.aspect);
      const d = sphere.radius / Math.sin(Math.min(vfov, hfov) / 2) * 1.02;
      camera.position.copy(sphere.center).add(new THREE.Vector3(0.55, 0.3, 0.78).normalize().multiplyScalar(d));
      controls.target.copy(sphere.center); controls.update();
    }
    home(); document.getElementById('reset').onclick = home;
    // #turn=DEG: fixed turntable camera for rendering frames (GIF/MP4 for the phone), UI hidden
    const tm = location.hash.match(/turn=(-?\d+)/);
    if (tm) {
      const a = +tm[1] * Math.PI / 180, d = camera.position.distanceTo(sphere.center);
      camera.position.set(sphere.center.x + d * 0.93 * Math.sin(a), sphere.center.y + d * 0.28, sphere.center.z + d * 0.93 * Math.cos(a));
      controls.target.copy(sphere.center); controls.update();
      for (const id of ['bar', 'btns', 'legend']) document.getElementById(id).style.display = 'none';
    }
    document.getElementById('servo').onclick = () => servos.forEach(s => s.visible = !s.visible);
    document.getElementById('legend').innerHTML = META.legend.map(([n, c]) => `<span><i style="background:${c}"></i>${n}</span>`).join('');
    addEventListener('resize', () => { camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix(); renderer.setSize(innerWidth, innerHeight); });
    msg.remove();
    (function loop() { requestAnimationFrame(loop); controls.update(); renderer.render(scene, camera); })();
  } catch (e) { msg.textContent = 'Ошибка: ' + e.message; console.error(e); }
}
document.addEventListener('DOMContentLoaded', start);
</script>
__BLOBS__
</body></html>
"""

if __name__ == "__main__":
    if sys.argv[1:2] == ["--html"]:  # regenerate the page from the saved .json (no SolidWorks)
        out = Path(sys.argv[2])
        write_html(json.loads(out.with_suffix(".json").read_text(encoding="utf-8")), out)
    else:
        asm = Path(sys.argv[1]) if len(sys.argv) > 1 else bj.PROJ / "cad" / "body" / "Robot.SLDASM"
        out = Path(sys.argv[2]) if len(sys.argv) > 2 else bj.PROJ / "docs" / "viewer" / "robot_3d.html"
        asyncio.run(main(asm, out))
