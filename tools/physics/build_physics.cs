// SOLIDWORKS Motion physics model of the robo-boxer, driven through the .NET interop
// (the Motion Study API is not reachable from pywin32).
//   build_physics.exe build <spec.json> <out.SLDASM>   flat assembly: parts, locks per link, 15 revolutes
//   build_physics.exe gravtest <out.SLDASM>             Motion Analysis, gravity only, 0.6 s -> joints must be free
// Spec: scripts/gen_phys_spec.py. Leaves 0 documents open.
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Web.Script.Serialization;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SolidWorks.Interop.swmotionstudy;

class Phys {
    static SldWorks app;
    static void Log(string s) { Console.WriteLine(s); }
    static double D(object o) { return Convert.ToDouble(o); }
    static double[] Arr(object o) { return ((IEnumerable)o).Cast<object>().Select(D).ToArray(); }

    static HashSet<string> OpenTitles() {
        var s = new HashSet<string>();
        for (var d = (ModelDoc2)app.GetFirstDocument(); d != null; d = (ModelDoc2)d.GetNext()) s.Add(d.GetTitle());
        return s;
    }

    static int Main(string[] a) {
        app = (SldWorks)Marshal.GetActiveObject("SldWorks.Application");
        app.LoadAddIn(@"C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\cmotionsw.dll");
        var mine = OpenTitles();   // documents open before this run are not ours: never close them
        Log("docs open before: " + mine.Count);
        try {
            if (a[0] == "build") return Build(a[1], a[2]);
            if (a[0] == "gravtest") return GravTest(a[1]);
            if (a[0] == "run") return Run(a[1], a[2]);
            if (a[0] == "motordbg") return MotorDbg(a[1], a[2]);
            if (a[0] == "motortest") return MotorTest(a[1], a[2]);
            if (a[0] == "plottest") return PlotTest(a[1], a[2]);
            if (a[0] == "render") return Render(a[1], a[2], a[3], double.Parse(a[4], System.Globalization.CultureInfo.InvariantCulture));
            if (a[0] == "massfix") return MassFix(a[1], double.Parse(a[2], System.Globalization.CultureInfo.InvariantCulture));
        } catch (Exception ex) { Log("FATAL " + ex); }
        finally {
            foreach (var t in OpenTitles()) if (!mine.Contains(t)) app.CloseDoc(t);
            Log("closed own docs; open now: " + OpenTitles().Count + " (before: " + mine.Count + ")");
        }
        return 1;
    }

    static double[] Pt(object o) {
        if (o is double[]) return (double[])o;
        var mp = o as MathPoint; if (mp != null) return (double[])mp.ArrayData;
        var mv = o as MathVector; if (mv != null) return (double[])mv.ArrayData;
        throw new InvalidCastException("result type " + Microsoft.VisualBasic.Information.TypeName(o));
    }

    static string CompName(object c) { return c == null ? "null" : ((Component2)c).Name2; }

    static Mate2 AddMate(AssemblyDoc asm, int type, int align, out int err) {
        return asm.AddMate5(type, align, false, 0, 0, 0, 1, 1, 0, 0, 0, false, false, 0, out err);
    }

    static int Build(string specPath, string outPath) {
        var spec = (Dictionary<string, object>)new JavaScriptSerializer { MaxJsonLength = int.MaxValue }
            .DeserializeObject(File.ReadAllText(specPath));
        var parts = ((object[])spec["parts"]).Cast<Dictionary<string, object>>().ToList();
        var anchors = (Dictionary<string, object>)spec["anchors"];
        var joints = ((object[])spec["joints"]).Cast<Dictionary<string, object>>().ToList();
        string tmpl = app.GetUserPreferenceStringValue((int)swUserPreferenceStringValue_e.swDefaultTemplateAssembly);
        var doc = (ModelDoc2)app.NewDocument(tmpl, 0, 0, 0);
        var asm = (AssemblyDoc)doc;
        var mu = (MathUtility)app.GetMathUtility();
        int e = 0, w = 0;
        // load each part once, WITHOUT a window: every visible part document costs ~500 MB of SolidWorks memory
        // (probe 2026-11-15: 27 distinct v3 parts -> 13 GB, "low memory", SOLIDWORKS Error Report)
        app.DocumentVisible(false, (int)swDocumentTypes_e.swDocPART);
        try {
            foreach (var path in parts.Select(p => (string)p["path"]).Distinct())
                app.OpenDoc6(path, (int)swDocumentTypes_e.swDocPART, (int)swOpenDocOptions_e.swOpenDocOptions_Silent, "", ref e, ref w);
        } finally { app.DocumentVisible(true, (int)swDocumentTypes_e.swDocPART); }
        Log("parts loaded (hidden): " + parts.Select(p => (string)p["path"]).Distinct().Count());
        app.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
        var comps = new Component2[parts.Count];
        for (int i = 0; i < parts.Count; i++) {
            var c = (Component2)asm.AddComponent5((string)parts[i]["path"], 0, "", false, "", 0, 0, 0);
            if (c == null) { Log("!! insert failed " + parts[i]["name"]); return 2; }
            comps[i] = c;
        }
        doc.ClearSelection2(true);
        foreach (var c in comps) c.Select4(true, null, false);
        asm.UnfixComponent();                       // first inserted comp is fixed by default
        doc.ClearSelection2(true);
        for (int i = 0; i < parts.Count; i++)
            comps[i].Transform2 = (MathTransform)mu.CreateTransform(Arr(parts[i]["m"]));
        doc.EditRebuild3();
        int baseAnchor = Convert.ToInt32(anchors[(string)spec["fixed_link"]]);
        comps[baseAnchor].Select4(false, null, false); asm.FixComponent(); doc.ClearSelection2(true);
        Log("inserted " + parts.Count + " parts, fixed " + comps[baseAnchor].Name2);

        // rigid links: lock every part to its link anchor
        int locks = 0, lockErr = 0;
        for (int i = 0; i < parts.Count; i++) {
            int an = Convert.ToInt32(anchors[(string)parts[i]["link"]]);
            if (an == i) continue;
            doc.ClearSelection2(true);
            comps[an].Select4(false, null, false); comps[i].Select4(true, null, false);
            int err; var m = AddMate(asm, (int)swMateType_e.swMateLOCK, (int)swMateAlign_e.swMateAlignCLOSEST, out err);
            if (m == null) lockErr++; else locks++;
        }
        doc.ClearSelection2(true); doc.EditRebuild3();
        Log("lock mates: " + locks + " ok, " + lockErr + " failed");

        // revolutes
        var ext = doc.Extension;
        int jOk = 0;
        foreach (var j in joints) {
            var lever = comps[Convert.ToInt32(j["lever"])]; var housing = comps[Convert.ToInt32(j["housing"])];
            var bridge = comps[Convert.ToInt32(j["bridge"])];
            double[] ps = Arr(j["p_spigot"]), pc = Arr(j["p_cup"]);
            var before = (double[])lever.Transform2.ArrayData;
            doc.ClearSelection2(true);
            var sm = (SelectionMgr)doc.SelectionManager;
            // pick the cylinder of the WANTED component through the point (no view ray: with a 0.1 mm gap
            // SelectByID2 picked the lever's own face twice -> revolutes 10/17, 2026-11-13); fallback = old way
            bool s1 = PickCyl(doc, sm, lever, ps, false) || ext.SelectByID2("", "FACE", ps[0], ps[1], ps[2], false, 1, null, 0);
            bool s2 = PickCyl(doc, sm, bridge, pc, true) || ext.SelectByID2("", "FACE", pc[0], pc[1], pc[2], true, 1, null, 0);
            string f1 = s1 ? CompName(sm.GetSelectedObjectsComponent4(1, -1)) : "-";
            string f2 = s2 ? CompName(sm.GetSelectedObjectsComponent4(2, -1)) : "-";
            int ec; var mc = AddMate(asm, (int)swMateType_e.swMateCONCENTRIC, (int)swMateAlign_e.swMateAlignCLOSEST, out ec);
            doc.ClearSelection2(true);
            // v3 (2026-11-15): the plane pair perpendicular to the joint axis is named in the spec (parts are modelled
            // in the robot frame, so the base plane normal to the axis coincides in both links at the zero pose)
            var pa = j.ContainsKey("plane_a") ? comps[Convert.ToInt32(j["plane_a"])] : lever;
            var pb = j.ContainsKey("plane_b") ? comps[Convert.ToInt32(j["plane_b"])] : housing;
            string pn = j.ContainsKey("plane_name") ? (string)j["plane_name"] : "Top Plane";
            ((Feature)pa.FeatureByName(pn)).Select2(false, 1);
            ((Feature)pb.FeatureByName(pn)).Select2(true, 1);
            int ep; var mp = AddMate(asm, (int)swMateType_e.swMateCOINCIDENT, (int)swMateAlign_e.swMateAlignCLOSEST, out ep);
            doc.ClearSelection2(true); doc.EditRebuild3();
            var after = (double[])lever.Transform2.ArrayData;
            double moved = before.Zip(after, (x, y) => Math.Abs(x - y)).Max();
            bool ok = mc != null && mp != null && f1 == lever.Name2 && f2 == bridge.Name2 && moved < 1e-6;
            if (ok) jOk++;
            Log(string.Format("  {0,-18} faces[{1} | {2}] concentric={3}({4}) coincident={5}({6}) moved={7:E1} {8}",
                j["role"], f1, f2, mc != null, ec, mp != null, ep, moved, ok ? "ok" : "!!"));
        }
        Log("revolutes ok: " + jOk + "/" + joints.Count);
        Directory.CreateDirectory(Path.GetDirectoryName(outPath));
        int se = 0, sw = 0;
        doc.Extension.SaveAs(outPath, 0, (int)swSaveAsOptions_e.swSaveAsOptions_Silent, null, ref se, ref sw);
        Log("saved " + outPath + " err=" + se + " exists=" + File.Exists(outPath));
        return jOk == joints.Count ? 0 : 3;
    }

    // Cylindrical face of component c passing through world point p (m): part-space test of |dist to axis - r|.
    // Selected with mark 1 (mate selection), appended if `append`. Same idea as sw_motion CylFace.
    static bool PickCyl(ModelDoc2 doc, SelectionMgr sm, Component2 c, double[] p, bool append) {
        var T = (double[])c.Transform2.ArrayData;
        double[] d = { p[0] - T[9], p[1] - T[10], p[2] - T[11] };
        double[] q = { d[0] * T[0] + d[1] * T[1] + d[2] * T[2], d[0] * T[3] + d[1] * T[4] + d[2] * T[5], d[0] * T[6] + d[1] * T[7] + d[2] * T[8] };
        object best = null; double bestErr = 1e9;
        foreach (var bo in (object[])c.GetBodies2((int)swBodyType_e.swSolidBody) ?? new object[0]) {
            var body = bo as Body2; if (body == null) continue;
            foreach (var fo in (object[])body.GetFaces() ?? new object[0]) {
                var fc = fo as Face2; if (fc == null) continue;
                var su = fc.GetSurface() as Surface; if (su == null || !su.IsCylinder()) continue;
                var cp = (double[])su.CylinderParams;
                double[] o = { cp[0], cp[1], cp[2] }, ax = { cp[3], cp[4], cp[5] }; double r = cp[6];
                double an = Math.Sqrt(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]);
                double[] v = { q[0] - o[0], q[1] - o[1], q[2] - o[2] };
                double along = (v[0] * ax[0] + v[1] * ax[1] + v[2] * ax[2]) / an;
                double dist = Math.Sqrt(Math.Max(0, v[0] * v[0] + v[1] * v[1] + v[2] * v[2] - along * along));
                double err = Math.Abs(dist - r);
                if (err < bestErr) { bestErr = err; best = fc; }
            }
        }
        if (best == null || bestErr >= 5e-5) return false;
        if (!append) doc.ClearSelection2(true);
        var sd = (SelectData)sm.CreateSelectData(); sd.Mark = 1;
        return ((Entity)best).Select4(append, sd);
    }

    // what did Motion actually store in each motor? (no save)
    static int MotorDbg(string asmPath, string study) {
        int e = 0, w = 0;
        var doc = (ModelDoc2)app.OpenDoc6(asmPath, (int)swDocumentTypes_e.swDocASSEMBLY,
            (int)swOpenDocOptions_e.swOpenDocOptions_Silent, "", ref e, ref w);
        var mgr = (MotionStudyManager)doc.Extension.GetMotionStudyManager();
        Log("studies: " + string.Join(", ", ((object[])mgr.GetMotionStudyNames()).Cast<string>()));
        var ms = (MotionStudy)mgr.GetMotionStudy(study); ms.Activate();
        Log("type=" + ms.StudyType + " dur=" + F(ms.GetDuration()) + " features=" + ms.GetMotionFeaturesCount());
        foreach (object o in (object[])ms.GetMotionFeatures()) {
            var f = (Feature)o;
            var def = f.GetDefinition();
            var md = def as SimulationMotorFeatureData;
            if (md == null) { Log("  " + f.Name + " [" + f.GetTypeName2() + "]"); continue; }
            var rc = md.RelativeComponent;
            Log(string.Format("  {0,-22} motion={1} drive={2} rev={3} rel={4} dir={5} expr='{6}' val(0.8)={7}",
                f.Name, md.MotionType, md.DriveType, md.ReverseDirection, rc == null ? "-" : rc.Name2,
                md.DirectionReference == null ? "null" : Microsoft.VisualBasic.Information.TypeName(md.DirectionReference),
                md.Expression, F(SafeVal(md, 0.8))));
        }
        app.CloseDoc(doc.GetTitle());
        return 0;
    }
    // which way of defining a rotary motor profile actually drives the joint? (one joint, 4 variants, no save)
    static int MotorTest(string asmPath, string role) {
        var js = new JavaScriptSerializer { MaxJsonLength = int.MaxValue };
        var spec = (Dictionary<string, object>)js.DeserializeObject(File.ReadAllText(Path.Combine(Path.GetDirectoryName(asmPath), "phys_spec.json")));
        var parts = ((object[])spec["parts"]).Cast<Dictionary<string, object>>().ToList();
        var anchors = (Dictionary<string, object>)spec["anchors"];
        var j = ((object[])spec["joints"]).Cast<Dictionary<string, object>>().First(x => (string)x["role"] == role);
        int e = 0, w = 0;
        var doc = (ModelDoc2)app.OpenDoc6(asmPath, (int)swDocumentTypes_e.swDocASSEMBLY, (int)swOpenDocOptions_e.swOpenDocOptions_Silent, "", ref e, ref w);
        var asm = (AssemblyDoc)doc; var ext = doc.Extension; var sm = (SelectionMgr)doc.SelectionManager;
        Func<int, Component2> comp = i => {
            string st = Path.GetFileNameWithoutExtension((string)parts[i]["path"]);
            int n = parts.Take(i + 1).Count(p => Path.GetFileNameWithoutExtension((string)p["path"]) == st);
            return (Component2)asm.GetComponentByName(st + "-" + n);
        };
        var rotor = comp(Convert.ToInt32(anchors[(string)j["rotor"]]));
        var stator = comp(Convert.ToInt32(anchors[(string)j["stator"]]));
        var mgr = (MotionStudyManager)ext.GetMotionStudyManager();
        var ps = Arr(j["p_spigot"]);
        var variants = new[] { "gravity_only", "loc_const", "loc_expr" };
        foreach (var v in variants) {
            var ms = (MotionStudy)mgr.CreateMotionStudy(); ms.Name = "T_" + v; ms.StudyType = 4; ms.Activate();
            ms.SetDuration(1.0);
            Feature f = null;
            if (v == "gravity_only") {
                var g = (SimulationGravityFeatureData)ms.CreateDefinition((int)swFeatureNameID_e.swFmAEMGravity);
                g.Axis = (int)swSimulationGravityAxisName_e.swSimulationGravityAxis_X; g.ReverseDirection = false;
                f = (Feature)ms.CreateFeature(g);
            } else {
                doc.ClearSelection2(true);
                ext.SelectByID2("", "FACE", ps[0], ps[1], ps[2], false, 0, null, 0);
                var face = sm.GetSelectedObject6(1, -1); doc.ClearSelection2(true);
                var md = (SimulationMotorFeatureData)ms.CreateDefinition((int)swFeatureNameID_e.swFmAEMRotationalMotor);
                md.Location = face; md.DirectionReference = face; md.RelativeComponent = stator;
                if (v == "loc_const") md.ConstantSpeedMotor(10);     // rpm -> 60 deg/s
                if (v == "loc_expr") {
                    md.DriveType = (int)swSimulationMotorDriveType_e.swSimulationMotorDrive_Displacement;
                    md.MotionType = (int)swSimulationMotorMotionType_e.swSimulationMotorMotion_Function;
                    md.Expression = "STEP(TIME,0,0,0.8,60D)";
                }
                f = (Feature)ms.CreateFeature(md);
            }
            var back = f == null ? null : f.GetDefinition() as SimulationMotorFeatureData;
            if (back != null) Log("   location=" + (back.Location == null ? "null" : "set") + " dir=" + (back.DirectionReference == null ? "null" : "set"));
            string rb = back == null ? "no readback" : string.Format("motion={0} expr='{1}' val(0.8)={2} val(0.25)={3}",
                back.MotionType, back.Expression, F(SafeVal(back, 0.8)), F(SafeVal(back, 0.25)));
            bool calc = ms.Calculate();
            var R0 = RotOf(rotor, stator, ms, 0.0); var R1 = RotOf(rotor, stator, ms, 0.8); var R2 = RotOf(rotor, stator, ms, 0.25);
            var bx = new List<string>();
            foreach (var tt in new[] { 0.0, 0.25, 0.8 }) { ms.SetTime(tt); var b = (double[])rotor.GetBox(false, false);
                bx.Add(string.Format(System.Globalization.CultureInfo.InvariantCulture, "t{0}:({1:F0},{2:F0},{3:F0})", tt, (b[0] + b[3]) * 500, (b[1] + b[4]) * 500, (b[2] + b[5]) * 500)); }
            Log(string.Format("{0,-12} feat={1} calc={2} | {3} | Transform2 angle t0.25={4:F1} t0.8={5:F1} | rotor box centre {6}",
                v, f != null, calc, rb, AngleBetween(R0, R2), AngleBetween(R0, R1), string.Join(" ", bx)));
            mgr.DeleteMotionStudy(ms.Name);
        }
        app.CloseDoc(doc.GetTitle());
        return 0;
    }
    // frames of a solved study rendered by SolidWorks: render <asm> <studyPrefix> <outDir> <fps>
    static double rollDeg = -90, tiltDeg = 15, yawDeg = 35; static int maxFrames = 0;
    static int Render(string asmPath, string studyPrefix, string outDir, double fps) {
        var ev = System.Environment.GetEnvironmentVariable("PHYS_VIEW");   // "roll,tilt,yaw,maxFrames" for quick tries
        if (!string.IsNullOrEmpty(ev)) { var v = ev.Split(',').Select(s => double.Parse(s, System.Globalization.CultureInfo.InvariantCulture)).ToArray();
            rollDeg = v[0]; tiltDeg = v[1]; yawDeg = v[2]; if (v.Length > 3) maxFrames = (int)v[3]; }
        int e = 0, w = 0;
        var doc = (ModelDoc2)app.OpenDoc6(asmPath, (int)swDocumentTypes_e.swDocASSEMBLY, (int)swOpenDocOptions_e.swOpenDocOptions_Silent, "", ref e, ref w);
        app.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
        var mgr = (MotionStudyManager)doc.Extension.GetMotionStudyManager();
        string name = ((object[])mgr.GetMotionStudyNames()).Cast<string>().Where(s => s.StartsWith(studyPrefix)).OrderBy(s => s.Length).ThenBy(s => s).Last();
        var ms = (MotionStudy)mgr.GetMotionStudy(name); ms.Activate();
        Log("render study " + name + " duration " + F(ms.GetDuration()) + " out-of-date=" + ((MotionStudyResults)ms.GetResults(4)).IsOutOfDate());
        app.Visible = true;                                   // hidden window -> exported images are stale
        app.FrameState = (int)swWindowState_e.swWindowMaximized;
        foreach (var f in Directory.Exists(outDir) ? Directory.GetFiles(outDir, "f_*.bmp") : new string[0]) File.Delete(f);
        var mv = (ModelView)doc.ActiveView;
        mv.EnableGraphicsUpdate = true;
        // custom camera via Orientation3/RollBy/RotateAboutCenter had no effect on the captured view (2026-11-01):
        // use the standard isometric view; the robot (model x down) is stood upright by rotating the images later
        doc.ShowNamedView2("*Isometric", (int)swStandardViews_e.swIsometricView);
        System.Threading.Thread.Sleep(1500);                 // let the maximised window settle before fitting
        ms.SetTime(Math.Min(1.3, ms.GetDuration())); doc.GraphicsRedraw2(); doc.ViewZoomtofit2();
        // closed loop: grab test frames at the widest poses; zoom out until the model never touches the border
        string probe = Path.Combine(Path.GetTempPath(), "phys_fit_probe.bmp");
        for (int tries = 0; tries < 6; tries++) {
            bool touch = false;
            foreach (double tp in new[] { 0.0, 0.8, 1.45, ms.GetDuration() }) {
                ms.SetTime(Math.Min(tp, ms.GetDuration())); mv.GraphicsRedraw(null); doc.GraphicsRedraw2();
                doc.SaveBMP(probe, 0, 0);
                if (TouchesBorder(probe)) { touch = true; break; }
            }
            Log("  fit try " + tries + ": touches border = " + touch);
            if (!touch) break;
            doc.ViewZoomout(); System.Threading.Thread.Sleep(300);
        }
        var fist = (Component2)((AssemblyDoc)doc).GetComponentByName("Fist_TPU-1");
        Directory.CreateDirectory(outDir);
        foreach (var f in Directory.GetFiles(outDir, "f_*.png")) File.Delete(f);
        int n = (int)Math.Round(ms.GetDuration() * fps), ok = 0;
        if (maxFrames > 0) n = Math.Min(n, maxFrames - 1);
        for (int i = 0; i <= n; i++) {
            double tt = maxFrames > 0 ? i * ms.GetDuration() / Math.Max(1, n) : i / fps;
            ms.SetTime(tt); mv.GraphicsRedraw(null); doc.GraphicsRedraw2();
            if (fist != null && (i == 0 || i == n || maxFrames > 0)) { var b = (double[])fist.GetBox(false, false);
                Log(string.Format(System.Globalization.CultureInfo.InvariantCulture, "  t={0:F2} fist centre ({1:F0},{2:F0},{3:F0})", tt, (b[0]+b[3])*500, (b[1]+b[4])*500, (b[2]+b[5])*500)); }
            // SaveAs(.png) of an assembly wrote a stale picture (same image for every time step, 2026-11-01);
            // SaveBMP grabs the live graphics area
            string bmp = Path.Combine(outDir, string.Format("f_{0:D4}.bmp", i));
            doc.SaveBMP(bmp, 0, 0);          // window size: no aspect crop
            if (File.Exists(bmp)) ok++;
        }
        Log("frames " + ok + "/" + (n + 1) + " -> " + outDir);
        ms.SetTime(0);
        app.CloseDoc(doc.GetTitle());   // not saved: rendering changes nothing
        return ok == n + 1 ? 0 : 6;
    }
    // model pixels (mid-grey/dark, not the pure-black window edge) within 12 px of the image border?
    static bool TouchesBorder(string bmpPath) {
        using (var b = new System.Drawing.Bitmap(bmpPath)) {
            int w = b.Width, h = b.Height, m = 40;
            // every part is drawn with dark edge lines -> model = dark-but-not-black pixel (light parts match the background)
            Func<int, int, bool> model = (x, y) => { var c = b.GetPixel(x, y); int v = (c.R + c.G + c.B) / 3; return v > 25 && v < 110; };
            for (int x = 0; x < w; x += 2) for (int k = 2; k < m; k += 3) if (model(x, k) || model(x, h - 1 - k)) return true;
            for (int y = 0; y < h; y += 2) for (int k = 2; k < m; k += 3) if (model(k, y) || model(w - 1 - k, y)) return true;
        }
        return false;
    }

    static double[,] Mul(double[,] a, double[,] b) {
        var r = new double[3, 3];
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) for (int k = 0; k < 3; k++) r[i, j] += a[i, k] * b[k, j];
        return r;
    }

    // one motor + gravity, then try plot recipes with the motor feature SELECTED (no save)
    static int PlotTest(string asmPath, string role) {
        var js = new JavaScriptSerializer { MaxJsonLength = int.MaxValue };
        var spec = (Dictionary<string, object>)js.DeserializeObject(File.ReadAllText(Path.Combine(Path.GetDirectoryName(asmPath), "phys_spec.json")));
        var parts = ((object[])spec["parts"]).Cast<Dictionary<string, object>>().ToList();
        var anchors = (Dictionary<string, object>)spec["anchors"];
        var j = ((object[])spec["joints"]).Cast<Dictionary<string, object>>().First(x => (string)x["role"] == role);
        int e = 0, w = 0;
        var doc = (ModelDoc2)app.OpenDoc6(asmPath, (int)swDocumentTypes_e.swDocASSEMBLY, (int)swOpenDocOptions_e.swOpenDocOptions_Silent, "", ref e, ref w);
        var asm = (AssemblyDoc)doc; var ext = doc.Extension; var sm = (SelectionMgr)doc.SelectionManager;
        int si = Convert.ToInt32(anchors[(string)j["stator"]]);
        string st = Path.GetFileNameWithoutExtension((string)parts[si]["path"]);
        var stator = (Component2)asm.GetComponentByName(st + "-" + parts.Take(si + 1).Count(p => Path.GetFileNameWithoutExtension((string)p["path"]) == st));
        var mgr = (MotionStudyManager)ext.GetMotionStudyManager();
        var ms = (MotionStudy)mgr.CreateMotionStudy(); ms.Name = "PlotTest"; ms.StudyType = 4; ms.Activate(); ms.SetDuration(1.0);
        var g = (SimulationGravityFeatureData)ms.CreateDefinition((int)swFeatureNameID_e.swFmAEMGravity);
        g.Axis = (int)swSimulationGravityAxisName_e.swSimulationGravityAxis_X; g.ReverseDirection = false; ms.CreateFeature(g);
        var ps = Arr(j["p_spigot"]);
        doc.ClearSelection2(true); ext.SelectByID2("", "FACE", ps[0], ps[1], ps[2], false, 0, null, 0);
        var face = sm.GetSelectedObject6(1, -1); doc.ClearSelection2(true);
        var md = (SimulationMotorFeatureData)ms.CreateDefinition((int)swFeatureNameID_e.swFmAEMRotationalMotor);
        md.Location = face; md.DirectionReference = face; md.RelativeComponent = stator;
        md.DriveType = 0; md.MotionType = 3; md.Expression = "STEP(TIME,0,0,0.8,60D)";
        var mf = (Feature)ms.CreateFeature(md);
        Log("motor=" + (mf != null) + " name=" + (mf == null ? "" : mf.Name) + " type=" + (mf == null ? "" : mf.GetTypeName2()) + " calc=" + ms.Calculate());
        var res = (CosmosMotionStudyResults)ms.GetResults(4);
        foreach (int comp in new[] { 3, 0, 1, 2 })
            foreach (bool select in new[] { true, false }) {
                try {
                    doc.ClearSelection2(true);
                    if (select) mf.Select2(false, 0);
                    var pd = (MotionPlotFeatureData)res.CreatePlotFeatureData();
                    var xa = (MotionPlotAxisFeatureData)res.CreatePlotXAxisFeatureData(); xa.Type = 0;
                    var ya = (MotionPlotAxisFeatureData)res.CreatePlotYAxisFeatureData();
                    ya.Type = (int)swMotionPlotAxisType_e.swMotionPlotAxisType_MOTION_APPLIED_TORQUE; ya.Component = comp;
                    if (!select) ya.Entities = new object[] { mf };
                    var plot = res.InsertPlotFeature(pd, xa, ya);
                    var po = res.GetValues(pd, xa, ya) as MotionPlotFeatureOutput;
                    var ty = new List<string>(); if (po != null) Flatten(po.GetYAxis(ya), ty);
                    Log(string.Format("  comp={0} select={1} plot={2} nY={3} plots_in_study={4} first={5}", comp, select, plot != null, ty.Count, res.GetPlotCount(), ty.Count > 0 ? ty[ty.Count / 2] : "-"));
                } catch (Exception ex) { Log("  comp=" + comp + " select=" + select + " ERR " + ex.Message); }
            }
        app.CloseDoc(doc.GetTitle());
        return 0;
    }

    static double[] RotOf(Component2 rotor, Component2 stator, MotionStudy ms, double t) {
        ms.SetTime(t);
        var a = (double[])rotor.Transform2.ArrayData; var b = (double[])stator.Transform2.ArrayData;
        // relative rotation rotor * stator^T (3x3, row-major)
        var r = new double[9];
        for (int i = 0; i < 3; i++) for (int k = 0; k < 3; k++) { double s = 0; for (int m = 0; m < 3; m++) s += a[i * 3 + m] * b[k * 3 + m]; r[i * 3 + k] = s; }
        return r;
    }
    static double AngleBetween(double[] A, double[] B) {  // angle of A^T B
        double tr = 0; for (int i = 0; i < 3; i++) for (int m = 0; m < 3; m++) tr += A[m * 3 + i] * B[m * 3 + i];
        return Math.Acos(Math.Max(-1, Math.Min(1, (tr - 1) / 2))) * 180 / Math.PI;
    }

    static double SafeVal(SimulationMotorFeatureData md, double t) { try { return Convert.ToDouble(md.GetInterpolatedValue(t)); } catch { return double.NaN; } }

    // mass override (all configurations) in a part file, e.g. servo copy to the datasheet mass
    static int MassFix(string partPath, double grams) {
        int e = 0, w = 0;
        var doc = (ModelDoc2)app.OpenDoc6(partPath, (int)swDocumentTypes_e.swDocPART,
            (int)swOpenDocOptions_e.swOpenDocOptions_Silent, "", ref e, ref w);
        var mp = (MassProperty)doc.Extension.CreateMassProperty();
        double before = mp.Mass;
        mp.OverrideMass = true;
        bool ok = mp.SetOverrideMassValue(grams / 1000.0, (int)swInConfigurationOpts_e.swAllConfiguration, null);
        doc.ForceRebuild3(false);
        int se = 0, sw2 = 0;
        doc.Save3((int)swSaveAsOptions_e.swSaveAsOptions_Silent, ref se, ref sw2);
        double after = ((MassProperty)doc.Extension.CreateMassProperty()).Mass;
        Log(string.Format(System.Globalization.CultureInfo.InvariantCulture, "mass {0:F2} g -> {1:F2} g (set={2}, save err={3})", before * 1000, after * 1000, ok, se));
        return Math.Abs(after * 1000 - grams) < 0.05 ? 0 : 5;
    }

    static void Flatten(object o, List<string> acc) {   // GetValues may return double[], object[] of arrays, ...
        if (o == null) return;
        if (o is double[]) { acc.AddRange(((double[])o).Select(F)); return; }
        if (o is string) return;
        var en = o as IEnumerable;
        if (en != null) { foreach (var x in en) { if (x is double) acc.Add(F((double)x)); else Flatten(x, acc); } return; }
        if (o is double) acc.Add(F((double)o));
    }

    static string F(double v) { return v.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture); }

    // Motion Analysis with one rotary motor per servo; writes <motion>_result.json next to the motion file.
    static int Run(string asmPath, string motionPath) {
        var js = new JavaScriptSerializer { MaxJsonLength = int.MaxValue };
        var mot = (Dictionary<string, object>)js.DeserializeObject(File.ReadAllText(motionPath));
        string specPath = Path.Combine(Path.GetDirectoryName(motionPath), "phys_spec.json");
        var spec = (Dictionary<string, object>)js.DeserializeObject(File.ReadAllText(specPath));
        var parts = ((object[])spec["parts"]).Cast<Dictionary<string, object>>().ToList();
        var anchors = (Dictionary<string, object>)spec["anchors"];
        var joints = ((object[])spec["joints"]).Cast<Dictionary<string, object>>().ToDictionary(j => (string)j["role"]);
        int e = 0, w = 0;
        var doc = (ModelDoc2)app.OpenDoc6(asmPath, (int)swDocumentTypes_e.swDocASSEMBLY,
            (int)swOpenDocOptions_e.swOpenDocOptions_Silent, "", ref e, ref w);
        var asm = (AssemblyDoc)doc; var ext = doc.Extension; var sm = (SelectionMgr)doc.SelectionManager;
        // spec index -> component (insertion order = SW numbering per stem); verify by position
        var comps = new Component2[parts.Count]; var cnt = new Dictionary<string, int>();
        for (int i = 0; i < parts.Count; i++) {
            string st = Path.GetFileNameWithoutExtension((string)parts[i]["path"]); cnt[st] = (cnt.ContainsKey(st) ? cnt[st] : 0) + 1;
            comps[i] = (Component2)asm.GetComponentByName(st + "-" + cnt[st]);
            var m = Arr(parts[i]["m"]); var t = (double[])comps[i].Transform2.ArrayData;
            if (Math.Abs(m[9] - t[9]) + Math.Abs(m[10] - t[10]) + Math.Abs(m[11] - t[11]) > 1e-6)
                throw new Exception("component mapping mismatch at " + comps[i].Name2);
        }
        // masses, CM and inertia about CM (part frame, SI) for the inverse-dynamics torques
        double total = 0; var byStem = new Dictionary<string, double>();
        var massJson = new List<string>();
        for (int i = 0; i < parts.Count; i++) {
            var md = (ModelDoc2)comps[i].GetModelDoc2();
            var mp = (MassProperty)md.Extension.CreateMassProperty();
            mp.UseSystemUnits = true;
            double kg = mp.Mass; total += kg; byStem[(string)parts[i]["stem"]] = kg;
            var cm = (double[])mp.CenterOfMass; var I = (double[])mp.GetMomentOfInertia(0);
            massJson.Add("{\"i\":" + i + ",\"link\":\"" + parts[i]["link"] + "\",\"m\":" + F(kg) + ",\"cm\":[" + string.Join(",", cm.Select(F)) + "],\"I\":[" + string.Join(",", I.Select(x => x.ToString("E6", System.Globalization.CultureInfo.InvariantCulture))) + "]}");
        }
        Log("mass total = " + F(total * 1000) + " g; per part: " + string.Join(", ", byStem.Select(kv => kv.Key + "=" + F(kv.Value * 1000) + "g")));

        var mgr = (MotionStudyManager)ext.GetMotionStudyManager();
        string sname = "Phys_" + mot["scenario"];
        var names = mgr.GetMotionStudyNames() as object[];
        // never DeleteMotionStudy before CreateFeature: it crashed the COM server (RPC_E_SERVERFAULT, 2026-11-01)
        for (int k = 2; names != null && names.Cast<string>().Contains(sname); k++) sname = "Phys_" + mot["scenario"] + "_" + k;
        var ms = (MotionStudy)mgr.CreateMotionStudy();
        ms.Name = sname; ms.StudyType = 4; ms.Activate();
        var g = (SimulationGravityFeatureData)ms.CreateDefinition((int)swFeatureNameID_e.swFmAEMGravity);
        g.Axis = (int)swSimulationGravityAxisName_e.swSimulationGravityAxis_X; g.ReverseDirection = false;  // robot +x = down
        ms.CreateFeature(g);
        var motorFeat = new Dictionary<string, Feature>();
        foreach (Dictionary<string, object> mo in (object[])mot["motors"]) {
            string role = (string)mo["role"]; var j = joints[role];
            var ps = Arr(j["p_spigot"]);
            doc.ClearSelection2(true);
            if (!ext.SelectByID2("", "FACE", ps[0], ps[1], ps[2], false, 0, null, 0)) throw new Exception("no spigot face " + role);
            var face = sm.GetSelectedObject6(1, -1);
            doc.ClearSelection2(true);
            var md = (SimulationMotorFeatureData)ms.CreateDefinition((int)swFeatureNameID_e.swFmAEMRotationalMotor);
            md.Location = face;            // REQUIRED: without Location the motor drives nothing (2026-11-01)
            md.DirectionReference = face;
            md.ReverseDirection = (bool)mo["reverse"];
            md.RelativeComponent = comps[Convert.ToInt32(anchors[(string)j["stator"]])];
            md.DriveType = (int)swSimulationMotorDriveType_e.swSimulationMotorDrive_Displacement;
            md.MotionType = (int)swSimulationMotorMotionType_e.swSimulationMotorMotion_Function;
            md.Expression = (string)mo["expr"];
            var f = (Feature)ms.CreateFeature(md);
            if (f == null) throw new Exception("motor not created " + role + " expr=" + md.Expression);
            f.Name = "M_" + role; motorFeat[role] = f;
        }
        Log("motors: " + motorFeat.Count);
        ms.SetDuration(D(mot["duration"]));
        ((MotionStudyProperties)ms.GetProperties(4)).SetFrameRate(Convert.ToInt32(mot["fps"]));
        var t0 = DateTime.Now; bool calc = ms.Calculate();
        Log("calculate: " + calc + " in " + F((DateTime.Now - t0).TotalSeconds) + " s");

        var outp = new System.Text.StringBuilder("{\"calc\":" + calc.ToString().ToLower() + ",\"mass_g\":" + F(total * 1000)
            + ",\"parts\":[" + string.Join(",", massJson) + "],\"torque\":{");
        // NOTE: Motion plot API (CreatePlot*/InsertPlotFeature/GetValues) returns no data in SW2022 via interop
        // (8 recipes tried 2026-11-01) -> torques are computed by inverse dynamics in scripts/check_phys.py.
        var res = (CosmosMotionStudyResults)ms.GetResults(4);
        bool firstR = true;
        int variant = -1;
        foreach (var kv in motorFeat.Where(x => false)) {
            var tx = new List<string>(); var ty = new List<string>();
            for (int v = (variant < 0 ? 0 : variant); v < (variant < 0 ? 4 : variant + 1); v++) {
                tx.Clear(); ty.Clear();
                try {
                    var pd = (MotionPlotFeatureData)res.CreatePlotFeatureData();
                    var xa = (MotionPlotAxisFeatureData)res.CreatePlotXAxisFeatureData();
                    xa.Type = (int)swMotionPlotAxisType_e.swMotionPlotAxisType_XAXISTIME;
                    var ya = (MotionPlotAxisFeatureData)res.CreatePlotYAxisFeatureData();
                    ya.Type = (int)swMotionPlotAxisType_e.swMotionPlotAxisType_MOTION_APPLIED_TORQUE;
                    ya.Component = (int)swMotionPlotAxisComponent_e.swMotionPlotAxisComponent_MAGNITUDE;
                    object ent = (v == 3) ? kv.Value.GetSpecificFeature2() : (object)kv.Value;
                    ya.Entities = new object[] { ent };
                    if (v >= 1) pd.AddYAxis(ya);
                    object plot = null;
                    if (v >= 2) plot = res.InsertPlotFeature(pd, xa, ya);
                    var po = res.GetValues(pd, xa, ya) as MotionPlotFeatureOutput;
                    if (po != null) { Flatten(po.GetXAxis(), tx); Flatten(po.GetYAxis(ya), ty); }
                    if (ty.Count == 0 && plot != null) {                 // last resort: CSV of the inserted plot
                        string csv = Path.Combine(Path.GetDirectoryName(motionPath), "torque_" + kv.Key + ".csv");
                        if (res.ExportCSVFile(plot, csv) && File.Exists(csv))
                            foreach (var ln in File.ReadAllLines(csv).Skip(1)) {
                                var c = ln.Split(','); double a0, a1;
                                if (c.Length >= 2 && double.TryParse(c[0], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out a0)
                                    && double.TryParse(c[1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out a1)) { tx.Add(F(a0)); ty.Add(F(a1)); }
                            }
                    }
                    if (variant < 0) Log("  plot recipe v" + v + ": plot=" + (plot != null) + " nT=" + tx.Count + " nY=" + ty.Count);
                } catch (Exception ex) { if (variant < 0) Log("  plot recipe v" + v + " ERR " + ex.Message); }
                if (ty.Count > 0) { variant = v; break; }
            }
            if (!firstR) outp.Append(","); firstR = false;
            outp.Append("\"" + kv.Key + "\":{\"t\":[" + string.Join(",", tx) + "],\"y\":[" + string.Join(",", ty) + "]}");
        }
        outp.Append("},\"poses\":{");
        // poses of link anchors at sample times (for sign check / trajectories)
        bool firstT = true;
        foreach (var tt in ((object[])mot["times"]).Select(D)) {
            ms.SetTime(tt);
            if (!firstT) outp.Append(","); firstT = false;
            outp.Append("\"" + F(tt) + "\":{" + string.Join(",", anchors.Select(kv =>
                "\"" + kv.Key + "\":[" + string.Join(",", ((double[])comps[Convert.ToInt32(kv.Value)].Transform2.ArrayData).Take(12).Select(F)) + "]")) + "}");
        }
        outp.Append("}}");
        string resPath = Path.ChangeExtension(motionPath, null) + "_result.json";
        File.WriteAllText(resPath, outp.ToString());
        Log("results -> " + resPath);
        ms.SetTime(0);
        // video: SaveToAVI returned False (and may open a codec dialog that hangs COM) -> use `render` mode frames
        ms.SetTime(0);
        int se = 0, sw2 = 0;
        doc.Extension.SaveAs(asmPath, 0, (int)swSaveAsOptions_e.swSaveAsOptions_Silent, null, ref se, ref sw2);
        Log("assembly saved with study (err=" + se + ")");
        return calc ? 0 : 4;
    }

    static int GravTest(string asmPath) {
        int e = 0, w = 0;
        var doc = (ModelDoc2)app.OpenDoc6(asmPath, (int)swDocumentTypes_e.swDocASSEMBLY,
            (int)swOpenDocOptions_e.swOpenDocOptions_Silent, "", ref e, ref w);
        var asm = (AssemblyDoc)doc;
        var mgr = (MotionStudyManager)doc.Extension.GetMotionStudyManager();
        var ms = (MotionStudy)mgr.CreateMotionStudy();
        ms.Name = "GravTest"; ms.StudyType = 4; ms.Activate();
        var g = (SimulationGravityFeatureData)ms.CreateDefinition((int)swFeatureNameID_e.swFmAEMGravity);
        g.Axis = (int)swSimulationGravityAxisName_e.swSimulationGravityAxis_X;
        Log("gravity created: " + (ms.CreateFeature(g) != null) + " strength=" + g.Strength + " reverse=" + g.ReverseDirection);
        ms.SetDuration(0.6);
        bool calc = ms.Calculate();
        Log("calculate: " + calc);
        Log("duration=" + ms.GetDuration());
        var names = new[] { "Fist_TPU-1", "Fist_TPU-2", "Leg_Foot-1", "Leg_Foot-2", "Pelvis-1" };
        foreach (double t in new[] { 0.0, 0.2, 0.4, 0.6 }) {
            bool st = ms.SetTime(t);
            var line = new List<string>();
            foreach (var name in names) {
                var c = (Component2)asm.GetComponentByName(name);
                var b = (double[])c.GetBox(false, false);
                line.Add(string.Format(System.Globalization.CultureInfo.InvariantCulture, "{0}=({1:F0},{2:F0},{3:F0})",
                    name.Replace("_TPU", "").Replace("Leg_", ""), (b[0] + b[3]) * 500, (b[1] + b[4]) * 500, (b[2] + b[5]) * 500));
            }
            Log(string.Format(System.Globalization.CultureInfo.InvariantCulture, "  t={0:F1} setTime={1} gotTime={2:F2}  {3}", t, st, ms.GetTime(), string.Join("  ", line)));
        }
        return calc ? 0 : 4;
    }
}
