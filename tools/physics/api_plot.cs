// Dump Motion plot-related interfaces and enums (no SolidWorks connection).
using System;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;

class A {
    static void Main() {
        foreach (var f in new[] { "SolidWorks.Interop.sldworks.dll", "SolidWorks.Interop.swmotionstudy.dll" }) {
            var asm = Assembly.LoadFrom(f);
            foreach (var t in asm.GetTypes().Where(x => x.IsInterface && Regex.IsMatch(x.Name, "^I.*(Plot|MotionResult)")).OrderBy(x => x.Name)) {
                Console.WriteLine("-- " + t.Name);
                foreach (var p in t.GetProperties()) Console.WriteLine("     P " + p.PropertyType.Name + " " + p.Name);
                foreach (var m in t.GetMethods().Where(m => !m.IsSpecialName)) Console.WriteLine("     M " + m.Name + "(" + string.Join(",", m.GetParameters().Select(p => p.ParameterType.Name + " " + p.Name)) + ")->" + m.ReturnType.Name);
            }
        }
        var c = Assembly.LoadFrom("SolidWorks.Interop.swconst.dll");
        foreach (var et in c.GetTypes().Where(x => x.IsEnum && Regex.IsMatch(x.Name, "Plot|MotionResult|Result.*Motion|Motion.*Result|MotionPlot")))
            Console.WriteLine("ENUM " + et.Name + ": " + string.Join(", ", Enum.GetNames(et).Select(n => n + "=" + Convert.ToInt32(Enum.Parse(et, n)))));
    }
}
