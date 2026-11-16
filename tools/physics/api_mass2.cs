using System; using System.Linq; using System.Reflection;
class A { static void Main(){ var a=Assembly.LoadFrom("SolidWorks.Interop.sldworks.dll"); var t=a.GetType("SolidWorks.Interop.sldworks.IMassProperty");
 foreach(var m in t.GetMethods().Where(m=>m.Name.Contains("Moment")||m.Name.Contains("Inertia")||m.Name.Contains("Principal"))) Console.WriteLine("M "+m.Name+"("+string.Join(",",m.GetParameters().Select(p=>p.ParameterType.Name+" "+p.Name))+")->"+m.ReturnType.Name);
 foreach(var p in t.GetProperties()) Console.WriteLine("P "+p.Name);
 var c=Assembly.LoadFrom("SolidWorks.Interop.swconst.dll"); foreach(var et in c.GetTypes().Where(x=>x.IsEnum&&x.Name.Contains("MassPropertyMoment"))) Console.WriteLine(et.Name+": "+string.Join(", ",Enum.GetNames(et).Select(n=>n+"="+Convert.ToInt32(Enum.Parse(et,n))))); } }
