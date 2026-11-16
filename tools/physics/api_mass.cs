using System; using System.Linq; using System.Reflection;
class A { static void Main(){ var a=Assembly.LoadFrom("SolidWorks.Interop.sldworks.dll");
 foreach(var n in new[]{"IMassProperty2","IMassProperty"}){ var t=a.GetType("SolidWorks.Interop.sldworks."+n); if(t==null){Console.WriteLine("no "+n);continue;}
  Console.WriteLine("-- "+n); foreach(var p in t.GetProperties()) if(p.Name.Contains("Override")||p.Name.Contains("Mass")) Console.WriteLine("  P "+p.PropertyType.Name+" "+p.Name+" set="+p.CanWrite);
  foreach(var m in t.GetMethods().Where(m=>!m.IsSpecialName&&(m.Name.Contains("Override")||m.Name.Contains("Mass")))) Console.WriteLine("  M "+m.Name+"("+string.Join(",",m.GetParameters().Select(p=>p.ParameterType.Name))+")"); }
 var e=a.GetType("SolidWorks.Interop.sldworks.IModelDocExtension"); foreach(var m in e.GetMethods().Where(m=>m.Name.Contains("MassProperty"))) Console.WriteLine("ext."+m.Name); } }
