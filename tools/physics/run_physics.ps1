# SOLIDWORKS Motion physics run for robo-boxer (needs SolidWorks 2022 running, COSMOSMotion licence).
# Steps: compile tool -> servo copy with datasheet mass -> spec -> flat assembly -> motor profiles -> Motion Analysis -> check.
# Usage: .\run_physics.ps1 [-Scenario punch] [-NoBuild]
param([string]$Scenario = "punch", [switch]$NoBuild)
$ErrorActionPreference = "Stop"
$T = $PSScriptRoot
$P = Split-Path (Split-Path $T)
$py = "D:\AI prodject\solidworks-mcp\SolidworksMCP-python\.venv\Scripts\python.exe"
$fw = "C:\Windows\Microsoft.NET\Framework64\v4.0.30319"
$r = "C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist"
$phys = Join-Path $P "cad\physics"
New-Item -ItemType Directory -Force $phys | Out-Null
& $py (Join-Path $T "sw_guard.py") pre      # memory control: start SW / graceful restart if idle and bloated
Copy-Item "$r\SolidWorks.Interop.sldworks.dll", "$r\SolidWorks.Interop.swconst.dll", "$r\SolidWorks.Interop.swmotionstudy.dll" $T -Force
& "$fw\csc.exe" /nologo /platform:x64 /out:"$T\build_physics.exe" /r:"$T\SolidWorks.Interop.sldworks.dll" /r:"$T\SolidWorks.Interop.swconst.dll" /r:"$T\SolidWorks.Interop.swmotionstudy.dll" /r:"$fw\System.Web.Extensions.dll" /r:"$fw\Microsoft.VisualBasic.dll" "$T\build_physics.cs"
if ($LASTEXITCODE) { throw "compile failed" }
$asm = Join-Path $phys "Robot_Physics.SLDASM"
if (-not $NoBuild) {
    $servo = Join-Path $phys "MG90S_phys.SLDPRT"
    Copy-Item (Join-Path $P "cad\purchased\MG90S_grabcad.SLDPRT") $servo -Force
    & "$T\build_physics.exe" massfix $servo 13.4          # TowerPro: 13.4 g
    if ($LASTEXITCODE) { throw "massfix failed" }
    & $py (Join-Path $P "scripts\gen_phys_spec.py")
    if ($LASTEXITCODE) { throw "spec failed" }
    if (Test-Path $asm) { Remove-Item $asm }
    & "$T\build_physics.exe" build (Join-Path $phys "phys_spec.json") $asm
    if ($LASTEXITCODE) { throw "build failed" }
}
# The axis sense SolidWorks takes from the spigot face is not stable between rebuilds ->
# calibrate on THIS assembly: run, compare achieved vs commanded angles, flip, rerun (max 3 passes).
$rev = Join-Path $phys "motor_reverse.json"; $sug = Join-Path $phys "motor_reverse_suggested.json"
for ($pass = 1; $pass -le 3; $pass++) {
    & $py (Join-Path $P "scripts\gen_phys_motion.py") $Scenario | Out-Null
    & "$T\build_physics.exe" run $asm (Join-Path $phys "motion_$Scenario.json")
    if ($LASTEXITCODE) { throw "motion run failed" }
    $chk = & $py (Join-Path $P "scripts\check_phys.py") (Join-Path $phys "motion_$Scenario.json")
    $chk
    if (-not ($chk -match "!! FLIP")) { "[calib] signs ok after pass $pass"; break }
    Copy-Item $sug $rev -Force; "[calib] pass $pass -> flipping and rerunning"
}
& $py (Join-Path $T "sw_guard.py") post     # memory control after the run
