# Rebuild v3 parts in batches with a SolidWorks restart between them (2026-11-16: one "all" run grew SW to 6.4 GB
# after 9 parts and sw_watch stopped it). Usage: v3_rebuild_batches.ps1 "shank shank_r foot" "foot_r torso" ...
param([Parameter(ValueFromRemainingArguments=$true)][string[]]$Batches)
$py = "D:\AI prodject\solidworks-mcp\SolidworksMCP-python\.venv\Scripts\python.exe"
$scr = "D:\AI prodject\robo-boxer\scripts"
$h = "D:\AI prodject\solidworks-mcp\SolidworksMCP-python\scripts"
$log = "D:\AI prodject\robo-boxer\cad\v3\logs\rebuild_wedge_1116b.log"
$env:ROBO_V = "3"; $env:PYTHONIOENCODING = "utf-8"
$i = 0
foreach ($b in $Batches) {
  $i++
  Write-Output "=== batch $i/$($Batches.Count): $b"
  Push-Location $h; & $py -c "import audit_harness as h; print(h.start_solidworks())" 2>$null; Pop-Location
  Push-Location $scr
  cmd /c "`"$py`" -u sw_watch.py --mem 6000 -- `"$py`" -u v3_parts.py $b >> `"$log`" 2>&1"
  Write-Output "batch $i rc $LASTEXITCODE"
  & $py sw_docs.py exit 2>$null
  Pop-Location
  Start-Sleep 5
}
Write-Output "=== done"
