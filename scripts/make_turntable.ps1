# Turntable video/GIF of a viewer page (make_viewer.py output) for the phone:
# renders N frames with headless Edge (#turn=DEG), then ffmpeg -> MP4 + GIF.
# Usage: .\make_turntable.ps1 -Html "D:\AI prodject\grok\projects\robo-boxer\docs\viewer\robot_3d.html" [-Frames 36] [-W 540] [-H 860]
param(
    [Parameter(Mandatory = $true)][string]$Html,
    [int]$Frames = 36,
    [int]$W = 540,
    [int]$H = 860
)
$ErrorActionPreference = 'Stop'
$edge = 'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe'
if (-not (Test-Path $edge)) { $edge = 'C:\Program Files\Microsoft\Edge\Application\msedge.exe' }
$ffmpeg = Get-ChildItem "$env:LOCALAPPDATA\Microsoft\WinGet\Packages" -Recurse -Filter ffmpeg.exe -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty FullName
if (-not $ffmpeg) { throw 'ffmpeg not found (winget install Gyan.FFmpeg)' }
$base = [IO.Path]::Combine([IO.Path]::GetDirectoryName($Html), [IO.Path]::GetFileNameWithoutExtension($Html))
$dir = "$base`_frames"
New-Item -ItemType Directory -Force -Path $dir | Out-Null
Get-ChildItem $dir -Filter *.png -ErrorAction SilentlyContinue | Remove-Item -Force
$url = 'file:///' + ($Html -replace '\\', '/' -replace ' ', '%20')
$step = 360 / $Frames
for ($i = 0; $i -lt $Frames; $i++) {
    $deg = [int][Math]::Round($i * $step)
    $png = Join-Path $dir ('f{0:D3}.png' -f $i)
    $args = @('--headless=new', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', "--window-size=$W,$H",
              '--virtual-time-budget=15000', "--screenshot=`"$png`"", "$url#turn=$deg")
    Start-Process -FilePath $edge -ArgumentList $args -Wait -WindowStyle Hidden
    if (-not (Test-Path $png)) { throw "frame $i not rendered" }
    Write-Host ("frame {0}/{1} ({2} deg)" -f ($i + 1), $Frames, $deg)
}
$mp4 = "$base`_turntable.mp4"; $gif = "$base`_turntable.gif"
& $ffmpeg -y -loglevel error -framerate 12 -i "$dir\f%03d.png" -vf "scale=$W`:-2,format=yuv420p" -c:v libx264 -crf 23 -movflags +faststart $mp4
& $ffmpeg -y -loglevel error -framerate 12 -i "$dir\f%03d.png" -vf "scale=360:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse" -loop 0 $gif
Get-Item $mp4, $gif | Select-Object Name, @{n = 'KB'; e = { [int]($_.Length / 1KB) } }
