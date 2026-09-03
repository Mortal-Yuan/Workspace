param(
    [string]$Port = 'COM3'
)

$ErrorActionPreference = 'Stop'
$scriptPath = Join-Path $PSScriptRoot 'usb_camera_monitor.py'
$pythonCandidates = @(
    'C:\Espressif\tools\python\v5.4.4\venv\Scripts\python.exe',
    'python.exe'
)
$python = $null
foreach ($candidate in $pythonCandidates) {
    try {
        & $candidate -c 'import serial, tkinter' 2>$null
        if ($LASTEXITCODE -eq 0) {
            $python = $candidate
            break
        }
    } catch {
        continue
    }
}
if (-not $python) {
    throw 'Python with both pyserial and tkinter is required.'
}

& $python $scriptPath --port $Port
exit $LASTEXITCODE
