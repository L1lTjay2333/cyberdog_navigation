$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
$taskPython = Join-Path $PSScriptRoot '..\..\tools\reconstruction_runtime\.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $taskPython)) { $taskPython = (Get-Command python -ErrorAction Stop).Source }
try {
    $null = Invoke-RestMethod 'http://127.0.0.1:8765/state' -TimeoutSec 2
    Write-Host 'Web console is already running: http://127.0.0.1:8765/'
    return
} catch { }
Write-Host 'Open http://127.0.0.1:8765/ in your browser. Keep this window open.'
& $taskPython -B tools/map_viewer.py
