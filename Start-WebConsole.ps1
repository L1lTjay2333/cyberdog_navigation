$ErrorActionPreference = 'Stop'
$taskPython = Join-Path $PSScriptRoot '..\..\tools\reconstruction_runtime\.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $taskPython)) { throw 'Missing reconstruction runtime.' }
Start-Process -WindowStyle Hidden -FilePath $taskPython -ArgumentList '-B','tools/web_supervisor.py' -WorkingDirectory $PSScriptRoot
Write-Host 'CyberDog workspace: http://127.0.0.1:8765/'
Write-Host 'Services run in the background. No capture or motion starts automatically.'
