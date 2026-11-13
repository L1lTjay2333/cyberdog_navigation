$ErrorActionPreference = 'Stop'
$consoleUrl = 'http://127.0.0.1:8765/'

function Test-ConsoleReady {
    try {
        $response = Invoke-WebRequest -Uri $consoleUrl -UseBasicParsing -TimeoutSec 2
        return ($response.StatusCode -eq 200 -and $response.Content.Contains('id="home-folders"'))
    } catch {
        return $false
    }
}

$taskPython = Join-Path $PSScriptRoot '..\..\tools\reconstruction_runtime\.venv\Scripts\python.exe'
if (-not (Test-ConsoleReady)) {
    if (-not (Test-Path -LiteralPath $taskPython -PathType Leaf)) {
        throw "CyberDog Python runtime not found: $taskPython"
    }
    New-Item -ItemType Directory -Path (Join-Path $PSScriptRoot 'evidence') -Force | Out-Null
    Write-Host 'Starting CyberDog workspace...'
    Start-Process -WindowStyle Hidden -FilePath $taskPython -ArgumentList '-B','tools/web_supervisor.py' -WorkingDirectory $PSScriptRoot

    $deadline = (Get-Date).AddSeconds(20)
    while (-not (Test-ConsoleReady)) {
        if ((Get-Date) -ge $deadline) {
            throw "CyberDog workspace did not respond at $consoleUrl. Check evidence\service_8765.log."
        }
        Start-Sleep -Milliseconds 300
    }
}
Start-Process -FilePath $consoleUrl
