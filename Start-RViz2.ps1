$ErrorActionPreference = 'Stop'
$Project = 'D:\cyberdog\cyberdog_navigation'
$Viewer = Join-Path $Project 'tools\map_viewer.py'

$listener = Get-NetTCPConnection -LocalPort 8765 -State Listen -ErrorAction SilentlyContinue
if (-not $listener) {
    Start-Process -FilePath python `
        -ArgumentList '-B', $Viewer `
        -WorkingDirectory 'D:\cyberdog' `
        -WindowStyle Hidden
    Start-Sleep -Seconds 2
}

wsl --distribution Ubuntu-20.04 -- `
    bash /mnt/d/cyberdog/cyberdog_navigation/tools/start_rviz2.sh
