@echo off
cd /d "%~dp0"
echo Navigation console: http://127.0.0.1:8765/
echo Keep this window open. Motion remains locked until hardware validation.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0Start-WebConsole.ps1"
pause
