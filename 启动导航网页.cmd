@echo off
cd /d "%~dp0"
echo Navigation console: http://127.0.0.1:8765/
echo Keep this window open. Motion remains locked until hardware validation.
python -B tools\map_viewer.py
pause
