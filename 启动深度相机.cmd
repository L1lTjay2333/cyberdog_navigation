@echo off
cd /d "%~dp0"
python -B tools/probe_depth_camera_once.py --keep-active
pause
