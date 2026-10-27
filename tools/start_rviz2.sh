#!/usr/bin/env bash
source /opt/ros/galactic/setup.bash
set -eo pipefail
export QT_QPA_PLATFORM=xcb
export QT_OPENGL=software
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe

evidence=/mnt/d/cyberdog/cyberdog_navigation/evidence
python3 /mnt/d/cyberdog/cyberdog_navigation/tools/rviz_map_bridge.py \
  >>"$evidence/rviz_map_bridge.log" 2>&1 &
bridge_pid=$!

cleanup() {
  kill "$bridge_pid" 2>/dev/null || true
  wait "$bridge_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

rviz2 -d /mnt/d/cyberdog/cyberdog_navigation/config/live_map.rviz \
  >"$evidence/rviz2.log" 2>&1
