#!/usr/bin/env bash
# Read-only discovery. Does not activate nodes or send goals/services/velocity.
set -u
printf '\n=== identity ===\n'
hostname
date -Iseconds
printf '\n=== ROS environment configuration ===\n'
if [ -f /etc/mi/ros2_env.conf ]; then
  source /etc/mi/ros2_env.conf
fi
if ! command -v ros2 >/dev/null 2>&1; then
  for setup in /opt/ros2/cyberdog/setup.bash /opt/ros/galactic/setup.bash; do
    if [ -f "$setup" ]; then source "$setup"; fi
  done
fi
printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-UNSET}"
printf '\n=== nodes ===\n'
timeout 20 ros2 node list
printf '\n=== actions and types ===\n'
timeout 20 ros2 action list -t
printf '\n=== services and types ===\n'
timeout 20 ros2 service list -t
printf '\n=== topics and types ===\n'
timeout 20 ros2 topic list -t
printf '\n=== installed protocol definitions ===\n'
find /opt/ros2/cyberdog/share/protocol -maxdepth 2 -type f \( -name '*.action' -o -name '*.srv' \) -print 2>/dev/null
printf '\n=== custom web service status ===\n'
systemctl --user is-active cyberdog-web-remote.service || true
