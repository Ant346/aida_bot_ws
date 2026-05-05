#!/bin/bash
# Minimal startup for GUI tools (skip colcon / rosdep on each launch).
set -e

if [[ -r /run/host-etc-hostname ]]; then
  _hn=$(head -n1 /run/host-etc-hostname | tr -d ' \t\r')
  [[ -n "$_hn" ]] && hostname "$_hn" 2>/dev/null || true
  unset _hn
fi

# shellcheck source=/entrypoint-x11.bash
[[ -f /entrypoint-x11.bash ]] && source /entrypoint-x11.bash

if [[ -f /opt/ros/${ROS_DISTRO:-humble}/setup.bash ]]; then
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
fi
if [[ -f /workspace/install/setup.bash ]]; then
  source /workspace/install/setup.bash
fi
exec "$@"
