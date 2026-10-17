#!/usr/bin/env bash
# Source on a Pi, or configure the PC discovery environment before sourcing.
tb4_multi_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f /etc/turtlebot4/setup.bash ]]; then
  source /etc/turtlebot4/setup.bash
fi
if [[ ! -f "${tb4_multi_root}/install/setup.bash" ]]; then
  echo 'Build this repo first: bash scripts/build.sh' >&2
  return 1
fi
source "${tb4_multi_root}/install/setup.bash"
if ! ros2 pkg prefix tb4_multi_bringup >/dev/null 2>&1; then
  echo "Build from ${tb4_multi_root}: bash scripts/build.sh" >&2
  return 1
fi
export ROS_SUPER_CLIENT=False
export ROS_LOCALHOST_ONLY=0
