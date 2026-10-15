#!/usr/bin/env bash
set -e

tb4_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${tb4_workspace}/robot_env.sh"
export ROS_SUPER_CLIENT=False
export ROS_LOG_DIR="${tb4_workspace}/log/slam"

if pgrep -f '^/[^ ]*/lib/slam_toolbox/[^ ]*slam_toolbox_node([[:space:]]|$)' > /dev/null; then
  echo 'SLAM is already running on this Pi. Stop that session before starting another.' >&2
  exit 1
fi

tb4_tools_prefix="$(ros2 pkg prefix tb4_tools)"
exec "${tb4_tools_prefix}/lib/tb4_tools/start_relative_mapping" \
  --ros-args -p "origin_file:=${tb4_workspace}/log/mapping_start_origin.json" "$@"
