#!/usr/bin/env bash
set -e
tb4_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${tb4_workspace}/robot_env.sh"
# Super-client is for graph inspection, not the application participants.
export ROS_SUPER_CLIENT=False
export ROS_LOG_DIR="${tb4_workspace}/log/localization"
if pgrep -f '^/[^ ]*/lib/nav2_amcl/amcl([[:space:]]|$)' > /dev/null ||
  pgrep -f '^/[^ ]*/lib/slam_toolbox/[^ ]*slam_toolbox_node([[:space:]]|$)' > /dev/null ||
  pgrep -f '^/[^ ]*/lib/tb4_tools/start_relative_mapping([[:space:]]|$)' > /dev/null; then
  echo 'AMCL or SLAM is already running. Stop that session before localization.' >&2
  exit 1
fi
exec ros2 launch turtlebot4_navigation localization.launch.py \
  "map:=${tb4_workspace}/maps/my_room.yaml" use_sim_time:=false \
  set_initial_pose:=true initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0 "$@"
