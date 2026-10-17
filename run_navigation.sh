#!/usr/bin/env bash
set -e
tb4_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${tb4_workspace}/robot_env.sh"
export ROS_SUPER_CLIENT=False
export ROS_LOG_DIR="${tb4_workspace}/log/nav2"
if pgrep -f '^/[^ ]*/lib/nav2_(controller/controller_server|planner/planner_server|bt_navigator/bt_navigator|behaviors/behavior_server|smoother/smoother_server|waypoint_follower/waypoint_follower|velocity_smoother/velocity_smoother)([[:space:]]|$)' > /dev/null ||
  pgrep -f '^/[^ ]*/lib/rclcpp_components/component_container[^ ]* .*__node:=nav2_container([[:space:]]|$)' > /dev/null; then
  echo 'Nav2 is already running. Stop that session before starting another.' >&2
  exit 1
fi
tb4_tools_prefix="$(ros2 pkg prefix tb4_tools)"
exec ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=false \
  use_composition:=False \
  "params_file:=${tb4_tools_prefix}/share/tb4_tools/config/nav2_pi.yaml" "$@"
