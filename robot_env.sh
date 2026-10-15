#!/usr/bin/env bash
# Source this file in each Pi terminal to use the robot's DDS configuration.
tb4_env_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /etc/turtlebot4/setup.bash
source "${tb4_env_workspace}/install/setup.bash"
export ROS_SUPER_CLIENT=True
