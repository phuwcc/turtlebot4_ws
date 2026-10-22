#!/usr/bin/env bash
# Install once on each host; no driver/service is started by this script.
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:-}" in
  pc) tb4_packages=(ros-humble-rviz2 ros-humble-nav2-bringup ros-humble-navigation2
                   ros-humble-slam-toolbox ros-humble-turtlebot4-description) ;;
  robot) tb4_packages=(ros-humble-turtlebot4-bringup ros-humble-create3-republisher) ;;
  *) echo 'Usage: bash scripts/setup.sh pc|robot' >&2; exit 2 ;;
esac
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-yaml \
  ros-humble-rmw-fastrtps-cpp ros-humble-nav2-common "${tb4_packages[@]}"
exec bash "$tb4_repo/scripts/build.sh"
