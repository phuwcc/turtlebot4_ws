#!/usr/bin/env bash
# One entry point per laptop terminal; validate DDS before starting any ROS node.
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
tb4_mode="${1:-rviz}"
tb4_config="$HOME/.config/turtlebot4/pc_env.sh"
case "$tb4_mode" in
  configure) exec python3 "$tb4_repo/scripts/pc_config.py" ;;
  rviz|menu|check) ;;
  *) echo 'Usage: bash scripts/laptop.sh configure|rviz|menu|check' >&2; exit 2 ;;
esac
# Compatibility aliases; the PC navigation session already owns RViz and fleet control.
case "$tb4_mode" in
  rviz) exec bash "$tb4_repo/scripts/pc-rviz.sh" "${@:2}" ;;
  menu) exec bash "$tb4_repo/scripts/pc-menu.sh" "${@:2}" ;;
esac
if ! python3 "$tb4_repo/scripts/pc_config.py" --check; then
  echo 'Cần cấu hình lại số robot và IP trước khi chạy ROS.'
  python3 "$tb4_repo/scripts/pc_config.py"
fi
source /opt/ros/humble/setup.bash
if [[ ! -f "$tb4_repo/install/setup.bash" ]]; then
  echo "Chưa build. Chạy: bash $tb4_repo/scripts/build.sh" >&2
  exit 1
fi
source "$tb4_repo/install/setup.bash"
source "$tb4_config"
case "$tb4_mode" in
  check)
    ros2 daemon stop
    ros2 node list --no-daemon --spin-time 10
    for ((tb4_index=1; tb4_index<=TB4_ROBOT_COUNT; tb4_index++)); do
      ros2 topic info "/robot${tb4_index}/scan" --verbose --no-daemon --spin-time 5
      ros2 topic info "/robot${tb4_index}/odom" --verbose --no-daemon --spin-time 5
    done ;;
esac
