#!/usr/bin/env bash
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
usage() {
  echo 'Usage: bash scripts/robot.sh robot1|robot2|robot3 hardware|teleop|check [launch arguments]'
  echo 'Navigation/mapping chạy trên PC qua scripts/pc-*.sh.'
}
if [[ $# -lt 2 ]]; then usage; exit 2; fi
tb4_id="$1"; tb4_mode="$2"; shift 2
case "$tb4_id" in robot1|robot2|robot3) ;; *) usage; exit 2;; esac
source "${tb4_repo}/scripts/env.sh"
tb4_state="${XDG_STATE_HOME:-$HOME/.local/state}/tb4_multi_robot/${tb4_id}"
mkdir -p "$tb4_state"
export ROS_LOG_DIR="${tb4_repo}/log/${tb4_id}/${tb4_mode}"
case "$tb4_mode" in
  hardware)
    if pgrep -f '^/[^ ]*/lib/(rplidar_ros/rplidar_composition|create3_republisher/create3_republisher|turtlebot4_node/turtlebot4_node)([[:space:]]|$)' >/dev/null; then
      echo 'Hardware is already running on this Pi. Stop its old bringup before starting this profile.' >&2
      exit 1
    fi
    [[ -e /dev/RPLIDAR ]] || { echo '/dev/RPLIDAR is missing' >&2; exit 1; }
    exec flock -n "$(dirname "$tb4_state")/hardware.lock" \
      ros2 launch tb4_multi_bringup hardware.launch.py "robot_id:=$tb4_id" "$@";;
  mapping|navigation|rviz)
    echo 'Chế độ này đã chuyển sang PC. Dùng scripts/pc-start.sh, pc-mapping.sh hoặc pc-rviz.sh.' >&2
    exit 2;;
  teleop)
    exec ros2 run tb4_multi_tools teleop_keyboard --ros-args -r "__ns:=/$tb4_id" -p remote:=true "$@";;
  check)
    export ROS_SUPER_CLIENT=True
    ros2 node list --no-daemon --spin-time 5
    ros2 topic info "/$tb4_id/scan" --verbose --no-daemon --spin-time 5
    ros2 topic info "/$tb4_id/odom" --verbose --no-daemon --spin-time 5;;
  *) usage; exit 2;;
esac
