#!/usr/bin/env bash
# Pi only: replace stock bringup with repository hardware + command watchdog.
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
tb4_robot="${1:-}"
case "$tb4_robot" in robot1|robot2|robot3) ;; *) echo 'Usage: bash scripts/robot-start.sh robot1|robot2|robot3' >&2; exit 2;; esac
source "$tb4_repo/scripts/env.sh"
if [[ "${ROBOT_NAMESPACE:-}" != "/$tb4_robot" ]]; then
  echo "ROBOT_NAMESPACE=${ROBOT_NAMESPACE:-unset}, không khớp /$tb4_robot. Sửa turtlebot4-setup trước." >&2
  exit 1
fi
[[ -e /dev/RPLIDAR ]] || { echo '/dev/RPLIDAR is missing' >&2; exit 1; }
ros2 pkg prefix tb4_multi_bringup >/dev/null
sudo systemctl disable --now turtlebot4
exec bash "$tb4_repo/scripts/robot.sh" "$tb4_robot" hardware
