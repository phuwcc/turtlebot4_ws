#!/usr/bin/env bash
# Pi only: stop the stock service and this robot's launch and orphaned drivers.
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
tb4_robot="${1:-}"
case "$tb4_robot" in robot1|robot2|robot3) ;; *) echo 'Usage: bash scripts/robot-stop.sh robot1|robot2|robot3' >&2; exit 2;; esac
[[ -f /etc/turtlebot4/setup.bash ]] || { echo 'Chạy lệnh này trên Pi của robot.' >&2; exit 1; }
source /etc/turtlebot4/setup.bash
[[ "${ROBOT_NAMESPACE:-}" == "/$tb4_robot" ]] || { echo 'Robot ID không khớp namespace của Pi này.' >&2; exit 1; }
sudo systemctl stop turtlebot4
exec python3 "$tb4_repo/scripts/stop_hardware.py" "$tb4_robot"
