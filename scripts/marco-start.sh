#!/usr/bin/env bash
# Start a MARCO mission in the running PC session (multi_robot.bash --marco).
# Usage: bash marco_start.bash config/marco/MISSION.json
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
tb4_mission="${1:-}"
[[ -n "$tb4_mission" ]] || { echo 'Usage: bash marco_start.bash MISSION.json' >&2; exit 2; }
tb4_mission="$(realpath -- "$tb4_mission")"
python3 -c 'import json,sys; m=json.load(open(sys.argv[1])); assert isinstance(m.get("tasks"),list) and m["tasks"], "mission needs tasks"' "$tb4_mission" \
  || { echo "Mission không hợp lệ: $tb4_mission" >&2; exit 2; }
source /opt/ros/humble/setup.bash
source "$tb4_repo/install/setup.bash"
source "$HOME/.config/turtlebot4/pc_env.sh"
# The coordinator discovers through the PC-local hub started by scripts/pc.sh.
grep -q ':11888 ' <<<"$(ss -lun 2>/dev/null)" || { echo 'Chưa có phiên MARCO trên PC: chạy multi_robot.bash --marco trước.' >&2; exit 1; }
export ROS_DISCOVERY_SERVER="$(python3 -c 'from tb4_multi_bringup.fleet_settings import discovery_hub_client; print(discovery_hub_client())')"
export ROS_SUPER_CLIENT=False
unset FASTRTPS_DEFAULT_PROFILES_FILE
echo "Gửi mission tới MARCO coordinator: $tb4_mission"
timeout 30 ros2 topic pub --once -w 1 /tb4_view/marco/start std_msgs/msg/String "{data: '$tb4_mission'}" >/dev/null \
  || { echo 'Không tìm thấy coordinator MARCO (marco_rviz) trong 30 giây.' >&2; exit 1; }
echo 'Đã gửi. Theo dõi: log/pc/marco_rviz/status.json hoặc nhãn MARCO trên RViz.'
