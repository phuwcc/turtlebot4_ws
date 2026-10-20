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
  rviz)
    exec ros2 launch tb4_multi_bringup fleet_visualization.launch.py "robot_count:=$TB4_ROBOT_COUNT" ;;
  menu)
    echo "Phiên này dùng $TB4_ROBOT_COUNT robot. RViz phải chạy ở terminal khác."
    while read -r -p "Chọn robot [1-$TB4_ROBOT_COUNT], hủy goal [c], thoát [q]: " tb4_choice; do
      case "$tb4_choice" in
        1|2|3)
          if (( tb4_choice <= TB4_ROBOT_COUNT )); then
            ros2 param set /fleet_view selected_robot "robot${tb4_choice}"
          else
            echo 'Robot này không có trong phiên đang cấu hình.'
          fi ;;
        c) ros2 service call /tb4_view/cancel std_srvs/srv/Trigger '{}' ;;
        q) break ;;
        *) echo 'Nhập số robot, c hoặc q.' ;;
      esac
    done ;;
  check)
    ros2 daemon stop
    ros2 node list --no-daemon --spin-time 10
    for ((tb4_index=1; tb4_index<=TB4_ROBOT_COUNT; tb4_index++)); do
      ros2 topic info "/robot${tb4_index}/scan" --verbose --no-daemon --spin-time 5
      ros2 topic info "/robot${tb4_index}/odom" --verbose --no-daemon --spin-time 5
    done ;;
esac
