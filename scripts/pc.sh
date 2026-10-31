#!/usr/bin/env bash
# PC only. Every mode loads its own ROS/DDS environment.
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
tb4_mode="${1:-nav}"; shift || true
case "$tb4_mode" in
  configure) exec python3 "$tb4_repo/scripts/pc_config.py" "$@" ;;
  menu) exec python3 "$tb4_repo/scripts/socket_client.py" menu "$@" ;;
  check) exec bash "$tb4_repo/scripts/laptop.sh" check "$@" ;;
  nav|mapping|save-map|teleop|rviz|diagnose|hardware-test) ;;
  *) echo 'Usage: pc.sh configure|nav|mapping|save-map|teleop|menu|check|rviz|diagnose|hardware-test' >&2; exit 2 ;;
esac
if ! python3 "$tb4_repo/scripts/pc_config.py" --check; then
  python3 "$tb4_repo/scripts/pc_config.py"
fi
source /opt/ros/humble/setup.bash
[[ -f "$tb4_repo/install/setup.bash" ]] || { echo 'Chạy bash scripts/build.sh trước.' >&2; exit 1; }
source "$tb4_repo/install/setup.bash"
source "$HOME/.config/turtlebot4/pc_env.sh"
if [[ -f "$HOME/.config/turtlebot4/runtime_env.sh" ]]; then
  source "$HOME/.config/turtlebot4/runtime_env.sh"
fi
if [[ -n "${TB4_TF2_RUNTIME_LIB:-}" ]]; then
  [[ -f "$TB4_TF2_RUNTIME_LIB/libtf2.so" && -f "$TB4_TF2_RUNTIME_LIB/libtf2_ros.so" ]] || {
    echo 'TB4_TF2_RUNTIME_LIB thiếu libtf2.so hoặc libtf2_ros.so.' >&2; exit 1;
  }
  export LD_LIBRARY_PATH="$TB4_TF2_RUNTIME_LIB:${LD_LIBRARY_PATH:-}"
fi
if [[ -z "${TB4_ROBOT_IDS:-}" ]]; then
  tb4_ids=()
  for ((tb4_i=1; tb4_i<=TB4_ROBOT_COUNT; tb4_i++)); do tb4_ids+=("robot$tb4_i"); done
  TB4_ROBOT_IDS="$(IFS=,; echo "${tb4_ids[*]}")"
  export TB4_ROBOT_IDS
fi
tb4_state="${XDG_STATE_HOME:-$HOME/.local/state}/tb4_multi_robot/pc"
mkdir -p "$tb4_state"
export ROS_LOG_DIR="$tb4_repo/log/pc/$tb4_mode"
# Prevent nav/mapping/teleop or a second fleet controller from overlapping on this PC.
case "$tb4_mode" in nav|mapping|teleop|rviz|hardware-test)
  exec 9>"$tb4_state/session.lock"
  flock -n 9 || { echo 'PC đang chạy nav/mapping/teleop/RViz. Dừng phiên đó trước.' >&2; exit 1; } ;;
esac
case "$tb4_mode" in
  hardware-test)
    IFS=, read -r -a tb4_robots <<< "$TB4_ROBOT_IDS"
    exec python3 "$tb4_repo/tests/hardware_acceptance.py" --robots "${tb4_robots[@]}" \
      --output "$ROS_LOG_DIR/result.json" "$@" ;;
  diagnose)
    exec python3 "$tb4_repo/scripts/pc_diagnose.py" "$@" ;;
  nav)
    tb4_map="${1:-$tb4_repo/maps/site.yaml}"; if (( $# )); then shift; fi
    python3 "$tb4_repo/scripts/pc_diagnose.py" --preflight --map "$tb4_map"
    exec ros2 launch tb4_multi_bringup pc_navigation.launch.py \
      "robot_count:=$TB4_ROBOT_COUNT" "map:=$tb4_map" "$@" ;;
  mapping|teleop)
    tb4_robot="${1:-${TB4_ROBOT_IDS%%,*}}"; if (( $# )); then shift; fi
    case "$tb4_robot" in robot1|robot2|robot3) ;; *) echo 'Cần robot1, robot2 hoặc robot3.' >&2; exit 2;; esac
    [[ ",$TB4_ROBOT_IDS," == *",$tb4_robot,"* ]] || { echo 'Robot không có trong phiên.' >&2; exit 2; }
    if [[ "$tb4_mode" == teleop ]]; then
      exec ros2 run tb4_multi_tools teleop_keyboard --ros-args -r "__ns:=/$tb4_robot" -p remote:=true "$@"
    fi
    # Mapping uses the same PC session; teleop runs on the Pi in a separate terminal.
    exec ros2 launch tb4_multi_bringup pc_mapping.launch.py "robot_id:=$tb4_robot" "robot_count:=$TB4_ROBOT_COUNT" \
      "origin_file:=$tb4_state/${tb4_robot}_mapping_origin.json" "$@" ;;
  save-map)
    tb4_robot="${1:-${TB4_ROBOT_IDS%%,*}}"; tb4_map="${2:-$tb4_repo/maps/site}"
    case "$tb4_robot" in robot1|robot2|robot3) ;; *) exit 2;; esac
    [[ ",$TB4_ROBOT_IDS," == *",$tb4_robot,"* ]] || exit 2
    mkdir -p "$(dirname "$tb4_map")"
    exec ros2 run nav2_map_server map_saver_cli -f "$tb4_map" --ros-args \
      -r "__ns:=/$tb4_robot" -r "map:=/$tb4_robot/map" -p save_map_timeout:=15.0 ;;
  rviz)
    exec ros2 launch tb4_multi_bringup fleet_visualization.launch.py \
      "robot_count:=$TB4_ROBOT_COUNT" socket:=true "$@" ;;
esac
