#!/usr/bin/env bash
set -eo pipefail
sim_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
if [[ -n "${MARCO_TB4_WORKSPACE:-}" ]]; then
  source "$MARCO_TB4_WORKSPACE/environment.sh"
fi
source "$sim_dir/../install/tb4_marco_bridge/share/tb4_marco_bridge/local_setup.bash"
export TB4_SIM_MAPPING=0 TB4_SIM_GUI=0
while (( $# )); do
  case "$1" in
    --mapping) export TB4_SIM_MAPPING=1 ;;
    --gui) export TB4_SIM_GUI=1 ;;
    *) break ;;
  esac
  shift
done
missing=()
command -v ign >/dev/null || missing+=(ign)
packages=(turtlebot4_ignition_bringup irobot_create_ignition_bringup irobot_create_control ros_ign_gazebo ros_gz_bridge rviz2)
if [[ "$TB4_SIM_MAPPING" == 1 ]]; then
  packages+=(slam_toolbox nav2_map_server)
fi
for package in "${packages[@]}"; do
  ros2 pkg prefix "$package" >/dev/null 2>&1 || missing+=("$package")
done
if (( ${#missing[@]} )); then
  echo "Thiếu gói mô phỏng: ${missing[*]}" >&2
  exit 1
fi
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/tb4-marco-gazebo-$UID.lock"
flock -n 9 || { echo 'Đã có một phiên Gazebo MARCO đang chạy.' >&2; exit 1; }
export IGN_PARTITION="marco-t-sim-$$"
export TB4_SIM_OUTPUT="$(mktemp -d /tmp/marco-gazebo.XXXXXXXX)"
# Keep the lock in the launch process as well, even if this shell is killed.
setsid ros2 launch "$sim_dir/gazebo.launch.py" >"$TB4_SIM_OUTPUT/launch.log" 2>&1 &
launch_pid=$!
cleanup() {
  if [[ -n "${mission_pid:-}" ]]; then
    kill -TERM "$mission_pid" 2>/dev/null || true
    wait "$mission_pid" 2>/dev/null || true
  fi
  # Signal the launch leader first so ROS can shut down its children in order.
  kill -INT "$launch_pid" 2>/dev/null || true
  for ((attempt=0; attempt<50; attempt++)); do
    pgrep -s "$launch_pid" >/dev/null || break
    sleep .2
  done
  # Gazebo GUI and server use separate process groups in the launch session.
  pkill -KILL -s "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
echo "Gazebo/RViz log: $TB4_SIM_OUTPUT/launch.log"
exec_status=0
exec 8<&0
sim_program=gazebo_mission.py
if [[ "$TB4_SIM_MAPPING" == 1 ]]; then
  sim_program=mapping_teleop.py
fi
/usr/bin/python3 -u -B "$sim_dir/$sim_program" "$@" <&8 9>&- &
mission_pid=$!
wait "$mission_pid" || exec_status=$?
mission_pid=
exit "$exec_status"
