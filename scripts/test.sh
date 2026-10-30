#!/usr/bin/env bash
# Tests enforce isolated localhost ROS domains; no production robot commands.
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$tb4_repo/install/setup.bash"
cd "$tb4_repo"
colcon test --packages-select tb4_tools tb4_multi_tools tb4_multi_bringup --event-handlers console_direct+
colcon test-result --verbose
python3 tests/test_pc_config.py -v
python3 tests/test_diagnostics.py -v
python3 tests/test_hardware_stop.py -v
python3 src/tb4_multi_bringup/test/test_fleet_view.py -v
python3 src/tb4_multi_bringup/test/test_remote_control.py -v
python3 src/tb4_multi_bringup/test/test_native_runtime.py -v
python3 tests/smoke_navigation.py --pc robot1
python3 tests/smoke_navigation.py --pc --localization-only robot1
python3 tests/smoke_fleet_localization.py
python3 tests/smoke_navigation.py robot2
python3 tests/smoke_navigation.py robot3
python3 tests/smoke_two_robots.py
