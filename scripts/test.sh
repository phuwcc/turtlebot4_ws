#!/usr/bin/env bash
# Tests enforce isolated localhost ROS domains; no production robot commands.
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$tb4_repo/install/setup.bash"
cd "$tb4_repo"
python3 tests/test_pc_config.py -v
python3 src/tb4_multi_bringup/test/test_fleet_view.py -v
python3 src/tb4_multi_bringup/test/test_remote_control.py -v
python3 tests/smoke_navigation.py --pc robot1
python3 tests/smoke_fleet_localization.py
