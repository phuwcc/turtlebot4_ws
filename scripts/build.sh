#!/usr/bin/env bash
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$tb4_repo"
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
exec colcon build --base-paths src --packages-select tb4_multi_tools tb4_multi_bringup \
  --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG' \
  -DBUILD_TESTING=ON "$@"
