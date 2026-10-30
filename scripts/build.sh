#!/usr/bin/env bash
# Serial, low-priority build. Keep the desktop responsive during C++ compilation.
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$tb4_repo"
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
tb4_command=(colcon build --base-paths src --packages-select tb4_tools tb4_multi_tools tb4_multi_bringup
  --symlink-install --executor sequential
  --cmake-args -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG'
  -DBUILD_TESTING=ON "$@")
if [[ "${TB4_BUILD_RESOURCE_LIMITS:-1}" == 1 ]] && command -v systemd-run >/dev/null &&
   systemctl --user show-environment >/dev/null 2>&1; then
  echo 'Build tuần tự, ưu tiên thấp, CPU tối đa 75% một lõi và RAM tối đa 2,5 GiB (có thể cấu hình).'
  exec systemd-run --user --scope --quiet --collect \
    -p "CPUQuota=${TB4_BUILD_CPU_QUOTA:-75%}" \
    -p "MemoryHigh=${TB4_BUILD_MEMORY_HIGH:-2048M}" \
    -p "MemoryMax=${TB4_BUILD_MEMORY_MAX:-2560M}" \
    nice -n 10 ionice -c 3 "${tb4_command[@]}"
fi
echo 'Build tuần tự, nice=10; user cgroup không sẵn có nên không áp dụng giới hạn CPU/RAM.'
exec nice -n 10 ionice -c 3 "${tb4_command[@]}"
