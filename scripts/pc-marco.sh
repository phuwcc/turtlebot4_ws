#!/usr/bin/env bash
# Start localization + Nav2 + socket/RViz. No mission or motion is started here.
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
tb4_map="${1:-$tb4_repo/maps/my_room.yaml}"; if (( $# )); then shift; fi
tb4_command=(bash "$tb4_repo/scripts/pc-start.sh" "$tb4_map" managed_startup:=true \
  "localization_hints:=$tb4_repo/config/marco/localization.yaml" \
  "nav_params_file:=$tb4_repo/config/marco/nav2.yaml" "$@")
if [[ "${TB4_PC_RESOURCE_LIMITS:-1}" == 1 ]] && command -v systemd-run >/dev/null && \
   systemctl --user show-environment >/dev/null 2>&1; then
  echo 'MARCO/ROS session: CPU budget 200% (configurable), memory high 1536M/max 2048M.'
  exec systemd-run --user --scope --quiet --collect \
    -p "CPUQuota=${TB4_PC_CPU_QUOTA:-200%}" \
    -p "MemoryHigh=${TB4_PC_MEMORY_HIGH:-1536M}" \
    -p "MemoryMax=${TB4_PC_MEMORY_MAX:-2048M}" \
    nice -n 5 "${tb4_command[@]}"
fi
echo 'User cgroup unavailable or limits disabled; using nice=5 without a CPU/RAM cap.'
exec nice -n 5 "${tb4_command[@]}"
