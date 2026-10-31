#!/usr/bin/env bash
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$tb4_repo/install/setup.bash"
export PYTHONDONTWRITEBYTECODE=1
export MARCO_ROOT="${MARCO_ROOT:-$tb4_repo/MARCO}"
cd "$tb4_repo"
# Prevent two local mission dispatchers from competing for the same robot leases.
tb4_state="${XDG_STATE_HOME:-$HOME/.local/state}/tb4_multi_robot/pc"
mkdir -p "$tb4_state"
if [[ "${1:-}" == run ]]; then
  exec 8>"$tb4_state/marco.lock"
  flock -n 8 || { echo 'MARCO mission đã chạy trên PC.' >&2; exit 1; }
fi
exec /usr/bin/python3 -B "$tb4_repo/install/tb4_marco_bridge/lib/tb4_marco_bridge/marco" "$@"
