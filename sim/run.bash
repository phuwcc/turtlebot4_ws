#!/usr/bin/env bash
set -eo pipefail
sim_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$sim_dir/../install/tb4_marco_bridge/share/tb4_marco_bridge/local_setup.bash"
exec /usr/bin/python3 -B "$sim_dir/run.py" "$@"
