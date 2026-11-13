#!/usr/bin/env bash
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 -B "$tb4_repo/scripts/real_robot.py" "$@"
