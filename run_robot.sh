#!/usr/bin/env bash
set -eo pipefail
tb4_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$tb4_root/scripts/run.py" robot "$@"
