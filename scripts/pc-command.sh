#!/usr/bin/env bash
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$tb4_repo/scripts/socket_client.py" "$@"
