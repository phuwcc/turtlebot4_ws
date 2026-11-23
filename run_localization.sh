#!/usr/bin/env bash
# PC: start the fleet localization and RViz without Nav2 or velocity relay.
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
tb4_map="${1:-$tb4_repo/maps/room_real.yaml}"; if (( $# )); then shift; fi
exec bash "$tb4_repo/scripts/pc-start.sh" "$tb4_map" localization_only:=true "$@"
