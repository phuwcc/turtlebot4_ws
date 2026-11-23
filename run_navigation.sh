#!/usr/bin/env bash
# PC: start the fleet AMCL localization, EKF, Nav2 and RViz.
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$tb4_repo/scripts/pc-start.sh" "$@"
