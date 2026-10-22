#!/usr/bin/env bash
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$tb4_repo/scripts/pc.sh" check "$@"
