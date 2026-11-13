#!/usr/bin/env bash
# Pi only: persistent hardware bringup, including display/LED and velocity guard.
set -eo pipefail
tb4_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
tb4_robot="${1:-}"
tb4_action="${2:-status}"
case "$tb4_robot" in robot1|robot2|robot3) ;; *) echo 'Usage: robot-service.sh robot1|robot2|robot3 install|start|stop|status|disable' >&2; exit 2;; esac
case "$tb4_action" in install|start|stop|status|disable) ;; *) echo 'Unknown service action' >&2; exit 2;; esac
[[ -f /etc/turtlebot4/setup.bash ]] || { echo 'Run this on the robot Pi.' >&2; exit 1; }
source /etc/turtlebot4/setup.bash
[[ "${ROBOT_NAMESPACE:-}" == "/$tb4_robot" ]] || { echo 'Robot ID does not match this Pi.' >&2; exit 1; }
tb4_unit=tb4-hardware.service
case "$tb4_action" in
  install)
    source "$tb4_repo/scripts/env.sh"
    ros2 pkg prefix tb4_multi_bringup >/dev/null
    # Stop manual/stock bringup before systemd takes ownership of the drivers.
    bash "$tb4_repo/scripts/robot-stop.sh" "$tb4_robot"
    tb4_file="$(mktemp)"
    trap 'rm -f "$tb4_file"' EXIT
    python3 - "$tb4_repo" "$tb4_robot" "$(id -un)" > "$tb4_file" <<'PY'
import sys
root, robot, user = sys.argv[1:]
def quote(value):
    return '"' + value.replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%') + '"'
print(f'''[Unit]
Description=TurtleBot hardware with display, LED and command watchdog
Wants=network-online.target
After=network-online.target
Conflicts=turtlebot4.service
StartLimitIntervalSec=0

[Service]
Type=simple
User={user}
ExecStart=/usr/bin/bash {quote(root + '/scripts/robot.sh')} {robot} hardware
Restart=on-failure
RestartSec=10
KillMode=control-group
TimeoutStopSec=20

[Install]
WantedBy=multi-user.target
''')
PY
    sudo install -m 644 "$tb4_file" "/etc/systemd/system/$tb4_unit"
    sudo systemctl daemon-reload
    sudo systemd-analyze verify "/etc/systemd/system/$tb4_unit"
    sudo systemctl disable turtlebot4.service
    sudo systemctl enable --now "$tb4_unit"
    ;;
  start) sudo systemctl start "$tb4_unit" ;;
  stop) sudo systemctl stop "$tb4_unit" ;;
  disable) sudo systemctl disable --now "$tb4_unit" ;;
  status) systemctl --no-pager --full status "$tb4_unit" ;;
esac
