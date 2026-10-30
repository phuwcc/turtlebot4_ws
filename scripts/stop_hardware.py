#!/usr/bin/env python3
"""Stop this user's repository hardware processes for one exact robot namespace."""
import argparse
import os
from pathlib import Path
import signal
import time

DRIVERS = (
    '/lib/turtlebot4_node/turtlebot4_node',
    '/lib/create3_republisher/create3_republisher',
    '/lib/rplidar_ros/rplidar_composition',
    '/lib/turtlebot4_base/turtlebot4_base_node',
    '/lib/robot_state_publisher/robot_state_publisher',
    '/lib/joint_state_publisher/joint_state_publisher',
)


def matches_hardware(argv, robot):
    """Match launch argv or known hardware executables, never a shell command string."""
    for index, arg in enumerate(argv[:2]):
        if (Path(arg).name == 'ros2' and
                argv[index+1:index+4] == ['launch', 'tb4_multi_bringup', 'hardware.launch.py'] and
                'robot_id:=' + robot in argv[index+4:]):
            return True
    if '__ns:=/' + robot not in argv:
        return False
    for arg in argv[:2]:
        if arg.endswith(DRIVERS):
            return True
        if (arg.endswith('/lib/tb4_multi_bringup/remote_velocity') and
                '__node:=remote_velocity_guard' in argv):
            return True
    return False


def identity(pid):
    """Return start time for PID reuse protection, or None for an exited process."""
    try:
        fields = (Path('/proc') / str(pid) / 'stat').read_text().rsplit(') ', 1)[1].split()
        return None if fields[0] == 'Z' else fields[19]
    except (OSError, IndexError):
        return None


def terminate(processes, timeout=8.):
    """SIGTERM also reaches nohup children that inherited SIGINT as ignored."""
    for pid, started in processes.items():
        if identity(pid) == started:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = [pid for pid, started in processes.items() if identity(pid) == started]
        if not remaining:
            return []
        time.sleep(.1)
    return [pid for pid, started in processes.items() if identity(pid) == started]


def find_hardware(robot):
    processes = {}
    for entry in Path('/proc').glob('[0-9]*'):
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            argv = (entry / 'cmdline').read_bytes().decode(errors='replace').split('\0')[:-1]
            if matches_hardware(argv, robot):
                started = identity(int(entry.name))
                if started is not None:
                    processes[int(entry.name)] = started
        except OSError:
            continue
    return processes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('robot', choices=['robot1', 'robot2', 'robot3'])
    args = parser.parse_args()
    processes = find_hardware(args.robot)
    if processes:
        print('Dừng hardware ' + args.robot + ', PID: ' + ', '.join(map(str, sorted(processes))), flush=True)
    remaining = sorted(set(terminate(processes)) | set(find_hardware(args.robot)))
    if remaining:
        print('Chưa dừng được PID: ' + ', '.join(map(str, remaining)) + '. Chưa mở hardware mới.', flush=True)
        return 1
    print('Hardware ' + args.robot + ' đã dừng; có thể chạy lại robot-start.sh.', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
