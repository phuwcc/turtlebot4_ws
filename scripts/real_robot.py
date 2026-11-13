#!/usr/bin/env python3
"""Real TB4 entry points. Existing ROS scripts retain locks, leases and watchdogs."""
import argparse
import fcntl
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import sys
import time

from pc_config import load_environment, environment_for_robot

ROOT = Path(__file__).resolve().parents[1]


def script(name, *args):
    return ['bash', str(ROOT / 'scripts' / name), *map(str, args)]


def checked(argv, **kwargs):
    return subprocess.run(argv, cwd=ROOT, check=True, **kwargs)


def file_path(value):
    path = Path(value).expanduser()
    return (ROOT / path).resolve() if not path.is_absolute() else path.resolve()


def fleet():
    path = Path.home() / '.config/turtlebot4/pc_env.sh'
    try:
        values = load_environment(path)
    except (OSError, ValueError) as error:
        raise ValueError('Cấu hình IP trước: bash scripts/pc-configure.sh (' + str(error) + ')') from error
    ids = values.get('TB4_ROBOT_IDS', ','.join('robot'+str(i+1) for i in range(int(values['TB4_ROBOT_COUNT'])))).split(',')
    return values, ids


def ssh_command(robot, values, argv, tty=False):
    # Discovery slots can differ from robot IDs; use the validated configuration.
    single = environment_for_robot(values, robot)
    address = next(s for s in single['ROS_DISCOVERY_SERVER'].split(';') if s).split(':')[0]
    host = os.environ.get('TB4_SSH_' + robot.upper(), os.environ.get('TB4_SSH_USER', 'ubuntu') + '@' + address)
    command = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=6', '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=3']
    key = Path(os.environ.get('TB4_SSH_KEY', str(Path.home() / '.ssh/id_ed25519_tb4_debug'))).expanduser()
    if key.is_file():
        command += ['-i', str(key), '-o', 'IdentitiesOnly=yes']
    if tty:
        command.append('-t')
    directory = os.environ.get('TB4_REMOTE_REPO')
    cd = shlex.quote(directory) if directory else '"$HOME/turtlebot4_ws"'
    return command + [host, 'cd ' + cd + ' && ' + shlex.join(argv)]


def hardware(robot, values, install=False, read_only=False):
    if read_only:
        argv = ['bash', '-c', 'systemctl is-active tb4-hardware.service && systemctl is-enabled tb4-hardware.service && test -e /dev/RPLIDAR']
    else:
        argv = ['bash', 'scripts/robot-service.sh', robot, 'install' if install else 'start']
    result = checked(ssh_command(robot, values, argv), capture_output=True, text=True, timeout=90)
    print(robot + ': ' + (result.stdout.strip() or 'hardware service started'), flush=True)


def start_hardware(ids, values, install=False, read_only=False):
    with ThreadPoolExecutor(max_workers=len(ids)) as pool:
        futures = [pool.submit(hardware, r, values, install, read_only) for r in ids]
        for future in futures:
            future.result()


def ros_command(argv):
    # Arguments remain argv, never interpolated into shell source.
    return ['bash', '-c', '''set -e
source /opt/ros/humble/setup.bash
source "$1/install/setup.bash"
if [[ -f "$HOME/.config/turtlebot4/runtime_env.sh" ]]; then source "$HOME/.config/turtlebot4/runtime_env.sh"; fi
if [[ -n "${TB4_TF2_RUNTIME_LIB:-}" ]]; then export LD_LIBRARY_PATH="$TB4_TF2_RUNTIME_LIB:${LD_LIBRARY_PATH:-}"; fi
shift
exec "$@"
''', 'tb4-real', str(ROOT), *map(str, argv)]


def preflight(ids, values, map_ready=False, undocked=False, timeout=45):
    args = ['/usr/bin/python3', '-B', ROOT/'scripts/real_preflight.py', '--robots', *ids, '--timeout', str(timeout)]
    if map_ready:
        args.append('--map-ready')
    if undocked:
        args.append('--require-undocked')
    checked(ros_command(args), env={**os.environ, **values}, timeout=timeout+15)


def require_pc():
    if Path('/etc/turtlebot4/setup.bash').exists():
        raise ValueError('Mapping/MARCO chạy trên PC; Pi chỉ chạy: bash run.bash hardware robot1 (hoặc robot2).')
    if not (ROOT/'install/setup.bash').is_file():
        raise ValueError('Build PC trước: bash scripts/setup.sh pc')


def ensure_no_session():
    lock = Path(os.environ.get('XDG_STATE_HOME', str(Path.home()/'.local/state'))) / 'tb4_multi_robot/pc/session.lock'
    lock.parent.mkdir(parents=True, exist_ok=True)
    with lock.open('a') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ValueError('Đang có phiên PC mapping/navigation/teleop; dừng phiên đó trước.') from error
    try:
        connection = socket.create_connection(('127.0.0.1', 8765), timeout=1)
    except OSError:
        return
    connection.close()
    raise ValueError('Cổng 8765 đã có gateway; dừng phiên PC cũ trước khi mở phiên mới.')


def stop_child(child):
    if child.poll() is not None:
        return
    os.killpg(child.pid, signal.SIGINT)
    try:
        child.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(child.pid, signal.SIGTERM)
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait()


def log_directory(mode):
    path = ROOT/'log/real'/datetime.now().strftime('%Y%m%d-%H%M%S-%f')/mode
    path.mkdir(parents=True, exist_ok=True)
    return path


def check_child(child):
    if child.poll() is not None:
        raise RuntimeError('ROS đã thoát; xem session.log trong thư mục log vừa in.')


def wait_navigation(child, ids, timeout):
    # Use the existing protocol and its complete per-robot readiness, not a sleep.
    from socket_client import send
    deadline = time.monotonic() + timeout
    previous = None
    while time.monotonic() < deadline:
        check_child(child)
        try:
            status = send({'command': 'status'})
            robots = status.get('robots', {})
            ready = {r: bool(robots.get(r, {}).get('localized') and robots.get(r, {}).get('path_ready')) for r in ids}
            if ready != previous:
                print('Nav2/localization: ' + json.dumps(ready), flush=True)
                previous = ready
            if status.get('ok') and set(robots) == set(ids) and all(ready.values()):
                return
        except (OSError, ValueError):
            pass
        time.sleep(.5)
    raise RuntimeError('Chưa đạt định vị/Nav2 trong thời hạn. Kiểm tra pose trên RViz; chưa gửi mission.')


def validate_mission(filename):
    path = file_path(filename)
    mission = json.loads(path.read_text())
    if not isinstance(mission, dict) or not isinstance(mission.get('tasks'), list) or not mission['tasks']:
        raise ValueError('Mission cần danh sách tasks không rỗng.')
    return path


def mission_run(path, execute, directory):
    checked(script('marco.sh', 'plan', path, '--out', directory/'preview.json'))
    if execute:
        checked(script('marco.sh', 'run', path, '--execute', '--out', directory/'execution.json'))


def multi(args, values, ids):
    require_pc()
    from pc_diagnose import validate_map
    map_path = file_path(args.map)
    validate_map(map_path)
    if args.poses is None:
        if not sys.stdin.isatty():
            raise ValueError('Cần --poses FILE.yaml chứa vị trí x/y/yaw thật của từng robot trên map.')
        args.poses = input('File pose thật của hai robot trên map (YAML): ').strip()
        if not args.poses:
            raise ValueError('Chưa khai báo pose; chưa khởi động localization.')
    poses = file_path(args.poses)
    # Validate every active robot before touching remote hardware or starting ROS.
    sys.path.insert(0, str(ROOT/'src/tb4_multi_bringup'))
    from tb4_multi_bringup.fleet_settings import startup_poses
    loaded = startup_poses(str(poses), len(ids), ','.join(ids))
    if set(loaded) != set(ids):
        raise ValueError('File pose phải chứa đủ các robot đang chọn: ' + ','.join(ids))
    mission = validate_mission(args.mission) if args.mission else None
    core = Path(os.environ.get('MARCO_ROOT', str(ROOT/'MACRO'))).expanduser()
    if not (core/'src/core/coordination/relocation_search.py').is_file():
        raise ValueError('Thiếu MACRO: git submodule update --init --recursive MACRO; bash scripts/build.sh')
    ensure_no_session()
    checked(ros_command(['/usr/bin/python3', '-B', '-c',
        'import sys;sys.dont_write_bytecode=True;sys.path.insert(0,sys.argv[1]);from core.assignment_cpp import require_native;from core.coordination.native_geometry import backend;require_native();backend()', str(core/'src')]),
        env={**os.environ, **values})
    start_hardware(ids, values, args.install_service)
    preflight(ids, values, undocked=args.execute)
    directory = log_directory('multi')
    print('ROS log: ' + str(directory/'session.log'), flush=True)
    command = script('pc-marco.sh', map_path, 'initial_poses:='+str(poses),
                     'rviz:='+str(not args.no_rviz).lower())
    with (directory/'session.log').open('w') as stream:
        child = subprocess.Popen(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            wait_navigation(child, ids, args.timeout)
            print('Hai robot sẵn sàng. MACRO chạy khi nhận mission, có kiểm tra conflict và watchdog.', flush=True)
            if mission:
                mission_run(mission, args.execute, directory)
                if not args.keep_alive:
                    return
            print('Giữ phiên ROS/RViz; Ctrl+C để dừng. Terminal khác: bash run.bash mission FILE.json --execute', flush=True)
            child.wait()
            check_child(child)
        finally:
            stop_child(child)


def mapping(args, values, ids):
    require_pc()
    if args.robot not in ids:
        raise ValueError(args.robot + ' chưa có trong cấu hình PC.')
    if not args.no_teleop and not sys.stdin.isatty():
        raise ValueError('Lái bàn phím cần terminal tương tác; dùng --no-teleop để chỉ mở SLAM/RViz.')
    ensure_no_session()
    single = environment_for_robot(values, args.robot)
    directory = log_directory('mapping')
    target = file_path(args.save_as) if args.save_as else ROOT/'maps'/('scan_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    if target.suffix in ('.yaml', '.pgm'):
        target = target.with_suffix('')
    start_hardware([args.robot], values, args.install_service)
    preflight([args.robot], single, undocked=not args.no_teleop)
    print('ROS log: ' + str(directory/'session.log') + '\nMap sẽ lưu: ' + str(target), flush=True)
    command = script('pc-mapping.sh', args.robot, 'origin_file:='+str(directory/'origin.json'),
                     'rviz:='+str(not args.no_rviz).lower())
    with (directory/'session.log').open('w') as stream:
        child = subprocess.Popen(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        ready = False
        try:
            preflight([args.robot], single, map_ready=True)
            check_child(child)
            ready = True
            print('SLAM sẵn sàng. i/, tiến/lùi, j/l quay, k/Space dừng; Ctrl+C để thoát và lưu map.', flush=True)
            if args.no_teleop:
                child.wait()
                check_child(child)
            else:
                checked(ssh_command(args.robot, values, ['bash', 'scripts/robot-teleop.sh', args.robot], tty=True))
        except KeyboardInterrupt:
            pass
        finally:
            try:
                if ready and child.poll() is None:
                    checked(script('pc-save-map.sh', args.robot, target))
                    print('Đã lưu map: ' + str(target) + '.yaml', flush=True)
            finally:
                stop_child(child)


def parser():
    p = argparse.ArgumentParser(description='TB4 thật qua SSH + ROS 2 Humble + MACRO; không chạy simulator.')
    sub = p.add_subparsers(dest='mode')
    for name in ('check', 'hardware', 'stop'):
        s = sub.add_parser(name)
        s.add_argument('robot', nargs='?', choices=('robot1', 'robot2', 'robot3', 'all'), default='all')
        if name == 'hardware':
            s.add_argument('--install-service', action='store_true', help='Cài/bật hardware tự chạy sau reboot trên Pi')
    s = sub.add_parser('mapping')
    s.add_argument('--robot', choices=('robot1', 'robot2', 'robot3'), default='robot1')
    s.add_argument('--save-as', help='Tên map, không có đuôi; mặc định tên mới theo thời gian')
    s.add_argument('--no-teleop', action='store_true')
    s.add_argument('--no-rviz', action='store_true')
    s.add_argument('--install-service', action='store_true')
    s = sub.add_parser('multi')
    s.add_argument('--map', default='maps/my_room.yaml')
    s.add_argument('--poses', help='YAML x/y/yaw thật trên map, đủ các robot')
    s.add_argument('--mission')
    s.add_argument('--execute', action='store_true')
    s.add_argument('--keep-alive', action='store_true')
    s.add_argument('--no-rviz', action='store_true')
    s.add_argument('--install-service', action='store_true')
    s.add_argument('--timeout', type=float, default=180.)
    s = sub.add_parser('mission')
    s.add_argument('filename')
    s.add_argument('--execute', action='store_true')
    s = sub.add_parser('teleop')
    s.add_argument('robot', choices=('robot1', 'robot2', 'robot3'), default='robot1', nargs='?')
    s = sub.add_parser('save-map')
    s.add_argument('filename')
    s.add_argument('--robot', choices=('robot1', 'robot2', 'robot3'), default='robot1')
    return p


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    if not args.mode:
        if Path('/etc/turtlebot4/setup.bash').exists():
            checked(script('robot-service.sh', os.environ.get('ROBOT_NAMESPACE', '').strip('/') or
                           input('Robot ID [robot1/robot2]: ').strip(), 'start'))
            return 0
        if not sys.stdin.isatty():
            p.print_help()
            return 0
        print('1. Kiểm tra hai TB4\n2. Bật hardware hai TB4\n3. Quét và lưu map bằng robot1\n4. Chạy ROS đa robot + MACRO')
        choice = input('Chọn [1/2/3/4]: ').strip()
        modes = {'1': 'check', '2': 'hardware', '3': 'mapping', '4': 'multi'}
        if choice not in modes:
            raise ValueError('Chọn 1..4.')
        args = p.parse_args([modes[choice]])
    if args.mode == 'multi' and (not 0 < args.timeout <= 600 or (args.execute and not args.mission)):
        p.error('--timeout phải trong 0..600; --execute cần --mission FILE.json')
    if args.mode in ('hardware', 'stop') and Path('/etc/turtlebot4/setup.bash').exists():
        if args.robot == 'all':
            raise ValueError('Trên Pi phải chọn robot1 hoặc robot2.')
        checked(script('robot-service.sh', args.robot,
                       'install' if getattr(args, 'install_service', False) else ('start' if args.mode == 'hardware' else 'stop')))
        return 0
    values, ids = fleet()
    if getattr(args, 'robot', 'all') not in ('all', *ids):
        raise ValueError('Robot chưa có trong cấu hình PC.')
    if args.mode == 'multi':
        multi(args, values, ids)
    elif args.mode == 'mapping':
        mapping(args, values, ids)
    elif args.mode == 'mission':
        require_pc()
        path = validate_mission(args.filename)
        preflight(ids, values, undocked=args.execute)
        mission_run(path, args.execute, log_directory('mission'))
    elif args.mode == 'save-map':
        require_pc()
        checked(script('pc-save-map.sh', args.robot, file_path(args.filename)))
    elif args.mode == 'teleop':
        require_pc()
        start_hardware([args.robot], values)
        preflight([args.robot], environment_for_robot(values, args.robot), undocked=True)
        checked(ssh_command(args.robot, values, ['bash', 'scripts/robot-teleop.sh', args.robot], tty=True))
    else:
        selected = ids if args.robot == 'all' else [args.robot]
        if args.mode == 'stop':
            for robot in selected:
                checked(ssh_command(robot, values, ['bash', 'scripts/robot-stop.sh', robot]), timeout=45)
        else:
            start_hardware(selected, values, getattr(args, 'install_service', False), read_only=args.mode == 'check')
            require_pc()
            env = values if selected == ids else environment_for_robot(values, selected[0])
            preflight(selected, env)
    return 0


if __name__ == '__main__':
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('\nĐã dừng phiên PC. Hardware/watchdog trên Pi vẫn chạy.', file=sys.stderr)
        raise SystemExit(130)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        if isinstance(error, subprocess.CalledProcessError) and error.stderr:
            print(error.stderr, file=sys.stderr)
        print('Chưa chạy tiếp: ' + str(error), file=sys.stderr)
        raise SystemExit(1)
