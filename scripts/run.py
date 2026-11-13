#!/usr/bin/env python3
"""Two interactive entry points; existing scripts own ROS, locks and watchdogs."""
import argparse
import json
import os
from pathlib import Path
import subprocess

from pc_config import configure, load_environment


PC_MODES = {
    '1': 'MARCO + Nav2 + RViz',
    '2': 'Chỉ localization + RViz (không điều hướng)',
    '3': 'Quét map bằng một robot (mặc định robot1)',
    '4': 'Chọn robot / pose / goal / hủy goal',
    '5': 'Xem kế hoạch mission JSON',
    '6': 'Thực thi mission JSON',
    '7': 'Lưu map đang quét',
    '8': 'Đổi số robot và IP',
}
ROBOT_MODES = {'1': 'Hardware + watchdog', '2': 'Lái bàn phím khi quét map (dừng navigation trước)',
               '3': 'Dừng hardware', '4': 'Kiểm tra node và cảm biến'}


def choose(title, choices, input_fn=input, default=None):
    print(title)
    for key, label in choices.items():
        print(f'  {key}. {label}')
    while True:
        value = input_fn('Chọn' + (f' [{default}]' if default else '') + ': ').strip()
        value = value or default
        if value in choices:
            return value
        print('Lựa chọn không hợp lệ; nhập lại.')


def file_input(repo, title, suffixes, input_fn=input, default=None):
    while True:
        value = input_fn(title + (f' [{default}]' if default else '') + ': ').strip()
        value = value or default
        if value:
            path = Path(value).expanduser()
            if not path.is_absolute():
                path = repo / path
            if path.is_file() and path.suffix.lower() in suffixes:
                return path.resolve()
        print('File không tồn tại hoặc sai định dạng; nhập lại.')


def robot_choice(ids, input_fn=input, default=None):
    if default is None:
        default = os.environ.get('ROBOT_NAMESPACE', '').strip('/')
    choices = {str(i+1): r for i,r in enumerate(ids)}
    key = str(ids.index(default)+1) if default in ids else '1'
    return choices[choose('Robot nào?', choices, input_fn, key)]


def script(repo, name, *args):
    return ['bash', str(repo / 'scripts' / name), *map(str, args)]


def robot_command(repo, mode, robot):
    if robot not in ('robot1', 'robot2', 'robot3'):
        raise ValueError('Robot ID không hợp lệ.')
    commands = {'1': ('robot-start.sh', robot), '2': ('robot-teleop.sh', robot),
                '3': ('robot-stop.sh', robot), '4': ('robot.sh', robot, 'check')}
    if mode not in commands:
        raise ValueError('Chế độ robot không hợp lệ.')
    return script(repo, *commands[mode])


def pc_command(repo, mode, ids, input_fn=input):
    if mode in ('1', '2'):
        default = 'maps/my_room.yaml' if (repo / 'maps/my_room.yaml').is_file() else None
        map_file = file_input(repo, 'Map YAML', ('.yaml', '.yml'), input_fn, default)
        pose_mode = choose('Vị trí khởi tạo (phải đúng vị trí robot thật):', {
            '1': 'Bố trí đã khai báo: robot1 ở (0,0,yaw=0); gợi ý các robot khác trong config/marco/localization.yaml',
            '2': 'Nhập file x/y/yaw hiện tại cho từng robot (sau khi robot đã di chuyển)',
        }, input_fn)
        args = []
        if pose_mode == '2':
            args.append('initial_poses:=' + str(file_input(repo, 'File initial poses YAML', ('.yaml', '.yml'), input_fn)))
        if mode == '2':
            args.append('localization_only:=true')
        return script(repo, 'pc-marco.sh', map_file, *args)
    if mode == '3':
        return script(repo, 'pc-mapping.sh', robot_choice(ids, input_fn, default='robot1'))
    if mode == '4':
        return script(repo, 'pc-menu.sh')
    if mode in ('5', '6'):
        mission = file_input(repo, 'Mission JSON (goal trên map thực tế)', ('.json',), input_fn)
        with mission.open() as stream:
            data = json.load(stream)
        if not isinstance(data, dict) or not isinstance(data.get('tasks'), list) or not data['tasks']:
            raise ValueError('Mission JSON phải có danh sách tasks không rỗng.')
        if mode == '5':
            return script(repo, 'marco.sh', 'plan', mission)
        # Preview first. An error stops this function before any execution request.
        subprocess.run(script(repo, 'marco.sh', 'plan', mission), cwd=repo, check=True)
        answer = input_fn('Đã xem preview, robot khỏi dock, vùng chạy trống và đang giám sát? Nhập CHAY để thực thi: ').strip()
        if answer != 'CHAY':
            raise ValueError('Đã dừng ở preview; chưa yêu cầu thực thi mission.')
        return script(repo, 'marco.sh', 'run', mission, '--execute')
    if mode == '7':
        robot = robot_choice(ids, input_fn, default='robot1')
        value = input_fn('Tên map lưu, không cần đuôi file [maps/site]: ').strip() or 'maps/site'
        path = Path(value).expanduser()
        if not path.is_absolute():
            path = repo / path
        return script(repo, 'pc-save-map.sh', robot, path)
    raise ValueError('Chế độ PC không hợp lệ.')


def ensure_built(repo, role, input_fn=input):
    if not (repo / 'install/setup.bash').is_file():
        answer = input_fn(f'Workspace chưa build. Cài gói còn thiếu và build cho {role}? [y/N]: ').strip().lower()
        if answer not in ('y', 'yes'):
            raise ValueError('Cần build workspace trước khi chạy.')
        subprocess.run(script(repo, 'setup.sh', role), cwd=repo, check=True)
        if not (repo / 'install/setup.bash').is_file():
            raise ValueError('Build chưa tạo install/setup.bash.')


def ensure_marco(repo, input_fn=input):
    root = Path(os.environ.get('MARCO_ROOT', str(repo / 'MACRO'))).expanduser()
    if (root / 'src/core/assignment.py').is_file():
        return
    if root.resolve() != (repo / 'MACRO').resolve():
        raise ValueError('MARCO_ROOT không trỏ tới checkout MARCO hợp lệ.')
    answer = input_fn('Chưa có MARCO. Tải submodule đã ghim và build nếu cần? [y/N]: ').strip().lower()
    if answer not in ('y', 'yes'):
        raise ValueError('Cần MARCO submodule cho chế độ này.')
    subprocess.run(['git', 'submodule', 'update', '--init', '--recursive', 'MACRO'], cwd=repo, check=True)
    if not (root / 'src/core/assignment.py').is_file():
        raise ValueError('Submodule chưa sẵn sàng.')
    if (repo / 'install/setup.bash').is_file():
        subprocess.run(script(repo, 'build.sh'), cwd=repo, check=True,
                       env={**os.environ, 'TB4_BUILD_PROFILE': 'pc'})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role', choices=('pc', 'robot'))
    parser.add_argument('--mode', choices=tuple(PC_MODES), help='Chọn sẵn mục trong menu; thông tin còn thiếu vẫn được hỏi')
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    try:
        if args.role == 'robot':
            mode = args.mode or choose('Robot: chọn công việc', ROBOT_MODES, default='1')
            robot = robot_choice(['robot1', 'robot2', 'robot3'])
            command = robot_command(repo, mode, robot)
            if mode != '3':
                ensure_built(repo, 'robot')
        else:
            mode = args.mode or choose('PC: chọn công việc', PC_MODES, default='1')
            config = Path.home() / '.config/turtlebot4/pc_env.sh'
            if mode == '8':
                configure(config)
                print('Khởi động lại phiên PC để áp dụng cấu hình mới.')
                return 0
            try:
                values = load_environment(config)
            except (OSError, ValueError):
                configure(config)
                values = load_environment(config)
            else:
                print('Cấu hình đã lưu: ' + values['ROS_DISCOVERY_SERVER'])
                reuse = choose('Dùng cấu hình mạng này?', {'1': 'Dùng tiếp', '2': 'Nhập lại số robot/IP'}, default='1')
                if reuse == '2':
                    configure(config)
                    values = load_environment(config)
            ids = values.get('TB4_ROBOT_IDS', '').split(',') if values.get('TB4_ROBOT_IDS') else [f'robot{i+1}' for i in range(int(values['TB4_ROBOT_COUNT']))]
            if mode in ('1', '5', '6'):
                ensure_marco(repo)
            ensure_built(repo, 'pc')
            command = pc_command(repo, mode, ids)
        os.chdir(repo)
        os.execvpe(command[0], command, os.environ.copy())
    except (OSError, ValueError, EOFError, KeyboardInterrupt, subprocess.CalledProcessError) as error:
        print('Chưa khởi động tác vụ mới: ' + str(error))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
