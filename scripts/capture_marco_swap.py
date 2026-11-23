#!/usr/bin/env python3
"""Capture the two live starting poses for a head-on MARCO swap."""

import argparse
import json
import math
import os
from pathlib import Path

from socket_client import send


def swap_mission(snapshot):
    names = ('robot1', 'robot2')
    robots = snapshot.get('robots', {})
    if not snapshot.get('ok') or set(robots) != set(names):
        raise ValueError('Cần phiên MARCO với đúng robot1 và robot2.')

    poses = {}
    for name in names:
        state = robots[name]
        pose = state.get('pose')
        if (state.get('busy') or not state.get('localized') or not state.get('odom_fresh')
                or not isinstance(pose, list) or len(pose) != 3
                or not all(type(value) in (int, float) and math.isfinite(value) for value in pose)):
            raise ValueError(f'{name} cần đứng yên và có pose định vị hợp lệ.')
        poses[name] = pose

    a, b = (poses[name] for name in names)
    if a[:2] == b[:2]:
        raise ValueError('Pose A và B đang trùng nhau.')
    for name, start, other in (('robot1', a, b), ('robot2', b, a)):
        bearing = math.atan2(other[1] - start[1], other[0] - start[0])
        if abs(math.remainder(start[2] - bearing, math.tau)) > math.pi / 4:
            raise ValueError(f'{name} cần hướng về robot còn lại (sai lệch tối đa 45°).')

    return {
        'description': 'Đổi chỗ đối đầu. A là pose đầu của robot1, B là pose đầu của robot2.',
        'priority_order': list(names),
        'robot_radius': 0.175,
        'safety_margin': 0.05,
        'tracking_tolerance': 0.06,
        'smooth_deviation_m': 0.02,
        'speed_mps': 0.10,
        'action_timeout_s': 180,
        # Both tasks must be ready for MARCO to see the head-on conflict.
        'tasks': [
            {'id': 1, 'label': 'robot1 → B', 'goal': b, 'candidates': ['robot1'], 'depends_on': []},
            {'id': 2, 'label': 'robot2 → A', 'goal': a, 'candidates': ['robot2'], 'depends_on': []},
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='New mission JSON file')
    args = parser.parse_args()
    try:
        port = int(os.environ.get('TB4_SOCKET_PORT', '8765'))
        mission = swap_mission(send({'command': 'snapshot'}, port=port))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x', encoding='utf-8') as output:
            json.dump(mission, output, indent=2, ensure_ascii=False)
            output.write('\n')
    except (OSError, ValueError) as error:
        parser.exit(1, f'Chưa tạo testcase đổi chỗ: {error}\n')
    print(f'Đã lưu testcase đổi chỗ vào {args.output}')


if __name__ == '__main__':
    main()
