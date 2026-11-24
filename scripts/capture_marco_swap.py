#!/usr/bin/env python3
"""Capture live starting poses for a two- or three-robot MARCO swap."""

import argparse
import json
import math
import os
from pathlib import Path

from socket_client import send


def swap_mission(snapshot, *, three_robots=False):
    names = ('robot1', 'robot2', 'robot3') if three_robots else ('robot1', 'robot2')
    robots = snapshot.get('robots', {})
    if not snapshot.get('ok') or set(robots) != set(names):
        raise ValueError('Cần phiên MARCO với đúng ' + ', '.join(names) + '.')

    poses = {}
    for name in names:
        state = robots[name]
        pose = state.get('pose')
        if (state.get('busy') or not state.get('localized') or not state.get('odom_fresh')
                or not isinstance(pose, list) or len(pose) != 3
                or not all(type(value) in (int, float) and math.isfinite(value) for value in pose)):
            raise ValueError(f'{name} cần đứng yên và có pose định vị hợp lệ.')
        velocity = state.get('velocity')
        if velocity is not None and (
                not isinstance(velocity, list) or len(velocity) != 2
                or not all(type(value) in (int, float) and math.isfinite(value) for value in velocity)
                or abs(velocity[0]) > .015 or abs(velocity[1]) > .05):
            raise ValueError(f'{name} cần đứng yên trước khi lưu pose.')
        poses[name] = pose.copy()

    if len({tuple(pose[:2]) for pose in poses.values()}) != len(names):
        raise ValueError('Các vị trí đầu phải khác nhau.')
    targets = names[1:] + names[:1]
    if not three_robots:
        for name, target in zip(names, targets):
            start, other = poses[name], poses[target]
            bearing = math.atan2(other[1] - start[1], other[0] - start[0])
            if abs(math.remainder(start[2] - bearing, math.tau)) > math.pi / 4:
                raise ValueError(f'{name} cần hướng về robot còn lại (sai lệch tối đa 45°).')

    return {
        'description': (
            'Đổi chỗ theo vòng. A/B/C là pose đầu của robot1/robot2/robot3.' if three_robots
            else 'Đổi chỗ đối đầu. A là pose đầu của robot1, B là pose đầu của robot2.'),
        'priority_order': ['robot1', 'robot3', 'robot2'] if three_robots else list(names),
        'robot_radius': 0.175,
        'safety_margin': 0.05,
        'tracking_tolerance': 0.06,
        'smooth_deviation_m': 0.02,
        'speed_mps': 0.10,
        'action_timeout_s': 180,
        # All tasks must be ready for MARCO to see the full swap conflict.
        'tasks': [
            {'id': index, 'label': f'{name} → {label}', 'goal': poses[target],
             'candidates': [name], 'depends_on': []}
            for index, (name, target, label) in enumerate(zip(names, targets, 'BCA' if three_robots else 'BA'), 1)
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='New mission JSON file')
    parser.add_argument('--three-robots', action='store_true', help='Rotate the initial poses of robot1, robot2 and robot3')
    args = parser.parse_args()
    try:
        port = int(os.environ.get('TB4_SOCKET_PORT', '8765'))
        mission = swap_mission(send({'command': 'snapshot'}, port=port), three_robots=args.three_robots)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x', encoding='utf-8') as output:
            json.dump(mission, output, indent=2, ensure_ascii=False)
            output.write('\n')
    except (OSError, ValueError) as error:
        parser.exit(1, f'Chưa tạo testcase đổi chỗ: {error}\n')
    print(f'Đã lưu testcase đổi chỗ vào {args.output}')


if __name__ == '__main__':
    main()
