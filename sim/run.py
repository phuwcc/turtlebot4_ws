#!/usr/bin/env python3
"""Run MARCO against simulated poses with no ROS or hardware connection."""
import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tb4_marco_bridge'))
from tb4_marco_bridge.planning import Planner
from occupancy_map import load_occupancy_map
from simulation_planner import SimulationPlanner
from t_junction import junction, swap_mission


def crossing():
    return dict(width=41, height=41, resolution=.1, origin=[-2.05, -2.05, 0.],
                data=[100 if x in (0, 40) or y in (0, 40) else 0
                      for y in range(41) for x in range(41)])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=['room', 'crossing', 't-junction'], default='room')
    parser.add_argument('--out', type=Path, help='Save the report with paths and final poses')
    parser.add_argument('--swap', nargs=3, metavar=('R1', 'R2', 'R3'),
                        help='T-junction destinations in robot order, e.g. B C A')
    parser.add_argument('--poses', type=Path, help='T-junction starting poses as JSON')
    args = parser.parse_args()
    if (args.swap or args.poses) and args.scenario != 't-junction':
        parser.error('--swap and --poses require --scenario t-junction')
    order = None
    if args.scenario == 't-junction':
        grid, poses = junction()
        if args.poses:
            try:
                supplied = json.loads(args.poses.read_text())
                if not isinstance(supplied, dict) or supplied.keys() != poses.keys():
                    raise ValueError('Expected robot1, robot2 and robot3')
                for pose in supplied.values():
                    if (not isinstance(pose, list) or len(pose) != 3
                            or not all(type(v) in (int, float) and math.isfinite(v) for v in pose)):
                        raise ValueError('Every pose must be [x, y, yaw] with finite values')
                poses = {robot: supplied[robot] for robot in poses}
            except (OSError, ValueError) as error:
                parser.error(str(error))
        destinations = dict(zip('ABC', (pose.copy() for pose in poses.values())))
        for (robot, _), (label, pose) in zip(poses.items(), destinations.items()):
            print(f'{label} = {robot} at ({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})')
        try:
            order = [label.upper() for label in (args.swap or
                     input('Nhap dich cho robot1 robot2 robot3 (vi du B C A): ').split())]
            order = ['C' if label == '3' else label for label in order]
        except EOFError:
            parser.error('Supply destinations with --swap B C A')
        if sorted(order) != ['A', 'B', 'C']:
            parser.error('Enter A, B, C exactly once each')
    if order:
        mission = swap_mission(order, destinations)
    elif args.scenario == 'room':
        grid = load_occupancy_map(ROOT / 'maps/room_real.yaml')
        poses = {'robot1': [0., 0., 0.], 'robot2': [0., .5, 0.]}
        mission_file = Path(__file__).with_name('room_mission.json')
    else:
        grid = crossing()
        poses = {'robot1': [-1., 0., 0.], 'robot2': [0., -1., math.pi / 2]}
        mission_file = ROOT / 'config/marco/testcases/04_crossing.json'
    if not order:
        mission = json.loads(mission_file.read_text())
    mission['priority_robot'] = 'robot1'
    state = {'map': grid, 'robots': {name: {'pose': pose} for name, pose in poses.items()}}
    planner_type = SimulationPlanner if order else Planner
    planner = planner_type(ROOT / 'MACRO')
    completed, batches, intent = [], [], None
    report = dict(scenario=args.scenario, priority_robot='robot1', initial_poses=poses,
                  completed_tasks=completed, batches=batches, result='failed')
    if order:
        report['destinations'] = destinations
        report['requested_destinations'] = dict(zip(poses, order))
    try:
        report['assignment'] = planner.assign(state, mission)
        for _ in range(32):
            batch = planner.next_batch(state, mission, report['assignment'], completed, intent)
            batches.append(batch)
            if not batch['actions']:
                break
            intent = batch.get('intent')
            for action in batch['actions']:
                state['robots'][action['robot']]['pose'] = action['points'][-1]
                if action['task'] is not None:
                    completed.append(action['task'])
            print(f"Batch {len(batches)}: {batch['audit']['strategy']} "
                  + ', '.join(a['robot'] for a in batch['actions']))
            if set(completed) == {task['id'] for task in mission['tasks']}:
                report['result'] = 'succeeded'
                break
        if report['result'] != 'succeeded':
            report['error'] = batches[-1].get('reason') or 'Batch limit reached'
    except (RuntimeError, ValueError) as error:
        report['error'] = str(error)
    report['final_poses'] = {name: robot['pose'] for name, robot in state['robots'].items()}
    if args.out:
        args.out.write_text(json.dumps(report, indent=2) + '\n')
    print(report['result'], 'completed tasks:', completed, report.get('error', ''))
    return int(report['result'] != 'succeeded')


if __name__ == '__main__':
    sys.exit(main())
