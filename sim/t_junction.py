"""Photo-inspired single-lane corridor and named parking spots."""
import math
from pathlib import Path

from occupancy_map import load_occupancy_map

MAP_FILE = Path(__file__).with_name('maps') / 't_junction.yaml'
START_POSES = {
    'robot1': [.475, 2.525, 0.],
    'robot2': [3.775, 2.525, math.pi],
    'robot3': [2.125, .475, math.pi / 2],
}


def junction():
    grid = load_occupancy_map(MAP_FILE)
    if grid['origin'] != [0., 0., 0.]:
        raise ValueError('The T-junction world requires a map origin of [0, 0, 0]')
    return grid, {robot: pose.copy() for robot, pose in START_POSES.items()}


def swap_mission(order, destinations):
    return dict(priority_robot='robot1', robot_radius=.175, safety_margin=.05,
                   tracking_tolerance=.04, speed_mps=.05,
                   tasks=[dict(id=i, goal=destinations[label], candidates=[f'robot{i}'],
                               depends_on=[]) for i, label in enumerate(order, 1)])
