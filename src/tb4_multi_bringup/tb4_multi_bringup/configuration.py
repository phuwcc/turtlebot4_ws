import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def share(package='tb4_multi_bringup'):
    return Path(get_package_share_directory(package))


def robot_arguments():
    return [
        DeclareLaunchArgument('robot_id', choices=['robot1', 'robot2', 'robot3'],
                              description='Identity of this physical robot'),
        DeclareLaunchArgument('robot_config', default_value='',
                              description='Optional alternate robot YAML'),
    ]


def load_robot(context):
    identity = LaunchConfiguration('robot_id').perform(context)
    if identity not in ('robot1', 'robot2', 'robot3'):
        raise ValueError('robot_id must be robot1, robot2 or robot3')
    supplied = LaunchConfiguration('robot_config').perform(context)
    path = Path(supplied) if supplied else share() / 'config/robots' / (identity + '.yaml')
    with path.open() as stream:
        cfg = yaml.safe_load(stream)
    if cfg['namespace'] != identity:
        raise ValueError('namespace must match robot_id')
    if cfg['model'] not in ('standard', 'lite'):
        raise ValueError('model must be standard or lite')
    private = cfg['create3_namespace']
    if not private.startswith('/') or '//' in private or private == '/' + identity:
        raise ValueError('create3_namespace must be a separate absolute namespace')
    if private != '/' + identity + '/_do_not_use':
        raise ValueError('Expected Create 3 namespace /' + identity + '/_do_not_use')
    for frame in cfg['frames'].values():
        if not frame or frame.startswith('/') or any(c.isspace() for c in frame):
            raise ValueError('TF frame IDs must be nonempty relative names')
    return cfg


def initial_pose(context):
    enabled = LaunchConfiguration('initialize_pose').perform(context) == 'true'
    values = [LaunchConfiguration(k).perform(context) for k in ('initial_x', 'initial_y', 'initial_yaw')]
    if enabled and not all(values):
        raise ValueError('initialize_pose=true requires initial_x, initial_y and initial_yaw')
    pose = [float(v) if v else 0.0 for v in values]
    if not all(math.isfinite(v) for v in pose):
        raise ValueError('Initial pose must be finite')
    return enabled, pose
