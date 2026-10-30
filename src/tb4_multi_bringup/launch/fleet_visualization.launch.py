"""One RViz for the configured number of active robot TF trees (1–3)."""
import os
from pathlib import Path
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tb4_multi_bringup.configuration import share, pc_service_profile
from tb4_multi_bringup.fleet_settings import active_robots, rviz_for_count, validate_discovery


def setup(context):
    count = int(LaunchConfiguration('robot_count').perform(context))
    ids = LaunchConfiguration('robot_ids').perform(context)
    robots = active_robots(count, ids)
    validate_discovery(os.environ.get('ROS_DISCOVERY_SERVER'), count, ids, os.environ.get('TB4_DISCOVERY_IDS'))
    socket_enabled = LaunchConfiguration('socket').perform(context) == 'true'
    parameters = {'robot_count': count, 'robot_ids': ','.join(robots)}
    if socket_enabled:
        parameters['socket_port'] = int(LaunchConfiguration('socket_port').perform(context))
    for robot in robots:
        with (share() / 'config/robots' / (robot + '.yaml')).open() as stream:
            frames = yaml.safe_load(stream)['frames']
        parameters[robot + '.map_frame'] = frames['map']
        parameters[robot + '.base_frame'] = frames['base']
    with (share() / 'config/fleet.rviz').open() as stream:
        view = rviz_for_count(yaml.safe_load(stream), count, ids)
    with tempfile.NamedTemporaryFile(mode='w', prefix='tb4_fleet_', suffix='.rviz', delete=False) as stream:
        yaml.safe_dump(view, stream, sort_keys=False)
        rviz_path = Path(stream.name)

    def cleanup(context):
        rviz_path.unlink(missing_ok=True)
        return []

    bridge = Node(package='tb4_multi_bringup', executable='socket_control' if socket_enabled else 'fleet_view',
                  name='fleet_view', parameters=[parameters], output='screen')
    rviz = Node(package='rviz2', executable='rviz2', name='fleet_rviz',
                arguments=['-d', str(rviz_path)], condition=IfCondition(LaunchConfiguration('rviz')),
                remappings=[('/tf', '/tb4_view/tf'), ('/tf_static', '/tb4_view/tf_static')],
                output='screen')
    return pc_service_profile(context) + [
        SetEnvironmentVariable('ROS_SUPER_CLIENT', 'False'),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        RegisterEventHandler(OnProcessExit(target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='Fleet display bridge exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason='Fleet RViz closed'))])),
        bridge, rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_count', default_value=os.environ.get('TB4_ROBOT_COUNT', '3'),
                              choices=['1', '2', '3'], description='Number of active robots (robot1..robotN)'),
        DeclareLaunchArgument('robot_ids', default_value=os.environ.get('TB4_ROBOT_IDS', '')),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('socket', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('socket_port', default_value='8765'),
        OpaqueFunction(function=setup),
    ])
