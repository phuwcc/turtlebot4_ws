"""PC owns localization, all Nav2 stacks, socket gateway and a single RViz."""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from tb4_multi_bringup.configuration import share
from tb4_multi_bringup.fleet_settings import active_robots, validate_discovery


def setup(context):
    count = int(LaunchConfiguration('robot_count').perform(context))
    robots = active_robots(count)
    validate_discovery(os.environ.get('ROS_DISCOVERY_SERVER'), count)
    map_path = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    if not map_path.is_file():
        raise ValueError('Map missing on PC: ' + str(map_path))
    directory = share() / 'launch'
    actions = []
    for robot in robots:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(directory / 'navigation.launch.py')),
            launch_arguments={'robot_id': robot, 'map': str(map_path)}.items()))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(directory / 'fleet_visualization.launch.py')),
        launch_arguments={'robot_count': str(count), 'socket': 'true',
                          'rviz': LaunchConfiguration('rviz'),
                          'socket_port': LaunchConfiguration('socket_port')}.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_count', default_value=os.environ.get('TB4_ROBOT_COUNT', '1'),
                              choices=['1', '2', '3']),
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('socket_port', default_value='8765'),
        OpaqueFunction(function=setup),
    ])
