"""Mapping and fleet display on PC; the Pi only streams hardware data."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from tb4_multi_bringup.configuration import robot_arguments, share


def generate_launch_description():
    directory = share() / 'launch'
    return LaunchDescription(robot_arguments() + [
        DeclareLaunchArgument('robot_count', default_value=os.environ.get('TB4_ROBOT_COUNT', '1')),
        DeclareLaunchArgument('origin_file', default_value=''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(directory / 'mapping.launch.py')),
            launch_arguments={'robot_id': LaunchConfiguration('robot_id'),
                              'robot_config': LaunchConfiguration('robot_config'),
                              'origin_file': LaunchConfiguration('origin_file')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(directory / 'fleet_visualization.launch.py')),
            launch_arguments={'robot_count': LaunchConfiguration('robot_count'), 'socket': 'true'}.items()),
    ])
