from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from tb4_multi_bringup.configuration import load_robot, robot_arguments, share


def setup(context):
    cfg = load_robot(context)
    nav = share('nav2_bringup')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav / 'launch/rviz_launch.py')),
        launch_arguments={'namespace': cfg['namespace'], 'use_namespace': 'true',
                          'rviz_config': str(nav / 'rviz/nav2_namespaced_view.rviz')}.items())]


def generate_launch_description():
    return LaunchDescription(robot_arguments() + [OpaqueFunction(function=setup)])
