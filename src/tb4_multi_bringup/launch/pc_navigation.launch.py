"""PC owns localization, all Nav2 stacks, socket gateway and a single RViz."""
import math
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from tb4_multi_bringup.configuration import share
from tb4_multi_bringup.fleet_settings import (active_robots, startup_poses, validate_discovery, localization_hints,
                                              robot_discovery_servers)


def setup(context):
    count = int(LaunchConfiguration('robot_count').perform(context))
    ids = LaunchConfiguration('robot_ids').perform(context)
    robots = active_robots(count, ids)
    coordinated=LaunchConfiguration('coordinated_goals').perform(context)=='true'
    interactive_marco = coordinated and LaunchConfiguration('marco_rviz_goals').perform(context) == 'true'
    # MARCO coordinator (RViz goals or a MACRO-style mission file) supports any
    # fleet of two or more robots; MARCO itself picks the priority branch.
    if coordinated and (len(robots) < 2 or LaunchConfiguration('localization_only').perform(context)=='true'):
        raise ValueError('MARCO requires at least two robots with navigation')
    hints = localization_hints(LaunchConfiguration('localization_hints').perform(context), count, ids, float(LaunchConfiguration('anchor_yaw').perform(context)))
    poses = startup_poses(LaunchConfiguration('initial_poses').perform(context), count, ids)
    robot_servers = robot_discovery_servers()
    validate_discovery(robot_servers, count, ids, os.environ.get('TB4_DISCOVERY_IDS'))
    map_path = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    if not map_path.is_file():
        raise ValueError('Map missing on PC: ' + str(map_path))
    directory = share() / 'launch'
    spacing = float(LaunchConfiguration('robot_startup_spacing_s').perform(context))
    if not math.isfinite(spacing) or spacing < 0:
        raise ValueError('robot_startup_spacing_s must be finite and nonnegative')
    actions = []
    # The PC discovery hub is owned by scripts/pc.sh (nav), which keeps it alive
    # past this launch so the Pi servers learn that PC nodes are gone.
    for index, robot in enumerate(robots):
        arguments = {'robot_id': robot, 'map': str(map_path),
                     'localization_profile': LaunchConfiguration('localization_profile'),
                     'peer_footprints': LaunchConfiguration('peer_footprints'),
                     'peer_ids': ','.join(r for r in robots if r != robot)
                                 if LaunchConfiguration('peer_footprints').perform(context) == 'true' else '',
                     'nav_params_file': LaunchConfiguration('nav_params_file'),
                     # MARCO (CLI or RViz) only drives FollowPath + is_path_valid. The full
                     # Nav2 stack adds four DDS participants per robot whose discovery
                     # traffic saturated office Wi-Fi with three robots.
                     'path_only': str(coordinated).lower(),
                     'localization_only': LaunchConfiguration('localization_only'),
                     'initialize_pose': 'true' if robot in poses else 'false'}
        arguments.update(poses.get(robot, {}))
        arguments.update(hints.get(robot, {}))
        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(directory / 'navigation.launch.py')),
            launch_arguments=arguments.items())
        if spacing and index:
            robot_launch = TimerAction(period=index * spacing, actions=[robot_launch])
        actions.append(robot_launch)
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(directory / 'fleet_visualization.launch.py')),
        launch_arguments={'robot_count': str(count), 'robot_ids': ','.join(robots), 'socket': 'true',
                          'require_localization': 'true', 'navigation_tf': 'true',
                          'coordinated_goals': LaunchConfiguration('coordinated_goals'),
                          'show_path_goals': str(coordinated and not interactive_marco).lower(),
                          'path_controller': LaunchConfiguration('path_controller'),
                          'rviz': LaunchConfiguration('rviz'),
                          'socket_port': LaunchConfiguration('socket_port')}.items()))
    if LaunchConfiguration('peer_footprints').perform(context) == 'true':
        from launch_ros.actions import Node
        actions.append(Node(package='tb4_multi_bringup', executable='fleet_footprints', name='fleet_footprints',
                            parameters=[{'robot_ids': ','.join(robots), 'map_topic': '/' + robots[0] + '/map'}],
                            output='screen'))
    if interactive_marco:
        from launch_ros.actions import Node
        from launch.actions import RegisterEventHandler, EmitEvent
        from launch.event_handlers import OnProcessExit
        from launch.events import Shutdown
        coordinator=Node(package='tb4_marco_bridge',executable='marco_rviz',name='marco_rviz',
            parameters=[{'robot_ids':','.join(robots),'socket_port':LaunchConfiguration('socket_port'),
                         'speed_mps':float(LaunchConfiguration('marco_speed_mps').perform(context)),
                         # Max path deviation before abort; with safety_margin 0.015 a robot
                         # passing a held peer keeps >= ~6 cm body gap in the worst case.
                         'tracking_tolerance':.06,'safety_margin':.015,
                         'mission_file':LaunchConfiguration('marco_mission')}],output='screen')
        actions += [RegisterEventHandler(OnProcessExit(target_action=coordinator,
            on_exit=[EmitEvent(event=Shutdown(reason='MARCO coordinator exited'))])),coordinator]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_count', default_value=os.environ.get('TB4_ROBOT_COUNT', '1'),
                              choices=['1', '2', '3', '4']),
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('localization_profile', default_value='strict', choices=['strict','navigation']),
        DeclareLaunchArgument('peer_footprints', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('coordinated_goals', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('marco_speed_mps', default_value='0.16',
                              description='Speed the MARCO coordinator predicts with; must match the Nav2 controller'),
        DeclareLaunchArgument('marco_mission', default_value='',
                              description='MACRO-style mission JSON (objects + task DAG) run by the MARCO coordinator'),
        DeclareLaunchArgument('marco_rviz_goals', default_value='true', choices=['true','false'],
                              description='Start the RViz goal coordinator when using MARCO'),
        DeclareLaunchArgument('path_controller', default_value='FollowPath'),
        DeclareLaunchArgument('anchor_yaw', default_value='0.0'),
        DeclareLaunchArgument('localization_hints', default_value=''),
        DeclareLaunchArgument('nav_params_file', default_value=str(share() / 'config/common/nav2.yaml')),
        DeclareLaunchArgument('initial_poses', default_value='',
                              description='YAML with x, y, yaw (map, radians) per active robot; '
                                          'empty waits for RViz 2D Pose Estimate'),
        DeclareLaunchArgument('robot_ids', default_value=os.environ.get('TB4_ROBOT_IDS', '')),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('localization_only', default_value='false', choices=['true', 'false'],
                              description='Only map/localization/RViz; navigation goals unavailable'),
        DeclareLaunchArgument('socket_port', default_value='8765'),
        DeclareLaunchArgument('robot_startup_spacing_s', default_value='0',
                              description='Delay between starting each robot namespace on the PC'),
        OpaqueFunction(function=setup),
    ])
