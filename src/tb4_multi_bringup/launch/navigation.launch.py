from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap, SetParametersFromFile
from nav2_common.launch import RewrittenYaml
from tb4_multi_bringup.configuration import initial_pose, load_robot, robot_arguments, share, pc_service_profile


def setup(context):
    cfg = load_robot(context)
    ns, frames = cfg['namespace'], cfg['frames']
    map_file = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    if not map_file.is_file():
        raise ValueError('Map YAML does not exist: ' + str(map_file))
    initialize, pose = initial_pose(context)
    localization_only = LaunchConfiguration('localization_only').perform(context) == 'true'
    amcl = RewrittenYaml(
        source_file=str(share() / 'config/common/amcl.yaml'),
        param_rewrites={
            'use_sim_time': 'false', 'set_initial_pose': str(initialize).lower(),
            'base_frame_id': frames['base'], 'odom_frame_id': frames['odom'],
            'global_frame_id': frames['map'], 'scan_topic': '/' + ns + '/scan',
            'amcl.ros__parameters.initial_pose.x': str(pose[0]),
            'amcl.ros__parameters.initial_pose.y': str(pose[1]),
            'amcl.ros__parameters.initial_pose.yaw': str(pose[2]),
        }, convert_types=True)
    nav = RewrittenYaml(
        source_file=str(share() / 'config/common/nav2.yaml'),
        param_rewrites={
            'use_sim_time': 'false', 'robot_base_frame': frames['base'],
            'odom_topic': '/' + ns + '/odom', 'topic': '/' + ns + '/scan',
            'bt_navigator.ros__parameters.global_frame': frames['map'],
            'local_costmap.local_costmap.ros__parameters.global_frame': frames['odom'],
            'global_costmap.global_costmap.ros__parameters.global_frame': frames['map'],
            'behavior_server.ros__parameters.global_frame': frames['odom'],
        }, convert_types=True)
    launch_dir = share('nav2_bringup') / 'launch'
    lifecycle = RewrittenYaml(source_file=str(share() / 'config/common/lifecycle.yaml'),
                              root_key=ns, param_rewrites={}, convert_types=True)
    actions = pc_service_profile(context) + [SetEnvironmentVariable('ROS_SUPER_CLIENT', 'False'),
               SetParametersFromFile(lifecycle), PushRosNamespace(ns), SetRemap('/tf', '/' + ns + '/tf'),
               SetRemap('/tf_static', '/' + ns + '/tf_static'),
               # Node-specific rules preserve controller -> velocity smoother wiring.
               SetRemap('velocity_smoother:cmd_vel_smoothed', 'pc_cmd_vel'),
               SetRemap('behavior_server:cmd_vel', 'pc_cmd_vel'),
               SetRemap('/' + ns + '/global_costmap/scan', '/' + ns + '/scan'),
               SetRemap('/' + ns + '/local_costmap/scan', '/' + ns + '/scan'),
               SetRemap('/' + ns + '/global_costmap/map', '/' + ns + '/map'),
               SetRemap('/' + ns + '/local_costmap/map', '/' + ns + '/map')]
    actions += [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_dir / 'localization_launch.py')),
            launch_arguments={'namespace': ns, 'map': str(map_file), 'use_sim_time': 'false',
                              'params_file': amcl, 'use_composition': 'False'}.items()),
    ]
    if not localization_only:
        actions += [
            Node(package='tb4_multi_bringup', executable='remote_velocity',
                 name='remote_velocity_stamp', parameters=[{'mode': 'stamp'}], output='screen'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_dir / 'navigation_launch.py')),
                launch_arguments={'namespace': ns, 'use_sim_time': 'false', 'params_file': nav,
                                  'use_composition': 'False'}.items()),
        ]
    return [GroupAction(actions)]


def generate_launch_description():
    return LaunchDescription(robot_arguments() + [
        DeclareLaunchArgument('map', description='Absolute path to the shared saved map YAML'),
        DeclareLaunchArgument('localization_only', default_value='false', choices=['true', 'false'],
                              description='Map, AMCL and display only; no navigation or velocity relay'),
        DeclareLaunchArgument('initialize_pose', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('initial_x', default_value=''),
        DeclareLaunchArgument('initial_y', default_value=''),
        DeclareLaunchArgument('initial_yaw', default_value=''),
        OpaqueFunction(function=setup),
    ])
