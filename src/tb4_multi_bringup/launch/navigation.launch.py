import math
from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap, SetParametersFromFile
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from tb4_multi_bringup.configuration import initial_pose, load_robot, pc_service_profile, robot_arguments, share


def setup(context):
    cfg = load_robot(context)
    ns, raw = cfg['namespace'], cfg['frames']
    frames = {**raw, 'odom': f"{ns}/{raw['odom']}", 'base': f"{ns}/{raw['base']}"}
    map_file = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    if not map_file.is_file():
        raise ValueError('Map YAML does not exist: ' + str(map_file))
    initialize, pose = initial_pose(context)
    use_hint = LaunchConfiguration('initial_hint').perform(context) == 'true'
    if use_hint and not initialize:
        pose = [float(LaunchConfiguration(key).perform(context)) for key in ('hint_x', 'hint_y', 'hint_yaw')]
        if not all(math.isfinite(value) for value in pose):
            raise ValueError('AMCL initial pose must be finite')
    localization_only = LaunchConfiguration('localization_only').perform(context) == 'true'
    path_only = LaunchConfiguration('path_only').perform(context) == 'true'
    interactive = LaunchConfiguration('localization_profile').perform(context) in ('navigation', 'demo')
    # With peer footprints the local costmap static layer reads the shared map plus
    # the other robots' bodies (fleet_footprints); the global costmap keeps the clean map.
    costmap_map = '/' + ns + '/fleet_map' if LaunchConfiguration('peer_footprints').perform(context) == 'true' else 'map'
    nav_params_file = Path(LaunchConfiguration('nav_params_file').perform(context)).expanduser().resolve()
    if not nav_params_file.is_file():
        raise ValueError('Nav2 parameters do not exist: ' + str(nav_params_file))
    amcl = yaml.safe_load((share() / 'config/common/amcl.yaml').read_text())
    ekf = yaml.safe_load((share() / 'config/common/ekf.yaml').read_text())
    odom_topic = f'/{ns}/odometry/filtered'
    nav = RewrittenYaml(
        source_file=str(nav_params_file),
        param_rewrites={
            'use_sim_time': 'false', 'robot_base_frame': frames['base'],
            'odom_topic': odom_topic, 'topic': '/' + ns + '/scan',
            'bt_navigator.ros__parameters.global_frame': frames['map'],
            'local_costmap.local_costmap.ros__parameters.global_frame': frames['odom'],
            'global_costmap.global_costmap.ros__parameters.global_frame': frames['map'],
            'behavior_server.ros__parameters.global_frame': frames['odom'],
            # MARCO nav2_tb1.yaml: only the local costmap sees peer bodies; the
            # global costmap (path validation) keeps the clean map so a nominal
            # route through a peer that will move is not rejected up front.
            'local_costmap.local_costmap.ros__parameters.static_layer.map_topic': costmap_map,
            'global_costmap.global_costmap.ros__parameters.static_layer.map_topic': 'map',
        }, convert_types=True)
    launch_dir = share('nav2_bringup') / 'launch'
    lifecycle = RewrittenYaml(source_file=str(share() / 'config/common/lifecycle.yaml'),
                              root_key=ns, param_rewrites={}, convert_types=True)
    actions = pc_service_profile(context) + [SetEnvironmentVariable('ROS_SUPER_CLIENT', 'False'),
               SetParametersFromFile(lifecycle), PushRosNamespace(ns),
               SetRemap('/tf', f'/{ns}/navigation/tf'), SetRemap('/tf_static', f'/{ns}/navigation/tf_static'),
               # Node-specific rules preserve controller -> velocity smoother wiring.
               SetRemap('controller_server:odom', odom_topic),
               SetRemap('velocity_smoother:cmd_vel_smoothed', 'pc_cmd_vel'),
               SetRemap('behavior_server:cmd_vel', 'pc_cmd_vel'),
               SetRemap('/' + ns + '/global_costmap/scan', '/' + ns + '/scan'),
               SetRemap('/' + ns + '/local_costmap/scan', '/' + ns + '/scan'),
               SetRemap('/' + ns + '/global_costmap/map', '/' + ns + '/map'),
               SetRemap('/' + ns + '/local_costmap/map', '/' + ns + '/map')]
    inputs = Node(package='tb4_multi_bringup', executable='localization_inputs', output='screen',
                  parameters=[ekf['localization_inputs']['ros__parameters'],
                              {'raw_base_frame': raw['base'], 'raw_odom_frame': raw['odom'],
                               'map_frame': raw['map']}])
    filter_node = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
                       parameters=[ekf['ekf_filter_node']['ros__parameters'],
                                   {'map_frame': frames['map'], 'world_frame': frames['odom'],
                                    'odom_frame': frames['odom'], 'base_link_frame': frames['base']}],
                       output='screen')
    localizer = Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
                     remappings=[('amcl_pose', 'pose')],
                     parameters=[amcl['amcl']['ros__parameters'],
                                 {'global_frame_id': frames['map'], 'odom_frame_id': frames['odom'],
                                  'base_frame_id': frames['base'], 'scan_topic': f'/{ns}/scan',
                                  'map_topic': 'map', 'set_initial_pose': initialize or use_hint,
                                  'initial_pose': {'x': pose[0], 'y': pose[1], 'yaw': pose[2]},
                                  'update_min_d': .02, 'update_min_a': .05}])
    supervisor = Node(package='tb4_multi_bringup', executable='localization_supervisor', output='screen',
                      parameters=[{'base_frame': frames['base'], 'map_frame': frames['map'],
                                   'initial_pose_known': initialize or use_hint,
                                   'hint_x': pose[0], 'hint_y': pose[1],
                                   'hint_std_xy': float(LaunchConfiguration('hint_std_xy').perform(context)),
                                   'navigation': not localization_only,
                                   'localization_profile': LaunchConfiguration('localization_profile'),
                                   'peer_ids': LaunchConfiguration('peer_ids')}])
    for node in (inputs, filter_node, localizer, supervisor):
        actions.extend([RegisterEventHandler(OnProcessExit(target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='Localization process exited'))])), node])
    actions.extend([
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': str(map_file), 'frame_id': frames['map'], 'topic_name': 'map'}],
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}], output='screen'),
    ])
    if not localization_only:
        actions.append(Node(package='tb4_multi_bringup', executable='remote_velocity',
                            name='remote_velocity_stamp', parameters=[{'mode': 'stamp', 'require_permission': True,
                                'max_linear_speed': .16 if interactive else .05,
                                'max_angular_speed': .60 if interactive else .2}], output='screen'))
        if path_only:
            params = ParameterFile(RewrittenYaml(source_file=nav, root_key=ns,
                                                  param_rewrites={}, convert_types=True), allow_substs=True)
            actions += [
                Node(package='nav2_controller', executable='controller_server',
                     parameters=[params], remappings=[('cmd_vel', 'cmd_vel_nav')], output='screen'),
                Node(package='nav2_planner', executable='planner_server', parameters=[params], output='screen'),
                Node(package='nav2_velocity_smoother', executable='velocity_smoother', parameters=[params],
                     remappings=[('cmd_vel', 'cmd_vel_nav')], output='screen'),
                Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                     name='lifecycle_manager_navigation', output='screen',
                     parameters=[{'use_sim_time': False, 'autostart': False,
                                  'node_names': ['controller_server', 'planner_server', 'velocity_smoother']}]),
            ]
        else:
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_dir / 'navigation_launch.py')),
                launch_arguments={'namespace': ns, 'use_sim_time': 'false', 'params_file': nav,
                                  'autostart': 'false',
                                  'use_composition': 'False'}.items()))
    return [GroupAction(actions)]


def generate_launch_description():
    return LaunchDescription(robot_arguments() + [
        DeclareLaunchArgument('map', description='Absolute path to the shared saved map YAML'),
        DeclareLaunchArgument('localization_profile', default_value='strict', choices=['strict','navigation','demo']),
        DeclareLaunchArgument('peer_ids', default_value='',
                              description='Comma list of other robots whose bodies are excluded from scan-match scoring'),
        DeclareLaunchArgument('peer_footprints', default_value='false', choices=['true', 'false'],
                              description='Costmaps use /<robot>/fleet_map with peer footprints'),
        DeclareLaunchArgument('initial_hint', default_value='false', choices=['true','false']),
        *[DeclareLaunchArgument(key, default_value=value) for key,value in [('hint_x','0.0'),('hint_y','0.0'),('hint_yaw','0.0'),('hint_std_xy','0.2'),('hint_std_yaw','0.2')]],
        DeclareLaunchArgument('nav_params_file', default_value=str(share() / 'config/common/nav2.yaml'),
                              description='Nav2 YAML; accepts a separate low-speed validation profile'),
        DeclareLaunchArgument('localization_only', default_value='false', choices=['true', 'false'],
                              description='Map, AMCL and EKF only; no navigation or velocity relay'),
        DeclareLaunchArgument('path_only', default_value='false', choices=['true', 'false'],
                              description='Load the Nav2 servers used by guarded FollowPath missions'),
        DeclareLaunchArgument('initialize_pose', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('initial_x', default_value=''),
        DeclareLaunchArgument('initial_y', default_value=''),
        DeclareLaunchArgument('initial_yaw', default_value=''),
        OpaqueFunction(function=setup),
    ])
