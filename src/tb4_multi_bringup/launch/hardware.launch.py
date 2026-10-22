from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from nav2_common.launch import RewrittenYaml
from tb4_multi_bringup.configuration import load_robot, robot_arguments, share


def setup(context):
    cfg = load_robot(context)
    ns, frames = cfg['namespace'], cfg['frames']
    params = RewrittenYaml(source_file=str(share('turtlebot4_bringup') / 'config/turtlebot4.yaml'),
                           root_key=ns, param_rewrites={}, convert_types=True)
    actions = [PushRosNamespace(ns), SetRemap('/tf', '/' + ns + '/tf'),
               SetRemap('/tf_static', '/' + ns + '/tf_static'),
        Node(package='turtlebot4_node', executable='turtlebot4_node', name='turtlebot4_node',
             parameters=[params, {'model': cfg['model']}], output='screen'),
        Node(package='tb4_multi_bringup', executable='remote_velocity',
             name='remote_velocity_guard', parameters=[{'mode': 'guard'}], output='screen'),
        Node(package='create3_republisher', executable='create3_republisher', name='create3_repub',
             parameters=[str(share('create3_republisher') / 'bringup/params.yaml'),
                         {'robot_namespace': cfg['create3_namespace']}], output='screen'),
        Node(package='rplidar_ros', executable='rplidar_composition', name='rplidar_composition',
             parameters=[{'serial_port': '/dev/RPLIDAR', 'serial_baudrate': 115200,
                          'frame_id': frames['lidar'], 'inverted': False,
                          'angle_compensate': True, 'auto_standby': True}], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            share('turtlebot4_description') / 'launch/robot_description.launch.py')),
            launch_arguments={'model': cfg['model'], 'namespace': ns, 'use_sim_time': 'false'}.items()),
    ]
    if cfg['model'] == 'standard':
        actions.append(Node(package='turtlebot4_base', executable='turtlebot4_base_node',
                            name='turtlebot4_base_node', parameters=[params], output='screen'))
    # Camera and joystick are intentionally not started in this LiDAR-only profile.
    return [GroupAction(actions)]


def generate_launch_description():
    return LaunchDescription(robot_arguments() + [OpaqueFunction(function=setup)])
