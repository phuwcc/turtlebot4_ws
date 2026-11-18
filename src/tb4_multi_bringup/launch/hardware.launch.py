from launch import LaunchDescription
from launch.actions import GroupAction, OpaqueFunction
from launch.substitutions import Command
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
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
             # The stock power saver calls OAKD services on dock transitions.
             # This profile has no camera; LiDAR handles standby in its driver.
             parameters=[params, {'model': cfg['model'], 'power_saver': False}], output='screen'),
        Node(package='tb4_multi_bringup', executable='remote_velocity',
             name='remote_velocity_guard', parameters=[{'mode': 'guard'}], output='screen'),
        Node(package='create3_republisher', executable='create3_republisher', name='create3_repub',
             parameters=[str(share('create3_republisher') / 'bringup/params.yaml'),
                         {'robot_namespace': cfg['create3_namespace']}], output='screen'),
        Node(package='rplidar_ros', executable='rplidar_composition', name='rplidar_composition',
             parameters=[{'serial_port': '/dev/RPLIDAR', 'serial_baudrate': 115200,
                          'frame_id': frames['lidar'], 'inverted': False,
                          'angle_compensate': True, 'auto_standby': True}], output='screen'),
        # Same URDF/TF as turtlebot4_description/robot_description.launch.py, but the
        # Python joint_state_publisher is replaced by a native C++ publisher.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'use_sim_time': False, 'robot_description': ParameterValue(Command([
                 'xacro ', str(share('turtlebot4_description') / 'urdf' / cfg['model'] / 'turtlebot4.urdf.xacro'),
                 ' gazebo:=ignition namespace:=', ns]), value_type=str)}]),
        Node(package='tb4_multi_bringup', executable='wheel_joint_states',
             name='joint_state_publisher', output='screen'),
    ]
    if cfg['model'] == 'standard':
        actions.append(Node(package='turtlebot4_base', executable='turtlebot4_base_node',
                            name='turtlebot4_base_node', parameters=[params], output='screen'))
    # Camera and joystick are intentionally not started in this LiDAR-only profile.
    return [GroupAction(actions)]


def generate_launch_description():
    return LaunchDescription(robot_arguments() + [OpaqueFunction(function=setup)])
