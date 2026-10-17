from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from tb4_multi_bringup.configuration import load_robot, robot_arguments, share


def setup(context):
    cfg = load_robot(context)
    ns, frames = cfg['namespace'], cfg['frames']
    params = RewrittenYaml(source_file=str(share() / 'config/common/slam.yaml'),
                           root_key=ns, param_rewrites={}, convert_types=True)
    origin = LaunchConfiguration('origin_file').perform(context)
    return [Node(
        package='tb4_multi_tools', executable='start_relative_mapping',
        namespace=ns, name='mapping_start_origin', output='screen',
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        parameters=[{
            'slam_params_file': params,
            'origin_file': origin or '/tmp/tb4_multi_origin_' + ns + '.json',
            'odom_frame': frames['odom'], 'base_frame': frames['base'],
            'mapping_frame': frames['mapping'], 'map_frame': frames['map'],
            'odom_topic': 'odom', 'capture_timeout': float(LaunchConfiguration('capture_timeout').perform(context)),
        }])]


def generate_launch_description():
    return LaunchDescription(robot_arguments() + [
        DeclareLaunchArgument('origin_file', default_value=''),
        DeclareLaunchArgument('capture_timeout', default_value='25.0'),
        OpaqueFunction(function=setup),
    ])
