"""Three original robot TF trees -> one isolated display TF tree on the laptop."""
import yaml

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from tb4_multi_bringup.configuration import share


def generate_launch_description():
    parameters = {}
    for robot in ('robot1', 'robot2', 'robot3'):
        with (share() / 'config/robots' / (robot + '.yaml')).open() as stream:
            frames = yaml.safe_load(stream)['frames']
        parameters[robot + '.map_frame'] = frames['map']
        parameters[robot + '.base_frame'] = frames['base']
    bridge = Node(package='tb4_multi_bringup', executable='fleet_view',
                  name='fleet_view', parameters=[parameters], output='screen')
    rviz = Node(package='rviz2', executable='rviz2', name='fleet_rviz',
                arguments=['-d', str(share() / 'config/fleet.rviz')],
                remappings=[('/tf', '/tb4_view/tf'), ('/tf_static', '/tb4_view/tf_static')],
                output='screen')
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='Fleet display bridge exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason='Fleet RViz closed'))])),
        bridge, rviz,
    ])
