"""Launch the T corridor and three TurtleBot4 models with an optional Gazebo GUI."""
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node

import sys
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from t_junction import junction


def generate_launch_description():
    tb4 = HERE.parent / 'MACRO/src/simulations/ros2/tb4'
    output = Path(os.environ['TB4_SIM_OUTPUT'])
    world_file = output / 't_junction.sdf'
    tree = ET.parse(tb4 / 'worlds/three_robot_t.sdf')
    world = tree.find('world')
    world.find('physics/max_step_size').text = '0.001'
    world.find("model[@name='floor']/pose").text = '2.125 1.475 -.05 0 0 0'
    for model in world.findall('model'):
        if model.attrib['name'] != 'floor':
            world.remove(model)
    grid, poses = junction()
    mapping = os.environ.get('TB4_SIM_MAPPING') == '1'
    if mapping:
        poses = {'robot1': poses['robot1']}
    width = grid['width']
    resolution = grid['resolution']
    for y in range(grid['height']):
        row = grid['data'][y * width:(y + 1) * width] + [0]
        start = None
        for x, occupied in enumerate(row):
            if occupied and start is None:
                start = x
            if not occupied and start is not None:
                model = ET.SubElement(world, 'model', name=f'wall_{y}_{start}')
                ET.SubElement(model, 'static').text = 'true'
                ET.SubElement(model, 'pose').text = f'{(start+x)*resolution/2} {(y+.5)*resolution} .25 0 0 0'
                link = ET.SubElement(model, 'link', name='wall')
                for kind in ('collision', 'visual'):
                    element = ET.SubElement(link, kind, name=kind)
                    box = ET.SubElement(ET.SubElement(element, 'geometry'), 'box')
                    ET.SubElement(box, 'size').text = f'{(x-start)*resolution} {resolution} .5'
                start = None
    tree.write(world_file)
    resources = [str(Path(get_package_share_directory(package)).parent)
                 for package in ('turtlebot4_description', 'irobot_create_description')]
    gazebo_command = ['ign', 'gazebo', '-r']
    if os.environ.get('TB4_SIM_GUI') != '1':
        gazebo_command += ['-s', '--headless-rendering']
    gazebo_command.append(str(world_file))
    actions = [
        ExecuteProcess(cmd=gazebo_command,
                       additional_env={'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.pathsep.join([
                           str(Path(get_package_share_directory('ign_ros2_control')).parents[1] / 'lib'),
                           os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')]),
                           'IGN_GAZEBO_RESOURCE_PATH': os.pathsep.join(
                           resources + [os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])}),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(HERE / ('mapping.rviz' if mapping else 'gazebo.rviz'))],
             parameters=[{'use_sim_time': True}]),
    ]
    if mapping:
        scan = '/world/three_robot_t/model/robot1/turtlebot4/link/rplidar_link/sensor/rplidar/scan'
        slam_config = Path(get_package_share_directory('slam_toolbox')) / 'config/mapper_params_online_async.yaml'
        actions.extend([
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 arguments=[scan + '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
                 remappings=[(scan, '/sim/raw_scan')]),
            Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
                 parameters=[str(slam_config), {
                     'use_sim_time': True, 'odom_frame': 'robot1/odom', 'base_frame': 'robot1/base_link',
                     'scan_topic': '/robot1/scan', 'map_update_interval': 1.0,
                     'minimum_time_interval': .2, 'minimum_travel_distance': .08,
                     'minimum_travel_heading': .12,
                     'min_laser_range': .164, 'max_laser_range': 12.,
                 }]),
        ])
    for index, (robot, (x, y, yaw)) in enumerate(poses.items()):
        description = Node(
            package='robot_state_publisher', executable='robot_state_publisher', namespace=robot,
            parameters=[{'use_sim_time': True, 'frame_prefix': robot + '/',
                         'robot_description': Command(
                ['python3 ', str(tb4 / 'make_robot_description.py'), ' standard ', robot])}])
        entity = Node(
            package='ros_ign_gazebo', executable='create', namespace=robot,
            arguments=['-name', robot + '/turtlebot4', '-x', str(x), '-y', str(y),
                       '-z', '0.0', '-Y', str(yaw), '-topic', 'robot_description'])
        control = Path(get_package_share_directory('irobot_create_control')) / 'config/control.yaml'
        joint_state = Node(package='controller_manager', executable='spawner', namespace=robot,
                           arguments=['joint_state_broadcaster', '-c', 'controller_manager'])
        drive = Node(package='controller_manager', executable='spawner', namespace=robot,
                     parameters=[str(control)],
                     arguments=['diffdrive_controller', '-c', 'controller_manager'])
        # Fleet publishes each map-to-base transform directly from odometry.
        # The TF aliases used by the separate Nav2 demo are unnecessary here.
        actions.append(TimerAction(period=2. + 3. * index, actions=[
            RegisterEventHandler(OnProcessExit(target_action=joint_state, on_exit=[drive])),
            description, entity, joint_state]))
    return LaunchDescription(actions)
