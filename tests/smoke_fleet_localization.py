#!/usr/bin/env python3
"""Two AMCL/EKF stacks and URDF publishers in isolated DDS.

Checks startup poses, RViz's complete prefixed model TF and subsequent pose resets.
No navigation goals or motor commands are sent.
"""
import json
import argparse
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET

os.environ.update(ROS_DOMAIN_ID='176', ROS_LOCALHOST_ONLY='1',
                  RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_SUPER_CLIENT='False')
for key in ('ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE',
            'FASTDDS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI', 'ROS_NAMESPACE',
            'TB4_ROBOT_IDS', 'TB4_DISCOVERY_IDS'):
    os.environ.pop(key, None)

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--rviz', action='store_true', help='Also open the fleet RViz window')
args = parser.parse_args()

root = Path(__file__).resolve().parents[1]
logs = root / 'log/fleet_localization_smoke'
logs.mkdir(parents=True, exist_ok=True)
(logs / 'result.json').unlink(missing_ok=True)
os.environ['ROS_LOG_DIR'] = str(logs)
image = bytes(254 if 0 < x < 99 and 0 < y < 99 else 0
              for y in range(100) for x in range(100))
(logs / 'room.pgm').write_bytes(b'P5\n100 100\n255\n' + image)
(logs / 'room.yaml').write_text('image: room.pgm\nresolution: 0.1\n'
                              'origin: [-5.0, -5.0, 0.0]\nnegate: 0\n'
                              'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
positions = {'robot1': 0.0, 'robot2': 1.0}
(logs / 'poses.yaml').write_text(yaml.safe_dump({
    robot: dict(x=x, y=0.0, yaw=0.0) for robot, x in positions.items()}))
rclpy.init(args=[])
node = rclpy.create_node('fleet_localization_test', cli_args=[
    '--ros-args', '-r', '/tf:=/tb4_view/tf', '-r', '/tf_static:=/tb4_view/tf_static'])
buffer = Buffer()
listener = TransformListener(buffer, node)
processes, streams, publishers, visual_links, descriptions, subscriptions = [], [], {}, {}, {}, []
filtered, tf_parents = {}, {}
velocity = {robot: (0., 0.) for robot in positions}
with socket.socket() as probe:
    probe.bind(('127.0.0.1', 0))
    socket_port = probe.getsockname()[1]


def start(command, filename):
    stream = (logs / filename).open('w')
    streams.append(stream)
    processes.append(subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                      start_new_session=True))


def sensors():
    stamp = node.get_clock().now().to_msg()
    for robot, x in positions.items():
        pubs = publishers[robot]
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'odom'
        odom.child_frame_id, odom.pose.pose.orientation.w = 'base_link', 1.0
        odom.pose.pose.position.x = x
        odom.twist.twist.linear.x = velocity[robot][0]
        pubs['odom'].publish(odom)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'imu'
        imu.angular_velocity.z = velocity[robot][1]
        pubs['imu'].publish(imu)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id, transform.transform.rotation.w = 'base_link', 1.0
        transform.transform.translation.x = x
        pubs['tf'].publish(TFMessage(transforms=[transform]))
        joints = JointState()
        joints.header.stamp = stamp
        joints.name, joints.position = pubs['joints'], [0.0] * len(pubs['joints'])
        pubs['joint_states'].publish(joints)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'rplidar_link'
        scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / 180
        scan.angle_max = scan.angle_min + 179 * scan.angle_increment
        scan.range_min, scan.range_max, scan.scan_time = 0.15, 12.0, 0.1
        # Match the URDF's lidar translation and pi/2 rotation in the room.
        for index in range(180):
            angle = scan.angle_min + index * scan.angle_increment + math.pi / 2
            dx, dy = math.cos(angle), math.sin(angle)
            rx = ((4.9 if dx >= 0 else -4.9) - (x - 0.04)) / dx if abs(dx) > 1e-6 else 1e6
            ry = (4.9 if dy >= 0 else -4.9) / dy if abs(dy) > 1e-6 else 1e6
            scan.ranges.append(min(rx, ry))
        pubs['scan'].publish(scan)


def wait_for(predicate, timeout=65):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        assert all(p.poll() is None for p in processes), 'A test process exited; inspect ' + str(logs)
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError('Timed out; inspect ' + str(logs))


def gateway(**request):
    try:
        with socket.create_connection(('127.0.0.1', socket_port), timeout=2) as connection:
            connection.sendall((json.dumps(request) + '\n').encode())
            return json.loads(connection.makefile('rb').readline())
    except (OSError, json.JSONDecodeError):
        return {}


def fleet_ready():
    states = gateway(command='snapshot').get('robots', {})
    return set(states) == set(positions) and all(state['localized'] for state in states.values())


def record_tf(msg):
    for tf in msg.transforms:
        tf_parents.setdefault(tf.child_frame_id, set()).add(tf.header.frame_id)


try:
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for robot in positions:
        urdf = subprocess.check_output(['xacro', str(Path(get_package_share_directory(
            'turtlebot4_description')) / 'urdf/standard/turtlebot4.urdf.xacro'),
            'gazebo:=ignition', 'namespace:=' + robot], text=True)
        model = ET.fromstring(urdf)
        visual_links[robot] = [link.attrib['name'] for link in model.findall('link')
                               if link.find('visual') is not None]
        params = logs / (robot + '_description.yaml')
        params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'robot_description': urdf}}}))
        start(['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', '--ros-args',
               '-r', '__ns:=/' + robot, '-r', '/tf:=tf', '-r', '/tf_static:=tf_static',
               '--params-file', str(params)], robot + '_model.log')
        publishers[robot] = {topic: node.create_publisher(kind, '/' + robot + '/' + topic, qos)
            for topic, kind, qos in [('odom', Odometry, qos_profile_sensor_data),
                                     ('imu', Imu, qos_profile_sensor_data),
                                     ('scan', LaserScan, qos_profile_sensor_data),
                                     ('tf', TFMessage, 20), ('joint_states', JointState, 10)]}
        publishers[robot]['joints'] = [joint.attrib['name'] for joint in model.findall('joint')
                                      if joint.attrib['type'] != 'fixed' and joint.find('mimic') is None]
        subscriptions.append(node.create_subscription(String, '/' + robot + '/robot_description',
            lambda msg, r=robot: descriptions.__setitem__(r, msg.data), latched))
        subscriptions.append(node.create_subscription(Odometry, f'/{robot}/odometry/filtered',
            lambda msg, r=robot: filtered.__setitem__(r, msg), qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(TFMessage, f'/{robot}/navigation/tf',
                                                       record_tf, qos_profile_sensor_data))
    timer = node.create_timer(0.1, sensors)
    start(['ros2', 'launch', 'tb4_multi_bringup', 'pc_navigation.launch.py', 'robot_count:=2',
           'rviz:=' + str(args.rviz).lower(), 'socket_port:=' + str(socket_port), 'map:=' + str(logs / 'room.yaml'),
           'initial_poses:=' + str(logs / 'poses.yaml')], 'navigation.log')

    def complete_trees():
        return len(descriptions) == 2 and all(buffer.can_transform(
            'map', robot + '/' + link, rclpy.time.Time())
            for robot in positions for link in visual_links[robot])

    wait_for(complete_trees)
    wait_for(fleet_ready)
    locations = {robot: buffer.lookup_transform('map', robot + '/base_link',
                 rclpy.time.Time()).transform.translation.x for robot in positions}
    for robot, x in positions.items():
        assert abs(locations[robot] - x) < 0.35, (robot, locations)
        assert filtered[robot].header.frame_id == f'{robot}/odom'
        assert filtered[robot].child_frame_id == f'{robot}/base_link'
        assert tf_parents[f'{robot}/odom'] == {'map'}
        assert tf_parents[f'{robot}/base_link'] == {f'{robot}/odom'}
        tf_publishers = node.get_publishers_info_by_topic(f'/{robot}/navigation/tf')
        names = [endpoint.node_name for endpoint in tf_publishers]
        for authority in ('amcl', 'ekf_filter_node', 'localization_inputs'):
            assert names.count(authority) == 1, (robot, names)
        assert not {'create3_repub', 'robot_state_publisher'} & set(names), (robot, names)
    assert 'base_link' not in tf_parents and 'odom' not in tf_parents
    assert all(name != 'slam_toolbox' for name, _ in node.get_node_names_and_namespaces())
    velocity['robot1'] = (.06, .15)
    wait_for(lambda: filtered['robot1'].twist.twist.linear.x > .04
             and filtered['robot1'].twist.twist.angular.z > .1, timeout=8)
    velocity['robot1'] = (0., 0.)
    wait_for(lambda: abs(filtered['robot1'].twist.twist.linear.x) < .01
             and abs(filtered['robot1'].twist.twist.angular.z) < .02, timeout=8)
    for robot in positions:
        assert any(endpoint.node_name == 'amcl' for endpoint in
                   node.get_subscriptions_info_by_topic('/' + robot + '/initialpose'))
    # Reset an automatically initialized robot through the same gateway used by RViz.
    positions['robot2'] = 2.0
    response = gateway(command='pose', robot='robot2', x=2., y=0., yaw=0.)
    assert response['ok'], response
    wait_for(lambda: abs(buffer.lookup_transform('map', 'robot2/base_link',
             rclpy.time.Time()).transform.translation.x - 2.0) < 0.35, timeout=20)
    wait_for(fleet_ready, timeout=20)
    assert abs(buffer.lookup_transform('map', 'robot1/base_link',
               rclpy.time.Time()).transform.translation.x - locations['robot1']) < 0.15
    result = {'robots': locations, 'visual_links_with_tf': {
        r: len(links) for r, links in visual_links.items()}, 'manual_reset_after_auto_init': True,
        'marco_global_poses_ready': True, 'navigation_tf_authorities_checked': True, 'fused_wheel_and_gyro': True}
    (logs / 'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2), flush=True)
    if args.rviz:
        preview_end = time.monotonic() + 15
        while time.monotonic() < preview_end:
            rclpy.spin_once(node, timeout_sec=0.05)
finally:
    for process in processes:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    for process in processes:
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
    for stream in streams:
        stream.close()
    node.destroy_node()
    rclpy.try_shutdown()
