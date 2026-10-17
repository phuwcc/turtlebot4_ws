#!/usr/bin/env python3
"""Real SLAM binaries with fake sensors on an isolated, localhost-only DDS domain."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

# Never attach the fake robots or command publishers to the real robot domain.
os.environ['ROS_DOMAIN_ID'] = '167'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ['ROS_SUPER_CLIENT'] = 'False'
for key in ('ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'ROS_NAMESPACE'):
    os.environ.pop(key, None)

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


root = Path(__file__).resolve().parents[1]
logs = root / 'log/two_robot_smoke'
logs.mkdir(parents=True, exist_ok=True)
(logs/'result.json').unlink(missing_ok=True)
os.environ['ROS_LOG_DIR'] = str(logs)
rclpy.init()
node = rclpy.create_node('fake_sensors')
executor = SingleThreadedExecutor()
executor.add_node(node)
processes, streams, observers, listeners, buffers = [], [], [], [], {}
maps = {'robot1': [], 'robot2': []}
origins = {'robot1': (2.0, -3.0, .4), 'robot2': (-1.5, 4.0, -.7)}
publishers = {}
result = {'test_domain': 167}


def pump(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=.02)


def sensors():
    stamp = node.get_clock().now().to_msg()
    for identity, (x, y, yaw) in origins.items():
        pubs = publishers[identity]
        odom = Odometry()
        odom.header.stamp = stamp; odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x; odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw/2); odom.pose.pose.orientation.w = math.cos(yaw/2)
        pubs['odom'].publish(odom)
        tf = TransformStamped(); tf.header = odom.header; tf.child_frame_id = 'base_link'
        tf.transform.translation.x = x; tf.transform.translation.y = y
        tf.transform.rotation = odom.pose.pose.orientation
        pubs['tf'].publish(TFMessage(transforms=[tf]))
        scan = LaserScan(); scan.header.stamp = stamp; scan.header.frame_id = 'rplidar_link'
        scan.angle_min = -math.pi; scan.angle_increment = 2*math.pi/180
        scan.angle_max = scan.angle_min + 179*scan.angle_increment
        scan.range_min = .2; scan.range_max = 12.; scan.scan_time = .1
        scan.ranges = [min(3/max(abs(math.cos(-math.pi+i*scan.angle_increment)), 1e-6),
                           2/max(abs(math.sin(-math.pi+i*scan.angle_increment)), 1e-6))
                       for i in range(180)]
        pubs['scan'].publish(scan)


try:
    for identity in origins:
        prefix = '/' + identity
        publishers[identity] = {
            'odom': node.create_publisher(Odometry, prefix+'/odom', qos_profile_sensor_data),
            'scan': node.create_publisher(LaserScan, prefix+'/scan', qos_profile_sensor_data),
            'tf': node.create_publisher(TFMessage, prefix+'/tf', 20),
        }
        static_pub = node.create_publisher(TFMessage, prefix+'/tf_static',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        publishers[identity]['static'] = static_pub
        static = TransformStamped(); static.header.frame_id = 'base_link'
        static.child_frame_id = 'rplidar_link'; static.transform.rotation.w = 1.
        static_pub.publish(TFMessage(transforms=[static]))
        observer = rclpy.create_node('observer', namespace=prefix,
            cli_args=['--ros-args', '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'])
        observers.append(observer); buffers[identity] = Buffer()
        executor.add_node(observer)
        listeners.append(TransformListener(buffers[identity], observer))
        observer.create_subscription(OccupancyGrid, prefix+'/map',
            lambda m, identity=identity: maps[identity].append(m),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    timer = node.create_timer(.1, sensors)
    pump(1)
    for identity in origins:
        stream = (logs / (identity+'.log')).open('w'); streams.append(stream)
        processes.append(subprocess.Popen([
            'ros2', 'launch', 'tb4_multi_bringup', 'mapping.launch.py', 'robot_id:='+identity,
            'origin_file:='+str(logs/(identity+'_origin.json')), 'capture_timeout:=20.0'],
            stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))
    end = time.monotonic() + 35
    while time.monotonic() < end:
        pump(.2)
        assert all(p.poll() is None for p in processes), 'A robot mapping process exited'
        if all(len(v) >= 3 for v in maps.values()):
            break
    print('Map counts:', {k: len(v) for k,v in maps.items()}, flush=True)
    print('Map endpoints:', {k: len(node.get_publishers_info_by_topic('/'+k+'/map')) for k in origins}, flush=True)
    for identity in origins:
        assert maps[identity], 'No namespaced map for '+identity
        tf = buffers[identity].lookup_transform('mapping_odom','base_link',rclpy.time.Time())
        q = tf.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        assert math.hypot(tf.transform.translation.x, tf.transform.translation.y) < .01
        assert abs(yaw) < .01, 'TF from the other robot contaminated '+identity
        origin = json.loads((logs/(identity+'_origin.json')).read_text())
        assert abs(origin['captured_x']-origins[identity][0]) < 1e-6
        result[identity] = {'maps': len(maps[identity]), 'origin': origin,
                            'mapping_base_xy': [tf.transform.translation.x,tf.transform.translation.y]}
    names = node.get_node_names_and_namespaces()
    assert ('slam_toolbox','/robot1') in names and ('slam_toolbox','/robot2') in names
    assert not node.get_publishers_info_by_topic('/map'), 'Global map leaked'
    assert not node.get_publishers_info_by_topic('/tf'), 'Global TF leaked'
    result['namespaces_isolated'] = True
    print(json.dumps(result,indent=2),flush=True)
    (logs/'result.json').write_text(json.dumps(result,indent=2))
finally:
    for p in processes:
        if p.poll() is None: os.killpg(p.pid, signal.SIGINT)
    for p in processes:
        try: p.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGTERM); p.wait(timeout=5)
    for stream in streams: stream.close()
    executor.shutdown()
    for observer in observers: observer.destroy_node()
    node.destroy_node(); rclpy.shutdown()
