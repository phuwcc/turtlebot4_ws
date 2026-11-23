"""Drive one simulated TurtleBot4 and save its SLAM Toolbox occupancy map."""
import argparse
import os
from pathlib import Path
import select
import subprocess
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Point, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


class MappingTeleop(Node):
    def __init__(self):
        super().__init__('sim_mapping_teleop')
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/robot1/odom', self.publish_odom, qos_profile_sensor_data)
        self.scan = self.create_publisher(LaserScan, '/robot1/scan', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/sim/raw_scan', self.publish_scan, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_display = self.create_publisher(Marker, '/sim/slam_grid', latched)
        self.create_subscription(OccupancyGrid, '/map', self.draw_map, latched)
        self.cmd = self.create_publisher(Twist, '/robot1/diffdrive_controller/cmd_vel_unstamped', 10)

    def publish_odom(self, msg):
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = 'robot1/odom'
        tf.child_frame_id = 'robot1/base_link'
        p = msg.pose.pose
        tf.transform.translation.x = p.position.x
        tf.transform.translation.y = p.position.y
        tf.transform.rotation = p.orientation
        self.tf.sendTransform(tf)

    def publish_scan(self, msg):
        msg.header.frame_id = 'robot1/rplidar_link'
        self.scan.publish(msg)

    def draw_map(self, msg):
        marker = Marker()
        marker.header = msg.header
        marker.header.stamp.sec = marker.header.stamp.nanosec = 0
        marker.ns = 'SLAM occupancy'
        marker.type = Marker.CUBE_LIST
        marker.pose = msg.info.origin
        resolution = msg.info.resolution
        marker.scale.x = marker.scale.y = resolution
        marker.scale.z = .012
        marker.color.a = 1.
        width = msg.info.width
        free = ColorRGBA(r=.81, g=.84, b=.84, a=1.)
        occupied = ColorRGBA(r=.20, g=.26, b=.27, a=1.)
        for index, value in enumerate(msg.data):
            if value < 0:
                continue
            marker.points.append(Point(x=(index % width + .5)*resolution,
                                       y=(index // width + .5)*resolution, z=-.01))
            marker.colors.append(occupied if value >= 50 else free)
        self.map_display.publish(marker)

    def save_map(self, path):
        self.cmd.publish(Twist())
        try:
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(path),
                 '--fmt', 'pgm', '--ros-args', '-p', 'save_map_timeout:=5.0'],
                timeout=10.)
        except subprocess.TimeoutExpired:
            print('Hết thời gian chờ lưu map.', flush=True)
            return
        if result.returncode == 0:
            print(f'Đã lưu {path}.yaml và {path}.pgm', flush=True)
        else:
            print('Chưa lưu được map. Kiểm tra RViz đã nhận bản đồ SLAM.', flush=True)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-out', type=Path, default=Path(os.environ['TB4_SIM_OUTPUT']) / 'scan')
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error('Teleop cần chạy trong terminal tương tác')
    path = args.map_out.expanduser().resolve()
    if path.suffix in ('.yaml', '.pgm'):
        path = path.with_suffix('')
    path.parent.mkdir(parents=True, exist_ok=True)
    directions = {'w': (.10, 0.), 's': (-.08, 0.), 'a': (0., .40), 'd': (0., -.40)}
    rclpy.init(args=[])
    node = MappingTeleop()
    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    print('W/S tiến/lùi, A/D quay, Space dừng, M lưu map, Q thoát. Giữ phím để tiếp tục.', flush=True)
    print(f'Bản đồ sẽ lưu tại {path}.yaml', flush=True)
    command, expires, last_publish = Twist(), 0., 0.
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            if select.select([fd], [], [], 0)[0]:
                key = os.read(fd, 1).decode(errors='ignore').lower()
                if key == 'q':
                    break
                if key in directions:
                    command.linear.x, command.angular.z = directions[key]
                    expires = time.monotonic() + .35
                elif key in (' ', 'm'):
                    command, expires = Twist(), 0.
                    node.cmd.publish(command)
                    if key == 'm':
                        node.save_map(path)
            now = time.monotonic()
            if now - last_publish >= .05:
                node.cmd.publish(command if now < expires else Twist())
                last_publish = now
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)
        if rclpy.ok():
            node.cmd.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
