#!/usr/bin/env python3
"""ROS telemetry gates in an isolated localhost domain; never connect to robots."""
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import unittest

os.environ.update(ROS_DOMAIN_ID='184', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
for key in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
    os.environ.pop(key, None)
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan, BatteryState
from nav_msgs.msg import Odometry, OccupancyGrid
from irobot_create_msgs.msg import DockStatus, WheelStatus

ROOT = Path(__file__).resolve().parents[1]


class RealPreflightTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('preflight_fake_telemetry')
        cls.settings = {'stale': False, 'battery': .8, 'wheels': True}
        cls.publishers = {}
        for topic, typ in [('scan', LaserScan), ('odom', Odometry), ('battery_state', BatteryState),
                           ('dock_status', DockStatus), ('wheel_status', WheelStatus), ('map', OccupancyGrid)]:
            qos = (QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                   if topic in ('map', 'dock_status') else qos_profile_sensor_data)
            cls.publishers[topic] = cls.node.create_publisher(typ, '/robot1/'+topic, qos)
        cls.timer = cls.node.create_timer(.05, cls.publish)
        cls.executor = SingleThreadedExecutor();cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True);cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown();cls.thread.join(timeout=3);cls.node.destroy_node();rclpy.shutdown()

    @classmethod
    def publish(cls):
        scan = LaserScan();scan.header.stamp = cls.node.get_clock().now().to_msg()
        if cls.settings['stale']:
            scan.header.stamp.sec -= 3
        scan.range_min = .1;scan.range_max = 6.;scan.ranges = [1.]*36
        odom = Odometry();odom.header.stamp = scan.header.stamp;odom.pose.pose.orientation.w = 1.
        battery = BatteryState();battery.percentage = cls.settings['battery'];battery.voltage = 15.
        dock = DockStatus();dock.is_docked = False
        wheels = WheelStatus();wheels.wheels_enabled = cls.settings['wheels']
        grid = OccupancyGrid();grid.info.width = grid.info.height = 2;grid.info.resolution = .1;grid.data = [0]*4
        for topic, msg in [('scan', scan), ('odom', odom), ('battery_state', battery), ('dock_status', dock),
                           ('wheel_status', wheels), ('map', grid)]:
            cls.publishers[topic].publish(msg)

    def setUp(self):
        self.settings.update(stale=False, battery=.8, wheels=True)

    def run_helper(self, timeout=5):
        result = subprocess.run([sys.executable, '-B', str(ROOT/'scripts/real_preflight.py'), '--robots', 'robot1',
            '--map-ready', '--require-undocked', '--timeout', str(timeout)], capture_output=True, text=True, timeout=timeout+10)
        return result, json.loads(result.stdout.strip().splitlines()[-1])

    def test_live_telemetry_map_and_enabled_wheels_pass(self):
        result, report = self.run_helper()
        self.assertEqual(result.returncode, 0, result.stderr+result.stdout)
        self.assertTrue(report['ready']);self.assertEqual(report['wheels_enabled'], {'robot1': True})

    def test_old_timestamps_rejected_despite_continuing_messages(self):
        self.settings['stale'] = True
        result, report = self.run_helper(3)
        self.assertEqual(result.returncode, 1)
        self.assertIn('robot1: stale odom', report['errors'])

    def test_disabled_wheels_and_low_battery_block_driving(self):
        self.settings.update(wheels=False, battery=.1)
        result, report = self.run_helper(3)
        self.assertEqual(result.returncode, 1)
        self.assertTrue(any('wheels disabled' in v for v in report['errors']))
        self.assertTrue(any('battery low' in v for v in report['errors']))


if __name__ == '__main__':
    unittest.main()
