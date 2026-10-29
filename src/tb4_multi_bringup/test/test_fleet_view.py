"""Run on an isolated ROS domain: synthetic TF, latched maps and fake Nav2 only."""
import os
from pathlib import Path as FilePath
import threading
import tempfile
import time
import unittest

# Enforce isolation even when invoked directly from a robot-configured shell.
os.environ.update(ROS_DOMAIN_ID='173', ROS_LOCALHOST_ONLY='1',
                  ROS_SUPER_CLIENT='False', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
for key in ('ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE',
            'FASTDDS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI', 'ROS_NAMESPACE'):
    os.environ.pop(key, None)
os.environ.setdefault('ROS_LOG_DIR', '/tmp/tb4_fleet_test_logs')

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

from tb4_multi_bringup.fleet_view import FleetView, ROBOTS, adapt_message, display_frame
from tb4_multi_bringup.fleet_settings import active_robots, rviz_for_count, startup_poses, validate_discovery
import yaml


class FleetViewTest(unittest.TestCase):
    def test_startup_poses(self):
        self.assertEqual(startup_poses('', 2), {})
        with tempfile.TemporaryDirectory() as directory:
            path = FilePath(directory) / 'poses.yaml'
            path.write_text('robot1: {x: 1, y: 2, yaw: 0.5}\n'
                            'robot2: {x: -3, y: 4, yaw: -1}\n')
            poses = startup_poses(str(path), 2)
            self.assertEqual(poses['robot1']['initial_x'], '1')
            self.assertEqual(poses['robot2']['initial_x'], '-3')
            for text in ('[]', 'robot1: {x: 1, y: 2}',
                         'robot1: {x: .nan, y: 2, yaw: 0}',
                         'robot1: {x: true, y: 2, yaw: 0}',
                         'robot1: {x: 1, y: 2, yaw: 0}',
                         'robot4: {x: 1, y: 2, yaw: 0}'):
                path.write_text(text)
                with self.subTest(text=text), self.assertRaises(ValueError):
                    startup_poses(str(path), 2)

    def wait_for(self, predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.02)
        self.fail('Timed out waiting for ROS messages/action state')

    def test_configuration_filters_and_rejects_broken_discovery(self):
        template = yaml.safe_load((FilePath(__file__).resolve().parents[1] /
                                   'config/fleet.rviz').read_text())
        for count in (1, 2, 3):
            view = rviz_for_count(template, count)
            groups = [d['Name'] for d in view['Visualization Manager']['Displays'] if d['Name'] in ROBOTS]
            self.assertEqual(groups, list(ROBOTS[:count]))
            valid = ';'.join(f'10.11.63.{119 + index}:11811' for index in range(count)) + ';'
            validate_discovery(valid, count)
        self.assertEqual(len([d for d in template['Visualization Manager']['Displays']
                              if d['Name'] in ROBOTS]), 3)
        for value, count in ((':11811;:11811;:11811;', 3), ('', 1),
                             ('10.11.63.119:11811;:11811;', 2),
                             ('999.0.0.1:11811;', 1), ('10.11.63.119:11811;', 2)):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_discovery(value, count)
        for count in (0, 4, True):
            with self.assertRaises(ValueError):
                active_robots(count)

    def test_only_active_robot_interfaces(self):
        for count in (1, 2, 3):
            with self.subTest(count=count):
                rclpy.init()
                bridge = FleetView(parameter_overrides=[Parameter('robot_count', value=count)])
                try:
                    self.assertEqual(tuple(bridge.action_clients), ROBOTS[:count])
                    self.assertEqual(tuple(bridge.pose_pubs), ROBOTS[:count])
                    topics = {subscription.topic_name for subscription in bridge.subscriptions}
                    for index, robot in enumerate(ROBOTS):
                        self.assertEqual('/' + robot + '/scan' in topics, index < count)
                    if count < 3:
                        rejected = bridge.set_parameters([
                            Parameter('selected_robot', value=ROBOTS[count])])[0]
                        self.assertFalse(rejected.successful)
                    immutable = bridge.set_parameters([Parameter('robot_count', value=1)])[0]
                    self.assertFalse(immutable.successful)
                finally:
                    bridge.destroy_node()
                    rclpy.shutdown()

    def test_three_robots_and_controls(self):
        self.assertEqual(display_frame('robot3', '/base_link'), 'robot3/base_link')
        self.assertEqual(display_frame('robot3', 'robot3/odom'), 'robot3/odom')
        self.assertEqual(display_frame('robot3', 'world', 'world'), 'map')
        path = Path()
        path.header.frame_id = 'map'
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        path.poses.append(pose)
        self.assertEqual(adapt_message(path, 'robot3').poses[0].header.frame_id, 'robot3/odom')
        self.assertEqual(path.poses[0].header.frame_id, 'odom')

        rclpy.init()
        bridge = FleetView()
        fake = Node('fake_fleet')
        observer = Node('fleet_observer')
        executor = MultiThreadedExecutor(num_threads=8)
        for node in (bridge, fake, observer):
            executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        servers, subs = [], []
        action_group = ReentrantCallbackGroup()
        done = {r: threading.Event() for r in ROBOTS}
        requests = {r: [] for r in ROBOTS}
        received = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        try:
            subs.append(observer.create_subscription(MarkerArray, '/tb4_view/labels',
                lambda m: received.__setitem__('labels', m), latched))
            self.wait_for(lambda: 'labels' in received)
            self.assertEqual(received['labels'].markers[-1].text, 'Pose / Goal -> robot1')

            def execute(handle, robot):
                requests[robot].append(handle.request.pose)
                deadline = time.monotonic() + 15
                while not done[robot].is_set() and time.monotonic() < deadline:
                    if handle.is_cancel_requested:
                        handle.canceled()
                        return NavigateToPose.Result()
                    time.sleep(0.02)
                handle.succeed()
                return NavigateToPose.Result()

            for robot in ROBOTS:
                servers.append(ActionServer(fake, NavigateToPose, '/' + robot + '/navigate_to_pose',
                    execute_callback=lambda h, r=robot: execute(h, r),
                    cancel_callback=lambda h: CancelResponse.ACCEPT, callback_group=action_group))
                subs.append(observer.create_subscription(PoseWithCovarianceStamped,
                    '/' + robot + '/initialpose',
                    lambda m, r=robot: received.__setitem__(r + '_pose', m), 10))
            self.wait_for(lambda: all(c.server_is_ready() for c in bridge.action_clients.values()))
            rejected = bridge.set_parameters([Parameter('selected_robot', value='robot4')])[0]
            self.assertFalse(rejected.successful)
            self.assertEqual(bridge.get_parameter('selected_robot').value, 'robot1')

            tf_pub = fake.create_publisher(TFMessage, '/robot3/tf', 10)
            scan_pub = fake.create_publisher(LaserScan, '/robot3/scan', 10)
            amcl_pub = fake.create_publisher(PoseWithCovarianceStamped, '/robot3/amcl_pose', 10)
            subs.append(observer.create_subscription(PoseWithCovarianceStamped, '/tb4_view/robot3/amcl_pose',
                lambda m: received.__setitem__('amcl', m), bridge.live))
            subs.append(observer.create_subscription(TFMessage, '/tb4_view/tf',
                lambda m: received.__setitem__('tf', m), 10))
            subs.append(observer.create_subscription(LaserScan, '/tb4_view/robot3/scan',
                lambda m: received.__setitem__('scan', m), bridge.live))
            self.wait_for(lambda: tf_pub.get_subscription_count() > 0 and scan_pub.get_subscription_count() > 0)
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'odom'
            transform.transform.rotation.w = 1.0
            tf_pub.publish(TFMessage(transforms=[transform]))
            scan = LaserScan()
            scan.header.frame_id = 'rplidar_link'
            scan_pub.publish(scan)
            self.wait_for(lambda: 'tf' in received and 'scan' in received)
            self.assertEqual(received['tf'].transforms[0].header.frame_id, 'map')
            self.assertEqual(received['tf'].transforms[0].child_frame_id, 'robot3/odom')
            self.assertEqual(received['scan'].header.frame_id, 'robot3/rplidar_link')
            self.wait_for(lambda: amcl_pub.get_subscription_count() > 0)
            amcl = PoseWithCovarianceStamped()
            amcl.header.frame_id = 'map'
            amcl.pose.pose.orientation.w = 1.0
            amcl_pub.publish(amcl)
            self.wait_for(lambda: 'amcl' in received)
            self.assertEqual(received['amcl'].header.frame_id, 'map')

            # Publish separate static batches; a late RViz must receive the complete tree.
            static_pubs = []
            for robot in ROBOTS:
                pub = fake.create_publisher(TFMessage, '/' + robot + '/tf_static', latched)
                static_pubs.append(pub)
                self.wait_for(lambda: pub.get_subscription_count() > 0)
                for child in ('base_link', 'rplidar_link'):
                    transform = TransformStamped()
                    transform.header.frame_id = 'odom'
                    transform.child_frame_id = child
                    transform.transform.rotation.w = 1.0
                    pub.publish(TFMessage(transforms=[transform]))
                    self.wait_for(lambda: robot + '/' + child in bridge.static_cache)
            subs.append(observer.create_subscription(TFMessage, '/tb4_view/tf_static',
                lambda m: received.__setitem__('static', m), latched))
            self.wait_for(lambda: 'static' in received)
            self.assertEqual(len(received['static'].transforms), 6)

            map_pub = fake.create_publisher(OccupancyGrid, '/robot3/map', latched)
            self.wait_for(lambda: map_pub.get_subscription_count() > 0)
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            grid.info.width = grid.info.height = 1
            grid.info.resolution = 0.1
            grid.data = [0]
            map_pub.publish(grid)
            subs.append(observer.create_subscription(OccupancyGrid, '/tb4_view/robot3/map',
                lambda m: received.__setitem__('map', m), latched))
            self.wait_for(lambda: 'map' in received)
            self.assertEqual(received['map'].header.frame_id, 'map')

            pose_pub = observer.create_publisher(PoseWithCovarianceStamped, '/tb4_view/initialpose', 10)
            goal_pub = observer.create_publisher(PoseStamped, '/tb4_view/goal_pose', 10)
            self.wait_for(lambda: pose_pub.get_subscription_count() > 0 and goal_pub.get_subscription_count() > 0)
            for index, robot in enumerate(ROBOTS):
                self.assertTrue(bridge.set_parameters([Parameter('selected_robot', value=robot)])[0].successful)
                self.wait_for(lambda: received['labels'].markers[-1].text == 'Pose / Goal -> ' + robot)
                initial = PoseWithCovarianceStamped()
                initial.header.frame_id = 'map'
                initial.pose.pose.orientation.w = 1.0
                initial.pose.pose.position.x = float(index + 1)
                pose_pub.publish(initial)
                self.wait_for(lambda: robot + '_pose' in received)
                self.assertEqual(received[robot + '_pose'].pose.pose.position.x, float(index + 1))
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.pose.orientation.w = 1.0
                goal.pose.position.x = float(index + 10)
                goal_pub.publish(goal)
                self.wait_for(lambda: len(requests[robot]) == 1)
                self.assertEqual(requests[robot][0].header.frame_id, 'map')
                self.assertEqual(requests[robot][0].pose.position.x, float(index + 10))
            self.assertTrue(all(bridge.busy.values()))
            # Same robot cannot get a second concurrent goal; invalid frame cannot route.
            goal_pub.publish(goal)
            goal.header.frame_id = 'odom'
            goal_pub.publish(goal)
            time.sleep(0.2)
            self.assertEqual([len(requests[r]) for r in ROBOTS], [1, 1, 1])
            cancel = observer.create_client(Trigger, '/tb4_view/cancel')
            self.assertTrue(cancel.wait_for_service(timeout_sec=3))
            future = cancel.call_async(Trigger.Request())
            self.wait_for(future.done)
            self.assertTrue(future.result().success)
            self.wait_for(lambda: not bridge.busy['robot3'])
            self.assertTrue(bridge.busy['robot1'] and bridge.busy['robot2'])
            done['robot1'].set()
            done['robot2'].set()
            self.wait_for(lambda: not any(bridge.busy.values()))
        finally:
            for event in done.values():
                event.set()
            executor.shutdown(timeout_sec=3)
            thread.join(timeout=3)
            for server in servers:
                server.destroy()
            for node in (observer, fake, bridge):
                node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
