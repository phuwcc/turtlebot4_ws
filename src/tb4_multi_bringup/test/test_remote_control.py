"""Isolated TCP + ROS integration; never connect to production discovery."""
import os
os.environ.update(ROS_DOMAIN_ID='174', ROS_LOCALHOST_ONLY='1',
                  ROS_SUPER_CLIENT='False', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
for key in ('ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE',
            'FASTDDS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI', 'ROS_NAMESPACE'):
    os.environ.pop(key, None)

import json
import socket
import threading
import time
import unittest

import rclpy
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tb4_multi_bringup.remote_velocity import RemoteVelocity
from tb4_multi_bringup.socket_control import SocketControl


class RemoteControlTest(unittest.TestCase):
    def wait_for(self, predicate, timeout=6):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return
            time.sleep(.02)
        self.fail('Timed out')

    def test_socket_and_rviz_share_goal_ownership(self):
        rclpy.init()
        control = SocketControl(parameter_overrides=[Parameter('robot_count', value=2),
                                                     Parameter('socket_port', value=0)])
        fake = Node('fake_nav2')
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(control); executor.add_node(fake)
        thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
        received = []
        poses = []
        finished = threading.Event()

        def execute(handle):
            received.append(handle.request.pose)
            while not finished.is_set() and not handle.is_cancel_requested:
                time.sleep(.02)
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.succeed()
            return NavigateToPose.Result()

        server = ActionServer(fake, NavigateToPose, '/robot2/navigate_to_pose', execute,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        pose_sub = fake.create_subscription(PoseWithCovarianceStamped, '/robot2/initialpose', poses.append, 10)
        port = control.server.server_address[1]

        def send(request):
            with socket.create_connection(('127.0.0.1', port), timeout=3) as connection:
                connection.sendall((json.dumps(request) + '\n').encode())
                return json.loads(connection.makefile('rb').readline())

        try:
            self.wait_for(lambda: control.action_clients['robot2'].server_is_ready())
            self.wait_for(lambda: control.pose_pubs['robot2'].get_subscription_count() > 0)
            self.assertEqual(send({'command': 'goal', 'robot': 'robot1', 'x': 1, 'y': 2})['reason'], 'nav_unavailable')
            self.assertFalse(send({'command': 'goal', 'robot': 'robot3', 'x': 1, 'y': 2})['ok'])
            self.assertFalse(send({'command': 'goal', 'robot': 'robot2', 'x': float('nan'), 'y': 2})['ok'])
            self.assertFalse(send({'command': 'goal', 'robot': 'robot2', 'x': True, 'y': 2})['ok'])
            self.assertTrue(send({'command': 'select', 'robot': 'robot2'})['ok'])
            self.assertEqual(control.get_parameter('selected_robot').value, 'robot2')
            self.assertTrue(send({'command': 'pose', 'robot': 'robot2', 'x': 1, 'y': 2, 'yaw': .5})['ok'])
            self.wait_for(lambda: len(poses) == 1)
            self.assertEqual(poses[0].header.frame_id, 'map')
            self.assertTrue(send({'command': 'goal', 'robot': 'robot2', 'x': 3, 'y': 4})['ok'])
            self.wait_for(lambda: len(received) == 1)
            self.assertEqual(received[0].pose.position.x, 3)
            self.assertFalse(send({'command': 'goal', 'robot': 'robot2', 'x': 8, 'y': 9})['ok'])
            self.assertFalse(send({'command': 'pose', 'robot': 'robot2', 'x': 8, 'y': 9})['ok'])
            self.wait_for(lambda: send({'command': 'status'})['robots']['robot2']['accepted'])
            # RViz sends to the same callback, which must reject replacement of the socket goal.
            duplicate = PoseStamped(); duplicate.header.frame_id = 'map'; duplicate.pose.orientation.w = 1.
            self.assertIsNone(control.goal_pose(duplicate))
            self.assertEqual(len(received), 1)
            self.assertTrue(send({'command': 'cancel', 'robot': 'robot2'})['ok'])
            self.wait_for(lambda: not send({'command': 'status'})['robots']['robot2']['busy'])
            self.assertEqual(send({'command': 'status'})['robots']['robot2']['last_result'], 'canceled')
            self.assertFalse(send({'command': 'cancel', 'robot': 'robot1'})['ok'])
            with socket.create_connection(('127.0.0.1', port), timeout=3) as connection:
                connection.sendall(b'[]\n')
                self.assertFalse(json.loads(connection.makefile('rb').readline())['ok'])
        finally:
            finished.set()
            executor.shutdown(); thread.join(timeout=3)
            server.destroy(); fake.destroy_node(); control.destroy_node(); rclpy.shutdown()

    def test_keyboard_publishes_stamped_commands_and_times_out(self):
        import pty
        import subprocess
        rclpy.init()
        observer = Node('keyboard_observer')
        commands = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        sub = observer.create_subscription(TwistStamped, '/robot2/remote_cmd_vel', commands.append, qos)
        master, slave = pty.openpty()
        process = subprocess.Popen(['ros2', 'run', 'tb4_multi_tools', 'teleop_keyboard',
            '--ros-args', '-r', '__ns:=/robot2', '-p', 'remote:=true', '-p', 'speed:=0.03'],
            stdin=slave, stdout=slave, stderr=slave, start_new_session=True)
        os.close(slave)

        def pump(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(observer, timeout_sec=.02)

        try:
            deadline = time.monotonic() + 6
            while not commands and time.monotonic() < deadline:
                pump(.1)
            self.assertTrue(commands)
            os.write(master, b'i'); pump(.3)
            self.assertTrue(any(m.twist.linear.x == .03 for m in commands))
            self.assertTrue(all(m.header.stamp.sec > 0 for m in commands))
            pump(.7)
            self.assertEqual(commands[-1].twist.linear.x, 0.)
            os.write(master, b'k'); pump(.1); os.write(master, b'\x03')
            process.wait(timeout=4)
            self.assertEqual(process.returncode, 0)
        finally:
            if process.poll() is None:
                process.terminate(); process.wait(timeout=4)
            os.close(master); observer.destroy_node(); rclpy.shutdown()

    def test_pi_guard_expiry_old_packets_and_multiple_sources(self):
        rclpy.init()
        guard = RemoteVelocity(namespace='/robot1')
        source = Node('pc_velocity_source', namespace='/robot1')
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(guard); executor.add_node(source)
        thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        publisher = source.create_publisher(TwistStamped, 'remote_cmd_vel', qos)
        observed = []
        sub = source.create_subscription(Twist, 'cmd_vel', lambda m: observed.append(m.linear.x), 10)
        extra = None

        def send(speed, age=0):
            message = TwistStamped()
            stamp = source.get_clock().now().nanoseconds - int(age * 1e9)
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
            message.twist.linear.x = speed
            publisher.publish(message)
            return message

        try:
            self.wait_for(lambda: publisher.get_subscription_count() > 0 and observed)
            send(.08)
            self.wait_for(lambda: .08 in observed)
            self.wait_for(lambda: len(observed) > 5 and observed[-1] == 0.)
            baseline = len(observed)
            send(.2, age=3)
            time.sleep(.15)
            self.assertTrue(all(v == 0 for v in observed[baseline:]))
            send(float('nan')); time.sleep(.1)
            self.assertEqual(observed[-1], 0.)
            latest = send(.04)
            self.wait_for(lambda: observed[-1] == .04)
            latest.header.stamp.nanosec = max(0, latest.header.stamp.nanosec - 1000)
            latest.twist.linear.x = .22
            publisher.publish(latest); time.sleep(.1)
            self.assertNotEqual(observed[-1], .22)
            extra = source.create_publisher(TwistStamped, 'remote_cmd_vel', qos)
            self.wait_for(lambda: guard.count_publishers('remote_cmd_vel') == 2)
            send(.07); time.sleep(.15)
            self.assertEqual(observed[-1], 0.)
        finally:
            executor.shutdown(); thread.join(timeout=3)
            guard.destroy_node(); source.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
