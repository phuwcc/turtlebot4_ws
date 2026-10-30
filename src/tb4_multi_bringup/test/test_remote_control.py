"""Native keyboard timeout test, isolated from physical robots."""
import os
os.environ.update(ROS_DOMAIN_ID='174', ROS_LOCALHOST_ONLY='1',
                  ROS_SUPER_CLIENT='False', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
for key in ('ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE',
            'FASTDDS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI', 'ROS_NAMESPACE'):
    os.environ.pop(key, None)
import time
import unittest
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class KeyboardTest(unittest.TestCase):
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


if __name__ == '__main__':
    unittest.main()
