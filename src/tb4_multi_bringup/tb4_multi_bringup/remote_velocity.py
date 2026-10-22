"""PC stamps commands; Pi rejects old commands and stops on stream loss."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class RemoteVelocity(Node):
    def __init__(self, **kwargs):
        super().__init__('remote_velocity', **kwargs)
        self.mode = self.declare_parameter('mode', 'guard').value
        self.timeout = self.declare_parameter('timeout', 0.5).value
        if self.mode not in ('guard', 'stamp') or not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('mode must be guard/stamp; timeout must be finite and positive')
        self.last = None
        self.last_stamp = -math.inf
        self.command = Twist()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        if self.mode == 'stamp':
            self.pub = self.create_publisher(TwistStamped, 'remote_cmd_vel', qos)
            self.sub = self.create_subscription(Twist, 'pc_cmd_vel', self.stamp, qos)
        else:
            self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
            self.sub = self.create_subscription(TwistStamped, 'remote_cmd_vel', self.receive, qos)
            self.timer = self.create_timer(0.05, self.tick)

    def stamp(self, message):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.twist = message
        self.pub.publish(command)

    def receive(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        values = [getattr(v, axis) for v in (message.twist.linear, message.twist.angular)
                  for axis in ('x', 'y', 'z')]
        if (not all(math.isfinite(v) for v in values) or not -0.1 <= age <= self.timeout
                or stamp <= self.last_stamp):
            return
        # Preserve the original command's expiry instead of refreshing its lifetime on arrival.
        self.last_stamp = stamp
        self.last = time.monotonic() - max(0.0, age)
        self.command = message.twist

    def tick(self):
        active = (self.last is not None and time.monotonic() - self.last < self.timeout
                  and self.count_publishers('remote_cmd_vel') == 1)
        self.pub.publish(self.command if active else Twist())

    def destroy_node(self):
        if self.mode == 'guard':
            self.pub.publish(Twist())
        return super().destroy_node()


def main():
    rclpy.init()
    node = RemoteVelocity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
