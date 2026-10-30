#!/usr/bin/env python3
"""Live PC->Pi acceptance check. Motion requires explicit --allow-motion.

Run after sourcing the PC ROS environment, with Nav2 and teleop stopped.
Tests robots sequentially at 0.04 m/s for at most two seconds each.
Never undocks, changes E-Stop, or sends Create 3 actions directly.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from irobot_create_msgs.msg import DockStatus
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from sensor_msgs.msg import LaserScan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robots', nargs='+', choices=['robot1', 'robot2', 'robot3'], required=True)
    parser.add_argument('--allow-motion', action='store_true')
    parser.add_argument('--timeout', type=float, default=45.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = rclpy.create_node('tb4_hardware_acceptance')
    data = {r: {} for r in args.robots}
    counts = {r: {} for r in args.robots}
    subscriptions, publishers = [], {}
    buffers = {r: Buffer() for r in args.robots}
    report = {'motion_requested': args.allow_motion, 'robots': {}}

    def received(robot, topic, message):
        data[robot][topic] = (time.monotonic(), message)
        counts[robot][topic] = counts[robot].get(topic, 0) + 1

    def transforms(robot, message, static=False):
        for transform in message.transforms:
            if static:
                buffers[robot].set_transform_static(transform, 'hardware')
            else:
                buffers[robot].set_transform(transform, 'hardware')

    for robot in args.robots:
        subscriptions.append(node.create_subscription(TFMessage, '/'+robot+'/tf',
            lambda m,r=robot: transforms(r,m), qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(TFMessage, '/'+robot+'/tf_static',
            lambda m,r=robot: transforms(r,m,True),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        for topic, kind in [('scan', LaserScan), ('odom', Odometry),
                            ('cmd_vel', Twist), ('dock_status', DockStatus)]:
            subscriptions.append(node.create_subscription(kind, '/' + robot + '/' + topic,
                lambda m, r=robot, t=topic: received(r, t, m), qos_profile_sensor_data))
        if args.allow_motion:
            publishers[robot] = node.create_publisher(TwistStamped,
                '/' + robot + '/remote_cmd_vel', qos_profile_sensor_data)

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.02, max(0., end-time.monotonic())))

    def command(robot, speed=0.):
        message = TwistStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.twist.linear.x = speed
        publishers[robot].publish(message)

    def fresh(robot, topic, limit=.5):
        entry = data[robot].get(topic)
        return entry is not None and time.monotonic()-entry[0] < limit

    def position(robot):
        p = data[robot]['odom'][1].pose.pose.position
        return p.x, p.y

    def distance(a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            pump(.1)
            if all(all(fresh(r, t, 1.) for t in ('scan', 'odom', 'cmd_vel', 'dock_status'))
                   for r in args.robots):
                break
        for robot in args.robots:
            result = report['robots'][robot] = {'counts': dict(counts[robot])}
            try:
                for topic in ('scan', 'odom', 'cmd_vel', 'dock_status'):
                    assert fresh(robot, topic, 1.), 'Missing/stale ' + topic
                result['docked'] = data[robot]['dock_status'][1].is_docked
                odom = data[robot]['odom'][1]
                stamp = odom.header.stamp.sec + odom.header.stamp.nanosec/1e9
                age = node.get_clock().now().nanoseconds/1e9 - stamp
                result['odom_age_seconds'] = round(age, 3)
                assert -.1 <= age <= .5, 'Odometry timestamp outside command clock tolerance'
                assert all(math.isfinite(v) for v in position(robot)), 'Nonfinite odometry'
                assert buffers[robot].can_transform('odom', 'base_link', rclpy.time.Time()), 'Missing odom -> base_link TF'
                if args.allow_motion:
                    assert not result['docked'], 'Robot is docked; undock manually before testing'
                    deadline = time.monotonic()+15.
                    while time.monotonic() < deadline:
                        endpoints = node.get_publishers_info_by_topic('/'+robot+'/remote_cmd_vel')
                        if len(endpoints)==1 and endpoints[0].node_name == node.get_name():
                            break
                        pump(.1)
                    result['command_publishers'] = [{'node':e.node_namespace+'/'+e.node_name,
                        'gid':bytes(e.endpoint_gid).hex()} for e in endpoints]
                    assert len(endpoints)==1 and endpoints[0].node_name==node.get_name(), 'Expected only this test publisher; see command_publishers'
                    assert publishers[robot].get_subscription_count() > 0, 'No Pi guard subscription'
                    # Discovery of the reverse graph can lag the publisher's match.
                    # Send only stop commands while both sides discover the source.
                    warmup_end = time.monotonic()+10.
                    while time.monotonic() < warmup_end:
                        command(robot)
                        pump(.05)
                    assert fresh(robot, 'odom') and fresh(robot, 'scan'), 'Sensors lost during discovery warmup'
                    assert not data[robot]['dock_status'][1].is_docked, 'Robot docked during warmup'
                    cmd = data[robot]['cmd_vel'][1]
                    assert abs(cmd.linear.x) < .001 and abs(cmd.angular.z) < .001, 'Robot not initially stopped'
                    start = position(robot)
                    others = {r: position(r) for r in args.robots if r != robot and fresh(r, 'odom')}
                    deadline = time.monotonic()+2.
                    sent, observed_nonzero = 0, False
                    while time.monotonic() < deadline:
                        assert fresh(robot, 'scan') and fresh(robot, 'odom'), 'Sensor stream lost during movement'
                        scan = data[robot]['scan'][1]
                        assert buffers[robot].can_transform('base_link', scan.header.frame_id, rclpy.time.Time()), 'Missing lidar TF'
                        transform = buffers[robot].lookup_transform('base_link', scan.header.frame_id, rclpy.time.Time()).transform
                        q = transform.rotation
                        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                        points = []
                        for i, radius in enumerate(scan.ranges):
                            if not math.isfinite(radius) or not scan.range_min <= radius <= scan.range_max:
                                continue
                            angle = scan.angle_min+i*scan.angle_increment+yaw
                            x = radius*math.cos(angle)+transform.translation.x
                            y = radius*math.sin(angle)+transform.translation.y
                            if x > 0. and abs(y) < .28:
                                points.append(x)
                        result['front_clearance_m'] = round(min(points),3) if points else None
                        assert not points or min(points) > .45, 'Obstacle within 0.45m forward corridor; see front_clearance_m'
                        if distance(start, position(robot)) >= .10:
                            break
                        command(robot, .04)
                        sent += 1
                        pump(.05)
                        cmd = data[robot]['cmd_vel'][1]
                        observed_nonzero |= cmd.linear.x > .01
                    # Stop publishing and measure the guard's stream-loss timeout.
                    last_publish = time.monotonic()
                    guard_stopped_at = None
                    deadline = last_publish+1.2
                    while time.monotonic() < deadline:
                        pump(.02)
                        if fresh(robot, 'cmd_vel', .15) and data[robot]['cmd_vel'][0] > last_publish:
                            cmd = data[robot]['cmd_vel'][1]
                            if abs(cmd.linear.x)<.001 and abs(cmd.angular.z)<.001:
                                guard_stopped_at = time.monotonic()-last_publish
                                break
                    for _ in range(5):
                        command(robot)
                        pump(.05)
                    pump(.7)
                    result.update(commands_sent=sent, guard_forwarded=observed_nonzero,
                        distance_m=round(distance(start, position(robot)),4),
                        watchdog_stop_seconds=round(guard_stopped_at,3) if guard_stopped_at is not None else None,
                        other_robot_displacement_m={r:round(distance(p,position(r)),4) for r,p in others.items()})
                    assert observed_nonzero, 'Pi guard did not forward a movement command'
                    assert guard_stopped_at is not None and guard_stopped_at <= .85, 'Watchdog stop missing or late'
                    assert .01 <= result['distance_m'] <= .15, 'Measured movement outside 1-15cm'
                    assert all(v < .02 for v in result['other_robot_displacement_m'].values()), 'Other robot moved'
                    assert fresh(robot, 'odom'), 'No fresh odometry after stop'
                    twist = data[robot]['odom'][1].twist.twist
                    assert abs(twist.linear.x)<.02 and abs(twist.angular.z)<.05, 'Robot did not stop in odometry'
                result['passed'] = True
            except (AssertionError, KeyError) as error:
                result.update(passed=False, error=str(error))
            finally:
                if args.allow_motion:
                    for _ in range(5):
                        command(robot)
                        pump(.05)
        report['passed'] = all(r['passed'] for r in report['robots'].values())
    finally:
        if args.allow_motion:
            for robot in publishers:
                command(robot)
            pump(.1)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report.get('passed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
