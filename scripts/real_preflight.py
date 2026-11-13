#!/usr/bin/env python3
"""Bounded, read-only checks of real robot telemetry; no velocity publishers."""
import argparse
import json
import math
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robots', nargs='+', choices=('robot1', 'robot2', 'robot3'), required=True)
    parser.add_argument('--timeout', type=float, default=45.)
    parser.add_argument('--map-ready', action='store_true')
    parser.add_argument('--require-undocked', action='store_true')
    args = parser.parse_args()
    if not 0 < args.timeout <= 300:
        parser.error('timeout must be 0..300 seconds')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import LaserScan, BatteryState
    from nav_msgs.msg import Odometry, OccupancyGrid
    from irobot_create_msgs.msg import DockStatus, WheelStatus
    rclpy.init(args=[])
    node = rclpy.create_node('real_robot_preflight')
    seen, subscriptions = {}, []
    def receive(robot, kind, message):
        seen[robot, kind] = (time.monotonic(), message)
    for robot in args.robots:
        topics = [('scan', LaserScan), ('odom', Odometry), ('battery_state', BatteryState),
                  ('dock_status', DockStatus), ('wheel_status', WheelStatus)]
        if args.map_ready:
            topics.append(('map', OccupancyGrid))
        for kind, typ in topics:
            qos = (QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                   if kind in ('map', 'dock_status') else qos_profile_sensor_data)
            subscriptions.append(node.create_subscription(typ, '/' + robot + '/' + kind,
                lambda m, r=robot, k=kind: receive(r, k, m), qos))
    def problems():
        errors = []
        for robot in args.robots:
            for kind in ('scan', 'odom', 'battery_state', 'dock_status', 'wheel_status') + (('map',) if args.map_ready else ()):
                sample = seen.get((robot, kind))
                if sample is None:
                    errors.append(robot + ': missing ' + kind)
                    continue
                received, m = sample
                if kind in ('scan', 'odom'):
                    stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                    age = node.get_clock().now().nanoseconds * 1e-9 - stamp
                    if time.monotonic() - received > .7 or not -.1 <= age <= .7:
                        errors.append(robot + ': stale ' + kind)
                if kind == 'scan' and not any(math.isfinite(v) and m.range_min < v < m.range_max for v in m.ranges):
                    errors.append(robot + ': no valid LiDAR ranges')
                if kind == 'odom' and not all(math.isfinite(v) for v in (
                        m.pose.pose.position.x, m.pose.pose.position.y,
                        m.twist.twist.linear.x, m.twist.twist.angular.z)):
                    errors.append(robot + ': invalid odom')
                if kind == 'battery_state':
                    if time.monotonic() - received > 10 or not math.isfinite(m.percentage) or not .12 < m.percentage <= 1.:
                        errors.append(robot + ': battery low/unknown/stale; charge before running')
                if kind == 'dock_status' and args.require_undocked and m.is_docked:
                    errors.append(robot + ': still docked; undock before driving')
                if kind == 'wheel_status':
                    if time.monotonic() - received > 3:
                        errors.append(robot + ': stale wheel status')
                    elif args.require_undocked and not m.wheels_enabled:
                        errors.append(robot + ': wheels disabled/E-stop; release E-stop on the robot before driving')
                if kind == 'map' and (not m.info.width or not m.info.height or not m.data):
                    errors.append(robot + ': empty map')
        return errors
    try:
        deadline = time.monotonic() + args.timeout
        stable = None
        errors = []
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            errors = problems()
            stable = None if errors else (stable or time.monotonic())
            if stable is not None and time.monotonic() - stable >= 2:
                print(json.dumps({'ready': True, 'robots': args.robots,
                    'battery_percent': {r: round(seen[r, 'battery_state'][1].percentage * 100, 1) for r in args.robots},
                    'wheels_enabled': {r: seen[r, 'wheel_status'][1].wheels_enabled for r in args.robots}}), flush=True)
                return 0
        print(json.dumps({'ready': False, 'errors': errors}), flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
