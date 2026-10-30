#!/usr/bin/env python3
"""Read-only PC diagnostics and startup checks; never publish motion commands."""
import argparse
import json
import os
from pathlib import Path
import time

import yaml


def validate_map(filename):
    path = Path(filename).expanduser().resolve()
    if not path.is_file():
        raise ValueError('Không thấy map YAML: ' + str(path))
    with path.open() as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or not isinstance(config.get('image'), str) or not config['image']:
        raise ValueError('Map YAML thiếu image: ' + str(path))
    image = Path(config['image']).expanduser()
    if not image.is_absolute():
        image = path.parent / image
    if not image.is_file():
        raise ValueError('Không thấy ảnh map: ' + str(image))
    return {'yaml': str(path), 'image': str(image.resolve())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preflight', action='store_true')
    parser.add_argument('--map')
    parser.add_argument('--config', type=Path, default=Path.home() / '.config/turtlebot4/pc_env.sh')
    parser.add_argument('--timeout', type=float, default=30.)
    args = parser.parse_args()
    if not 0 < args.timeout <= 300:
        parser.error('--timeout must be between 0 and 300 seconds')
    from pc_config import load_environment
    try:
        environment = load_environment(args.config)
        robots = environment.get('TB4_ROBOT_IDS', ','.join('robot' + str(i) for i in range(1, int(environment['TB4_ROBOT_COUNT']) + 1))).split(',')
        map_info = validate_map(args.map) if args.map else None
    except (OSError, ValueError, yaml.YAMLError) as error:
        print('Cấu hình không hợp lệ: ' + str(error))
        return 1

    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry, OccupancyGrid
    from lifecycle_msgs.srv import GetState
    from rcl_interfaces.srv import GetParameters
    rclpy.init(args=[])
    node = rclpy.create_node('pc_navigation_diagnose')
    counts = {}
    ages = {}
    subscriptions = []
    clients = []
    futures = {}
    service_pending = []

    def received(topic, message):
        counts[topic] = counts.get(topic, 0) + 1
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        ages[topic] = round(node.get_clock().now().nanoseconds * 1e-9 - stamp, 3)

    for robot in ('robot1', 'robot2', 'robot3'):
        for suffix, kind in [('scan', LaserScan), ('odom', Odometry)]:
            topic = '/' + robot + '/' + suffix
            subscriptions.append(node.create_subscription(kind, topic,
                lambda m, t=topic: received(t, m), qos_profile_sensor_data))
        if robot not in robots or args.preflight:
            continue
        topic = '/' + robot + '/map'
        subscriptions.append(node.create_subscription(OccupancyGrid, topic,
            lambda m, t=topic: received(t, m),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        for name in ('map_server', 'amcl', 'controller_server', 'planner_server', 'bt_navigator'):
            client = node.create_client(GetState, '/' + robot + '/' + name + '/get_state')
            clients.append(client)
            service_pending.append((robot + '/' + name, client, GetState.Request()))
        client = node.create_client(GetParameters, '/' + robot + '/map_server/get_parameters')
        clients.append(client)
        service_pending.append((robot + '/map_file', client, GetParameters.Request(names=['yaml_filename'])))
    print('ROS_DISTRO=' + os.environ.get('ROS_DISTRO', '?') + ', RMW=' + os.environ.get('RMW_IMPLEMENTATION', '?'), flush=True)
    print('Discovery: ' + environment['ROS_DISCOVERY_SERVER'], flush=True)
    for key in ('ROS_DOMAIN_ID', 'ROS_SUPER_CLIENT', 'ROS_LOCALHOST_ONLY',
                'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                'RMW_FASTRTPS_USE_QOS_FROM_XML', 'ROS_NAMESPACE'):
        print(key + '=' + os.environ.get(key, '(unset)'), flush=True)
    print('Robot được cấu hình: ' + ', '.join(robots), flush=True)
    if map_info:
        print('Map: ' + json.dumps(map_info), flush=True)
    print('Đang kiểm tra dữ liệu ROS trong ' + str(args.timeout) + ' giây...', flush=True)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            for key, client, request in service_pending:
                if key not in futures and client.service_is_ready():
                    futures[key] = client.call_async(request)
        print('Dữ liệu nhận được: ' + json.dumps(counts, sort_keys=True))
        print('Độ trễ timestamp scan/odom (giây): ' + json.dumps(ages, sort_keys=True))
        missing = [f'/{r}/{t}' for r in robots for t in ('scan', 'odom') if not counts.get(f'/{r}/{t}')]
        failures = []
        for robot in robots:
            for suffix in ('scan', 'odom'):
                topic = '/' + robot + '/' + suffix
                if topic in ages and not -.1 <= ages[topic] <= 1.:
                    print('TIMESTAMP KHÔNG HỢP LỆ: ' + topic + ': ' + str(ages[topic]) + ' giây')
                    failures.append(topic)
        for key, client, request in service_pending:
            future = futures.get(key)
            if future is None or not future.done():
                print(key + ': TIMEOUT / service không có phản hồi')
                failures.append(key)
            elif future.exception() is not None:
                print(key + ': ERROR / ' + str(future.exception()))
                failures.append(key)
            else:
                result = future.result()
                print(key + ': ' + (result.current_state.label if hasattr(result, 'current_state') else str(result)))
                if hasattr(result, 'current_state') and result.current_state.label != 'active':
                    failures.append(key)
        if not args.preflight:
            for robot in robots:
                topic = '/' + robot + '/map'
                if not counts.get(topic):
                    print('THIẾU MAP: ' + topic)
                    failures.append(topic)
        if failures:
            print('NAVIGATION CHƯA SẴN SÀNG: ' + ', '.join(failures))
            print('Nếu map_server/AMCL chưa active, xử lý lifecycle/service trước khi đặt 2D Pose Estimate.')
        if missing:
            print('THIẾU DỮ LIỆU: ' + ', '.join(missing))
            detected = [r for r in ('robot1', 'robot2', 'robot3') if counts.get('/' + r + '/scan')]
            print('Hardware đang thấy: ' + (', '.join(detected) or 'không có'))
            print('Kiểm tra robot đã chọn, IP/discovery server ID và robot-start.sh trên đúng Pi.')
            return 1
        print('Scan/odom có dữ liệu cho các robot đã chọn.')
        return 1 if failures else 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
