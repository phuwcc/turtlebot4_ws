"""Drive the Gazebo robots along MARCO batches using simulated wheel odometry."""
import argparse
import math
from pathlib import Path
import sys
import time

import rclpy
from geometry_msgs.msg import Point, TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray

from t_junction import junction, swap_mission

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tb4_marco_bridge'))
from simulation_planner import SimulationPlanner


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def straight_segments(points):
    """Keep corners and final pose, skipping intermediate collinear samples."""
    result = [points[0]]
    for point, following in zip(points[1:-1], points[2:]):
        prev = result[-1]
        ax, ay = point[0] - prev[0], point[1] - prev[1]
        bx, by = following[0] - point[0], following[1] - point[1]
        if abs(ax*by - ay*bx) > 1e-8 or ax*bx + ay*by <= 0:
            result.append(point)
    return result + [points[-1]]


class Fleet(Node):
    def __init__(self):
        super().__init__('marco_sim')
        self.grid, self.spawn = junction()
        self.poses, self.received, self.destinations, self.routes = {}, {}, {}, {}
        self.tf = TransformBroadcaster(self)
        self.commands = {}
        self.peak_speed = {robot: 0. for robot in self.spawn}
        for robot in self.spawn:
            base = f'/{robot}/diffdrive_controller'
            self.commands[robot] = self.create_publisher(Twist, base + '/cmd_vel_unstamped', 10)
            self.create_subscription(Odometry, f'/{robot}/odom',
                                     lambda m, r=robot: self.receive(r, m), qos_profile_sensor_data)
        self.markers = self.create_publisher(MarkerArray, '/sim/fleet', 10)
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/sim/map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        message = OccupancyGrid()
        message.header.frame_id = 'map'
        message.info.width, message.info.height = self.grid['width'], self.grid['height']
        message.info.resolution = self.grid['resolution']
        message.info.origin.orientation.w = 1.
        message.data = self.grid['data']
        self.map_pub.publish(message)
        self.create_timer(.1, self.draw)

    def receive(self, robot, msg):
        self.peak_speed[robot] = max(self.peak_speed[robot], abs(msg.twist.twist.linear.x))
        p = msg.pose.pose
        q = p.orientation
        x, y, heading = self.spawn[robot]
        dx, dy = p.position.x, p.position.y
        # The differential drive controller starts its odometry at zero.
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        self.poses[robot] = [x + math.cos(heading)*dx - math.sin(heading)*dy,
                             y + math.sin(heading)*dx + math.cos(heading)*dy,
                             wrap(heading + yaw)]
        self.received[robot] = time.monotonic()
        x, y, yaw = self.poses[robot]
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'map'
        transform.child_frame_id = f'{robot}/base_link'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2)
        transform.transform.rotation.w = math.cos(yaw / 2)
        self.tf.sendTransform(transform)

    def draw(self):
        markers = MarkerArray()

        def marker(namespace, index, kind, rgb):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns, m.id, m.type = namespace, index, kind
            m.pose.orientation.w = 1.
            m.color.r, m.color.g, m.color.b = rgb
            m.color.a = 1.
            m.lifetime.nanosec = 500_000_000
            markers.markers.append(m)
            return m

        # Cube markers render the walls even on drivers where RViz's map shader fails.
        walls = marker('Occupied cells', 0, Marker.CUBE_LIST, (.22, .29, .30))
        resolution = self.grid['resolution']
        walls.scale.x = walls.scale.y = resolution
        walls.scale.z = .04
        width = self.grid['width']
        height = self.grid['height']
        floor = marker('Free space background', 0, Marker.CUBE, (.80, .83, .83))
        floor.scale.x, floor.scale.y, floor.scale.z = width*resolution, height*resolution, .02
        floor.pose.position = Point(x=floor.scale.x/2, y=floor.scale.y/2, z=-.01)
        walls.points = [Point(x=(index % width+.5)*resolution,
                              y=(index // width+.5)*resolution, z=.02)
                        for index, occupied in enumerate(self.grid['data']) if occupied]

        for i, (label, (x, y, _)) in enumerate(self.destinations.items()):
            m = marker('Start labels', i, Marker.TEXT_VIEW_FACING, (.08, .12, .14))
            m.text, m.scale.z = label, .14
            m.pose.position = Point(x=x, y=y+.19, z=.1)
        colors = [(0.2, .65, 1.), (1., .5, .2), (.3, .9, .4)]
        for robot, (x, y, yaw) in self.poses.items():
            i = int(robot[-1]) - 1
            color = colors[i]
            name = marker('Robot names', i, Marker.TEXT_VIEW_FACING, color)
            name.text, name.scale.z = robot, .10
            name.pose.position = Point(x=x, y=y, z=.62)
            arrow = marker(robot + ' heading', i, Marker.ARROW, color)
            arrow.points = [Point(x=x, y=y, z=.13),
                            Point(x=x+.23*math.cos(yaw), y=y+.23*math.sin(yaw), z=.13)]
            arrow.scale.x, arrow.scale.y, arrow.scale.z = .035, .065, .06
            if robot in self.routes:
                path = marker(robot + ' path', i, Marker.LINE_STRIP, color)
                path.scale.x = .025
                path.points = [Point(x=p[0], y=p[1], z=.03) for p in self.routes[robot]]
        self.markers.publish(markers)

    def stop(self):
        for publisher in self.commands.values():
            publisher.publish(Twist())

    def execute(self, actions, speed):
        pending = {a['robot']: (straight_segments(a['points']), 1) for a in actions}
        self.routes = {a['robot']: a['points'] for a in actions}
        deadline = time.monotonic() + 240.
        try:
            # Planning is synchronous, so refresh odometry before sending motion.
            started = time.monotonic()
            while any(self.received.get(robot, 0.) < started for robot in self.spawn):
                if time.monotonic() - started > 1.:
                    raise RuntimeError('Lost simulated odometry')
                rclpy.spin_once(self, timeout_sec=.02)
            while pending and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.02)
                now = time.monotonic()
                if any(now - self.received.get(robot, 0) > 1. for robot in self.spawn):
                    raise RuntimeError('Lost simulated odometry')
                for robot, (points, index) in list(pending.items()):
                    x, y, yaw = self.poses[robot]
                    gx, gy, goal_yaw = points[index]
                    distance = math.hypot(gx-x, gy-y)
                    twist = Twist()
                    if distance < .012 and index + 1 < len(points):
                        pending[robot] = (points, index+1)
                    else:
                        error = wrap((goal_yaw if distance < .012 else math.atan2(gy-y, gx-x)) - yaw)
                        if distance < .012 and abs(error) < .08:
                            del pending[robot]
                        else:
                            twist.angular.z = max(-.6, min(.6, 2*error))
                            if distance >= .012 and abs(error) < .12:
                                twist.linear.x = min(speed, 2*distance)
                    self.commands[robot].publish(twist)
            if pending:
                raise RuntimeError('Gazebo batch timed out')
        finally:
            self.stop()
            self.routes.clear()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--swap', nargs=3)
    parser.add_argument('--speed', type=float, default=.2, help='Gazebo speed cap in m/s')

    args = parser.parse_args()
    if not 0 < args.speed <= .2:
        parser.error('--speed must be greater than zero and at most 0.20 m/s')
    rclpy.init(args=[])
    fleet = Fleet()
    try:
        deadline = time.monotonic() + 90.
        while time.monotonic() < deadline and set(fleet.poses) != set(fleet.spawn):
            rclpy.spin_once(fleet, timeout_sec=.1)
        if set(fleet.poses) != set(fleet.spawn):
            raise RuntimeError('Timed out waiting for three Gazebo robots')
        fleet.destinations = {label: fleet.poses[robot].copy()
                              for label, robot in zip('ABC', fleet.spawn)}
        for label, robot in zip('ABC', fleet.spawn):
            print(f'{label} = {robot}: {fleet.destinations[label]}', flush=True)
        fleet.draw()
        # Keep ROS callbacks live while waiting for keyboard input.
        if args.swap:
            order = args.swap
        else:
            import select
            print('Nhập đích cho robot1 robot2 robot3 (ví dụ B C A): ', end='', flush=True)
            while not select.select([sys.stdin], [], [], 0)[0]:
                rclpy.spin_once(fleet, timeout_sec=.1)
            order = sys.stdin.readline().split()
        order = ['C' if label == '3' else label.upper() for label in order]
        if sorted(order) != ['A', 'B', 'C']:
            raise ValueError('Enter A, B and C exactly once')
        mission = swap_mission(order, fleet.destinations)
        mission['speed_mps'] = args.speed
        planner = SimulationPlanner(ROOT / 'MACRO')

        def snapshot():
            return {'map': fleet.grid, 'robots': {r: {'pose': p.copy()} for r, p in fleet.poses.items()}}

        mission_started = time.monotonic()
        assignment = planner.assign(snapshot(), mission)
        completed, intent = [], None
        for batch_number in range(1, 33):
            planning_started = time.monotonic()
            batch = planner.next_batch(snapshot(), mission, assignment, completed, intent)
            actions = batch['actions']
            if not actions:
                if batch['reason'] == 'completed':
                    elapsed = time.monotonic() - mission_started
                    print(f'Hoàn tất cả ba task trong {elapsed:.1f} s thực.', flush=True)
                    for robot, label in zip(fleet.spawn, order):
                        error = math.dist(fleet.poses[robot][:2], fleet.destinations[label][:2])
                        print(f'{robot} → {label}: sai số {error*100:.2f} cm, '
                              f'đỉnh odom {fleet.peak_speed[robot]:.3f} m/s', flush=True)
                    return
                raise RuntimeError(batch['reason'])
            planning_seconds = time.monotonic() - planning_started
            print(f"Lượt {batch_number}: {batch['audit']['strategy']}, "
                  f"{', '.join(a['robot'] for a in actions)} "
                  f"(lập kế hoạch {planning_seconds:.2f} s)", flush=True)
            execution_started = time.monotonic()
            fleet.execute(actions, args.speed)
            print(f'Lượt {batch_number} hoàn thành sau '
                  f'{time.monotonic() - execution_started:.1f} s thực', flush=True)
            completed.extend(a['task'] for a in actions if a['task'] is not None)
            intent = batch.get('intent')
        raise RuntimeError('Exceeded 32 MARCO batches')
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError) as error:
        print(error, file=sys.stderr)
        return 1
    finally:
        if rclpy.ok():
            fleet.stop()
        fleet.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
