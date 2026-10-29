"""Laptop-only frame adapter and explicitly selected Nav2 control for one RViz."""
from copy import deepcopy
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PolygonStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from tb4_multi_bringup.fleet_settings import ROBOTS, active_robots
COLORS = ((0.2, 0.8, 1.0), (1.0, 0.55, 0.1), (0.3, 1.0, 0.3))


def display_frame(robot, frame, map_frame='map'):
    """Keep the shared map; isolate every other frame, without double prefixing."""
    frame = frame.lstrip('/')
    if not frame:
        return ''
    if frame == map_frame:
        return 'map'
    return frame if frame.startswith(robot + '/') else robot + '/' + frame


def adapt_message(message, robot, map_frame='map'):
    result = deepcopy(message)
    if isinstance(result, TFMessage):
        for transform in result.transforms:
            transform.header.frame_id = display_frame(robot, transform.header.frame_id, map_frame)
            transform.child_frame_id = display_frame(robot, transform.child_frame_id, map_frame)
    else:
        result.header.frame_id = display_frame(robot, result.header.frame_id, map_frame)
        if isinstance(result, Path):
            for pose in result.poses:
                pose.header.frame_id = display_frame(robot, pose.header.frame_id, map_frame)
    return result


def valid_pose(pose):
    values = (pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    norm = sum(v * v for v in values[3:])
    return all(math.isfinite(v) for v in values) and abs(norm - 1.0) < 0.01


class FleetView(Node):
    def __init__(self, **kwargs):
        super().__init__('fleet_view', **kwargs)
        count = self.declare_parameter('robot_count', 3).value
        self.robots = active_robots(count)
        self.declare_parameter('selected_robot', 'robot1')
        if self.get_parameter('selected_robot').value not in self.robots:
            raise ValueError('selected_robot must be active in this session')
        self.maps = {}
        self.bases = {}
        for robot in self.robots:
            self.maps[robot] = self.declare_parameter(robot + '.map_frame', 'map').value
            self.bases[robot] = self.declare_parameter(robot + '.base_frame', 'base_link').value
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.live = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.tf_pub = self.create_publisher(TFMessage, '/tb4_view/tf', 100)
        self.static_pub = self.create_publisher(TFMessage, '/tb4_view/tf_static', self.latched)
        self.static_cache = {}
        self.subscriptions_owned = []
        self.publishers_owned = []
        self.action_clients = {}
        self.goals = {robot: None for robot in self.robots}
        self.busy = {robot: False for robot in self.robots}
        self.last_result = {robot: None for robot in self.robots}
        self.cancel_requested = {robot: False for robot in self.robots}
        self.pose_pubs = {}
        for robot in self.robots:
            self.subscriptions_owned.append(self.create_subscription(
                TFMessage, '/' + robot + '/tf',
                lambda msg, r=robot: self.tf_pub.publish(adapt_message(msg, r, self.maps[r])),
                self.live))
            self.subscriptions_owned.append(self.create_subscription(
                TFMessage, '/' + robot + '/tf_static',
                lambda msg, r=robot: self.static_transform(msg, r), self.latched))
            for topic, kind, qos in (
                ('scan', LaserScan, self.live), ('map', OccupancyGrid, self.latched),
                ('amcl_pose', PoseWithCovarianceStamped, self.live),
                ('plan', Path, self.live), ('local_plan', Path, self.live),
                ('global_costmap/costmap', OccupancyGrid, self.latched),
                ('local_costmap/costmap', OccupancyGrid, self.latched),
                ('local_costmap/published_footprint', PolygonStamped, self.live),
            ):
                publisher = self.create_publisher(kind, '/tb4_view/' + robot + '/' + topic, qos)
                self.publishers_owned.append(publisher)
                self.subscriptions_owned.append(self.create_subscription(
                    kind, '/' + robot + '/' + topic,
                    lambda msg, r=robot, p=publisher: p.publish(adapt_message(msg, r, self.maps[r])), qos))
            self.pose_pubs[robot] = self.create_publisher(
                PoseWithCovarianceStamped, '/' + robot + '/initialpose', 10)
            self.action_clients[robot] = ActionClient(self, NavigateToPose, '/' + robot + '/navigate_to_pose')
        self.subscriptions_owned.append(self.create_subscription(
            PoseWithCovarianceStamped, '/tb4_view/initialpose', self.initial_pose, 10))
        self.subscriptions_owned.append(self.create_subscription(
            PoseStamped, '/tb4_view/goal_pose', self.goal_pose, 10))
        self.cancel_service = self.create_service(Trigger, '/tb4_view/cancel', self.cancel_goal)
        self.marker_pub = self.create_publisher(MarkerArray, '/tb4_view/labels', self.latched)
        self.timer = self.create_timer(0.5, self.publish_labels)
        self.get_logger().info('Fleet view ready: ' + ', '.join(self.robots) + '; selected robot1.')

    def validate_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'selected_robot' and parameter.value not in self.robots:
                return SetParametersResult(successful=False, reason='Active robots: ' + ', '.join(self.robots))
            if parameter.name == 'robot_count' or parameter.name.endswith(('.map_frame', '.base_frame')):
                return SetParametersResult(successful=False, reason='Frame settings require restarting the launch')
        return SetParametersResult(successful=True)

    def static_transform(self, message, robot):
        for transform in adapt_message(message, robot, self.maps[robot]).transforms:
            self.static_cache[transform.child_frame_id] = transform
        self.static_pub.publish(TFMessage(transforms=list(self.static_cache.values())))

    def control_target(self, message):
        if message.header.frame_id != 'map' or not valid_pose(message.pose.pose if
                isinstance(message, PoseWithCovarianceStamped) else message.pose):
            self.get_logger().error('Pose/goal rejected: use Fixed Frame map and a finite, normalized pose.')
            return None
        return self.get_parameter('selected_robot').value

    def initial_pose(self, message, robot=None):
        robot = robot or self.control_target(message)
        if robot is None:
            return
        if self.busy[robot]:
            self.get_logger().error('Cancel the active goal before setting initial pose for ' + robot)
            return
        result = deepcopy(message)
        result.header.frame_id = self.maps[robot]
        result.header.stamp = self.get_clock().now().to_msg()
        self.pose_pubs[robot].publish(result)
        self.get_logger().info('Initial pose -> ' + robot)
        return True

    def goal_pose(self, message, robot=None):
        robot = robot or self.control_target(message)
        if robot is None:
            return
        if self.busy[robot]:
            self.get_logger().error(robot + ' has an active/pending goal. Cancel it before sending another.')
            return
        client = self.action_clients[robot]
        if not client.server_is_ready():
            self.get_logger().error('Nav2 action server not ready for ' + robot + '; goal not sent.')
            return
        goal = NavigateToPose.Goal()
        goal.pose = deepcopy(message)
        goal.pose.header.frame_id = self.maps[robot]
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.busy[robot] = True
        self.last_result[robot] = None
        self.cancel_requested[robot] = False
        try:
            future = client.send_goal_async(goal)
            future.add_done_callback(lambda f, r=robot: self.goal_response(f, r))
            return True
        except Exception as error:
            self.busy[robot] = False
            self.get_logger().error(str(error))

    def goal_response(self, future, robot):
        try:
            handle = future.result()
            if not handle.accepted:
                self.last_result[robot] = "rejected"
                self.busy[robot] = False
                self.get_logger().error('Goal rejected by ' + robot)
                return
            self.goals[robot] = handle
            handle.get_result_async().add_done_callback(lambda f, r=robot: self.goal_result(f, r))
            self.get_logger().info('Goal accepted by ' + robot)
            if self.cancel_requested[robot]:
                self.request_cancel(robot)
        except Exception as error:
            self.busy[robot] = False
            self.get_logger().error('Goal response failed: ' + str(error))

    def goal_result(self, future, robot):
        self.goals[robot] = None
        self.busy[robot] = False
        self.cancel_requested[robot] = False
        try:
            status = future.result().status
            self.last_result[robot] = {4: 'succeeded', 5: 'canceled', 6: 'aborted'}.get(status, str(status))
            self.get_logger().info(robot + ' goal finished, status=' + str(status))
        except Exception as error:
            self.get_logger().error('Goal result failed: ' + str(error))

    def request_cancel(self, robot):
        future = self.goals[robot].cancel_goal_async()
        future.add_done_callback(lambda f, r=robot: self.cancel_response(f, r))

    def cancel_response(self, future, robot):
        try:
            if future.result().goals_canceling:
                self.get_logger().info('Cancellation accepted by ' + robot)
            else:
                self.get_logger().warning('Cancellation not accepted by ' + robot)
        except Exception as error:
            self.get_logger().error('Cancellation failed: ' + str(error))

    def cancel_goal(self, request, response):
        return self.cancel_robot(self.get_parameter('selected_robot').value, response)

    def cancel_robot(self, robot, response):
        response.success = self.busy[robot]
        response.message = ('Cancellation requested for ' if response.success else 'No owned goal for ') + robot
        if response.success:
            self.cancel_requested[robot] = True
            if self.goals[robot] is not None:
                self.request_cancel(robot)
        return response

    def publish_labels(self):
        selected = self.get_parameter('selected_robot').value
        markers = []
        for index, robot in enumerate(self.robots):
            marker = Marker()
            marker.header.frame_id = display_frame(robot, self.bases[robot], self.maps[robot])
            marker.ns = 'robots'
            marker.id = index
            marker.type = Marker.TEXT_VIEW_FACING
            marker.pose.orientation.w = 1.0
            marker.pose.position.z = 0.65
            marker.scale.z = 0.22
            marker.color.r, marker.color.g, marker.color.b = COLORS[index]
            marker.color.a = 1.0
            marker.text = robot + (' [SELECTED]' if robot == selected else '')
            markers.append(marker)
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'selection'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.orientation.w = 1.0
        marker.pose.position.y = -0.7
        marker.pose.position.z = 0.1
        marker.scale.z = 0.22
        marker.color.r, marker.color.g, marker.color.b = COLORS[ROBOTS.index(selected)]
        marker.color.a = 1.0
        marker.text = 'Pose / Goal -> ' + selected
        markers.append(marker)
        self.marker_pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = FleetView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
