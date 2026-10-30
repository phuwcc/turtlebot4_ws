"""Loopback JSON-lines TCP gateway. All ROS work runs on the ROS executor."""
from concurrent.futures import Future, TimeoutError
import json
import math
import queue
import socketserver
import threading

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Trigger
from rclpy.parameter import Parameter
from tb4_multi_bringup.fleet_view import FleetView

MAX_REQUEST = 8192


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.request.settimeout(3)
        future = None
        try:
            raw = self.rfile.readline(MAX_REQUEST + 1)
            if len(raw) > MAX_REQUEST or not raw.endswith(b'\n'):
                raise ValueError('Expected one JSON object followed by newline, maximum 8192 bytes')
            request = json.loads(raw)
            if not isinstance(request, dict):
                raise ValueError('Expected JSON object')
            future = Future()
            self.server.pending.put_nowait((request, future))
            response = future.result(timeout=2)
        except (ValueError, OSError, TimeoutError, queue.Full) as error:
            if future is not None:
                future.cancel()
            response = {'ok': False, 'error': str(error) or 'Server busy; request expired'}
        try:
            self.wfile.write((json.dumps(response, allow_nan=False) + '\n').encode())
        except OSError:
            pass


class SocketControl(FleetView):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        port = self.declare_parameter('socket_port', 8765).value
        self.server = Server(('127.0.0.1', port), Handler)
        self.server.pending = queue.Queue(maxsize=32)
        self.socket_timer = self.create_timer(0.02, self.process_requests)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self.get_logger().info(f'Task socket listening on 127.0.0.1:{self.server.server_address[1]}')

    def process_requests(self):
        for _ in range(32):
            try:
                request, future = self.server.pending.get_nowait()
            except queue.Empty:
                return
            if not future.set_running_or_notify_cancel():
                continue
            try:
                response = self.dispatch(request)
            except (ValueError, TypeError, KeyError, OverflowError) as error:
                response = {'ok': False, 'error': str(error)}
            future.set_result(response)

    def dispatch(self, request):
        command = request.get('command')
        if command == 'status':
            return {'ok': True, 'selected_robot': self.get_parameter('selected_robot').value, 'robots': {r: {
                'nav_ready': self.action_clients[r].server_is_ready(),
                'busy': self.busy[r], 'accepted': self.goals[r] is not None,
                'cancel_requested': self.cancel_requested[r], 'last_result': self.last_result[r],
            } for r in self.robots}}
        robot = request.get('robot')
        if robot not in self.robots:
            raise ValueError('robot must be active: ' + ', '.join(self.robots))
        if command == 'select':
            result = self.set_parameters([Parameter('selected_robot', value=robot)])[0]
            return {'ok': result.successful, 'selected_robot': robot, 'message': result.reason}
        if command == 'cancel':
            result = self.cancel_robot(robot, Trigger.Response())
            return {'ok': result.success, 'message': result.message}
        if command not in ('pose', 'goal'):
            raise ValueError('command must be status, select, pose, goal or cancel')
        values = [request.get(key, 0.0 if key == 'yaw' else None) for key in ('x', 'y', 'yaw')]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
               for v in values):
            raise ValueError('x, y and yaw must be finite numbers; yaw is in radians')
        x, y, yaw = values
        message = PoseStamped() if command == 'goal' else PoseWithCovarianceStamped()
        message.header.frame_id = self.maps[robot]
        message.header.stamp = self.get_clock().now().to_msg()
        pose = message.pose if command == 'goal' else message.pose.pose
        pose.position.x, pose.position.y = float(x), float(y)
        pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        if self.busy[robot]:
            return {'ok': False, 'reason': 'robot_busy', 'message': 'Robot has an active/pending goal; cancel it first'}
        if command == 'goal' and not self.action_clients[robot].server_is_ready():
            return {'ok': False, 'reason': 'nav_unavailable',
                    'message': 'Nav2 action server is unavailable. Run bash scripts/pc-diagnose.sh on PC.'}
        if command == 'pose':
            message.pose.covariance[0] = message.pose.covariance[7] = 0.25
            message.pose.covariance[35] = 0.06854
        ok = bool(self.goal_pose(message, robot) if command == 'goal' else self.initial_pose(message, robot))
        return {'ok': ok, 'message': ('Goal submitted; poll status for acceptance/result' if command == 'goal'
                else 'Initial pose published') if ok else 'Request was not submitted; inspect fleet_view logs'}

    def destroy_node(self):
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join(timeout=2)
        return super().destroy_node()


def main():
    rclpy.init()
    node = SocketControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
