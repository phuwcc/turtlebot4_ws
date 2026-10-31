"""Black-box C++ integration tests using fake sensors/Nav2 in an isolated ROS domain."""
import os
os.environ.update(ROS_DOMAIN_ID='173', ROS_LOCALHOST_ONLY='1',
                  ROS_SUPER_CLIENT='False', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
for key in ('ROS_DISCOVERY_SERVER','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','CYCLONEDDS_URI','ROS_NAMESPACE'):
    os.environ.pop(key, None)
import json
import math
import socket
import threading
import time
import unittest
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray
from native_process import NativeProcess


class NativeRuntimeTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('native_test_observer')
        self.executor = MultiThreadedExecutor(num_threads=6)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.processes = []
        self.latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def tearDown(self):
        for process in self.processes:
            process.close()
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        self.node.destroy_node()
        rclpy.shutdown()

    def wait(self, predicate, timeout=8):
        end = time.monotonic()+timeout
        while time.monotonic()<end:
            try:
                if predicate(): return
            except (OSError, KeyError): pass
            time.sleep(.02)
        self.fail('Native runtime condition timed out')

    def start(self, executable, params=None, namespace=None):
        process = NativeProcess(executable, params, namespace)
        self.processes.append(process)
        if process.port: process.wait_ready()
        return process

    def test_active_interfaces_and_parameter_rejections(self):
        for count, ids in ((1,''),(2,''),(3,''),(1,'robot2')):
            app = self.start('socket_control', {'robot_count':count,'robot_ids':ids})
            expected = ids.split(',') if ids else ['robot'+str(i+1) for i in range(count)]
            self.assertEqual(list(app.status()['robots']),expected)
            self.assertEqual(app.status()['selected_robot'],expected[0])
            self.wait(lambda:any(name=='fleet_view' for name,ns in self.node.get_node_names_and_namespaces()))
            def interfaces():
                return dict(self.node.get_subscriber_names_and_types_by_node('fleet_view','/'))
            self.wait(lambda: all('/'+r+'/scan' in interfaces() for r in expected))
            topics = interfaces()
            for robot in ('robot1','robot2','robot3'):
                self.assertEqual('/'+robot+'/scan' in topics,robot in expected)
            client = self.node.create_client(SetParameters,'/fleet_view/set_parameters')
            self.assertTrue(client.wait_for_service(timeout_sec=4))
            for parameter in (Parameter('robot_count',value=1), Parameter('selected_robot',value='robot4')):
                future = client.call_async(SetParameters.Request(parameters=[parameter.to_parameter_msg()]))
                self.wait(future.done)
                self.assertFalse(future.result().results[0].successful)
            app.close();self.processes.remove(app)
            self.wait(lambda: not any(name=='fleet_view' for name,ns in self.node.get_node_names_and_namespaces()))

    def test_bridge_latched_frames_and_display_scan_rate(self):
        app = self.start('socket_control', {'robot_count':3})
        received={};subscriptions=[];publishers=[]
        def subscribe(kind,topic,key,qos):
            subscriptions.append(self.node.create_subscription(kind,topic,lambda m:received.__setitem__(key,m),qos))
        subscribe(MarkerArray,'/tb4_view/labels','labels',self.latched)
        self.wait(lambda:'labels' in received)
        self.assertEqual(received['labels'].markers[-1].text,'Pose / Goal -> robot1')
        # Separate static batches must all survive for a late RViz subscriber.
        for robot in ('robot1','robot2','robot3'):
            publisher=self.node.create_publisher(TFMessage,'/'+robot+'/tf_static',self.latched)
            publishers.append(publisher);self.wait(lambda:publisher.get_subscription_count()>0)
            for child in ('base_link','rplidar_link'):
                t=TransformStamped();t.header.frame_id='odom';t.child_frame_id=child;t.transform.rotation.w=1.
                publisher.publish(TFMessage(transforms=[t]));time.sleep(.1)
        subscribe(TFMessage,'/tb4_view/tf_static','static',self.latched)
        self.wait(lambda:'static' in received and len(received['static'].transforms)==6)
        self.assertEqual({t.child_frame_id for t in received['static'].transforms},
            {r+'/'+c for r in ('robot1','robot2','robot3') for c in ('base_link','rplidar_link')})
        grid=self.node.create_publisher(OccupancyGrid,'/robot3/map',self.latched);publishers.append(grid)
        self.wait(lambda:grid.get_subscription_count()>0)
        m=OccupancyGrid();m.header.frame_id='map';m.info.width=m.info.height=1;m.info.resolution=.1;m.data=[0]
        grid.publish(m)
        subscribe(OccupancyGrid,'/tb4_view/robot3/map','map',self.latched)
        self.wait(lambda:'map' in received)
        self.assertEqual(received['map'].header.frame_id,'map')
        for topic in ('plan','local_plan'):
            path=self.node.create_publisher(Path,'/robot3/'+topic,10);publishers.append(path)
            subscribe(Path,'/tb4_view/robot3/'+topic,topic,qos_profile_sensor_data)
            self.wait(lambda:path.get_subscription_count()>0)
            m=Path();m.header.frame_id='map';p=PoseStamped();p.header.frame_id='odom';m.poses=[p];path.publish(m)
            self.wait(lambda:topic in received)
            self.assertEqual(received[topic].poses[0].header.frame_id,'robot3/odom')
        samples=[]
        subscriptions.append(self.node.create_subscription(LaserScan,'/tb4_view/robot3/scan',samples.append,qos_profile_sensor_data))
        scan=self.node.create_publisher(LaserScan,'/robot3/scan',qos_profile_sensor_data);publishers.append(scan)
        self.wait(lambda:scan.get_subscription_count()>0)
        end=time.monotonic()+1.
        while time.monotonic()<end:
            m=LaserScan();m.header.frame_id='rplidar_link';scan.publish(m);time.sleep(.01)
        time.sleep(.15)
        self.assertGreaterEqual(len(samples),2)
        self.assertLessEqual(len(samples),6)
        self.assertTrue(all(m.header.frame_id=='robot3/rplidar_link' for m in samples))

    def test_tcp_and_rviz_goal_ownership_cancel_and_validation(self):
        app=self.start('socket_control',{'robot_count':3})
        goals={r:[] for r in ('robot1','robot2','robot3')};done={r:threading.Event() for r in goals}
        poses=[];servers=[];group=ReentrantCallbackGroup()
        def execute(handle,robot):
            goals[robot].append(handle.request.pose)
            while not done[robot].is_set() and not handle.is_cancel_requested:time.sleep(.02)
            if handle.is_cancel_requested:handle.canceled()
            else:handle.succeed()
            return NavigateToPose.Result()
        try:
            # No action server exists yet; reject promptly rather than marking busy.
            self.assertEqual(app.request({'command':'goal','robot':'robot1','x':0,'y':0})['reason'],'nav_unavailable')
            for robot in goals:
                servers.append(ActionServer(self.node,NavigateToPose,'/'+robot+'/navigate_to_pose',
                    execute_callback=lambda h,r=robot:execute(h,r),cancel_callback=lambda _:CancelResponse.ACCEPT,
                    callback_group=group))
            sub=self.node.create_subscription(PoseWithCovarianceStamped,'/robot2/initialpose',poses.append,10)
            self.wait(lambda:all(s['nav_ready'] for s in app.status()['robots'].values()))
            for invalid in ({'command':'goal','robot':'robot4','x':0,'y':0},
                            {'command':'pose','robot':'robot2','x':True,'y':0},
                            {'command':'pose','robot':'robot2','x':float('nan'),'y':0},[]):
                self.assertFalse(app.request(invalid)['ok'])
            self.assertTrue(app.request({'command':'select','robot':'robot2'})['ok'])
            self.assertEqual(app.status()['selected_robot'],'robot2')
            self.assertTrue(app.request({'command':'pose','robot':'robot2','x':1.,'y':2.,'yaw':.5})['ok'])
            self.wait(lambda:poses)
            self.assertEqual(poses[-1].header.frame_id,'map');self.assertEqual(poses[-1].pose.pose.position.x,1.)
            rviz=self.node.create_publisher(PoseStamped,'/tb4_view/goal_pose',10)
            self.wait(lambda:rviz.get_subscription_count()>0)
            request={'command':'goal','robot':'robot2','x':3.,'y':4.}
            self.assertTrue(app.request(request)['ok'])
            self.wait(lambda:len(goals['robot2'])==1)
            self.assertFalse(app.request(request)['ok'])
            self.assertFalse(app.request({'command':'pose','robot':'robot2','x':0,'y':0})['ok'])
            p=PoseStamped();p.header.frame_id='map';p.pose.orientation.w=1.;rviz.publish(p)
            p.header.frame_id='odom';rviz.publish(p);time.sleep(.15)
            self.assertEqual(len(goals['robot2']),1)
            self.assertTrue(app.request({'command':'cancel','robot':'robot2'})['ok'])
            self.wait(lambda:app.status()['robots']['robot2']['last_result']=='canceled')
            # Each robot owns its own goal; canceling one cannot affect the others.
            for robot in goals:
                self.assertTrue(app.request({'command':'goal','robot':robot,'x':1,'y':1})['ok'])
            self.wait(lambda:all(s['accepted'] for s in app.status()['robots'].values()))
            self.assertTrue(app.request({'command':'select','robot':'robot3'})['ok'])
            cancel=self.node.create_client(Trigger,'/tb4_view/cancel')
            self.assertTrue(cancel.wait_for_service(timeout_sec=3))
            future=cancel.call_async(Trigger.Request());self.wait(future.done);self.assertTrue(future.result().success)
            self.wait(lambda:not app.status()['robots']['robot3']['busy'])
            self.assertTrue(app.status()['robots']['robot1']['busy']);self.assertTrue(app.status()['robots']['robot2']['busy'])
            for event in done.values():event.set()
            self.wait(lambda:not any(s['busy'] for s in app.status()['robots'].values()))
            # Bounded parser rejects oversized frames and continues serving later requests.
            with socket.create_connection(('127.0.0.1',app.port),timeout=3) as connection:
                connection.sendall(b'x'*65537+b'\n')
                self.assertFalse(json.loads(connection.makefile('rb').readline())['ok'])
            self.assertTrue(app.status()['ok'])
        finally:
            for event in done.values():event.set()
            time.sleep(.1)
            for server in servers:server.destroy()

    def test_native_guard_expiry_invalid_packets_multi_source_and_shutdown(self):
        guard=self.start('remote_velocity',{'mode':'guard'},'/robot1')
        observed=[]
        sub=self.node.create_subscription(Twist,'/robot1/cmd_vel',lambda m:observed.append((time.monotonic(),m.linear.x)),10)
        pub=self.node.create_publisher(TwistStamped,'/robot1/remote_cmd_vel',qos_profile_sensor_data)
        def send(speed,age=0):
            m=TwistStamped();stamp=self.node.get_clock().now().nanoseconds-int(age*1e9)
            m.header.stamp.sec,m.header.stamp.nanosec=divmod(stamp,10**9);m.twist.linear.x=speed;pub.publish(m);return m
        self.wait(lambda:pub.get_subscription_count()>0 and observed)
        send(.08);self.wait(lambda:any(v==.08 for _,v in observed))
        self.wait(lambda:observed[-1][1]==0.)
        baseline=len(observed)
        send(.2,age=3);send(float('nan'));send(.2,age=-.2);time.sleep(.15)
        self.assertTrue(all(v==0 for _,v in observed[baseline:]))
        latest=send(.04);self.wait(lambda:observed[-1][1]==.04)
        latest.twist.linear.x=.22;pub.publish(latest);time.sleep(.1)
        self.assertNotEqual(observed[-1][1],.22)
        extra=self.node.create_publisher(TwistStamped,'/robot1/remote_cmd_vel',qos_profile_sensor_data)
        self.wait(lambda:self.node.count_publishers('/robot1/remote_cmd_vel')==2)
        send(.07);time.sleep(.15);self.assertEqual(observed[-1][1],0.)
        self.node.destroy_publisher(extra)
        self.wait(lambda:self.node.count_publishers('/robot1/remote_cmd_vel')==1)
        send(.03);self.wait(lambda:observed[-1][1]==.03)
        # Pre-shutdown publishes stop while the context is still usable.
        guard.close();self.processes.remove(guard)
        self.wait(lambda:observed[-1][1]==0.)

    def test_native_stamp_relay(self):
        self.start('remote_velocity',{'mode':'stamp'},'/robot2')
        stamped=[]
        sub=self.node.create_subscription(TwistStamped,'/robot2/remote_cmd_vel',stamped.append,qos_profile_sensor_data)
        pub=self.node.create_publisher(Twist,'/robot2/pc_cmd_vel',qos_profile_sensor_data)
        self.wait(lambda:pub.get_subscription_count()>0)
        m=Twist();m.linear.x=.03
        def stream():
            pub.publish(m)
            return stamped
        self.wait(stream)
        self.assertEqual(stamped[-1].twist.linear.x,.03)
        self.assertGreater(stamped[-1].header.stamp.sec,0)


if __name__=='__main__':unittest.main()
