#!/usr/bin/env python3
"""One namespaced AMCL/Nav2 stack, fake sensors, planning-only, isolated DDS."""
import json
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import time

parser=argparse.ArgumentParser()
parser.add_argument('robot_id', nargs='?', choices=['robot1','robot2','robot3'], default='robot1')
parser.add_argument('--pc', action='store_true', help='Test combined PC launch, headless socket and command routing')
args=parser.parse_args()
identity=args.robot_id
if args.pc and identity != 'robot1':
    parser.error('--pc uses robot1 (one-robot PC session)')
prefix='/'+identity
domain=172 if args.pc else {'robot1':168, 'robot2':169, 'robot3':170}[identity]
os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
                  RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_SUPER_CLIENT='False')
for key in ('ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI', 'ROS_NAMESPACE'):
    os.environ.pop(key, None)

import rclpy
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

root = Path(__file__).resolve().parents[1]
logs = root / 'log/navigation_smoke' / ('pc' if args.pc else identity)
logs.mkdir(parents=True, exist_ok=True)
(logs/'result.json').unlink(missing_ok=True)
os.environ['ROS_LOG_DIR'] = str(logs)
# A ten-meter square room: map and fake scan describe the same environment.
image = bytearray(254 if 0<x<99 and 0<y<99 else 0 for y in range(100) for x in range(100))
(logs/'room.pgm').write_bytes(b'P5\n100 100\n255\n'+image)
(logs/'room.yaml').write_text('image: room.pgm\nresolution: 0.1\norigin: [-5.0, -5.0, 0.0]\n'
                             'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
rclpy.init(args=[]); n=rclpy.create_node('navigation_test_sensors')
odom_pub=n.create_publisher(Odometry,prefix+'/odom',qos_profile_sensor_data)
scan_pub=n.create_publisher(LaserScan,prefix+'/scan',qos_profile_sensor_data)
tf_pub=n.create_publisher(TFMessage,prefix+'/tf',20)
static_pub=n.create_publisher(TFMessage,prefix+'/tf_static',
    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
t=TransformStamped();t.header.frame_id='base_link';t.child_frame_id='rplidar_link';t.transform.rotation.w=1.
static_pub.publish(TFMessage(transforms=[t]))


def sensors():
    stamp=n.get_clock().now().to_msg()
    odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom';odom.child_frame_id='base_link'
    odom.pose.pose.orientation.w=1.;odom_pub.publish(odom)
    t=TransformStamped();t.header=odom.header;t.child_frame_id='base_link';t.transform.rotation.w=1.
    tf_pub.publish(TFMessage(transforms=[t]))
    scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='rplidar_link'
    scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/180;scan.angle_max=-math.pi+179*scan.angle_increment
    scan.range_min=.15;scan.range_max=12.;scan.scan_time=.1
    scan.ranges=[min(4.9/max(abs(math.cos(-math.pi+i*scan.angle_increment)),1e-6),
                     4.9/max(abs(math.sin(-math.pi+i*scan.angle_increment)),1e-6)) for i in range(180)]
    scan_pub.publish(scan)


def pump(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)


timer=n.create_timer(.1,sensors)
names=['amcl','map_server','controller_server','planner_server','bt_navigator']
clients={name:n.create_client(GetState,prefix+'/'+name+'/get_state') for name in names}
planner=ActionClient(n,ComputePathToPose,prefix+'/compute_path_to_pose')
stream=(logs/'navigation.log').open('w')
command = (['ros2','launch','tb4_multi_bringup','pc_navigation.launch.py', 'robot_count:=1',
    'rviz:=false', 'socket_port:=18765', 'map:='+str(logs/'room.yaml')] if args.pc else
    ['ros2','launch','tb4_multi_bringup','navigation.launch.py','robot_id:='+identity,
    'map:='+str(logs/'room.yaml'),'initialize_pose:=true','initial_x:=0.0','initial_y:=0.0','initial_yaw:=0.0'])
p=subprocess.Popen(command,
    stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
result={'test_domain':domain,'robot_id':identity}
try:
    if args.pc:
        import socket
        def task(request):
            with socket.create_connection(('127.0.0.1',18765),timeout=4) as connection:
                connection.sendall((json.dumps(request)+'\n').encode())
                return json.loads(connection.makefile('rb').readline())
    deadline=time.monotonic()+65
    states={}
    while time.monotonic()<deadline:
        assert p.poll() is None,'Navigation launch exited'
        pump(2)
        if args.pc:
            try:
                task({'command':'pose','robot':'robot1','x':0.,'y':0.,'yaw':0.})
            except OSError:
                pass
        if not all(c.service_is_ready() for c in clients.values()):continue
        futures={k:c.call_async(GetState.Request()) for k,c in clients.items()}
        end=time.monotonic()+4
        while time.monotonic()<end and not all(f.done() for f in futures.values()):pump(.1)
        states={k:f.result().current_state.label if f.done() and f.result() else 'TIMEOUT' for k,f in futures.items()}
        if all(s=='active' for s in states.values()):break
    result['lifecycle']=states
    assert states and all(s=='active' for s in states.values()),'Not all lifecycle nodes active: '+str(states)
    assert planner.server_is_ready(),'Namespaced planner unavailable'
    if args.pc:
        assert task({'command':'status'})['robots']['robot1']['nav_ready']
        assert task({'command':'pose','robot':'robot1','x':0.,'y':0.,'yaw':0.})['ok']
        pump(4)
        result['pc_socket']=True
    publishers={e.node_name for e in n.get_publishers_info_by_topic(prefix+'/pc_cmd_vel')}
    assert {'velocity_smoother','behavior_server'} <= publishers, publishers
    assert not n.get_publishers_info_by_topic(prefix+'/cmd_vel'), 'PC bypassed Pi guard'
    nav_publishers = {e.node_name for e in n.get_publishers_info_by_topic(prefix+'/cmd_vel_nav')}
    assert nav_publishers == {'controller_server'}, nav_publishers
    assert 'velocity_smoother' in {e.node_name for e in n.get_subscriptions_info_by_topic(prefix+'/cmd_vel_nav')}
    assert n.get_publishers_info_by_topic(prefix+'/remote_cmd_vel'), 'Missing stamped command relay'
    result['command_routing']=True
    goal=ComputePathToPose.Goal();goal.goal.header.frame_id='map'
    goal.goal.pose.position.x=.5;goal.goal.pose.orientation.w=1.;goal.planner_id='GridBased'
    f=planner.send_goal_async(goal)
    end=time.monotonic()+10
    while time.monotonic()<end and not f.done():pump(.05)
    assert f.done() and f.result().accepted,'Planning goal not accepted'
    f=f.result().get_result_async();end=time.monotonic()+15
    while time.monotonic()<end and not f.done():pump(.05)
    assert f.done() and f.result(),'Planning result timeout'
    result['path_status']=f.result().status;result['path_poses']=len(f.result().result.path.poses)
    assert result['path_status']==4 and result['path_poses']>1,'Planning failed'
    assert not n.get_publishers_info_by_topic('/map'),'Global map leaked'
    assert not n.get_publishers_info_by_topic('/tf'),'Global TF leaked'
    result['namespaced_only']=True
    print(json.dumps(result,indent=2),flush=True)
    (logs/'result.json').write_text(json.dumps(result,indent=2))
finally:
    if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=18)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
    stream.close();n.destroy_node();rclpy.shutdown()
