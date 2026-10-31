#!/usr/bin/env python3
"""Observe short goals on two robots; --execute explicitly enables MARCO motion.

Source the PC ROS environment first. This observer never publishes velocity.
The mission must cap each action to <=0.35 m; two goals run sequentially.
"""
import argparse
import json
import math
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import DockStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from tb4_marco_bridge.cli import request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mission', type=Path)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--output', type=Path, default=Path('log/marco/real_trial.json'))
    args = parser.parse_args()
    mission = json.loads(args.mission.read_text())
    assert 0 < mission.get('max_action_length_m', 100) <= .35
    assert mission.get('speed_mps') <= .05
    assert 1 <= len(mission['tasks']) <= 2
    active={t['candidates'][0] for t in mission['tasks'] if len(t['candidates'])==1}
    assert len(active)==len(mission['tasks']) and active <= {'robot1','robot2'}
    if len(mission['tasks'])==2:
        assert mission['tasks'][0]['id'] in mission['tasks'][1]['depends_on']
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[])
    node = rclpy.create_node('marco_short_trial_observer')
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    data = {r: {} for r in ('robot1', 'robot2')}
    buffers = {r: Buffer() for r in data}
    subscriptions = []
    report = {'motion_requested': args.execute, 'passed': False, 'robots': {}}
    process = None

    def receive(robot, topic, message):
        data[robot][topic] = (time.monotonic(), message)

    def transform(robot, message, static):
        for t in message.transforms:
            if static: buffers[robot].set_transform_static(t, 'hardware')
            else: buffers[robot].set_transform(t, 'hardware')

    latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for robot in data:
        for topic, kind in [('scan', LaserScan), ('odom', Odometry), ('cmd_vel', Twist), ('dock_status', DockStatus)]:
            subscriptions.append(node.create_subscription(kind, '/'+robot+'/'+topic,
                lambda m,r=robot,t=topic: receive(r,t,m), latched if topic=='dock_status' else qos_profile_sensor_data))
        for topic, static in [('tf',False), ('tf_static',True)]:
            subscriptions.append(node.create_subscription(TFMessage, '/'+robot+'/'+topic,
                lambda m,r=robot,s=static: transform(r,m,s), latched if static else qos_profile_sensor_data))

    def check(initial=False):
        moving = []
        for robot, topics in data.items():
            for topic in ('scan','odom','cmd_vel','dock_status'):
                assert topic in topics, robot+' missing '+topic
                if topic != 'dock_status': assert time.monotonic()-topics[topic][0] < .7, robot+' stale '+topic
            assert not topics['dock_status'][1].is_docked, robot+' is docked'
            odom = topics['odom'][1]
            stamp = odom.header.stamp.sec+odom.header.stamp.nanosec/1e9
            assert -.1 <= node.get_clock().now().nanoseconds/1e9-stamp <= .5, robot+' stale odom timestamp'
            p = odom.pose.pose.position
            assert math.isfinite(p.x) and math.isfinite(p.y)
            result = report['robots'].setdefault(robot, {'start_odom':[p.x,p.y], 'travel_m':0., 'max_linear_mps':0., 'max_angular_rps':0.})
            previous = result.get('end_odom', result['start_odom'])
            result['travel_m'] += math.hypot(p.x-previous[0], p.y-previous[1])
            result['end_odom'] = [p.x,p.y]
            assert result['travel_m'] < (.4 if robot in active else .02), robot+' exceeded trial travel bound'
            cmd = topics['cmd_vel'][1]
            result['max_linear_mps'] = max(result['max_linear_mps'], abs(cmd.linear.x))
            result['max_angular_rps'] = max(result['max_angular_rps'], abs(cmd.angular.z))
            assert abs(cmd.linear.x) <= .055 and abs(cmd.angular.z) <= .205, robot+' command exceeds trial limits'
            if abs(cmd.linear.x)>.001 or abs(cmd.angular.z)>.001: moving.append(robot)
            scan = topics['scan'][1]
            t = buffers[robot].lookup_transform('base_link', scan.header.frame_id, rclpy.time.Time()).transform
            q=t.rotation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            points=[]; front=[]
            for i,r in enumerate(scan.ranges):
                if not math.isfinite(r) or not scan.range_min <= r <= scan.range_max: continue
                a=scan.angle_min+i*scan.angle_increment+yaw
                x=r*math.cos(a)+t.translation.x; y=r*math.sin(a)+t.translation.y
                points.append(math.hypot(x,y))
                if x>0 and abs(y)<.24: front.append(x)
            assert len(points)>=20, robot+' insufficient lidar data'
            result['minimum_clearance_m']=min(result.get('minimum_clearance_m',100.),min(points))
            assert min(points)>.23, robot+' obstacle too close'
            if initial:
                if robot in active: assert not front or min(front)>.45, robot+' forward corridor is obstructed'
                assert robot not in moving, robot+' not initially stopped'
        assert len(moving)<=1, 'Both robots moving during sequential trial'

    try:
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            if all(all(t in v for t in ('scan','odom','cmd_vel','dock_status')) for v in data.values()): break
            time.sleep(.1)
        check(initial=True)
        snapshot=request({'command':'snapshot'},args.port)
        assert all(s['localized'] and s['odom_fresh'] and not s['busy'] for s in snapshot['robots'].values())
        report['before']=snapshot
        if args.execute:
            execution=args.output.with_name('real_execution.json')
            with args.output.with_suffix('.log').open('w') as stream:
                process=subprocess.Popen(['bash','scripts/marco.sh','run',str(args.mission),'--execute','--port',str(args.port),'--out',str(execution)],stdout=stream,stderr=subprocess.STDOUT)
                deadline=time.monotonic()+100
                while process.poll() is None:
                    check()
                    assert time.monotonic()<deadline, 'Trial deadline exceeded'
                    time.sleep(.05)
            assert process.returncode==0, 'MARCO execution failed; inspect '+str(execution)
            report['execution']=json.loads(execution.read_text())
            assert report['execution']['result']=='succeeded'
            for robot in active: assert report['robots'][robot]['travel_m']>.08, robot+' did not physically move'
            deadline=time.monotonic()+1.5
            while time.monotonic()<deadline:
                check()
                assert all(abs(v['cmd_vel'][1].linear.x)<.001 and abs(v['cmd_vel'][1].angular.z)<.001 for v in data.values()), 'Robot did not stop'
                time.sleep(.05)
        report['after']=request({'command':'snapshot'},args.port)
        report['passed']=True
    except BaseException as error:
        report['error']=type(error).__name__+': '+str(error)
        raise
    finally:
        if process is not None:
            for robot in data:
                try: request({'command':'cancel','robot':robot},args.port)
                except Exception: pass
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired: process.kill();process.wait()
            stop_deadline=time.monotonic()+3
            stable_since=None
            while time.monotonic()<stop_deadline:
                stopped=all('cmd_vel' in v and time.monotonic()-v['cmd_vel'][0]<.7 and
                    abs(v['cmd_vel'][1].linear.x)<.001 and abs(v['cmd_vel'][1].angular.z)<.001 for v in data.values())
                stable_since=(stable_since or time.monotonic()) if stopped else None
                if stable_since and time.monotonic()-stable_since>.5: break
                time.sleep(.05)
            report['stopped_after_trial']=stable_since is not None and time.monotonic()-stable_since>.5
            if not report['stopped_after_trial']: report['passed']=False
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True)
        executor.shutdown(timeout_sec=3);thread.join(timeout=3)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
