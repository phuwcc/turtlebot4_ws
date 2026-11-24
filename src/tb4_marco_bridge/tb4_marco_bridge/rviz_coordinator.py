"""Live RViz goals -> unmodified MARCO core -> leased Nav2 FollowPath actions."""
import json
import fcntl
import sys
import math
import os
from pathlib import Path
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .cli import (Worker, request, ready, execute_batch, ReplanRequested, CorridorViolation,
                  LocalizationUnavailable, wait_recovery)
from .planning import robot_index
from .smoothing import smooth_points


class RvizCoordinator(Node):
    def __init__(self):
        super().__init__('marco_rviz')
        self.robots=self.declare_parameter('robot_ids','robot1,robot2').value.split(',')
        # Any fleet of robotN IDs; MARCO itself picks the priority branch (dag_aware).
        if len(set(self.robots))!=len(self.robots) or not self.robots or any(robot_index(r)<1 for r in self.robots):
            raise ValueError('robot_ids must be unique robotN names')
        # Optional MACRO-style mission (objects + task DAG). When set, the
        # coordinator runs it once the fleet is ready instead of waiting for clicks.
        mission_file=self.declare_parameter('mission_file','').value
        self.file_mission=json.loads(Path(mission_file).read_text()) if mission_file else None
        self.file_done=False
        if self.file_mission is not None and not isinstance(self.file_mission.get('tasks'),list):
            raise ValueError('mission_file needs a tasks list')
        self.port=self.declare_parameter('socket_port',8765).value
        default=Path(__file__).resolve().parents[3]/'MACRO'
        self.root=Path(self.declare_parameter('marco_root',os.environ.get('MARCO_ROOT',str(default))).value)
        self.speed=float(self.declare_parameter('speed_mps',.16).value)
        self.tolerance=float(self.declare_parameter('tracking_tolerance',.06).value)
        if not 0<self.speed<=.25 or not .04<=self.tolerance<=.10:
            raise ValueError('Invalid coordinated speed/tracking tolerance')
        # Corner rounding after the MARCO core; 0 disables. Must leave most of
        # the tracking corridor (checked against the reserved MARCO path).
        self.smooth_deviation=float(self.declare_parameter('smooth_deviation',.03).value)
        # MARCO execute.py DEMO_TRIGGER_HORIZON_S default.
        self.trigger_horizon=float(self.declare_parameter('trigger_horizon_s',3.).value)
        if not 0<self.trigger_horizon<=10:raise ValueError('trigger_horizon_s must be within 0..10 s')
        self.predicted_contacts=None
        # Coordination clearance comes from MACRO itself (same radius as the
        # swept trigger and the ROS/grid simulations), not from adapter margins.
        sys.path.insert(0,str(self.root/'src'))
        from simulations.ros2.marco_geometry import ROBOT_RADIUS
        # Small safety layer on top of MACRO's radius (0.22 m = body 0.175 + 0.045).
        # One value drives MARCO planning, the swept trigger and the bridge's
        # last-line peer guard, so they never disagree.
        self.safety_margin=float(self.declare_parameter('safety_margin',.015).value)
        # Seconds the whole fleet must stay localized and Nav2-ready before a
        # mission (or a new RViz goal set) starts.
        self.start_stable_s=float(self.declare_parameter('start_stable_s',2.).value)
        if not 0<=self.start_stable_s<=60:raise ValueError('start_stable_s must be within 0..60 s')
        if not 0<=self.safety_margin<=.05:raise ValueError('safety_margin must be within 0..0.05 m')
        self.coordination_radius=float(ROBOT_RADIUS)+self.safety_margin
        if not 0<=self.smooth_deviation<=self.tolerance/2:
            raise ValueError('smooth_deviation must be within 0..tracking_tolerance/2')
        self.directory=Path(self.declare_parameter('log_directory',str(Path.cwd()/'log/pc/marco_rviz')).value)
        self.directory.mkdir(parents=True,exist_ok=True)
        default_lock=Path(os.environ.get('XDG_STATE_HOME',str(Path.home()/'.local/state')))/'tb4_multi_robot/pc/marco.lock'
        lock_path=Path(self.declare_parameter('mission_lock_path',str(default_lock)).value)
        lock_path.parent.mkdir(parents=True,exist_ok=True)
        self.mission_lock=lock_path.open('a')
        fcntl.flock(self.mission_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.lock=threading.Lock();self.stop=threading.Event()
        self.goals={};self.revision=0;self.changed_at=0.;self.phase='idle';self.detail='Waiting for goals';self.active=[]
        self.worker=None
        self.status_pub=self.create_publisher(String,'/tb4_view/marco_status',10)
        self.markers=self.create_publisher(MarkerArray,'/tb4_view/marco_coordination',10)
        # Manual start: the session (RViz, localization, Nav2) comes up idle; poses
        # are set with the Pose robotN tools, then `marco_start.bash MISSION.json`
        # publishes the mission path here and the MARCO run begins.
        self.manual_start=False
        self.start_subscription=self.create_subscription(String,'/tb4_view/marco/start',self.receive_start,10)
        self.goal_subscriptions=[self.create_subscription(PoseStamped,'/tb4_view/'+r+'/goal_pose',
            lambda m,r=r:self.receive_goal(r,m),10) for r in self.robots]
        self.timer=self.create_timer(.3,self.publish_status)
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()

    def receive_goal(self,robot,message):
        p=message.pose.position;q=message.pose.orientation
        values=[p.x,p.y,q.x,q.y,q.z,q.w]
        if message.header.frame_id!='map' or not all(math.isfinite(x) for x in values) or abs(sum(x*x for x in values[2:])-1.)>.02:
            self.get_logger().error('Rejected invalid map goal for '+robot);return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        with self.lock:
            self.revision+=1;self.goals[robot]={'pose':[p.x,p.y,yaw],'revision':self.revision}
            self.changed_at=time.monotonic()
        self.get_logger().info('MARCO goal queued for '+robot)

    def receive_start(self,message):
        try:
            path=Path(message.data).expanduser() if message.data else None
            mission=json.loads(path.read_text()) if path else self.file_mission
            if not isinstance(mission,dict) or not isinstance(mission.get('tasks'),list) or not mission['tasks']:
                raise ValueError('mission needs a non-empty tasks list')
        except (OSError,ValueError) as error:
            self.set_state('error','Start rejected: '+str(error));return
        with self.lock:
            if self.phase=='executing' or self.phase=='planning':
                self.get_logger().warn('MARCO start ignored: a mission is already running');return
            self.file_mission=mission;self.file_done=False;self.manual_start=True
            self.revision+=1;self.changed_at=0.
        self.get_logger().info('MARCO start requested: '+(str(path) if path else 'mission_file'))

    def contact(self,actions,report,hits,moving_only=False):
        """MARCO swept-contact trigger (three_robot_t/swept_trigger.predicted_contacts)."""
        if self.predicted_contacts is None:
            from simulations.ros2.three_robot_t.swept_trigger import predicted_contacts
            self.predicted_contacts=predicted_contacts
        snapshot=request({'command':'snapshot'},self.port)
        poses={r:snapshot['robots'][r]['pose'] for r in self.robots}
        finished={e['robot'] for e in report if e.get('event')=='completed'}
        routes={r:[poses[r][:2]] for r in self.robots}   # idle robots hold position
        speeds={r:0. for r in self.robots}
        for a in actions:
            if a['robot'] in finished:continue
            routes[a['robot']]=[p[:2] for p in a.get('reserved',a['points'])];speeds[a['robot']]=self.speed
        pairs=[(a,b) for i,a in enumerate(self.robots) for b in self.robots[i+1:]]
        if moving_only:
            # Inside a MARCO intervention held robots were already cleared by the
            # plan; only pairs that are both driving (e.g. a released yielder and
            # the priority robot) need the swept trigger.
            pairs=[(a,b) for a,b in pairs if speeds[a]>0 and speeds[b]>0]
            if not pairs:return False
        found=self.predicted_contacts(poses,routes,pairs,speeds,horizon_s=self.trigger_horizon,radius_m=self.coordination_radius)
        if found:
            hits[:]=found
            self.get_logger().info('MARCO contact trigger: '+json.dumps(found))
        return bool(found)

    def releaser(self,mission,assignment,completed,report):
        """Build the concurrent-execution hook: about once per second ask MARCO's
        planner which waiting robots may start now, given the remaining paths of
        the robots that are driving."""
        last=[0.]
        def remaining(action,pose):
            points=action.get('reserved',action['points'])
            k=min(range(len(points)),key=lambda i:math.dist(points[i][:2],pose[:2]))
            return [list(pose[:2])]+[p[:2] for p in points[k+1:]]
        def extra(active,snapshot,status):
            if time.monotonic()-last[0]<1.:return []
            last[0]=time.monotonic()
            done=completed+[e['task'] for e in report if e.get('event')=='completed' and e.get('task') is not None]
            moving={r:remaining(a,snapshot['robots'][r]['pose']) for r,a in active.items()
                    if status[r]['busy'] or 'acting_since' not in a}
            snap=dict(snapshot);snap['map']=self.base_map
            try:
                batch=self.worker.call({'command':'release','snapshot':snap,'mission':mission,
                    'assignment':assignment,'completed':done,'moving':moving})
            except RuntimeError as error:
                self.get_logger().warn('release check failed: '+str(error));return []
            for action in batch['actions']:
                action['reserved']=action['points']
                action['points']=smooth_points(action['points'],self.smooth_deviation)
                request({'command':'preview','robot':action['robot'],'points':action['points']},self.port)
            if batch['actions']:
                self.get_logger().info('MARCO release: '+', '.join(a['robot'] for a in batch['actions']))
                with self.lock:self.active=sorted(set(self.active)|{a['robot'] for a in batch['actions']})
            return batch['actions']
        return extra

    def ready_snapshot(self,timeout=8.):
        """Fresh fleet snapshot; ride out a few Wi-Fi-delayed (stale) scans."""
        deadline=time.monotonic()+timeout
        while True:
            snapshot=request({'command':'snapshot'},self.port)
            try:
                ready(snapshot);return snapshot
            except LocalizationUnavailable as error:
                if not error.is_stale or time.monotonic()>deadline:raise
            self.stop.wait(.2)

    def set_state(self,phase,detail,active=()):
        with self.lock:self.phase=phase;self.detail=detail;self.active=list(active)
        self.get_logger().info(phase+': '+detail)

    def interrupted(self,revision):
        with self.lock:return self.stop.is_set() or self.revision!=revision

    def publish_status(self):
        with self.lock:
            status={'phase':self.phase,'detail':self.detail,'priority':'MARCO dag_aware',
                    'active':list(self.active),'goals':dict(self.goals),'revision':self.revision}
        self.status_pub.publish(String(data=json.dumps(status,allow_nan=False)))
        temporary=self.directory/'status.tmp'
        temporary.write_text(json.dumps(status,indent=2)+'\n');temporary.replace(self.directory/'status.json')
        array=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;array.markers.append(clear)
        def label(identifier,text,x,y,z,color):
            m=Marker();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
            m.ns='marco';m.id=identifier;m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD
            m.pose.position.x=x;m.pose.position.y=y;m.pose.position.z=z;m.pose.orientation.w=1.
            m.scale.z=.16;m.color.r,m.color.g,m.color.b=color;m.color.a=1.;m.text=text
            array.markers.append(m)
        label(0,'MARCO (dag_aware) | '+status['detail'],0.,-.95,.2,(1.,1.,1.))
        colors={'robot1':(.2,.8,1.),'robot2':(1.,.55,.1),'robot3':(.4,1.,.3)}
        for index,(robot,goal) in enumerate(status['goals'].items(),1):
            pose=goal['pose'];color=colors.get(robot,(.9,.9,.2))
            suffix='moving' if robot in status['active'] else 'waiting'
            label(index,'Goal '+robot+' ['+suffix+']',pose[0],pose[1],.35,color)
            arrow=Marker();arrow.header.frame_id='map';arrow.header.stamp=self.get_clock().now().to_msg()
            arrow.ns='marco_goals';arrow.id=index;arrow.type=Marker.ARROW;arrow.action=Marker.ADD
            arrow.pose.position.x=pose[0];arrow.pose.position.y=pose[1];arrow.pose.position.z=.08
            arrow.pose.orientation.z=math.sin(pose[2]/2);arrow.pose.orientation.w=math.cos(pose[2]/2)
            arrow.scale.x=.35;arrow.scale.y=.07;arrow.scale.z=.10
            arrow.color.r,arrow.color.g,arrow.color.b=color;arrow.color.a=1.;array.markers.append(arrow)
        self.markers.publish(array)

    def run(self):
        blocked=-1;stable_since=None
        try:
            while not self.stop.wait(.15):
                with self.lock:goals=dict(self.goals);revision=self.revision;changed=self.changed_at
                pending_file=self.file_mission is not None and not self.file_done
                if (not goals and not pending_file) or revision==blocked or time.monotonic()-changed<.35:continue
                try:
                    snapshot=request({'command':'snapshot','include_map':True},self.port)
                    ready(snapshot)
                    status=request({'command':'status'},self.port)
                    if not status.get('coordinated_goals'):raise RuntimeError('Gateway is not in MARCO mode')
                    if any(s['busy'] for s in status['robots'].values()):raise RuntimeError('A robot is still executing')
                    if not all(s['path_ready'] for s in status['robots'].values()):raise RuntimeError('Nav2 FollowPath not ready')
                except (OSError,RuntimeError) as error:
                    stable_since=None
                    self.set_state('waiting',str(error)[:160]);self.stop.wait(1.);continue
                # Start the MARCO scenario only after every robot has been localized,
                # fresh and Nav2-ready without interruption for start_stable_s.
                stable_since=stable_since or time.monotonic()
                held=time.monotonic()-stable_since
                if held<(0. if self.manual_start else self.start_stable_s):
                    self.set_state('waiting','All robots localized; holding %.1f/%.0f s before starting MARCO'
                                   %(held,self.start_stable_s))
                    self.stop.wait(.5);continue
                stable_since=None;self.manual_start=False
                try:
                    if self.worker is None:self.worker=Worker(self.root,self.directory/'worker.log')
                    mission={'speed_mps':self.speed,'tracking_tolerance':self.tolerance,
                        'robot_radius':.175,'safety_margin':.05,'coordination_radius':self.coordination_radius,'action_timeout_s':180.,'nominal_until_contact':True,'intervene':False,
                        'tasks':[{'id':robot_index(r),'goal':g['pose'],'candidates':[r],'depends_on':[]} for r,g in sorted(goals.items())]}
                    if pending_file and not goals:
                        # MACRO scenario: objects + task DAG; durations drive DAG slack.
                        mission.update(objects=self.file_mission.get('objects',[]),tasks=self.file_mission['tasks'])
                    self.set_state('planning','Planning (MARCO chooses the priority branch)')
                    assignment=self.worker.call({'command':'assign','snapshot':snapshot,'mission':mission})
                    base_map=snapshot['map'];map_revision=snapshot['map_revision'];completed=[];intent=None;recoveries=0;self.base_map=base_map
                    for _ in range(32):
                        if self.interrupted(revision):raise ReplanRequested('New goal')
                        snapshot=self.ready_snapshot()
                        if snapshot['map_revision']!=map_revision:raise RuntimeError('Map changed; set goals again')
                        snapshot['map']=base_map
                        batch=self.worker.call({'command':'next','snapshot':snapshot,'mission':mission,
                            'assignment':assignment,'completed':completed,'intent':intent})
                        intent=batch.get('intent')
                        for action in batch['actions']:
                            action['reserved']=action['points']
                            action['points']=smooth_points(action['points'],self.smooth_deviation)
                        with (self.directory/'plans.jsonl').open('a') as stream:
                            stream.write(json.dumps({'revision':revision,'mission':mission,'batch':batch})+'\n')
                        if not batch['actions']:
                            if batch.get('reason')!='completed':raise RuntimeError('Task graph cannot progress')
                            self.file_done=self.file_done or (pending_file and not goals);self.set_state('idle','Goals completed');break
                        for action in batch['actions']:
                            request({'command':'preview','robot':action['robot'],'points':action['points']},self.port)
                        strategy=batch['audit']['strategy']
                        self.set_state('executing',strategy,[a['robot'] for a in batch['actions']])
                        report=[];hits=[]
                        nominal=strategy=='nominal'
                        watch=lambda:self.interrupted(revision) or self.contact(batch['actions'],report,hits,moving_only=not nominal)
                        release=self.releaser(mission,assignment,completed,report)
                        try:
                            done=execute_batch(batch['actions'],mission,self.port,map_revision,report,watch,release)
                        except ReplanRequested:
                            if not hits or self.interrupted(revision):raise
                            # MARCO trigger: robots were stopped by execute_batch; let them
                            # settle, then plan the intervention from the conflict poses.
                            done=[e['task'] for e in report if e.get('event')=='completed' and e.get('task') is not None]
                            self.set_state('waiting','Contact predicted '+'/'.join(hits[0]['robots'])+
                                ' (%.2f m in %.1f s); MARCO intervention'%(hits[0]['predicted_separation_m'],hits[0]['interval_s'][1]))
                            self.stop.wait(1.)
                            for _ in range(25):
                                if not any(s['busy'] for s in request({'command':'status'},self.port)['robots'].values()):break
                                self.stop.wait(.2)
                            else:raise RuntimeError('Robots did not stop after the contact trigger')
                            self.stop.wait(.5)   # MARCO settle time before planning
                            mission['intervene']=True;intent=None
                        except (CorridorViolation,LocalizationUnavailable) as error:
                            # Execution fault (Wi-Fi-delayed scans, AMCL loss, corridor
                            # drift): execute_batch already cancelled the leased paths.
                            # Wait for a stopped, localized fleet and replan from the
                            # measured poses instead of abandoning the mission.
                            if isinstance(error,LocalizationUnavailable) and not error.is_stale:raise
                            recoveries+=1
                            if recoveries>3:raise
                            done=[e['task'] for e in report if e.get('event')=='completed' and e.get('task') is not None]
                            self.set_state('waiting','Recovering (%d/3): %s'%(recoveries,str(error)[:120]))
                            wait_recovery(self.port,map_revision,tuple(self.robots),timeout=30.)
                            intent=None
                        else:
                            # Intervention finished: remaining routes run nominally again.
                            if mission['intervene'] and intent is None:mission['intervene']=False
                        completed+=done
                        with self.lock:
                            for r,goal in goals.items():
                                if robot_index(r) in done and self.goals.get(r,{}).get('revision')==goal['revision']:
                                    del self.goals[r]
                    else:raise RuntimeError('MARCO intervention limit exceeded')
                except ReplanRequested:
                    self.set_state('waiting','Stopping previous paths before replanning')
                except Exception as error:
                    # Path leases and execute_batch's finally block stop owned
                    # actions. Keep rejected goals visible; a new click retries.
                    blocked=revision;self.set_state('error',str(error))
        finally:
            if self.worker is not None:self.worker.close();self.worker=None

    def close(self):
        self.stop.set();self.thread.join(timeout=8)
        self.mission_lock.close()


def main():
    rclpy.init();node=RvizCoordinator()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
