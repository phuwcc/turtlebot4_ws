"""Live RViz goals -> unmodified MARCO core -> leased Nav2 FollowPath actions."""
import json
import fcntl
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

from .cli import Worker, request, ready, execute_batch, ReplanRequested


class RvizCoordinator(Node):
    def __init__(self):
        super().__init__('marco_rviz')
        self.robots=self.declare_parameter('robot_ids','robot1,robot2').value.split(',')
        if self.robots!=['robot1','robot2']:
            raise ValueError('RViz MARCO currently requires robot1,robot2')
        self.port=self.declare_parameter('socket_port',8765).value
        default=Path(__file__).resolve().parents[3]/'MACRO'
        self.root=Path(self.declare_parameter('marco_root',os.environ.get('MARCO_ROOT',str(default))).value)
        self.speed=float(self.declare_parameter('speed_mps',.12).value)
        self.tolerance=float(self.declare_parameter('tracking_tolerance',.08).value)
        if not 0<self.speed<=.12 or not .04<=self.tolerance<=.10:
            raise ValueError('Invalid coordinated speed/tracking tolerance')
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

    def set_state(self,phase,detail,active=()):
        with self.lock:self.phase=phase;self.detail=detail;self.active=list(active)
        self.get_logger().info(phase+': '+detail)

    def interrupted(self,revision):
        with self.lock:return self.stop.is_set() or self.revision!=revision

    def publish_status(self):
        with self.lock:
            status={'phase':self.phase,'detail':self.detail,'priority_order':self.robots,
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
        label(0,'MARCO: robot1 > robot2 | '+status['detail'],0.,-.95,.2,(1.,1.,1.))
        for index,(robot,goal) in enumerate(status['goals'].items(),1):
            pose=goal['pose'];color=(.2,.8,1.) if robot=='robot1' else (1.,.55,.1)
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
        blocked=-1
        try:
            while not self.stop.wait(.15):
                with self.lock:goals=dict(self.goals);revision=self.revision;changed=self.changed_at
                if not goals or revision==blocked or time.monotonic()-changed<.35:continue
                try:
                    snapshot=request({'command':'snapshot','include_map':True},self.port)
                    ready(snapshot)
                    status=request({'command':'status'},self.port)
                    if not status.get('coordinated_goals'):raise RuntimeError('Gateway is not in MARCO mode')
                    if any(s['busy'] for s in status['robots'].values()):continue
                    if not all(s['path_ready'] for s in status['robots'].values()):continue
                except (OSError,RuntimeError) as error:
                    self.set_state('waiting',str(error));self.stop.wait(1.);continue
                try:
                    if self.worker is None:self.worker=Worker(self.root,self.directory/'worker.log')
                    mission={'priority_order':self.robots,'speed_mps':self.speed,'tracking_tolerance':self.tolerance,
                        'robot_radius':.175,'safety_margin':.05,'action_timeout_s':180.,
                        'tasks':[{'id':int(r[-1]),'goal':g['pose'],'candidates':[r],'depends_on':[]} for r,g in sorted(goals.items())]}
                    self.set_state('planning','Planning; robot1 has priority')
                    assignment=self.worker.call({'command':'assign','snapshot':snapshot,'mission':mission})
                    base_map=snapshot['map'];map_revision=snapshot['map_revision'];completed=[];intent=None
                    for _ in range(32):
                        if self.interrupted(revision):raise ReplanRequested('New goal')
                        snapshot=request({'command':'snapshot'},self.port);ready(snapshot)
                        if snapshot['map_revision']!=map_revision:raise RuntimeError('Map changed; set goals again')
                        snapshot['map']=base_map
                        batch=self.worker.call({'command':'next','snapshot':snapshot,'mission':mission,
                            'assignment':assignment,'completed':completed,'intent':intent})
                        intent=batch.get('intent')
                        with (self.directory/'plans.jsonl').open('a') as stream:
                            stream.write(json.dumps({'revision':revision,'mission':mission,'batch':batch})+'\n')
                        if not batch['actions']:
                            if batch.get('reason')!='completed':raise RuntimeError('Task graph cannot progress')
                            self.set_state('idle','Goals completed');break
                        for action in batch['actions']:
                            request({'command':'preview','robot':action['robot'],'points':action['points']},self.port)
                        self.set_state('executing',batch['audit']['strategy'],[a['robot'] for a in batch['actions']])
                        done=execute_batch(batch['actions'],mission,self.port,map_revision,[],lambda:self.interrupted(revision))
                        completed+=done
                        with self.lock:
                            for r,goal in goals.items():
                                if int(r[-1]) in done and self.goals.get(r,{}).get('revision')==goal['revision']:
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
