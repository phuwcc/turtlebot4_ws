"""RViz goal staging: two goals, explicit preview/execute, guarded MARCO CLI."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import socket
import time
import sys
from datetime import datetime
import rclpy
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy


class GoalDraft:
    def __init__(self):
        self.goals = {}

    def set(self, robot, message):
        p, q = message.pose.position, message.pose.orientation
        values = [p.x, p.y, q.x, q.y, q.z, q.w]
        if message.header.frame_id != 'map' or not all(math.isfinite(v) for v in values):
            raise ValueError('Goal must be finite and in map frame')
        if abs(sum(v*v for v in values[2:])-1) > .01:
            raise ValueError('Goal quaternion must be normalized')
        self.goals[robot] = [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]

    def mission(self):
        if set(self.goals) != {'robot1', 'robot2'}:
            raise ValueError('Set both robot1 and robot2 goals first')
        return dict(priority_robot='robot1', max_localization_replans=2, speed_mps=.05, robot_radius=.175,
                    safety_margin=.05, tracking_tolerance=.04, action_timeout_s=240,
                    tasks=[dict(id=i, candidates=[r], goal=self.goals[r], depends_on=[])
                           for i,r in enumerate(('robot1','robot2'),1)])


class RvizGoals:
    def __init__(self, root):
        self.root=Path(root); self.node=rclpy.create_node('marco_rviz_goals')
        self.draft=GoalDraft();self.child=None;self.stream=None;self.preview_ok=False
        self.draft_path=self.root/'log/rviz_marco/draft.json'
        if self.draft_path.exists():
            try:self.draft.goals=json.loads(self.draft_path.read_text())
            except (ValueError,OSError):pass
        self.runtime_text='';self.status_at=0.;self.last_results=None
        self.directory=None;self.mode=None;self.text='Set goal R1, then goal R2; Preview before Execute'
        self.pub=self.node.create_publisher(MarkerArray,'/tb4_view/marco_goals',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.subs=[]
        for robot in ('robot1','robot2'):
            self.subs.append(self.node.create_subscription(PoseStamped,'/tb4_view/marco/'+robot+'/goal',lambda m,r=robot:self.goal(r,m),10))
        for mode in ('preview','execute','cancel'):
            self.subs.append(self.node.create_subscription(PointStamped,'/tb4_view/marco/'+mode,lambda m,mode=mode:self.command(mode),10))
        self.node.create_timer(.25,self.poll)
        self.markers()

    def goal(self,robot,message):
        if self.child is not None:
            self.text='Wait for current operation or Cancel';self.markers();return
        try:
            self.draft.set(robot,message);self.preview_ok=False
            self.draft_path.parent.mkdir(parents=True,exist_ok=True)
            self.draft_path.write_text(json.dumps(self.draft.goals))
            self.text=robot+' goal staged; set both goals and click Preview'
        except ValueError as e:self.text=str(e)
        self.markers()

    def command(self,mode):
        if mode=='cancel':
            if self.child is not None:os.killpg(self.child.pid,signal.SIGINT)
            self.preview_ok=False;self.text='Cancel requested';self.markers();return
        if self.child is not None:return
        try:
            mission=self.draft.mission()
            if mode=='execute' and not self.preview_ok:raise ValueError('Click Preview and wait for success before Execute')
            if mode=='preview':
                self.directory=self.root/'log/rviz_marco'/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
                self.directory.mkdir(parents=True)
                (self.directory/'mission.json').write_text(json.dumps(mission,indent=2))
            command=['bash',str(self.root/'run.bash'),'mission',str(self.directory/'mission.json')]
            if mode=='execute':command.append('--execute')
            self.stream=(self.directory/(mode+'.log')).open('w')
            self.child=subprocess.Popen(command,cwd=self.root,stdout=self.stream,stderr=subprocess.STDOUT,start_new_session=True)
            self.mode=mode;self.text=mode+' running; robot1 priority';self.markers()
        except (ValueError,OSError) as e:self.text=str(e);self.markers()

    def runtime_status(self):
        if time.monotonic()-self.status_at < 1:return
        self.status_at=time.monotonic()
        try:
            with socket.create_connection(('127.0.0.1',int(os.environ.get('TB4_SOCKET_PORT','8765'))),timeout=.2) as c:
                c.sendall(b'{"command":"status"}\n')
                with c.makefile('rb') as f:status=json.loads(f.readline(65536))['robots']
            results={r:v.get('last_result') for r,v in status.items()}
            self.runtime_text='\n'+'; '.join(r+': '+('RUNNING' if v['busy'] else (v.get('last_result') or 'NOT DISPATCHED')) for r,v in status.items())
            if self.child is None and any(v=='canceled' for v in results.values()):
                if self.last_results is None or results!=self.last_results:
                    self.text='Execution stopped/canceled; preview success is not task completion'
                    self.preview_ok=False
            self.last_results=results
            self.markers()
        except (OSError,ValueError,KeyError):pass

    def poll(self):
        self.runtime_status()
        if self.child is None or self.child.poll() is None:return
        code=self.child.returncode;self.stream.close();self.child=None
        self.preview_ok=self.mode=='preview' and code==0
        self.text=self.mode+(' succeeded' if code==0 else ' failed')
        if code != 0:
            lines=(self.directory/(self.mode+'.log')).read_text(errors='replace').splitlines()
            reasons=[line for line in lines if line.startswith(('RuntimeError:', 'ValueError:', 'TimeoutError:'))]
            if reasons:self.text+=': '+reasons[-1][:220]
            else:self.text+='; inspect '+str(self.directory)
        self.node.get_logger().info(self.text);self.markers()

    def markers(self):
        array=MarkerArray()
        clear=Marker();clear.action=Marker.DELETEALL;array.markers.append(clear)
        for i,(r,p) in enumerate(self.draft.goals.items()):
            m=Marker();m.header.frame_id='map';m.ns='marco_goals';m.id=i;m.type=Marker.ARROW;m.action=Marker.ADD
            m.pose.position.x=p[0];m.pose.position.y=p[1];m.pose.orientation.z=math.sin(p[2]/2);m.pose.orientation.w=math.cos(p[2]/2)
            m.scale.x=.35;m.scale.y=.07;m.scale.z=.07;m.color.a=1.;m.color.r=float(r=='robot1');m.color.g=float(r=='robot2');array.markers.append(m)
        m=Marker();m.header.frame_id='map';m.ns='marco_goals';m.id=20;m.type=Marker.TEXT_VIEW_FACING;m.pose.orientation.w=1.;m.pose.position.z=.5
        if self.draft.goals:
            position=next(iter(self.draft.goals.values()));m.pose.position.x=position[0];m.pose.position.y=position[1]+.4
        m.scale.z=.12;m.color.a=1.;m.color.r=m.color.g=1.;m.text=self.text+self.runtime_text;array.markers.append(m);self.pub.publish(array)

    def close(self):
        if self.child is not None:
            os.killpg(self.child.pid,signal.SIGINT)
            try:self.child.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(self.child.pid,signal.SIGTERM);self.child.wait(timeout=5)
        if self.stream:self.stream.close()
        self.node.destroy_node()


def main():
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    rclpy.init();app=RvizGoals(os.environ['TB4_REPO_ROOT'])
    try:rclpy.spin(app.node)
    except KeyboardInterrupt:pass
    finally:app.close();rclpy.shutdown()

if __name__=='__main__':main()
