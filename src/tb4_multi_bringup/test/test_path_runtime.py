"""Native path ownership/lease/permission and managed AMCL tests, isolated DDS."""
import os
os.environ.update(ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_SUPER_CLIENT='False',RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
for k in ('ROS_DISCOVERY_SERVER','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','CYCLONEDDS_URI','ROS_NAMESPACE'):os.environ.pop(k,None)
import json,math,threading,time,unittest
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from rclpy.action import ActionServer,CancelResponse
from geometry_msgs.msg import TransformStamped,PoseWithCovarianceStamped,Twist,TwistStamped
from std_msgs.msg import Header,String
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from nav2_msgs.action import FollowPath
from nav2_msgs.srv import IsPathValid,ManageLifecycleNodes
from std_srvs.srv import Empty
from native_process import NativeProcess

class PathRuntimeTest(unittest.TestCase):
    def setUp(self):
        rclpy.init();self.node=Node('path_test_sensors');self.executor=MultiThreadedExecutor(num_threads=6);self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start();self.apps=[]
        self.latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL);self.keep=[];self.paused=False;self.sensor_x={};self.covariance_override=None
    def tearDown(self):
        for app in self.apps:app.close()
        self.executor.shutdown(timeout_sec=3);self.thread.join(timeout=3);self.node.destroy_node();rclpy.shutdown()
    def wait(self,f,timeout=10):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if f():return
            time.sleep(.02)
        self.fail('Runtime condition timed out')
    def start(self,name,params,ns=None):
        app=NativeProcess(name,params,ns);self.apps.append(app)
        if app.port:app.wait_ready()
        return app
    def sensors(self,robot,pose_quality=True,publish_loc=True):
        prefix='/'+robot
        tf=self.node.create_publisher(TFMessage,prefix+'/tf',100)
        odom=self.node.create_publisher(Odometry,prefix+'/odom',qos_profile_sensor_data)
        scan=self.node.create_publisher(LaserScan,prefix+'/scan',qos_profile_sensor_data)
        loc=self.node.create_publisher(String,prefix+'/localization_status',self.latched)
        amcl=self.node.create_publisher(PoseWithCovarianceStamped,prefix+'/amcl_pose',self.latched)
        grid=self.node.create_publisher(OccupancyGrid,prefix+'/map',self.latched)
        m=OccupancyGrid();m.header.frame_id='map';m.info.resolution=.05;m.info.width=m.info.height=81
        m.info.origin.position.x=m.info.origin.position.y=-2.025;m.info.origin.orientation.w=1.
        m.data=[100 if x in (0,80) or y in (0,80) else 0 for y in range(81) for x in range(81)];grid.publish(m)
        self.quality=pose_quality;self.loc_ready=True
        def publish():
            if self.paused:return
            stamp=self.node.get_clock().now().to_msg()
            a=TransformStamped();a.header.stamp=stamp;a.header.frame_id='map';a.child_frame_id='odom';a.transform.rotation.w=1.
            x=self.sensor_x.get(robot,0.)
            b=TransformStamped();b.header.stamp=stamp;b.header.frame_id='odom';b.child_frame_id='base_link';b.transform.rotation.w=1.;b.transform.translation.x=x
            tf.publish(TFMessage(transforms=[a,b]));o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.child_frame_id='base_link';o.pose.pose.orientation.w=1.;o.pose.pose.position.x=x;odom.publish(o)
            s=LaserScan();s.header.stamp=stamp;s.header.frame_id='base_link';s.angle_min=-math.pi;s.angle_increment=2*math.pi/72;s.range_min=.1;s.range_max=8.
            s.ranges=[min(((2-x) if math.cos(v)>0 else (2+x))/max(abs(math.cos(v)),1e-9),2/max(abs(math.sin(v)),1e-9)) for v in (s.angle_min+i*s.angle_increment for i in range(72))];scan.publish(s)
            p=PoseWithCovarianceStamped();p.header.frame_id='map';p.header.stamp=stamp;p.pose.pose.orientation.w=1.;p.pose.pose.position.x=x
            p.pose.covariance[0]=p.pose.covariance[7]=p.pose.covariance[35]=.001 if self.quality else 1.
            if self.covariance_override is not None:
                p.pose.covariance[0]=p.pose.covariance[7]=self.covariance_override
            amcl.publish(p)
            if publish_loc:loc.publish(String(data=json.dumps({'ready':self.loc_ready})))
        timer=self.node.create_timer(.05,publish);self.keep += [tf,odom,scan,loc,amcl,grid,timer]

    def test_path_validation_lease_and_permission(self):
        self.sensors('robot1');self.valid=True;self.goals=[];self.cancels=0;group=ReentrantCallbackGroup()
        def validate(req,res):res.is_valid=self.valid;return res
        service=self.node.create_service(IsPathValid,'/robot1/is_path_valid',validate,callback_group=group)
        def execute(handle):
            self.goals.append(handle.request.path)
            deadline=time.monotonic()+8
            while not handle.is_cancel_requested and time.monotonic()<deadline:time.sleep(.02)
            if handle.is_cancel_requested:self.cancels+=1;handle.canceled()
            else:handle.abort()
            return FollowPath.Result()
        action=ActionServer(self.node,FollowPath,'/robot1/follow_path',execute_callback=execute,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=group)
        self.keep += [service,action]
        app=self.start('socket_control',{'robot_count':1,'require_localization':True})
        self.wait(lambda:app.request({'command':'snapshot'})['robots']['robot1']['localized'])
        self.wait(lambda:app.status()['robots']['robot1']['path_ready'])
        outputs=[];sub=self.node.create_subscription(TwistStamped,'/robot1/remote_cmd_vel',outputs.append,qos_profile_sensor_data)
        stamp=self.start('remote_velocity',{'mode':'stamp','require_permission':True},'/robot1')
        pub=self.node.create_publisher(Twist,'/robot1/pc_cmd_vel',10)
        def drive():m=Twist();m.linear.x=.4;m.angular.z=.8;pub.publish(m)
        timer=self.node.create_timer(.05,drive);self.keep += [sub,pub,timer]
        self.wait(lambda:len(outputs)>3);self.assertTrue(all(m.twist.linear.x==0 for m in outputs))
        payload={'command':'path','robot':'robot1','owner':'a'*32,'points':[[0,0,0],[.1,0,0]]}
        self.assertTrue(app.request(payload)['ok'])
        self.wait(lambda:len(self.goals)==1)
        self.wait(lambda:any(m.twist.linear.x>0 for m in outputs))
        self.assertTrue(all(abs(m.twist.linear.x)<=.05 and abs(m.twist.angular.z)<=.2 for m in outputs))
        self.assertFalse(app.request({'command':'lease','robot':'robot1','owner':'b'*32})['ok'])
        self.wait(lambda:self.cancels==1,timeout=5)
        self.wait(lambda:not app.status()['robots']['robot1']['busy'])
        self.assertEqual(app.status()['robots']['robot1']['last_result'],'canceled')
        time.sleep(.5);self.assertTrue(all(m.twist.linear.x==0 for m in outputs[-4:]))
        self.valid=False;self.assertTrue(app.request(payload)['ok'])
        self.wait(lambda:app.status()['robots']['robot1']['last_result']=='invalid_path')
        self.assertEqual(len(self.goals),1)
        self.valid=True;self.assertTrue(app.request(payload)['ok']);self.wait(lambda:len(self.goals)==2)
        self.loc_ready=False
        self.wait(lambda:self.cancels==2)
        self.wait(lambda:outputs[-1].twist.linear.x==0)

    def test_global_amcl_and_origin_anchor_gate_navigation(self):
        for robot,anchor in [('robot1',True),('robot2',False)]:
            self.paused=False;self.sensors(robot,pose_quality=anchor,publish_loc=False)
            commands=[];global_calls=[];anchors=[];group=ReentrantCallbackGroup()
            def manager(req,res):commands.append(req.command);res.success=True;return res
            def empty(req,res):return res
            def globalize(req,res):global_calls.append(True);return res
            self.keep += [self.node.create_service(ManageLifecycleNodes,'/'+robot+'/lifecycle_manager_localization/manage_nodes',manager,callback_group=group),
                self.node.create_service(ManageLifecycleNodes,'/'+robot+'/lifecycle_manager_navigation/manage_nodes',manager,callback_group=group),
                self.node.create_service(Empty,'/'+robot+'/reinitialize_global_localization',globalize,callback_group=group),
                self.node.create_service(Empty,'/'+robot+'/request_nomotion_update',empty,callback_group=group),
                self.node.create_subscription(PoseWithCovarianceStamped,'/'+robot+'/initialpose',anchors.append,10)]
            statuses=[];self.keep.append(self.node.create_subscription(String,'/'+robot+'/localization_status',lambda m:statuses.append(json.loads(m.data)),self.latched))
            app=self.start('localization_supervisor',{'anchor':anchor,'navigation':True},'/'+robot)
            self.wait(lambda:any(s['phase']=='localizing' for s in statuses),timeout=15)
            if anchor:
                self.wait(lambda:bool(anchors));self.assertEqual(anchors[-1].pose.pose.position.x,0.);self.assertEqual(anchors[-1].pose.pose.position.y,0.)
            else:
                self.assertTrue(global_calls);time.sleep(1);self.assertFalse(any(s['ready'] for s in statuses));self.quality=True
            self.wait(lambda:any(s['ready'] for s in statuses),timeout=12)
            self.assertGreaterEqual(commands.count(ManageLifecycleNodes.Request.STARTUP),2)
            self.paused=True
            self.wait(lambda:statuses[-1]['ready'] is False)
            self.wait(lambda:ManageLifecycleNodes.Request.PAUSE in commands)
            app.close();self.apps.remove(app)

    def test_marco_cli_through_two_fake_hardware_chains(self):
        import subprocess,sys
        from pathlib import Path
        root=Path(__file__).resolve().parents[3]
        if not (root/'MARCO/src/core/assignment.py').is_file():self.skipTest('MARCO checkout is on the PC only')
        folder=root/'log/marco_test';folder.mkdir(parents=True,exist_ok=True)
        positions={'robot1':[-1.,0.,0.],'robot2':[-1.,1.,0.]};velocities={r:[0.,0.] for r in positions};seen_goals=[];peak=[0.]
        group=ReentrantCallbackGroup();command_pubs={}
        for robot in positions:
            prefix='/'+robot
            tf=self.node.create_publisher(TFMessage,prefix+'/tf',100)
            odom=self.node.create_publisher(Odometry,prefix+'/odom',qos_profile_sensor_data)
            loc=self.node.create_publisher(String,prefix+'/localization_status',self.latched)
            command_pubs[robot]=self.node.create_publisher(Twist,prefix+'/pc_cmd_vel',10)
            def command(m,r=robot):velocities[r]=[m.linear.x,m.angular.z];peak[0]=max(peak[0],abs(m.linear.x))
            self.keep.append(self.node.create_subscription(Twist,prefix+'/cmd_vel',command,qos_profile_sensor_data))
            def publish(r=robot,t=tf,o=odom,l=loc):
                p=positions[r];v=velocities[r];p[0]+=.05*v[0]*math.cos(p[2]);p[1]+=.05*v[0]*math.sin(p[2]);p[2]+=.05*v[1]
                stamp=self.node.get_clock().now().to_msg();a=TransformStamped();a.header.stamp=stamp;a.header.frame_id='map';a.child_frame_id='odom';a.transform.rotation.w=1.
                b=TransformStamped();b.header.stamp=stamp;b.header.frame_id='odom';b.child_frame_id='base_link';b.transform.translation.x=p[0];b.transform.translation.y=p[1];b.transform.rotation.z=math.sin(p[2]/2);b.transform.rotation.w=math.cos(p[2]/2)
                t.publish(TFMessage(transforms=[a,b]));m=Odometry();m.header.stamp=stamp;m.header.frame_id='odom';m.child_frame_id='base_link';m.pose.pose.position.x=p[0];m.pose.pose.position.y=p[1];m.pose.pose.orientation.w=1.;o.publish(m);l.publish(String(data='{"ready":true}'))
            self.keep += [tf,odom,loc,self.node.create_timer(.05,publish)]
            def validate(req,res):res.is_valid=True;return res
            self.keep.append(self.node.create_service(IsPathValid,prefix+'/is_path_valid',validate,callback_group=group))
            def execute(handle,r=robot):
                points=handle.request.path.poses;seen_goals.append(r);end=time.monotonic()+15
                while time.monotonic()<end and not handle.is_cancel_requested:
                    p=positions[r];goal=points[-1].pose.position
                    if math.hypot(p[0]-goal.x,p[1]-goal.y)<.025:
                        command_pubs[r].publish(Twist());handle.succeed();return FollowPath.Result()
                    angle=math.atan2(goal.y-p[1],goal.x-p[0]);err=math.atan2(math.sin(angle-p[2]),math.cos(angle-p[2]))
                    m=Twist();m.linear.x=.05 if abs(err)<.2 else 0.;m.angular.z=max(-.15,min(.15,err));command_pubs[r].publish(m);time.sleep(.05)
                command_pubs[r].publish(Twist())
                if handle.is_cancel_requested:handle.canceled()
                else:handle.abort()
                return FollowPath.Result()
            self.keep.append(ActionServer(self.node,FollowPath,prefix+'/follow_path',execute_callback=execute,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=group))
        grid=self.node.create_publisher(OccupancyGrid,'/robot1/map',self.latched)
        m=OccupancyGrid();m.header.frame_id='map';m.info.resolution=.05;m.info.width=m.info.height=81;m.info.origin.position.x=m.info.origin.position.y=-2.025;m.info.origin.orientation.w=1.
        m.data=[100 if x in (0,80) or y in (0,80) else 0 for y in range(81) for x in range(81)];grid.publish(m);self.keep.append(grid)
        app=self.start('socket_control',{'robot_count':2,'require_localization':True})
        for r in positions:
            self.start('remote_velocity',{'mode':'stamp','require_permission':True},'/'+r)
            self.start('remote_velocity',{'mode':'guard'},'/'+r)
        self.wait(lambda:all(s['localized'] for s in app.request({'command':'snapshot'})['robots'].values()))
        self.wait(lambda:all(s['path_ready'] for s in app.status()['robots'].values()))
        mission={'tasks':[{'id':1,'goal':[-.8,0,0],'candidates':['robot1']},{'id':2,'goal':[-.8,1,0],'candidates':['robot2']},{'id':3,'goal':[-.6,0,0],'candidates':['robot1'],'depends_on':[1,2]}]}
        filename=folder/'e2e-mission.json';filename.write_text(json.dumps(mission))
        common=[str(filename),'--root',str(root/'MARCO'),'--port',str(app.port)]
        preview=subprocess.run([sys.executable,'-B','-m','tb4_marco_bridge.cli','plan',*common,'--out',str(folder/'e2e-preview.json')],capture_output=True,text=True,timeout=12)
        self.assertEqual(preview.returncode,0,preview.stderr);self.assertEqual(seen_goals,[])
        run=subprocess.run([sys.executable,'-B','-m','tb4_marco_bridge.cli','run',*common,'--execute','--out',str(folder/'e2e-result.json')],capture_output=True,text=True,timeout=40)
        self.assertEqual(run.returncode,0,run.stderr)
        result=json.loads((folder/'e2e-result.json').read_text());self.assertEqual(result['result'],'succeeded');self.assertEqual(set(result['completed_tasks']),{1,2,3})
        self.assertGreater(peak[0],.01);self.assertLessEqual(peak[0],.05)
        events=result['events'];third=next(i for i,e in enumerate(events) if e.get('event')=='dispatch' and e['action']['task']==3)
        self.assertEqual({e['task'] for e in events[:third] if e.get('event')=='completed'},{1,2})
        self.wait(lambda:all(abs(v[0])<.001 for v in velocities.values()))

    def test_permission_reordering_and_stale_headers(self):
        outputs=[]
        self.keep.append(self.node.create_subscription(TwistStamped,'/robot1/remote_cmd_vel',outputs.append,qos_profile_sensor_data))
        permissions=self.node.create_publisher(Header,'/robot1/command_permission',qos_profile_sensor_data)
        commands=self.node.create_publisher(Twist,'/robot1/pc_cmd_vel',10)
        app=self.start('remote_velocity',{'mode':'stamp','require_permission':True},'/robot1')
        def command():m=Twist();m.linear.x=.3;commands.publish(m)
        self.keep += [permissions,commands,self.node.create_timer(.05,command)]
        self.wait(lambda:permissions.get_subscription_count()>0 and len(outputs)>2)
        first=Header();first.stamp=self.node.get_clock().now().to_msg();first.frame_id='allow'
        for _ in range(3):permissions.publish(first);time.sleep(.02)
        self.wait(lambda:outputs[-1].twist.linear.x>0)
        stop=Header();stop.stamp=self.node.get_clock().now().to_msg();stop.frame_id='stop';permissions.publish(stop)
        self.wait(lambda:outputs[-1].twist.linear.x==0)
        permissions.publish(first);time.sleep(.15)
        self.assertTrue(all(m.twist.linear.x==0 for m in outputs[-3:]))
        future=Header();future.stamp=self.node.get_clock().now().to_msg();future.stamp.sec+=5;future.frame_id='allow';permissions.publish(future)
        time.sleep(.15);self.assertTrue(all(m.twist.linear.x==0 for m in outputs[-3:]))

    def test_neighbor_hint_is_a_prior_not_a_forced_transform(self):
        self.sensors('robot2',publish_loc=False);anchors=[];statuses=[];global_calls=[];group=ReentrantCallbackGroup()
        def manager(req,res):res.success=True;return res
        def empty(req,res):return res
        def globalize(req,res):global_calls.append(True);return res
        self.keep += [self.node.create_service(ManageLifecycleNodes,'/robot2/lifecycle_manager_localization/manage_nodes',manager,callback_group=group),
            self.node.create_service(Empty,'/robot2/reinitialize_global_localization',globalize,callback_group=group),
            self.node.create_service(Empty,'/robot2/request_nomotion_update',empty,callback_group=group),
            self.node.create_subscription(PoseWithCovarianceStamped,'/robot2/initialpose',anchors.append,10),
            self.node.create_subscription(String,'/robot2/localization_status',lambda m:statuses.append(json.loads(m.data)),self.latched)]
        app=self.start('localization_supervisor',{'anchor':False,'navigation':False,'hint':True,'hint_x':0.,'hint_y':.5,'hint_yaw':0.,'hint_std_xy':.2,'hint_std_yaw':.2},'/robot2')
        self.wait(lambda:bool(anchors),timeout=15)
        self.assertAlmostEqual(anchors[-1].pose.pose.position.y,.5)
        self.assertAlmostEqual(anchors[-1].pose.covariance[0],.04)
        self.assertEqual(global_calls,[])
        self.wait(lambda:any(s.get('reason')=='outside_initial_hint_region' for s in statuses))
        self.assertFalse(statuses[-1]['ready'])

    def test_failed_navigation_start_does_not_reset_valid_amcl(self):
        self.sensors('robot1',publish_loc=False);local_commands=[];statuses=[];group=ReentrantCallbackGroup()
        def local(req,res):local_commands.append(req.command);res.success=True;return res
        def nav(req,res):res.success=False;return res
        def empty(req,res):return res
        self.keep += [self.node.create_service(ManageLifecycleNodes,'/robot1/lifecycle_manager_localization/manage_nodes',local,callback_group=group),
            self.node.create_service(ManageLifecycleNodes,'/robot1/lifecycle_manager_navigation/manage_nodes',nav,callback_group=group),
            self.node.create_service(Empty,'/robot1/request_nomotion_update',empty,callback_group=group),
            self.node.create_subscription(PoseWithCovarianceStamped,'/robot1/initialpose',lambda m:None,10),
            self.node.create_subscription(String,'/robot1/localization_status',lambda m:statuses.append(json.loads(m.data)),self.latched)]
        app=self.start('localization_supervisor',{'anchor':True,'navigation':True},'/robot1')
        self.wait(lambda:any(s['phase']=='nav_lifecycle_failed' for s in statuses),timeout=18)
        self.assertFalse(statuses[-1]['ready'])
        self.assertEqual(local_commands,[ManageLifecycleNodes.Request.STARTUP])

    def test_converged_robot_can_navigate_beyond_initial_hint(self):
        self.sensors('robot2',publish_loc=False);statuses=[];group=ReentrantCallbackGroup()
        def local(req,res):res.success=True;return res
        def empty(req,res):return res
        self.keep += [self.node.create_service(ManageLifecycleNodes,'/robot2/lifecycle_manager_localization/manage_nodes',local,callback_group=group),
            self.node.create_service(Empty,'/robot2/request_nomotion_update',empty,callback_group=group),
            self.node.create_subscription(PoseWithCovarianceStamped,'/robot2/initialpose',lambda m:None,10),
            self.node.create_subscription(String,'/robot2/localization_status',lambda m:statuses.append(json.loads(m.data)),self.latched)]
        self.start('localization_supervisor',{'anchor':False,'navigation':False,'hint':True,'hint_x':0.,'hint_y':0.,'hint_std_xy':.2},'/robot2')
        self.wait(lambda:statuses and statuses[-1]['ready'],timeout=18)
        self.sensor_x['robot2']=.55
        self.wait(lambda:statuses[-1].get('pose',[0])[0]>.5)
        self.wait(lambda:statuses[-1]['ready'])
        self.assertNotEqual(statuses[-1].get('reason'),'outside_initial_hint_region')
        self.covariance_override=-1e-14
        self.wait(lambda:statuses[-1].get('covariance_xyyaw',[1])[0]<0 and statuses[-1]['ready'])
        self.covariance_override=-.001
        self.wait(lambda:statuses[-1].get('covariance_xyyaw',[1])[0]<-.0001 and not statuses[-1]['ready'])

if __name__=='__main__':unittest.main()
