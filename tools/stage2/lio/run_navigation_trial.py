#!/usr/bin/env python3
"""Single external goal after sensor/LIO readiness; existing production runner."""
import sys,json,time,math,argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'));sys.path.insert(0,str(ROOT/'tools/stage2'))
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,Imu,LaserScan
from std_msgs.msg import String
from run_provincial_low_speed_validation import Runner,yaw_of
from run_provincial_forward_integration import offline_leg,MAP,FIELD
from summarize_provincial_stage15_terminal_v3 import sdf_walls
from provincial_safety_geometry import body_wall_gap
from sensor_trace import SensorTrace

class LioRunner(Runner):
    def __init__(self,out,phase):
        self.lio_latest={};self.out=out;self.phase=phase
        self.sensor_trace=SensorTrace(out/('lio_'+phase+'_01.sensors.jsonl'))
        super().__init__('start' if phase=='short' else 'full','lio_'+phase+'_01',240,'hold-start',out,
                         terminal_validation=True,raw_every_message=True)
        for topic in ['/lio/odometry','/localization/lio_navigation_odometry']:
            self.create_subscription(Odometry,topic,lambda m,t=topic:self.record_lio(t,m),100)
        for topic in ['/lio/adapter_diagnostics','/lio/sensor_diagnostics','/localization/diagnostics','/simulation/imu_kinematics']:
            self.create_subscription(String,topic,lambda m,t=topic:self.trace(t,m),100)
        self.create_subscription(LaserScan,'/scan',lambda m:self.trace('/scan',m),qos_profile_sensor_data)
        # Observer buffering only. Preserve high-rate messages during bounded
        # offline/graph preflight work; never change publisher sensor rates.
        capture_qos=QoSProfile(depth=2048,reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic in ['/simulation/lidar3d/points','/lio/lidar','/lio/cloud_registered']:
            self.create_subscription(PointCloud2,topic,lambda m,t=topic:self.record_sensor(t,m),capture_qos)
        for topic in ['/simulation/imu3d','/lio/imu']:
            self.create_subscription(Imu,topic,lambda m,t=topic:self.record_sensor(t,m),capture_qos)
        (out/'recorder_ready.json').write_text(json.dumps({'monotonic_ns':time.monotonic_ns(),'before_world':True}))

    def record_lio(self,topic,msg):
        self.trace(topic,msg);self.lio_latest[topic]=(time.monotonic(),msg)
    def record_sensor(self,topic,msg):
        self.sensor_trace.sim_s=self.native_trace.sim_s;self.sensor_trace.record(topic,msg)
        self.lio_latest[topic]=(time.monotonic(),msg)

    def wait_ready(self,timeout_s=60):
        end=time.monotonic()+timeout_s;next_query=0;since=None
        while time.monotonic()<end and rclpy.ok():
            rclpy.spin_once(self,timeout_sec=.005);now=time.monotonic()
            if now<next_query:continue
            next_query=now+.1
            mandatory=['/lio/odometry','/localization/lio_navigation_odometry','/simulation/lidar3d/points',
                '/lio/lidar','/lio/cloud_registered','/simulation/imu3d','/lio/imu']
            sensors=all(t in self.lio_latest and now-self.lio_latest[t][0]<(.2 if 'imu' in t or 'odometry' in t else .2) for t in mandatory)
            core=self.gt is not None and self.odom is not None and self.final_command is not None
            core=core and now-self.gt_received_steady<=.2 and now-self.odom_received_steady<=.5 and now-self.final_command_received_steady<=.2
            valid=(sensors and core and (self.out/'parent_preflight_ready.json').exists()
                and self.native_trace.sim_s is not None and self.native_trace.sim_s>=10 and self.goal_receiver_ready(now)
                and math.dist([self.gt.pose.pose.position.x,self.gt.pose.pose.position.y],
                    [self.odom.pose.pose.position.x,self.odom.pose.pose.position.y])<=.02)
            since=(since or now) if valid else None
            if since is not None and now-since>=1:
                p=self.gt.pose.pose.position;reference=json.loads(FIELD.read_text())['reference_route']
                preflight=offline_leg((p.x,p.y),self.goal,self.grid,sdf_walls(),reference)
                preflight['measured_start_gt']=[p.x,p.y,yaw_of(self.gt.pose.pose.orientation)]
                (self.out/(self.name+'.offline_preflight.json')).write_text(json.dumps(preflight,indent=2))
                graph={}
                for topic in ['/gazebo/odometry','/lio/lidar','/lio/imu','/lio/odometry','/lio/cloud_registered','/localization/lio_navigation_odometry']:
                    graph[topic]={'pub':[dict(name=e.node_name,gid=list(e.endpoint_gid),qos=str(e.qos_profile)) for e in self.get_publishers_info_by_topic(topic)],
                        'sub':[dict(name=e.node_name,gid=list(e.endpoint_gid),qos=str(e.qos_profile)) for e in self.get_subscriptions_info_by_topic(topic)]}
                truth_names=[e['name'] for e in graph['/gazebo/odometry']['sub']]
                if any(n in truth_names for n in ['lio_mapping','lio_sensor_adapter','lio_odometry_adapter','gazebo_navigation_interface']):
                    raise RuntimeError('GT information input not isolated')
                expected={'/lio/lidar':'lio_sensor_adapter','/lio/imu':'lio_sensor_adapter','/lio/odometry':'lio_mapping',
                    '/lio/cloud_registered':'lio_mapping','/localization/lio_navigation_odometry':'lio_odometry_adapter'}
                if any([e['name'] for e in graph[t]['pub']]!=[n] for t,n in expected.items()):
                    raise RuntimeError('LIO source publisher identity mismatch')
                (self.out/'lio_goal_preflight.json').write_text(json.dumps({'graph':graph,'monotonic_ns':time.monotonic_ns(),
                    'stable_s':now-since,'sensor_latest_stamps':{t:m.header.stamp.sec+m.header.stamp.nanosec*1e-9 for t,(_,m) in self.lio_latest.items()}},indent=2))
                return True
        return False

    def gt_cb(self,msg):
        super().gt_cb(msg)
        if self.active:
            p=msg.pose.pose;gap=body_wall_gap(p.position.x,p.position.y,yaw_of(p.orientation),self.walls)
            if gap<.08:raise RuntimeError('TRUTH_CLEARANCE_STOP '+str(gap))

    def run(self):
        try:return super().run()
        except RuntimeError as error:
            self.active=False;(self.out/'trial_exception.json').write_text(json.dumps({'error':str(error)}))
            return self.save('trial_exception')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--phase',choices=['short','full'],required=True);a=p.parse_args()
    rclpy.init();n=LioRunner(a.output,a.phase)
    try:result=n.run()
    except KeyboardInterrupt:result=n.save('experiment_canceled_before_or_after_goal')
    finally:
        n.sensor_trace.close()
        if n.native_trace and not n.native_trace.closed:n.native_trace.close()
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    raise SystemExit(0 if result=='terminal_pass' and n.terminal_evidence_pass else 2)
