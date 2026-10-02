#!/usr/bin/env python3
"""Record before world; stationary LIO gate. Never publishes robot commands."""
import sys,json,time,math,argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'tools'))
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,Imu
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from provincial_native_trace import NativeTrace
from sensor_trace import SensorTrace

class Observer(Node):
    def __init__(self,out):
        super().__init__('lio_stationary_observer');self.out=out
        self.native=NativeTrace(out/'stationary.native.jsonl');self.sensor=SensorTrace(out/'stationary.sensors.jsonl')
        self.started=time.monotonic();self.latest={};self.samples=[];self.gt=None;self.lio=None
        self.create_subscription(Clock,'/clock',lambda m:self.record('/clock',m),100)
        for topic in ['/gazebo/odometry','/lio/odometry','/localization/lio_navigation_odometry']:
            self.create_subscription(Odometry,topic,lambda m,t=topic:self.record(t,m),100)
        for topic in ['/lio/sensor_diagnostics','/lio/adapter_diagnostics','/simulation/imu_kinematics']:
            self.create_subscription(String,topic,lambda m,t=topic:self.record(t,m),100)
        for topic in ['/simulation/lidar3d/points','/lio/lidar','/lio/cloud_registered']:
            self.create_subscription(PointCloud2,topic,lambda m,t=topic:self.sensor_record(t,m),qos_profile_sensor_data)
        for topic in ['/simulation/imu3d','/lio/imu']:
            self.create_subscription(Imu,topic,lambda m,t=topic:self.sensor_record(t,m),qos_profile_sensor_data)
        (out/'recorder_ready.json').write_text(json.dumps({'monotonic_ns':time.monotonic_ns(),'commands_published':0}))
    def sensor_record(self,topic,msg):
        self.sensor.sim_s=self.native.sim_s;self.sensor.record(topic,msg);self.latest[topic]=time.monotonic()
    def record(self,topic,msg):
        self.native.record(topic,msg);self.latest[topic]=time.monotonic()
        if topic=='/gazebo/odometry':self.gt=msg
        if topic=='/localization/lio_navigation_odometry':
            self.lio=msg
            if self.gt is not None:
                p=msg.pose.pose.position;g=self.gt.pose.pose.position
                q=msg.pose.pose.orientation;a=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                v=msg.twist.twist
                self.samples.append({'stamp_s':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                    'xy':[p.x,p.y],'z':p.z,'yaw':a,'xy_error':math.hypot(p.x-g.x,p.y-g.y),
                    'speed':math.hypot(v.linear.x,v.linear.y),'yaw_speed':abs(v.angular.z),
                    'dt_gt':abs((msg.header.stamp.sec-self.gt.header.stamp.sec)+(msg.header.stamp.nanosec-self.gt.header.stamp.nanosec)*1e-9)})
    def run(self):
        while time.monotonic()-self.started<45 and rclpy.ok():
            rclpy.spin_once(self,timeout_sec=.01)
            if self.native.sim_s is not None and self.native.sim_s>=20:return

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    rclpy.init();n=Observer(a.output)
    try:n.run()
    finally:
        h=n.native.close();s=n.sensor.close()
        chosen=[x for x in n.samples if x['stamp_s']>=10]
        report={'pass':bool(chosen) and len(chosen)>=300 and all(x['xy_error']<=.02 and x['speed']<=.02 and x['yaw_speed']<=.03 and x['dt_gt']<=.025 for x in chosen) and h['pass'] and s['pass'],
            'samples':len(chosen),'max_xy_error':max((x['xy_error'] for x in chosen),default=None),
            'max_speed':max((x['speed'] for x in chosen),default=None),'samples_summary':chosen,
            'native_health':h,'sensor_health':s,'no_navigation_goal_or_velocity_published':True}
        (a.output/'stationary_summary.json').write_text(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()
    raise SystemExit(0 if report['pass'] else 2)
