#!/usr/bin/env python3
"""Simulation-only stand-in for MCU radar axes -> physical body wheel axes.

Only this electrical half sees the simulator joint. It publishes no localization
or synthetic encoder feedback to the host. It is NOT a real serial driver.
"""
from runtime_support import load_existing
load_existing()
import math,time,json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist,TwistStamped
from std_msgs.msg import String
from gimbal_contract import JointBuffer,radar_to_body
from lio_contract import stamp_s

class RadarSimElectrical(Node):
    def __init__(self):
        super().__init__('radar_sim_electrical')
        self.joints=JointBuffer();self.command=None;self.received=None
        self.pub=self.create_publisher(Twist,'/model/omni_robot/cmd_vel',10)
        self.diag=self.create_publisher(String,'/simulation/radar_electrical_diagnostics',100)
        self.create_subscription(JointState,'/gimbal/joint_states',self.joint,1)
        self.create_subscription(TwistStamped,'/navigation/radar_cmd_vel',self.radar_command,1)
        self.create_timer(.02,self.update)

    def joint(self,msg):
        try:
            i=msg.name.index('gimbal_yaw')
            self.joints.add(stamp_s(msg.header.stamp),msg.position[i],msg.velocity[i],time.monotonic(),self.get_clock().now().nanoseconds*1e-9)
        except (ValueError,IndexError,TypeError):self.joints.fault=True

    def radar_command(self,msg):
        self.command=msg;self.received=time.monotonic();self.update()

    def update(self):
        now=self.get_clock().now().nanoseconds*1e-9;steady=time.monotonic();out=Twist();action='allowed'
        try:
            m=self.command
            if m is None or self.received is None or not 0<=steady-self.received<=.2:raise ValueError('radar_command_missing_or_stale')
            if m.header.frame_id!='radar_control' or not -.01<=now-stamp_s(m.header.stamp)<=.2:raise ValueError('radar_command_wrong_frame_or_time')
            v=m.twist
            if not all(math.isfinite(x) for x in [v.linear.x,v.linear.y,v.angular.z]):raise ValueError('nonfinite_radar_command')
            if abs(v.angular.z)>1e-9:raise ValueError('chassis_yaw_request_rejected')
            if v.linear.x==0. and v.linear.y==0.:action='explicit_stop'
            else:
                _,angle,_=self.joints.latest(now,steady)
                out.linear.x,out.linear.y=radar_to_body(v.linear.x,v.linear.y,angle)
        except ValueError as e:action=str(e)
        out.angular.z=0.;self.pub.publish(out)
        self.diag.publish(String(data=json.dumps(dict(sim_s=now,action=action,final_body=[out.linear.x,out.linear.y,0.],
            scope='simulation MCU only; measured joint never sent to localization/host'),allow_nan=False)))

def main():
    rclpy.init();node=RadarSimElectrical()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
