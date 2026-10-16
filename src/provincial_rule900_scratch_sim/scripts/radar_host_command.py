#!/usr/bin/env python3
"""Host: guarded body-centre velocity -> radar-axis command for electrical MCU.

Never subscribes to Gazebo truth or an encoder. No wheel-speed conversion here.
"""
from runtime_support import load_existing
load_existing()
import json,math,time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from gimbal_contract import body_to_radar
from radar_fixed_contract import configured_mount,RadarHeadingBuffer

class RadarHostCommand(Node):
    def __init__(self):
        super().__init__('radar_host_command')
        self.mount=configured_mount(self);self.heading=RadarHeadingBuffer(self.mount.initial_base[3])
        self.command=None;self.received=None
        self.pub=self.create_publisher(TwistStamped,'/navigation/radar_cmd_vel',10)
        self.diag=self.create_publisher(String,'/navigation/radar_command_diagnostics',100)
        self.create_subscription(Odometry,'/localization/radar_pose',self.radar_pose,1)
        self.create_subscription(Twist,'/navigation/guarded_body_cmd_vel',self.body_command,1)
        self.create_timer(.02,self.update)

    def radar_pose(self,msg):
        try:self.heading.add_pose(msg,self.get_clock().now().nanoseconds*1e-9,time.monotonic())
        except (ValueError,TypeError,AttributeError):pass

    def body_command(self,msg):
        self.command=(msg.linear.x,msg.linear.y,msg.angular.z);self.received=time.monotonic()
        # In particular, do not wait one timer cycle to forward a stop.
        if (not all(math.isfinite(x) for x in self.command)
                or abs(self.command[2])>1e-9
                or (self.command[0]==0. and self.command[1]==0.)):
            self.update()
        # Nonzero commands are emitted by the fixed20ms timer, not once for
        # every upstream callback. Invalid/zero input still stops immediately.

    def update(self):
        now=self.get_clock().now().nanoseconds*1e-9;steady=time.monotonic()
        out=TwistStamped();out.header.stamp=self.get_clock().now().to_msg();out.header.frame_id='radar_control'
        action='allowed';angle=None
        try:
            if self.command is None or self.received is None or not 0<=steady-self.received<=.2:raise ValueError('command_missing_or_stale')
            if not all(math.isfinite(x) for x in self.command):raise ValueError('nonfinite_command')
            if abs(self.command[2])>1e-9:raise ValueError('chassis_yaw_request_rejected')
            if self.command[0]==0. and self.command[1]==0.:action='explicit_stop'
            else:
                _,angle,_=self.heading.latest(now,steady)
                out.twist.linear.x,out.twist.linear.y=body_to_radar(*self.command[:2],angle)
        except ValueError as e:action=str(e)
        out.twist.angular.z=0.;self.pub.publish(out)
        self.diag.publish(String(data=json.dumps(dict(sim_s=now,action=action,
            derived_relative_yaw=angle,radar_command=[out.twist.linear.x,out.twist.linear.y,0.],
            source='LIO radar pose; no encoder/GT; electrical wheel conversion external',
            velocity_origin='physical chassis centre, axes aligned to radar'),allow_nan=False)))

def main():
    rclpy.init();node=RadarHostCommand()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():
            stop=TwistStamped();stop.header.stamp=node.get_clock().now().to_msg();stop.header.frame_id='radar_control';node.pub.publish(stop)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
