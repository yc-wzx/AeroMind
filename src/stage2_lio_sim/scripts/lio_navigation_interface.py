#!/usr/bin/env python3
"""Opt-in LIO input; reuse existing hold clocks and unchanged control/guards."""
import sys,math,time
from pathlib import Path
from ament_index_python.packages import get_package_prefix
sys.path.insert(0,str(Path(get_package_prefix('uav_planning'))/'lib/uav_planning'))
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from gazebo_navigation_interface import GazeboNavigationInterface,stamp_seconds
from stage2_navigation_interface import Stage2NavigationInterface
from stage2_localized_interface import LocalizedInterface

class LioNavigationInterface(LocalizedInterface):
    def __init__(self):
        self.last_match_sim=self.last_match_steady=self.last_scan_stamp=None
        self.scan_input_fault=False;self._localization_stage_hold=None;self._localization_update_fresh=None
        self.last_stage2_stamp=None;self.last_lio_received=None
        GazeboNavigationInterface.__init__(self)
        if self.imperfect_sensors or self.resolve_topic_name('/gazebo/odometry')!='/localization/lio_navigation_odometry':
            raise RuntimeError('LIO navigation requires non-GT remap and ideal drive')
        self.match_timeout=.5
        self.localization_diag=self.create_publisher(String,'/localization/diagnostics',100)
        self.create_subscription(PointCloud2,'/lio/cloud_registered',self.lio_correction,10)
        self.get_logger().info('STAGE2 LIO input only; no GT/no artificial odometry/no 2D pose correction')

    def gazebo_odom(self,msg):
        old=self.last_stage2_stamp
        Stage2NavigationInterface.gazebo_odom(self,msg)
        if self.last_stage2_stamp!=old:self.last_lio_received=time.monotonic()

    def lio_correction(self,msg):
        t=stamp_seconds(msg.header.stamp);age=self.get_clock().now().nanoseconds*1e-9-t
        if msg.header.frame_id=='lio_odom' and msg.width*msg.height>=24 and -.05<=age<=.2:
            if self.last_match_sim is None or t>self.last_match_sim:
                self.last_match_sim=t;self.last_match_steady=time.monotonic()

    def localization_fresh(self):
        return (self.last_lio_received is not None and time.monotonic()-self.last_lio_received<=.2
            and self.last_match_sim is not None and self.last_match_steady is not None
            and 0<=self.get_clock().now().nanoseconds*1e-9-self.last_match_sim<=.5
            and time.monotonic()-self.last_match_steady<=.5)

    def scan_callback(self,scan):
        # Existing 2D scan remains solely for obstacle sensing, not pose correction.
        GazeboNavigationInterface.scan_callback(self,scan)

def main():
    rclpy.init();node=LioNavigationInterface()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
