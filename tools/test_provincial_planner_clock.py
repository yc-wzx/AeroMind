"""Actual EGO binary clock contract on isolated DDS domain; no Gazebo/drive."""
import json,math,os,signal,subprocess,time,unittest
from pathlib import Path
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from traj_utils.msg import Bspline
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/uav_planning/scripts'))
from grid_route import GridRoute

class PlannerClockTest(unittest.TestCase):
    def test_actual_binary_uses_published_simulation_clock(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'),'188');self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'),'1')
        rclpy.init();node=Node('stage15_clock_contract_probe');clock_pub=node.create_publisher(Clock,'/clock',100);odom_pub=node.create_publisher(Odometry,'/ground/odometry',100)
        from sensor_msgs.msg import PointCloud2
        cloud_pub=node.create_publisher(PointCloud2,'/cloud_registered_2d',10);goal_pub=node.create_publisher(PoseStamped,'/move_base_simple/goal',10);messages=[]
        sub=node.create_subscription(Bspline,'/planning/bspline',lambda m:messages.append(m),10)
        out=ROOT/'tools/results/provincial_stage15_completion_20261001';log=(out/'isolated_cpp_clock.log').open('x')
        process=subprocess.Popen([str(ROOT/'install/ego_planner/lib/ego_planner/ego_planner_node'),'--ros-args','--params-file',str(ROOT/'src/uav_bringup/config/ego_provincial_2025_provisional.yaml'),'-p','use_sim_time:=true','-r','odom_world:=/ground/odometry','-r','grid_map/odom:=/ground/odometry','-r','grid_map/cloud:=/cloud_registered_2d'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        grid=GridRoute(ROOT/'src/uav_bringup/maps/provincial_2025_provisional.pgm',clearance=.4);points=grid.boundary_points();sim=20.;goal_sent=False;started=time.monotonic();last_tick=0
        try:
            while time.monotonic()-started<15 and not messages:
                now=time.monotonic()
                if now-last_tick>=.02:
                    sim+=.02;last_tick=now;st=Clock();ns=round(sim*1e9);st.clock.sec,st.clock.nanosec=divmod(ns,10**9);clock_pub.publish(st)
                    odom=Odometry();odom.header.stamp=st.clock;odom.header.frame_id='odom';odom.child_frame_id='base_link';odom.pose.pose.position.x=4.7;odom.pose.pose.position.y=.5;odom.pose.pose.orientation.z=math.sin(math.pi/4);odom.pose.pose.orientation.w=math.cos(math.pi/4);odom_pub.publish(odom)
                    h=Header();h.stamp=st.clock;h.frame_id='odom';cloud_pub.publish(create_cloud_xyz32(h,points))
                if not goal_sent and now-started>2 and goal_pub.get_subscription_count()>0:
                    goal=PoseStamped();goal.header.frame_id='odom';goal.pose.position.x=4.7;goal.pose.position.y=1.8;goal.pose.orientation.w=1.;goal_pub.publish(goal);goal_sent=True
                rclpy.spin_once(node,timeout_sec=.005)
                self.assertIsNone(process.poll(),'actual planner exited')
            self.assertTrue(goal_sent);self.assertTrue(messages,'actual planner did not publish a spline')
            stamps=[m.start_time.sec+m.start_time.nanosec*1e-9 for m in messages]
            self.assertTrue(all(20<=x<=sim+1 for x in stamps),str(stamps))
            (out/'isolated_cpp_clock_verification.json').write_text(json.dumps({'pass':True,'ros_domain_id':188,'gazebo_started':False,'drive_published':False,'actual_planner_binary':str(ROOT/'install/ego_planner/lib/ego_planner/ego_planner_node'),'sim_clock_s':sim,'spline_starts_s':stamps,'goal_count':1},indent=2))
        finally:
            try:os.killpg(process.pid,signal.SIGINT);process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            log.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':unittest.main()
