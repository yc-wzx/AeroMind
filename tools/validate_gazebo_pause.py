import subprocess,time,math,json
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
rclpy.init(); n=rclpy.create_node("pause_resume_validation")
state={}
n.create_subscription(Odometry,"/gazebo/odometry",lambda m:state.update(odom=m),10)
n.create_subscription(Clock,"/clock",lambda m:state.update(clock=m.clock),10)
pub=n.create_publisher(PoseStamped,"/goal_pose",10)
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end: rclpy.spin_once(n,timeout_sec=.05)
def control(paused):
 p=subprocess.run(["ign","service","-s","/world/competition/control","--reqtype","ignition.msgs.WorldControl","--reptype","ignition.msgs.Boolean","--timeout","3000","--req","pause: "+str(paused).lower()],capture_output=True,text=True,check=True)
 assert "true" in p.stdout,p.stdout
def goal(x,y):
 g=PoseStamped();g.header.frame_id="odom";g.pose.position.x=x;g.pose.position.y=y
 g.pose.orientation.z=math.sqrt(.5);g.pose.orientation.w=math.sqrt(.5);pub.publish(g)
spin(1.5);assert pub.get_subscription_count()>0
goal(4.7,.5);spin(2.0)
try:
 control(True);spin(.5)
 p=state["odom"].pose.pose.position;before=(p.x,p.y);c=state["clock"];ct=c.sec+c.nanosec*1e-9
 spin(2.5)
 p=state["odom"].pose.pose.position;shift=math.hypot(p.x-before[0],p.y-before[1])
 c=state["clock"];clock_delta=c.sec+c.nanosec*1e-9-ct
 print(json.dumps({"paused_pose_delta":shift,"paused_sim_time_delta":clock_delta}),flush=True)
 assert shift<.002 and abs(clock_delta)<.01
finally: control(False)
spin(2.0)
p=state["odom"].pose.pose.position
movement=math.hypot(p.x-before[0],p.y-before[1])
print("resume_movement_m",movement,flush=True);assert movement>.05
# Return control to a visible mission ending at the shooting zone.
goal(8.7,4.25)
n.destroy_node();rclpy.shutdown()