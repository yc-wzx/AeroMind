import rclpy,time,math,json
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); node=rclpy.create_node("gazebo_roundtrip_check")
state={"pose":None,"scan":None}; samples=[]; results=[]
def odom(m):
 state["pose"]=m; p=m.pose.pose.position
 samples.append((p.x,p.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w))
def scan(m): state["scan"]=m
node.create_subscription(Odometry,"/gazebo/odometry",odom,30)
node.create_subscription(LaserScan,"/scan",scan,qos_profile_sensor_data)
pub=node.create_publisher(PoseStamped,"/goal_pose",5)
start=time.monotonic()
while (state["pose"] is None or pub.get_subscription_count()==0 or time.monotonic()-start<1.0) and time.monotonic()-start<10: rclpy.spin_once(node,timeout_sec=.1)
assert state["pose"] is not None and pub.get_subscription_count()>0
for label,x,y in [("return_start",4.7,.5),("shooting_zone",8.7,4.25)]:
 goal=PoseStamped();goal.header.frame_id="odom"
 goal.pose.position.x=x;goal.pose.position.y=y;goal.pose.orientation.z=math.sqrt(.5);goal.pose.orientation.w=math.sqrt(.5)
 pub.publish(goal)
 print("GOAL",label,flush=True)
 start=time.monotonic(); settled=None; ok=False
 while time.monotonic()-start<65:
  rclpy.spin_once(node,timeout_sec=.1)
  m=state["pose"];p=m.pose.pose.position;v=m.twist.twist.linear
  error=math.hypot(p.x-x,p.y-y);speed=math.hypot(v.x,v.y)
  if error<.1 and speed<.02:
   if settled is None: settled=time.monotonic()
   if time.monotonic()-settled>1:ok=True;break
  else:settled=None
 result={"goal":label,"success":ok,"error_m":error,"speed":speed,"wall_seconds":time.monotonic()-start,"position":[p.x,p.y]}
 results.append(result);print(json.dumps(result),flush=True)
 if not ok:break
segments=json.load(open(str(__import__("pathlib").Path(__file__).resolve().parents[1]/"src/uav_planning/config/competition_field_2025.json")))["collision_segments"]
overlaps=0; min_margin=999
for x,y,z,w in samples:
 yaw=2*math.atan2(z,w)
 # AABB is conservative if yaw deviates from 90deg.
 hx=abs(math.cos(yaw))*.26+abs(math.sin(yaw))*.21
 hy=abs(math.sin(yaw))*.26+abs(math.cos(yaw))*.21
 for x1,y1,x2,y2 in segments:
  if x1==x2:
   dx=max(abs(x-x1)-hx-.0275,0);dy=max(min(y1,y2)-y-hy,y-max(y1,y2)-hy,0)
  else:
   dx=max(min(x1,x2)-x-hx,x-max(x1,x2)-hx,0);dy=max(abs(y-y1)-hy-.0275,0)
  d=math.hypot(dx,dy);min_margin=min(min_margin,d)
  if d==0: overlaps+=1
report={"legs":results,"samples":len(samples),"overlap_samples":overlaps,"min_body_clearance_m":min_margin}
print(json.dumps(report,indent=2),flush=True)
open("/tmp/gazebo_roundtrip.json","w").write(json.dumps(report,indent=2))
node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if all(r["success"] for r in results) and len(results)==2 and overlaps==0 else 1)