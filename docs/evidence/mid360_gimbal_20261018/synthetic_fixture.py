"""Isolated kinematic fixture for recorder tests, never a hardware estimator."""
import json,math,os,sys,time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

if os.environ.get('ROS_DOMAIN_ID')!='220' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise SystemExit('Requires isolated localhost domain 220')
control=sys.argv[1];rclpy.init()
node=Node('mid360_real_lio',namespace='/hardware/mid360/lio');adapter=Node('mid360_imu_unit_adapter')
for name,value in {'common.odometry_output_contract':'scan_corrected_primary_v1','common.visualization_frame':'imu',
    'common.imu_frame':'mid360_imu','common.map_frame':'mid360_local_odom','common.base_frame':'','common.time_sync_en':False,
    'mapping.extrinsic_T':[-.011,-.02329,.04412],'mapping.extrinsic_R':[1.,0.,0.,0.,1.,0.,0.,0.,1.]}.items():node.declare_parameter(name,value)
pred=node.create_publisher(Odometry,'odometry_predicted',1000);corrected=node.create_publisher(Odometry,'odometry',1000)
mixed=node.create_publisher(Odometry,'odometry_mixed_legacy',1000);tf=node.create_publisher(TFMessage,'/tf',1000)
imu_pub=adapter.create_publisher(Imu,'/hardware/mid360/imu_si',1000);count=0
def tick():
    global count
    count+=1;now=time.time_ns()
    with open(control) as f:c=json.load(f)
    elapsed=time.monotonic()-c['start'];fraction=max(0.,min(1.,elapsed/c['duration']))
    theta=c['from_angle']+(c['target']-c['from_angle'])*fraction
    rate=(c['target']-c['from_angle'])/c['duration'] if 0<elapsed<c['duration'] else 0.
    msg=Odometry();msg.header.frame_id='mid360_local_odom';msg.child_frame_id='mid360_imu'
    msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(now,10**9)
    msg.pose.pose.position.x=.1*math.cos(theta);msg.pose.pose.position.y=-.1*math.sin(theta)
    msg.pose.pose.orientation.z=-math.sin(theta/2);msg.pose.pose.orientation.w=math.cos(theta/2)
    pred.publish(msg);mixed.publish(msg)
    imu=Imu();imu.header.stamp=msg.header.stamp;imu.header.frame_id='mid360_imu';imu.linear_acceleration.z=-9.80665
    imu.angular_velocity.z=-rate;imu_pub.publish(imu)
    if count%20==0:
        msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(now-2_000_000,10**9)
        corrected.publish(msg);mixed.publish(msg)
        transform=TransformStamped();transform.header=msg.header;transform.child_frame_id=msg.child_frame_id
        transform.transform.translation.x=msg.pose.pose.position.x;transform.transform.translation.y=msg.pose.pose.position.y
        transform.transform.rotation=msg.pose.pose.orientation;message=TFMessage();message.transforms=[transform];tf.publish(message)
node.create_timer(.005,tick)
ex=rclpy.executors.SingleThreadedExecutor();ex.add_node(node);ex.add_node(adapter)
print('READY synthetic kinematics only; no hardware/motors',flush=True)
try:ex.spin()
except KeyboardInterrupt:pass
finally:
    ex.shutdown();node.destroy_node();adapter.destroy_node()
    if rclpy.ok():rclpy.shutdown()
