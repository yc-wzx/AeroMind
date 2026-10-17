"""Test-only native capture fixture, isolated localhost ROS domain 218."""
import os,time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

if os.environ.get('ROS_DOMAIN_ID')!='218' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise SystemExit('Synthetic source requires isolated localhost domain 218')
rclpy.init()
node=Node('mid360_real_lio',namespace='/hardware/mid360/lio')
for name,value in {
    'common.visualization_frame':'imu','common.imu_frame':'mid360_imu',
    'common.map_frame':'mid360_local_odom','common.base_frame':'','common.time_sync_en':False,
    'mapping.extrinsic_T':[-.011,-.02329,.04412],
    'mapping.extrinsic_R':[1.,0.,0.,0.,1.,0.,0.,0.,1.],
    'common.odometry_output_contract':'scan_corrected_primary_v1'}.items():node.declare_parameter(name,value)
pred=node.create_publisher(Odometry,'odometry_predicted',1000)
corrected=node.create_publisher(Odometry,'odometry',1000)
mixed=node.create_publisher(Odometry,'odometry_mixed_legacy',1000)
tf=node.create_publisher(TFMessage,'/tf',1000)
count=0
def tick():
    global count
    now=time.time_ns();count+=1
    msg=Odometry();msg.header.frame_id='mid360_local_odom';msg.child_frame_id='mid360_imu'
    msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(now,10**9);msg.pose.pose.orientation.w=1.
    pred.publish(msg);mixed.publish(msg)
    if count%20==0:
        msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(now-2000000,10**9)
        corrected.publish(msg);mixed.publish(msg)
        transform=TransformStamped();transform.header=msg.header;transform.child_frame_id=msg.child_frame_id
        transform.transform.rotation.w=1.
        message=TFMessage();message.transforms=[transform];tf.publish(message)
node.create_timer(.005,tick)
print('READY: synthetic source only, no physical radar or motors',flush=True)
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:node.destroy_node();rclpy.shutdown()
