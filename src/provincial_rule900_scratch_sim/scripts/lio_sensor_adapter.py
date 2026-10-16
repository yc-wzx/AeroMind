#!/usr/bin/env python3
"""Only simulated measurements -> SPARK sensor input. No odometry/GT subscriptions."""
import copy,json,time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,PointField,Imu
from std_msgs.msg import String
from lio_contract import points,pack_mid360,valid_imu,stamp_s

class SensorAdapter(Node):
    def __init__(self):
        super().__init__('lio_sensor_adapter')
        self.cloud_pub=self.create_publisher(PointCloud2,'/lio/lidar',10)
        self.imu_pub=self.create_publisher(Imu,'/lio/imu',qos_profile_sensor_data)
        self.diag=self.create_publisher(String,'/lio/sensor_diagnostics',100)
        self.last={};self.counts={'cloud':0,'imu':0,'rejected':0}
        self.create_subscription(PointCloud2,'/simulation/lidar3d/points',self.cloud,qos_profile_sensor_data)
        self.create_subscription(Imu,'/simulation/imu3d',self.imu,qos_profile_sensor_data)

    def check_time(self,key,msg):
        t=stamp_s(msg.header.stamp);age=self.get_clock().now().nanoseconds*1e-9-t
        if not -.05 <= age <= .2 or (key in self.last and t <= self.last[key]):
            raise ValueError('stale/future/nonmonotonic '+key)
        self.last[key]=t
        return t

    def report(self,**data):
        self.diag.publish(String(data=json.dumps(data,allow_nan=False)))

    def cloud(self,msg):
        try:
            xyz,invalid=points(msg);t=self.check_time('cloud',msg)
            out=PointCloud2();out.header=copy.deepcopy(msg.header);out.header.frame_id='lio_lidar'
            out.height=1;out.width=len(xyz);out.point_step=32;out.row_step=32*len(xyz);out.is_dense=True
            layout=[('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('tag',16,2),('line',17,2),('timestamp',24,8)]
            out.fields=[PointField(name=n,offset=o,datatype=d,count=1) for n,o,d in layout]
            out.data=pack_mid360(xyz,msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec)
            self.cloud_pub.publish(out);self.counts['cloud']+=1
            self.report(event='cloud_accepted',stamp_s=t,point_count=len(xyz),invalid_returns=invalid,
                instantaneous=True,point_time_span_s=0,synthetic_intensity_line=True)
        except (ValueError,TypeError,AttributeError) as error:
            self.counts['rejected']+=1;self.report(event='rejected',stream='cloud',reason=str(error))

    def imu(self,msg):
        try:
            valid_imu(msg);self.check_time('imu',msg)
            out=copy.deepcopy(msg);out.header.frame_id='lio_imu'
            # Gazebo absolute orientation is excluded; LIO uses gyro/specific force.
            out.orientation.x=out.orientation.y=out.orientation.z=out.orientation.w=0.
            out.orientation_covariance=[-1.]+[0.]*8
            self.imu_pub.publish(out);self.counts['imu']+=1
        except (ValueError,TypeError,AttributeError) as error:
            self.counts['rejected']+=1;self.report(event='rejected',stream='imu',reason=str(error))

def main():
    rclpy.init();node=SensorAdapter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
