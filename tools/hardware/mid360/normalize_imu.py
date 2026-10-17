#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from core import IMU_RAW, IMU_SI, imu_to_si

class Normalizer(Node):
    def __init__(self):
        super().__init__('mid360_imu_unit_adapter')
        self.publisher = self.create_publisher(Imu, IMU_SI, qos_profile_sensor_data)
        self.subscription = self.create_subscription(Imu, IMU_RAW, self.receive, qos_profile_sensor_data)
        self.last_error_s = -float('inf')

    def receive(self, message):
        try:
            out = imu_to_si(message)
        except ValueError as error:
            import time
            now = time.monotonic()
            if now-self.last_error_s>1.:
                self.get_logger().error('IMU REJECTED: '+str(error));self.last_error_s=now
            return
        self.publisher.publish(out)

def main():
    rclpy.init(); node=Normalizer()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
