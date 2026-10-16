#!/usr/bin/env python3
"""Stage 2 adapter reuses frozen navigation/control without reading GT.

Launch MUST remap inherited /gazebo/odometry subscription to the independent
navigation odometry source. It remains body-frame twist at this boundary;
the existing interface converts it to its legacy world-velocity convention.
"""
import math
import rclpy
from gazebo_navigation_interface import GazeboNavigationInterface, stamp_seconds
from stage2_simulated_odometry import extract


class Stage2NavigationInterface(GazeboNavigationInterface):
    def __init__(self):
        super().__init__()
        if self.imperfect_sensors:
            raise RuntimeError('Legacy imperfect_sensors must remain false in Stage 2')
        if self.resolve_topic_name('/gazebo/odometry') != '/simulation/navigation_odometry':
            raise RuntimeError('Stage 2 refuses a direct ground-truth subscription')
        self.last_stage2_stamp = None
        self.get_logger().info('STAGE2 pose input=/simulation/navigation_odometry; drive remains ideal')

    def gazebo_odom(self, message):
        try:
            extract(message)
            stamp = stamp_seconds(message.header.stamp)
            age = self.get_clock().now().nanoseconds*1e-9-stamp
            if (message.header.frame_id != 'odom' or message.child_frame_id != 'base_link'
                    or not math.isfinite(age) or age > .2 or age < -.05
                    or (self.last_stage2_stamp is not None and stamp <= self.last_stage2_stamp)):
                raise ValueError('invalid frame, stale/future or nonmonotonic navigation input')
        except ValueError as error:
            self.get_logger().error('STAGE2 NAV ODOM REJECTED: ' + str(error))
            return
        self.last_stage2_stamp = stamp
        super().gazebo_odom(message)


def main():
    rclpy.init()
    node = Stage2NavigationInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
