#!/usr/bin/env python3
"""Only this simulation source reads truth for navigation fault generation.

Does not publish velocity commands, TF or /ground/odometry. No physical sensor
claim: random-walk pose perturbations are not differentiated into twist noise.
"""
import copy
import json
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from stage2_odometry_model import ErrorParameters, OdometryErrorModel, wrap


def extract(message):
    pose, twist = message.pose.pose, message.twist.twist
    q = pose.orientation
    values = (pose.position.x, pose.position.y, pose.position.z,
              q.x, q.y, q.z, q.w, twist.linear.x, twist.linear.y, twist.linear.z,
              twist.angular.x, twist.angular.y, twist.angular.z,
              *message.pose.covariance, *message.twist.covariance)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('nonfinite source odometry')
    if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .02:
        raise ValueError('invalid source quaternion')
    angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    return (pose.position.x, pose.position.y, angle), (twist.linear.x, twist.linear.y, twist.angular.z)


class SimulatedOdometry(Node):
    def __init__(self):
        super().__init__('stage2_simulated_odometry')
        defaults = ErrorParameters()
        settings = {name: self.declare_parameter(name, value).value
                    for name, value in defaults.__dict__.items()}
        self.model = OdometryErrorModel(ErrorParameters(**settings))
        self.max_age_s = self.declare_parameter('max_source_age_s', .2).value
        self.accepted = self.rejected = 0
        self.pub = self.create_publisher(Odometry, '/simulation/navigation_odometry', 30)
        self.diag = self.create_publisher(String, '/simulation/odometry_diagnostics', 100)
        self.create_subscription(Odometry, '/gazebo/odometry', self.receive, 100)

    def receive(self, source):
        stamp = source.header.stamp.sec+source.header.stamp.nanosec*1e-9
        try:
            age = self.get_clock().now().nanoseconds*1e-9-stamp
            if not math.isfinite(age) or age > self.max_age_s or age < -.05:
                raise ValueError('stale/future source odometry')
            truth, twist = extract(source)
            pose, velocity = self.model.advance(stamp, truth, twist)
        except ValueError as error:
            self.rejected += 1
            self.get_logger().error('STAGE2 ODOM REJECTED: ' + str(error))
            d = String()
            d.data = json.dumps({'accepted': False, 'stamp_s': stamp,
                                 'reason': str(error), 'rejected_count': self.rejected})
            self.diag.publish(d)
            return
        output = copy.deepcopy(source)
        output.header.frame_id, output.child_frame_id = 'odom', 'base_link'
        if not self.model.parameters.zero_error:
            p, q = output.pose.pose.position, output.pose.pose.orientation
            p.x, p.y, p.z = *pose[:2], 0.0
            q.x, q.y, q.z, q.w = 0.0, 0.0, math.sin(pose[2]/2), math.cos(pose[2]/2)
            output.twist.twist.linear.x, output.twist.twist.linear.y = velocity[:2]
            output.twist.twist.angular.z = velocity[2]
            # Only injected random-walk variance; not calibrated uncertainty.
            var = self.model.parameters.position_walk_m_sqrt_s**2*self.model.elapsed_s
            output.pose.covariance[0] += var
            output.pose.covariance[7] += var
            output.pose.covariance[35] += self.model.parameters.yaw_walk_rad_sqrt_s**2*self.model.elapsed_s
        self.pub.publish(output)
        self.accepted += 1
        d = String()
        d.data = json.dumps({'accepted': True, 'stamp_s': stamp,
                             'truth_xy_yaw': truth, 'navigation_xy_yaw': pose,
                             'position_error_m': math.dist(truth[:2], pose[:2]),
                             'yaw_error_rad': wrap(pose[2]-truth[2]),
                             'elapsed_s': self.model.elapsed_s,
                             'accepted_count': self.accepted, 'rejected_count': self.rejected})
        self.diag.publish(d)


def main():
    rclpy.init()
    node = SimulatedOdometry()
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
