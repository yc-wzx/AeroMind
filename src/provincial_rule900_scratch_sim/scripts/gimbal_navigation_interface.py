#!/usr/bin/env python3
"""Keep planner/rectangle checks in chassis frame; transform moving scans."""
import copy,json,math,time
from collections import deque
from functools import partial
from nav_msgs.msg import Odometry
from gimbal_static_ray import ray_has_unmapped_return
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from lio_safe_navigation_interface import LioSafeNavigationInterface
from lio_safety_geometry import scan_points
from gazebo_navigation_interface import GazeboNavigationInterface,stamp_seconds
from gimbal_contract import JointBuffer,Mount
from gimbal_command_governor import limit_planar_command,GOVERNOR_COMPUTE_BUDGET_S

class GimbalNavigationInterface(LioSafeNavigationInterface):
    def __init__(self):
        self.joints=JointBuffer();self.mount=Mount();self.approach_plan_identity=None
        super().__init__()
        self.create_subscription(JointState,'/gimbal/joint_states',self.joint,1)
        if self.grid_route is not None:
            self.grid_route.ray_has_unmapped_return=partial(ray_has_unmapped_return,self.grid_route)
        self.governor_pub=self.create_publisher(String,'/gimbal/command_governor',100)

    def create_subscription(self,msg_type,topic,callback,qos_profile,*args,**kwargs):
        # Only the control consumer prefers latest measurements. Publication,
        # estimator inputs and timestamp/history gap checks are unchanged.
        if msg_type is Odometry and topic=="/gazebo/odometry" and callback==self.gazebo_odom:
            qos_profile=1
        return super().create_subscription(msg_type,topic,callback,qos_profile,*args,**kwargs)

    def joint(self,msg):
        try:
            i=msg.name.index('gimbal_yaw')
            self.joints.add(stamp_seconds(msg.header.stamp),msg.position[i],msg.velocity[i],time.monotonic(),
                            self.get_clock().now().nanoseconds*1e-9)
        except (ValueError,IndexError,TypeError):self.joints.fault=True

    def route_goal(self,goal):
        q=goal.pose.orientation
        values=[q.x,q.y,q.z,q.w]
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        error=math.atan2(math.sin(yaw-math.pi/2),math.cos(yaw-math.pi/2))
        if not all(math.isfinite(v) for v in values) or abs(error)>.03 or abs(q.x)>.01 or abs(q.y)>.01:
            self.get_logger().warn('GOAL REJECTED: fixed chassis heading requires yaw=90 degrees');return
        super().route_goal(goal)

    def localization_fresh(self):
        try:self.joints.latest(self.get_clock().now().nanoseconds*1e-9,time.monotonic())
        except ValueError:return False
        return super().localization_fresh()

    def internal_waypoint_handoff_ready(self,distance,speed):
        # A low speed alone can indicate an interrupted/ending spline, not
        # arrival at a corner. Restrict internal handoffs before the production
        # completion event is logged. Final endpoint acceptance is unchanged.
        if self.waypoints:
            return distance<.008 and speed<.01
        return True

    def waypoint_approach_eligible(self,remaining):
        # EGO can end inside the old 0.10m activation band. A stricter internal
        # handoff must still have a guarded way to finish the final centimetres.
        if self.waypoints:
            return .008<remaining<.40
        return super().waypoint_approach_eligible(remaining)

    def _approach_identity(self):
        if self.active_waypoint is None:return None
        return (self.route_diag_id,self.active_waypoint_diag_id,id(self.active_waypoint))

    def planned_trajectory_callback(self,marker):
        before=self.planned_path_received_at
        super().planned_trajectory_callback(marker)
        if (self.planned_path_received_at!=before and self.planned_path_safe is True
                and self.active_waypoint is not None and marker.points):
            endpoint=marker.points[-1];target=self.active_waypoint.pose.position
            if math.hypot(endpoint.x-target.x,endpoint.y-target.y)<=.05:
                self.approach_plan_identity=self._approach_identity()

    def waypoint_approach_delay_elapsed(self,now_steady):
        if super().waypoint_approach_delay_elapsed(now_steady):return True
        if (self.waypoints and self.active_waypoint is not None
                and self.planned_path_safe is True
                and self.approach_plan_identity==self._approach_identity()
                and self._task_localization_fresh()):
            p=self.active_waypoint.pose.position
            return .008<math.hypot(p.x-self.x,p.y-self.y)<.10
        return False

    def scan_callback(self,scan):
        if scan.header.frame_id!='omni_robot/gimbal_link/lidar':
            self.scan_received_at=None;self.safety_scan_fault=True;return
        history=self.odom_history
        try:
            t=stamp_seconds(scan.header.stamp)
            angle,_=self.joints.at(t,self.get_clock().now().nanoseconds*1e-9,time.monotonic())
            matched=min(history,key=lambda p:abs(p[0]-t))
            if abs(matched[0]-t)>.025:raise ValueError('scan/base association exceeds25ms')
            sensor=self.mount.sensor_planar_pose(matched[1:],angle)
            # Reuse numeric ray validation after strict moving-frame validation.
            # No coordinate conversion is implied by this private frame alias.
            validation=copy.deepcopy(scan);validation.header.frame_id='omni_robot/base_link/lidar'
            points=scan_points(validation)
            if self.safety_scan is not None and t<=self.safety_scan['stamp']:
                raise ValueError('nonmonotonic scan')
            self.safety_scan=dict(points=points,pose=sensor,pose_stamp=matched[0],stamp=t,received=time.monotonic())
            self.safety_scan_fault=False
            # Only scan projection gets the moving sensor pose. Chassis estimate
            # and rectangle orientation remain the measured chassis pose.
            self.odom_history=deque([(matched[0],*sensor)])
            GazeboNavigationInterface.scan_callback(self,scan)
        except (ValueError,IndexError):
            self.scan_received_at=None;self.safety_scan_fault=True
        finally:self.odom_history=history

    def check_sensor_command(self,command):
        if not math.isfinite(command.angular.z):
            return super().check_sensor_command(command)
        planar=copy.deepcopy(command);planar.angular.z=0.
        now=self.get_clock().now().nanoseconds*1e-9
        began=time.monotonic(); original=[planar.linear.x,planar.linear.y,0.]
        report=dict(action='not_exercised',scale=None)
        observation=self.safety_scan
        if (all(math.isfinite(v) for v in original) and any(original[:2])
                and self.estimate is not None and self.last_stage2_stamp is not None
                and not self.safety_scan_fault and observation is not None
                and 0<=now-observation['stamp']<=.2
                and began-observation['received']<=.2):
            try:
                limited,report=limit_planar_command(self.estimate,original,
                    self.provincial_walls,now-self.last_stage2_stamp,
                    self.provincial_min_body_gap+self.pose_safety_margin)
                planar.linear.x,planar.linear.y,planar.angular.z=limited
            except ValueError:
                report=dict(action='invalid_input_stop',scale=0.)
                planar.linear.x=planar.linear.y=0.
            if time.monotonic()-began>GOVERNOR_COMPUTE_BUDGET_S:
                planar.linear.x=planar.linear.y=0.
                report.update(action='governor_compute_budget_stop',scale=0.)
        governor_elapsed=time.monotonic()-began
        result=super().check_sensor_command(planar)
        report.update(sim_s=now,requested=[v if math.isfinite(v) else repr(v) for v in original],
                      limited=[planar.linear.x if math.isfinite(planar.linear.x) else repr(planar.linear.x),
                               planar.linear.y if math.isfinite(planar.linear.y) else repr(planar.linear.y),0.],
                      final=[result.linear.x,result.linear.y,result.angular.z],
                      final_guard_action=self.last_sensor_action,
                      internal_waypoint=self.active_waypoint_diag_id,
                      final_approach_active=self.final_approach_active,
                      stage_elapsed_s=None if self.stage_sent_at is None else began-self.stage_sent_at,
                      approach_plan_current=self.approach_plan_identity==self._approach_identity() and self.active_waypoint is not None,
                      compute_wall_s=time.monotonic()-began,
                      governor_compute_wall_s=governor_elapsed,
                      control_started_monotonic_s=began,
                      pose_age_s=None if self.last_stage2_stamp is None else now-self.last_stage2_stamp,
                      lio_receive_age_s=None if self.last_lio_received is None else began-self.last_lio_received,
                      correction_sim_age_s=None if self.last_match_sim is None else now-self.last_match_sim,
                      correction_receive_age_s=None if self.last_match_steady is None else began-self.last_match_steady,
                      scan_sim_age_s=None if observation is None else now-observation["stamp"],
                      scan_receive_age_s=None if observation is None else began-observation["received"],
                      scan_input_fault=self.safety_scan_fault,
                      joint_sim_age_s=None if not self.joints.samples else now-self.joints.samples[-1][0],
                      joint_receive_age_s=None if not self.joints.samples else began-self.joints.samples[-1][3],
                      localization_fresh_snapshot=self._localization_update_fresh,
                      scope='same-direction planar speed limit; original final guard remains authoritative')
        self.governor_pub.publish(String(data=json.dumps(report,allow_nan=False)))
        return result

def main():
    rclpy.init();node=GimbalNavigationInterface()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
