#!/usr/bin/env python3
"""Independent circular-body entry point; qualified legacy files untouched."""
import json,hashlib,math,time
from pathlib import Path
from runtime_support import load_existing
load_existing()
import rclpy
from radar_fixed_navigation import RadarFixedNavigation
from circle_geometry import RADIUS,CONTRACT_PATH,circle_in_zone
from circle_route_gate import route_preflight


class CircleNavigation(RadarFixedNavigation):
    def __init__(self):
        super().__init__()
        radius=float(self.declare_parameter('circle_envelope_radius_m',.30).value)
        if radius!=RADIUS:raise ValueError('Circular contract radius mismatch')
        self.circle_terminal_zone=None
        self._input_first_callbacks=(self.update,self._flush_pending_scan)
        body=next(s for s in self.subscriptions if getattr(s.callback,'__self__',None) is self and getattr(s.callback,'__name__',None)=='gazebo_odom')
        heading=next(s for s in self.subscriptions if getattr(s.callback,'__self__',None) is self and getattr(s.callback,'__name__',None)=='radar_pose')
        clock=next(s for s in self.subscriptions if s.topic_name=='/clock')
        goal=next(s for s in self.subscriptions if s.topic_name=='/goal_pose')
        # Depth1 actual /clock both before input validation and immediately before
        # the original consumer. No clock is inferred from a pose/header stamp.
        self._input_first_subscriptions=(clock,body,heading,self.scan_body_history_subscription,clock)
        self._input_first_goal_subscriptions=(goal,)
        self._input_first_last_diag=-math.inf
        from std_msgs.msg import String
        self._input_first_diag=self.create_publisher(String,'/navigation/input_first_diagnostics',20)

        self.get_logger().info('CIRCLE ENVELOPE ACTIVE: radius=0.300m; .08m gap + .04m pose reserve + unchanged timing reserve; contract_sha256='+hashlib.sha256(CONTRACT_PATH.read_bytes()).hexdigest())

    def record_input_first(self,started,entries,total):
        if started-self._input_first_last_diag<.1:return
        from std_msgs.msg import String
        self._input_first_diag.publish(String(data=json.dumps(dict(start_monotonic_s=started,sim_s=self.get_clock().now().nanoseconds*1e-9,inputs=entries,timer_and_prefetch_wall_s=total,body_stamp_s=self.last_stage2_stamp,bounded_actual_messages_only=True),allow_nan=False)))
        self._input_first_last_diag=started

    def route_goal(self,goal):
        preflight_started=time.monotonic()
        start=(self.x,self.y);target=(goal.pose.position.x,goal.pose.position.y)
        try:
            pre=route_preflight(start,target,self.grid_route,self.provincial_walls,
                                self.field['reference_route'],self.provincial_min_body_gap,self.pose_safety_margin,require_explicit_grid_route=False)
            if not pre['pass']:raise ValueError(pre['reason']+' min_gap='+str(pre['minimum_circle_gap_m']))
        except (ValueError,TypeError,AttributeError) as error:
            self.get_logger().warn('GOAL REJECTED: CIRCLE ENVELOPE RESERVED CLEARANCE: '+str(error));return
        self.get_logger().info("CIRCLE ROUTE PREFLIGHT: wall_s=%.6f; same frozen clearance thresholds"%(time.monotonic()-preflight_started))
        previous=self.route_diag_id
        super().route_goal(goal)
        if self.route_diag_id!=previous:
            # Standard workflow endpoints only; generic mid-route goals remain
            # point goals with the full circle route guard above.
            self.circle_terminal_zone=None
            zones=[self.field['start'],self.field['shooting_zone']]
            for zone in zones:
                xy=(zone['x'],zone['y'])
                bounds=(xy[0]-zone['width']/2,xy[1]-zone['height']/2,
                        xy[0]+zone['width']/2,xy[1]+zone['height']/2)
                if math.dist(target,xy)<1e-6:self.circle_terminal_zone=bounds

    def internal_waypoint_handoff_ready(self,distance,speed):
        if not self.waypoints and self.circle_terminal_zone is not None:
            if not circle_in_zone(self.x,self.y,self.circle_terminal_zone):return False
        return super().internal_waypoint_handoff_ready(distance,speed)


def main():
    # Use the same qualified bounded RMW dispatcher as the radar entry point.
    from input_first_executor import InputFirstExecutor
    rclpy.init();node=CircleNavigation();executor=InputFirstExecutor();executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
