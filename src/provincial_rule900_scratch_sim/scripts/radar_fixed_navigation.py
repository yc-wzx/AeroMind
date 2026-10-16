#!/usr/bin/env python3
"""Reuse qualified route, footprint, hold/resume and final guards with LIO heading."""
from runtime_support import load_existing
load_existing()
import math,time,json,copy
from collections import deque
from lio_safety_geometry import scan_points
from radar_scan_batch import ray_answers
from radar_path_geometry import path_minimum_gap
from gazebo_navigation_interface import stamp_seconds,wrap_angle
from stage2_localized_interface import _LocalizationTaskPaused
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import String
from gimbal_navigation_interface import GimbalNavigationInterface
from lio_safe_navigation_interface import LioSafeNavigationInterface
from stage2_simulated_odometry import extract
from radar_fixed_contract import configured_mount,RadarHeadingBuffer,planar_yaw,rotation,wrap

class ScanDiagnosticHeadingBuffer(RadarHeadingBuffer):
    # Record the production association result without changing its acceptance.
    def latest(self,now,steady,age=.05):
        sample=self.samples[-1] if self.samples else None
        report=dict(now_sim_s=now,now_monotonic_s=steady,requested_age_limit_s=age,
            input_fault=self.fault,stamp_s=None if sample is None else sample[0],
            sim_age_s=None if sample is None else now-sample[0],
            receive_age_s=None if sample is None else steady-sample[3])
        try:result=super().latest(now,steady,age)
        except ValueError as error:
            self.last_latest_check=dict(report,accepted=False,reason=str(error));raise
        self.last_latest_check=dict(report,accepted=True,reason=None)
        return result

    def at(self, stamp, now, steady):
        try:
            result=super().at(stamp,now,steady)
        except ValueError as error:
            self.last_scan_association={'accepted':False,'reason':str(error)}
            raise
        self.last_scan_association={'accepted':True,'reason':None}
        return result

class _ScanBodyHistoryTap:
    """History has its own bounded drain; latest-control queue remains depth 1."""
    def __init__(self, owner, topic):
        self.owner=owner;self._subscription_batch_limits={topic:8};self.info=None
    def record_rmw_info(self, topic, info):self.info=info
    def receive(self, message):self.owner.scan_body_odom(message)


class RadarFixedNavigation(GimbalNavigationInterface):
    def __init__(self):
        self.scan_body_history=deque(maxlen=150)
        self.scan_body_history_fault=True
        self.last_scan_body_diag=-math.inf
        self.scan_body_rejected_count=0
        super().__init__()
        self.mount=configured_mount(self)
        self.joints=ScanDiagnosticHeadingBuffer(self.mount.initial_base[3])
        self.last_input_diag=-math.inf
        self.last_freshness_diag=-math.inf;self.goal_freshness_capture=None
        self.freshness_diag=self.create_publisher(String,'/navigation/freshness_diagnostics',20)
        self.goal_freshness_diag=self.create_publisher(String,'/navigation/goal_freshness_diagnostics',20)
        self.last_scan_diag=-math.inf
        self.pending_scans=deque()
        self.create_timer(.02,self._flush_pending_scan)
        self.scan_diag=self.create_publisher(String,'/navigation/scan_association_diagnostics',20)
        self.path_validation_diag=self.create_publisher(String,'/navigation/path_validation_diagnostics',20)
        self.input_message_info={}
        self.input_diag=self.create_publisher(String,'/navigation/radar_input_diagnostics',100)
        self.heading_diag=self.create_publisher(String,'/navigation/radar_heading_diagnostics',100)
        # Latest body input remains depth1. Heading history has bounded dispatch.
        self._subscription_batch_limits={'/localization/radar_pose':8}
        self.create_subscription(Odometry,'/localization/radar_pose',self.radar_pose,1)
        self.scan_body_diag=self.create_publisher(String,'/navigation/scan_body_history_diagnostics',20)
        topic=self.resolve_topic_name('/gazebo/odometry')
        if topic!='/localization/lio_navigation_odometry':
            raise RuntimeError('Scan body history must use the existing non-GT LIO body stream')
        self.scan_body_history_tap=_ScanBodyHistoryTap(self,topic)
        self.scan_body_history_subscription=rclpy.node.Node.create_subscription(
            self,Odometry,topic,self.scan_body_history_tap.receive,
            QoSProfile(depth=32,reliability=ReliabilityPolicy.BEST_EFFORT))

    def create_subscription(self,msg_type,topic,callback,qos_profile,*args,**kwargs):
        # Block the parent encoder subscription rather than fabricate JointState.
        if topic=='/gimbal/joint_states':return None
        if msg_type is Odometry and topic in ('/gazebo/odometry','/localization/radar_pose'):
            # Body control consumes latest state; scan association needs heading history.
            # Keep a bounded best-effort heading queue; all original age/gap guards apply.
            # Use Node directly so the parent's integer-depth reliable override
            # cannot silently undo this explicit best-effort consumer contract.
            return rclpy.node.Node.create_subscription(self,msg_type,topic,callback,
                QoSProfile(depth=32 if topic=='/localization/radar_pose' else 1,
                    reliability=ReliabilityPolicy.BEST_EFFORT),*args,**kwargs)
        return super().create_subscription(msg_type,topic,callback,qos_profile,*args,**kwargs)

    def record_rmw_info(self,topic,info):
        self.input_message_info[topic]=info
        if topic==self.resolve_topic_name('/gazebo/odometry'):
            self.input_message_info['/gazebo/odometry']=info

    def gazebo_odom(self,msg):
        began=time.monotonic();now=self.get_clock().now().nanoseconds*1e-9
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9;previous=self.last_stage2_stamp
        super().gazebo_odom(msg)
        accepted=self.last_stage2_stamp!=previous
        # High-rate body input only updates state; scan work runs on scan/timer.
        reason=None
        if not accepted:
            try:extract(msg)
            except ValueError as error:reason=str(error)
            if reason is None:
                if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':reason='frame'
                elif not math.isfinite(now-t):reason='nonfinite_age'
                elif now-t>.2:reason='stale_age'
                elif now-t<-.05:reason='future_age'
                elif previous is not None and t<=previous:reason='nonmonotonic_stamp'
                else:reason='unclassified_parent_rejection'
        if began-self.last_input_diag>=.1 or not accepted:
            self.input_diag.publish(String(data=json.dumps(dict(source='navigation odometry callback entry',
                sim_s=now,stamp_s=t,receive_monotonic_s=began,callback_system_timestamp_ns=time.time_ns(),age_s=now-t,
                frame=msg.header.frame_id,child=msg.child_frame_id,
                previous_stamp_s=previous,accepted=accepted,rejection_reason=reason,
                message_info=self.input_message_info.get('/gazebo/odometry'),callback_wall_s=time.monotonic()-began),allow_nan=False)))
            self.last_input_diag=began

    def radar_pose(self,msg):
        began=time.monotonic()
        try:
            self.joints.add_pose(msg,self.get_clock().now().nanoseconds*1e-9,began)
            action='accepted'
            # Preserve heading history promptly; do not transform/publish scans here.
            # Existing scan callback and 20 ms timer retain original deadlines.
        except (ValueError,TypeError,AttributeError) as e:action='rejected: '+str(e)
        self.heading_diag.publish(String(data=json.dumps(dict(action=action,
            sim_s=self.get_clock().now().nanoseconds*1e-9,stamp_s=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            receive_monotonic_s=began,callback_wall_s=time.monotonic()-began,scan_work_deferred_to_scan_or_timer=True,source='LIO-derived radar heading, NOT measured joint',
            fixed_body_yaw=float(self.mount.initial_base[3]),physical_body_heading_observed=False,callback_system_timestamp_ns=time.time_ns(),message_info=self.input_message_info.get('/localization/radar_pose')),allow_nan=False)))

    def scan_body_odom(self,msg):
        """Keep only actual validated body poses for scan-time association.

        Does not update control estimate, last_stage2_stamp, velocity, heading,
        or any control freshness clock. Same frame/finite/age acceptance as the
        existing Stage2 body adapter; no interpolation or extrapolation.
        """
        received=time.monotonic();now=self.get_clock().now().nanoseconds*1e-9
        accepted=False;reason=None;t=stamp_seconds(msg.header.stamp)
        try:
            pose,_=extract(msg)
            if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':raise ValueError('body history frame')
            if not math.isfinite(now-t) or not -.05<=now-t<=.2:raise ValueError('body history stale/future')
            if self.scan_body_history and t<=self.scan_body_history[-1][0]:raise ValueError('body history nonmonotonic')
            self.scan_body_history.append((t,*pose))
            while self.scan_body_history and t-self.scan_body_history[0][0]>.6+1e-9:self.scan_body_history.popleft()
            self.scan_body_history_fault=False;accepted=True
        except (ValueError,TypeError,AttributeError) as error:
            self.scan_body_history_fault=True;self.scan_body_rejected_count+=1;reason=str(error)
        if received-self.last_scan_body_diag>=.1 or not accepted:
            self.scan_body_diag.publish(String(data=json.dumps(dict(
                accepted=accepted,reason=reason,sim_s=now,stamp_s=t,receive_monotonic_s=received,
                history_count=len(self.scan_body_history),oldest_stamp_s=self.scan_body_history[0][0] if self.scan_body_history else None,
                newest_stamp_s=self.scan_body_history[-1][0] if self.scan_body_history else None,
                history_fault=self.scan_body_history_fault,rejected_count=self.scan_body_rejected_count,
                message_info=getattr(self.scan_body_history_tap,'info',None),
                source='/localization/lio_navigation_odometry; actual received validated messages only',
                scope='Scan association history ONLY; latest control state and all original age/gap guards unchanged'),allow_nan=False)))
            self.last_scan_body_diag=received

    def scan_callback(self,msg):
        # Bounded FIFO: a newer ray must not starve an older associable ray.
        received=time.monotonic()
        try:
            if msg.header.frame_id!='omni_robot/gimbal_link/lidar':raise ValueError('wrong scan frame')
            validation=copy.deepcopy(msg);validation.header.frame_id='omni_robot/base_link/lidar'
            scan_points(validation)
            t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if self.safety_scan is not None and t<=self.safety_scan['stamp']:raise ValueError('nonmonotonic scan')
            if self.pending_scans and t<=self.pending_scans[-1][0].header.stamp.sec+self.pending_scans[-1][0].header.stamp.nanosec*1e-9:raise ValueError('nonmonotonic pending scan')
            if len(self.pending_scans)>=4:raise ValueError('bounded scan queue overflow')
        except (ValueError,TypeError,AttributeError) as error:
            self.pending_scans=deque();self.scan_received_at=None;self.safety_scan_fault=True
            self.scan_diag.publish(String(data=json.dumps(dict(stage='rejected',reason=str(error),receive_monotonic_s=received))))
            return
        self.pending_scans.append((msg,received))
        self._flush_pending_scan()

    def _flush_pending_scan(self):
        # All original pending-scan checks run against the dedicated received
        # history. Do not replace latest-control state or leave the alias set.
        saved=self.odom_history
        self.odom_history=deque() if self.scan_body_history_fault else self.scan_body_history
        try:self._flush_pending_scan_using_history()
        finally:self.odom_history=saved

    def _flush_pending_scan_using_history(self):
        pending=getattr(self,'pending_scans',None)
        while pending:
            msg,received=pending[0];t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            now=self.get_clock().now().nanoseconds*1e-9;steady=time.monotonic()
            if not (-.05<=now-t<=.2 and 0<=steady-received<=.2):
                pending.popleft();self.scan_received_at=None;self.safety_scan_fault=True
                self.scan_diag.publish(String(data=json.dumps(dict(stage='expired',stamp_s=t,sim_s=now,
                    receive_monotonic_s=received,process_monotonic_s=steady,reason='original scan sim/receive deadline'))))
                continue
            try:
                self.joints.at(t,now,steady)
                if not self.odom_history:raise ValueError('body pose history unavailable')
                nearest=min(self.odom_history,key=lambda p:abs(p[0]-t))
                if abs(nearest[0]-t)>.025:raise ValueError('scan/base association exceeds25ms')
            except ValueError as error:
                if steady-self.last_scan_diag>=.1:
                    self.scan_diag.publish(String(data=json.dumps(dict(stage='deferred',stamp_s=t,sim_s=now,
                        receive_monotonic_s=received,process_monotonic_s=steady,reason=str(error),queued_scans=len(pending),
                        last_heading_stamp_s=self.joints.samples[-1][0] if self.joints.samples else None))))
                    self.last_scan_diag=steady
                return
            pending.popleft()
            self._consume_associated_scan(msg,received)

    def _prepare_scan_ray_cache(self,msg):
        # Pure, per-scan memoization of the original ray predicate. The parent
        # still owns validation, association and all actual state/publication.
        if self.grid_route is None:return None
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        angle,_=self.joints.at(t,self.get_clock().now().nanoseconds*1e-9,time.monotonic())
        matched=min(self.odom_history,key=lambda p:abs(p[0]-t))
        if abs(matched[0]-t)>.025:raise ValueError('scan/base association exceeds25ms')
        x,y,yaw=self.mount.sensor_planar_pose(matched[1:],angle)
        angles=[];distances=[]
        for i,d in enumerate(msg.ranges):
            if math.isfinite(d) and msg.range_min<d<msg.range_max:
                angles.append(yaw+msg.angle_min+i*msg.angle_increment);distances.append(d)
        answers=ray_answers(self.grid_route,(x,y),angles,distances)
        cache={((x,y),a,d,.60):v for a,d,v in zip(angles,distances,answers)}
        original=self.grid_route.ray_has_unmapped_return
        def cached(origin,angle,distance,margin=.60):
            key=(tuple(origin),angle,distance,margin)
            # A parent association/argument difference falls back to its exact
            # original query, never an approximate or previous-scan answer.
            return cache[key] if key in cache else original(origin,angle,distance,margin)
        return cached

    def _consume_associated_scan(self,msg,received):
        began=time.monotonic();t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        history=self.odom_history
        closest=min(history,key=lambda p:abs(p[0]-t))[0] if history else None
        samples=self.joints.samples
        first=samples[0][0] if samples else None;last=samples[-1][0] if samples else None
        self.joints.last_scan_association=None
        original_ray=None
        if self.grid_route is not None:
            original_ray=self.grid_route.ray_has_unmapped_return
            try:self.grid_route.ray_has_unmapped_return=self._prepare_scan_ray_cache(msg)
            except ValueError:pass # Parent still independently rejects bad association.
        try:
            super().scan_callback(msg)
        finally:
            if original_ray is not None:self.grid_route.ray_has_unmapped_return=original_ray
        if self.safety_scan is not None and self.safety_scan['stamp']==t and not self.safety_scan_fault:
            self.safety_scan['received']=received
            self.scan_received_at=received
            self.dynamic_scan_received_at=received
        association=self.joints.last_scan_association
        accepted=self.safety_scan is not None and self.safety_scan['stamp']==t and not self.safety_scan_fault
        if not accepted or began-self.last_scan_diag>=.1:
            self.scan_diag.publish(String(data=json.dumps(dict(
                stamp_s=t,sim_s=self.get_clock().now().nanoseconds*1e-9,
                receive_monotonic_s=received,process_monotonic_s=began,frame=msg.header.frame_id,
                accepted=accepted,heading_association=association,
                first_heading_stamp_s=first,last_heading_stamp_s=last,
                nearest_body_stamp_s=closest,body_time_difference_s=None if closest is None else closest-t,
                last_accepted_scan_s=None if self.safety_scan is None else self.safety_scan['stamp'],
                body_history_source='dedicated actual LIO body subscription; control unchanged',
                body_history_count=len(history),
                scan_fault=self.safety_scan_fault,callback_wall_s=time.monotonic()-began),allow_nan=False)))
            self.last_scan_diag=began

    def localization_fresh(self):
        # Call the original production gate exactly once, preserve its result.
        result=super().localization_fresh()
        now=self.get_clock().now().nanoseconds*1e-9;steady=time.monotonic()
        report=dict(result=result,sim_s=now,monotonic_s=steady,
            approach_qualified_clock=copy.deepcopy(getattr(self,'_approach_executable_clock',None)),
            heading_latest_call=copy.deepcopy(getattr(self.joints,'last_latest_check',None)),
            body_stamp_s=self.last_stage2_stamp,
            body_receive_age_s=None if self.last_lio_received is None else steady-self.last_lio_received,
            correction_stamp_s=self.last_match_sim,
            correction_sim_age_s=None if self.last_match_sim is None else now-self.last_match_sim,
            correction_receive_age_s=None if self.last_match_steady is None else steady-self.last_match_steady)
        capture=getattr(self,'goal_freshness_capture',None)
        if capture is not None:capture.append(report)
        if hasattr(self,'freshness_diag') and steady-self.last_freshness_diag>=.1:
            self.freshness_diag.publish(String(data=json.dumps(report,allow_nan=False)))
            self.last_freshness_diag=steady
        return result

    def planned_trajectory_callback(self,marker):
        # Radar-only replacement of the qualified scalar path calculation.
        # Keep parent marker gates, every .05s rectangle sample, LIO reserve
        # and gimbal approach identity update; no latest-pose/scan work here.
        started=time.monotonic();before=self.planned_path_received_at;decision='ignored'
        try:
            if not self.provincial_rect_guard or not marker.points or self.active_waypoint is None:return
            if marker.header.frame_id not in ('odom','map'):return
            if stamp_seconds(marker.header.stamp)<=stamp_seconds(self.active_waypoint.header.stamp):return
            q=self.active_waypoint.pose.orientation
            goal_yaw=math.atan2(2.*(q.w*q.z+q.x*q.y),1.-2.*(q.y*q.y+q.z*q.z))
            try:
                minimum=path_minimum_gap(marker.points,self.yaw,wrap_angle(goal_yaw-self.yaw),
                    self.provincial_walls,[self.provincial_min_body_gap,
                    self.provincial_min_body_gap+self.pose_safety_margin])
            except ValueError:
                # Invalid path never refreshes or inherits a safe old plan.
                decision='invalid';self.planned_path_min_gap=0.;self.planned_path_safe=False
                self.planned_path_received_at=None;self.planned_path_valid_until=None
                return
            decision='validated'
            self.planned_path_min_gap=minimum
            self.planned_path_safe=minimum>=self.provincial_min_body_gap+self.pose_safety_margin
            self.planned_path_received_at=time.monotonic()
            self.planned_path_valid_until=stamp_seconds(marker.header.stamp)+max(.2,(len(marker.points)-1)*.05)
            if self.planned_path_received_at!=before and self.planned_path_safe is True:
                endpoint=marker.points[-1];target=self.active_waypoint.pose.position
                if math.hypot(endpoint.x-target.x,endpoint.y-target.y)<=.05:
                    self.approach_plan_identity=self._approach_identity()
        finally:
            elapsed=time.monotonic()-started
            if decision!='ignored' and hasattr(self,'path_validation_diag'):
                self.path_validation_diag.publish(String(data=json.dumps(dict(
                    stage=decision,point_count=len(marker.points),callback_wall_s=elapsed,
                    min_gap_m=self.planned_path_min_gap,planned_path_safe=self.planned_path_safe,
                    stamp_s=stamp_seconds(marker.header.stamp),
                    sim_s=self.get_clock().now().nanoseconds*1e-9,
                    route=self.route_diag_id,waypoint=self.active_waypoint_diag_id,
                    scope='Same .05s samples; exact SAT/edge geometry with AABB pruning; scalar fallback near acceptance boundaries'),allow_nan=False)))
            if elapsed>.05:
                self.get_logger().warn(f'RMUC callback timing: method=planned_trajectory_callback, duration_s={elapsed:.6f}, sim_s={self.get_clock().now().nanoseconds*1e-9:.6f}')

    def _retry_measurements_ready(self):
        # Mirror measurement prerequisites of the existing final sensor gate.
        # Missing plans / unsafe geometry remain executable retry failures;
        # unavailable measurements must not spend the task retry budget.
        now=self.get_clock().now().nanoseconds*1e-9;steady=time.monotonic()
        scan=self.safety_scan
        if (self.safety_scan_fault or scan is None or
                not 0 <= now-scan['stamp'] <= .2 or
                not steady-scan['received'] <= .2):
            return False
        return bool(self.estimate is not None and self.last_stage2_stamp is not None
            and all(math.isfinite(v) for v in self.estimate)
            and -.05 <= now-self.last_stage2_stamp <= .05)

    def observe_guard_resume(self,action,now_steady):
        if self._task_localization_fresh() and not self._retry_measurements_ready():
            # Original update has already published its independently guarded
            # zero and odometry. Skip only completion/retry/recovery tail.
            super().observe_guard_resume('stale_stop',now_steady)
            raise _LocalizationTaskPaused()
        super().observe_guard_resume(action,now_steady)

    def _publish_hold_clock(self,phase,now,hold):
        self.localization_diag.publish(String(data=json.dumps(dict(
            event='localization_task_clock',phase=phase,
            route=hold['identity'][0],waypoint=hold['identity'][1],
            monotonic_s=now,sim_s=self.get_clock().now().nanoseconds*1e-9,
            hold_duration_s=max(0.,now-hold['started_at']),
            stage_sent_at_before_hold=hold['sent_at'],
            progress_at_before_hold=hold['progress_at'],
            measurements_ready=getattr(self,'_task_measurements_ready',None),
            reason='localization or scan/map-pose measurement unavailable; retry clocks paused',
            scope='Same existing scan/pose freshness prerequisites; geometry and missing-plan retries retained'),allow_nan=False)))

    def _sync_retry_hold_clock(self,fresh,now):
        # Pause executable deadlines; retain time already earned before a hold.
        # Repeated brief outages must not grant a whole new retry window.
        identity=self._stage_identity()
        hold=getattr(self,'_localization_stage_hold',None)
        if hold is not None and hold['identity']!=identity:
            self._publish_hold_clock('discard_previous_task',now,hold)
            hold=self._localization_stage_hold=None
        if identity is None:
            return
        if not fresh:
            if hold is None:
                hold=self._localization_stage_hold=dict(identity=identity,started_at=now,
                    sent_at=self.stage_sent_at,progress_at=getattr(self,'stage_last_progress_at',None))
                self._publish_hold_clock('paused',now,hold)
            if hold['sent_at'] is not None:
                self.stage_sent_at=now
            if hold['progress_at'] is not None:
                self.stage_last_progress_at=now
        elif hold is not None:
            paused=max(0.0,now-hold['started_at'])
            if hold['sent_at'] is not None:
                self.stage_sent_at=hold['sent_at']+paused
            if hold['progress_at'] is not None:
                self.stage_last_progress_at=hold['progress_at']+paused
            self._publish_hold_clock('resumed',now,hold)
            self._localization_stage_hold=None

    def _sync_stage_clock(self,fresh,now):
        # Internal approach clock is separate from retry/progress hold clocks.
        # Preserve elapsed retry time across localization holds.
        measurements_ready=self._retry_measurements_ready()
        fresh=bool(fresh and measurements_ready)
        self._task_measurements_ready=measurements_ready
        self._sync_retry_hold_clock(fresh,now)
        identity=self._stage_identity()
        previous=getattr(self,'_approach_executable_clock',None)
        if identity is None or self.stage_sent_at is None or not math.isfinite(now):
            self._approach_executable_clock=None
            return
        if previous is None or previous['identity']!=identity or now<previous['last']:
            previous={'identity':identity,'elapsed':0.,'last':now,'fresh':False}
        dt=now-previous['last']
        # No extrapolation through missing control ticks or a pause interval.
        if fresh and previous['fresh'] and 0<=dt<=.2:
            previous['elapsed']+=dt
        previous.update(last=now,fresh=bool(fresh))
        self._approach_executable_clock=previous

    def internal_waypoint_handoff_ready(self,distance,speed):
        # Final acceptance needs the same guarded centimetre-scale settling
        # already used at internal corners. The 5cm truth audit is unchanged;
        # truth is not an input to this odometry-only completion decision.
        if self.waypoints:
            return super().internal_waypoint_handoff_ready(distance,speed)
        return distance < .008 and speed < .01

    def waypoint_approach_eligible(self,remaining):
        if self.waypoints:
            return super().waypoint_approach_eligible(remaining)
        # Cover a spline that stops inside the legacy 10cm activation band.
        return .008 < remaining < .40

    def waypoint_approach_delay_elapsed(self,now):
        if self.stage_sent_at is None or self.active_waypoint is None or not self._task_localization_fresh():return False
        if super().waypoint_approach_delay_elapsed(now):return True
        state=getattr(self,'_approach_executable_clock',None)
        return bool(state is not None and state['identity']==self._stage_identity() and state['fresh'] and 0<=now-state['last']<=.2 and state['elapsed']>3.)

    def route_goal(self,goal):
        try:
            q=goal.pose.orientation;yaw=planar_yaw(rotation([q.x,q.y,q.z,q.w]))
            if abs(wrap(yaw-self.mount.initial_base[3]))>.03:raise ValueError('goal requires physical chassis rotation')
        except ValueError as e:
            self.get_logger().warn('GOAL REJECTED: '+str(e));return
        checks=[];self.goal_freshness_capture=checks
        before=self.route_diag_id
        try:LioSafeNavigationInterface.route_goal(self,goal)
        finally:
            self.goal_freshness_capture=None
            self.goal_freshness_diag.publish(String(data=json.dumps(dict(
                event='production_route_goal_freshness',checks=checks,
                goal_xy=[goal.pose.position.x,goal.pose.position.y],
                route_before=before,route_after=self.route_diag_id,
                source='original localization_fresh return values at actual goal callback; no acceptance override'),allow_nan=False)))

    def joint(self,msg):
        raise RuntimeError('host encoder callback prohibited in fixed-heading radar mode')

def main():
    rclpy.init();node=RadarFixedNavigation()
    from rmw_diagnostic_executor import RmwDiagnosticExecutor
    executor=RmwDiagnosticExecutor();executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
