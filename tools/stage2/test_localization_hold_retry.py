"""Real production update/task callbacks under controlled isolated clocks."""
import json
import unittest
from geometry_msgs.msg import Twist
from localization_hold_fixture import Clock, full_node, isolated, tick, goal
from localization_hold_fixture import ROOT, FIELD
from grid_route import GridRoute


class LocalizationHoldTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.n,self.logs=full_node(self.clock)
        self.context=isolated(self.clock,self.logs);self.context.__enter__()
    def tearDown(self):self.context.__exit__(None,None,None)
    def stopped(self):
        m=self.n.drive_pub.messages[-1]
        self.assertEqual((m.linear.x,m.linear.y,m.angular.z),(0.,0.,0.))
    def update(self,t,fresh=False):tick(self.n,self.clock,t,fresh)

    def test_long_loss_no_retry_or_dispatch(self):
        target=self.n.active_waypoint;header=target.header.stamp
        for t in (10.,11.,15.,30.):self.update(t);self.stopped()
        self.assertEqual(self.n.stage_retry_count,0);self.assertEqual(self.n.route_pub.messages,[])
        self.assertIs(self.n.active_waypoint,target);self.assertEqual(target.header.stamp,header)
        self.assertEqual(len(self.n.odom_pub.messages),4)
        self.assertFalse(any('Retrying' in text for _,text in self.logs))

    def test_failed_input_keeps_stop_then_fresh_plan_allows_motion(self):
        self.update(10.);self.n.scan_input_fault=True
        # A recent stamp alone cannot clear a hard-format fault.
        self.n.last_match_sim=self.n.last_match_steady=10.1
        self.update(10.1);self.stopped()
        self.n.planned_path_safe=False
        self.update(12.,True);self.stopped()
        self.assertEqual(self.n.stage_sent_at,12.)
        self.n.planned_path_safe=True;self.update(12.02,True)
        self.assertGreater(self.n.drive_pub.messages[-1].linear.x,0)
        self.assertEqual(self.n.stage_retry_count,0)

    def test_missing_plan_after_resume_does_not_move(self):
        self.update(10.);self.n.planned_path_valid_until=None
        self.update(12.,True);self.stopped()
        self.assertEqual(json.loads(self.n.actuation_diag_pub.messages[-1].data)['action'],'missing_plan')

    def test_old_trajectory_after_resume_does_not_move(self):
        self.update(10.);self.n.planned_path_valid_until=11.
        self.update(12.,True);self.stopped()

    def test_real_no_progress_retries_after_original_4s(self):
        self.update(10.);self.update(12.,True)
        self.update(15.99,True);self.assertEqual(self.n.route_pub.messages,[])
        self.update(16.01,True)
        self.assertEqual(self.n.stage_retry_count,1);self.assertEqual(len(self.n.route_pub.messages),1)
        self.assertEqual(self.n.active_waypoint_diag_id,'R0001:W00')
        self.assertTrue(any('Retrying' in text for _,text in self.logs))

    def test_repeated_loss_resets_only_on_new_hold_resume(self):
        self.update(10.);self.update(12.,True);self.assertEqual(self.n.stage_sent_at,12.)
        self.update(13.,True);self.assertEqual(self.n.stage_sent_at,12.)
        self.n.scan_input_fault=True;self.update(14.);self.update(25.)
        self.update(26.,True);self.assertEqual(self.n.stage_sent_at,26.)
        self.update(27.,True);self.assertEqual(self.n.stage_sent_at,26.)
        phases=[json.loads(m.data)['phase'] for m in self.n.localization_diag.messages]
        self.assertEqual(phases,['paused','resumed','paused','resumed'])
        self.assertEqual(self.n.stage_retry_count,0)

    def test_different_route_does_not_restore_old_clock(self):
        self.update(10.);self.n.route_diag_id='R0002';self.n.active_waypoint_diag_id='R0002:W00'
        self.n.active_waypoint=goal(y=1.5);self.n.stage_sent_at=11.;self.n.stage_last_progress_at=11.
        self.n.stage_best_distance=.83;self.update(12.,True)
        self.assertEqual(self.n.stage_sent_at,11.)
        self.assertEqual(self.n.route_diag_id,'R0002');self.assertEqual(self.n.stage_retry_count,0)

    def test_same_coordinates_new_waypoint_object_is_distinct(self):
        self.update(10.);self.n.active_waypoint=goal();self.n.stage_sent_at=11.;self.n.stage_last_progress_at=11.
        self.update(12.,True);self.assertEqual(self.n.stage_sent_at,11.)

    def test_waypoint_change_does_not_inherit_exhausted_none(self):
        self.n.stage_sent_at=None;self.update(10.)
        self.n.active_waypoint=goal(y=1.8);self.n.active_waypoint_diag_id='R0001:W01'
        self.n.stage_sent_at=11.;self.n.stage_last_progress_at=11.;self.n.stage_best_distance=1.13
        self.update(12.,True);self.assertEqual(self.n.stage_sent_at,11.)

    def test_exhausted_stage_not_revived_or_retry_counter_reset(self):
        self.n.stage_sent_at=None;self.n.stage_retry_count=4
        self.update(10.);self.update(12.,True)
        self.assertIsNone(self.n.stage_sent_at);self.assertEqual(self.n.stage_retry_count,4)
        self.assertEqual(self.n.route_pub.messages,[])

    def test_initial_idle_stops_without_clock_side_effect(self):
        self.n.active_waypoint=None;self.n.active_waypoint_diag_id=None;self.n.route_diag_id=None
        self.n.waypoints=[];self.n.stage_sent_at=None;self.n.stage_last_progress_at=None
        self.update(10.);self.stopped();self.assertEqual(self.n.localization_diag.messages,[])

    def test_completed_task_stops_and_drops_saved_hold(self):
        self.update(10.);self.n.active_waypoint=None;self.n.active_waypoint_diag_id=None
        self.n.waypoints=[];self.n.stage_sent_at=None;self.n.stage_last_progress_at=None
        self.update(12.,True);self.stopped()
        self.assertIsNone(self.n._localization_stage_hold);self.assertIsNone(self.n.stage_sent_at)

    def test_near_goal_loss_cannot_false_complete_or_dispatch_next(self):
        self.n.estimate=(4.7,1.14,self.n.yaw);self.n.x=4.7;self.n.y=1.14
        self.n.stage_best_distance=.01;self.update(10.);self.stopped()
        self.assertEqual(self.n.active_waypoint_diag_id,'R0001:W00')
        self.assertEqual(len(self.n.waypoints),1);self.assertEqual(self.n.route_pub.messages,[])
        self.assertFalse(any('diagnostic completed' in text for _,text in self.logs))

    def test_fresh_near_goal_can_complete_and_send_next_normally(self):
        self.n.estimate=(4.7,1.14,self.n.yaw);self.n.x=4.7;self.n.y=1.14
        self.n.stage_best_distance=.01;self.update(10.)
        self.n.command=Twist();self.update(12.,True)
        self.assertEqual(self.n.active_waypoint_diag_id,'R0001:W01')
        self.assertEqual(len(self.n.route_pub.messages),1)
        self.assertTrue(any('diagnostic completed' in text for _,text in self.logs))

    def test_invalid_localization_does_not_dispatch_pending_only(self):
        self.n.active_waypoint=None;self.n.active_waypoint_diag_id=None;self.n.stage_sent_at=None
        self.update(10.);self.stopped();self.assertEqual(self.n.route_pub.messages,[])
        self.assertEqual(len(self.n.waypoints),1)
        self.update(12.,True);self.assertEqual(len(self.n.route_pub.messages),1)

    def test_invalid_localization_cannot_requeue_using_old_scan(self):
        self.n.dynamic_scan_received_at=10.;self.n.dynamic_obstacle_points=[(4.7,1.15)]
        self.update(10.);self.assertEqual(self.n.active_waypoint_diag_id,'R0001:W00')
        self.assertEqual(len(self.n.waypoints),1);self.assertEqual(self.n.route_pub.messages,[])

    def test_valid_dynamic_blocking_still_works(self):
        self.n.dynamic_scan_received_at=10.;self.n.dynamic_obstacle_points=[(4.7,1.15)]
        self.update(10.,True)
        self.assertEqual(self.n.active_waypoint_diag_id,'R0001:W01')
        self.assertEqual(len(self.n.route_pub.messages),1)

    def test_expired_upstream_command_still_stops(self):
        self.clock.t=10.;self.n.last_command_time=1.
        self.n.last_match_sim=self.n.last_match_steady=10.
        self.n.latest_odom.header.stamp=self.clock.stamp();self.n.update();self.stopped()

    def test_wall_expiry_stops_when_sim_paused(self):
        self.n.last_match_sim=10.;self.n.last_match_steady=1.;self.update(10.);self.stopped()

    def test_unrelated_update_exception_propagates_and_restores_sensor(self):
        received=self.n.last_odom_received
        def failure(message):raise ValueError('test transport error')
        self.n.drive_pub.publish=failure
        with self.assertRaisesRegex(ValueError,'test transport error'):self.n.update()
        self.assertEqual(self.n.last_odom_received,received)
        self.assertIsNone(self.n._localization_update_fresh)

    def test_stale_goal_callback_rejects_without_changing_original_task(self):
        original=self.n.active_waypoint;self.update(10.)
        self.n.route_goal(goal(y=1.35))
        self.assertIs(self.n.active_waypoint,original);self.assertEqual(self.n.route_diag_id,'R0001')
        self.assertEqual(self.n.route_pub.messages,[])
        self.assertTrue(any('GOAL REJECTED: LOCALIZATION NOT FRESH' in text for _,text in self.logs))

    def test_actual_new_goal_callback_uses_new_task_clock(self):
        self.update(10.)
        self.n.field=FIELD;self.n.route_diag_sequence=1
        self.n.grid_route=GridRoute(ROOT/'src/uav_bringup/maps/provincial_2025_provisional.pgm',clearance=.4)
        self.clock.t=11.;self.n.last_match_sim=self.n.last_match_steady=11.
        self.n.route_goal(goal(y=1.35))
        self.assertEqual(self.n.route_diag_id,'R0002');self.assertEqual(self.n.stage_sent_at,11.)
        self.update(12.,True);self.assertEqual(self.n.stage_sent_at,11.)
        self.assertEqual(len(self.n.route_pub.messages),1);self.stopped()  # new safe plan still required

    def test_exhausted_recovery_cannot_skip_stage_while_stale(self):
        self.n.stage_retry_count=4;self.update(10.);self.update(30.)
        self.assertEqual(self.n.stage_retry_count,4);self.assertEqual(self.n.active_waypoint_diag_id,'R0001:W00')
        self.assertEqual(self.n.route_pub.messages,[]);self.assertEqual(len(self.n.waypoints),1)

    def test_wrong_frame_callback_stops_full_task_tail(self):
        from scan_guard_fixture import recorded_scan
        self.n.last_match_sim=self.n.last_match_steady=10.
        bad=recorded_scan();bad.header.frame_id='camera';bad.header.stamp=self.clock.stamp()
        self.n.scan_callback(bad);self.update(10.)
        self.assertTrue(self.n.scan_input_fault);self.stopped()
        self.assertEqual(self.n.route_pub.messages,[])

    def test_task_clock_diagnostics_are_transitions_not_heartbeat_resets(self):
        self.update(10.);self.update(10.1);self.update(11.);self.update(12.,True)
        self.update(12.1,True)
        events=[json.loads(m.data) for m in self.n.localization_diag.messages]
        self.assertEqual([e['phase'] for e in events],['paused','resumed'])
        self.assertEqual(events[-1]['hold_duration_s'],2.)
        self.assertEqual(self.n.stage_sent_at,12.)


if __name__=='__main__':unittest.main()
