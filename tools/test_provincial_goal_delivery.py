"""Production runner handoff tests; DDS integration uses a private ROS domain.

No Gazebo is started. Synthetic telemetry and goals only reach isolated stubs.
"""
import json
import math
import os
import signal
import subprocess
import tempfile
import threading
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, LivelinessPolicy
from rcl_interfaces.msg import Log
from rosgraph_msgs.msg import Clock

from run_provincial_low_speed_validation import Runner, Odometry, Twist, PoseStamped
from run_provincial_full_observed import ROOT, INPUTS, local_input_closure, run_exit_code


def endpoint(name, gid=1, qos=None):
    return types.SimpleNamespace(
        node_name=name, node_namespace='/',
        topic_type='geometry_msgs/msg/PoseStamped', endpoint_gid=[gid]*16,
        qos_profile=qos or QoSProfile(depth=5, liveliness=LivelinessPolicy.AUTOMATIC))


def readiness_state(endpoints, matched):
    return types.SimpleNamespace(
        get_subscriptions_info_by_topic=lambda _: endpoints,
        get_publishers_info_by_topic=lambda _: [endpoint('provincial_low_speed_validation', 3)],
        get_name=lambda: 'provincial_low_speed_validation', get_namespace=lambda: '/',
        goal_pub=types.SimpleNamespace(qos_profile=QoSProfile(depth=10),
                                     get_subscription_count=lambda: matched),
        receiver_stable_since=None, receiver_signature=None,
        goal_delivery={})


def route_log(goal=(8.7, 4.25), route='R0001', logger='gazebo_navigation_interface',
              at_ns=None):
    msg = Log()
    msg.name = logger
    msg.msg = (f'RMUC route diagnostic route={route} planned='
               f'{route}:W00=(4.700,1.800);{route}:W01=(8.700,1.800);'
               f'{route}:W02=({goal[0]:.3f},{goal[1]:.3f})')
    ns = time.time_ns() if at_ns is None else at_ns
    msg.stamp.sec, msg.stamp.nanosec = divmod(ns, 1_000_000_000)
    return msg


def acceptance_state():
    state = types.SimpleNamespace(
        goal=(8.7, 4.25), accepted=False, final_waypoint_id=None,
        acceptance_timeout_s=5.0,
        start_wall=time.monotonic(), seen_route_ids=set(),
        goal_delivery={'publish_wall_ns': time.time_ns()-1_000_000,
                       'acceptance': None, 'ignored_events': []})
    state.fresh_navigation_event = types.MethodType(Runner.fresh_navigation_event, state)
    return state


class ProductionLogicTests(unittest.TestCase):
    def test_self_subscription_never_means_navigation_ready(self):
        state = readiness_state([endpoint('provincial_low_speed_validation')], 1)
        self.assertFalse(Runner.goal_receiver_ready(state, 1))
        self.assertFalse(Runner.goal_receiver_ready(state, 100))

    def test_unrelated_observer_does_not_satisfy_readiness(self):
        state = readiness_state([endpoint('provincial_low_speed_validation'),
                                 endpoint('another_observer', 2)], 2)
        self.assertFalse(Runner.goal_receiver_ready(state, 1))
        self.assertFalse(Runner.goal_receiver_ready(state, 100))

    def test_named_compatible_receiver_requires_stable_matches(self):
        state = readiness_state([endpoint('provincial_low_speed_validation'),
                                 endpoint('gazebo_navigation_interface', 2)], 2)
        self.assertFalse(Runner.goal_receiver_ready(state, 1))
        self.assertTrue(Runner.goal_receiver_ready(state, 1.6))
        state.goal_pub.get_subscription_count = lambda: 1
        self.assertFalse(Runner.goal_receiver_ready(state, 1.7))

    def test_same_gid_name_resolution_starts_stability_timer(self):
        unresolved = endpoint('not_yet_resolved', 2)
        state = readiness_state([endpoint('provincial_low_speed_validation'), unresolved], 2)
        self.assertFalse(Runner.goal_receiver_ready(state, 1))
        # DDS may learn the name/QoS after the GID is already present.
        unresolved.node_name = 'gazebo_navigation_interface'
        self.assertFalse(Runner.goal_receiver_ready(state, 2))
        self.assertTrue(Runner.goal_receiver_ready(state, 2.6))

    def test_same_gid_match_recovery_restarts_stability_timer(self):
        state = readiness_state([endpoint('provincial_low_speed_validation'), endpoint('gazebo_navigation_interface',2)], 1)
        self.assertFalse(Runner.goal_receiver_ready(state, 1))
        state.goal_pub.get_subscription_count = lambda: 2
        self.assertFalse(Runner.goal_receiver_ready(state, 2))
        self.assertTrue(Runner.goal_receiver_ready(state, 2.6))

    def test_incompatible_qos_and_ambiguous_named_nodes_fail(self):
        qos = QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        state = readiness_state([endpoint('gazebo_navigation_interface', qos=qos)], 1)
        self.assertFalse(Runner.goal_receiver_ready(state, 1))
        state = readiness_state([endpoint('gazebo_navigation_interface'),
                                 endpoint('gazebo_navigation_interface', 2)], 2)
        self.assertFalse(Runner.goal_receiver_ready(state, 1))

    def test_stale_wrong_logger_goal_and_previous_route_rejected(self):
        for kind in ('stale', 'logger', 'goal', 'route', 'deadline'):
            with self.subTest(kind=kind):
                state = acceptance_state()
                msg = route_log()
                if kind == 'stale':
                    msg.stamp.sec = 0
                if kind == 'logger':
                    msg.name = 'other_node'
                if kind == 'goal':
                    msg = route_log(goal=(4.7, 1.15))
                if kind == 'route':
                    state.seen_route_ids.add('R0001')
                if kind == 'deadline':
                    state.start_wall -= 6
                Runner.accept_route_event(state, msg)
                self.assertFalse(state.accepted)
                self.assertTrue(state.goal_delivery['ignored_events'])

    def test_matching_fresh_ack_accepted_once(self):
        state = acceptance_state()
        msg = route_log()
        Runner.accept_route_event(state, msg)
        Runner.accept_route_event(state, msg)
        self.assertTrue(state.accepted)
        self.assertEqual(state.final_waypoint_id, 'R0001:W02')
        self.assertEqual(state.goal_delivery['ignored_events'], [])

    def test_capture_includes_previously_missed_local_dependencies(self):
        files = set(local_input_closure(INPUTS))
        for path in ('tools/analyze_provincial_forward_clearance.py',
                     'src/uav_planning/scripts/scan_matcher.py',
                     'src/uav_planning/scripts/planar_navigation_simulator.py',
                     'install/uav_planning/lib/uav_planning/scan_matcher.py',
                     'install/uav_planning/lib/uav_planning/planar_navigation_simulator.py'):
            self.assertIn(path, files)

    def test_launcher_propagates_trial_failure(self):
        progress = {'status': 'RUN_FINISHED_PENDING_INDEPENDENT_AUDIT',
                    'runner_returncode': 2, 'remaining_gazebo_servers': []}
        self.assertEqual(run_exit_code(progress), 2)
        progress['runner_returncode'] = 0
        self.assertEqual(run_exit_code(progress), 0)


class DDSDeliveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID') != '187' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('tests require ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_actual_runner_no_ack_stops_early_without_resend(self):
        with tempfile.TemporaryDirectory() as folder:
            runner = Runner('full', 'silent_peer', 1, 'hold-start', Path(folder))
            try:
                odom = Odometry()
                odom.header.stamp.sec = 1
                odom.pose.pose.position.x = 4.7
                odom.pose.pose.position.y = .5
                odom.pose.pose.orientation.z = math.sin(math.pi/4)
                odom.pose.pose.orientation.w = math.cos(math.pi/4)
                runner.gt = runner.odom = odom
                runner.acceptance_timeout_s = .15
                with patch.object(runner, 'wait_ready', return_value=True), \
                     patch.object(runner, 'goal_receiver_ready', return_value=True), \
                     patch.object(runner, 'get_node_names', return_value=['ego_planner_node']), \
                     patch.object(runner.goal_pub, 'publish') as publish:
                    started = time.monotonic()
                    result = runner.run()
                self.assertEqual(result, 'goal_acceptance_timeout')
                self.assertLess(time.monotonic()-started, .8)
                self.assertEqual(publish.call_count, 1)
                self.assertEqual(runner.goal_delivery['publish_count'], 1)
                self.assertFalse(runner.accepted)
            finally:
                runner.destroy_node()

    def test_delayed_real_dds_receiver_observed_and_acknowledged(self):
        peer = Node('gazebo_navigation_interface')
        planner = Node('ego_planner_node')
        gt_pub = peer.create_publisher(Odometry, '/gazebo/odometry', 100)
        odom_pub = peer.create_publisher(Odometry, '/ground/odometry', 100)
        final_pub = peer.create_publisher(Twist, '/model/omni_robot/cmd_vel', 100)
        clock_pub = peer.create_publisher(Clock, '/clock', 100)
        started = time.monotonic()
        state = {'sim_t': 1., 'subscription': None, 'goals': []}

        def goal(msg):
            state['goals'].append((time.monotonic(), msg))
            peer.get_logger().info(route_log().msg)

        def tick():
            state['sim_t'] += .02
            if state['subscription'] is None and time.monotonic()-started >= .8:
                state['subscription'] = peer.create_subscription(PoseStamped, '/goal_pose', goal, 5)
                state['receiver_created_at'] = time.monotonic()
            odom = Odometry()
            ns = int(state['sim_t']*1e9)
            odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(ns, 1_000_000_000)
            odom.pose.pose.position.x, odom.pose.pose.position.y = 4.7, .5
            odom.pose.pose.orientation.z = math.sin(math.pi/4)
            odom.pose.pose.orientation.w = math.cos(math.pi/4)
            clock = Clock()
            clock.clock = odom.header.stamp
            clock_pub.publish(clock)
            gt_pub.publish(odom)
            odom_pub.publish(odom)
            final_pub.publish(Twist())

        timer = peer.create_timer(.02, tick)
        executor = SingleThreadedExecutor()
        executor.add_node(peer)
        executor.add_node(planner)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        runner = None
        try:
            with tempfile.TemporaryDirectory() as folder:
                runner = Runner('full', 'isolated_dds', 1.2, 'hold-start',
                                Path(folder), raw_every_message=True)
                result = runner.run()
                self.assertEqual(result, 'timeout')  # stationary stub, not navigation PASS
                self.assertTrue(runner.accepted)
                self.assertEqual(len(state['goals']), 1)
                self.assertGreaterEqual(state['goals'][0][0]-state['receiver_created_at'], .5)
                self.assertEqual(runner.goal_delivery['acceptance']['route_id'], 'R0001')
                native = [json.loads(line) for line in
                          (Path(folder)/'isolated_dds.native.jsonl').read_text().splitlines()]
                self.assertEqual(sum(row['topic'] == '/goal_pose' for row in native), 1)
                self.assertTrue(any(row['topic'] == '/rosout/relevant' for row in native))
        finally:
            if runner is not None:
                if runner.native_trace:
                    runner.native_trace.close()
                runner.destroy_node()
            executor.shutdown(timeout_sec=3)
            thread.join(timeout=3)
            peer.destroy_timer(timer)
            peer.destroy_node()
            planner.destroy_node()

    def test_real_navigation_process_accepts_goal_with_synthetic_telemetry(self):
        """Only delivery/routing is exercised; no planner or physical motion."""
        telemetry = Node('isolated_telemetry')
        planner_presence = Node('ego_planner_node')
        gt_pub = telemetry.create_publisher(Odometry, '/gazebo/odometry', 100)
        clock_pub = telemetry.create_publisher(Clock, '/clock', 100)
        state = {'sim_t': 1.}

        def tick():
            state['sim_t'] += .02
            odom = Odometry()
            ns = int(state['sim_t']*1e9)
            odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(ns, 1_000_000_000)
            odom.pose.pose.position.x, odom.pose.pose.position.y = 4.7, .5
            odom.pose.pose.orientation.z = math.sin(math.pi/4)
            odom.pose.pose.orientation.w = math.cos(math.pi/4)
            clock = Clock()
            clock.clock = odom.header.stamp
            clock_pub.publish(clock)
            gt_pub.publish(odom)

        timer = telemetry.create_timer(.02, tick)
        executor = SingleThreadedExecutor()
        executor.add_node(telemetry)
        executor.add_node(planner_presence)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        runner = process = None
        try:
            with tempfile.TemporaryDirectory() as folder:
                logfile = Path(folder)/'production_navigation.log'
                params = ROOT/'tools/results/provincial_stage15_full_observed_20260928_preflight2/runtime_parameters/gazebo_navigation_interface.yaml'
                with logfile.open('w') as stream:
                    process = subprocess.Popen([
                        'python3', str(ROOT/'install/uav_planning/lib/uav_planning/gazebo_navigation_interface.py'),
                        '--ros-args', '-r', '__node:=gazebo_navigation_interface',
                        '--params-file', str(params)], stdout=stream,
                        stderr=subprocess.STDOUT, start_new_session=True)
                    runner = Runner('full', 'isolated_production_nav', 1.2,
                                    'hold-start', Path(folder), raw_every_message=True)
                    result = runner.run()
                    self.assertEqual(result, 'timeout')  # no physical simulator
                    self.assertTrue(runner.accepted)
                    self.assertEqual(runner.goal_delivery['publish_count'], 1)
                    self.assertEqual(runner.final_waypoint_id, 'R0001:W02')
                    self.assertEqual(runner.goal_delivery['acceptance']['route_id'], 'R0001')
                    self.assertIn('RMUC route diagnostic route=R0001', logfile.read_text())
                    from summarize_provincial_full_observed import native_audit
                    from analyze_provincial_forward_clearance import walls_from_sdf
                    walls = walls_from_sdf(ROOT/'src/uav_bringup/worlds/provincial_2025_training.sdf')
                    summary = json.loads((Path(folder)/'isolated_production_nav.summary.json').read_text())
                    review = native_audit(Path(folder), summary, walls, 'isolated_production_nav')
                    self.assertTrue(review['checks']['raw_route_acceptance_matches_delivery_evidence'])
                    # A summary-only acknowledgement must not replace raw evidence.
                    raw = Path(folder)/'isolated_production_nav.native.jsonl'
                    lines = [line for line in raw.read_text().splitlines()
                             if json.loads(line)['topic'] != '/rosout/relevant']
                    raw.write_text('\n'.join(lines)+'\n')
                    review = native_audit(Path(folder), summary, walls, 'isolated_production_nav')
                    self.assertFalse(review['checks']['raw_route_acceptance_matches_delivery_evidence'])
        finally:
            if runner is not None:
                if runner.native_trace:
                    runner.native_trace.close()
                runner.destroy_node()
            if process is not None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
                except ProcessLookupError:
                    pass
            executor.shutdown(timeout_sec=3)
            thread.join(timeout=3)
            telemetry.destroy_timer(timer)
            telemetry.destroy_node()
            planner_presence.destroy_node()


if __name__ == '__main__':
    unittest.main(verbosity=2)
