"""No Gazebo or ROS publishers: production recorder fault injection."""
import importlib.util
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch
from geometry_msgs.msg import Twist
from provincial_native_trace import NativeTrace
from summarize_provincial_full_observed import recorder_health_audit


class BlockingStream:
    def __init__(self, target, fail=False):
        self.target, self.fail = target, fail
        self.entered, self.release = threading.Event(), threading.Event()
    def write(self, value):
        self.entered.set()
        if not self.release.wait(3):
            raise RuntimeError('test writer not released')
        if self.fail:
            raise OSError('injected disk failure')
        return self.target.write(value)
    def flush(self):
        self.target.flush()
    def close(self):
        self.target.close()


class TraceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.path = self.root/'test.native.jsonl'
        self.trace = None
    def tearDown(self):
        if self.trace:
            self.trace.close()
        self.tmp.cleanup()
    def make(self, **kwargs):
        self.trace = NativeTrace(self.path, **kwargs)
        return self.trace
    def audit(self, count=1):
        return recorder_health_audit(self.root, 'test', {'/cmd_vel': {'count': count}}, True)
    def test_old_production_write_blocks_callback(self):
        old = Path(__file__).parent/'results/provincial_stage15_capture_fix_20260929/before/tools/provincial_native_trace.py'
        spec = importlib.util.spec_from_file_location('native_before', old)
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        trace = module.NativeTrace(self.path)
        blocker = BlockingStream(trace.stream); trace.stream = blocker
        done = threading.Event()
        thread = threading.Thread(target=lambda: (trace.record('/cmd_vel', Twist()), done.set()))
        thread.start()
        try:
            self.assertTrue(blocker.entered.wait(1))
            self.assertFalse(done.wait(.05))
        finally:
            blocker.release.set(); thread.join(2); trace.close()
        self.assertTrue(done.is_set())
    def test_slow_disk_does_not_block_receive_or_retimestamp(self):
        trace = self.make()
        blocker = BlockingStream(trace.stream); trace.stream = blocker
        before = time.monotonic_ns()
        trace.record('/cmd_vel', Twist())
        self.assertTrue(blocker.entered.wait(1))
        # Writer is still blocked; this second production record must return.
        trace.record('/cmd_vel', Twist())
        callback_end = time.monotonic_ns()
        blocker.release.set()
        self.assertTrue(trace.close()['pass'])
        rows = [json.loads(line) for line in self.path.read_text().splitlines()]
        self.assertEqual([row['receive_sequence'] for row in rows], [1,2])
        self.assertTrue(all(before <= row['receive_monotonic_ns'] <= callback_end for row in rows))
    def test_all_messages_and_original_values_preserved(self):
        trace = self.make()
        msg = Twist()
        for i in range(2000):
            msg.linear.x = float(i)
            trace.record('/cmd_vel', msg)
        health = trace.close()
        rows = [json.loads(line) for line in self.path.read_text().splitlines()]
        self.assertTrue(health['pass'])
        self.assertEqual([row['data']['linear']['x'] for row in rows], list(range(2000)))
        self.assertEqual([row['receive_sequence'] for row in rows], list(range(1,2001)))
        self.assertTrue(all(row['message_stamp_s'] is None for row in rows))
        self.assertTrue(self.audit(2000)['pass'])
    def test_overflow_is_failure_not_silent_loss(self):
        trace = self.make(queue_capacity=1)
        blocker = BlockingStream(trace.stream); trace.stream = blocker
        trace.record('/cmd_vel', Twist()); self.assertTrue(blocker.entered.wait(1))
        try:
            trace.record('/cmd_vel', Twist())
            with self.assertRaises(RuntimeError):
                trace.record('/cmd_vel', Twist())
        finally:
            blocker.release.set()
        self.assertFalse(trace.close()['pass'])
        self.assertFalse(self.audit(2)['pass'])
    def test_writer_failure_is_fail_closed(self):
        trace = self.make()
        blocker = BlockingStream(trace.stream, fail=True); trace.stream = blocker
        trace.record('/cmd_vel', Twist()); self.assertTrue(blocker.entered.wait(1))
        blocker.release.set()
        self.assertFalse(trace.close()['pass'])
        self.assertFalse(self.audit(0)['pass'])
    def test_close_timeout_is_fail_closed(self):
        trace = self.make(close_timeout_s=.01)
        blocker = BlockingStream(trace.stream); trace.stream = blocker
        trace.record('/cmd_vel', Twist()); self.assertTrue(blocker.entered.wait(1))
        try:
            self.assertFalse(trace.close()['pass'])
        finally:
            blocker.release.set(); trace.worker.join(2)
        self.assertFalse(self.audit()['pass'])
    def test_diagnostics_preserve_operation_interval(self):
        trace = self.make()
        with trace.observe('ros_graph_query'):
            trace.record('/cmd_vel', Twist())
        self.assertTrue(trace.close()['pass'])
        rows = [json.loads(line) for line in trace.timing_path.read_text().splitlines()]
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['operation'], 'ros_graph_query')
        self.assertEqual(rows[0]['end_monotonic_ns']-rows[0]['start_monotonic_ns'], rows[0]['duration_ns'])
        self.assertTrue(self.audit()['pass'])
    def test_missing_health_rejected(self):
        self.assertFalse(self.audit()['pass'])
    def test_legacy_health_absence_explicit(self):
        report = recorder_health_audit(self.root, 'test', {}, False)
        self.assertTrue(report['pass'])
        self.assertFalse(report['writer_health_verified'])
    def test_deleted_raw_row_rejected(self):
        trace = self.make(); trace.record('/cmd_vel', Twist()); trace.close()
        self.assertFalse(self.audit(0)['pass'])
    def test_missing_timing_rejected(self):
        trace = self.make(); trace.record('/cmd_vel', Twist()); trace.close()
        trace.timing_path.unlink()
        self.assertFalse(self.audit()['pass'])
    def test_nonfinite_value_retained_not_zero_filled(self):
        trace = self.make(); msg = Twist(); msg.linear.x = float('nan')
        trace.record('/cmd_vel', msg); trace.close()
        from summarize_provincial_full_observed import finite_tree
        row = json.loads(self.path.read_text())
        self.assertFalse(finite_tree(row['data']))
    def test_existing_path_refused(self):
        self.path.write_text('original')
        with self.assertRaises(FileExistsError): self.make()
        self.assertEqual(self.path.read_text(), 'original')


class NativeContinuityTests(unittest.TestCase):
    def audit(self, topic, gap=.02, bad_receive=False):
        from summarize_provincial_full_observed import native_audit
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rows = []
            for name in ('/gazebo/odometry', '/ground/odometry', '/model/omni_robot/cmd_vel'):
                for i in range(2):
                    data = {'linear': {'x': 0., 'y': 0., 'z': 0.},
                            'angular': {'x': 0., 'y': 0., 'z': 0.}}
                    if 'odometry' in name:
                        data = {'pose': {'pose': {'position': {'x': 0., 'y': 0., 'z': 0.},
                            'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}},
                            'twist': {'twist': data}}
                    received = 1_000_000_000 + int(i*(gap if name == topic else .02)*1e9)
                    if bad_receive and name == topic and i: received = float('nan')
                    rows.append({'topic': name, 'receive_sequence': i+1,
                        'receive_monotonic_ns': received, 'receive_sim_s': i*.02,
                        'message_stamp_s': i*.02 if 'odometry' in name else None,
                        'data': data})
            (root/'trial.native.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            summary = {'initial_gt_pose': [0,0,0], 'terminal_snapshot': {'gt_stamp_s': .02},
                       'duration_sim_s': .02, 'goal_delivery': {'publish_monotonic_ns': 0}}
            with patch('provincial_safety_geometry.polygon_distance', return_value=.1):
                return native_audit(root, summary, [{'polygon': [], 'name': 'test'}], 'trial')
    def test_normal_receive_cadence_passes_continuity(self):
        self.assertTrue(self.audit('/gazebo/odometry')['checks']['native_receive_continuity'])
    def test_wall_receive_stall_rejected_despite_dense_sim_stamps(self):
        report = self.audit('/gazebo/odometry', .343483)
        self.assertFalse(report['checks']['native_receive_continuity'])
        self.assertAlmostEqual(report['topics']['/gazebo/odometry']['max_stamp_gap_s'], .02)
    def test_existing_command_and_odom_thresholds_retained(self):
        self.assertFalse(self.audit('/model/omni_robot/cmd_vel', .201)['checks']['native_receive_continuity'])
        self.assertFalse(self.audit('/ground/odometry', .501)['checks']['native_receive_continuity'])
        self.assertTrue(self.audit('/ground/odometry', .499)['checks']['native_receive_continuity'])
    def test_nonfinite_receive_time_rejected(self):
        report = self.audit('/gazebo/odometry', bad_receive=True)
        self.assertFalse(report['checks']['all_critical_native_fields_finite'])
        self.assertFalse(report['pass'])


if __name__ == '__main__':
    unittest.main()
