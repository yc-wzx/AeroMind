"""Actual readiness and runner/launch entry tests; no ROS init or live publishers."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
from scan_capture_readiness import CaptureReadiness,SENSORS,NAVIGATION
from run_scan_resume_startup_trial import StartupRunner
from run_scan_resume_trial import ScanResumeRunner
from run_localized_trial import Runner
import run_scan_resume_startup_experiment as entry
from audit_scan_resume_startup_v4 import fully_timed_loss
from test_scan_resume import synthetic_rows


def data(topic,t):
    if topic in ('/scan','/guarded_scan'):
        return dict(header={'frame_id':'omni_robot/base_link/lidar','stamp':{'sec':int(t),'nanosec':round((t-int(t))*1e9)}},
            angle_min=0.,angle_max=1.,angle_increment=1.,range_min=.08,range_max=16.,scan_time=0.,time_increment=0.,ranges=[1.,2.],intensities=[])
    if topic=='/clock':return {'clock':{'sec':int(t),'nanosec':round((t-int(t))*1e9)}}
    if topic=='/model/omni_robot/cmd_vel':return {'linear':{'x':0.,'y':0.,'z':0.},'angular':{'x':0.,'y':0.,'z':0.}}
    return {'pose':{'pose':{'position':{'x':4.7,'y':.5,'z':0.},'orientation':{'x':0.,'y':0.,'z':.70710678,'w':.70710678}}}}


class CaptureTests(unittest.TestCase):
    def setUp(self):self.g=CaptureReadiness();self.graph={t:[dict(name=n,gid=[1,2],receiver_gid=[3,4],compatible=True)] for t,n in {**SENSORS,**NAVIGATION}.items()}
    def feed(self,t,nav=False,missing=None):
        for topic in {**SENSORS,**(NAVIGATION if nav else {})}:
            if topic==missing:continue
            self.g.observe(topic,data(topic,t),round(t*1e9),t if topic in ('/scan','/guarded_scan','/simulation/navigation_odometry') else None)
        if nav:self.g.observe('/localization/diagnostics',{'data':json.dumps({'accepted':True,'stamp_s':t,'associated_odom_stamp_s':t})},round(t*1e9),None)
    def ready(self,phase='sensors',missing=None):
        for i in range(10,19):
            t=i/10;self.feed(t,phase=='goal',missing);r=self.g.check(phase,round(t*1e9),self.graph)
        return r
    def test_sensor_gate_needs_real_samples_and_stability(self):self.assertTrue(self.ready()['ready'])
    def test_full_goal_gate_requires_all_streams(self):self.assertTrue(self.ready('goal')['ready'])
    def test_each_missing_stream_blocks(self):
        for topic in {**SENSORS,**NAVIGATION}:
            with self.subTest(topic=topic):
                self.g=CaptureReadiness();self.assertFalse(self.ready('goal',topic)['ready'])
    def test_observer_cannot_replace_bridge(self):
        self.graph['/scan'][0]['name']='observer';self.assertFalse(self.ready()['ready'])
    def test_unknown_gid_not_ready(self):
        self.graph['/guarded_scan'][0]['gid']=[0,0];self.assertFalse(self.ready()['ready'])
    def test_incompatible_qos_not_ready(self):
        self.graph['/scan'][0]['compatible']=False;self.assertFalse(self.ready()['ready'])
    def test_missing_initial_accepted_scan_remains_failure(self):
        self.g.observe('/localization/diagnostics',{'data':json.dumps({'accepted':True,'stamp_s':.9,'associated_odom_stamp_s':.9})},900000000,None)
        self.assertFalse(self.ready('goal')['ready'])
    def test_nan_critical_field_not_ready(self):
        d=data('/ground/odometry',.8);d['pose']['pose']['position']['x']=float('nan')
        self.g.observe('/ground/odometry',d,800000000,None);self.assertFalse(self.ready('goal')['ready'])
    def test_changed_guarded_payload_not_ready(self):
        self.ready();self.g.history['/guarded_scan'][1.8]['ranges'][0]=9.
        self.assertFalse(self.g.check('sensors',1800000000,self.graph)['ready'])
    def test_stale_stream_not_ready(self):
        self.ready('goal');self.assertFalse(self.g.check('goal',2400000000,self.graph)['ready'])
    def test_writer_error_not_ready(self):
        self.ready('goal');self.assertFalse(self.g.check('goal',1800000000,self.graph,writer_ok=False)['ready'])
    def test_duplicate_source_stamp_not_ready(self):
        self.feed(1.);self.feed(1.);self.assertFalse(self.g.check('sensors',1000000000,self.graph)['ready'])


class EntryTests(unittest.TestCase):
    def test_existing_directory_no_process_launch(self):
        with tempfile.TemporaryDirectory() as d,patch.object(entry.subprocess,'Popen') as p:
            with self.assertRaises(FileExistsError):entry.run(Path(d),'one_goal')
            p.assert_not_called()
    def test_invalid_trial_no_process_launch(self):
        with tempfile.TemporaryDirectory() as d,patch.object(entry.subprocess,'Popen') as p:
            with self.assertRaises(ValueError):entry.run(Path(d)/'new','../bad')
            p.assert_not_called()
    def test_actual_runner_cannot_publish_without_ready(self):
        runner=StartupRunner.__new__(StartupRunner);runner.wait_ready=Mock(return_value=False);runner.save=Mock(return_value='not_ready');runner.goal_pub=Mock()
        self.assertEqual(Runner.run(runner),'not_ready');runner.goal_pub.publish.assert_not_called()
    def test_trace_records_before_readiness_without_filter(self):
        from geometry_msgs.msg import Twist
        runner=StartupRunner.__new__(StartupRunner);runner.active=False;runner.capture_gate=CaptureReadiness()
        with patch.object(ScanResumeRunner,'trace') as record:
            runner.trace('/model/omni_robot/cmd_vel',Twist())
            record.assert_called_once();self.assertIn('/model/omni_robot/cmd_vel',runner.capture_gate.latest)
    def test_actual_humble_qos_api_used_by_capture_entry(self):
        from contextlib import nullcontext
        from rclpy.qos import QoSProfile,QoSLivelinessPolicy,QoSReliabilityPolicy
        r=StartupRunner.__new__(StartupRunner);r.native_trace=NS(observe=lambda x:nullcontext(),errors=[],closed=False)
        r.capture_gate=CaptureReadiness()
        def endpoints(topic):
            q=QoSProfile(depth=100,liveliness=QoSLivelinessPolicy.AUTOMATIC,
                reliability=QoSReliabilityPolicy.BEST_EFFORT if topic in ('/scan','/guarded_scan') else QoSReliabilityPolicy.RELIABLE)
            return [NS(node_name=SENSORS[topic],endpoint_gid=[1,2],qos_profile=q)]
        r.get_publishers_info_by_topic=endpoints
        r.get_name=lambda:'capture';r.get_namespace=lambda:'/'
        r.get_subscriptions_info_by_topic=lambda topic:[NS(node_name='capture',node_namespace='/',endpoint_gid=[3,4],qos_profile=endpoints(topic)[0].qos_profile)]
        report=r.capture_ready('sensors')
        self.assertTrue(all(report['checks']['publisher:'+t] for t in SENSORS))
    def test_old_production_readiness_allows_missing_scan_prefix(self):
        import time
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Twist
        r=Runner.__new__(Runner);r.gt=r.odom=Odometry();r.final_command=Twist()
        r.gt_received_steady=r.odom_received_steady=r.final_command_received_steady=time.monotonic()
        r.native_trace=NS(counts={'/clock':1});r.goal_receiver_ready=Mock(return_value=True)
        r.observed_node_names=Mock(return_value=['ego_planner_node'])
        with patch('run_provincial_low_speed_validation.rclpy.ok',return_value=True),patch('run_provincial_low_speed_validation.rclpy.spin_once'):
            self.assertTrue(Runner.wait_ready(r,.1))
        # Actual old readiness has no scan or matching-diagnostic observations.
        self.assertFalse(CaptureReadiness().check('goal',time.monotonic_ns(),{})['ready'])
    def test_ready_checks_are_bounded_while_callbacks_drain(self):
        tick=[0.];r=StartupRunner.__new__(StartupRunner)
        r.capture_gate=CaptureReadiness();r.goal_receiver_ready=Mock(return_value=True)
        r.observed_node_names=Mock(return_value=['ego_planner_node'])
        r.capture_ready=Mock(side_effect=lambda phase:dict(ready=tick[0]>=.5,monotonic_ns=round(tick[0]*1e9)))
        with tempfile.TemporaryDirectory() as d:
            r.gate_directory=Path(d)
            with patch('run_scan_resume_startup_trial.time.monotonic',side_effect=lambda:tick[0]),patch('run_scan_resume_startup_trial.rclpy.ok',return_value=True),patch('run_scan_resume_startup_trial.rclpy.spin_once',side_effect=lambda *a,**k:tick.__setitem__(0,tick[0]+.02)) as spin:
                self.assertTrue(r.wait_ready(1.))
                self.assertGreaterEqual(spin.call_count,25);self.assertLessEqual(r.capture_ready.call_count,6)
    def test_progress_has_required_profile_schema(self):
        record=entry.initial_progress('one_goal')
        self.assertEqual(record['profile_path'],'src/uav_bringup/config/stage2_odometry_zero.yaml')
        self.assertEqual(record['goals_requested'],0);self.assertEqual(record['launches'],0)
    def test_dead_process_cannot_trigger_restart(self):
        process=Mock();process.poll.return_value=1
        with tempfile.TemporaryDirectory() as d,patch.object(entry.subprocess,'Popen') as p:
            with self.assertRaises(RuntimeError):entry.wait_file(Path(d)/'missing',[process],.1)
            p.assert_not_called()
    def test_timeout_cannot_trigger_restart(self):
        process=Mock();process.poll.return_value=None
        with tempfile.TemporaryDirectory() as d,patch.object(entry.subprocess,'Popen') as p:
            with self.assertRaises(RuntimeError):entry.wait_file(Path(d)/'missing',[process],.001)
            p.assert_not_called()
    def test_navigation_split_contains_no_world(self):
        from launch.actions import ExecuteProcess
        from launch_ros.actions import Node
        file=entry.ROOT/'src/uav_bringup/launch/stage2_scan_startup_split.py'
        spec=importlib.util.spec_from_file_location('split',file);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        nav=module.split_description(True);sensors=module.split_description(False)
        self.assertEqual(sum(isinstance(a,Node) for a in nav.entities),1)
        self.assertFalse(any(isinstance(a,ExecuteProcess) and not isinstance(a,Node) for a in nav.entities))
        self.assertFalse(any(isinstance(a,Node) and a.node_executable=='stage2_localized_interface.py' for a in sensors.entities))


class LossAuditTests(unittest.TestCase):
    def test_valid_fully_timed_loss(self):self.assertTrue(fully_timed_loss(synthetic_rows())['pass'])
    def test_sensor_first_does_not_allow_untimed_command(self):
        rows=synthetic_rows();next(r for r in rows if r['topic']=='/model/omni_robot/cmd_vel')['receive_sim_s']=None
        with self.assertRaises(ValueError):fully_timed_loss(rows)
    def test_nonfinite_command_rejected(self):
        rows=synthetic_rows();next(r for r in rows if r['topic']=='/model/omni_robot/cmd_vel')['data']['linear']['x']=float('nan')
        with self.assertRaises(ValueError):fully_timed_loss(rows)
    def test_lost_scan_retains_failure(self):
        rows=synthetic_rows();rows=[r for r in rows if not(r['topic']=='/guarded_scan' and r['message_stamp_s']==.9)]
        self.assertFalse(fully_timed_loss(rows)['pass'])
    def test_nonzero_hold_command_retains_failure(self):
        rows=synthetic_rows();next(r for r in rows if r['topic']=='/model/omni_robot/cmd_vel' and r['receive_sim_s']==2.)['data']['linear']['x']=.1
        self.assertFalse(fully_timed_loss(rows)['pass'])


if __name__=='__main__':unittest.main()
