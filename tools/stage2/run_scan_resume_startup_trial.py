#!/usr/bin/env python3
"""One recorder, created before the world; phased startup, one inherited goal."""
import argparse
import json
from pathlib import Path
import time
import rclpy
from rclpy.qos import qos_check_compatible, QoSCompatibility
from rosidl_runtime_py.convert import message_to_ordereddict
from run_scan_resume_trial import ScanResumeRunner
from scan_capture_readiness import CaptureReadiness, SENSORS, NAVIGATION


def write_once(path,data):
    with path.open('x') as f:json.dump(data,f,indent=2)


class StartupRunner(ScanResumeRunner):
    def __init__(self,name,directory):
        self.capture_gate=CaptureReadiness()
        super().__init__(name,directory)
        self.gate_directory=directory

    def trace(self,topic,msg):
        received=time.monotonic_ns()
        super().trace(topic,msg)
        if not self.active:
            h=getattr(msg,'header',None)
            stamp=(h.stamp.sec+h.stamp.nanosec*1e-9) if h is not None else None
            self.capture_gate.observe(topic,message_to_ordereddict(msg),received,stamp)

    def capture_ready(self,phase):
        topics=dict(SENSORS)
        if phase=='goal':topics.update(NAVIGATION)
        graph={}
        for topic in topics:
            with self.native_trace.observe('capture_startup_graph_query'):
                eps=self.get_publishers_info_by_topic(topic)
                readers=[e for e in self.get_subscriptions_info_by_topic(topic)
                    if e.node_name==self.get_name() and e.node_namespace==self.get_namespace()]
            graph[topic]=[dict(name=e.node_name,gid=list(e.endpoint_gid),qos=str(e.qos_profile),
                receiver_gid=list(readers[0].endpoint_gid) if len(readers)==1 else [],
                receiver_qos=str(readers[0].qos_profile) if len(readers)==1 else None,
                compatible=len(readers)==1 and qos_check_compatible(e.qos_profile,readers[0].qos_profile)[0] == QoSCompatibility.OK) for e in eps]
        return self.capture_gate.check(phase,time.monotonic_ns(),graph,
            writer_ok=not self.native_trace.errors and not self.native_trace.closed)

    def await_startup(self,timeout=100):
        write_once(self.gate_directory/'recorder_constructed.json',dict(monotonic_ns=time.monotonic_ns(),
            single_native_writer=True,recorder_created_before_world=True))
        end=time.monotonic()+timeout;next_query=0
        while time.monotonic()<end and rclpy.ok():
            rclpy.spin_once(self,timeout_sec=.02)
            if (self.gate_directory/'cancel_capture.json').exists():raise RuntimeError('Parent canceled startup')
            now=time.monotonic()
            if now<next_query:continue
            next_query=now+.1
            if self.capture_gate.faults:raise RuntimeError(str(self.capture_gate.faults))
            phase='goal' if (self.gate_directory/'navigation_start.json').exists() else 'sensors'
            report=self.capture_ready(phase)
            file=self.gate_directory/('sensors_capture_ready.json' if phase=='sensors' else 'navigation_capture_ready.json')
            if report['ready'] and not file.exists():write_once(file,report)
            if (self.gate_directory/'allow_goal.json').exists():
                if phase!='goal' or not report['ready']:raise RuntimeError('Goal release before full capture readiness')
                write_once(self.gate_directory/'goal_release_capture_ready.json',report)
                return
        raise RuntimeError('Capture startup readiness timeout: '+json.dumps(self.capture_gate.last_report))

    def wait_ready(self,timeout_s=45):
        end=time.monotonic()+timeout_s;next_query=0
        while time.monotonic()<end and rclpy.ok():
            rclpy.spin_once(self,timeout_sec=.02)
            now=time.monotonic()
            if now<next_query:continue
            next_query=now+.1
            report=self.capture_ready('goal')
            if report['ready'] and self.goal_receiver_ready(time.monotonic()) and 'ego_planner_node' in self.observed_node_names():
                write_once(self.gate_directory/'before_publish_capture_ready.json',report)
                return True
        write_once(self.gate_directory/'capture_wait_timeout.json',self.capture_gate.last_report)
        return False


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--name',required=True);p.add_argument('--output-dir',required=True,type=Path);a=p.parse_args()
    rclpy.init();runner=StartupRunner(a.name,a.output_dir);result='startup_exception'
    try:
        runner.await_startup()
        result=runner.run()
    except Exception as error:
        write_once(a.output_dir/'capture_startup_failure.json',dict(error=repr(error),gate=runner.capture_gate.last_report))
    finally:
        if runner.native_trace:runner.native_trace.close()
        runner.destroy_node();rclpy.shutdown()
    raise SystemExit(0 if result=='terminal_pass' and runner.terminal_evidence_pass else 2)
