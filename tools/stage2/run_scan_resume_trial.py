#!/usr/bin/env python3
"""One short goal and asynchronous test-only scan loss stimulus. No speed publisher."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from run_localized_trial import Stage2Runner, Runner, ROOT, SafetyAbort, yaw_of


class ScanResumeRunner(Stage2Runner):
    def __init__(self, name, directory):
        Runner.__init__(self, 'start', name, 90, 'hold-start', directory,
                        terminal_validation=True, raw_every_message=True)
        self.abort_evidence = None
        self.phase = 'armed'
        self.future = None
        self.stable_since = None
        self.hold_ack_sim = None
        self.stimulus = []
        self.client = self.create_client(SetBool, '/scan_relay/hold')
        for topic in ('/localization/navigation_odometry', '/simulation/navigation_odometry'):
            self.create_subscription(Odometry, topic, lambda m,t=topic:self.trace(t,m), 100)
        for topic in ('/localization/diagnostics', '/simulation/odometry_diagnostics', '/scan_relay/status'):
            self.create_subscription(String, topic, lambda m,t=topic:self.trace(t,m), 100)
        for topic in ('/scan', '/guarded_scan'):
            self.create_subscription(LaserScan, topic, lambda m,t=topic:self.trace(t,m), qos_profile_sensor_data)

    def note(self, event, **extra):
        p=self.gt.pose.pose if self.gt else None
        self.stimulus.append(dict(event=event, phase=self.phase,
            monotonic_ns=time.monotonic_ns(), sim_s=self.native_trace.sim_s,
            gt_xy=[p.position.x,p.position.y] if p else None, **extra))

    def request(self, held):
        if not self.client.service_is_ready():
            raise RuntimeError('Relay service unavailable; no automatic retry')
        self.phase='hold_requested' if held else 'resume_requested'
        self.note('service_request', held=held)
        self.future=self.client.call_async(SetBool.Request(data=held))
        self.request_wall=time.monotonic()

    def gt_cb(self, msg):
        super().gt_cb(msg)
        if not self.active or not self.accepted:
            return
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        speed=math.hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y)
        wz=abs(msg.twist.twist.angular.z)
        cmd=self.final_command
        fresh=(cmd is not None and self.final_command_received_steady is not None and
               time.monotonic()-self.final_command_received_steady<=.2)
        cv=math.hypot(cmd.linear.x,cmd.linear.y) if fresh else math.inf
        cw=abs(cmd.angular.z) if fresh else math.inf
        y=msg.pose.pose.position.y
        if self.future is not None:
            if self.future.done():
                response=self.future.result()
                self.future=None
                if not response.success:raise RuntimeError('Relay refused service')
                self.phase='holding' if self.phase=='hold_requested' else 'resumed'
                self.note('service_ack', response=response.message)
                if self.phase=='holding':self.hold_ack_sim=t
            elif time.monotonic()-self.request_wall>2:
                raise RuntimeError('Relay service confirmation timeout')
        active_fresh=(self.last_actuation_received_steady is not None and
            time.monotonic()-self.last_actuation_received_steady<=.2 and
            (self.last_actuation or {}).get('navigation_state')=='active' and
            (self.last_actuation or {}).get('waypoint') is not None)
        if self.phase=='armed' and active_fresh and .60<=y<=.70 and speed>.03 and cv>.03:
            if math.dist((msg.pose.pose.position.x,y),self.goal)<.35:
                raise RuntimeError('Insufficient stopping space; stimulus refused')
            self.note('motion_before_hold', gt_speed=speed, final_speed=cv,
                      active_waypoint=(self.last_actuation or {}).get('waypoint'))
            self.request(True)
        elif self.phase=='holding':
            stopped=speed<=.02 and wz<=.03 and cv<=.02 and cw<=.03
            if stopped:
                if self.stable_since is None:
                    self.stable_since=t
                    self.note('stable_stop_begin')
                if t-self.stable_since>=2:
                    self.note('stable_stop_2s')
                    self.request(False)
            else:self.stable_since=None
            if t-self.hold_ack_sim>5:
                raise RuntimeError('Failed to establish stable hold; stop experiment')

    def save(self, result):
        if result=='terminal_pass' and self.phase!='resumed':result='not_exercised'
        self.note('trial_finish', runner_result=result)
        value=super().save(result)
        (self.output_dir/'hold_stimulus.json').write_text(json.dumps(
            dict(phase=self.phase, events=self.stimulus,
                 gt_use='test stimulus and read-only safety abort; no pose/control feedback'),indent=2)+'\n')
        return value

    def run(self):
        try:return super().run()
        except Exception as error:
            self.active=False
            self.note('trial_exception', error=repr(error))
            return self.save('stimulus_exception')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--name',required=True)
    p.add_argument('--output-dir',required=True,type=Path);a=p.parse_args()
    rclpy.init();runner=ScanResumeRunner(a.name,a.output_dir)
    try:runner.run()
    finally:
        if runner.native_trace:runner.native_trace.close()
        runner.destroy_node();rclpy.shutdown()
    sys.exit(0 if runner.terminal_evidence_pass and runner.phase=='resumed' else 2)
