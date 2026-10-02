#!/usr/bin/env python3
"""Existing strict Full runner plus raw injected odometry and a test abort.

The GT clearance check only ends this experiment. It is not a navigation
guard and never corrects pose or publishes commands into the control chain.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools'))
from run_provincial_low_speed_validation import Runner, yaw_of
from provincial_safety_geometry import body_wall_gap


class SafetyAbort(RuntimeError):
    pass


class Stage2Runner(Runner):
    def __init__(self, name, output_dir):
        super().__init__('full', name, 240, 'hold-start', output_dir,
                         terminal_validation=True, raw_every_message=True)
        self.abort_evidence = None
        self.create_subscription(Odometry, '/simulation/navigation_odometry',
                                 lambda m:self.trace('/simulation/navigation_odometry', m), 100)
        self.create_subscription(String, '/simulation/odometry_diagnostics',
                                 lambda m:self.trace('/simulation/odometry_diagnostics', m), 100)

    def gt_cb(self, msg):
        super().gt_cb(msg)
        if self.active:
            p = msg.pose.pose
            gap = body_wall_gap(p.position.x, p.position.y, yaw_of(p.orientation), self.walls)
            if not math.isfinite(gap) or gap < .08:
                self.abort_evidence = {'reason':'TRUTH_SAMPLED_CLEARANCE_BELOW_0.08',
                                       'gap_m':gap, 'x':p.position.x,'y':p.position.y,
                                       'stamp_s':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9}
                raise SafetyAbort(str(self.abort_evidence))

    def run(self):
        try:
            return super().run()
        except SafetyAbort:
            self.active = False
            (self.output_dir/'safety_abort.json').write_text(json.dumps(self.abort_evidence, indent=2)+'\n')
            return self.save('truth_clearance_stop')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--name', required=True)
    p.add_argument('--output-dir', required=True, type=Path)
    a = p.parse_args()
    rclpy.init()
    runner = Stage2Runner(a.name, a.output_dir)
    try:
        result = runner.run()
    finally:
        if runner.native_trace:
            runner.native_trace.close()
        runner.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if runner.terminal_evidence_pass else 2)
