#!/usr/bin/env python3
"""Original single-goal runner plus independent sensor-gate observation."""
import argparse
import json
import sys
import time
from collections import Counter
from pathlib import Path
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tools/stage2/lio'))
from run_navigation_trial import LioRunner
from std_msgs.msg import String
import rclpy


class SafeRunner(LioRunner):
    def __init__(self, out, phase):
        self.ledger = Counter()
        super().__init__(out, phase)
        self.create_subscription(String, '/lio/safety_diagnostics',
            lambda m: self.trace('/lio/safety_diagnostics', m), 100)

    def trace(self, topic, message):
        super().trace(topic, message)
        if topic == '/lio/odometry': self.ledger['inputs'] += 1
        if topic == '/localization/lio_navigation_odometry': self.ledger['outputs'] += 1
        if topic == '/lio/adapter_diagnostics':
            self.ledger['dispositions'] += 1
            if json.loads(message.data).get('event') == 'accepted':
                self.ledger['accepted'] += 1

    def save(self, result):
        # Close on a complete OBSERVED callback ledger, never remove received
        # rows or invent the final disposition. Bounded to preserve the normal
        # terminal snapshot freshness. Continuous publishers need not stop.
        end = time.monotonic()+.10
        while ((self.ledger['inputs'] != self.ledger['dispositions'] or
                self.ledger['outputs'] != self.ledger['accepted']) and
               time.monotonic() < end):
            rclpy.spin_once(self, timeout_sec=.001)
        (self.out/'capture_callback_boundary.json').write_text(json.dumps({
            'counts': dict(self.ledger),
            'closed_observed_ledger': self.ledger['inputs'] == self.ledger['dispositions']
                and self.ledger['outputs'] == self.ledger['accepted'],
            'scope': 'All received rows retained; bounded100ms drain, not publisher losslessness proof'
        }, indent=2))
        return super().save(result)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--phase', choices=['short', 'full'], required=True)
    args = parser.parse_args()
    rclpy.init()
    node = SafeRunner(args.output, args.phase)
    try:
        result = node.run()
    except KeyboardInterrupt:
        result = node.save('experiment_canceled_before_or_after_goal')
    finally:
        node.sensor_trace.close()
        if node.native_trace and not node.native_trace.closed:
            node.native_trace.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if result == 'terminal_pass' and node.terminal_evidence_pass else 2)
