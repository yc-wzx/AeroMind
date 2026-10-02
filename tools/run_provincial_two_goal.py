#!/usr/bin/env python3
"""One-world A-area two-goal lifecycle trial; evaluation publishes goals only."""
import argparse
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from run_provincial_safety_fix_matrix import ROOT, existing_servers, wait_for_launch


class LifecycleObserver(Node):
    def __init__(self):
        super().__init__('stage15_lifecycle_observer')
        self.final_commands = []
        self.diagnostics = []
        self.create_subscription(Twist, '/model/omni_robot/cmd_vel',
                                 self.on_command, 100)
        self.create_subscription(String, '/ground/planning/actuation_diagnostics',
                                 self.on_diagnostic, 100)

    def on_command(self, msg):
        self.final_commands.append({
            'wall_monotonic': time.monotonic(),
            'vx':msg.linear.x, 'vy':msg.linear.y, 'wz':msg.angular.z})

    def on_diagnostic(self, msg):
        try:
            data=json.loads(msg.data)
        except (TypeError, ValueError):
            return
        self.diagnostics.append({'wall_monotonic':time.monotonic(), **data})


def max_speed(events, first, last):
    values=[math.hypot(e['vx'],e['vy']) for e in events
            if first<=e['wall_monotonic']<last]
    return max(values) if values else None


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    out=args.output_dir.resolve()
    if out.exists():
        raise FileExistsError(out)
    if existing_servers():
        raise RuntimeError('existing training Gazebo server; refusing mixed telemetry')
    out.mkdir(parents=True)
    paths=(
        'tools/run_provincial_two_goal.py',
        'tools/run_provincial_low_speed_validation.py',
        'tools/provincial_terminal_gate.py',
        'src/uav_planning/scripts/gazebo_navigation_interface.py',
        'src/uav_planning/config/provincial_2025_provisional.json',
        'src/uav_bringup/maps/provincial_2025_provisional.pgm',
        'src/uav_bringup/worlds/provincial_2025_training.sdf',
        'install/uav_planning/lib/uav_planning/gazebo_navigation_interface.py',
    )
    (out/'input_manifest.json').write_text(json.dumps({
        'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'goals':[[4.7,0.90],[4.7,1.15]],
        'sha256':{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths},
    },indent=2)+'\n')
    launch_log=(out/'launch.log').open('x')
    launch=subprocess.Popen(['ros2','launch','uav_bringup',
        'provincial_2025_provisional.launch.py',
        'gui:=false','rviz:=false','auto_goal:=false'],
        stdout=launch_log,stderr=subprocess.STDOUT,start_new_session=True)
    observer=None
    events={}
    try:
        wait_for_launch()
        rclpy.init()
        observer=LifecycleObserver()
        thread=threading.Thread(target=rclpy.spin,args=(observer,),daemon=True)
        thread.start()
        events['monitor_start']=time.monotonic()
        time.sleep(1.5)
        plans=(('trial_a_seq_01',(4.7,0.5),(4.7,0.90)),
               ('trial_a_seq_02',(4.7,0.90),(4.7,1.15)))
        for index,(name,start,goal) in enumerate(plans):
            if index:
                events['between_start']=time.monotonic()
                time.sleep(1.0)
            events[name+'_start']=time.monotonic()
            cmd=[sys.executable,str(ROOT/'tools/run_provincial_low_speed_validation.py'),
                 '--route','start','--name',name,'--yaw-mode','hold-start',
                 '--terminal-validation','--timeout','70','--output-dir',str(out),
                 '--start',str(start[0]),str(start[1]),
                 '--goal',str(goal[0]),str(goal[1])]
            with (out/f'{name}.runner.log').open('x') as log:
                result=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,
                                      timeout=100)
            events[name+'_end']=time.monotonic()
            if result.returncode:
                raise RuntimeError(f'{name} failed with code {result.returncode}')
        time.sleep(1.0)
        events['monitor_end']=time.monotonic()
    finally:
        if observer is not None:
            observer.destroy_node()
            rclpy.shutdown()
            thread.join(timeout=3)
            with (out/'observer_final_commands.jsonl').open('x') as stream:
                for item in observer.final_commands:
                    stream.write(json.dumps(item)+'\n')
            with (out/'observer_diagnostics.jsonl').open('x') as stream:
                for item in observer.diagnostics:
                    stream.write(json.dumps(item)+'\n')
        try:
            os.killpg(launch.pid,signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            launch.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid,signal.SIGTERM)
            launch.wait(timeout=8)
        launch_log.close()
    if existing_servers():
        raise RuntimeError('Gazebo server survived cleanup')
    first=json.loads((out/'trial_a_seq_01.summary.json').read_text())
    second=json.loads((out/'trial_a_seq_02.summary.json').read_text())
    speeds={
        'initial_idle':max_speed(observer.final_commands,
                                 events['monitor_start'],events['trial_a_seq_01_start']),
        'between_goals':max_speed(observer.final_commands,
                                  events['between_start'],events['trial_a_seq_02_start']),
        'second_goal':max_speed(observer.final_commands,
                                events['trial_a_seq_02_start'],events['trial_a_seq_02_end']),
    }
    result={
        'classification':'TRAINING-ONLY / PROVISIONAL',
        'goals':[[4.7,0.90],[4.7,1.15]],
        'first':{'result':first['result'],'error_m':first['gt_goal_error_m'],
                 'waypoint_id':first['final_waypoint_id'],
                 'completion_id':first['navigation_completion']['waypoint_id'],
                 'evidence_pass':first['terminal_evidence_pass']},
        'second':{'result':second['result'],'error_m':second['gt_goal_error_m'],
                  'waypoint_id':second['final_waypoint_id'],
                  'completion_id':second['navigation_completion']['waypoint_id'],
                  'evidence_pass':second['terminal_evidence_pass']},
        'phase_max_final_cmd_planar_mps':speeds,
    }
    result['pass']=(first['terminal_evidence_pass'] and
                    second['terminal_evidence_pass'] and
                    first['final_waypoint_id']!=second['final_waypoint_id'] and
                    speeds['initial_idle'] is not None and
                    speeds['initial_idle']<=0.02 and
                    speeds['between_goals'] is not None and
                    speeds['between_goals']<=0.02 and
                    speeds['second_goal'] is not None and
                    speeds['second_goal']>0.02)
    (out/'two_goal_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False))
    if not result['pass']:
        raise SystemExit(2)


if __name__=='__main__':
    main()
