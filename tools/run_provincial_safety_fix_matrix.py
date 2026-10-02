#!/usr/bin/env python3
"""Run isolated low-speed provincial trials, one Gazebo server at a time.

Run after sourcing /opt/ros/humble and this workspace's install/setup.bash.
This script publishes only the runner's authorized /goal_pose messages and
uses Gazebo set_pose to establish each recorded initial condition.
"""
import argparse
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'tools/results/provincial_stage15_safety_fix'
WORLD = '/world/provincial_2025_training/set_pose'
WORLD_PATH = str(ROOT/'install/uav_bringup/share/uav_bringup/worlds/provincial_2025_training.sdf')
TRIALS = {
    'A': ('start', None, 90),
    'B': ('turn', (4.7, 1.15, 0.0), 110),
    'C': ('shoot', (5.8, 1.8, 0.0), 140),
}


def existing_servers():
    listing = subprocess.run(['ps','-eo','args'],capture_output=True,text=True,check=True).stdout
    return [line for line in listing.splitlines()
            if line.startswith('ign gazebo -s -r') and WORLD_PATH in line]


def wait_for_launch():
    deadline=time.monotonic()+35
    while time.monotonic()<deadline:
        query=subprocess.run(['ign','service','-i','-s',WORLD],
                             capture_output=True,text=True)
        providers=query.stdout.count('ignition.msgs.Pose, ignition.msgs.Boolean')
        if providers==1:
            nodes=subprocess.run(['ros2','node','list'],capture_output=True,text=True,
                                 timeout=5).stdout
            if '/ego_planner_node' in nodes and '/gazebo_navigation_interface' in nodes:
                return
        time.sleep(.5)
    raise RuntimeError('single Gazebo server and navigation nodes were not ready')


def set_pose(pose):
    x,y,yaw=pose
    request=(f'name: "omni_robot" position: {{ x: {x} y: {y} z: 0.15 }} '
             f'orientation: {{ z: {math.sin(yaw/2)} '
             f'w: {math.cos(yaw/2)} }}')
    reply=subprocess.run(['ign','service','-s',WORLD,
                          '--reqtype','ignition.msgs.Pose',
                          '--reptype','ignition.msgs.Boolean',
                          '--timeout','5000','--req',request],
                         capture_output=True,text=True,timeout=10)
    if reply.returncode or 'data: true' not in reply.stdout:
        raise RuntimeError(f'Gazebo set_pose failed: {reply.stdout} {reply.stderr}')


def run_one(letter, run_number, out, terminal_validation=False, initial_yaw_deg=None):
    if existing_servers():
        raise RuntimeError('another provisional Gazebo server is active; refusing mixed telemetry')
    route,pose,timeout=TRIALS[letter]
    if pose is not None and initial_yaw_deg is not None:
        pose=(pose[0],pose[1],math.radians(initial_yaw_deg))
    name=f'trial_{letter.lower()}_{run_number:02d}'
    if any((out/file).exists() for file in
           (f'launch_{name}.log', f'{name}.summary.json', f'{name}.csv')):
        raise FileExistsError(f'refusing to overwrite existing trial {name} in {out}')
    log=(out/f'launch_{name}.log').open('x')
    launch=subprocess.Popen(['ros2','launch','uav_bringup',
                             'provincial_2025_provisional.launch.py',
                             'gui:=false','rviz:=false','auto_goal:=false'],
                            stdout=log,stderr=subprocess.STDOUT,
                            start_new_session=True)
    result={'name':name,'route':route,'world_isolated':True}
    try:
        wait_for_launch()
        if pose is not None:
            set_pose(pose)
        runner_cmd=[sys.executable,str(ROOT/'tools/run_provincial_low_speed_validation.py'),
                    '--route',route,'--name',name,'--yaw-mode','hold-start',
                    '--timeout',str(timeout),'--output-dir',str(out)]
        if terminal_validation:
            runner_cmd.append('--terminal-validation')
        with (out/f'{name}.runner.log').open('x') as runner_log:
            finished=subprocess.run(runner_cmd,stdout=runner_log,
                                    stderr=subprocess.STDOUT,timeout=timeout+60)
        result['runner_returncode']=finished.returncode
        summary=out/f'{name}.summary.json'
        if summary.exists():
            data=json.loads(summary.read_text())
            result.update({key:data.get(key) for key in (
                'result','assessment','gt_goal_error_m',
                'minimum_physical_wall_clearance_m','retry_count','stuck_count',
                'initial_gt_pose','actuation_counts')})
    finally:
        try:
            os.killpg(launch.pid,signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            launch.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid,signal.SIGTERM)
            launch.wait(timeout=8)
        log.close()
        time.sleep(1)
    if existing_servers():
        raise RuntimeError(f'Gazebo server survived cleanup after {name}')
    print(json.dumps(result,ensure_ascii=False),flush=True)
    return result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('trials',nargs='+',help='Examples: A02 B03 C02')
    parser.add_argument('--output-dir',type=Path,default=OUT)
    parser.add_argument('--terminal-validation',action='store_true')
    parser.add_argument('--initial-yaw-deg',type=float)
    args=parser.parse_args()
    out=args.output_dir.resolve()
    out.mkdir(parents=True,exist_ok=True)
    evidence={
        'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'terminal_validation':args.terminal_validation,
        'initial_yaw_deg':args.initial_yaw_deg,
        'trial_ids':args.trials,
        'sha256':{}
    }
    inputs=(
        'tools/run_provincial_low_speed_validation.py',
        'tools/provincial_terminal_gate.py',
        'tools/run_provincial_safety_fix_matrix.py',
        'src/uav_planning/scripts/gazebo_navigation_interface.py',
        'src/uav_planning/scripts/ego_trajectory_executor.py',
        'src/uav_planning/config/provincial_2025_provisional.json',
        'src/uav_bringup/launch/provincial_2025_provisional.launch.py',
        'src/uav_bringup/config/ego_provincial_2025_provisional.yaml',
        'src/uav_bringup/maps/provincial_2025_provisional.pgm',
        'src/uav_bringup/worlds/provincial_2025_training.sdf',
        'install/ego_planner/lib/ego_planner/ego_planner_node',
        'install/uav_planning/lib/uav_planning/gazebo_navigation_interface.py',
    )
    for relative in inputs:
        path=ROOT/relative
        if not path.exists():
            raise FileNotFoundError(path)
        evidence['sha256'][relative]=hashlib.sha256(path.read_bytes()).hexdigest()
    manifest=out/'input_manifest.json'
    if manifest.exists():
        raise FileExistsError(f'refusing to overwrite prior manifest: {manifest}')
    manifest.write_text(json.dumps(evidence,indent=2)+'\n')
    results=[]
    for token in args.trials:
        if len(token)<3 or token[0] not in TRIALS or not token[1:].isdigit():
            raise ValueError(f'invalid trial id {token}')
        results.append(run_one(token[0],int(token[1:]),out,
                               args.terminal_validation,args.initial_yaw_deg))
        (out/'matrix_progress.json').write_text(
            json.dumps(results,indent=2,ensure_ascii=False)+'\n')


if __name__=='__main__':
    main()
