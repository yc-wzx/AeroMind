#!/usr/bin/env python3
"""One world/goal. Capture precedes localization; no automatic restart/retry."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import rclpy
import run_scan_resume_experiment as prior
from run_scan_resume_experiment import (ROOT,observed,frozen_check,write,existing_servers,
    GridRoute,MAP,FIELD,sdf_walls,offline_leg,server_processes,PoseObserver,
    wait_odometry_graph,wait_scan_graph,scan_runtime_parameters_valid)
from audit_scan_resume_startup_v4 import audit


def wait_file(path,processes,timeout=40):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if any(p.poll() is not None for p in processes):raise RuntimeError('Startup process exited before '+path.name)
        if path.exists():
            try:return json.loads(path.read_text())
            except json.JSONDecodeError:pass  # writer still closing a one-shot marker
        time.sleep(.05)
    raise RuntimeError('Readiness timeout: '+path.name)


def stop(process):
    if process is None or process.poll() is not None:return
    try:os.killpg(process.pid,signal.SIGINT)
    except ProcessLookupError:return
    try:process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)


def initial_progress(trial_id):
    return dict(name=trial_id,profile='zero',profile_path='src/uav_bringup/config/stage2_odometry_zero.yaml',
        launches=0,goals_requested=0,navigation_only_launches=0,status='PREPARED')


def run(output_dir,trial_id):
    out=Path(output_dir).resolve()
    if out.exists():raise FileExistsError(out)
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}',trial_id):raise ValueError('Invalid trial ID')
    if existing_servers():raise RuntimeError('Existing Gazebo; no mixed world')
    if not all(frozen_check().values()):raise RuntimeError('Frozen Stage1.5 inputs differ')
    for exe in ('stage2_localized_interface.py','stage2_simulated_odometry.py','stage2_scan_relay.py'):
        if not os.access(ROOT/'install/uav_planning/lib/uav_planning'/exe,os.X_OK):raise RuntimeError('Missing executable '+exe)
    grid=GridRoute(MAP,clearance=.4);walls=sdf_walls();reference=json.loads(FIELD.read_text())['reference_route']
    nominal=offline_leg((4.7,.5),(4.7,1.15),grid,walls,reference)
    out.mkdir();write(out/'nominal_preflight.json',nominal)
    observed.OUT=out;observed.NAME=trial_id
    profile=ROOT/'src/uav_bringup/config/stage2_odometry_zero.yaml'
    common=['gui:=false','rviz:=false','auto_goal:=false','odometry_profile:='+str(profile)]
    observed.LAUNCH=['ros2','launch','uav_bringup','provincial_stage2_scan_startup_sensors.launch.py']+common
    nav_command=['ros2','launch','uav_bringup','provincial_stage2_scan_startup_navigation.launch.py']+common
    recorder_command=[sys.executable,'-B',str(ROOT/'tools/stage2/run_scan_resume_startup_trial.py'),
        '--name',trial_id,'--output-dir',str(out)]
    extras=['src/uav_bringup/config/stage2_odometry_zero.yaml','src/uav_planning/CMakeLists.txt',
        'tools/stage2/run_scan_resume_startup_experiment.py','tools/stage2/run_scan_resume_startup_trial.py',
        'tools/stage2/audit_scan_resume_startup_v4.py']
    for file in ('stage2_scan_startup_split.py','provincial_stage2_scan_startup_sensors.launch.py',
        'provincial_stage2_scan_startup_navigation.launch.py','provincial_stage2_scan_resume.launch.py'):
        extras+=['src/uav_bringup/launch/'+file,'install/uav_bringup/share/uav_bringup/launch/'+file]
    for file in ('stage2_odometry_model','stage2_simulated_odometry','stage2_navigation_interface',
        'stage2_localized_interface','stage2_wall_localization','stage2_scan_relay'):
        extras+=['src/uav_planning/scripts/'+file+'.py','install/uav_planning/lib/uav_planning/'+file+'.py']
    observed.INPUTS=tuple(dict.fromkeys(observed.INPUTS+tuple(extras)))
    observed.PARAM_NODES=('/gazebo_navigation_interface','/ego_planner_node','/ego_goal_adapter',
        '/ego_trajectory_executor','/stage2_simulated_odometry','/stage2_scan_relay')
    manifest=observed.capture_inputs();manifest['runner_argv']=recorder_command
    manifest['navigation_only_launch_argv']=nav_command
    manifest['mission_plan']=dict(external_goal_xy=[4.7,1.15],external_goals=1,reset=False,stage=2,profile='zero',
        localization='opt-in scan correction, no GT feedback',capture_before_localization=True)
    write(out/'input_manifest.json',manifest)
    progress=initial_progress(trial_id)
    processes=[];logs=[];node=None;recorder=None
    try:
        log=(out/(trial_id+'.runner.log')).open('x');logs.append(log)
        recorder=subprocess.Popen(recorder_command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(recorder)
        wait_file(out/'recorder_constructed.json',processes,15)
        write(out/'world_start.json',dict(monotonic_ns=time.monotonic_ns(),command=observed.LAUNCH))
        log=(out/'launch.log').open('x');logs.append(log)
        world=subprocess.Popen(observed.LAUNCH,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(world)
        progress.update(launches=1,status='WAITING_SENSOR_CAPTURE');write(out/'progress.json',progress)
        sensor=wait_file(out/'sensors_capture_ready.json',processes)
        if sensor['ready'] is not True:raise RuntimeError('Sensors not actually ready')
        if len(server_processes())!=1:raise RuntimeError('Expected one Gazebo')
        write(out/'navigation_start.json',dict(monotonic_ns=time.monotonic_ns(),command=nav_command))
        log=(out/'navigation_launch.log').open('x');logs.append(log)
        nav=subprocess.Popen(nav_command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(nav)
        progress.update(navigation_only_launches=1,status='WAITING_NAV_CAPTURE');write(out/'progress.json',progress)
        wait_file(out/'navigation_capture_ready.json',processes)
        observed.capture_runtime_params()
        rclpy.init();node=PoseObserver();pose=node.fresh_pose()
        if math.dist(pose[:2],(4.7,.5))>.2 or abs(pose[2]-math.pi/2)>.05:raise RuntimeError('Unexpected actual spawn')
        write(out/(trial_id+'.offline_preflight.json'),dict(measured_start_gt=pose,**offline_leg(pose[:2],(4.7,1.15),grid,walls,reference)))
        write(out/'odometry_graph.json',wait_odometry_graph(node));write(out/'scan_graph.json',wait_scan_graph(node))
        if not scan_runtime_parameters_valid(out):raise RuntimeError('Runtime parameters differ')
        write(out/'allow_goal.json',dict(monotonic_ns=time.monotonic_ns(),only_one_goal_authorized=True))
        progress.update(goals_requested=1,status='RUNNING_ONE_SHORT_GOAL');write(out/'progress.json',progress)
        recorder.wait(timeout=110)
        progress.update(runner_returncode=recorder.returncode,status='FINISHED_PENDING_AUDIT')
    except Exception as error:
        progress.update(status='FAILED_PRESERVED',failure=repr(error))
        if not (out/'cancel_capture.json').exists():write(out/'cancel_capture.json',dict(error=repr(error)))
    finally:
        if node is not None:node.destroy_node();rclpy.shutdown()
        # End only this experiment, allowing the single recorder to close first.
        for process in reversed(processes):stop(process)
        for log in logs:log.close()
        progress['remaining_gazebo_servers']=server_processes();write(out/'progress.json',progress)
        observed.final_hashes(manifest)
    try:report=audit(out)
    except (OSError,ValueError,TypeError,KeyError,IndexError) as error:report=dict(all_pass=False,status='INCOMPLETE_EVIDENCE',error=repr(error))
    write(out/'independent_audit_startup_v4.json',report)
    print(json.dumps({k:report.get(k) for k in ('all_pass','checks','loss_resume','error')},ensure_ascii=False),flush=True)
    if not all(frozen_check().values()):raise RuntimeError('Frozen inputs changed')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--trial-id',required=True);a=p.parse_args()
    raise SystemExit(0 if run(a.output_dir,a.trial_id)['all_pass'] else 2)
