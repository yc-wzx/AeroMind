#!/usr/bin/env python3
"""One zero-error Full, then one Full per isolated synthetic error profile."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import rclpy

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools'))
import run_provincial_full_observed as observed
from run_provincial_forward_integration import MAP, FIELD, PoseObserver, offline_leg, server_processes
from run_provincial_safety_fix_matrix import existing_servers, wait_for_launch
from summarize_provincial_stage15_terminal_v3 import sdf_walls
from grid_route import GridRoute
from audit_localized import audit

BASE = ROOT/'tools/results/stage2_localization_20261001'
PROFILES = ('zero','scale','yaw_bias','random_walk')

def write(p, d):
    p.write_text(json.dumps(d, ensure_ascii=False, indent=2)+'\n')

def frozen_check():
    a=json.loads((ROOT/'tools/results/provincial_stage15_completion_20261001/provisional_stage15_completion_audit_v3.json').read_text())
    return {p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==v
            for p,v in a['runtime_input_sha256'].items()}

def wait_odometry_graph(node, timeout=10):
    deadline=time.monotonic()+timeout
    stable_since=None
    observations=[]
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
        truth=node.get_subscriptions_info_by_topic('/gazebo/odometry')
        injected=node.get_subscriptions_info_by_topic('/simulation/navigation_odometry')
        pubs=node.get_publishers_info_by_topic('/simulation/navigation_odometry')
        graph={'truth_subscriber_names':[s.node_name for s in truth],
               'injected_subscriber_names':[s.node_name for s in injected],
               'injected_publisher_names':[s.node_name for s in pubs],
               'injected_endpoints':[{'name':s.node_name,'gid':list(s.endpoint_gid),
                    'type':str(s.endpoint_type),'qos':str(s.qos_profile)} for s in injected+pubs],
               'scope':'Observed graph before goal; not a packet-level information-flow proof.'}
        valid=('stage2_simulated_odometry' in graph['truth_subscriber_names'] and
               'gazebo_navigation_interface' not in graph['truth_subscriber_names'] and
               'gazebo_navigation_interface' in graph['injected_subscriber_names'] and
               graph['injected_publisher_names']==['stage2_simulated_odometry'] and
               '_NODE_NAME_UNKNOWN_' not in graph['truth_subscriber_names']+graph['injected_subscriber_names'])
        observations.append({'monotonic_s':time.monotonic(),'valid':valid})
        stable_since=(stable_since or time.monotonic()) if valid else None
        if stable_since is not None and time.monotonic()-stable_since>=1:
            graph['stable_s']=time.monotonic()-stable_since
            graph['observations']=observations
            return graph
        time.sleep(.05)
    raise RuntimeError('Ground-truth isolation graph identity did not become stable')

def run_profile(profile):
    out=BASE/profile
    if out.exists():raise FileExistsError(out)
    if existing_servers():raise RuntimeError('Existing Gazebo; refuse new world')
    for executable in ('stage2_simulated_odometry.py','stage2_localized_interface.py'):
        if not os.access(ROOT/'install/uav_planning/lib/uav_planning'/executable,os.X_OK):
            raise RuntimeError('New installed node is not executable: '+executable)
    before=frozen_check()
    if not all(before.values()):raise RuntimeError('Frozen Stage 1.5 inputs changed')
    grid=GridRoute(MAP,clearance=.4);walls=sdf_walls();ref=json.loads(FIELD.read_text())['reference_route']
    nominal=offline_leg((4.7,.5),(8.7,4.25),grid,walls,ref)
    out.mkdir();write(out/'nominal_preflight.json',nominal)
    name='stage2_localized_'+profile+'_01'
    profile_path='src/uav_bringup/config/stage2_odometry_'+profile+'.yaml'
    observed.OUT=out;observed.NAME=name
    observed.LAUNCH=['ros2','launch','uav_bringup','provincial_stage2_localized.launch.py',
                     'gui:=false','rviz:=false','auto_goal:=false',
                     'odometry_profile:='+str(ROOT/profile_path)]
    extras=['src/uav_planning/CMakeLists.txt',profile_path,
            'src/uav_bringup/launch/provincial_stage2_localized.launch.py',
            'install/uav_bringup/share/uav_bringup/launch/provincial_stage2_localized.launch.py',
            'tools/stage2/run_localized_experiment.py','tools/stage2/run_localized_trial.py','tools/stage2/audit_localized.py']
    extras.append('tools/stage2/stage2_terminal_evidence.py')
    for module in ('stage2_odometry_model','stage2_simulated_odometry','stage2_navigation_interface','stage2_localized_interface','stage2_wall_localization'):
        extras += ['src/uav_planning/scripts/'+module+'.py','install/uav_planning/lib/uav_planning/'+module+'.py']
    observed.INPUTS=tuple(dict.fromkeys(observed.INPUTS+tuple(extras)))
    observed.PARAM_NODES=('/gazebo_navigation_interface','/ego_planner_node',
                          '/ego_goal_adapter','/ego_trajectory_executor','/stage2_simulated_odometry')
    manifest=observed.capture_inputs()
    manifest['runner_argv']=[sys.executable,'-B',str(ROOT/'tools/stage2/run_localized_trial.py'),
                            '--name',name,'--output-dir',str(out)]
    manifest['mission_plan']={'external_goals':1,'reset':False,'stage':2,'profile':profile,
                              'synthetic_error_not_hardware_calibrated':True, 'localization':'opt-in 2D scan-to-known-wall correction, no truth feedback'}
    write(out/'input_manifest.json',manifest)
    progress={'name':name,'profile':profile,'profile_path':profile_path,
              'launches':0,'goals_requested':0,'status':'PREPARED'}
    launch=node=log=None
    try:
        log=(out/'launch.log').open('x')
        launch=subprocess.Popen(observed.LAUNCH,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        progress.update(launches=1,launch_pid=launch.pid);write(out/'progress.json',progress)
        wait_for_launch()
        progress['gazebo_server_processes']=server_processes()
        if len(progress['gazebo_server_processes'])!=1:raise RuntimeError('Not one Gazebo server')
        observed.capture_runtime_params()
        rclpy.init();node=PoseObserver();actual=node.fresh_pose()
        if math.dist(actual[:2],(4.7,.5))>.2 or abs(actual[2]-math.pi/2)>.05:
            raise RuntimeError('Unexpected actual initial pose')
        measured=offline_leg(actual[:2],(8.7,4.25),grid,walls,ref)
        write(out/(name+'.offline_preflight.json'),dict(measured_start_gt=actual,**measured))
        graph=wait_odometry_graph(node)
        write(out/'odometry_graph.json',graph)
        if ('gazebo_navigation_interface' in graph['truth_subscriber_names'] or
            'gazebo_navigation_interface' not in graph['injected_subscriber_names'] or
            graph['injected_publisher_names']!=['stage2_simulated_odometry']):
            raise RuntimeError('Ground-truth isolation graph precheck failed')
        progress['goals_requested']=1
        progress['status']='RUNNING_ONE_FULL';write(out/'progress.json',progress)
        with (out/(name+'.runner.log')).open('x') as runner_log:
            result=subprocess.run(manifest['runner_argv'],stdout=runner_log,stderr=subprocess.STDOUT,timeout=270)
        progress.update(runner_returncode=result.returncode,status='FINISHED_PENDING_AUDIT')
    except Exception as error:
        progress.update(status='FAILED_PRESERVED',failure=type(error).__name__+': '+str(error))
    finally:
        if node is not None:node.destroy_node();rclpy.shutdown()
        if launch:
            try:os.killpg(launch.pid,signal.SIGINT)
            except ProcessLookupError:pass
            try:launch.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=8)
        if log:log.close()
        progress['remaining_gazebo_servers']=server_processes()
        write(out/'progress.json',progress)
        observed.final_hashes(manifest)
    try:
        report=audit(out)
    except (OSError,ValueError,KeyError,TypeError,IndexError) as error:
        report={'all_pass':False,'status':'INCOMPLETE_EVIDENCE','error':type(error).__name__+': '+str(error),
                'progress':progress,'profile':profile}
    write(out/'independent_audit_v1.json',report)
    print(json.dumps({'profile':profile,'all_pass':report['all_pass'],'checks':report.get('checks'),
                      'error':report.get('error'),'odometry':report.get('odometry')},ensure_ascii=False),flush=True)
    if not all(frozen_check().values()):raise RuntimeError('Frozen inputs changed during trial')
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--profile',required=True,choices=PROFILES);a=p.parse_args()
    if a.profile!='zero':
        previous=json.loads((BASE/'zero/independent_audit_v2.json').read_text())
        if not previous['all_pass']:raise RuntimeError('Zero-error Full has not passed; error run refused')
    r=run_profile(a.profile)
    sys.exit(0 if r['all_pass'] else 2)
