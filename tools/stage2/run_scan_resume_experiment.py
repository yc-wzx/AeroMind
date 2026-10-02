#!/usr/bin/env python3
"""Exactly one short scan-loss/resume trial; no live retry."""
import argparse
import hashlib
import json
import math
import os
import re
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
from audit_scan_resume_v2 import audit

BASE = ROOT/'tools/results/stage2_scan_resume_20261001'
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


def wait_scan_graph(node, timeout=10):
    deadline=time.monotonic()+timeout;since=None;observations=[]
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
        def endpoints(topic, kind):
            getter=node.get_subscriptions_info_by_topic if kind=='sub' else node.get_publishers_info_by_topic
            return [dict(name=e.node_name,gid=list(e.endpoint_gid),qos=str(e.qos_profile)) for e in getter(topic)]
        graph={t+':'+k:endpoints(t,k) for t in ('/scan','/guarded_scan') for k in ('sub','pub')}
        names=lambda t,k:[e['name'] for e in graph[t+':'+k]]
        valid=(names('/scan','pub')==['gazebo_bridge'] and 'stage2_scan_relay' in names('/scan','sub') and
               'gazebo_navigation_interface' not in names('/scan','sub') and
               names('/guarded_scan','pub')==['stage2_scan_relay'] and
               'gazebo_navigation_interface' in names('/guarded_scan','sub') and
               all(e['name']!='_NODE_NAME_UNKNOWN_' for v in graph.values() for e in v))
        observations.append(dict(monotonic_ns=time.monotonic_ns(),valid=valid))
        since=(since or time.monotonic()) if valid else None
        if since is not None and time.monotonic()-since>=1:
            return dict(valid=True,endpoints=graph,stable_s=time.monotonic()-since,observations=observations,
                        scope='Observed pre-goal graph; no packet-level or all-service monitoring claim')
    raise RuntimeError('Scan relay input graph not stable/isolated')

def scan_runtime_parameters_valid(out):
    import yaml
    params=next(iter(yaml.safe_load((out/'runtime_parameters/gazebo_navigation_interface.yaml').read_text()).values()))['ros__parameters']
    from stage2_wall_localization import verify_lidar_contract
    contract=verify_lidar_contract(params['localization_world_sdf'],params['localization_scan_frame_id'])
    write(out/'lidar_contract.json',contract)
    return (params['imperfect_sensors'] is False and params['grid_route_clearance']==.4 and
            params['localization_match_timeout_s']==.5)

def run_profile(profile, *, output_dir=None, trial_id='stage2_scan_resume_01'):
    if profile != 'zero':raise ValueError('This controlled trial requires zero profile')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', trial_id):
        raise ValueError('Invalid trial ID')
    out=Path(output_dir).resolve() if output_dir is not None else BASE
    if out.exists():raise FileExistsError(out)
    if existing_servers():raise RuntimeError('Existing Gazebo; refuse new world')
    for executable in ('stage2_simulated_odometry.py','stage2_localized_interface.py','stage2_scan_relay.py'):
        if not os.access(ROOT/'install/uav_planning/lib/uav_planning'/executable,os.X_OK):
            raise RuntimeError('New installed node is not executable: '+executable)
    before=frozen_check()
    if not all(before.values()):raise RuntimeError('Frozen Stage 1.5 inputs changed')
    grid=GridRoute(MAP,clearance=.4);walls=sdf_walls();ref=json.loads(FIELD.read_text())['reference_route']
    nominal=offline_leg((4.7,.5),(4.7,1.15),grid,walls,ref)
    out.mkdir();write(out/'nominal_preflight.json',nominal)
    name=trial_id
    profile_path='src/uav_bringup/config/stage2_odometry_'+profile+'.yaml'
    observed.OUT=out;observed.NAME=name
    observed.LAUNCH=['ros2','launch','uav_bringup','provincial_stage2_scan_resume.launch.py',
                     'gui:=false','rviz:=false','auto_goal:=false',
                     'odometry_profile:='+str(ROOT/profile_path)]
    extras=['src/uav_planning/CMakeLists.txt',profile_path,
            'src/uav_bringup/launch/provincial_stage2_scan_resume.launch.py',
            'install/uav_bringup/share/uav_bringup/launch/provincial_stage2_scan_resume.launch.py',
            'tools/stage2/run_scan_resume_experiment.py','tools/stage2/run_scan_resume_trial.py','tools/stage2/audit_scan_resume.py','tools/stage2/test_scan_resume.py']
    extras.append('tools/stage2/stage2_terminal_evidence.py')
    for module in ('stage2_odometry_model','stage2_simulated_odometry','stage2_navigation_interface','stage2_localized_interface','stage2_wall_localization','stage2_scan_relay'):
        extras += ['src/uav_planning/scripts/'+module+'.py','install/uav_planning/lib/uav_planning/'+module+'.py']
    observed.INPUTS=tuple(dict.fromkeys(observed.INPUTS+tuple(extras)))
    observed.PARAM_NODES=('/gazebo_navigation_interface','/ego_planner_node',
                          '/ego_goal_adapter','/ego_trajectory_executor','/stage2_simulated_odometry','/stage2_scan_relay')
    manifest=observed.capture_inputs()
    manifest['runner_argv']=[sys.executable,'-B',str(ROOT/'tools/stage2/run_scan_resume_trial.py'),
                            '--name',name,'--output-dir',str(out)]
    manifest['mission_plan']={'external_goal_xy':[4.7,1.15],'external_goals':1,'reset':False,'stage':2,'profile':profile,
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
        measured=offline_leg(actual[:2],(4.7,1.15),grid,walls,ref)
        write(out/(name+'.offline_preflight.json'),dict(measured_start_gt=actual,**measured))
        graph=wait_odometry_graph(node)
        write(out/'odometry_graph.json',graph)
        write(out/'scan_graph.json',wait_scan_graph(node))
        if ('gazebo_navigation_interface' in graph['truth_subscriber_names'] or
            'gazebo_navigation_interface' not in graph['injected_subscriber_names'] or
            graph['injected_publisher_names']!=['stage2_simulated_odometry']):
            raise RuntimeError('Ground-truth isolation graph precheck failed')
        if not scan_runtime_parameters_valid(out):raise RuntimeError('Scan guard runtime parameters invalid')
        progress['goals_requested']=1
        progress['status']='RUNNING_ONE_SHORT_SCAN_RESUME';write(out/'progress.json',progress)
        with (out/(name+'.runner.log')).open('x') as runner_log:
            result=subprocess.run(manifest['runner_argv'],stdout=runner_log,stderr=subprocess.STDOUT,timeout=120)
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
    p=argparse.ArgumentParser()
    p.add_argument('--output-dir',required=True,type=Path)
    p.add_argument('--trial-id',required=True)
    a=p.parse_args()
    r=run_profile('zero',output_dir=a.output_dir,trial_id=a.trial_id)
    sys.exit(0 if r['all_pass'] else 2)
