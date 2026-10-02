#!/usr/bin/env python3
"""One world per explicitly selected phase; no retries/automatic goal resends."""
import sys,json,time,os,signal,subprocess,argparse,hashlib
import yaml
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'));sys.path.insert(0,str(ROOT/'tools/stage2'))
import run_provincial_full_observed as observed
from run_scan_resume_experiment import frozen_check,existing_servers

def write(p,data):
    with p.open('x') as f:json.dump(data,f,indent=2)

def stop(p):
    if p.poll() is not None:return
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=15)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=8)

def parameter_snapshot(node,out):
    command=[sys.executable,'-B',str(Path(__file__).with_name('dump_parameters.py')),node]
    started=time.monotonic_ns()
    try:
        result=subprocess.run(command,capture_output=True,text=True,timeout=25)
        evidence={'command':command,'started_monotonic_ns':started,
                  'ended_monotonic_ns':time.monotonic_ns(),'returncode':result.returncode,
                  'stdout':result.stdout,'stderr':result.stderr}
    except subprocess.TimeoutExpired as error:
        evidence={'command':command,'started_monotonic_ns':started,
                  'ended_monotonic_ns':time.monotonic_ns(),'returncode':None,
                  'error':repr(error),'stdout':str(error.stdout or ''),'stderr':str(error.stderr or '')}
        write(out/(node.lstrip('/')+'.query.json'),evidence)
        raise RuntimeError('Actual parameter query timed out: '+node) from error
    write(out/(node.lstrip('/')+'.query.json'),evidence)
    if result.returncode:
        raise RuntimeError('Actual parameters unavailable: '+node+'; see '+node.lstrip('/')+'.query.json')
    (out/(node.lstrip('/')+'.yaml')).write_text(result.stdout)

def run(out,phase):
    if out.exists():raise FileExistsError(out)
    if existing_servers():raise RuntimeError('Gazebo already running')
    parameter_reader=Path(__file__).with_name('dump_parameters.py')
    if not parameter_reader.is_file():raise FileNotFoundError(parameter_reader)
    assert all(frozen_check().values())
    out.mkdir();observed.OUT=out;observed.NAME='lio_'+phase+'_01'
    observed.LAUNCH=['ros2','launch','uav_bringup','provincial_stage2_lio_noise_model.launch.py',
        'gui:=false','rviz:=false','navigation:='+('false' if phase=='stationary' else 'true'),'auto_goal:=false']
    extras=[]
    for folder in ['src/stage2_lio_sim','tools/stage2/lio','tools/stage2/lio_model_validation','tools/stage2/lio_model_validation_v2','src/third_party/spark-fast-lio/spark_fast_lio']:
        extras += [str(p.relative_to(ROOT)) for p in (ROOT/folder).rglob('*') if p.is_file() and '__pycache__' not in str(p) and p.suffix in ['.py','.cpp','.h','.hpp','.xml','.txt','.yaml']]
    extras+=['src/uav_bringup/launch/provincial_stage2_lio_noise_model.launch.py','src/uav_bringup/config/spark_provincial_noise_model_sim.yaml',
        'src/uav_bringup/worlds/provincial_stage2_lio.sdf','install/spark_fast_lio/lib/spark_fast_lio/spark_lio_mapping']
    for p in (ROOT/'install/stage2_lio_sim/lib/stage2_lio_sim').glob('*.py'):extras.append(str(p.relative_to(ROOT)))
    for p in (ROOT/'install/uav_planning/lib/uav_planning').glob('stage2*.py'):extras.append(str(p.relative_to(ROOT)))
    for f in ['launch/provincial_stage2_lio_noise_model.launch.py','config/spark_provincial_noise_model_sim.yaml','worlds/provincial_stage2_lio.sdf']:
        extras.append('install/uav_bringup/share/uav_bringup/'+f)
    observed.INPUTS=tuple(dict.fromkeys(observed.INPUTS+tuple(extras)))
    manifest=observed.capture_inputs()
    cmd=[sys.executable,'-B',str(Path(__file__).with_name('observe_stationary.py' if phase=='stationary' else 'run_navigation_trial.py')),'--output',str(out)]
    if phase!='stationary':cmd+=['--phase',phase]
    manifest['recorder_argv']=cmd;manifest['stage4_phase']=phase
    write(out/'experiment_metadata.json',{'recorder_argv':cmd,'stage4_phase':phase,
        'launch_command':observed.LAUNCH,'input_manifest':'input_manifest.json'})
    proc=[];logs=[];progress={'phase':phase,'launches':0,'goals_requested':0,'status':'PREPARED'}
    try:
        for command,logname in [(cmd,'recorder.log'),(observed.LAUNCH,'launch.log')]:
            if proc:
                end=time.monotonic()+20
                while not (out/'recorder_ready.json').exists():
                    if proc[0].poll() is not None or time.monotonic()>end:raise RuntimeError('Recorder not ready')
                    time.sleep(.05)
            log=(out/logname).open('x');logs.append(log)
            proc.append(subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
            if logname=='launch.log':progress['launches']=1
        time.sleep(4)
        nodes=['/lio_mapping','/lio_sensor_adapter','/lio_odometry_adapter']
        if phase!='stationary':nodes+=list(observed.PARAM_NODES)
        (out/'runtime_parameters').mkdir()
        for node in nodes:
            parameter_snapshot(node,out/'runtime_parameters')
        parameters={p.stem:next(iter(yaml.safe_load(p.read_text()).values()))['ros__parameters']
            for p in (out/'runtime_parameters').glob('*.yaml')}
        gate={'declared_noise_model':parameters['lio_mapping']['mapping.acc_cov']==1e-6 and parameters['lio_mapping']['mapping.gyr_cov']==1e-8 and parameters['lio_mapping']['mapping.b_acc_cov']==0.0 and parameters['lio_mapping']['mapping.b_gyr_cov']==0.0, 'LIO_3D':parameters['lio_mapping']['common.planar_mode'] is False,
              'fixed_extrinsic':parameters['lio_mapping']['mapping.extrinsic_est_en'] is False,
              'no_clock_offset_estimation':parameters['lio_mapping']['common.time_sync_en'] is False}
        if phase!='stationary':
            nav=parameters['gazebo_navigation_interface'];planner=parameters['ego_planner_node'];executor=parameters['ego_trajectory_executor']
            gate.update(imperfect_sensors_disabled=nav['imperfect_sensors'] is False,
                existing_clearance=nav['grid_route_clearance']==.4,
                existing_body_guard=nav['provincial_rect_guard'] is True and nav['provincial_min_body_gap_m']==.08,
                existing_planner_limits=planner['grid_map/obstacles_inflation']==.22 and planner['manager/max_vel']==.25 and planner['manager/max_acc']==.35,
                existing_executor=executor['max_vel_x_mps']==executor['max_vel_y_mps']==.25 and executor['max_yaw_rate_rad_s']==.3 and executor['position_kp']==1.2 and executor['yaw_kp']==2. and executor['trajectory_start_clock']=='ros',
                external_goals_only=nav['auto_goal'] is False)
        write(out/'runtime_parameter_gate.json',{'pass':all(gate.values()),'checks':gate})
        if not all(gate.values()):raise RuntimeError('Actual runtime parameter gate rejected; do not send goal')
        parameter_files=list((out/'runtime_parameters').glob('*'))+[out/'runtime_parameter_gate.json']
        write(out/'runtime_parameter_sha256.json',{str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest()
              for p in parameter_files if p.is_file()})
        dependencies={}
        for binary in ['install/spark_fast_lio/lib/spark_fast_lio/spark_lio_mapping','install/stage2_lio_sim/lib/libstage2_kinematic_imu.so']:
            result=subprocess.run(['ldd',binary],capture_output=True,text=True,timeout=10)
            dependencies[binary]={'returncode':result.returncode,'output':result.stdout,'stderr':result.stderr}
        write(out/'system_link_dependencies.json',dependencies)
        if any(v['returncode'] or 'not found' in v['output'] for v in dependencies.values()):
            raise RuntimeError('LIO/IMU shared dependency missing')
        graph={}
        for topic in ['/gazebo/odometry','/lio/lidar','/lio/imu','/lio/odometry','/localization/lio_navigation_odometry','/goal_pose']:
            p=subprocess.run(['ros2','topic','info',topic,'--verbose','--no-daemon'],capture_output=True,text=True,timeout=10)
            graph[topic]={'returncode':p.returncode,'text':p.stdout}
        write(out/'actual_ros_graph.json',graph)
        # Parent cannot release navigation until parameter and graph evidence exists.
        write(out/'parent_preflight_ready.json',{'monotonic_ns':time.monotonic_ns(),'phase':phase})
        code=proc[0].wait(timeout=300 if phase=='full' else 100)
        progress.update(status='RECORDER_EXITED',recorder_returncode=code)
    except Exception as error:
        progress.update(status='EXPERIMENT_ERROR',error=repr(error))
    finally:
        # Stop recorder first to close its raw files; then stop only this world.
        for p in proc:stop(p)
        for f in logs:f.close()
        summary=out/('lio_'+phase+'_01.summary.json')
        if summary.exists():progress['goals_requested']=json.loads(summary.read_text())['goal_delivery'].get('publish_count',0)
        progress['remaining_gazebo_servers']=existing_servers()
        write(out/'progress.json',progress);observed.final_hashes(manifest)
    return progress

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--phase',choices=['full'],required=True);a=p.parse_args()
    result=run(a.output.resolve(),a.phase)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['status']=='RECORDER_EXITED' and result.get('recorder_returncode')==0 else 2)
