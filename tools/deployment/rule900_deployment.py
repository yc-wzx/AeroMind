#!/usr/bin/env python3
"""Source rebuild, machine-local manifest and read-only simulation deployment checks."""
import argparse,ast,hashlib,importlib,json,math,os,shutil,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
NAME='provincial_rule900_scratch_sim'
PROFILE=ROOT/'tools/stage2/lio_scratch_reuse_profile'
PREFIX=ROOT/'install'/NAME
RUNTIME=PREFIX/'lib'/NAME/'lio'
MANIFEST=PREFIX/'share'/NAME/'deployment_manifest.json'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def source_inputs():
    inputs=set()
    for directory in [ROOT/'src'/NAME,PROFILE,ROOT/'src/third_party/ego-planner-swarm',ROOT/'src/uav_bringup/config',ROOT/'src/uav_planning/config',ROOT/'tools/deployment']:
        inputs.update(p for p in directory.rglob('*') if p.is_file() and '__pycache__' not in p.parts and not any(v in p.parts for v in ['.git','runtime','CMakeFiles']))
    return {str(p.relative_to(ROOT)):dict(sha256=sha(p),resolved=str(p.resolve())) for p in sorted(inputs)}
def run(command,**kwargs):subprocess.run([str(v) for v in command],check=True,**kwargs)
def build(jobs):
    build=ROOT/'build/rule900_lio';checks=ROOT/'build/rule900_checks';checks.mkdir(parents=True,exist_ok=True)
    before=source_inputs()
    run(['cmake','-S',PROFILE/'source','-B',build,'-DAEROMIND_BUILD_ISOLATED_TESTS=ON'])
    targets=['spark_lio_component','spark_lio_mapping','production_prediction_tests','calibration_tests','keyframe_tests']
    run(['cmake','--build',build,'--target',*targets,'-j'+str(jobs)])
    env=dict(os.environ,ROS_DOMAIN_ID='217',ROS_LOCALHOST_ONLY='1',AEROMIND_LIO_CONSISTENCY_FIX='1',AEROMIND_DECLARED_UNBIASED_IMU='1',AEROMIND_LIO_KEYFRAME_MAP='1',AEROMIND_PLANE_QUALITY_GUARD='0',LD_LIBRARY_PATH=str(build)+':'+os.environ.get('LD_LIBRARY_PATH',''))
    for key in ['AEROMIND_INPUT_TRACE','AEROMIND_LIO_TRACE','AEROMIND_MAP_LINEAGE_FILE','AEROMIND_PLANE_QUALITY_TRACE','AEROMIND_INTERNAL_TIMING_DIR','LD_PRELOAD']:env.pop(key,None)
    for name in targets[2:]:
        with (checks/(name+'.log')).open('w') as stream:
            run(['unshare','--user','--map-root-user','--mount','--propagation','private','bash',PROFILE/'readonly_source.sh',build/name],env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=60)
    if before!=source_inputs():raise RuntimeError('Source changed during build/tests; do not install')
    RUNTIME.mkdir(parents=True,exist_ok=True)
    for name in ['spark_lio_mapping','libspark_lio_component.so']:shutil.copy2(build/name,RUNTIME/name)
    (PREFIX/'share'/NAME/'lio').mkdir(exist_ok=True)
    shutil.copy2(PROFILE/'parameters.yaml',PREFIX/'share'/NAME/'lio/parameters.yaml')
    installed={}
    for p in PREFIX.rglob('*'):
        if p.is_file() and p!=MANIFEST and '__pycache__' not in p.parts:installed[str(p.relative_to(ROOT))]=dict(sha256=sha(p),resolved=str(p.resolve()))
    planner=ROOT/'install/ego_planner/lib/ego_planner/ego_planner_node'
    installed[str(planner.relative_to(ROOT))]=dict(sha256=sha(planner),resolved=str(planner.resolve()))
    result=dict(format=1,scope='Rebuilt simulation only; not historical binary qualification or hardware acceptance',build_root=str(ROOT),git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),sources=before,installed=installed,production_tests={name:True for name in targets[2:]},historical_flow_result_is_reference_only=True,real_sensors_qualified=False,system_image=False)
    MANIFEST.write_text(json.dumps(result,indent=2)+'\n')
    print('Machine-local build manifest:',MANIFEST)
def verify_manifest():
    data=json.loads(MANIFEST.read_text())
    if data['build_root']!=str(ROOT):raise RuntimeError('Checkout moved; rebuild in current directory')
    actual=source_inputs()
    if actual!=data['sources']:raise RuntimeError('Source inputs differ from local build manifest; rebuild')
    for path,item in data['installed'].items():
        p=ROOT/path
        if not p.is_file() or sha(p)!=item['sha256'] or str(p.resolve())!=item['resolved']:raise RuntimeError('Installed input changed: '+path)
    return data
def check():
    data=verify_manifest();scripts=PREFIX/'lib'/NAME;sys.path.insert(0,str(scripts))
    modules=[]
    for p in sorted(scripts.glob('*.py')):
        module=importlib.import_module(p.stem)
        if Path(module.__file__).resolve().parent!=scripts.resolve():raise RuntimeError('Foreign overlay module: '+p.stem)
        modules.append(p.stem)
    for library in [RUNTIME/'libspark_lio_component.so',PREFIX/'lib/libinverted_position_imu.so',PREFIX/'lib/libstage2_gimbal_joint.so']:
        result=subprocess.run(['ldd','-r',str(library)],capture_output=True,text=True)
        if result.returncode or any(s in result.stdout+result.stderr for s in ['not found','undefined symbol:','=> /opt/MVS/']):raise RuntimeError('Library resolution failed: '+str(library)+'\n'+result.stdout+result.stderr)
    from circle_route_gate import route_preflight
    from grid_route import GridRoute
    cfg=PREFIX/'share'/NAME/'config';field=json.loads((cfg/'provincial_rule900_training.json').read_text());grid=GridRoute(cfg/'provincial_rule900_training.pgm',clearance=.40)
    routes=[route_preflight(a,b,grid,field['collision_polygons'],field['reference_route']) for a,b in [((4.7,.5),(8.7,4.25)),((8.7,4.25),(4.7,.5))]]
    if not all(v['pass'] for v in routes):raise RuntimeError('Offline reachability failed')
    print(json.dumps(dict(pass_=True,scope='Read-only deployment/import/library/offline checks; no ROS motion target sent',modules=len(modules),sources=len(data['sources']),installed_files=len(data['installed']),routes=[dict(length_m=sum(math.dist(a,b) for a,b in zip([v['start']]+v['reference_stages'],v['reference_stages'])),minimum_circle_gap_m=v['minimum_circle_gap_m']) for v in routes],classification='TRAINING-ONLY / PROVISIONAL'),indent=2))
def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['build','check']);p.add_argument('--jobs',type=int,default=2);a=p.parse_args()
    if a.jobs<1:p.error('jobs must be positive')
    if a.mode=='build':build(a.jobs)
    else:check()
if __name__=='__main__':main()
