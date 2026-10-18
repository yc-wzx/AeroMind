"""New guarded LIO session; preserved driver and old installs are never changed."""
import argparse, hashlib, json, os, signal, subprocess, sys, time
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path.insert(0,str(HERE.parent/'mid360_lio_phased'))
from variant import verify,INSTALL,MANIFEST
sys.path.insert(0,str(HERE.parent/'mid360_lio'))
from inspect_raw import inspect

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def stop(proc):
    if proc is None or proc.poll() is not None:return
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(8)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid,signal.SIGTERM)
        try:proc.wait(5)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(5)

def main():
    p=argparse.ArgumentParser();p.add_argument('raw_summary');p.add_argument('--output-dir');a=p.parse_args()
    verify();raw=inspect(a.raw_summary)
    import rclpy
    from rclpy.node import Node
    rclpy.init();n=Node('mid360_guard_session_precheck')
    try:
        for _ in range(20):rclpy.spin_once(n,timeout_sec=.05)
        active=n.get_publishers_info_by_topic('/hardware/mid360/lio/odometry')
        checked=n.get_publishers_info_by_topic('/hardware/mid360/checked/points')
        if active or checked:raise RuntimeError('Existing LIO or input-guard publisher; refusing duplicate session')
    finally:n.destroy_node();rclpy.shutdown()
    out=Path(a.output_dir) if a.output_dir else Path.home()/'AeroMind_mid360_guard_checks'/('session_'+time.strftime('%Y%m%d_%H%M%S'))
    out.mkdir(parents=True,exist_ok=False)
    files=[p for p in HERE.iterdir() if p.is_file() and p.suffix in ['.py','.sh','.md']]
    snapshot=out/'input_snapshot';snapshot.mkdir()
    for f in files:(snapshot/f.name).write_bytes(f.read_bytes())
    binary=INSTALL/'spark_fast_lio/lib/spark_fast_lio/spark_lio_mapping'
    config=HERE.parent/'mid360_lio/lio_mid360_static.yaml'
    command=[str(binary),'--ros-args','-r','__node:=mid360_real_lio','-r','__ns:=/hardware/mid360/lio',
             '--params-file',str(config),'-r','lidar:=/hardware/mid360/checked/points','-r','imu:=/hardware/mid360/checked/imu_si']
    manifest=dict(scope='ISOLATED VALIDATED INPUT SESSION; SAME PHASED ESTIMATOR',raw_precheck=raw,
        old_variant_manifest_sha256=sha(MANIFEST),binary=dict(path=str(binary),resolved=str(binary.resolve()),sha256=sha(binary)),
        config=dict(path=str(config),sha256=sha(config)),tools={str(f):sha(f) for f in files},command=command,
        automatic_restart=False,parameters_changed=False,input_remappings_changed=True,
        git_head=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip())
    (out/'startup_inputs.json').write_text(json.dumps(manifest,indent=2)+'\n')
    guard=None;lio=None;failure=None;interrupted=False
    print('GUARDED SESSION:',out,flush=True)
    with (out/'guard.log').open('x') as glog,(out/'lio.log').open('x') as llog:
        try:
            guard=subprocess.Popen([sys.executable,'-B',str(HERE/'input_guard.py'),'--output-dir',str(out/'guard')],stdout=glog,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+15
            while True:
                if guard.poll() is not None:raise RuntimeError('Input guard ended before LIO launch')
                status=out/'guard/guard_status.json'
                state=json.loads(status.read_text()) if status.exists() else {}
                if state.get('state')=='FAULT_LATCHED':raise RuntimeError('Input guard fault: '+str(state.get('failure')))
                if state.get('state')=='READY':break
                if time.monotonic()>deadline:raise RuntimeError('Input guard did not become ready')
                time.sleep(.02)
            lio=subprocess.Popen(command,stdout=llog,stderr=subprocess.STDOUT,start_new_session=True)
            print('LIO STARTED; validated original messages only. No automatic restart.',flush=True)
            while True:
                if guard.poll() is not None:raise RuntimeError('Guard ended; stopping LIO to prevent stale-state continuation')
                if lio.poll() is not None:raise RuntimeError('LIO ended unexpectedly')
                state=json.loads((out/'guard/guard_status.json').read_text())
                if state.get('state')=='FAULT_LATCHED':raise RuntimeError('Input validity fault: '+str(state.get('failure')))
                if time.monotonic_ns()-state['monotonic_ns']>500_000_000:raise RuntimeError('Guard status stale')
                time.sleep(.02)
        except KeyboardInterrupt:interrupted=True
        except Exception as error:failure=str(error);print('SESSION STOP:',failure,flush=True)
        finally:stop(lio);stop(guard)
    verify()
    report=dict(failure=failure,operator_stopped=interrupted,guard_exit=guard.returncode if guard else None,
                lio_exit=lio.returncode if lio else None,old_sealed_inputs_verified=True,
                local_tools_unchanged=all(sha(f)==manifest['tools'][str(f)] for f in files),
                data_sha256={str(f.relative_to(out)):sha(f) for f in out.rglob('*') if f.is_file()},
                navigation_or_motor_commands=False,automatic_restart=False)
    (out/'session_end.json').write_text(json.dumps(report,indent=2)+'\n')
    return 2 if failure else 0
if __name__=='__main__':raise SystemExit(main())
