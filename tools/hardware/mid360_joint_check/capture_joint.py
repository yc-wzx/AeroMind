"""Read-only simultaneous CDR and serial capture. No actuator, TF or navigation publishers."""
import argparse,json,os,signal,subprocess,sys,time
from pathlib import Path
from review import HERE,sha,sources_ok,params_ok,save_review
sys.path.insert(0,str(HERE.parent/'mid360_gimbal_check'))
from record import Recorder
from variant import verify,MANIFEST

def snapshot(root):
    inputs=[]
    for directory in ['mid360','mid360_lio','mid360_lio_phased','mid360_gimbal_check','mid360_mount_check','xuc_feedback_check','mid360_joint_check']:
        for p in sorted((HERE.parent/directory).rglob('*')):
            if not p.is_file() or '__pycache__' in p.parts or p.suffix not in ['.py','.json','.yaml','.sh','.md']:continue
            relative=p.relative_to(HERE.parent);target=root/'input_snapshot'/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(p.read_bytes())
            inputs.append(dict(path=str(p),resolved=str(p.resolve()),sha256=sha(p),snapshot=str(target.relative_to(root))))
    (root/'local_inputs.json').write_text(json.dumps(inputs,indent=2)+'\n')
    return inputs

def serial_command(root,device,allow_test_pty=False):
    directory=HERE.parent/'xuc_feedback_check'
    if not allow_test_pty:
        return [sys.executable,'-B',str(directory/'capture.py'),'--device',str(device),'--baud','460800','--crc-profile','0x8408','--guided-turn','--output-dir',str(root/'encoder')]
    # Function-only virtual UART fixture hook; never exposed as a production CLI option.
    code="import sys;sys.path.insert(0,sys.argv[1]);from capture import capture_session;r=capture_session(sys.argv[2],sys.argv[3],460800,25,allow_test_pty=True,crc_profile='0x8408',guided_turn=True);raise SystemExit(0 if r.get('guided_turn_check',{}).get('motion_check_complete') else 2)"
    return [sys.executable,'-B','-c',code,str(directory),str(root/'encoder'),str(device)]

def run_trial(root,device,measurement_path,allow_test_pty=False):
    root=Path(root).resolve();root.mkdir(parents=True,exist_ok=False)
    recorder=None;proc=None;log=None;reader=None;health={};failure=None;interrupted=False
    before={};after={};sources_before={};sources_after={};sealed_before=False;sealed_after=False;inputs=[];command=[]
    try:
        verify();sealed_before=True
        (root/'variant_manifest_snapshot.json').write_bytes(MANIFEST.read_bytes())
        (root/'mount_snapshot.json').write_bytes(Path(measurement_path).read_bytes())
        from joint_math import mount
        mount(json.loads((root/'mount_snapshot.json').read_text()))
        inputs=snapshot(root);recorder=Recorder(root/'native.jsonl')
        print('检查LIO/IMU就绪；请保持底盘和云台静止。工具不发布运动指令。',flush=True)
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            if recorder.error:raise RuntimeError(recorder.error)
            if recorder.fresh():break
            time.sleep(.02)
        else:raise RuntimeError('LIO/IMU输入未就绪：'+json.dumps(recorder.counts))
        sources_before=recorder.publishers();before=recorder.parameters()
        if not sources_ok(sources_before) or not params_ok(before):raise RuntimeError('LIO/IMU来源或参数不匹配')
        if not recorder.fresh():raise RuntimeError('参数预检后输入已过期')
        print('已就绪。云台应已对准机械正前方，下面按自动提示进行25秒检查。',flush=True)
        command=serial_command(root,device,allow_test_pty)
        log=(root/'serial_launch.log').open('x');reader=(root/'serial_launch.log').open()
        proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+35
        while proc.poll() is None:
            if recorder.error:raise RuntimeError('ROS采集线程异常：'+recorder.error)
            if not recorder.fresh():raise RuntimeError('LIO/IMU输入失去新鲜度，请停止手动转动')
            if time.monotonic()>deadline:raise TimeoutError('串口采集超时，请停止手动转动')
            # Print only operator guidance; full child output remains in the append-only log.
            for line in reader:
                if line.startswith(('原位','现在','本次','Traceback')):print(line.rstrip(),flush=True)
            time.sleep(.02)
        for line in reader:
            if line.startswith(('原位','现在','本次','Traceback')):print(line.rstrip(),flush=True)
        if proc.returncode:raise RuntimeError('串口检查未通过；详见serial_launch.log和encoder/summary.json')
        sources_after=recorder.publishers();after=recorder.parameters()
    except KeyboardInterrupt:interrupted=True;failure='操作者中断'
    except Exception as error:failure=str(error)
    finally:
        if failure:print('本次结束，请停止手动转动。原因：'+failure,flush=True)
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=8)
            except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
        if recorder is not None:
            try:health=recorder.close()
            except Exception as error:failure=failure or 'Recorder close failed: '+str(error)
        if reader:reader.close()
        if log:log.close()
    try:verify();sealed_after=True
    except Exception:sealed_after=False
    unchanged=bool(inputs) and all(Path(i['path']).resolve()==Path(i['resolved']) and Path(i['path']).is_file() and sha(Path(i['path']))==i['sha256'] for i in inputs)
    files=['native.jsonl','mount_snapshot.json','variant_manifest_snapshot.json','local_inputs.json','serial_launch.log']
    files += ['encoder/'+n for n in ['raw.bin','reads.jsonl','capture_meta.json']]
    hashes={name:sha(root/name) for name in files if (root/name).is_file()}
    meta=dict(health=health,inputs_unchanged=unchanged,sealed_inputs_verified_before=sealed_before,sealed_inputs_verified_after=sealed_after,
        publishers_before=sources_before,publishers_after=sources_after,parameters_before=before,parameters_after=after,
        serial_returncode=proc.returncode if proc else None,serial_command=command,failure=failure,interrupted=interrupted,
        ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),data_sha256=hashes,
        operator_declaration='Fixed chassis, initial gimbal aligned to mechanical forward, manual single CCW turn then return',
        service_scope='Only ROS GetParameters requested; reset/set_pose service calls not monitored',
        dependency_scope='Local hardware tools copied; sealed raw/phased inputs verified; not a complete OS/runtime image')
    (root/'capture_meta.json').write_text(json.dumps(meta,indent=2,allow_nan=False)+'\n')
    try:result=save_review(root,'summary.json')
    except Exception as error:
        result=dict(evidence_complete=False,motion_check_complete=False,navigation_ready=False,capture_failure=failure,analysis_error=str(error))
        with (root/'summary.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(result,indent=2,allow_nan=False));print('SUMMARY:',root/'summary.json',flush=True)
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--device',default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0');p.add_argument('--output-dir')
    p.add_argument('--measurements',default=str(HERE.parent/'mid360_mount_check/mount_measurements.operator_20261018_v1.json'));a=p.parse_args()
    root=Path(a.output_dir) if a.output_dir else Path.home()/'AeroMind_mid360_joint_checks'/('turn_return_'+time.strftime('%Y%m%d_%H%M%S'))
    result=run_trial(root,a.device,a.measurements)
    return 0 if result.get('motion_check_complete') else 2
if __name__=='__main__':raise SystemExit(main())
