"""Operator-timed, receive-only straight push/return recording. No motion publisher."""
import argparse,json,os,select,sys,time
from pathlib import Path
from translation_review import HERE,sha,sources_ok,params_ok,save_review
from translation_math import reference,mount
from serial_reader import FeedbackReader
from capture_joint import snapshot
from record import Recorder
from variant import verify,MANIFEST

def input_snapshot(root):
    files=snapshot(root)
    for p in sorted(HERE.glob('*')):
        if not p.is_file() or p.suffix not in ['.py','.sh','.md','.json']:continue
        target=root/'input_snapshot'/HERE.name/p.name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(p.read_bytes())
        files.append(dict(path=str(p),resolved=str(p.resolve()),sha256=sha(p),snapshot=str(target.relative_to(root))))
    (root/'local_inputs.json').write_text(json.dumps(files,indent=2)+'\n')
    return files

def run_trial(root,device,measurements,ref,operator=None,allow_test_pty=False):
    reference(ref);root=Path(root).resolve();root.mkdir(parents=True,exist_ok=False)
    ros=None;uart=None;event_file=None;health={};failure=None;interrupted=False;inputs=[];before={};after={};sources_before={};sources_after={};sealed_before=False;sealed_after=False
    def check(deadline):
        if time.monotonic()>deadline:raise TimeoutError('超过180秒，本次结束；请停止手推')
        if ros.error:raise RuntimeError('ROS recorder: '+ros.error)
        if not ros.fresh() or not uart.fresh():raise RuntimeError('LIO/IMU/电控输入过期或无效；请停止手推')
    def wait(label,prompt,deadline):
        print(prompt,flush=True);began=time.monotonic()
        if operator is None and not sys.stdin.isatty():raise RuntimeError('请在NoMachine交互终端运行')
        while True:
            check(deadline)
            if operator is not None:
                if operator(label,time.monotonic()-began):return
            elif select.select([sys.stdin],[],[],.05)[0]:
                line=sys.stdin.readline()
                if not line:raise RuntimeError('Operator input closed')
                if not line.strip():return
                print('这里只需按回车。',flush=True)
            time.sleep(.02)
    def mark(phase,edge):
        event_file.write(json.dumps(dict(phase=phase,edge=edge,receive_monotonic_ns=time.monotonic_ns(),receive_wall_ns=time.time_ns()))+'\n');event_file.flush()
    def hold(phase,deadline):
        mark(phase,'start');print('静止记录5秒：请停稳并松手，保持底盘和云台朝向。',flush=True);end=time.monotonic()+5
        while time.monotonic()<end:check(deadline);time.sleep(.02)
        mark(phase,'end')
    try:
        verify();sealed_before=True;(root/'variant_manifest_snapshot.json').write_bytes(MANIFEST.read_bytes())
        (root/'mount_snapshot.json').write_bytes(Path(measurements).read_bytes());mount(json.loads((root/'mount_snapshot.json').read_text()))
        (root/'reference.json').write_text(json.dumps(ref,indent=2,allow_nan=False)+'\n');inputs=input_snapshot(root)
        event_file=(root/'events.jsonl').open('x');ros=Recorder(root/'native.jsonl');uart=FeedbackReader(root/'encoder');uart.start(device,allow_test_pty=allow_test_pty)
        print('检查真实LIO/IMU/电控输入；本工具不发送电机指令。',flush=True);deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            if ros.error or uart.error:raise RuntimeError(ros.error or uart.error)
            if ros.fresh() and uart.fresh():break
            time.sleep(.02)
        else:raise RuntimeError('输入未就绪，不开始手推')
        sources_before=ros.publishers();before=ros.parameters()
        if not sources_ok(sources_before) or not params_ok(before):raise RuntimeError('来源或参数不匹配')
        deadline=time.monotonic()+180
        print('A-B实测距离：'+str(ref['distance_m'])+' m；全程云台保持机械前向。',flush=True)
        wait('ready','将同一个底盘固定标记对准A，底盘朝向沿A→B；确认停稳后按回车。',deadline)
        hold('baseline',deadline)
        wait('outward_start','准备沿底盘正前方慢推到B。先按回车，再开始推。不要抬起、转动底盘或云台。',deadline)
        mark('outward','start');wait('outward_end','现在慢推到B，用同一固定标记对准B。停稳松手后，回来按回车。',deadline);mark('outward','end')
        hold('far_hold',deadline)
        wait('return_start','准备保持原朝向，沿原直线倒推回A。先按回车再开始。',deadline)
        mark('return','start');wait('return_end','现在慢推回A，用同一标记对准A，停稳松手后按回车。',deadline);mark('return','end')
        hold('final_hold',deadline)
        sources_after=ros.publishers();after=ros.parameters()
    except KeyboardInterrupt:interrupted=True;failure='操作者中断'
    except Exception as error:failure=str(error)
    finally:
        if failure:print('本次结束，请停止手推。原因：'+failure,flush=True)
        if uart is not None:
            try:uart.close(failure,interrupted)
            except Exception as error:failure=failure or 'UART close failed: '+str(error)
        if ros is not None:
            try:health=ros.close()
            except Exception as error:failure=failure or 'ROS close failed: '+str(error)
        if event_file:event_file.close()
    try:verify();sealed_after=True
    except Exception:sealed_after=False
    try:unchanged=bool(inputs) and all(Path(i['path']).resolve()==Path(i['resolved']) and sha(Path(i['path']))==i['sha256'] for i in inputs)
    except Exception:unchanged=False
    names=['native.jsonl','events.jsonl','reference.json','mount_snapshot.json','local_inputs.json','variant_manifest_snapshot.json','encoder/raw.bin','encoder/reads.jsonl','encoder/capture_meta.json']
    meta=dict(health=health,publishers_before=sources_before,publishers_after=sources_after,parameters_before=before,parameters_after=after,
        inputs_unchanged=unchanged,sealed_inputs_before=sealed_before,sealed_inputs_after=sealed_after,failure=failure,interrupted=interrupted,
        data_sha256={n:sha(root/n) for n in names if (root/n).is_file()},ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),
        operator_declaration='Same fixed chassis mark at measured A/B; straight ground push and return without chassis/gimbal yaw change',
        service_scope='Only GetParameters requested; no reset/set_pose calls monitored',dependency_scope='Local tools snapshot plus sealed build verification, not an OS image')
    (root/'capture_meta.json').write_text(json.dumps(meta,indent=2,allow_nan=False)+'\n')
    try:result=save_review(root,'summary.json')
    except Exception as error:
        result=dict(evidence_complete=False,diagnostic_complete=False,accuracy_accepted=False,navigation_ready=False,capture_failure=failure,analysis_error=str(error))
        with (root/'summary.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(result,indent=2,allow_nan=False));print('SUMMARY:',root/'summary.json',flush=True)
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--distance-m',required=True,type=float);p.add_argument('--device',default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0');p.add_argument('--output-dir')
    p.add_argument('--measurements',default=str(HERE.parent/'mid360_mount_check/mount_measurements.operator_20261018_v1.json'));a=p.parse_args()
    ref=dict(distance_m=a.distance_m,source='Operator tape-measured ground A-B distance',reference_point='Same fixed chassis mark aligned to A and B',direction='A to B along initial fixed chassis +X',uncertainty_m=None,independently_tracked=False)
    root=Path(a.output_dir) if a.output_dir else Path.home()/'AeroMind_mid360_translation_checks'/('push_return_'+time.strftime('%Y%m%d_%H%M%S'))
    result=run_trial(root,a.device,a.measurements,ref)
    return 0 if result.get('diagnostic_complete') else 2
if __name__=='__main__':raise SystemExit(main())
