import json,math,os,signal,subprocess,sys,time
from pathlib import Path
from unittest.mock import patch
ROOT=Path('/home/rmnav/aeromind-transfer-20261016');STAGE=ROOT/'tools/hardware/mid360_gimbal_check';PHASE=STAGE.parent/'mid360_lio_phased'
OUT=ROOT/'tools/results/mid360_gimbal_20261018';control=OUT/'control.json'
def set_control(start_angle,target):
    temp=OUT/'control.new'
    temp.write_text(json.dumps(dict(start=time.monotonic()+.1,duration=5.,from_angle=start_angle,target=target)))
    temp.replace(control)
set_control(0.,0.)
fixture=Path(__file__).parent/'mid360_gimbal_synthetic_source_20261018.py'
env=dict(os.environ,MID360_ROS_DOMAIN_ID='220')
with (OUT/'synthetic_source.txt').open('x') as source_log:
    source=subprocess.Popen(['bash',str(PHASE/'env.sh'),'python3','-B',str(fixture),str(control)],env=env,cwd=ROOT,
                            stdout=source_log,stderr=subprocess.STDOUT,start_new_session=True)
    script=r'''
import json,math,sys,time
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path.cwd()/'tools/hardware/mid360_gimbal_check'))
import record
root=Path.cwd()/'tools/results/mid360_gimbal_20261018';control=root/'control.json'
def set_control(start,target):
    temp=root/'control.next';temp.write_text(json.dumps(dict(start=time.monotonic()+.1,duration=5.,from_angle=start,target=target)));temp.replace(control)
def operator(tag,prompt,deadline):
    print('SYNTHETIC OPERATOR:',tag,flush=True)
    if tag=='start_turn':set_control(0.,math.pi/4)
    elif tag=='start_return':set_control(math.pi/4,0.)
    elif tag in ['finish_turn','finish_return']:time.sleep(5.3)
with patch.object(record,'wait_enter',side_effect=operator),patch('rclpy.get_global_executor',side_effect=AssertionError('Global executor forbidden')):
    code=record.main()
raise SystemExit(code)
'''
    try:
        run=subprocess.run(['bash',str(PHASE/'env.sh'),'python3','-B','-c',script],cwd=ROOT,env=env,text=True,capture_output=True,timeout=60)
        (OUT/'synthetic_capture.txt').write_text(run.stdout+run.stderr)
    finally:
        os.killpg(source.pid,signal.SIGINT)
        try:source.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(source.pid,signal.SIGTERM);source.wait(timeout=5)
if run.returncode:raise RuntimeError('Recorder E2E failed; see '+str(OUT/'synthetic_capture.txt'))
summary_path=Path(next(line[len('SUMMARY: '):] for line in run.stdout.splitlines() if line.startswith('SUMMARY: ')))
summary=json.loads(summary_path.read_text());assert summary['motion_check_complete']
repeat=subprocess.run(['bash',str(PHASE/'env.sh'),'python3','-B',str(STAGE/'analyze.py'),str(summary_path.parent),'--output-name','independent_repeat.json'],
                      cwd=ROOT,env=env,text=True,capture_output=True,timeout=20)
(OUT/'synthetic_reaudit.txt').write_text(repeat.stdout+repeat.stderr);assert repeat.returncode==0
reaudit=json.loads((summary_path.parent/'independent_repeat.json').read_text());assert reaudit==summary
report=dict(scope='Synthetic inverted-sensor/pivot-motion fixture, interactive recorder with scripted keyboard and independent reader; NOT REAL LIO ESTIMATION',
    domain=220,evidence_complete=True,motion_check_complete=True,independent_audit_identical=True,
    capture=str(summary_path),outward_deg=summary['outward_about_estimated_up_deg'],return_deg=summary['return_about_estimated_up_deg'],
    gyro_outward_deg=summary['imu_outward']['about_baseline_up_deg'],gyro_return_deg=summary['imu_return']['about_baseline_up_deg'],
    sensor_outward_displacement_m=summary['outward_sensor_displacement_m'],position_closure_m=summary['final_sensor_position_closure_m'],
    health=summary['health'],gates=summary['gates'])
(OUT/'synthetic_e2e.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
