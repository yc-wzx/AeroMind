import json,os,signal,subprocess
from pathlib import Path
ROOT=Path('/home/rmnav/aeromind-transfer-20261016');STAGE=ROOT/'tools/hardware/mid360_lio_phased'
OUT=ROOT/'tools/results/mid360_lio_phased_20261017';env=dict(os.environ,MID360_ROS_DOMAIN_ID='219')
query=r'''
import json,rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
rclpy.init();node=Node('phase_launcher_probe')
try:
    client=node.create_client(GetParameters,'/hardware/mid360/lio/mid360_real_lio/get_parameters')
    assert client.wait_for_service(timeout_sec=8),'Actual LIO parameter service unavailable'
    request=GetParameters.Request();request.names=['common.odometry_output_contract','use_sim_time','common.visualization_frame']
    future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=3)
    assert future.done()
    values=future.result().values
    assert values[0].string_value=='scan_corrected_primary_v1'
    assert values[1].type==1 and values[1].bool_value is False
    assert values[2].string_value=='imu'
    print(json.dumps(dict(actual_contract=values[0].string_value,use_sim_time=False,visualization_mode='imu',
                         scope='Actual candidate launch / namespace / parameter service without sensor data; no navigation acceptance')))
finally:
    node.destroy_node();rclpy.shutdown()
'''
with (OUT/'actual_launch.log').open('x') as log:
    child=subprocess.Popen(['bash',str(STAGE/'env.sh'),'ros2','launch',str(STAGE/'readonly_lio.launch.py')],
        cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        result=subprocess.run(['bash',str(STAGE/'env.sh'),'python3','-B','-c',query],cwd=ROOT,env=env,
                              text=True,capture_output=True,timeout=20)
        (OUT/'actual_launch_query.txt').write_text(result.stdout+result.stderr)
        assert result.returncode==0,result.stderr
        report=json.loads(result.stdout.strip())
        matches=[]
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit():continue
            try:
                command=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode()
                maps=(proc/'maps').read_text()
            except (OSError,UnicodeError):continue
            if str(ROOT/'install_mid360_lio_phased/spark_fast_lio/lib/spark_fast_lio/spark_lio_mapping') in command:
                relevant=[line.split()[-1] for line in maps.splitlines() if '/libspark_lio_component.so' in line]
                matches.append(dict(pid=int(proc.name),component_paths=sorted(set(relevant))))
        assert len(matches)==1,matches
        assert matches[0]['component_paths']==[str(ROOT/'install_mid360_lio_phased/spark_fast_lio/lib/libspark_lio_component.so')]
        report['actual_process_maps']=matches
        (OUT/'actual_launch_verification.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
    finally:
        os.killpg(child.pid,signal.SIGINT)
        try:child.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
