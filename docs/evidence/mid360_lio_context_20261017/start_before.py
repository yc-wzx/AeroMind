#!/usr/bin/env python3
import json
import os
import sys
import time
from pathlib import Path
import yaml
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'mid360'))
from core import POINTS,IMU_SI,ODOM,stamp_ns,inspect_cloud,imu_values,sha
from preflight import MANIFEST,verify_build
from inspect_raw import inspect

def live_ready():
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2,Imu
    context=Context();rclpy.init(context=context)
    node=Node('mid360_lio_input_precheck',context=context);latest={};counts={POINTS:0,IMU_SI:0};errors=[]
    def receive(topic,msg):
        try:
            stamp=stamp_ns(msg)
            inspect_cloud(msg) if topic==POINTS else imu_values(msg)
            old=latest.get(topic)
            if old is not None and stamp<=old['stamp_ns']:raise ValueError('nonmonotonic live '+topic)
            latest[topic]=dict(stamp_ns=stamp,receive_monotonic_s=time.monotonic())
            counts[topic]+=1
        except ValueError as error:errors.append(str(error))
    subs=[node.create_subscription(kind,topic,lambda msg,t=topic:receive(t,msg),qos_profile_sensor_data)
          for topic,kind in [(POINTS,PointCloud2),(IMU_SI,Imu)]]
    try:
        deadline=time.monotonic()+8.;endpoints={}
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
            if node.get_publishers_info_by_topic(ODOM):raise RuntimeError('A LIO odometry publisher already exists; do not start a duplicate')
            now=time.monotonic()
            if counts[POINTS]>=5 and counts[IMU_SI]>=20:
                endpoints={t:[p.node_name for p in node.get_publishers_info_by_topic(t)] for t in counts}
                if (endpoints[POINTS]==['mid360_real_driver'] and endpoints[IMU_SI]==['mid360_imu_unit_adapter']
                    and now-latest[POINTS]['receive_monotonic_s']<.3
                    and now-latest[IMU_SI]['receive_monotonic_s']<.1
                    and abs(latest[POINTS]['stamp_ns']-latest[IMU_SI]['stamp_ns'])*1e-9<.2
                    and not errors):
                    return dict(pass_=True,counts=counts,latest=latest,publishers=endpoints,
                                timestamps_modified=False,pose_scope='sensor-local only; no navigation frames')
        raise RuntimeError('Live raw/SI inputs not ready: '+json.dumps(dict(counts=counts,errors=errors[-5:])))
    finally:
        node.destroy_node();rclpy.shutdown(context=context)

def main():
    if len(sys.argv)!=2:raise RuntimeError('Expected raw capture summary.json path')
    verify_build();raw_report=inspect(sys.argv[1])
    params=yaml.safe_load((HERE/'lio_mid360_static.yaml').read_text())['/**']['ros__parameters']
    if params['common']['visualization_frame']!='imu' or params['common']['imu_frame']!='mid360_imu':
        raise RuntimeError('LIO visualization MODE must be imu; physical IMU FRAME is separately mid360_imu')
    ready=live_ready()
    out=Path.home()/'AeroMind_mid360_lio_checks'/('start_'+time.strftime('%Y%m%d_%H%M%S'))
    out.mkdir(parents=True,exist_ok=False)
    command=['ros2','launch',str(HERE/'readonly_lio.launch.py')]
    record=dict(scope='REAL READ-ONLY SENSOR-LOCAL LIO START; NOT NAVIGATION ACCEPTANCE',
                command=command,raw_summary=str(Path(sys.argv[1]).resolve()),raw_summary_sha256=sha(sys.argv[1]),
                raw_build_manifest_sha256=sha(MANIFEST),raw_inspection=raw_report,live_ready=ready,
                new_lio_stage_inputs={str(p):sha(p) for p in HERE.rglob('*') if p.is_file() and '__pycache__' not in p.parts},
                corrected_visualization_mode='imu',physical_imu_frame='mid360_imu',motor_commands=False)
    (out/'startup_inputs.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(raw_report,indent=2),flush=True);print('LIO START INPUTS:',out/'startup_inputs.json',flush=True)
    os.execvpe(command[0],command,os.environ)

if __name__=='__main__':main()
