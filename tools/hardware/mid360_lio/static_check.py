#!/usr/bin/env python3
"""Native sensor-local LIO stationary drift evidence, never a ground-truth claim."""
import argparse,base64,json,math,queue,sys,threading,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'mid360'))
from core import ODOM,stamp_ns,stream_stats,sha
from preflight import MANIFEST,verify_build

def pose_row(message):
    p=message.pose.pose.position;q=message.pose.pose.orientation;v=message.twist.twist
    pos=[p.x,p.y,p.z];quat=[q.x,q.y,q.z,q.w]
    velocity=[v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z]
    cov=list(message.pose.covariance);tcov=list(message.twist.covariance)
    if not all(math.isfinite(n) for n in pos+quat+velocity+cov+tcov):raise ValueError('nonfinite odometry')
    if abs(sum(n*n for n in quat)-1)>1e-3:raise ValueError('invalid quaternion norm; no normalization substituted')
    if message.header.frame_id!='mid360_local_odom' or message.child_frame_id!='mid360_imu':
        raise ValueError('unexpected sensor-local output frames')
    return dict(position=pos,quaternion=quat,velocity=velocity,pose_covariance=cov,twist_covariance=tcov)

def drift_stats(rows):
    if not rows:return dict(samples=0)
    positions=np.asarray([r['position'] for r in rows]);q=np.asarray([r['quaternion'] for r in rows])
    displacement=np.linalg.norm(positions-positions[0],axis=1)
    dots=np.clip(np.abs(q@q[0])/(np.linalg.norm(q,axis=1)*np.linalg.norm(q[0])),0,1)
    angle=np.degrees(2*np.arccos(dots))
    return dict(samples=len(rows),first_position_m=positions[0].tolist(),last_position_m=positions[-1].tolist(),
                max_translation_from_first_m=float(max(displacement)),end_translation_from_first_m=float(displacement[-1]),
                max_rotation_from_first_deg=float(max(angle)),end_rotation_from_first_deg=float(angle[-1]),
                meaning='Relative estimated sensor drift under operator-declared stationary condition; not true position error')

class Writer:
    def __init__(self,path):
        self.path=path;self.q=queue.Queue(2048);self.received=0;self.written=0;self.overflow=0;self.error=None
        self.thread=threading.Thread(target=self.work);self.thread.start()
    def work(self):
        try:
            with self.path.open('x') as stream:
                while True:
                    row=self.q.get()
                    if row is None:break
                    stream.write(json.dumps(row,allow_nan=False)+'\n');self.written+=1
        except Exception as error:self.error=str(error)
    def put(self,row):
        self.received+=1
        try:self.q.put_nowait(row)
        except queue.Full:self.overflow+=1
    def close(self):
        while self.thread.is_alive():
            try:self.q.put(None,timeout=.1);break
            except queue.Full:continue
        self.thread.join(20)
        lines=sum(1 for _ in self.path.open()) if self.path.exists() else 0
        return dict(received=self.received,written=self.written,actual_lines=lines,overflow=self.overflow,error=self.error,closed=not self.thread.is_alive())

class Observer(Node):
    def __init__(self,writer):
        super().__init__('mid360_static_lio_observer');self.writer=writer;self.rows=[];self.accepting=True
        self.subscription=self.create_subscription(Odometry,ODOM,self.receive,qos_profile_sensor_data)
    def receive(self,message):
        if not self.accepting:return
        row=dict(sequence=len(self.rows)+1,stamp_ns=None,receive_wall_ns=time.time_ns(),receive_monotonic_ns=time.monotonic_ns(),error=None)
        try:row['stamp_ns']=stamp_ns(message);row.update(pose_row(message))
        except ValueError as error:
            row['error']=str(error);row['original_cdr_base64']=base64.b64encode(serialize_message(message)).decode()
        self.rows.append(row);self.writer.put(row)
    def actual_params(self):
        client=self.create_client(GetParameters,'/hardware/mid360/lio/mid360_real_lio/get_parameters')
        if not client.wait_for_service(timeout_sec=2):return dict(error='parameter service unavailable')
        request=GetParameters.Request();request.names=['use_sim_time','common.visualization_frame','common.imu_frame','common.map_frame','common.base_frame','common.time_sync_en','mapping.extrinsic_T','mapping.extrinsic_R']
        future=client.call_async(request);deadline=time.monotonic()+2
        while not future.done() and time.monotonic()<deadline:rclpy.spin_once(self,timeout_sec=.01)
        if not future.done():return dict(error='parameter query timeout')
        return {name:dict(type=v.type,string_value=v.string_value,bool_value=v.bool_value,double_array_value=list(v.double_array_value)) for name,v in zip(request.names,future.result().values)}

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--seconds',type=float,default=30);args=parser.parse_args()
    if not 15<=args.seconds<=60:parser.error('seconds must be 15..60')
    verify_build();input_sha=sha(MANIFEST)
    stage_hashes={str(p):sha(p) for p in HERE.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    out=Path.home()/'AeroMind_mid360_lio_checks'/('static_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir(parents=True,exist_ok=False)
    writer=Writer(out/'odometry.jsonl');rclpy.init();node=Observer(writer);interrupted=False
    try:
        deadline=time.monotonic()+15
        while not node.rows and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        deadline=time.monotonic()+args.seconds if node.rows else time.monotonic()
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
        node.accepting=False
        publishers=[dict(name=p.node_name,namespace=p.node_namespace,type=p.topic_type) for p in node.get_publishers_info_by_topic(ODOM)]
        parameters=node.actual_params()
    except KeyboardInterrupt:interrupted=True;publishers=[];parameters={}
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        health=writer.close()
    rows=node.rows;stats=stream_stats(rows,required_span_s=max(1.,args.seconds-.5))
    good=[r for r in rows if r['error'] is None]
    inputs_unchanged=input_sha==sha(MANIFEST) and all(Path(p).is_file() and sha(p)==v for p,v in stage_hashes.items())
    params_ok=(parameters.get('common.visualization_frame',{}).get('string_value')=='imu' and
               parameters.get('common.imu_frame',{}).get('string_value')=='mid360_imu' and
               parameters.get('use_sim_time',{}).get('type')==1 and parameters['use_sim_time']['bool_value'] is False)
    complete=bool(stats['pass_'] and stats.get('max_receive_gap_s',math.inf)<=.2 and stats.get('max_sensor_gap_s',math.inf)<=.2 and
                  health['closed'] and not health['error'] and health['overflow']==0 and
                  health['received']==health['written']==health['actual_lines']==len(rows) and
                  len(publishers)==1 and publishers[0]['name']=='mid360_real_lio' and params_ok and inputs_unchanged and not interrupted)
    result=dict(scope='REAL SENSOR-LOCAL STATIC LIO DIAGNOSTIC; NOT NAVIGATION ACCEPTANCE',
                evidence_complete=complete,stream=stats,drift=drift_stats(good),health=health,publishers=publishers,
                actual_parameters=parameters,inputs_unchanged=inputs_unchanged,raw_build_manifest_sha256=input_sha,
                lio_stage_input_sha256=stage_hashes,interrupted=interrupted,
                receive_wall_minus_device_s_median=float(np.median([(r['receive_wall_ns']-r['stamp_ns'])*1e-9 for r in good])) if good else None,
                data_sha256={'odometry.jsonl':sha(out/'odometry.jsonl')},
                limits=['Operator must keep sensor/gimbal/chassis stationary','No ground truth','No chassis centre or gravity-aligned map qualification','No drift acceptance threshold invented','No motor commands or navigation feedback'])
    (out/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2,allow_nan=False));print('SUMMARY:',out/'summary.json',flush=True)
    raise SystemExit(0 if complete else 2)
if __name__=='__main__':main()
