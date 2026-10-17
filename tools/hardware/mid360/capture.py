#!/usr/bin/env python3
"""Native per-receipt raw bag + metadata capture; no navigation publications."""
import argparse
import collections
import json
import math
import queue
import threading
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu, PointCloud2
import rosbag2_py
from core import POINTS,IMU_RAW,IMU_SI,G,stamp_ns,imu_values,inspect_cloud,stream_stats,sha
from preflight import MANIFEST,verify_build

TOPICS={POINTS:(PointCloud2,'sensor_msgs/msg/PointCloud2'),IMU_RAW:(Imu,'sensor_msgs/msg/Imu'),IMU_SI:(Imu,'sensor_msgs/msg/Imu')}

class Recorder:
    def __init__(self,out):
        self.out=out;self.queue=queue.Queue(maxsize=512);self.received=collections.Counter();self.written=collections.Counter()
        self.overflow=0;self.error=None;self.ready=threading.Event();self.thread=threading.Thread(target=self.work)
        self.thread.start();self.ready.wait(10)
        if not self.ready.is_set() or self.error:raise RuntimeError('Raw writer could not start: '+str(self.error))
    def put(self,topic,serialized,row):
        self.received[topic]+=1
        try:self.queue.put_nowait((topic,serialized,row))
        except queue.Full:self.overflow+=1
    def work(self):
        writer=None
        try:
            writer=rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(self.out/'raw_bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
            for topic,(_,kind) in TOPICS.items():writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type=kind,serialization_format='cdr'))
            with (self.out/'receipts.jsonl').open('x') as stream:
                self.ready.set()
                while True:
                    item=self.queue.get()
                    if item is None:break
                    topic,serialized,row=item
                    # Storage time is receive-wall time. Header/device time remains untouched in CDR.
                    writer.write(topic,serialized,row['receive_wall_ns'])
                    stream.write(json.dumps(row,allow_nan=False)+'\n');self.written[topic]+=1
        except Exception as error:self.error=type(error).__name__+': '+str(error)
        finally:
            if writer is not None:del writer
            self.ready.set()
    def close(self):
        while self.thread.is_alive():
            try:self.queue.put(None,timeout=.1);break
            except queue.Full:continue
        self.thread.join(30)
        return dict(received=dict(self.received),written=dict(self.written),queue_overflow=self.overflow,
                    writer_error=self.error,thread_closed=not self.thread.is_alive())

class Observer(Node):
    def __init__(self,recorder):
        super().__init__('mid360_readonly_capture');self.recorder=recorder;self.rows=collections.defaultdict(list)
        self.subs=[self.create_subscription(kind,topic,lambda msg,t=topic:self.receive(t,msg),qos_profile_sensor_data) for topic,(kind,_) in TOPICS.items()]
    def receive(self,topic,message):
        received=time.monotonic_ns();wall=time.time_ns()
        row=dict(topic=topic,sequence=len(self.rows[topic])+1,receive_monotonic_ns=received,
                 receive_wall_ns=wall,frame=message.header.frame_id,stamp_ns=None,error=None)
        try:
            row['stamp_ns']=stamp_ns(message)
            row['wall_minus_device_s']=(wall-row['stamp_ns'])*1e-9
            row.update(inspect_cloud(message) if topic==POINTS else imu_values(message))
        except (ValueError,TypeError,AttributeError,BufferError) as error:row['error']=str(error)
        self.rows[topic].append(row)
        self.recorder.put(topic,serialize_message(message),row)

def summarize(rows,health,ledger):
    stats={t:stream_stats(rows[t]) for t in TOPICS}
    for t,s in stats.items():
        limit=.3 if t==POINTS else .1
        rates=(8.,12.) if t==POINTS else (150.,250.)
        s['precheck_receive_gap_limit_s']=limit;s['precheck_rate_range_hz']=rates
        s['pass_']=s['pass_'] and s.get('max_receive_gap_s',math.inf)<=limit and s.get('max_sensor_gap_s',math.inf)<=limit and rates[0]<=s.get('receive_hz',0)<=rates[1]
    raw=[r for r in rows[IMU_RAW] if not r['error']]
    si=[r for r in rows[IMU_SI] if not r['error']]
    mean=np.mean([r['acc'] for r in raw],axis=0).tolist() if raw else None
    raw_scale_ok=bool(raw) and all(.5<=r['acc_norm']<=1.5 for r in raw)
    si_scale_ok=bool(si) and all(.5*G<=r['acc_norm']<=1.5*G for r in si)
    # Match actual messages; never fill missing SI samples with old values.
    by_stamp={r['stamp_ns']:r for r in raw}
    pairs=[(by_stamp[r['stamp_ns']],r) for r in si if r['stamp_ns'] in by_stamp]
    match_ok=bool(pairs) and all(all(abs(a-b*G)<1e-9 for a,b in zip(s['acc'],r['acc'])) and s['gyro']==r['gyro'] for r,s in pairs)
    coverage=len(pairs)/max(1,min(len(raw),len(si)))
    imu_interval=(max((r['stamp_ns'] for r in raw),default=0)-min((r['stamp_ns'] for r in raw),default=0))*1e-9
    point_rows=[r for r in rows[POINTS] if not r['error']]
    overlap=0.
    if raw and point_rows:
        overlap=max(0.,(min(raw[-1]['stamp_ns'],point_rows[-1]['stamp_ns'])-max(raw[0]['stamp_ns'],point_rows[0]['stamp_ns']))*1e-9)
    bag_counts={}
    if health['thread_closed'] and health['writer_error'] is None:
        reader=rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(ledger/'raw_bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
        counts=collections.Counter()
        while reader.has_next():topic,_,_=reader.read_next();counts[topic]+=1
        bag_counts=dict(counts);del reader
    healthy=(health['thread_closed'] and health['writer_error'] is None and health['queue_overflow']==0 and health['received']==health['written']==bag_counts)
    return dict(scope='REAL MID360 RAW INPUT PRECHECK',
                pass_raw=all(s['pass_'] for s in stats.values()) and raw_scale_ok and si_scale_ok and match_ok and coverage>.98 and overlap>=10 and healthy,
                streams=stats,raw_acceleration_unit='g (pinned unmodified Livox driver2)',
                si_acceleration_unit='m/s^2; raw unchanged; separate conversion topic',
                raw_mean_acc_g=mean,raw_unit_check=raw_scale_ok,si_unit_check=si_scale_ok,
                imu_conversion_matched_pairs=len(pairs),imu_conversion_coverage=coverage,imu_conversion_check=match_ok,
                sensor_timestamp_overlap_s=overlap,imu_sensor_span_s=imu_interval,
                health=health,bag_message_counts=bag_counts,bag_counts_verified=healthy,
                timestamp_note='Original device stamps preserved; wall/device epoch difference reported, not silently corrected',
                reset_observation='No reset/set_pose services monitored; capture tool publishes no commands',
                limitations=['No real position truth','No navigation or motor integration','No body/gimbal lever-arm calibration','No statistical reliability or IMU noise calibration'])

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--seconds',type=float,default=15);parser.add_argument('--output-dir',type=Path)
    args=parser.parse_args()
    if args.seconds<12 or args.seconds>60:parser.error('Capture duration must be 12..60 seconds')
    verify_build()
    out=args.output_dir or Path.home()/'AeroMind_mid360_captures'/time.strftime('%Y%m%d_%H%M%S')
    out.mkdir(parents=True,exist_ok=False)
    recorder=Recorder(out);rclpy.init();node=Observer(recorder);interrupted=False
    try:
        deadline=time.monotonic()+args.seconds
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        endpoints={t:[dict(node_name=p.node_name,namespace=p.node_namespace,type=p.topic_type) for p in node.get_publishers_info_by_topic(t)] for t in TOPICS}
    except KeyboardInterrupt:interrupted=True;endpoints={}
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        health=recorder.close()
    result=summarize(node.rows,health,out)
    endpoint_ok=all(len(endpoints.get(t,[]))==1 and endpoints[t][0]['node_name']==('mid360_imu_unit_adapter' if t==IMU_SI else 'mid360_real_driver') for t in TOPICS)
    result.update(publishers=endpoints,publisher_identity_check=endpoint_ok,interrupted=interrupted,
                  build_manifest_sha256=sha(MANIFEST),output_dir=str(out))
    result['pass_raw']=bool(result['pass_raw'] and endpoint_ok and not interrupted)
    result['data_sha256']={str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file()}
    (out/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2,allow_nan=False));print('SUMMARY:',out/'summary.json',flush=True)
    raise SystemExit(0 if result['pass_raw'] else 2)
if __name__=='__main__':main()
