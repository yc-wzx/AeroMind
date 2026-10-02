#!/usr/bin/env python3
"""Read-only CDR sensor audit. Raw unmatched startup data remain visible."""
import sys,json,base64,math,hashlib,argparse
from collections import Counter,defaultdict
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/stage2_lio_sim/scripts'))
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2,Imu
from lio_contract import points,pack_mid360,valid_imu,stamp_s

def audit_sensor_file(path,start_stamp=None,end_stamp=None):
    source_cloud={};source_imu={};counts=Counter();last={};max_gap=defaultdict(float)
    faults=[];matches=Counter();orphans=Counter();imu_norm=[];zvalues=[]
    active_last={};active_gap=defaultdict(float);active_count=Counter();active_first={};active_stamp_last={}
    for line in path.open():
        row=json.loads(line);topic=row['topic'];counts[topic]+=1
        if row['receive_sequence']!=counts[topic]:faults.append('receive sequence '+topic)
        recv=row['receive_monotonic_ns']*1e-9;t=row['message_stamp_s']
        if topic in last:
            max_gap[topic]=max(max_gap[topic],recv-last[topic][0])
            if recv<=last[topic][0] or t<=last[topic][1]:faults.append('nonmonotonic '+topic)
        last[topic]=(recv,t)
        if (start_stamp is None or t>=start_stamp) and (end_stamp is None or t<=end_stamp):
            active_count[topic]+=1
            active_first.setdefault(topic,t);active_stamp_last[topic]=t
            if topic in active_last:active_gap[topic]=max(active_gap[topic],recv-active_last[topic])
            active_last[topic]=recv
        typ=PointCloud2 if row['ros_type']=='sensor_msgs/msg/PointCloud2' else Imu
        m=deserialize_message(base64.b64decode(row['cdr'],validate=True),typ)
        if abs(stamp_s(m.header.stamp)-t)>1e-9:faults.append('CDR/header disagreement')
        key=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
        if topic=='/simulation/lidar3d/points':source_cloud[key]=m
        elif topic=='/simulation/imu3d':
            valid_imu(m);source_imu[key]=m
            imu_norm.append(math.sqrt(sum(getattr(m.linear_acceleration,k)**2 for k in 'xyz')))
        elif topic=='/lio/lidar':
            if key not in source_cloud:orphans[topic]+=1;continue
            xyz,invalid=points(source_cloud[key]);expected=pack_mid360(xyz,key)
            if bytes(m.data)!=expected or m.header.frame_id!='lio_lidar' or m.point_step!=32:
                faults.append('formatted cloud differs from instantaneous source')
            matches[topic]+=1;zvalues.extend(float(x) for x in xyz[:,2])
        elif topic=='/lio/imu':
            if key not in source_imu:orphans[topic]+=1;continue
            a=source_imu[key]
            if any(getattr(getattr(m,v),k)!=getattr(getattr(a,v),k) for v in ['angular_velocity','linear_acceleration'] for k in 'xyz'):
                faults.append('IMU measurement changed')
            if m.header.frame_id!='lio_imu' or m.orientation_covariance[0]!=-1 or any(getattr(m.orientation,k)!=0 for k in 'xyzw'):
                faults.append('absolute orientation leaked')
            matches[topic]+=1
        elif topic=='/lio/cloud_registered':
            if m.header.frame_id!='lio_odom' or m.width*m.height<24:faults.append('invalid registered cloud')
    # Arrival cross-topic order is not generation order. Resolve early outputs
    # against complete raw source dictionaries in a second streaming pass.
    if orphans:
        for line in path.open():
            row=json.loads(line);topic=row['topic']
            if topic not in ['/lio/lidar','/lio/imu']:continue
            m=deserialize_message(base64.b64decode(row['cdr']),PointCloud2 if topic=='/lio/lidar' else Imu)
            key=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
            source=source_cloud if topic=='/lio/lidar' else source_imu
            if key not in source:faults.append('unmatched source '+topic)
            elif topic=='/lio/lidar':
                xyz,_=points(source[key])
                if bytes(m.data)!=pack_mid360(xyz,key):faults.append('second-pass cloud mismatch')
            else:
                a=source[key]
                if any(getattr(getattr(m,v),k)!=getattr(getattr(a,v),k) for v in ['angular_velocity','linear_acceleration'] for k in 'xyz'):faults.append('second-pass IMU mismatch')
    health=json.loads(path.with_suffix('.health.json').read_text())
    coverage={t:((start_stamp is None or active_first.get(t,float('inf'))-start_stamp<=(.2 if 'imu' in t else .5))
        and (end_stamp is None or end_stamp-active_stamp_last.get(t,float('-inf'))<=(.2 if 'imu' in t else .5))) for t in counts}
    checks={'critical_sensor_topics_present':all(counts[t]>=50 for t in ['/simulation/lidar3d/points','/lio/lidar','/simulation/imu3d','/lio/imu','/lio/cloud_registered']),
        'declared_window_boundary_coverage':all(coverage.values()),
        'all_payloads_time_frames_measurements_valid':not faults,
        'writer_counts_and_rows':health['pass'] and dict(counts)==health['written_counts']==health['received_counts'],
        'sensor_receive_continuity_in_declared_validation_window':all(active_count[t]>=50 and active_gap[t]<=(.2 if 'imu' in t else .5) for t in counts),
        'three_dimensional_returns':bool(zvalues) and max(zvalues)-min(zvalues)>.1,
        'specific_force_includes_gravity':bool(imu_norm) and abs(sum(imu_norm)/len(imu_norm)-9.81)<.2}
    return {'pass':all(checks.values()),'checks':checks,'counts':dict(counts),'max_receive_gaps_s':dict(max_gap),
        'faults':faults[:50],'cross_topic_arrival_orphans_before_second_pass':dict(orphans),
        'declared_validation_stamp_window':[start_stamp,end_stamp],'validation_receive_gaps_s':dict(active_gap),
        'window_boundary_coverage':coverage,'first_validation_stamps':active_first,'last_validation_stamps':active_stamp_last,
        'full_recording_continuity_pass':all(max_gap[t]<=(.2 if 'imu' in t else .5) for t in counts),
        'mean_acceleration_norm':sum(imu_norm)/len(imu_norm) if imu_norm else None,
        'z_range':[min(zvalues),max(zvalues)] if zvalues else None,'point_timing':'all rays instantaneous; no sequential time fabricated',
        'publisher_loss_absent_proven':False}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--start-stamp',type=float);p.add_argument('--end-stamp',type=float);a=p.parse_args()
    report=audit_sensor_file(a.input,a.start_stamp,a.end_stamp)
    with a.output.open('x') as f:json.dump(report,f,indent=2)
    print(json.dumps(report,indent=2));raise SystemExit(0 if report['pass'] else 2)
