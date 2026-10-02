"""Empirical measurement noise vs recorded physics; no feedback or calibration claim."""
import argparse,base64,json,sys,re
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

def run(source,log,output):
    physics={}
    for l in (source/'lio_full_01.native.jsonl').open():
        r=json.loads(l)
        if r['topic']=='/simulation/imu_kinematics':
            d=json.loads(r['data']['data']);physics[round(d['stamp_s'],8)]=d
    residual=[]
    for l in (source/'lio_full_01.sensors.jsonl').open():
        r=json.loads(l)
        if r['topic']!='/simulation/imu3d':continue
        d=physics.get(round(r['message_stamp_s'],8))
        if not d:continue
        m=deserialize_message(base64.b64decode(r['cdr']),Imu)
        a=np.array([getattr(m.linear_acceleration,k) for k in 'xyz'])-np.asarray(d['body_acceleration'])-[0,0,9.81]
        g=np.array([getattr(m.angular_velocity,k) for k in 'xyz'])-np.asarray(d['body_angular_velocity'])
        residual.append([d['stamp_s'],*a,*g])
    a=np.asarray(residual);mean=a[:,1:].mean(axis=0);std=a[:,1:].std(axis=0,ddof=1)
    states=[];h={}
    pattern=re.compile(r'OFFLINE_H t=(\S+) count=(\d+) eigen=(\S+) diag=(\S+) weak=(\S+)')
    for l in log.open():
        if 'OFFLINE_STATE' in l:
            fields={k:v for k,v in re.findall(r'(\w+)=(\S+)',l.split('OFFLINE_STATE')[1])}
            states.append({k:(float(v) if ',' not in v else [float(x) for x in v.split(',')]) for k,v in fields.items()})
        m=pattern.search(l)
        if m:h[float(m.group(1))]={'sim_s':float(m.group(1)),'count':int(m.group(2)),
            'eigenvalues':[float(x) for x in m.group(3).split(',')],'diagonal':[float(x) for x in m.group(4).split(',')],'weak_eigenvector':[float(x) for x in m.group(5).split(',')]}
    norms={k:max(float(np.linalg.norm(s[k])) for s in states) for k in ['ba','bg']} if states else {}
    result={'residual_pairs':len(a),'columns':['acc_x','acc_y','acc_z','gyro_x','gyro_y','gyro_z'],
        'empirical_mean':mean.tolist(),'empirical_stddev':std.tolist(),
        'declared_stddev':[.001]*3+[.0001]*3,'declared_variance':[1e-6]*3+[1e-8]*3,
        'empirical_std_matches_declaration_within_10_percent':bool(np.all(abs(std/np.array([.001]*3+[.0001]*3)-1)<.1)),
        'original_filter_variance':{'acc':.01,'gyro':.001,'acc_bias_walk':.0001,'gyro_bias_walk':.0001},
        'filter_Q_is_covariance':'IKFoM predict adds (dt*f_w)*Q*(dt*f_w)^T; parameters go directly to Q diagonal',
        'bias_walk_in_simulator':'not configured; Gaussian white measurement noise with zero bias defaults',
        'LIO_estimated_max_bias_norm':norms,'states':states,'measurement_translation_gram':list(h.values()),
        'limitations':['Known physics diagnostic needed for offline noise audit only, never replayed into LIO.','This is synthetic sensor noise verification, not hardware calibration.','Translation Gram does not replace the full coupled filter information matrix.']}
    with output.open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:result[k] for k in ['residual_pairs','empirical_stddev','empirical_std_matches_declaration_within_10_percent','LIO_estimated_max_bias_norm']},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--diagnostic-log',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.source,a.diagnostic_log,a.output)
