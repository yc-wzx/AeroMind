"""Same recorded sensors, offline pose comparison. Never a navigation input."""
import argparse
import json
import hashlib
import math
import re
import sys
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(1024*1024), b''): h.update(b)
    return h.hexdigest()


def load(path): return json.loads(path.read_text())


def aligned(row, raw_lio):
    p = row['data']['pose']['pose']
    position = np.array([p['position'][k] for k in 'xyz'])
    rotation = Rotation.from_quat([p['orientation'][k] for k in 'xyzw'])
    if raw_lio:
        fixed = Rotation.from_euler('z', math.pi/2)
        position = fixed.apply(position - rotation.apply([0,0,.25])) + [4.7,.5,.4]
        rotation = fixed*rotation
    yaw, pitch, roll = rotation.as_euler('ZYX')
    return [row['message_stamp_s'], *position, yaw, pitch, roll]


def errors(poses, gt):
    pairs = []
    for p in poses:
        at = np.searchsorted(gt[:,0], p[0])
        js = [j for j in [at-1,at] if 0 <= j < len(gt)]
        if not js: continue
        j = min(js,key=lambda j:abs(gt[j,0]-p[0]))
        if abs(gt[j,0]-p[0]) > .025: continue
        pairs.append([p[0], math.dist(p[1:3],gt[j,1:3]), p[1]-gt[j,1],p[2]-gt[j,2],
                      p[3]-.15,p[5],p[6]])
    return np.asarray(pairs)


def statistics(pairs, begin, end):
    active = pairs[(pairs[:,0] >= begin) & (pairs[:,0] <= end)]
    assert len(active) > 100
    return {'pairs':len(active), 'max_xy_error_m':float(max(active[:,1])),
            'RMS_xy_m':float(np.sqrt(np.mean(active[:,1]**2))),
            'last':active[-1].tolist(),
            'max_flat_height_model_deviation_m':float(max(abs(active[:,4]))),
            'max_pitch_deg':float(np.rad2deg(max(abs(active[:,5])))),
            'max_roll_deg':float(np.rad2deg(max(abs(active[:,6]))))}


def analyze(source, replay):
    gt, original = [], []
    begin = None
    for line in (source/'lio_full_01.native.jsonl').open():
        row = json.loads(line)
        if row['topic'] == '/gazebo/odometry': gt.append(aligned(row,False))
        if row['topic'] == '/localization/lio_navigation_odometry': original.append(aligned(row,False))
        if row['topic'] == '/goal_pose': begin = row['receive_sim_s']
    gt = np.array(gt); end = gt[-1,0]
    monotonic, skipped, last = [], 0, -float('inf')
    for line in (replay/'output.native.jsonl').open():
        row = json.loads(line)
        if row['topic'] != '/offline_lio/odometry': continue
        t = row['message_stamp_s']
        if t <= last: skipped += 1; continue
        monotonic.append(aligned(row,True)); last=t
    corrected, states = [], []
    for line in (replay/'lio.log').open():
        if 'OFFLINE_STATE' not in line: continue
        fields = {k:v for k,v in re.findall(r'(\w+)=(\S+)',line.split('OFFLINE_STATE')[1])}
        state = {k:([float(x) for x in v.split(',')] if ',' in v else float(v)) for k,v in fields.items()}
        states.append(state)
    source_counts = Counter()
    for line in (source/'lio_full_01.sensors.jsonl').open():
        r=json.loads(line)
        source_counts[r['topic']] += 1
    progress=load(replay/'progress.json');inputs=load(replay/'input_manifest.json')
    corrected_task_frames=0
    for line in (replay/'output.clouds.jsonl').open():
        row=json.loads(line)
        if row['topic']=='/offline_lio/cloud_registered' and begin <= row['message_stamp_s'] <= end:
            corrected_task_frames+=1
    delivery=Counter(json.loads(l)['kind'] for l in (replay/'input_delivery.jsonl').open())
    checks={'replay_finished':progress['finished'],
        'source_content_unchanged':progress['input_content_unchanged'],
        'every_forwarded_imu_sent':delivery['imu']==source_counts['/lio/imu']==progress['sent']['imu'],
        'every_forwarded_cloud_sent':delivery['cloud']==source_counts['/lio/lidar']==progress['sent']['cloud'],
        'isolated_DDS_no_GT_goal_velocity_inputs':inputs['DDS_domain'] in [73,74] and inputs['GT_and_navigation_input_replayed'] is False,
        'input_messages_unchanged':inputs['sensor_content_changed'] is False,
        'writer_health':load(replay/'output.native.health.json')['pass'] and load(replay/'output.clouds.health.json')['pass'],
        'raw_hashes_match':all(sha(replay/p)==h for p,h in load(replay/'data_sha256.json').items()),
        'localization_covers_task_end':last >= end-.11,
        'corrections_cover_task':corrected_task_frames >= int((end-begin)*9)}
    actual_variance = [float(m.group(1)) for line in (replay/'lio.log').open()
                      if (m:=re.search(r'POINT_VARIANCE_ACTUAL (\S+)',line))]
    requested=inputs['parameter_overrides'].get('mapping.laser_point_covariance')
    if requested is not None: checks['actual_variance_parameter'] = actual_variance == [requested]
    checks = {k: bool(v) for k,v in checks.items()}
    return {'checks':checks,'evidence_pass':all(checks.values()),
        'evaluator_sha256':sha(Path(__file__)),'command':sys.argv,
        'original_recording':statistics(errors(np.array(original),gt),begin,end),
        'replayed':statistics(errors(np.array(monotonic),gt),begin,end),
        'task_sim_bounds':[float(begin),float(end)], 'older_correction_messages_retained_but_not_selected':skipped,
        'source_sensor_counts':dict(source_counts),'replay_delivery_counts':dict(delivery),
        'corrected_state_count':len(states),'corrected_task_cloud_frames':corrected_task_frames,
        'internal_state_diagnostics_available':bool(states),
        'max_estimated_bias_norm':({k:max(float(np.linalg.norm(s[k])) for s in states) for k in ['ba','bg']} if states else None),
        'actual_variance_parameter':actual_variance,'overrides':inputs['parameter_overrides'],
        'limitations':['Same recorded data tests estimator only, not closed-loop navigation.',
            'Replay changes callback timing; it is not bit-exact reproduction.',
            'GT used only after replay to compute errors; no GT LIO input.',
            'Model-relative height is not independently recorded 3D truth.',
            'Sent/recorded counts cannot prove publishers lost no messages.']}


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True)
    p.add_argument('--replay',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=analyze(a.source,a.replay)
    serialized=json.dumps(r,indent=2,allow_nan=False)
    with a.output.open('x') as f:f.write(serialized)
    snapshot=a.output.with_suffix('.tool.py')
    with snapshot.open('xb') as f:f.write(Path(__file__).read_bytes())
    print(serialized);raise SystemExit(0 if r['evidence_pass'] else 2)
