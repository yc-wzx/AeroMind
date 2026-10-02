"""Point-set invariant on actual LIO outputs; no GT or point-order assumption."""
import argparse,base64,json
from collections import defaultdict
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools/stage2/lio_replay'))
from analyze_replay import xyz

def run(source,replay,out):
    inputs={};outputs={};candidates=defaultdict(list)
    for l in (source/'lio_full_01.sensors.jsonl').open():
        r=json.loads(l);t=round(r['message_stamp_s'],8)
        if r['topic']=='/lio/lidar':
            inputs[t]=xyz(deserialize_message(base64.b64decode(r['cdr']),PointCloud2))
    sensorfile=(replay/'output.clouds.jsonl') if replay else source/'lio_full_01.sensors.jsonl'
    odomfile=(replay/'output.native.jsonl') if replay else source/'lio_full_01.native.jsonl'
    for l in sensorfile.open():
        r=json.loads(l);t=round(r['message_stamp_s'],8)
        if r['topic'] in ['/lio/cloud_registered','/offline_lio/cloud_registered']:
            outputs[t]=xyz(deserialize_message(base64.b64decode(r['cdr']),PointCloud2))
    for l in odomfile.open():
        r=json.loads(l)
        if r['topic'] in ['/lio/odometry','/offline_lio/odometry']:
            t=round(r['message_stamp_s'],8)
            if t in inputs:candidates[t].append(r)
    checks=[]
    for t,points in inputs.items():
        if t not in outputs or not candidates[t]:continue
        tree=cKDTree(outputs[t]);possibilities=[]
        for r in candidates[t]:
            p=r['data']['pose']['pose'];rot=Rotation.from_quat([p['orientation'][k] for k in 'xyzw'])
            expected=rot.apply(points)+[p['position'][k] for k in 'xyz'];d,_=tree.query(expected)
            possibilities.append((float(np.median(d)),float(np.max(d)),int(np.sum(d>2e-5))))
        median,maximum,count=min(possibilities,key=lambda c:c[0])
        checks.append({'stamp_s':t,'median_residual_m':median,'maximum_residual_m':maximum,'points_exceeding_2e5_m':count,'point_count':len(points),'output_point_count':len(outputs[t])})
    result={'source':str(source),'replay':str(replay) if replay else 'ORIGINAL','diagnostic_threshold_m':2e-5,
        'frames':len(checks),'raw_input_frame_count':len(inputs),'recorded_output_frame_count':len(outputs),'raw_inputs_without_recorded_correction':sorted(set(inputs)-set(outputs)),
        'unpaired_recorded_output_stamps':sorted(set(outputs)-{c['stamp_s'] for c in checks}),
        'max_points_exceeding_threshold':max((c['points_exceeding_2e5_m'] for c in checks),default=None),
        'all_points_rigid_consistent':bool(checks) and all(c['points_exceeding_2e5_m']==0 and c['point_count']==c['output_point_count'] for c in checks),
        'frames_with_exactly_one_inconsistent_point':sum(c['points_exceeding_2e5_m']==1 for c in checks),
        'checks':checks,'GT_used':False,'all_recorded_outputs_checked':len(checks)==len(outputs),'scope':'Every recorded output paired with actual raw instantaneous input; input frames without correction listed, not fabricated. No GT. Not navigation acceptance'}
    serialized=json.dumps(result,indent=2,allow_nan=False)
    with out.open('x') as f:f.write(serialized)
    print(json.dumps({k:result[k] for k in ['frames','max_points_exceeding_threshold','all_points_rigid_consistent','frames_with_exactly_one_inconsistent_point']},indent=2))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--replay',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.source,a.replay,a.output)
