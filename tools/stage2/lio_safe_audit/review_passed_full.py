"""All native GT samples and localization diagnostics; no ROS transport."""
import collections
import json
import math
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tools'))
from analyze_provincial_forward_clearance import walls_from_sdf
from provincial_safety_geometry import rectangle, polygon_distance
OUT = ROOT/'tools/results/stage2_step4_lio_safe_profile_20261002'
RUN = OUT/'full_reserved_map_01'


def pose(row):
    p = row['data']['pose']['pose']; q = p['orientation']
    return [row['message_stamp_s'], p['position']['x'], p['position']['y'],
        math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))]


def main():
    topics = collections.defaultdict(list)
    with (RUN/'lio_full_01.native.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            if row['topic'] in ['/gazebo/odometry','/localization/lio_navigation_odometry',
                                '/lio/safety_diagnostics','/goal_pose','/model/omni_robot/cmd_vel']:
                topics[row['topic']].append(row)
    gt = np.array([pose(r) for r in topics['/gazebo/odometry']])
    nav = np.array([pose(r) for r in topics['/localization/lio_navigation_odometry']])
    begin = topics['/goal_pose'][0]['receive_sim_s']
    active = gt[gt[:,0] >= begin]
    walls = walls_from_sdf(RUN/'input_snapshot/src/uav_bringup/worlds/provincial_stage2_lio_position_imu.sdf')
    gaps, names = [], []
    for t, x, y, angle in active:
        gap, name = min((polygon_distance(rectangle(x,y,angle,.52,.42), w['polygon']), w['name']) for w in walls)
        gaps.append(gap); names.append(name)
    i = int(np.argmin(gaps)); point = active[i]
    errors = []
    for p in nav[nav[:,0] >= begin]:
        at = np.searchsorted(gt[:,0], p[0])
        candidates = [j for j in [at-1,at] if 0 <= j < len(gt)]
        j = min(candidates,key=lambda j:abs(gt[j,0]-p[0]))
        if abs(gt[j,0]-p[0]) <= .025:
            errors.append([p[0], math.dist(p[1:3],gt[j,1:3]), p[1]-gt[j,1], p[2]-gt[j,2]])
    error = np.array(errors)
    ds = [json.loads(r['data']['data']) for r in topics['/lio/safety_diagnostics']]
    diag = np.array([[d['sim_s'],d['sampled_reserved_map_gap_m'],d['required_map_gap_m']]
                    for d in ds if d.get('sampled_reserved_map_gap_m') is not None and d['sim_s'] >= begin])
    stationary_gt = gt[(gt[:,0] >= 10) & (gt[:,0] <= 15)]
    stationary_nav = nav[(nav[:,0] >= 10) & (nav[:,0] <= 15)]
    stationary_commands = [r for r in topics['/model/omni_robot/cmd_vel'] if 10 <= r['receive_sim_s'] <= 15]
    maximum_command = max(abs(r['data'][kind][axis]) for r in stationary_commands
                           for kind, axis in [('linear','x'),('linear','y'),('angular','z')])
    steady_error = max(math.dist(p[1:3],[4.7,.5]) for p in stationary_nav)
    dt = np.diff(active[:,0])
    receive = np.diff([r['receive_monotonic_ns']*1e-9 for r in topics['/gazebo/odometry'] if r['message_stamp_s'] >= begin])
    result = {'minimum': {'sim_s':float(point[0]),'x':float(point[1]),'y':float(point[2]),
              'yaw_rad':float(point[3]),'gap_m':float(gaps[i]),'nearest_wall':names[i]},
        'raw_GT_active_samples':len(active),'raw_GT_header_frequency_hz':float(1/np.mean(dt)),
        'raw_GT_max_header_interval_s':float(max(dt)), 'raw_GT_max_monotonic_receive_interval_s':float(max(receive)),
        'below_0_08m_sample_count':sum(g < .08 for g in gaps),
        'LIO_raw_nearest_GT_max_xy_m':float(max(error[:,1])),
        'stationary_10_to_15':{'pass':bool(begin > 15 and len(stationary_gt)>200 and len(stationary_nav)>1000 and
             maximum_command == 0 and steady_error < .02 and np.max(abs(stationary_gt[:,1:3]-[4.7,.5])) < 1e-6),
             'GT_samples':len(stationary_gt),'LIO_samples':len(stationary_nav),
             'maximum_final_command':maximum_command,'max_LIO_xy_error_m':steady_error},
        'plot_note':'Lines connect recorded samples for display only; no interpolated GT or continuous-time lower bound.',
        'contact':'CONTACT DATA UNAVAILABLE'}
    with (OUT/'full_native_geometry_review_v2.json').open('x') as f: json.dump(result,f,indent=2)
    fig, axes = plt.subplots(2,2,figsize=(12,9),layout='constrained')
    for ax in axes[0]:
        for w in walls: ax.add_patch(Polygon(w['polygon'], color='grey', alpha=.4))
        ax.plot([4.7,4.7,8.7,8.7],[.5,1.8,1.8,4.25],'k--',label='reference')
        ax.plot(active[:,1],active[:,2],label='GT raw50Hz')
        ax.plot(nav[:,1],nav[:,2],alpha=.8,label='LIO')
        ax.add_patch(Polygon(rectangle(*point[1:],.52,.42),fill=False,color='red'))
        ax.scatter([point[1]],[point[2]],c='red');ax.set_aspect('equal');ax.legend(fontsize=8)
    axes[0,0].set_xlim(4.2,9.2); axes[0,0].set_ylim(.2,4.7)
    axes[0,0].set_title('Simulated 3D lidar + IMU + LIO: Full PASS')
    axes[0,1].set_xlim(point[1]-.5,point[1]+.5);axes[0,1].set_ylim(point[2]-.45,point[2]+.6)
    axes[0,1].set_title(f'{names[i]}: sampled min {gaps[i]:.6f} m')
    axes[1,0].plot(active[:,0],gaps,label='GT sampled body gap')
    axes[1,0].plot(diag[:,0],diag[:,1],alpha=.6,label='LIO command forecast')
    axes[1,0].plot(diag[:,0],diag[:,2],alpha=.7,label='reserved map decision threshold')
    axes[1,0].axhline(.08,color='red',ls='--',label='physical acceptance .08m')
    axes[1,0].set_ylabel('m');axes[1,0].set_xlabel('sim s');axes[1,0].legend(fontsize=8);axes[1,0].grid()
    axes[1,1].plot(error[:,0],error[:,1]*100,label='xy norm')
    axes[1,1].plot(error[:,0],error[:,2]*100,alpha=.6,label='x residual')
    axes[1,1].plot(error[:,0],error[:,3]*100,alpha=.6,label='y residual')
    axes[1,1].set_ylabel('LIO-GT cm (nearest raw stamps)');axes[1,1].set_xlabel('sim s')
    axes[1,1].legend(fontsize=8);axes[1,1].grid()
    fig.savefig(OUT/'stage4_full_verified.png',dpi=140);plt.close(fig)
    print(json.dumps(result,indent=2))


if __name__ == '__main__': main()
