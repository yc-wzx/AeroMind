#!/usr/bin/env python3
"""Offline spline/Marker/guard association on existing evidence, no ROS output."""
import argparse
import hashlib
import json
import math
import copy
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import sys
import yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/uav_planning/scripts'))
from audit_localized import read, load_jsonl, sha, yaw
from ego_trajectory_executor import Spline
from gazebo_navigation_interface import GazeboNavigationInterface
from provincial_safety_geometry import wall_box, rectangle, polygon_distance
from planar_navigation_simulator import wrap_angle
from analyze_provincial_forward_clearance import walls_from_sdf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from rosidl_runtime_py.set_message import set_message_fields


def review(directory):
    name=read(directory/'progress.json')['name'];rows=load_jsonl(directory/(name+'.native.jsonl'))
    snapshot=directory/'input_snapshot'
    modules=['src/uav_planning/scripts/ego_trajectory_executor.py',
             'src/uav_planning/scripts/gazebo_navigation_interface.py',
             'src/uav_planning/scripts/provincial_safety_geometry.py']
    exact={p:sha(ROOT/p)==sha(snapshot/p) for p in modules}
    if not all(exact.values()):raise ValueError('Context replay code differs from runtime snapshot')
    params=next(iter(yaml.safe_load((directory/'runtime_parameters/ego_trajectory_executor.yaml').read_text()).values()))['ros__parameters']
    step=params['planned_path_sample_sec']
    bsplines=[r for r in rows if r['topic']=='/planning/bspline']
    generated=[]
    for r in bsplines:
        d=r['data'];curve=Spline([(p['x'],p['y']) for p in d['pos_pts']],int(d['order']),d['knots'])
        duration=curve.end-curve.start;count=max(2,math.ceil(duration/step)+1)
        points=[curve.evaluate(curve.start+min(duration,i*step)) for i in range(count)]
        generated.append((r,points,duration))
    field=read(snapshot/'src/uav_planning/config/provincial_2025_provisional.json')
    walls=walls_from_sdf(snapshot/'src/uav_bringup/worlds/provincial_2025_training.sdf')
    latest={};results=[]
    for row in rows:
        latest[row['topic']]=row
        if row['topic']!='/ground/planning/planned_trajectory':continue
        points=row['data']['points'];matches=[]
        for raw,calculated,duration in generated:
            if len(points)!=len(calculated):continue
            residual=max(math.dist([p['x'],p['y']],v) for p,v in zip(points,calculated))
            if residual<1e-9:
                matches.append(dict(traj_id=raw['data']['traj_id'],start_time=raw['data']['start_time'],
                   duration_s=duration,maximum_sample_residual_m=residual,
                   receive_offset_s=(row['receive_monotonic_ns']-raw['receive_monotonic_ns'])*1e-9))
        odom=latest.get('/ground/odometry');active=latest.get('/ground/planning/goal')
        if odom is None or active is None:raise ValueError('Raw plan context absent')
        position=odom['data']['pose']['pose'];angle=yaw(position['orientation'])
        waypoint=PoseStamped();set_message_fields(waypoint,copy.deepcopy(active['data']))
        marker=Marker();set_message_fields(marker,copy.deepcopy(row['data']))
        n=GazeboNavigationInterface.__new__(GazeboNavigationInterface)
        n.provincial_rect_guard=True;n.active_waypoint=waypoint;n.yaw=angle
        n.provincial_walls=[wall_box(s,.055) for s in field['collision_segments']]
        n.provincial_min_body_gap=.08;n.planned_path_min_gap=n.planned_path_safe=None
        logger=NS(warn=lambda text:None)
        clock=NS(now=lambda:NS(nanoseconds=round(row['message_stamp_s']*1e9)))
        with patch.object(GazeboNavigationInterface,'get_logger',lambda _:logger),patch.object(GazeboNavigationInterface,'get_clock',lambda _:clock):
            n.planned_trajectory_callback(marker)
        # Reproduce the same .30rad/s, .05s indexed yaw rule, separately identify wall.
        target_yaw=yaw(active['data']['pose']['orientation']);delta=wrap_angle(target_yaw-angle)
        samples=[]
        for i,p in enumerate(points):
            future_yaw=angle+max(-.30*i*.05,min(.30*i*.05,delta))
            polygon=rectangle(p['x'],p['y'],future_yaw,.52,.42)
            gap,wall=min((polygon_distance(polygon,w['polygon']),w['name']) for w in walls)
            samples.append(dict(index=i,x=p['x'],y=p['y'],yaw_rad=future_yaw,gap_m=gap,wall=wall))
        following=[r for r in rows if r['topic']=='/ground/planning/actuation_diagnostics' and
             row['receive_monotonic_ns']<=r['receive_monotonic_ns']<=row['receive_monotonic_ns']+200_000_000]
        results.append(dict(marker_stamp_s=row['message_stamp_s'],marker_id=row['data']['id'],
             unique_shape_match=len(matches)==1,matching_raw_splines=matches,
             context_odom_stamp_s=odom['message_stamp_s'],context_yaw_rad=angle,
             context_receive_age_s=(row['receive_monotonic_ns']-odom['receive_monotonic_ns'])*1e-9,
             context_goal_header_stamp=active['data']['header']['stamp'],
             production_callback_min_gap_m=n.planned_path_min_gap,production_callback_safe=n.planned_path_safe,
             nearest_sdf_wall_sample=min(samples,key=lambda p:p['gap_m']),
             following_raw_guard=[dict(receive_sim_s=r['receive_sim_s'],**json.loads(r['data']['data'])) for r in following]))
    return dict(plans=results,code_matches_runtime_snapshot=exact,plan_step_s=step,
       all_marker_shapes_match_unique_raw_spline=all(r['unique_shape_match'] for r in results),
       input_data_unchanged=True,source_sha256=sha(Path(__file__)),
       limits=['Marker ID is always 0; spline traj_id is associated by reproducing every sample, not by Marker ID.',
          'Latest prior-received ground odom/yaw is context, not a recording of navigation callback internal state.',
          'Goal header comes from adapted /ground/planning/goal, not separately recorded /navigation/segment_goal.',
          'Production future-yaw model replay with recorded context is not a proof of exact original callback ordering.',
          'No live run, interpolation or new GT; unsafe plan is not actual robot contact.'])


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    r=review(a.result_dir);a.output.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(r,ensure_ascii=False))
