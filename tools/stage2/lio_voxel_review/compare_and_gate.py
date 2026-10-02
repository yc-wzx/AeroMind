"""The predeclared two-recording acceptance gate, not navigation acceptance."""
import argparse
import json
import math
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/stage2/lio_optimization'))
from analyze_variance import sha


def metric_checks(baseline,candidate):
    try:
        b=baseline['replayed'];c=candidate['replayed']
        numbers=[b['max_xy_error_m'],c['max_xy_error_m'],b['RMS_xy_m'],c['RMS_xy_m'],b['last'][1],c['last'][1]]
        finite=all(isinstance(v,(float,int)) and math.isfinite(v) and v>=0 for v in numbers)
        return {'evidence_complete':baseline.get('evidence_pass') is True and candidate.get('evidence_pass') is True,
            'metrics_finite':finite,
            'max_error_improves_at_least_10_percent':finite and c['max_xy_error_m']<=.9*b['max_xy_error_m'],
            'RMS_not_worse':finite and c['RMS_xy_m']<=b['RMS_xy_m'],
            'terminal_error_not_worse':finite and c['last'][1]<=b['last'][1]}
    except (KeyError,TypeError,IndexError): return {'metrics_complete':False}


def evaluate(out):
    out=out.resolve()
    reports={};checks={}
    for tag in ['old','new']:
        baseline=out/(tag+'_full_voxel_010');candidate=out/(tag+'_full_voxel_005')
        bfile=out/(tag+'_full_voxel_010_analysis_v2.json' if tag=='old' else tag+'_full_voxel_010_analysis.json')
        cfile=out/(tag+'_full_voxel_005_analysis.json')
        b=json.loads(bfile.read_text());c=json.loads(cfile.read_text())
        local=metric_checks(b,c)
        manifests=[json.loads((d/'input_manifest.json').read_text()) for d in [baseline,candidate]]
        loaded=[json.loads((d/'actually_loaded_LIO_component.json').read_text()) for d in [baseline,candidate]]
        local['identical_recorded_input_bytes']=manifests[0]['raw_input_sha256']==manifests[1]['raw_input_sha256']
        local['identical_compiled_LIO_component']=loaded[0]['sha256']==loaded[1]['sha256']
        pointfiles=[out/(d.name+'_points.json') for d in [baseline,candidate]]
        points=[json.loads(p.read_text()) for p in pointfiles]
        local['all_recorded_point_sets_consistent']=all(p['all_points_rigid_consistent'] and p['all_recorded_outputs_checked'] and not p['unpaired_recorded_output_stamps'] and p['frames']>500 for p in points)
        local['current_pose_evaluator']=all(r['voxel_evaluator_sha256']==sha(Path(__file__).with_name('analyze_voxel.py')) for r in [b,c])
        checks[tag]=local
        reports[tag]={'baseline':b['replayed'],'candidate':c['replayed'],
            'report_sha256':{str(p.relative_to(ROOT)):sha(p) for p in [bfile,cfile,*pointfiles]},
            'point_frame_counts':[p['frames'] for p in points]}
    return {'offline_candidate_accepted':all(all(c.values()) for c in checks.values()),'checks':checks,
        'comparisons':reports,'predeclared_plan_sha256':sha(out/'experiment_plan.json'),
        'evaluator_sha256':sha(Path(__file__)),'command':sys.argv,
        'not_closed_loop_navigation_acceptance':True,'statistical_repeatability_proven':False}


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    r=evaluate(a.result_dir)
    with a.output.open('x') as f:json.dump(r,f,indent=2,allow_nan=False)
    print(json.dumps(r,indent=2));raise SystemExit(0 if r['offline_candidate_accepted'] else 2)
