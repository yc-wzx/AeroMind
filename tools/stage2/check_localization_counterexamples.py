#!/usr/bin/env python3
"""Corrupt temporary evidence copies, never historical originals."""
import copy
import json
from pathlib import Path
import tempfile
from audit_localized_replay import verify, ROOT, sha, read
from audit_localized import load_jsonl

BASE=ROOT/'tools/results/stage2_localization_20261001'
SOURCE=BASE/'zero'
NAME='stage2_localized_zero_01'


def run():
    rows=load_jsonl(SOURCE/(NAME+'.native.jsonl'))
    start=read(SOURCE/(NAME+'.summary.json'))['goal_delivery']['publish_monotonic_ns']
    def find(topic):
        return next(i for i,r in enumerate(rows) if r['topic']==topic and r['receive_monotonic_ns']>=start+2_000_000_000)
    tests={};tests['original']=verify(SOURCE)['all_pass']
    for case in ('missing_scan','changed_scan','changed_correction','changed_corrected_pose','missing_corrected_sample'):
        with tempfile.TemporaryDirectory(prefix='stage2_localization_counterexample_') as temp:
            directory=Path(temp)
            for p in SOURCE.iterdir():
                if p.name not in (NAME+'.native.jsonl','raw_data_sha256.json'):
                    (directory/p.name).symlink_to(p,target_is_directory=p.is_dir())
            altered=list(rows)
            if case in ('missing_scan','changed_scan'):
                i=find('/scan')
                if case=='missing_scan':del altered[i]
                else:
                    altered[i]=copy.deepcopy(rows[i])
                    altered[i]['data']['ranges']=[v+1. if isinstance(v,(int,float)) else v for v in altered[i]['data']['ranges']]
            elif case=='changed_correction':
                i=find('/localization/diagnostics');altered[i]=copy.deepcopy(rows[i])
                d=json.loads(altered[i]['data']['data']);d['delta'][0]+=.01
                altered[i]['data']['data']=json.dumps(d)
            elif case=='changed_corrected_pose':
                i=find('/localization/navigation_odometry');altered[i]=copy.deepcopy(rows[i])
                altered[i]['data']['pose']['pose']['position']['x']+=.1
            else:del altered[find('/localization/navigation_odometry')]
            with (directory/(NAME+'.native.jsonl')).open('w') as f:
                for row in altered:f.write(json.dumps(row,separators=(',',':'))+'\n')
            hashes=read(SOURCE/'raw_data_sha256.json');hashes[NAME+'.native.jsonl']=sha(directory/(NAME+'.native.jsonl'))
            (directory/'raw_data_sha256.json').write_text(json.dumps(hashes))
            result=verify(directory)
            tests[case]={'expected_pass':False,'actual_pass':result['all_pass'],
                         'failed_checks':[k for k,v in result['checks'].items() if not v],
                         'localization_checks':result['localization']['checks'],
                         'replay_checks':result['localization_replay']['checks']}
            print(case,result['all_pass'],flush=True)
    final={'all_expected':tests['original'] and all(not v['actual_pass'] for k,v in tests.items() if k!='original'),
           'cases':tests,'original_native_sha256':sha(SOURCE/(NAME+'.native.jsonl')),
           'scope':'Data-hash metadata is updated in temporary copies; rejection must come from evidence, not only digest mismatch.'}
    target=BASE/'localization_counterexamples_v1.json'
    if target.exists():raise FileExistsError(target)
    target.write_text(json.dumps(final,indent=2)+'\n')
    return final


if __name__=='__main__':
    result=run();print(json.dumps({'all_expected':result['all_expected']}))
    raise SystemExit(0 if result['all_expected'] else 2)
