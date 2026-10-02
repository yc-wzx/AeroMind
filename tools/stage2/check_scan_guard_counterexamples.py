#!/usr/bin/env python3
"""New contract counterexamples with updated temporary data digests."""
import copy
import json
from pathlib import Path
import tempfile
from audit_scan_guard import verify_guard, ROOT, read, sha, load_jsonl

SOURCE=ROOT/'tools/results/stage2_localization_20261001/zero'
OUT=ROOT/'tools/results/stage2_scan_guard_20261001'
NAME='stage2_localized_zero_01'


def main():
    original=load_jsonl(SOURCE/(NAME+'.native.jsonl'))
    start=read(SOURCE/(NAME+'.summary.json'))['goal_delivery']['publish_monotonic_ns']
    def find(topic):
        return next(i for i,r in enumerate(original) if r['topic']==topic and r['receive_monotonic_ns']>=start+2_000_000_000)
    checks={};checks['original']=verify_guard(SOURCE)['all_pass']
    for case in ('unknown_frame','inconsistent_angle_max','nan_angle_increment','nan_range_min','missing_scan','changed_correction'):
        with tempfile.TemporaryDirectory(prefix='stage2_scan_guard_evidence_') as temp:
            directory=Path(temp)
            for p in SOURCE.iterdir():
                if p.name not in (NAME+'.native.jsonl','raw_data_sha256.json'):
                    (directory/p.name).symlink_to(p,target_is_directory=p.is_dir())
            rows=list(original)
            i=find('/localization/diagnostics' if case=='changed_correction' else '/scan')
            rows[i]=copy.deepcopy(original[i])
            if case=='unknown_frame':rows[i]['data']['header']['frame_id']='camera'
            elif case=='inconsistent_angle_max':rows[i]['data']['angle_max']=1.
            elif case=='nan_angle_increment':rows[i]['data']['angle_increment']=float('nan')
            elif case=='nan_range_min':rows[i]['data']['range_min']=float('nan')
            elif case=='missing_scan':del rows[i]
            else:
                d=json.loads(rows[i]['data']['data']);d['delta'][0]+=.01;rows[i]['data']['data']=json.dumps(d)
            with (directory/(NAME+'.native.jsonl')).open('w') as stream:
                for row in rows:stream.write(json.dumps(row,separators=(',',':'))+'\n')
            hashes=read(SOURCE/'raw_data_sha256.json');hashes[NAME+'.native.jsonl']=sha(directory/(NAME+'.native.jsonl'))
            (directory/'raw_data_sha256.json').write_text(json.dumps(hashes))
            result=verify_guard(directory)
            checks[case]={'all_pass':result['all_pass'],'failed_checks':[k for k,v in result['checks'].items() if not v],
                          'contract_pass':result['static_scan_contract']['pass']}
            print(case,result['all_pass'],checks[case]['failed_checks'],flush=True)
    result={'all_expected':checks['original'] and all(not v['all_pass'] for k,v in checks.items() if k!='original'),
            'cases':checks,'original_native_sha256':sha(SOURCE/(NAME+'.native.jsonl')),
            'copies_only':True,'mutated_native_hash_updated':True}
    target=OUT/'scan_guard_evidence_counterexamples_v1.json'
    if target.exists():raise FileExistsError(target)
    target.write_text(json.dumps(result,indent=2)+'\n')
    raise SystemExit(0 if result['all_expected'] else 2)


if __name__=='__main__':main()
