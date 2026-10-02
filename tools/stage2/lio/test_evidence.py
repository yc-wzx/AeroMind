"""Mutation counterexamples on disposable copies; historical evidence is read-only."""
import sys,json,tempfile,hashlib,argparse
from pathlib import Path
from audit_navigation import audit

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def run(source,output):
    name='lio_short_01';reports=[]
    cases=['missing_raw_LIO','missing_completion_event','GT_stop_velocity_violation','nonfinite_navigation_pose']
    original_hashes={p.name:sha(p) for p in source.iterdir() if p.is_file()}
    for case in cases:
        with tempfile.TemporaryDirectory(prefix='stage2_lio_evidence_') as temporary:
            dest=Path(temporary)
            for p in source.iterdir():(dest/p.name).symlink_to(p.resolve(),target_is_directory=p.is_dir())
            target=name+('.events.jsonl' if case=='missing_completion_event' else '.native.jsonl')
            (dest/target).unlink()  # Remove only the temporary link, never the source file.
            edits=0
            with (source/target).open() as src,(dest/target).open('x') as out:
                for line in src:
                    row=json.loads(line);drop=False
                    if case=='missing_completion_event' and 'RMUC waypoint diagnostic completed' in row.get('message',''):
                        drop=True;edits+=1
                    elif case=='missing_raw_LIO' and row.get('topic')=='/lio/odometry':drop=True;edits+=1
                    elif case=='GT_stop_velocity_violation' and row.get('topic')=='/gazebo/odometry' and 27<=row['message_stamp_s']<=28:
                        row['data']['twist']['twist']['linear']['x']=.1;edits+=1
                    elif case=='nonfinite_navigation_pose' and row.get('topic')=='/ground/odometry' and 20<=row['message_stamp_s']<=21:
                        row['data']['pose']['pose']['position']['x']=float('nan');edits+=1
                    if not drop:out.write(json.dumps(row)+'\n')
            if not edits:raise RuntimeError('Counterexample did not modify target '+case)
            hashes=json.loads((source/'raw_data_sha256.json').read_text());hashes[target]=sha(dest/target)
            (dest/'raw_data_sha256.json').unlink();(dest/'raw_data_sha256.json').write_text(json.dumps(hashes))
            result=audit(dest)
            reports.append({'case':case,'edits':edits,'mutated_file':target,'mutated_sha256':hashes[target],
                'all_pass':result['all_pass'],'failed_checks':[k for k,v in result['checks'].items() if not v],
                'rehashed_raw_data_verified':result['checks']['original_raw_hashes_match']})
    unchanged=all(sha(source/p)==h for p,h in original_hashes.items())
    result={'pass':unchanged and all(not r['all_pass'] and r['rehashed_raw_data_verified'] for r in reports),
        'original_evidence_unchanged':unchanged,'cases':reports,'tool_sha256':sha(Path(__file__))}
    with output.open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2));return result['pass']

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    raise SystemExit(0 if run(a.source.resolve(),a.output) else 2)
