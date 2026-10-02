#!/usr/bin/env python3
"""Same v4 evidence gates; replace inherited legacy provenance with actual hashes."""
import argparse
import json
from pathlib import Path
import sys
from audit_scan_resume_startup_v4 import audit as audit_v4
from audit_localized import ROOT,read,sha


def audit(directory):
    result=audit_v4(directory)
    manifest=read(directory/'input_manifest.json')
    modules=['tools/stage2/audit_scan_resume_startup_v4.py','tools/stage2/audit_scan_resume_v3.py',
        'tools/stage2/audit_scan_resume_v2.py','tools/stage2/audit_scan_resume.py']
    hashes={m:dict(runtime_sha256=manifest['inputs'][m]['sha256'],
        snapshot_sha256=sha(directory/'input_snapshot'/m),current_sha256=sha(ROOT/m)) for m in modules}
    result['checks']['runtime_evaluator_content_attested']=all(v['runtime_sha256']==v['snapshot_sha256']==v['current_sha256'] for v in hashes.values())
    result['all_pass']=all(result['checks'].values());result['strict_trial_verified']=result['all_pass']
    result['live_scan_loss_guard_verified']=result['all_pass']
    result['current_evaluator_not_runtime_input']=dict(runtime_v2_v3_v4_dependencies=hashes,
        v5='Added after trial only to correct inherited legacy provenance text; recorded runtime v4/input/raw files retained.')
    result['evaluator_dependency_sha256']['tools/stage2/audit_scan_resume_startup_v4.py']=sha(ROOT/modules[0])
    result['evaluator_sha256']=sha(Path(__file__));result['command']=sys.argv
    result['correction_note_v5']='Actual hashes show v2/v3/v4 were captured runtime dependencies in this trial. No legacy runtime-v1 claim is applied; verdict gates and thresholds unchanged.'
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    try:r=audit(a.result_dir)
    except (OSError,ValueError,KeyError,TypeError,IndexError) as error:r=dict(all_pass=False,status='INCOMPLETE_EVIDENCE',error=repr(error))
    a.output.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'all_pass':r['all_pass'],'checks':r.get('checks'),'error':r.get('error')},ensure_ascii=False))
    raise SystemExit(0 if r['all_pass'] else 2)
