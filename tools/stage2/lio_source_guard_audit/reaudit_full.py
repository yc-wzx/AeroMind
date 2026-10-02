#!/usr/bin/env python3
"""Re-execute original strict task audit with corrected independent source algebra."""
import argparse,hashlib,importlib.util,json,sys
from pathlib import Path
from audit_input_history import audit_rows
ROOT=Path(__file__).resolve().parents[3]
path=ROOT/'tools/stage2/lio_model_validation_v2/audit_model_navigation.py'
spec=importlib.util.spec_from_file_location('preserved_v2',path);original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)

def evaluate(directory):
    result=original.evaluate(directory)
    history=audit_rows([json.loads(l) for l in (directory/'lio_full_01.native.jsonl').open()])
    before=result['LIO_source'];physical=dict(before['checks'])
    physical.pop('actual_output_causal_velocity_recomputed')
    checks=dict(physical,actual_input_history_causal_velocity_recomputed=history['pass'])
    result['LIO_source_original_output_only']=before
    result['LIO_source']={'pass':all(checks.values()),'checks':checks,'input_history':history,
        'change':'Rebuild independent velocity from all recorded callback input poses, including unpublished initialization history; original thresholds retained.'}
    result['checks']['actual_LIO_and_physical_IMU_source_recomputed']=result['LIO_source']['pass']
    result['all_pass']=all(result['checks'].values())
    result['corrected_source_audit_sha256']=hashlib.sha256(Path(__file__).with_name('audit_input_history.py').read_bytes()).hexdigest()
    result['reaudit_entry_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest();result['reaudit_command']=sys.argv
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    try:r=evaluate(a.result_dir)
    except Exception as e:r={'all_pass':False,'status':'INCOMPLETE_EVIDENCE','error':repr(e)}
    with a.output.open('x') as f:json.dump(r,f,indent=2)
    print(json.dumps({k:r.get(k) for k in ['all_pass','checks','error']},indent=2));raise SystemExit(0 if r['all_pass'] else 2)
