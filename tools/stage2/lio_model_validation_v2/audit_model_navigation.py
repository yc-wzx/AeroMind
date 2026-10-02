"""Original strict navigation audit plus simulator noise-model parameter gate."""
import sys,json,argparse,yaml,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/stage2/lio'))
from audit_navigation import audit

def evaluate(directory):
    result=audit(directory)
    params=next(iter(yaml.safe_load((directory/'runtime_parameters/lio_mapping.yaml').read_text()).values()))['ros__parameters']
    result['checks']['declared_simulation_noise_model']=all(params[k]==v for k,v in {
        'mapping.acc_cov':1e-6,'mapping.gyr_cov':1e-8,'mapping.b_acc_cov':0.0,'mapping.b_gyr_cov':0.0}.items())
    parameter_manifest=directory/'runtime_parameter_sha256.json'
    parameter_hashes=json.loads(parameter_manifest.read_text()) if parameter_manifest.exists() else {}
    expected={str(p.relative_to(directory)) for p in (directory/'runtime_parameters').glob('*') if p.is_file()}
    expected.add('runtime_parameter_gate.json')
    result['checks']['runtime_parameter_evidence_hashes']=set(parameter_hashes)==expected and all(
        (directory/p).is_file() and hashlib.sha256((directory/p).read_bytes()).hexdigest()==h for p,h in parameter_hashes.items())
    gate=directory/'runtime_parameter_gate.json'
    result['checks']['runtime_parameter_gate_passed']=gate.is_file() and json.loads(gate.read_text()).get('pass') is True
    result['all_pass']=all(result['checks'].values());result['model_evaluator_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    result['model_evaluator_command']=sys.argv
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    try:r=evaluate(a.result_dir)
    except Exception as e:
        needed=['lio_full_01.offline_preflight.json','lio_goal_preflight.json',
                'runtime_parameters/lio_mapping.yaml','runtime_parameter_gate.json']
        r={'all_pass':False,'error':repr(e),'status':'INCOMPLETE_EVIDENCE',
           'missing_required_files':[f for f in needed if not (a.result_dir/f).exists()],
           'model_evaluator_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
           'model_evaluator_command':sys.argv}
        progress=a.result_dir/'progress.json'
        if progress.exists():
            r['runtime_progress']=json.loads(progress.read_text())
            if r['runtime_progress'].get('goals_requested')==0:r['task_coverage']='NOT_EXERCISED'
    with a.output.open('x') as f:json.dump(r,f,indent=2)
    print(json.dumps({k:r.get(k) for k in ['all_pass','checks','error']},indent=2));raise SystemExit(0 if r['all_pass'] else 2)
