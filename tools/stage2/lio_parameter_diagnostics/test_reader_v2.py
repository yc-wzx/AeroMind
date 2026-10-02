#!/usr/bin/env python3
"""Real production node plus adversarial protocol fixtures, isolated DDS only."""
import argparse,json,os,signal,subprocess,sys,time
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[3]

def main(out):
    out.mkdir(exist_ok=False);env=os.environ.copy();env['ROS_DOMAIN_ID']='77';checks={};evidence=[]
    readers={'old':ROOT/'tools/stage2/lio_model_validation/dump_parameters.py',
             'new':ROOT/'tools/stage2/lio_model_validation_v2/dump_parameters.py'}
    def case(label,mode,reader,production=False):
        log=(out/(label+'.node.log')).open('x')
        command=[sys.executable,'-B',str(ROOT/'install/stage2_lio_sim/lib/stage2_lio_sim/lio_sensor_adapter.py')]
        if not production:command=[sys.executable,'-B',str(Path(__file__).with_name('parameter_fixture.py')),'--mode',mode]
        node=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=env)
        try:
            started=time.monotonic()
            q=subprocess.run([sys.executable,'-B',str(readers[reader]),'/lio_sensor_adapter' if production else '/parameter_protocol_fixture'],
                capture_output=True,text=True,env=env,timeout=25)
            result={'label':label,'mode':mode,'reader':reader,'returncode':q.returncode,'elapsed_wall_s':time.monotonic()-started,
                    'stdout':q.stdout,'stderr':q.stderr,'navigation_or_Gazebo_started':False}
            evidence.append(result)
            return result
        finally:
            node.send_signal(signal.SIGINT);node.wait(timeout=12);log.close()
    old=case('old_delayed','delayed','old');checks['old_delayed_response_fails']=old['returncode']!=0 and 'timed out' in old['stderr']
    new=case('new_delayed','delayed','new')
    checks['new_recovers_matching_read_only_response']=new['returncode']==0 and '"attempt": 2' in new['stderr'] and '"response_received": false' in new['stderr']
    if new['returncode']==0:
        params=yaml.safe_load(new['stdout'])['/parameter_protocol_fixture']['ros__parameters']
        checks['delayed_values_exact']=all(params[k]==v for k,v in {'mapping.acc_cov':1e-6,'mapping.gyr_cov':1e-8,'mapping.b_acc_cov':0.,'mapping.b_gyr_cov':0.,'common.planar_mode':False}.items())
    checks['delay_budget_bounded']=new['elapsed_wall_s']<25
    for mode in ['nan','missing','always_delayed']:
        result=case('new_'+mode,mode,'new')
        checks[mode+'_rejected']=result['returncode']!=0
        checks[mode+'_no_YAML_success_output']=not result['stdout'].strip()
    production=case('new_actual_sensor_adapter','normal','new',production=True)
    checks['actual_production_sensor_adapter_read']=production['returncode']==0 and yaml.safe_load(production['stdout'])['/lio_sensor_adapter']['ros__parameters']['use_sim_time'] is False
    report={'all_pass':all(checks.values()),'checks':checks,'evidence':evidence,
        'scope':'DDS domain77; actual sensor adapter without input plus intentionally delayed/incomplete protocol fixtures. No Gazebo, goals, velocity, parameter mutation or GT.',
        'causality_limit':'Injected 8.3s delay is a tool robustness counterexample, not proof of the live DDS response timeout cause.'}
    with (out/'verification.json').open('x') as f:json.dump(report,f,indent=2)
    print(json.dumps({'all_pass':report['all_pass'],'checks':checks},indent=2))
    return report['all_pass']

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    raise SystemExit(0 if main(args.output) else 2)
