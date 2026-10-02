#!/usr/bin/env python3
"""Exercise actual bounded parameter reader without Gazebo or navigation."""
import importlib.util,json,os,subprocess,sys,tempfile,time
from pathlib import Path
from unittest.mock import patch
import yaml

ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('model_runner',Path(__file__).with_name('run_model_experiment.py'))
runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)

def run(out):
    if os.environ.get('ROS_DOMAIN_ID')!='75':raise RuntimeError('Use isolated ROS_DOMAIN_ID=75')
    out.mkdir(exist_ok=False)
    old=ROOT/'tools/results/stage2_step4_lio_replay_20261001/model_full_01/input_snapshot/tools/stage2/lio_model_validation/dump_parameters.py'
    before=subprocess.run([sys.executable,'-B',str(old),'/lio_mapping'],capture_output=True,text=True)
    runner.write(out/'before_missing_entry.json',{'command':before.args,'returncode':before.returncode,'stderr':before.stderr})
    checks={'before_missing_entry_reproduced':before.returncode==2 and 'No such file' in before.stderr}
    node_code="""import rclpy
from rclpy.node import Node
rclpy.init()
n=Node('parameter_entry_fixture')
for k,v in [('mapping.acc_cov',1e-6),('mapping.gyr_cov',1e-8),('mapping.b_acc_cov',0.0),('mapping.b_gyr_cov',0.0),('common.planar_mode',False)]:n.declare_parameter(k,v)
try:rclpy.spin(n)
finally:n.destroy_node();rclpy.try_shutdown()
"""
    log=(out/'fixture.log').open('x')
    fixture=subprocess.Popen([sys.executable,'-B','-c',node_code],stdout=log,stderr=subprocess.STDOUT)
    try:
        time.sleep(1)
        runner.parameter_snapshot('/parameter_entry_fixture',out)
        p=yaml.safe_load((out/'parameter_entry_fixture.yaml').read_text())['/parameter_entry_fixture']['ros__parameters']
        checks['actual_ROS_service_values_exact']=all(p[k]==v for k,v in {'mapping.acc_cov':1e-6,'mapping.gyr_cov':1e-8,'mapping.b_acc_cov':0.0,'mapping.b_gyr_cov':0.0,'common.planar_mode':False}.items())
        q=json.loads((out/'parameter_entry_fixture.query.json').read_text())
        checks['query_command_output_timing_retained']=q['returncode']==0 and q['ended_monotonic_ns']>=q['started_monotonic_ns'] and bool(q['stdout'])
        try:runner.parameter_snapshot('/missing_parameter_fixture',out)
        except RuntimeError:checks['missing_service_rejected_and_saved']=json.loads((out/'missing_parameter_fixture.query.json').read_text())['returncode']!=0
        with patch.object(runner.subprocess,'run',side_effect=subprocess.TimeoutExpired(['fixture'],25)):
            try:runner.parameter_snapshot('/timeout_fixture',out)
            except RuntimeError:checks['timeout_rejected_and_saved']=json.loads((out/'timeout_fixture.query.json').read_text())['returncode'] is None
        try:runner.parameter_snapshot('/parameter_entry_fixture',out)
        except FileExistsError:checks['existing_evidence_not_overwritten']=True
    finally:
        fixture.send_signal(2);fixture.wait(timeout=10);log.close()
    report={'all_pass':len(checks)==6 and all(checks.values()),'checks':checks,'scope':'Isolated ROS parameter services only; no Gazebo, sensors, goals or velocity publication'}
    runner.write(out/'verification.json',report);print(json.dumps(report,indent=2))
    return report['all_pass']

if __name__=='__main__':raise SystemExit(0 if run(Path(sys.argv[1])) else 2)
