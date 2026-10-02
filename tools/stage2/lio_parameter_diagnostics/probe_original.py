#!/usr/bin/env python3
"""Isolated actual sensor adapter and original parameter client; no inputs/goals."""
from pathlib import Path
import subprocess,json,os,sys,signal
ROOT=Path(__file__).resolve().parents[3]
out=Path(sys.argv[1]);out.mkdir(exist_ok=False)
env=os.environ.copy();env['ROS_DOMAIN_ID']='76'
log=(out/'sensor_adapter.log').open('x')
process=subprocess.Popen([sys.executable,'-B',str(ROOT/'install/stage2_lio_sim/lib/stage2_lio_sim/lio_sensor_adapter.py'),
    '--ros-args','-p','use_sim_time:=false'],stdout=log,stderr=subprocess.STDOUT,env=env)
results=[]
try:
    for index in range(6):
        q=subprocess.run([sys.executable,'-B',str(ROOT/'tools/stage2/lio_model_validation/dump_parameters.py'),'/lio_sensor_adapter'],
            capture_output=True,text=True,env=env,timeout=25)
        results.append({'index':index,'returncode':q.returncode,'stdout':q.stdout,'stderr':q.stderr})
finally:
    process.send_signal(signal.SIGINT);process.wait(timeout=10);log.close()
(out/'old_reader_probe.json').write_text(json.dumps(results,indent=2))
print([(a['index'],a['returncode']) for a in results])
