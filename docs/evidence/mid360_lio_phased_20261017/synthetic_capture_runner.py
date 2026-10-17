import json,os,signal,subprocess
from pathlib import Path
ROOT=Path('/home/rmnav/aeromind-transfer-20261016');STAGE=ROOT/'tools/hardware/mid360_lio_phased'
OUT=ROOT/'tools/results/mid360_lio_phased_20261017';env=dict(os.environ,MID360_ROS_DOMAIN_ID='218')
fixture=Path(__file__).parent/'mid360_phase_synthetic_source_20261017.py'
with (OUT/'synthetic_source.log').open('x') as source_log:
    source=subprocess.Popen(['bash',str(STAGE/'env.sh'),'python3','-B',str(fixture)],cwd=ROOT,env=env,
        stdout=source_log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        run=subprocess.run(['bash',str(STAGE/'observe_static.sh'),'--seconds','15'],cwd=ROOT,env=env,
            text=True,capture_output=True,timeout=45)
        (OUT/'synthetic_capture.log').write_text(run.stdout+run.stderr)
    finally:
        os.killpg(source.pid,signal.SIGINT)
        try:source.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(source.pid,signal.SIGTERM);source.wait(timeout=5)
if run.returncode:raise RuntimeError('Synthetic capture failed; see preserved log '+str(OUT/'synthetic_capture.log'))
line=next(v for v in run.stdout.splitlines() if v.startswith('SUMMARY: '));summary_path=Path(line[len('SUMMARY: '):])
summary=json.loads(summary_path.read_text())
assert summary['evidence_complete'] is True
repeat=subprocess.run(['bash',str(STAGE/'env.sh'),'python3','-B',str(STAGE/'audit.py'),str(summary_path.parent),
    '--output-name','independent_repeat.json'],cwd=ROOT,env=env,text=True,capture_output=True,timeout=20)
(OUT/'synthetic_reaudit.log').write_text(repeat.stdout+repeat.stderr)
assert repeat.returncode==0
reaudit=json.loads((summary_path.parent/'independent_repeat.json').read_text());assert reaudit==summary
report=dict(scope='Synthetic Python publisher / native recorder / independent audit integration only; not physical LIO estimation',
    isolated_domain=218,evidence_complete=True,audit_repeat_identical=True,capture=str(summary_path),
    streams={k:{q:v.get(q) for q in ['count','receive_hz','regressed_stamps','max_receive_gap_s']} for k,v in summary['streams'].items()},
    phase_identity=summary['phase_identity'],health=summary['health'],gates=summary['gates'])
(OUT/'synthetic_e2e.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
