import json,os,subprocess,sys,time
from pathlib import Path
from variant import HERE,ROOT,INSTALL,MANIFEST,CONTRACT,verify,sha
sys.path.insert(0,str(HERE.parent/'mid360_lio'))
from start import live_ready
from inspect_raw import inspect

def main():
    if len(sys.argv)!=2:raise RuntimeError('Supply the original passing raw summary.json')
    record=verify();raw=inspect(sys.argv[1])
    prefix=subprocess.check_output(['ros2','pkg','prefix','spark_fast_lio'],text=True).strip()
    if Path(prefix).resolve()!=(INSTALL/'spark_fast_lio').resolve():raise RuntimeError('Wrong overlay selected: '+prefix)
    ready=live_ready()
    out=Path.home()/'AeroMind_mid360_lio_checks'/('start_phased_'+time.strftime('%Y%m%d_%H%M%S'))
    out.mkdir(parents=True,exist_ok=False)
    command=['ros2','launch',str(HERE/'readonly_lio.launch.py')]
    (out/'startup_inputs.json').write_text(json.dumps(dict(contract=CONTRACT,variant_manifest_sha256=sha(MANIFEST),
        variant_manifest=record,raw_inspection=raw,live_inputs=ready,selected_prefix=prefix,command=command),indent=2)+'\n')
    print('PHASED LIO START:',out/'startup_inputs.json',flush=True)
    os.execvpe(command[0],command,os.environ)
if __name__=='__main__':main()
