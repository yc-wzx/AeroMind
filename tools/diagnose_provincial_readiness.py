"""One no-goal Gazebo diagnosis. Recorder does not call run() or publish()."""
import argparse,json,os,signal,subprocess
from pathlib import Path
import rclpy
import run_provincial_full_observed as observed
from run_provincial_low_speed_validation import Runner
from run_provincial_safety_fix_matrix import existing_servers,wait_for_launch
from run_provincial_forward_integration import server_processes

p=argparse.ArgumentParser();p.add_argument('--output-dir',type=Path,required=True);args=p.parse_args();out=args.output_dir.resolve()
if out.exists() or existing_servers():raise RuntimeError('Refuse existing directory/server')
out.mkdir(parents=True);observed.OUT=out;observed.NAME='readiness_diagnostic';observed.INPUTS+=('tools/diagnose_provincial_readiness.py',);manifest=observed.capture_inputs();launch=None;runner=None;ready=False
log=(out/'launch.log').open('x')
try:
    launch=subprocess.Popen(observed.LAUNCH,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);wait_for_launch();observed.capture_runtime_params()
    rclpy.init();runner=Runner('full','readiness_diagnostic',1,'hold-start',out,raw_every_message=True)
    ready=runner.wait_ready();runner.save('diagnostic_ready' if ready else 'diagnostic_not_ready')
    print(json.dumps({'ready':ready,'goal_publish_count':runner.goal_delivery['publish_count'],'samples':runner.goal_delivery.get('readiness_samples')},ensure_ascii=False),flush=True)
finally:
    if runner:runner.destroy_node();rclpy.shutdown()
    if launch:
        try:os.killpg(launch.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:launch.wait(timeout=12)
        except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=8)
    log.close();observed.final_hashes(manifest)
    (out/'diagnostic_result.json').write_text(json.dumps({'ready':ready,'no_run_method_called':True,'goals_published':0,'servers_after':server_processes()},indent=2))
