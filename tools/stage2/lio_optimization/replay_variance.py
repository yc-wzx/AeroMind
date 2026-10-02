#!/usr/bin/env python3
"""Offline-only replay into a separate DDS domain; never publishes a goal/velocity/GT."""
import argparse,base64,hashlib,json,os,signal,subprocess,sys,threading,time
from collections import Counter
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu,PointCloud2
from nav_msgs.msg import Odometry
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/stage2/lio')]
from provincial_native_trace import NativeTrace
from sensor_trace import SensorTrace

def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
def write(path,data):
    with path.open('x') as f:json.dump(data,f,indent=2)

class Replay(Node):
    def __init__(self,out):
        super().__init__('sensor_replay_observer',namespace='/offline_lio')
        self.native=NativeTrace(out/'output.native.jsonl');self.sensor=SensorTrace(out/'output.clouds.jsonl')
        self.clock_pub=self.create_publisher(Clock,'/offline_lio/clock',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.cloud_pub=self.create_publisher(PointCloud2,'/offline_lio/lidar',10)
        self.imu_pub=self.create_publisher(Imu,'/offline_lio/imu',QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Odometry,'/offline_lio/odometry',lambda m:self.native.record('/offline_lio/odometry',m),1000)
        self.create_subscription(PointCloud2,'/offline_lio/cloud_registered',lambda m:self.sensor.record('/offline_lio/cloud_registered',m),100)
        self.create_subscription(Clock,'/offline_lio/clock',self.clock_cb,QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT))
    def clock_cb(self,m):
        self.native.sim_s=self.sensor.sim_s=m.clock.sec+m.clock.nanosec*1e-9
        self.native.record('/offline_lio/clock',m)

def run(source,out,speed,planar_model=False,binary_override=None,declared_noise_model=False,point_variance=None):
    if os.environ.get('ROS_DOMAIN_ID') not in ['73','74']:raise RuntimeError('Offline replay requires isolated ROS_DOMAIN_ID73 or74; domain0 forbidden')
    if out.exists():raise FileExistsError(out)
    out.mkdir(parents=True)
    seal=json.loads((ROOT/'tools/results/stage2_step4_lio_safe_profile_20261002/final_evidence_seal.json').read_text())
    changed=[p for p,h in seal['source_hashes'].items() if sha(ROOT/p)!=h]
    if changed:raise RuntimeError('Sealed source changed: '+repr(changed))
    params=json.loads((source/'input_manifest.json').read_text())
    hashes=json.loads((source/'raw_data_sha256.json').read_text())
    for file in ['lio_full_01.native.jsonl','lio_full_01.sensors.jsonl']:
        if sha(source/file)!=hashes[file]:raise RuntimeError('Raw source hash mismatch '+file)
    actual=yaml.safe_load((source/'runtime_parameters/lio_mapping.yaml').read_text())['/lio_mapping']['ros__parameters']
    effective=dict(actual);overrides={}
    if planar_model:
        overrides={'common.planar_mode':True,'common.planar_height':0.0}
        effective.update(overrides)
    if declared_noise_model:
        if planar_model:raise ValueError('Keep noise-model experiment separate from planar experiment')
        overrides={'mapping.acc_cov':.001**2,'mapping.gyr_cov':.0001**2,
            'mapping.b_acc_cov':0.0,'mapping.b_gyr_cov':0.0}
        effective.update(overrides)
    if point_variance is not None:
        if not binary_override or point_variance not in [.001,.0001]:
            raise ValueError('Only the planned opt-in variance comparison is allowed')
        overrides['mapping.laser_point_covariance']=point_variance
        effective.update(overrides)
    (out/'replay_tool_snapshot.py').write_bytes(Path(__file__).read_bytes())
    write(out/'input_manifest.json',{'original_seal':str(ROOT/'tools/results/stage2_step4_lio_safe_profile_20261002/final_evidence_seal.json'),
        'sealed_source_match':not changed,'raw_source':str(source),'raw_input_sha256':{p:hashes[p] for p in ['lio_full_01.native.jsonl','lio_full_01.sensors.jsonl']},
        'replay_tool_sha256':sha(Path(__file__)),'DDS_domain':int(os.environ['ROS_DOMAIN_ID']),'speed_factor':speed,'sensor_content_changed':False,
        'GT_and_navigation_input_replayed':False,'original_parameters':actual,'effective_parameters':effective,'parameter_overrides':overrides})
    parameter_file=out/'replay_parameters.yaml';parameter_file.write_text(yaml.safe_dump({'/**':{'ros__parameters':effective}}))
    events=[]
    for line in (source/'lio_full_01.native.jsonl').open():
        r=json.loads(line)
        if r['topic']=='/clock':events.append((r['receive_monotonic_ns'],'clock',r['data']))
    for line in (source/'lio_full_01.sensors.jsonl').open():
        r=json.loads(line)
        if r['topic'] in ['/lio/lidar','/lio/imu']:
            events.append((r['receive_monotonic_ns'],'cloud' if r['topic']=='/lio/lidar' else 'imu',r['cdr']))
    events.sort(key=lambda row:row[0])
    rclpy.init(args=['--ros-args','-r','/clock:=/offline_lio/clock','-p','use_sim_time:=true'])
    node=Replay(out);thread=threading.Thread(target=lambda:rclpy.spin(node),daemon=True);thread.start()
    binary=binary_override.resolve() if binary_override else ROOT/'install/spark_fast_lio/lib/spark_fast_lio/spark_lio_mapping'
    child_env=dict(os.environ)
    if binary_override:
        if not (binary.parent/'libspark_lio_component.so').is_file():
            raise RuntimeError('Diagnostic component must be built; launcher alone is insufficient')
        child_env['LD_LIBRARY_PATH']=str(binary.parent)+':'+child_env.get('LD_LIBRARY_PATH','')
    command=[str(binary),'--ros-args','-r','__ns:=/offline_lio','-r','__node:=offline_lio_mapping',
        '-r','/clock:=/offline_lio/clock','-r','/tf:=/offline_lio/tf','-r','/tf_static:=/offline_lio/tf_static','--params-file',str(parameter_file)]
    links=subprocess.run(['ldd',str(binary)],capture_output=True,text=True,env=child_env,check=True)
    if 'not found' in links.stdout:raise RuntimeError('Offline LIO dependency unresolved')
    (out/'actual_link_dependencies.txt').write_text(links.stdout)
    write(out/'launch_command.json',{'argv':command,'binary_sha256':sha(binary),'scope':'No Gazebo or navigation nodes',
        'binary_override':str(binary_override) if binary_override else None,'LD_LIBRARY_PATH':child_env.get('LD_LIBRARY_PATH')})
    log=(out/'lio.log').open('x');process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=child_env)
    sent=Counter();status={'finished':False};published=(out/'input_delivery.jsonl').open('x')
    try:
        deadline=time.monotonic()+20;stable=None
        while time.monotonic()<deadline:
            ready=all(any(e.node_name=='offline_lio_mapping' for e in node.get_subscriptions_info_by_topic(t)) for t in ['/offline_lio/lidar','/offline_lio/imu'])
            stable=(stable or time.monotonic()) if ready else None
            if stable is not None and time.monotonic()-stable>.5:break
            if process.poll() is not None:raise RuntimeError('LIO process exited before ready')
            time.sleep(.05)
        else:raise RuntimeError('Offline LIO input endpoints not ready')
        graph={t:{'pub':[e.node_name for e in node.get_publishers_info_by_topic(t)],'sub':[e.node_name for e in node.get_subscriptions_info_by_topic(t)]}
            for t in ['/offline_lio/lidar','/offline_lio/imu','/offline_lio/clock','/offline_lio/odometry']}
        write(out/'isolated_graph_before.json',graph)
        mapped_libraries=sorted({line.split(maxsplit=5)[-1].strip() for line in Path('/proc/'+str(process.pid)+'/maps').read_text().splitlines()
            if '/libspark_lio_component.so' in line})
        write(out/'actually_loaded_LIO_component.json',{'pid':process.pid,'paths':mapped_libraries,
            'sha256':{p:sha(Path(p)) for p in mapped_libraries}})
        expected_component=(binary.parent/'libspark_lio_component.so').resolve() if binary_override else (ROOT/'install/spark_fast_lio/lib/libspark_lio_component.so').resolve()
        if mapped_libraries!=[str(expected_component)]:raise RuntimeError('Actually loaded LIO component mismatch')
        start=time.monotonic_ns();origin=events[0][0]
        for original_ns,kind,data in events:
            due=start+int((original_ns-origin)/speed)
            delay=(due-time.monotonic_ns())*1e-9
            if delay>0:time.sleep(delay)
            if process.poll() is not None:raise RuntimeError('Offline LIO exited during replay')
            if kind=='clock':
                message=Clock();message.clock.sec=data['clock']['sec'];message.clock.nanosec=data['clock']['nanosec'];node.clock_pub.publish(message)
                content_sha=None
            else:
                raw=base64.b64decode(data,validate=True);message=deserialize_message(raw,PointCloud2 if kind=='cloud' else Imu)
                (node.cloud_pub if kind=='cloud' else node.imu_pub).publish(message);content_sha=hashlib.sha256(raw).hexdigest()
            sent[kind]+=1
            published.write(json.dumps({'kind':kind,'sequence':sent[kind],'original_receive_monotonic_ns':original_ns,
                'replay_publish_monotonic_ns':time.monotonic_ns(),'original_CDR_sha256':content_sha})+'\n')
        time.sleep(2);status['finished']=True
    except Exception as error:status['error']=repr(error)
    except KeyboardInterrupt:status['error']='REPLAY_INTERRUPTED; NOT_VALID_DIAGNOSTIC_RUN'
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        time.sleep(.2)
        if rclpy.ok():rclpy.shutdown()
        thread.join(timeout=5)
        node.native.close();node.sensor.close();node.destroy_node();published.close();log.close()
        status.update(sent=dict(sent),original_event_count=len(events),process_returncode=process.returncode,
            input_content_unchanged=all(sha(source/p)==hashes[p] for p in ['lio_full_01.native.jsonl','lio_full_01.sensors.jsonl']))
        write(out/'progress.json',status)
        write(out/'data_sha256.json',{p.name:sha(p) for p in out.iterdir() if p.is_file()})
    print(json.dumps(status,indent=2));return status['finished']

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--speed',type=float,default=.5);p.add_argument('--planar-model',action='store_true');p.add_argument('--declared-noise-model',action='store_true');p.add_argument('--binary',type=Path);p.add_argument('--point-variance',type=float);a=p.parse_args()
    if not 0<a.speed<=1:raise ValueError('Replay speed must be within (0,1]')
    raise SystemExit(0 if run(a.source.resolve(),a.output.resolve(),a.speed,a.planar_model,a.binary,a.declared_noise_model,a.point_variance) else 2)
