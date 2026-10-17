import argparse,base64,json,sys,time
from pathlib import Path
from variant import HERE,MANIFEST,verify,sha,CONTRACT
from audit import TOPICS,audit
sys.path.insert(0,str(HERE.parent/'mid360_lio'))
from static_check import Writer
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rcl_interfaces.srv import GetParameters

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--seconds',type=float,default=30);args=parser.parse_args()
    if not 15<=args.seconds<=60:parser.error('seconds must be 15..60')
    verify();manifest_bytes=MANIFEST.read_bytes();manifest_sha=sha(MANIFEST)
    out=Path.home()/'AeroMind_mid360_lio_checks'/('static_phased_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir(parents=True,exist_ok=False)
    (out/'variant_manifest_snapshot.json').write_bytes(manifest_bytes)
    writer=Writer(out/'native.jsonl');context=Context();rclpy.init(context=context)
    node=Node('mid360_phased_static_observer',context=context);executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    sequence=0;seen=set();accepting=True;interrupted=False;publishers={};parameters={}
    def receive(kind,payload):
        nonlocal sequence
        if not accepting:return
        receipt=time.monotonic_ns();wall=time.time_ns();sequence+=1;seen.add(kind)
        writer.put(dict(sequence=sequence,stream=kind,receive_monotonic_ns=receipt,receive_wall_ns=wall,
                        cdr_base64=base64.b64encode(payload).decode()))
    qos=QoSProfile(depth=1000,reliability=ReliabilityPolicy.RELIABLE)
    subs=[node.create_subscription(TFMessage if k=='tf' else Odometry,topic,lambda data,kind=k:receive(kind,data),qos,raw=True) for k,topic in TOPICS.items()]
    try:
        deadline=time.monotonic()+15
        while seen!=set(TOPICS) and time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
        deadline=time.monotonic()+args.seconds if seen==set(TOPICS) else time.monotonic()
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
        accepting=False
        publishers={k:[dict(name=p.node_name,namespace=p.node_namespace,type=p.topic_type) for p in node.get_publishers_info_by_topic(t)] for k,t in TOPICS.items()}
        client=node.create_client(GetParameters,'/hardware/mid360/lio/mid360_real_lio/get_parameters')
        if client.wait_for_service(timeout_sec=2):
            request=GetParameters.Request();request.names=['use_sim_time','common.visualization_frame','common.imu_frame','common.map_frame',
                'common.base_frame','common.time_sync_en','mapping.extrinsic_T','mapping.extrinsic_R','common.odometry_output_contract']
            future=client.call_async(request);deadline=time.monotonic()+2
            while not future.done() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
            if future.done():
                for name,value in zip(request.names,future.result().values):
                    parameters[name]=value.bool_value if value.type==1 else value.string_value if value.type==4 else list(value.double_array_value)
    except KeyboardInterrupt:interrupted=True
    finally:
        accepting=False;executor.remove_node(node);executor.shutdown();node.destroy_node();rclpy.shutdown(context=context);health=writer.close()
    try:verify();unchanged=manifest_sha==sha(MANIFEST)
    except Exception:unchanged=False
    meta=dict(seconds=args.seconds,health=health,publishers=publishers,actual_parameters=parameters,
              variant_manifest_sha256=manifest_sha,inputs_unchanged=unchanged,interrupted=interrupted,
              data_sha256={'native.jsonl':sha(out/'native.jsonl')},stationary_condition='operator-declared')
    (out/'capture_meta.json').write_text(json.dumps(meta,indent=2,allow_nan=False)+'\n')
    result=audit(out)
    (out/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2,allow_nan=False));print('SUMMARY:',out/'summary.json',flush=True)
    raise SystemExit(0 if result['evidence_complete'] else 2)
if __name__=='__main__':main()
