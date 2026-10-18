"""Interactive read-only mounted-gimbal check; background ROS capture never waits for keyboard input."""
import base64,json,select,subprocess,sys,threading,time
from pathlib import Path
from analyze import HERE,PHASE,STREAMS,CONTRACT,MANIFEST,verify,sha,analyze,pose_row,stamp_ns,imu_values
sys.path.insert(0,str(HERE.parent/'mid360_lio'))
from static_check import Writer
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from rcl_interfaces.srv import GetParameters

class Recorder:
    def __init__(self,path):
        self.writer=Writer(path);self.context=Context();rclpy.init(context=self.context)
        self.node=Node('mid360_manual_gimbal_observer',context=self.context)
        self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
        self.stop=threading.Event();self.latest={};self.counts={k:0 for k in STREAMS};self.sequence=0;self.error=None
        self.accepting=True;self.lock=threading.Lock()
        qos=QoSProfile(depth=1000,reliability=ReliabilityPolicy.RELIABLE)
        self.subs=[self.node.create_subscription(TFMessage if k=='tf' else Imu if k=='imu_si' else Odometry,
            topic,lambda data,kind=k:self.receive(kind,data),qos,raw=True) for k,topic in STREAMS.items()]
        self.thread=threading.Thread(target=self.spin);self.thread.start()
    def spin(self):
        try:
            while not self.stop.is_set():self.executor.spin_once(timeout_sec=.01)
        except Exception as error:
            if not self.stop.is_set():self.error=str(error)
    def receive(self,kind,payload):
        if not self.accepting:return
        receipt=time.monotonic_ns();wall=time.time_ns();self.sequence+=1
        self.writer.put(dict(sequence=self.sequence,stream=kind,receive_monotonic_ns=receipt,receive_wall_ns=wall,
                             cdr_base64=base64.b64encode(payload).decode()))
        with self.lock:self.latest[kind]=(receipt,payload);self.counts[kind]+=1
    def fresh(self):
        with self.lock:latest=dict(self.latest);counts=dict(self.counts)
        if not all(counts[k]>= (20 if k=='imu_si' else 3) for k in STREAMS):return False
        for kind,(receipt,payload) in latest.items():
            if (time.monotonic_ns()-receipt)*1e-9>(.1 if kind=='imu_si' else .2):return False
            msg=deserialize_message(payload,TFMessage if kind=='tf' else Imu if kind=='imu_si' else Odometry)
            if kind=='tf':
                target=[t for t in msg.transforms if t.header.frame_id=='mid360_local_odom' and t.child_frame_id=='mid360_imu']
                if len(target)!=1:return False
                stamp_ns(target[0])
            elif kind=='imu_si':
                imu_values(msg)
                if msg.header.frame_id!='mid360_imu':return False
            else:stamp_ns(msg);pose_row(msg)
        return True
    def publishers(self):
        return {k:[dict(name=p.node_name,namespace=p.node_namespace,type=p.topic_type) for p in self.node.get_publishers_info_by_topic(topic)] for k,topic in STREAMS.items()}
    def parameters(self):
        client=self.node.create_client(GetParameters,'/hardware/mid360/lio/mid360_real_lio/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=2):raise RuntimeError('LIO参数服务不可用')
            request=GetParameters.Request();request.names=['use_sim_time','common.odometry_output_contract','common.visualization_frame',
                'common.imu_frame','common.map_frame','common.base_frame','common.time_sync_en','mapping.extrinsic_T','mapping.extrinsic_R']
            future=client.call_async(request);deadline=time.monotonic()+2
            while not future.done() and time.monotonic()<deadline:time.sleep(.01)
            if not future.done():raise RuntimeError('LIO参数读取超时')
            return {name:v.bool_value if v.type==1 else v.string_value if v.type==4 else list(v.double_array_value) for name,v in zip(request.names,future.result().values)}
        finally:self.node.destroy_client(client)
    def close(self):
        self.stop.set();self.thread.join(5)
        if self.thread.is_alive():raise RuntimeError('Capture executor did not stop')
        self.accepting=False;self.executor.remove_node(self.node);self.executor.shutdown();self.node.destroy_node()
        if self.context.ok():rclpy.shutdown(context=self.context)
        health=self.writer.close()
        if self.error:health['error']='ROS executor: '+self.error
        return health

ACTIVE=None
def check_deadline(deadline):
    if time.monotonic()>deadline:raise TimeoutError('本轮超过180秒，已结束记录；请停止手动转动')
    if ACTIVE and ACTIVE.error:raise RuntimeError('ROS采集线程异常：'+ACTIVE.error)

def wait_enter(tag,prompt,deadline):
    print(prompt,flush=True)
    if not sys.stdin.isatty():raise RuntimeError('请在NoMachine普通终端交互运行此命令')
    while True:
        check_deadline(deadline)
        if select.select([sys.stdin],[],[],.1)[0]:
            line=sys.stdin.readline()
            if not line:raise RuntimeError('操作输入已关闭')
            if not line.strip():return
            print('这里只需按回车，不用输入命令。',flush=True)

def hold(label,deadline):
    print(label+'：保持底盘、云台和雷达静止5秒。',flush=True)
    end=time.monotonic()+5.
    while time.monotonic()<end:check_deadline(deadline);time.sleep(.05)

def source_ok(sources):
    return all(len(sources.get(k,[]))==1 and sources[k][0]['name']=='mid360_real_lio' and sources[k][0]['namespace']=='/hardware/mid360/lio' for k in ['corrected','predicted','mixed']) and len(sources.get('imu_si',[]))==1 and sources['imu_si'][0]['name']=='mid360_imu_unit_adapter'

def main():
    global ACTIVE
    verify();manifest=MANIFEST.read_bytes();manifest_sha=sha(MANIFEST)
    tools={str(p):sha(p) for p in HERE.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    out=Path.home()/'AeroMind_mid360_gimbal_checks'/('turn_return_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir(parents=True,exist_ok=False)
    (out/'variant_manifest_snapshot.json').write_bytes(manifest)
    recorder=Recorder(out/'native.jsonl');ACTIVE=recorder
    event_stream=(out/'events.jsonl').open('x');interrupted=False;failure=None;before_params={};parameters={};sources={};ready=False
    def mark(phase,edge):
        event_stream.write(json.dumps(dict(phase=phase,edge=edge,receive_monotonic_ns=time.monotonic_ns(),receive_wall_ns=time.time_ns()))+'\n');event_stream.flush()
    try:
        deadline=time.monotonic()+8
        print('正在检查真实雷达、新版LIO及IMU输入；本工具不发送运动指令。',flush=True)
        while time.monotonic()<deadline:
            if recorder.error:raise RuntimeError(recorder.error)
            if recorder.fresh():ready=True;break
            time.sleep(.02)
        if not ready:raise RuntimeError('输入未就绪：'+json.dumps(recorder.counts))
        sources=recorder.publishers();before_params=recorder.parameters()
        if not source_ok(sources) or before_params.get('common.odometry_output_contract')!=CONTRACT or before_params.get('use_sim_time') is not False:
            raise RuntimeError('输入来源或LIO版本不匹配')
        deadline=time.monotonic()+180
        wait_enter('ready','已就绪。底盘保持不动，记住或标记当前云台朝向。准备好后按回车，先记录原位静止。',deadline)
        mark('baseline','start');hold('原位记录',deadline);mark('baseline','end')
        wait_enter('start_turn','接下来从上向下看，逆时针（向左）慢转约45°。这只是操作目标，不是精度真值。先按回车，再开始转动。',deadline)
        mark('turn','start')
        wait_enter('finish_turn','现在慢转（建议用5秒以上）。转到位置并停止后，回来按回车。不要移动底盘。',deadline)
        mark('turn','end');mark('turned_hold','start');hold('转后记录',deadline);mark('turned_hold','end')
        wait_enter('start_return','准备沿原路慢慢转回起始标记。先按回车，再开始转回。',deadline)
        mark('return','start')
        wait_enter('finish_return','现在慢慢转回。对准起始标记并停止后，回来按回车。',deadline)
        mark('return','end');mark('final_hold','start');hold('回原位记录',deadline);mark('final_hold','end')
        sources=recorder.publishers();parameters=recorder.parameters()
    except KeyboardInterrupt:interrupted=True;failure='用户中断；请停止手动转动'
    except Exception as error:failure=str(error)
    finally:
        health=recorder.close();ACTIVE=None;event_stream.close()
    try:verify();unchanged=manifest_sha==sha(MANIFEST) and all(Path(p).is_file() and sha(p)==v for p,v in tools.items())
    except Exception:unchanged=False
    end_parameters_received=bool(parameters)
    if not parameters:parameters=before_params
    meta=dict(health=health,publishers=sources,actual_parameters=parameters,inputs_unchanged=unchanged,parameters_unchanged=bool(end_parameters_received and before_params and parameters==before_params),
        interrupted=interrupted,failure=failure,variant_manifest_sha256=manifest_sha,tool_sha256=tools,
        data_sha256={name:sha(out/name) for name in ['native.jsonl','events.jsonl']},
        operator_declarations='Mounted inverted radar, fixed chassis, CCW outward then CW return; not independently measured')
    (out/'capture_meta.json').write_text(json.dumps(meta,indent=2,allow_nan=False)+'\n')
    result=analyze(out)
    if failure:result['capture_failure']=failure;result['evidence_complete']=False;result['motion_check_complete']=False
    (out/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2,allow_nan=False));print('SUMMARY:',out/'summary.json',flush=True)
    if not ready:
        print('请保持两个启动终端运行：\n  bash tools/hardware/mid360/run.sh\n  bash tools/hardware/mid360_lio_phased/run.sh /home/rm/AeroMind_mid360_captures/20261017_214624/summary.json',flush=True)
    return 0 if result.get('motion_check_complete') else 2
if __name__=='__main__':raise SystemExit(main())
