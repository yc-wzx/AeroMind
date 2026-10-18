"""Synthetic DDS + virtual UART translation fixture. Never touches physical hardware."""
import argparse,json,math,os,pty,select,threading,time,tty,termios
from pathlib import Path
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from translation_record import run_trial
from translation_review import save_review,sha
from tests_translation import REF,PARAMS,PROFILE,MEASUREMENTS,frame,mount,rz

def main():
    p=argparse.ArgumentParser();p.add_argument('--output-dir',required=True);p.add_argument('--drop-imu',action='store_true');a=p.parse_args();root=Path(a.output_dir).resolve()
    stop=threading.Event();ready=threading.Event();errors=[];seen=[];master,slave=pty.openpty();device=os.ttyname(slave);tty.setraw(slave,termios.TCSANOW);os.close(slave);os.set_blocking(master,False)
    def producer():
        ctx=Context();rclpy.init(context=ctx);node=Node('mid360_real_lio',namespace='/hardware/mid360/lio',context=ctx);imu_node=Node('mid360_imu_unit_adapter',context=ctx)
        executor=SingleThreadedExecutor(context=ctx);executor.add_node(node);executor.add_node(imu_node)
        for k,v in PARAMS.items():
            if not node.has_parameter(k):node.declare_parameter(k,v)
        node.declare_parameter('common.base_frame','')
        pubs={k:node.create_publisher(TFMessage if k=='tf' else Odometry,topic,1000) for k,topic in {'corrected':'/hardware/mid360/lio/odometry','predicted':'/hardware/mid360/lio/odometry_predicted','mixed':'/hardware/mid360/lio/odometry_mixed_legacy','tf':'/tf'}.items()}
        ip=imu_node.create_publisher(Imu,'/hardware/mid360/imu_si',qos_profile_sensor_data)
        _,arm,_=mount(MEASUREMENTS);R=rz(.7);R0=R@__import__('numpy').diag([1.,-1.,-1.]);tick=0;deadline=time.monotonic();baseline=None
        try:
            ready.set()
            while not stop.is_set():
                event_file=root/'events.jsonl';events=[]
                if event_file.exists():events=[json.loads(s) for s in event_file.read_text().splitlines() if s]
                now=time.monotonic_ns();progress=0.
                if events:
                    event=events[-1];phase=event['phase'];edge=event['edge'];elapsed=(now-event['receive_monotonic_ns'])*1e-9
                    if baseline is None:baseline=events[0]['receive_monotonic_ns']
                    if phase=='outward':progress=min(1.,elapsed/4) if edge=='start' else 1.
                    elif phase=='far_hold':progress=1.
                    elif phase=='return':progress=max(0.,1-elapsed/4) if edge=='start' else 0.
                delta=R0@__import__('numpy').asarray([.5*progress,0.,0.]);pos=delta;stamp=time.time_ns()
                imu=Imu();imu.header.frame_id='mid360_imu';imu.header.stamp.sec,imu.header.stamp.nanosec=divmod(stamp,10**9);imu.linear_acceleration.z=-9.80665
                if not (a.drop_imu and baseline is not None and now-baseline>2_000_000_000):ip.publish(imu)
                msg=Odometry();msg.header.frame_id='mid360_local_odom';msg.child_frame_id='mid360_imu';msg.header.stamp=imu.header.stamp
                msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=pos.tolist();msg.pose.pose.orientation.z=math.sin(.35);msg.pose.pose.orientation.w=math.cos(.35)
                pubs['predicted'].publish(msg);pubs['mixed'].publish(msg)
                if tick%20==0:
                    pubs['corrected'].publish(msg);pubs['mixed'].publish(msg)
                    tr=TransformStamped();tr.header=msg.header;tr.child_frame_id=msg.child_frame_id;tr.transform.rotation=msg.pose.pose.orientation;tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=pos.tolist();tf=TFMessage();tf.transforms=[tr];pubs['tf'].publish(tf)
                try:os.write(master,frame(poly=0x8408,angle=.019))
                except OSError:pass
                if select.select([master],[],[],0)[0]:
                    try:seen.append(os.read(master,4096))
                    except OSError:pass
                executor.spin_once(timeout_sec=0);tick+=1;deadline+=.005;time.sleep(max(0,deadline-time.monotonic()))
        except Exception as error:errors.append(repr(error));ready.set()
        finally:
            executor.remove_node(node);executor.remove_node(imu_node);executor.shutdown();node.destroy_node();imu_node.destroy_node()
            if ctx.ok():rclpy.shutdown(context=ctx)
    thread=threading.Thread(target=producer);thread.start();ready.wait(5)
    def keyboard(label,elapsed):return elapsed>(4. if label in ['outward_end','return_end'] else .05)
    try:
        if errors:raise RuntimeError(errors)
        result=run_trial(root,device,PROFILE,REF,operator=keyboard,allow_test_pty=True)
    finally:stop.set();thread.join(5);os.close(master)
    independent=save_review(root,'independent_translation_review_v1.json');assert result==independent
    assert not any(seen),'Unexpected receiver TX payload';assert not errors,errors
    if a.drop_imu:
        assert not result['diagnostic_complete'];assert '过期' in result['capture_failure']
        events=[json.loads(s) for s in (root/'events.jsonl').read_text().splitlines()]
        assert not any(e['phase']=='outward' for e in events)
    else:
        assert result['diagnostic_complete'],result
        assert abs(result['statistics']['estimated_forward_distance_m']-.5)<1e-9
        assert result['statistics']['return_closure_3d_m']<1e-9
        assert not result['accuracy_accepted'] and not result['navigation_ready']
    report=dict(scope='SYNTHETIC DDS + VIRTUAL UART TRANSLATION; NOT PHYSICAL ACCEPTANCE',
        production_orchestrator_and_recorders_used=True,test_hooks='Virtual UART path and scripted operator Enter timing only',
        diagnostic_complete=result['diagnostic_complete'],expected_imu_failure=a.drop_imu,independent_review_identical=True,
        no_serial_transmit_payload_observed=not any(seen),health=result['health'],
        statistics={k:v for k,v in result.get('statistics',{}).items() if k!='mount_worksheet'},fixture_sha256=sha(Path(__file__)))
    name='imu_loss_verification.json' if a.drop_imu else 'e2e_verification.json'
    with (root.parent/name).open('x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
    print('VIRTUAL TRANSLATION VERIFIED:',root.parent/name)
if __name__=='__main__':main()
