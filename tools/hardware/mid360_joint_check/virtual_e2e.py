"""Synthetic DDS + virtual UART end-to-end fixture; no hardware or navigation node."""
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
from capture_joint import run_trial
from test_joint import PARAMS,MEASUREMENTS,PROFILE,waveform,frame
from joint_math import mount,rz
from review import save_review,sha

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output-dir',required=True);a=parser.parse_args();root=Path(a.output_dir)
    stop=threading.Event();ready=threading.Event();error=[];payload=[];master,slave=pty.openpty();device=os.ttyname(slave)
    # A fresh PTY defaults to canonical ECHO. After capture restores that prior
    # state the continuing peer would see kernel echo, unrelated to serial TX.
    # Give the fixture a raw initial state; production restoration is unchanged.
    tty.setraw(slave,termios.TCSANOW);os.close(slave);os.set_blocking(master,False)
    def peer():
        context=Context();rclpy.init(context=context);node=Node('mid360_real_lio',namespace='/hardware/mid360/lio',context=context)
        imu_node=Node('mid360_imu_unit_adapter',context=context);executor=SingleThreadedExecutor(context=context);executor.add_node(node);executor.add_node(imu_node)
        for key,value in PARAMS.items():
            if not node.has_parameter(key):node.declare_parameter(key,value)
        node.declare_parameter('common.base_frame','')
        pubs={k:node.create_publisher(TFMessage if k=='tf' else Odometry,topic,1000) for k,topic in {
            'corrected':'/hardware/mid360/lio/odometry','predicted':'/hardware/mid360/lio/odometry_predicted','mixed':'/hardware/mid360/lio/odometry_mixed_legacy','tf':'/tf'}.items()}
        ip=imu_node.create_publisher(Imu,'/hardware/mid360/imu_si',qos_profile_sensor_data)
        _,arm,_=mount(MEASUREMENTS);origin=None;tick=0;deadline=time.monotonic()
        try:
            ready.set()
            while not stop.is_set():
                log=root/'serial_launch.log'
                if origin is None and log.exists() and '原位静止' in log.read_text():origin=time.monotonic_ns()
                now=time.monotonic_ns();t=(now-origin)*1e-9 if origin else -1.
                theta,rate=waveform(t);R=rz(-theta);p=arm-R@arm;stamp=time.time_ns()
                imu=Imu();imu.header.frame_id='mid360_imu';imu.header.stamp.sec,imu.header.stamp.nanosec=divmod(stamp,10**9)
                imu.linear_acceleration.z=-9.80665;imu.angular_velocity.z=-rate;ip.publish(imu)
                msg=Odometry();msg.header.frame_id='mid360_local_odom';msg.child_frame_id='mid360_imu';msg.header.stamp=imu.header.stamp
                msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=p.tolist()
                msg.pose.pose.orientation.z=-math.sin(theta/2);msg.pose.pose.orientation.w=math.cos(theta/2)
                pubs['predicted'].publish(msg);pubs['mixed'].publish(msg)
                if tick%20==0:
                    pubs['corrected'].publish(msg);pubs['mixed'].publish(msg)
                    tr=TransformStamped();tr.header=msg.header;tr.child_frame_id=msg.child_frame_id;tr.transform.rotation=msg.pose.pose.orientation
                    tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=p.tolist()
                    tf=TFMessage();tf.transforms=[tr];pubs['tf'].publish(tf)
                if origin is not None:
                    try:os.write(master,frame(poly=0x8408,angle=.06366+theta))
                    except (BlockingIOError,OSError):pass
                    if select.select([master],[],[],0)[0]:
                        try:payload.append(os.read(master,4096))
                        except (BlockingIOError,OSError):pass
                executor.spin_once(timeout_sec=0);tick+=1;deadline+=.005
                time.sleep(max(0,deadline-time.monotonic()))
        except Exception as ex:error.append(repr(ex));ready.set()
        finally:
            executor.remove_node(node);executor.remove_node(imu_node);executor.shutdown();node.destroy_node();imu_node.destroy_node()
            if context.ok():rclpy.shutdown(context=context)
    thread=threading.Thread(target=peer);thread.start();ready.wait(5)
    try:
        if error:raise RuntimeError(error)
        result=run_trial(root,device,PROFILE,allow_test_pty=True)
    finally:stop.set();thread.join(5);os.close(master)
    independent=save_review(root,'independent_joint_review_v1.json')
    assert result==independent,'Independent review differs'
    assert result['motion_check_complete'],result.get('analysis_error') or result['gates']
    assert not any(payload),'Receive-only fixture observed unexpected TX payload'
    assert not error,error
    report=dict(scope='SYNTHETIC DDS + VIRTUAL UART; NOT PHYSICAL COMPENSATION ACCEPTANCE',
        production_recorder_used=True,production_orchestrator_used=True,actual_25_second_guidance=True,
        virtual_uart_only_hook='Function-only allow_test_pty hook; same capture_session implementation and judgement',
        motion_check_complete=result['motion_check_complete'],independent_review_identical=result==independent,
        no_serial_transmit_payload_observed=not any(payload),writer_health=result['health'],
        statistics={k:v for k,v in result['statistics'].items() if k not in ['mount_worksheet']},
        original_data_sha256=result['original_data_sha256'],fixture_sha256=sha(Path(__file__)))
    target=root.parent/'e2e_verification.json'
    with target.open('x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
    print('VIRTUAL E2E PASS:',target)
if __name__=='__main__':main()
