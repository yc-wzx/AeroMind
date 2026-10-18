import base64,copy,json,math,os,pty,select,struct,tempfile,threading,time,tty,termios,unittest,sys
from pathlib import Path
import numpy as np
from translation_math import derive_translation,mount,rz,intervals,reference
from translation_review import HERE,review_translation,save_review,native_audit,sha
from serial_reader import FeedbackReader
from test_joint import PARAMS,SOURCES,MEASUREMENTS,PROFILE,frame
from inspect_wire import inspect_bytes
from rclpy.serialization import serialize_message,deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

REF=dict(distance_m=.5,source='Synthetic exact reference, not physical measurement',reference_point='Same fixed chassis marker',direction='initial chassis +X')

def displacement(t):return 0. if t<6 else (t-6)/4 if t<10 else 1. if t<16 else (20-t)/4 if t<20 else 0.

def fixture(axis='x',scale=1.,global_yaw=.7,body_turn=0.,joint_turn=0.):
    origin=10**12;raw=bytearray();reads=[];records=[];poses=[];events=[]
    for phase,(a,b) in zip(['baseline','outward','far_hold','return','final_hold'],[(0,5),(6,10),(10,15),(16,20),(20,25)]):
        events.extend(dict(phase=phase,edge=edge,receive_monotonic_ns=origin+round(t*1e9)) for edge,t in [('start',a),('end',b)])
    _,arm,_=mount(MEASUREMENTS);R0=rz(global_yaw)@np.diag([1.,-1.,-1.]);centre0=rz(global_yaw)@arm
    def add(kind,msg,receive):records.append(dict(sequence=len(records)+1,stream=kind,receive_monotonic_ns=receive,receive_wall_ns=receive+10**18,cdr_base64=base64.b64encode(serialize_message(msg)).decode()))
    for tick in range(-200,5201):
        t=tick*.005;d=displacement(t);delta=np.zeros(3);delta['xyz'.index(axis)]=.5*d*scale
        angle=global_yaw-(body_turn+joint_turn)*d;R=rz(angle);pos=centre0+R0@delta-R@arm;now=origin+tick*5_000_000;stamp=10**18+now
        imu=Imu();imu.header.frame_id='mid360_imu';imu.header.stamp.sec,imu.header.stamp.nanosec=divmod(stamp,10**9);imu.linear_acceleration.z=-9.80665;add('imu_si',imu,now)
        msg=Odometry();msg.header.frame_id='mid360_local_odom';msg.child_frame_id='mid360_imu';msg.header.stamp=imu.header.stamp
        msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=pos.tolist();msg.pose.pose.orientation.z=math.sin(angle/2);msg.pose.pose.orientation.w=math.cos(angle/2)
        add('predicted',msg,now+1000);add('mixed',msg,now+2000)
        if tick%20==0:
            add('corrected',msg,now+3000);poses.append(dict(sequence=len(records),stamp_ns=stamp,receive_monotonic_ns=now+3000,position=pos.tolist(),quaternion=[0.,0.,math.sin(angle/2),math.cos(angle/2)]));add('mixed',msg,now+4000)
            tr=TransformStamped();tr.header=msg.header;tr.child_frame_id=msg.child_frame_id;tr.transform.rotation=msg.pose.pose.orientation
            tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=pos.tolist();tf=TFMessage();tf.transforms=[tr];add('tf',tf,now+5000)
        data=frame(poly=0x8408,angle=.019+joint_turn*d);offset=len(raw);raw.extend(data)
        reads.append(dict(sequence=len(reads)+1,offset=offset,length=39,read_enter_monotonic_ns=now,read_return_monotonic_ns=now+1,receive_wall_ns=now+10**18))
    _,enc=inspect_bytes(bytes(raw),reads,'0x8408');enc=[r for r in enc if r['crc_algorithm']=='diagnostic_8408_ffff']
    return dict(raw=bytes(raw),reads=reads,records=records,poses=poses,encoder=enc,events=events)

def write_fixture(path,data):
    (path/'encoder').mkdir();(path/'native.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in data['records']));(path/'events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in data['events']))
    (path/'reference.json').write_text(json.dumps(REF));(path/'mount_snapshot.json').write_bytes(PROFILE.read_bytes());(path/'variant_manifest_snapshot.json').write_text('{}')
    (path/'input_snapshot').mkdir();(path/'input_snapshot/fixture').write_text('SYNTHETIC INPUT ONLY')
    (path/'local_inputs.json').write_text(json.dumps([dict(snapshot='input_snapshot/fixture',sha256=sha(path/'input_snapshot/fixture'))]))
    (path/'encoder/raw.bin').write_bytes(data['raw']);(path/'encoder/reads.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in data['reads']))
    n=len(data['reads']);b=len(data['raw']);sm=dict(writer_health=dict(closed=True,error=None,overflow=0,received_bytes=b,written_bytes=b,received_chunks=n,written_chunks=n),data_sha256={k:sha(path/'encoder'/k) for k in ['raw.bin','reads.jsonl']},failure=None,interrupted=False,input_tools_unchanged=True,device='/dev/pts/synthetic',baud=460800,expected_crc_profile='0x8408',guided_turn=False)
    (path/'encoder/capture_meta.json').write_text(json.dumps(sm));n=len(data['records'])
    names=['native.jsonl','events.jsonl','reference.json','mount_snapshot.json','variant_manifest_snapshot.json','local_inputs.json','encoder/raw.bin','encoder/reads.jsonl','encoder/capture_meta.json']
    meta=dict(health=dict(closed=True,error=None,overflow=0,received=n,written=n,actual_lines=n),publishers_before=SOURCES,publishers_after=SOURCES,parameters_before=PARAMS,parameters_after=PARAMS,sealed_inputs_before=True,sealed_inputs_after=True,inputs_unchanged=True,failure=None,interrupted=False,data_sha256={k:sha(path/k) for k in names})
    (path/'capture_meta.json').write_text(json.dumps(meta));return meta

class MathTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.f=fixture()
    def stats(self,**kwargs):
        f=fixture(**kwargs) if kwargs else self.f
        return derive_translation(f['poses'],f['encoder'],f['events'],MEASUREMENTS,REF)[0]
    def test_inverted_mount_and_arbitrary_lio_gauge_keep_positive_forward(self):
        s=self.stats();self.assertAlmostEqual(s['estimated_forward_distance_m'],.5,places=7);self.assertLess(s['return_closure_3d_m'],1e-9);self.assertTrue(s['forward_direction_consistent'])
    def test_sideways_is_not_fitted_to_forward(self):
        s=self.stats(axis='y');self.assertAlmostEqual(s['estimated_forward_distance_m'],0.,places=7);self.assertAlmostEqual(s['far_lateral_displacement_m'],.5,places=7)
    def test_wrong_scale_is_reported_without_correction(self):
        s=self.stats(scale=1.2);self.assertAlmostEqual(s['forward_scale_ratio'],1.2,places=7);self.assertAlmostEqual(s['signed_forward_distance_difference_m'],.1,places=7)
    def test_reverse_direction_stays_negative(self):
        s=self.stats(scale=-1);self.assertLess(s['estimated_forward_distance_m'],0);self.assertFalse(s['forward_direction_consistent'])
    def test_body_yaw_not_hidden(self):self.assertAlmostEqual(self.stats(body_turn=.2)['max_sampled_body_yaw_change_deg'],math.degrees(.2),places=4)
    def test_joint_turn_recorded_and_compensated_not_forced_zero(self):
        s=self.stats(joint_turn=.1);self.assertGreater(s['max_sampled_encoder_change_deg'],5);self.assertLess(s['max_sampled_body_yaw_change_deg'],1e-4)
    def test_missing_measurement_or_provenance_rejected(self):
        for d in [None,0,float('nan'),float('inf'),-.5,True]:
            r=dict(REF,distance_m=d)
            with self.assertRaises(ValueError):reference(r)
        r=dict(REF,source='')
        with self.assertRaises(ValueError):reference(r)
    def test_missing_and_short_markers_rejected(self):
        with self.assertRaises(ValueError):intervals(self.f['events'][:-1])
        events=copy.deepcopy(self.f['events']);events[1]['receive_monotonic_ns']=events[0]['receive_monotonic_ns']+10**9
        with self.assertRaises(ValueError):intervals(events)
    def test_bad_pose_and_encoder_gap_rejected(self):
        f=copy.deepcopy(self.f);f['poses'][40]['position'][0]=float('nan')
        with self.assertRaises(ValueError):derive_translation(f['poses'],f['encoder'],f['events'],MEASUREMENTS,REF)
        f=copy.deepcopy(self.f);f['encoder']=[r for r in f['encoder'] if not 10**12+7*10**9<r['host_completion_receive_monotonic_ns']<10**12+8*10**9]
        with self.assertRaises(ValueError):derive_translation(f['poses'],f['encoder'],f['events'],MEASUREMENTS,REF)

class EvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.f=fixture()
    def test_full_native_audit(self):self.assertTrue(native_audit(self.f['records'],self.f['events'])[0]['evidence_complete'])
    def test_actual_independent_review_and_repeated_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);write_fixture(p,self.f);a=save_review(p,'a.json');b=save_review(p,'b.json');self.assertEqual(a,b);self.assertTrue(a['diagnostic_complete']);self.assertFalse(a['accuracy_accepted']);self.assertFalse(a['navigation_ready'])
            with self.assertRaises(FileExistsError):save_review(p,'a.json')
    def test_raw_tampering_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);write_fixture(p,self.f)
            with (p/'native.jsonl').open('a') as f:f.write('\n')
            with self.assertRaises(ValueError):review_translation(p)
    def test_capture_failure_not_passed(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);m=write_fixture(p,self.f);m['failure']='stale';(p/'capture_meta.json').write_text(json.dumps(m));self.assertFalse(review_translation(p)[0]['diagnostic_complete'])
    def test_missing_measurement_hash_or_imu_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);m=write_fixture(p,self.f);m['data_sha256'].pop('reference.json');(p/'capture_meta.json').write_text(json.dumps(m))
            with self.assertRaises(ValueError):review_translation(p)
        rows=[dict(r) for r in self.f['records'] if r['stream']!='imu_si']
        for i,r in enumerate(rows,1):r['sequence']=i
        self.assertFalse(native_audit(rows,self.f['events'])[0]['evidence_complete'])

class SerialTests(unittest.TestCase):
    def test_actual_readonly_uart_and_no_tx(self):
        master,slave=pty.openpty();device=os.ttyname(slave);tty.setraw(slave,termios.TCSANOW);os.close(slave);os.set_blocking(master,False);stop=threading.Event();seen=[]
        def peer():
            while not stop.is_set():
                try:os.write(master,frame())
                except OSError:pass
                if select.select([master],[],[],0)[0]:
                    try:seen.append(os.read(master,4096))
                    except OSError:pass
                time.sleep(.005)
        thread=threading.Thread(target=peer);thread.start()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                r=FeedbackReader(Path(tmp)/'encoder');r.start(device,allow_test_pty=True);time.sleep(.3);self.assertTrue(r.fresh());result=r.close();self.assertTrue(result['capture_complete']);self.assertTrue(result['receive_profile_matches']);self.assertFalse(any(seen))
        finally:stop.set();thread.join(2);os.close(master)
    def test_existing_owner_refused(self):
        master,slave=pty.openpty();device=os.ttyname(slave)
        try:
            with tempfile.TemporaryDirectory() as tmp:
                r=FeedbackReader(Path(tmp)/'encoder')
                with self.assertRaises(RuntimeError):r.start(device,allow_test_pty=True)
                self.assertFalse(r.close('occupied')['capture_complete'])
        finally:os.close(slave);os.close(master)

if __name__=='__main__':unittest.main(verbosity=2)
