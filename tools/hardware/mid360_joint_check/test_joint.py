import base64,copy,json,math,struct,sys,tempfile,unittest
from pathlib import Path
import numpy as np
from joint_math import derive,mount,rotation,rz
from review import HERE,inspect_native,review,save_review,sha,params_ok,sources_ok
from rclpy.serialization import serialize_message,deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from inspect_wire import inspect_bytes,crc16

def frame(poly=0x8408,angle=.06366,flags=15):
    data=bytearray(39);data[:2]=b'SP';data[22]=0
    struct.pack_into('<Hfff',data,23,flags,0.,0.,angle)
    struct.pack_into('<H',data,37,crc16(data[:37],poly))
    return bytes(data)

PROFILE=HERE.parent/'mid360_mount_check/mount_measurements.operator_20261018_v1.json'
MEASUREMENTS=json.loads(PROFILE.read_text())
PARAMS={'use_sim_time':False,'common.odometry_output_contract':'scan_corrected_primary_v1','common.visualization_frame':'imu','common.imu_frame':'mid360_imu','common.map_frame':'mid360_local_odom','common.time_sync_en':False,'mapping.extrinsic_T':[-.011,-.02329,.04412],'mapping.extrinsic_R':[1.,0.,0.,0.,1.,0.,0.,0.,1.]}
SOURCES={k:[dict(name='mid360_real_lio',namespace='/hardware/mid360/lio')] for k in ['corrected','predicted','mixed','tf']}
SOURCES['imu_si']=[dict(name='mid360_imu_unit_adapter',namespace='/')]

def waveform(t):
    return (0.,0.) if t<5 else ((t-5)*.16,.16) if t<10 else (.8,0.) if t<15 else ((20-t)*.16,-.16) if t<20 else (0.,0.)

def fixture(zero=.06366,body_motion=False,wrong_sign=False,receipt_delay=0):
    origin=10**12;raw=bytearray();reads=[];rows=[];poses=[]
    phases=[dict(phase=p,start_monotonic_ns=origin+i*5*10**9,end_monotonic_ns=origin+(i+1)*5*10**9) for i,p in enumerate(['baseline','turn_left','turned_hold','return','final_hold'])]
    _,arm,_=mount(MEASUREMENTS)
    def add(kind,msg,receipt):
        rows.append(dict(sequence=len(rows)+1,stream=kind,receive_monotonic_ns=receipt,receive_wall_ns=receipt+10**18,cdr_base64=base64.b64encode(serialize_message(msg)).decode()))
    for tick in range(-100,5101):
        t=tick*.005;theta,rate=waveform(t);receipt=origin+round(t*1e9);stamp=10**18+receipt
        body=.2*max(0,min(1,(t-5)/5)) if body_motion else 0.
        total=theta+body
        R=rz(-total);p=arm-R@arm+np.asarray([.1*max(0,min(1,(t-5)/5)) if body_motion else 0.,0.,0.])
        imu=Imu();imu.header.frame_id='mid360_imu';imu.header.stamp.sec,imu.header.stamp.nanosec=divmod(stamp,10**9)
        imu.linear_acceleration.z=-9.80665;imu.angular_velocity.z=-rate;add('imu_si',imu,receipt)
        msg=Odometry();msg.header.frame_id='mid360_local_odom';msg.child_frame_id='mid360_imu';msg.header.stamp=imu.header.stamp
        msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=p.tolist()
        msg.pose.pose.orientation.z=-math.sin(total/2);msg.pose.pose.orientation.w=math.cos(total/2)
        add('predicted',msg,receipt+1000);add('mixed',msg,receipt+2000)
        if tick%20==0:
            add('corrected',msg,receipt+3000);add('mixed',msg,receipt+4000)
            poses.append(dict(sequence=len(rows)-1,stamp_ns=stamp,receive_monotonic_ns=receipt+3000,position=p.tolist(),quaternion=[0.,0.,-math.sin(total/2),math.cos(total/2)]))
            tf=TFMessage();tr=TransformStamped();tr.header=msg.header;tr.child_frame_id=msg.child_frame_id;tr.transform.rotation=msg.pose.pose.orientation
            tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=p.tolist();tf.transforms=[tr];add('tf',tf,receipt+5000)
        angle=zero+(-theta if wrong_sign else theta);angle=math.atan2(math.sin(angle),math.cos(angle))
        data=frame(poly=0x8408,angle=angle);offset=len(raw);raw.extend(data)
        reads.append(dict(sequence=len(reads)+1,offset=offset,length=39,read_enter_monotonic_ns=receipt+receipt_delay,read_return_monotonic_ns=receipt+receipt_delay+1,receive_wall_ns=receipt+10**18))
    _,enc=inspect_bytes(bytes(raw),reads,'0x8408')
    enc=[r for r in enc if r['crc_algorithm']=='diagnostic_8408_ffff']
    return dict(raw=bytes(raw),reads=reads,records=rows,poses=poses,encoder=enc,phases=phases)

def write_fixture(path,data):
    (path/'encoder').mkdir();(path/'native.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in data['records']))
    (path/'encoder/raw.bin').write_bytes(data['raw']);(path/'encoder/reads.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in data['reads']))
    n=len(data['reads']);b=len(data['raw'])
    sm=dict(writer_health=dict(closed=True,error=None,overflow=0,received_bytes=b,written_bytes=b,received_chunks=n,written_chunks=n),
        data_sha256={name:sha(path/'encoder'/name) for name in ['raw.bin','reads.jsonl']},failure=None,interrupted=False,input_tools_unchanged=True,
        expected_crc_profile='0x8408',guided_turn=True,guided_phases=data['phases'],device='/dev/pts/synthetic',baud=460800)
    (path/'encoder/capture_meta.json').write_text(json.dumps(sm))
    (path/'mount_snapshot.json').write_bytes(PROFILE.read_bytes());(path/'variant_manifest_snapshot.json').write_text('{}')
    (path/'input_snapshot').mkdir();(path/'input_snapshot/fixture.txt').write_text('synthetic fixture, not real input closure')
    (path/'local_inputs.json').write_text(json.dumps([dict(snapshot='input_snapshot/fixture.txt',sha256=sha(path/'input_snapshot/fixture.txt'))]))
    count=len(data['records']);names=['native.jsonl','mount_snapshot.json','variant_manifest_snapshot.json','local_inputs.json','encoder/capture_meta.json','encoder/raw.bin','encoder/reads.jsonl']
    meta=dict(health=dict(closed=True,error=None,overflow=0,received=count,written=count,actual_lines=count),inputs_unchanged=True,sealed_inputs_verified_before=True,sealed_inputs_verified_after=True,
        publishers_before=SOURCES,publishers_after=SOURCES,parameters_before=PARAMS,parameters_after=PARAMS,failure=None,interrupted=False,serial_returncode=0,
        data_sha256={name:sha(path/name) for name in names})
    (path/'capture_meta.json').write_text(json.dumps(meta));return meta

class MathTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.data=fixture()
    def stats(self,**args):
        f=fixture(**args) if args else self.data
        return derive(f['poses'],f['encoder'],f['phases'],MEASUREMENTS)[0]
    def test_inversion_and_live_encoder_remove_only_declared_yaw_motion(self):
        v=self.stats();self.assertLess(v['max_absolute_body_yaw_change_deg'],1e-4);self.assertLess(v['max_sampled_centre_distance_m'],1e-8)
        self.assertGreater(v['hold_windows']['turned_hold']['gimbal_heading_change_median_deg'],45)
        self.assertGreater(v['max_sampled_sensor_distance_m'],.02)
    def test_nonzero_initial_angle_kept_as_candidate(self):
        self.assertAlmostEqual(self.stats()['mechanical_forward_encoder_candidate_rad'],.06366,places=6)
    def test_wrap_across_pi(self):self.assertLess(self.stats(zero=3.)['max_absolute_body_yaw_change_deg'],1e-4)
    def test_actual_chassis_motion_is_not_forced_stationary(self):
        s=self.stats(body_motion=True);self.assertGreater(s['max_absolute_body_yaw_change_deg'],11);self.assertGreater(s['max_sampled_centre_distance_m'],.09)
    def test_wrong_encoder_direction_leaves_large_residual(self):self.assertGreater(self.stats(wrong_sign=True)['max_absolute_body_yaw_change_deg'],90)
    def test_nearest_receipt_pairing_has_no_interpolation(self):
        d=self.data;_,rows=derive(d['poses'],d['encoder'],d['phases'],MEASUREMENTS)
        original={r['offset']:r for r in d['encoder']}
        for r in rows:self.assertEqual(r['encoder_angle_rad'],original[r['encoder_frame_offset']]['gimbal_relative_angle_rad'])
    def test_missing_encoder_interval_rejected(self):
        d=copy.deepcopy(self.data);d['encoder']=[r for r in d['encoder'] if not 10**12+6*10**9<r['host_completion_receive_monotonic_ns']<10**12+7*10**9]
        with self.assertRaises(ValueError):derive(d['poses'],d['encoder'],d['phases'],MEASUREMENTS)
    def test_regressed_receipt_rejected(self):
        d=copy.deepcopy(self.data);d['encoder'][300]['host_completion_receive_monotonic_ns']-=10**9
        with self.assertRaises(ValueError):derive(d['poses'],d['encoder'],d['phases'],MEASUREMENTS)
    def test_nonfinite_pose_or_invalid_angle_rejected(self):
        d=copy.deepcopy(self.data);d['poses'][30]['position'][0]=float('nan')
        with self.assertRaises(ValueError):derive(d['poses'],d['encoder'],d['phases'],MEASUREMENTS)
        d=copy.deepcopy(self.data);d['encoder'][400]['relative_yaw_valid']=False
        with self.assertRaises(ValueError):derive(d['poses'],d['encoder'],d['phases'],MEASUREMENTS)
    def test_nonnormalized_quaternion_and_unknown_mount_rejected(self):
        with self.assertRaises(ValueError):rotation([0.,0.,0.,2.])
        m=copy.deepcopy(MEASUREMENTS);m['base_to_yaw_axis_mm']['x']=1.
        with self.assertRaises(ValueError):mount(m)
        m['base_to_yaw_axis_mm']['x']=None
        with self.assertRaises(ValueError):mount(m)

class EvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.data=fixture()
    def test_native_corrected_predicted_mixed_tf_and_imu(self):
        v,rows=inspect_native(self.data['records'],self.data['phases']);self.assertTrue(v['evidence_complete']);self.assertGreater(len(rows),200)
    def test_missing_imu_and_missing_phase_rejected(self):
        rows=[dict(r) for r in self.data['records'] if r['stream']!='imu_si']
        for i,r in enumerate(rows,1):r['sequence']=i
        v,_=inspect_native(rows,self.data['phases']);self.assertFalse(v['evidence_complete'])
        with self.assertRaises(ValueError):inspect_native(self.data['records'],self.data['phases'][:-1])
    def test_original_nan_imu_rejected(self):
        rows=list(self.data['records']);r=dict(rows[0]);msg=deserialize_message(base64.b64decode(r['cdr_base64']),Imu);msg.angular_velocity.z=float('nan')
        r['cdr_base64']=base64.b64encode(serialize_message(msg)).decode();rows[0]=r
        with self.assertRaises(ValueError):inspect_native(rows,self.data['phases'])
    def test_source_and_parameter_fail_closed(self):
        self.assertTrue(params_ok(PARAMS));self.assertTrue(sources_ok(SOURCES))
        p=dict(PARAMS,use_sim_time=True);self.assertFalse(params_ok(p))
        s=copy.deepcopy(SOURCES);s['tf']=[];self.assertFalse(sources_ok(s))
    def test_closed_evidence_independently_recomputes(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);write_fixture(p,self.data);v,_=review(p)
            self.assertTrue(v['motion_check_complete']);self.assertFalse(v['navigation_ready']);self.assertFalse(v['compensation_accuracy_accepted'])
            a=save_review(p,'a.json');b=save_review(p,'b.json');self.assertEqual(a,b)
            with self.assertRaises(FileExistsError):save_review(p,'a.json')
    def test_tampered_native_rejected_by_hash(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);write_fixture(p,self.data)
            with (p/'native.jsonl').open('a') as f:f.write('\n')
            with self.assertRaises(ValueError):review(p)
    def test_capture_failure_cannot_pass(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);meta=write_fixture(p,self.data);meta['failure']='input stale';(p/'capture_meta.json').write_text(json.dumps(meta))
            self.assertFalse(review(p)[0]['motion_check_complete'])
    def test_serial_wrong_crc_or_missing_angle_cannot_pass(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);meta=write_fixture(p,self.data);sm=json.loads((p/'encoder/capture_meta.json').read_text());sm['expected_crc_profile']='0xa001'
            (p/'encoder/capture_meta.json').write_text(json.dumps(sm));meta['data_sha256']['encoder/capture_meta.json']=sha(p/'encoder/capture_meta.json')
            (p/'capture_meta.json').write_text(json.dumps(meta));self.assertFalse(review(p)[0]['evidence_complete'])

if __name__=='__main__':unittest.main(verbosity=2)
