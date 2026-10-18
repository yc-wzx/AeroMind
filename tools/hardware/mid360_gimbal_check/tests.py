import base64,copy,hashlib,json,math,tempfile,unittest
from pathlib import Path
import numpy as np
from rclpy.serialization import serialize_message,deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from analyze import analyze_records,analyze,qmean,rotation_vector_degrees,PHASES,CONTRACT


def fixture(moving=True):
    rows=[];events=[];base=10_000_000_000
    intervals=[(0,5),(5.001,8.001),(8.002,13.002),(13.003,16.003),(16.004,21.004)]
    for phase,(start,end) in zip(PHASES,intervals):
        events.extend(dict(phase=phase,edge=edge,receive_monotonic_ns=base+round(t*10**9),receive_wall_ns=10**12+round(t*10**9)) for edge,t in [('start',start),('end',end)])
    def add(kind,msg,receipt):
        rows.append(dict(sequence=len(rows)+1,stream=kind,receive_monotonic_ns=receipt,receive_wall_ns=10**12+receipt,
                         cdr_base64=base64.b64encode(serialize_message(msg)).decode()))
    for tick in range(-200,4401):
        t=tick*.005;receipt=base+tick*5_000_000;stamp=10**12+receipt
        if t<5.001:theta,rate=0.,0.
        elif t<8.001:theta,rate=(t-5.001)*math.pi/12,math.pi/12
        elif t<13.003:theta,rate=math.pi/4,0.
        elif t<16.003:theta,rate=(16.003-t)*math.pi/12,-math.pi/12
        else:theta,rate=0.,0.
        if not moving:theta=rate=0.
        imu=Imu();imu.header.frame_id='mid360_imu';imu.header.stamp.sec,imu.header.stamp.nanosec=divmod(stamp,10**9)
        imu.linear_acceleration.z=-9.80665;imu.angular_velocity.z=-rate;add('imu_si',imu,receipt)
        msg=Odometry();msg.header.frame_id='mid360_local_odom';msg.child_frame_id='mid360_imu'
        msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(stamp,10**9)
        msg.pose.pose.position.x=.1*math.cos(theta);msg.pose.pose.position.y=-.1*math.sin(theta)
        msg.pose.pose.orientation.z=-math.sin(theta/2);msg.pose.pose.orientation.w=math.cos(theta/2)
        if tick%4==0:add('predicted',msg,receipt+1000);add('mixed',msg,receipt+2000)
        if tick%20==0:
            msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(stamp-2_000_000,10**9)
            add('corrected',msg,receipt+3000);add('mixed',msg,receipt+4000)
            tf=TFMessage();transform=TransformStamped();transform.header=msg.header;transform.child_frame_id=msg.child_frame_id
            transform.transform.translation.x=msg.pose.pose.position.x;transform.transform.translation.y=msg.pose.pose.position.y
            transform.transform.rotation=msg.pose.pose.orientation;tf.transforms=[transform];add('tf',tf,receipt+5000)
    return rows,events


class GimbalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.rows,cls.events=fixture()
    def test_inverted_frame_and_lever_arm_not_false_drift(self):
        v=analyze_records(self.rows,self.events)
        self.assertTrue(v['evidence_complete']);self.assertTrue(v['rotation_exercised']);self.assertTrue(v['return_exercised'])
        self.assertTrue(v['direction_consistent_with_operator_declared_ccw_then_cw'])
        self.assertAlmostEqual(v['outward_about_estimated_up_deg'],45,places=4)
        self.assertLess(v['outward_rotation_vector_in_initial_sensor_deg'][2],0)
        self.assertAlmostEqual(v['outward_sensor_displacement_m'],2*.1*math.sin(math.pi/8),places=5)
        self.assertLess(v['final_sensor_position_closure_m'],1e-8)
        self.assertNotIn('corrected_drift',v['lio_output_audit'])
    def test_quaternion_sign_and_wrapped_relative_angle(self):
        self.assertAlmostEqual(abs(qmean([[0,0,0,1],[0,0,0,-1]])[3]),1.)
        q=lambda d:[0,0,math.sin(math.radians(d)/2),math.cos(math.radians(d)/2)]
        self.assertAlmostEqual(rotation_vector_degrees(q(179),q(-179))[2],2.,places=5)
    def test_incomplete_operator_markers_rejected(self):
        with self.assertRaises(ValueError):analyze_records(self.rows,self.events[:-1])
    def test_missing_imu_phase_rejected(self):
        rows=[dict(r) for r in self.rows if r['stream']!='imu_si']
        for i,r in enumerate(rows,1):r['sequence']=i
        with self.assertRaises(ValueError):analyze_records(rows,self.events)
    def test_nan_imu_rejected(self):
        rows=list(self.rows);index=next(i for i,r in enumerate(rows) if r['stream']=='imu_si')
        row=dict(rows[index]);msg=deserialize_message(base64.b64decode(row['cdr_base64']),Imu);msg.angular_velocity.x=float('nan')
        row['cdr_base64']=base64.b64encode(serialize_message(msg)).decode();rows[index]=row
        with self.assertRaises(ValueError):analyze_records(rows,self.events)
    def test_no_motion_cannot_be_called_exercised(self):
        rows,events=fixture(moving=False);v=analyze_records(rows,events)
        self.assertTrue(v['evidence_complete']);self.assertFalse(v['rotation_exercised']);self.assertFalse(v['return_exercised'])
    def save(self,path):
        (path/'native.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in self.rows))
        (path/'events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in self.events))
        (path/'variant_manifest_snapshot.json').write_text('{}\n')
        sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
        publishers={k:[dict(name='mid360_real_lio',namespace='/hardware/mid360/lio')] for k in ['corrected','predicted','mixed']}
        publishers['imu_si']=[dict(name='mid360_imu_unit_adapter',namespace='/')]
        meta=dict(data_sha256={n:sha(path/n) for n in ['native.jsonl','events.jsonl']},variant_manifest_sha256=sha(path/'variant_manifest_snapshot.json'),
            health=dict(received=len(self.rows),written=len(self.rows),actual_lines=len(self.rows),closed=True,error=None,overflow=0),
            publishers=publishers,actual_parameters={'common.odometry_output_contract':CONTRACT,'use_sim_time':False,'common.visualization_frame':'imu'},
            inputs_unchanged=True,parameters_unchanged=True,interrupted=False,failure=None)
        (path/'capture_meta.json').write_text(json.dumps(meta));return meta
    def test_independent_file_audit_and_tamper_rejection(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder);self.save(path);v=analyze(path)
            self.assertTrue(v['motion_check_complete'])
            with (path/'events.jsonl').open('a') as f:f.write('\n')
            with self.assertRaises(ValueError):analyze(path)
    def test_capture_failure_cannot_be_hidden_on_reaudit(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder);meta=self.save(path);meta['failure']='parameter request failed'
            (path/'capture_meta.json').write_text(json.dumps(meta))
            self.assertFalse(analyze(path)['evidence_complete'])

class RecorderReadinessTests(unittest.TestCase):
    def test_actual_subscription_qos_matches_real_adapter(self):
        from record import subscription_qos
        from rclpy.qos import qos_profile_sensor_data,qos_check_compatible,QoSCompatibility,ReliabilityPolicy
        imu=subscription_qos('imu_si')
        self.assertEqual(imu.reliability,ReliabilityPolicy.BEST_EFFORT)
        # SYSTEM_DEFAULT liveliness may yield WARNING; actual DDS matching is covered by E2E.
        self.assertNotEqual(qos_check_compatible(qos_profile_sensor_data,imu)[0],QoSCompatibility.ERROR)
        for kind in ['corrected','predicted','mixed','tf']:
            self.assertEqual(subscription_qos(kind).reliability,ReliabilityPolicy.RELIABLE)
        self.assertEqual(imu.depth,1000)
    def recorder(self):
        import threading,time
        from record import Recorder,STREAMS
        result=object.__new__(Recorder);result.lock=threading.Lock()
        result.counts={k:30 for k in STREAMS};result.latest={}
        rows,_=fixture()
        for row in rows:
            result.latest[row['stream']]=(time.monotonic_ns(),base64.b64decode(row['cdr_base64']))
        return result
    def test_missing_imu_still_not_ready(self):
        r=self.recorder();self.assertTrue(r.fresh());r.counts['imu_si']=0
        self.assertFalse(r.fresh())
    def test_stale_imu_still_not_ready(self):
        import time
        r=self.recorder();_,payload=r.latest['imu_si'];r.latest['imu_si']=(time.monotonic_ns()-200_000_000,payload)
        self.assertFalse(r.fresh())
    def test_nonfinite_imu_still_rejected(self):
        r=self.recorder();receipt,payload=r.latest['imu_si'];msg=deserialize_message(payload,Imu)
        msg.linear_acceleration.x=float('nan');r.latest['imu_si']=(receipt,serialize_message(msg))
        with self.assertRaises(ValueError):r.fresh()

if __name__=='__main__':unittest.main(verbosity=2)
