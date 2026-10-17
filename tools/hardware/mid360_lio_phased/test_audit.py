import base64,copy,unittest
from rclpy.serialization import serialize_message,deserialize_message
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from audit import audit_rows

def fixture():
    rows=[]
    def add(kind,msg):
        n=len(rows)+1
        rows.append(dict(sequence=n,stream=kind,receive_monotonic_ns=10**9+n*10**6,
            receive_wall_ns=10**12+n*10**6,cdr_base64=base64.b64encode(serialize_message(msg)).decode()))
    for cycle in range(5):
        for kind,offset,x in [('predicted',5,1.),('corrected',2,2.),('predicted',10,3.)]:
            msg=Odometry();msg.header.frame_id='mid360_local_odom';msg.child_frame_id='mid360_imu'
            msg.header.stamp.sec=10;msg.header.stamp.nanosec=(cycle*100+offset)*10**6
            msg.pose.pose.position.x=x;msg.pose.pose.orientation.w=1.
            add(kind,msg);add('mixed',msg)
            if kind=='corrected':
                transform=TransformStamped();transform.header=msg.header;transform.child_frame_id=msg.child_frame_id
                transform.transform.translation.x=x;transform.transform.rotation.w=1.
                tf=TFMessage();tf.transforms=[transform];add('tf',tf)
    return rows

class AuditTests(unittest.TestCase):
    def test_split_monotonic_mixed_regressions_preserved(self):
        rows=fixture();result=audit_rows(rows,.01)
        self.assertTrue(result['evidence_complete'])
        self.assertEqual(result['streams']['mixed']['regressed_stamps'],5)
        self.assertEqual(result['streams']['corrected']['regressed_stamps'],0)
        self.assertEqual(result['streams']['predicted']['regressed_stamps'],0)
        self.assertTrue(result['phase_identity']['tf_matches_corrected_pose'])
    def test_missing_internal_mixed_sample_fails(self):
        rows=fixture();del rows[17]
        for i,row in enumerate(rows,1):row['sequence']=i
        self.assertFalse(audit_rows(rows,.01)['evidence_complete'])
    def test_changed_payload_not_hidden_by_timestamp(self):
        rows=fixture();rows[17]['cdr_base64']=rows[3]['cdr_base64']
        self.assertFalse(audit_rows(rows,.01)['evidence_complete'])
    def test_bad_cdr_fails(self):
        rows=fixture();rows[10]['cdr_base64']='not-cdr!'
        self.assertFalse(audit_rows(rows,.01)['evidence_complete'])
    def test_receive_order_rejected(self):
        rows=fixture();rows[5]['receive_monotonic_ns']=1
        with self.assertRaises(ValueError):audit_rows(rows,.01)
    def test_missing_phase_fails(self):
        rows=[r for r in fixture() if r['stream']!='predicted']
        for i,row in enumerate(rows,1):row['sequence']=i
        self.assertFalse(audit_rows(rows,.01)['evidence_complete'])
    def test_tf_mismatch_fails(self):
        rows=fixture();row=next(r for r in rows if r['stream']=='tf' and r['sequence']>10)
        row['cdr_base64']=next(r['cdr_base64'] for r in rows if r['stream']=='tf')
        self.assertFalse(audit_rows(rows,.01)['evidence_complete'])

    def test_nonfinite_decoded_pose_fails(self):
        rows=fixture();msg=deserialize_message(base64.b64decode(rows[16]['cdr_base64']),Odometry)
        msg.pose.pose.position.x=float('nan')
        rows[16]['cdr_base64']=base64.b64encode(serialize_message(msg)).decode()
        self.assertFalse(audit_rows(rows,.01)['evidence_complete'])
    def test_internal_receive_stall_fails(self):
        rows=fixture()
        for row in rows[15:]:row['receive_monotonic_ns']+=300000000
        self.assertFalse(audit_rows(rows,.01)['evidence_complete'])

if __name__=='__main__':unittest.main(verbosity=2)
