import copy,json,math,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from nav_msgs.msg import Odometry
from static_check import pose_row,drift_stats,Writer
from inspect_raw import inspect,POINTS,IMU_RAW,IMU_SI

def odom():
    m=Odometry();m.header.frame_id='mid360_local_odom';m.child_frame_id='mid360_imu';m.pose.pose.orientation.w=1.
    return m

class Tests(unittest.TestCase):
    def test_pose_frames_and_nonfinite(self):
        pose_row(odom())
        for item in ['frame','quaternion','position','covariance']:
            m=odom()
            if item=='frame':m.child_frame_id='base_link'
            elif item=='quaternion':m.pose.pose.orientation.w=0.
            elif item=='position':m.pose.pose.position.x=float('nan')
            else:m.pose.covariance[0]=float('inf')
            with self.assertRaises(ValueError):pose_row(m)
    def test_drift_rotation_sign_and_translation(self):
        rows=[dict(position=[0,0,0],quaternion=[0,0,0,1]),dict(position=[.03,.04,0],quaternion=[0,0,0,-1])]
        v=drift_stats(rows);self.assertAlmostEqual(v['max_translation_from_first_m'],.05);self.assertAlmostEqual(v['max_rotation_from_first_deg'],0)
        rows[-1]['quaternion']=[0,0,math.sin(math.pi/4),math.cos(math.pi/4)]
        self.assertAlmostEqual(drift_stats(rows)['max_rotation_from_first_deg'],90)
        self.assertEqual(drift_stats([])['samples'],0)
    def test_raw_unmatched_boundary_and_inside_classification(self):
        with tempfile.TemporaryDirectory() as path:
            root=Path(path);rows=[]
            for topic,times in [(POINTS,range(1,6)),(IMU_RAW,range(1,6)),(IMU_SI,[2,4])]:
                rows.extend(dict(topic=topic,stamp_ns=t*10**9,receive_wall_ns=(t+100)*10**9,error=None) for t in times)
            (root/'receipts.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            report=dict(streams={t:dict(count=sum(r['topic']==t for r in rows)) for t in [POINTS,IMU_RAW,IMU_SI]})
            with patch('inspect_raw.verify_capture',return_value=report):v=inspect(root/'summary.json')
            self.assertEqual(v['raw_imu_without_si_in_capture'],3)
            self.assertEqual((v['unmatched_before_si_window'],v['unmatched_inside_si_window'],v['unmatched_after_si_window']),(1,1,1))
            self.assertEqual(v['timestamp_epochs'][IMU_RAW]['wall_minus_device_s_median'],100)
    def test_writer_counts_and_raw_fields_preserved(self):
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/'odom.jsonl';w=Writer(path);rows=[dict(index=i,value=.1*i) for i in range(100)]
            for row in rows:w.put(row)
            health=w.close();self.assertEqual(health['received'],health['written']);self.assertEqual(health['actual_lines'],100)
            self.assertEqual([json.loads(s) for s in path.read_text().splitlines()],rows)

if __name__=='__main__':unittest.main(verbosity=2)
