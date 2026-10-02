"""Mutate temporary copies of a real corrected frame; no ROS publishers."""
import base64
import copy
import contextlib
import io
import json
from pathlib import Path
import struct
import tempfile
import unittest
from rclpy.serialization import deserialize_message,serialize_message
from sensor_msgs.msg import PointCloud2
from audit_all_point_frames import run

ROOT=Path(__file__).resolve().parents[3]
SOURCE=ROOT/'tools/results/stage2_step4_lio_safe_profile_20261002/full_reserved_map_01'
REPLAY=ROOT/'tools/results/stage2_step4_lio_optimization_20261002/instant_cloud_replay_01'


class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        t=24.4
        cls.sensors=[];cls.poses=[]
        for file,topic,renamed in [(SOURCE/'lio_full_01.sensors.jsonl','/lio/lidar','/lio/lidar'),
                (REPLAY/'output.clouds.jsonl','/offline_lio/cloud_registered','/lio/cloud_registered')]:
            for line in file.open():
                row=json.loads(line)
                if row['topic']==topic and round(row['message_stamp_s'],8)==t:
                    row['topic']=renamed;cls.sensors.append(row);break
        for line in (REPLAY/'output.native.jsonl').open():
            row=json.loads(line)
            if row['topic']=='/offline_lio/odometry' and round(row['message_stamp_s'],8)==t:
                row['topic']='/lio/odometry';cls.poses.append(row)
        assert len(cls.sensors)==2 and cls.poses

    def audit(self,change):
        sensors=copy.deepcopy(self.sensors);poses=copy.deepcopy(self.poses)
        change(sensors,poses)
        with tempfile.TemporaryDirectory(prefix='point_evidence_') as tmp:
            root=Path(tmp)
            (root/'lio_full_01.sensors.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in sensors))
            (root/'lio_full_01.native.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in poses))
            with contextlib.redirect_stdout(io.StringIO()):result=run(root,None,root/'audit.json')
        return result['all_points_rigid_consistent'] and result['all_recorded_outputs_checked']

    def mutate_point(self,sensors,poses,value):
        row=next(r for r in sensors if r['topic']=='/lio/cloud_registered')
        msg=deserialize_message(base64.b64decode(row['cdr']),PointCloud2)
        data=bytearray(msg.data);field=next(f for f in msg.fields if f.name=='x')
        x=struct.unpack_from('<f',data,field.offset)[0]
        struct.pack_into('<f',data,field.offset,x+.1 if value=='offset' else float('nan'))
        msg.data=bytes(data);row['cdr']=base64.b64encode(serialize_message(msg)).decode()

    def test_original_real_frame(self):self.assertTrue(self.audit(lambda s,p:None))
    def test_one_corrupted_point(self):self.assertFalse(self.audit(lambda s,p:self.mutate_point(s,p,'offset')))
    def test_nonfinite_output_point(self):self.assertFalse(self.audit(lambda s,p:self.mutate_point(s,p,'nan')))
    def test_missing_input(self):self.assertFalse(self.audit(lambda s,p:s.pop(0)))
    def test_missing_output(self):self.assertFalse(self.audit(lambda s,p:s.pop(1)))
    def test_missing_same_stamp_pose(self):self.assertFalse(self.audit(lambda s,p:p.clear()))


if __name__=='__main__':unittest.main(verbosity=2)
