import copy,json,math,tempfile,unittest
from pathlib import Path
import numpy as np
from preview_axis_centre import preview_rows,arm_in_imu,preview
from check_measurements import inspect

def worksheet():
    return json.loads(Path(__file__).with_name('mount_measurements.operator_20261018_v1.json').read_text())

class PreviewTests(unittest.TestCase):
    def test_known_geometry_is_declared_and_not_navigation_ready(self):
        d=worksheet();arm,r=arm_in_imu(d)
        for a,b in zip(arm,[-.036,-.02329,.54712]):self.assertAlmostEqual(a,b)
        self.assertTrue(r['measurement_data_complete']);self.assertFalse(r['navigation_ready'])
        self.assertFalse(r['yaw_feedback_contract_complete']);self.assertIsNone(d['yaw_feedback']['timestamp_basis'])
    def test_fixed_axis_stays_fixed_for_all_yaw_and_wrap_angles(self):
        d=worksheet();p=np.asarray(inspect(d)['geometry_candidate']['axis_ground_to_imu_at_zero_m']);rows=[]
        for a in [-179.,-90.,0.,45.,179.]:
            t=math.radians(a);c,s=math.cos(t),math.sin(t);Rz=np.asarray([[c,-s,0],[s,c,0],[0,0,1]])
            # Rz(yaw) Rx(pi) quaternion in xyzw order.
            rows.append(dict(position=(Rz@p).tolist(),quaternion=[math.cos(t/2),math.sin(t/2),0.,0.]))
        out,_,_=preview_rows(rows,d)
        for row in out:self.assertLess(np.linalg.norm(row['position']),1e-10)
        self.assertGreater(np.linalg.norm(np.asarray(rows[0]['position'])-rows[2]['position']),.05)
    def test_translation_is_preserved(self):
        d=worksheet();p=np.asarray(inspect(d)['geometry_candidate']['axis_ground_to_imu_at_zero_m']);translation=np.asarray([1.,-.5,.1])
        out,_,_=preview_rows([dict(position=(p+translation).tolist(),quaternion=[1.,0.,0.,0.])],d)
        self.assertLess(np.linalg.norm(np.asarray(out[0]['position'])-translation),1e-10)
    def test_nonconcentric_axis_not_approximated(self):
        d=worksheet();d['base_to_yaw_axis_mm']['x']=1.
        with self.assertRaises(ValueError):arm_in_imu(d)
    def test_bad_pose_rejected(self):
        for row in [dict(position=[float('nan'),0,0],quaternion=[0,0,0,1]),dict(position=[0,0,0],quaternion=[0,0,0,0])]:
            with self.assertRaises(ValueError):preview_rows([row],worksheet())

if __name__=='__main__':unittest.main(verbosity=2)
