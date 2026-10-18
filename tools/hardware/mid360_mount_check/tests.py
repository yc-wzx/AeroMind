import copy,json,unittest
from pathlib import Path
from check_measurements import inspect,vector_rotated_for_inverted_mount

def measured():
    d=json.loads(Path(__file__).with_name('mount_measurements.template.json').read_text())
    d['base_to_yaw_axis_mm']=dict(x=60.,y=-10.,source='SYNTHETIC TEST ONLY')
    d['yaw_axis_to_mount_face_at_zero_mm']=dict(x=30.,y=20.,height_above_ground=600.,source='SYNTHETIC TEST ONLY')
    d['native_lidar_x_yaw_from_gimbal_forward_deg']=dict(value=0.,source='SYNTHETIC TEST ONLY')
    d['mount_declaration'].update(native_z_points_down=True,source='SYNTHETIC TEST ONLY')
    return d

class Tests(unittest.TestCase):
    def test_blank_does_not_default_zero(self):
        r=inspect(json.loads(Path(__file__).with_name('mount_measurements.template.json').read_text()))
        self.assertFalse(r['measurement_data_complete']);self.assertIsNone(r['geometry_candidate'])
    def test_inverted_nominal_offsets_are_not_double_flipped(self):
        r=inspect(measured());g=r['geometry_candidate'];self.assertTrue(r['measurement_data_complete'])
        for a,b in zip(g['axis_ground_to_imu_at_zero_m'],[.041,-.00329,.59712]):self.assertAlmostEqual(a,b)
        self.assertAlmostEqual(g['axis_ground_to_lidar_at_zero_m'][2],.553)
        self.assertFalse(r['navigation_ready']);self.assertFalse(r['ros_publication'])
    def test_reverse_native_x_preserves_right_handed_rotation(self):
        for a,b in zip(vector_rotated_for_inverted_mount([11.,23.29,2.88],180.),[-11.,23.29,-2.88]):self.assertAlmostEqual(a,b)
    def test_nan_bool_and_unit_rejected(self):
        for value in [float('nan'),float('inf'),True]:
            d=measured();d['base_to_yaw_axis_mm']['x']=value;self.assertFalse(inspect(d)['measurement_data_complete'])
        d=measured();d['units']='m';self.assertFalse(inspect(d)['measurement_data_complete'])
    def test_zero_requires_explicit_provenance(self):
        d=measured();d['base_to_yaw_axis_mm']=dict(x=0.,y=0.,source=None)
        self.assertFalse(inspect(d)['measurement_data_complete'])
        d['base_to_yaw_axis_mm']['source']='Measured concentric';self.assertTrue(inspect(d)['measurement_data_complete'])
    def test_low_inverted_height_is_reported_not_accepted(self):
        d=measured();d['yaw_axis_to_mount_face_at_zero_mm']['height_above_ground']=400.
        r=inspect(d);self.assertEqual(r['installation_height_requirement'],'NOT_MET_BY_DECLARED_MEASUREMENT');self.assertFalse(r['navigation_ready'])
    def test_feedback_declaration_not_live_qualification(self):
        d=measured();d['yaw_feedback']['available']=True;r=inspect(d);self.assertFalse(r['yaw_feedback_contract_complete'])
        d['yaw_feedback'].update(interface='SYNTHETIC',unit='rad',positive_direction='CCW',zero_reference='mark',timestamp_basis='device timestamp')
        r=inspect(d);self.assertTrue(r['yaw_feedback_contract_complete']);self.assertFalse(r['live_feedback_verified']);self.assertFalse(r['navigation_ready'])

if __name__=='__main__':unittest.main(verbosity=2)
