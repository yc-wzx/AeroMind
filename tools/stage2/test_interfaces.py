import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/uav_planning/scripts'))
from nav_msgs.msg import Odometry
from stage2_simulated_odometry import extract, SimulatedOdometry
from stage2_odometry_model import ErrorParameters, OdometryErrorModel
from stage2_navigation_interface import Stage2NavigationInterface
from gazebo_navigation_interface import GazeboNavigationInterface

def message(t=1):
    m=Odometry();m.header.stamp.sec=int(t);m.header.stamp.nanosec=round((t-int(t))*1e9)
    m.header.frame_id='odom';m.child_frame_id='base_link';m.pose.pose.orientation.w=1.0
    return m

class InterfaceTests(unittest.TestCase):
    def test_nonfinite_covariance_rejected(self):
        m=message();m.pose.covariance[0]=math.nan
        with self.assertRaises(ValueError):extract(m)

    def test_zero_quaternion_rejected(self):
        m=message();m.pose.pose.orientation.w=0.0
        with self.assertRaises(ValueError):extract(m)

    def adapter(self):
        return types.SimpleNamespace(last_stage2_stamp=None,
            get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=1_000_000_000)),
            get_logger=lambda:types.SimpleNamespace(error=lambda s:None))

    def test_production_adapter_rejects_wrong_frame_stale_future_nan_and_duplicate(self):
        cases=[]
        m=message();m.header.frame_id='map';cases.append(m)
        cases.extend([message(.7),message(1.1)])
        m=message();m.pose.pose.position.x=math.nan;cases.append(m)
        s=self.adapter()
        with patch.object(GazeboNavigationInterface,'gazebo_odom') as production:
            for m in cases:Stage2NavigationInterface.gazebo_odom(s,m)
            self.assertFalse(production.called)
        # Real derived instance without invoking ROS construction; valid path
        # reaches the actual production base callback, not a copied expression.
        node=Stage2NavigationInterface.__new__(Stage2NavigationInterface)
        node.last_stage2_stamp=None
        with patch.object(Stage2NavigationInterface,'get_clock',lambda _:s.get_clock()),patch.object(Stage2NavigationInterface,'get_logger',lambda _:s.get_logger()),patch.object(GazeboNavigationInterface,'gazebo_odom') as production:
            Stage2NavigationInterface.gazebo_odom(node,message())
            Stage2NavigationInterface.gazebo_odom(node,message())
            self.assertEqual(production.call_count,1)

    def test_production_source_rejects_stale_and_no_output_for_gap(self):
        outputs=[];diagnostics=[]
        s=types.SimpleNamespace(model=OdometryErrorModel(ErrorParameters()),max_age_s=.2,
            accepted=0,rejected=0,pub=types.SimpleNamespace(publish=outputs.append),
            diag=types.SimpleNamespace(publish=diagnostics.append),
            get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=1_000_000_000)),
            get_logger=lambda:types.SimpleNamespace(error=lambda t:None))
        SimulatedOdometry.receive(s,message(.7));self.assertEqual(outputs,[])
        SimulatedOdometry.receive(s,message(1));self.assertEqual(len(outputs),1)
        SimulatedOdometry.receive(s,message(1));self.assertEqual(len(outputs),1)
        self.assertEqual(s.rejected,2)
        self.assertEqual(outputs[0].header.frame_id,'odom')


if __name__=='__main__':unittest.main()
