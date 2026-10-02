import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/uav_planning/scripts'))
from stage2_odometry_model import ErrorParameters, OdometryErrorModel
from audit_trial import model_steps, odometry_audit

def data(pose,velocity=(.1,0,0)):
    import math
    return {'header':{'frame_id':'odom'},'child_frame_id':'base_link',
       'pose':{'pose':{'position':{'x':pose[0],'y':pose[1],'z':0},
                      'orientation':{'x':0,'y':0,'z':math.sin(pose[2]/2),'w':math.cos(pose[2]/2)}}},
       'twist':{'twist':{'linear':{'x':velocity[0],'y':velocity[1],'z':0},
                         'angular':{'x':0,'y':0,'z':velocity[2]}}}}

def evidence(parameters):
    m=OdometryErrorModel(parameters);pairs=[];diagnostics=[]
    for i in range(40):
        t=round(i*.02,8);gt=(4.7+i*.002,.5,0)
        nav,_=m.advance(t,gt,(.1,0,0))
        pairs.append((t,data(gt),data(nav)))
        diagnostics.append({'accepted':True,'stamp_s':t,'accepted_count':i+1,'rejected_count':0})
    return pairs,diagnostics

class AuditTests(unittest.TestCase):
    def test_independent_recurrence_for_each_profile(self):
        for p in (ErrorParameters(),ErrorParameters(scale_x=1.005,scale_y=.995),
                  ErrorParameters(yaw_bias_rad_s=.0005),
                  ErrorParameters(position_walk_m_sqrt_s=.002,yaw_walk_rad_sqrt_s=.0003)):
            pairs,diag=evidence(p)
            self.assertTrue(model_steps(pairs,diag,p.__dict__)['pass'])
            self.assertTrue(model_steps(pairs[10:],diag[10:],p.__dict__)['pass'])

    def test_truth_reanchor_cannot_pass_scale_profile(self):
        p=ErrorParameters(scale_x=1.005);pairs,diag=evidence(p)
        pairs=copy.deepcopy(pairs)
        pairs[20][2]['pose']=copy.deepcopy(pairs[20][1]['pose'])
        self.assertFalse(model_steps(pairs,diag,p.__dict__)['pass'])

    def fixture(self,directory,mutate=None):
        p=ErrorParameters();pairs,diag=evidence(p);rows=[]
        for i,((t,gt,nav),d) in enumerate(zip(pairs,diag)):
            for topic,msg in (('/gazebo/odometry',gt),('/simulation/navigation_odometry',nav),
                              ('/ground/odometry',nav),('/simulation/odometry_diagnostics',{'data':json.dumps(d)})):
                rows.append({'topic':topic,'receive_sequence':i+1,
                             'message_stamp_s':t if topic!='/simulation/odometry_diagnostics' else None,
                             'receive_monotonic_ns':int(1e9+t*1e9),'data':copy.deepcopy(msg)})
        if mutate:mutate(rows)
        (directory/'t.native.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
        (directory/'t.summary.json').write_text(json.dumps({'goal_delivery':{'publish_monotonic_ns':0}}))
        (directory/'runtime_parameters').mkdir()
        (directory/'runtime_parameters/stage2_simulated_odometry.yaml').write_text(yaml.safe_dump({'stage2_simulated_odometry':{'ros__parameters':p.__dict__}}))
        return odometry_audit(directory,'t',True)

    def test_raw_identity_fixture_passes(self):
        with tempfile.TemporaryDirectory() as d:self.assertTrue(self.fixture(Path(d))['pass'])

    def test_injected_nan_rejected(self):
        def mutate(rows):
            next(r for r in rows if r['topic']=='/simulation/navigation_odometry')['data']['pose']['pose']['position']['x']=float('nan')
        with tempfile.TemporaryDirectory() as d:self.assertFalse(self.fixture(Path(d),mutate)['pass'])

    def test_missing_raw_truth_pair_rejected(self):
        def mutate(rows):rows.remove(next(r for r in rows if r['topic']=='/gazebo/odometry'))
        with tempfile.TemporaryDirectory() as d:self.assertFalse(self.fixture(Path(d),mutate)['pass'])

    def test_receive_gap_rejected(self):
        def mutate(rows):
            for r in rows:
                if r['receive_sequence']>=20 and r['topic']=='/simulation/navigation_odometry':r['receive_monotonic_ns']+=300000000
        with tempfile.TemporaryDirectory() as d:self.assertFalse(self.fixture(Path(d),mutate)['pass'])

if __name__=='__main__':unittest.main()
