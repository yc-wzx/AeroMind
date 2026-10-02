#!/usr/bin/env python3
"""Production relay/runner and evidence rejection tests; never publish ROS commands."""
import copy
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/uav_planning/scripts'))
from stage2_scan_relay import ScanRelay
from run_scan_resume_trial import ScanResumeRunner, Stage2Runner
from audit_scan_resume import loss_audit


class RelayTests(unittest.TestCase):
    def relay(self):
        relay=ScanRelay.__new__(ScanRelay)
        relay.held=False;relay.received=relay.forwarded=relay.dropped=0
        relay.last_received_stamp=relay.last_forwarded_stamp=None
        relay.sent=[];relay.output=NS(publish=relay.sent.append)
        relay.events=[];relay.emit=relay.events.append
        return relay

    def test_exact_object_forwarded(self):
        r=self.relay();m=NS(header=NS(stamp=NS(sec=1,nanosec=2)),ranges=[1,float('inf')])
        r.scan(m);self.assertIs(r.sent[0],m);self.assertEqual((r.received,r.forwarded,r.dropped),(1,1,0))

    def test_pause_discards_resume_never_replays(self):
        r=self.relay();r.hold(NS(data=True),NS())
        old=NS(header=NS(stamp=NS(sec=1,nanosec=0)));r.scan(old)
        r.hold(NS(data=False),NS());self.assertEqual(r.sent,[])
        new=NS(header=NS(stamp=NS(sec=2,nanosec=0)));r.scan(new)
        self.assertEqual(r.sent,[new]);self.assertEqual(r.dropped,1);self.assertEqual(r.events,['hold','resume'])

    def test_multiple_dropped_no_buffer(self):
        r=self.relay();r.held=True
        for i in range(100):r.scan(NS(header=NS(stamp=NS(sec=i,nanosec=0))))
        self.assertEqual(r.sent,[]);self.assertEqual(r.received,r.dropped)
        self.assertNotIn('buffer',r.__dict__)


def synthetic_rows():
    rows=[]
    def add(topic,t,data,stamp=None):
        rows.append(dict(topic=topic,receive_sim_s=t,message_stamp_s=stamp,
                         receive_monotonic_ns=round((100+t)*1e9),data=data))
    def status(event,t,held,received,forwarded,dropped):
        add('/scan_relay/status',t,{'data':json.dumps(dict(event=event,sim_s=t,
          monotonic_ns=round((100+t)*1e9),held=held,received=received,forwarded=forwarded,dropped=dropped,
          last_forwarded_stamp_s=.9,last_received_stamp_s=t))})
    status('hold',1.,True,50,50,0);status('resume',3.46,False,170,50,120)
    for i in range(251):
        t=round(i*.02,8);speed=.1 if t<1.42 or t>=3.52 else 0.
        y=.62 if t<1.42 else .66
        add('/gazebo/odometry',t,dict(pose={'pose':{'position':{'x':4.7,'y':y,'z':0}}},
          twist={'twist':{'linear':{'x':0.,'y':speed,'z':0.},'angular':{'x':0.,'y':0.,'z':0.}}}),t)
        add('/model/omni_robot/cmd_vel',t,dict(linear={'x':speed,'y':0.,'z':0.},angular={'x':0.,'y':0.,'z':0.}))
    for i in range(51):
        t=round(i*.1,8)
        d=dict(header={'frame_id':'omni_robot/base_link/lidar','stamp':{'sec':int(t),'nanosec':round((t-int(t))*1e9)}},
            angle_min=0.,angle_max=1.,angle_increment=1.,range_min=.08,range_max=16.,scan_time=0.,time_increment=0.,ranges=[1.,2.],intensities=[])
        add('/scan',t,d,t)
        if t<1. or t>=3.5:
            add('/guarded_scan',t+.001,copy.deepcopy(d),t)
            add('/localization/diagnostics',t+.01,{'data':json.dumps({'accepted':True,'stamp_s':t})})
    for t in (1.6,2.,3.):add('/ground/planning/actuation_diagnostics',t,{'data':json.dumps({'action':'stale_stop'})})
    return sorted(rows,key=lambda r:r['receive_monotonic_ns'])


class EvidenceTests(unittest.TestCase):
    def test_valid_raw_loss_resume(self):self.assertTrue(loss_audit(synthetic_rows())['pass'])
    def rejected(self,change):
        rows=synthetic_rows();change(rows);self.assertFalse(loss_audit(rows)['pass'])
    def test_nonzero_during_hold_rejected(self):
        def change(rows):
            for r in rows:
                if r['topic']=='/model/omni_robot/cmd_vel' and r['receive_sim_s']==2.:r['data']['linear']['x']=.1
        self.rejected(change)
    def test_replayed_old_scan_rejected(self):
        def change(rows):
            r=next(r for r in rows if r['topic']=='/guarded_scan');new=copy.deepcopy(r)
            new.update(receive_sim_s=3.6,receive_monotonic_ns=103600000000);rows.append(new)
            rows.sort(key=lambda r:r['receive_monotonic_ns'])
        self.rejected(change)
    def test_guarded_payload_change_rejected(self):
        self.rejected(lambda rows:next(r for r in rows if r['topic']=='/guarded_scan')['data']['ranges'].__setitem__(0,9.))
    def test_missing_hold_rejected(self):
        self.rejected(lambda rows:rows.__setitem__(slice(None),[r for r in rows if not (r['topic']=='/scan_relay/status' and '"hold"' in r['data']['data'])]))
    def test_gt_hold_gap_rejected(self):
        self.rejected(lambda rows:rows.__setitem__(slice(None),[r for r in rows if not(r['topic']=='/gazebo/odometry' and 2.<=r['receive_sim_s']<=2.4)]))
    def test_completion_during_hold_rejected(self):
        self.rejected(lambda rows:rows.append(dict(topic='/rosout/relevant',receive_monotonic_ns=102000000000,
                data={'msg':'RMUC waypoint diagnostic completed fake'})))
    def test_no_resumed_motion_rejected(self):
        def change(rows):
            for r in rows:
                if r['topic']=='/model/omni_robot/cmd_vel' and r['receive_sim_s']>=3.5:r['data']['linear']['x']=0.
        self.rejected(change)


class StimulusTests(unittest.TestCase):
    def runner(self):
        r=ScanResumeRunner.__new__(ScanResumeRunner)
        r.active=r.accepted=True;r.phase='armed';r.future=None;r.stable_since=None
        r.final_command=NS(linear=NS(x=.1,y=0.),angular=NS(z=0.))
        import time
        r.final_command_received_steady=time.monotonic()
        r.last_actuation_received_steady=time.monotonic()
        r.goal=(4.7,1.15);r.last_actuation={'waypoint':'R0001:W00','navigation_state':'active'}
        r.notes=[];r.requests=[];r.note=lambda event,**kw:r.notes.append(event)
        r.request=r.requests.append
        return r
    def msg(self,y=.62,t=1.,speed=.1):
        return NS(header=NS(stamp=NS(sec=int(t),nanosec=int((t-int(t))*1e9))),
             pose=NS(pose=NS(position=NS(x=4.7,y=y))),twist=NS(twist=NS(linear=NS(x=0.,y=speed),angular=NS(z=0.))))
    def test_motion_triggers_single_hold(self):
        r=self.runner()
        with patch.object(Stage2Runner,'gt_cb',return_value=None):r.gt_cb(self.msg())
        self.assertEqual(r.requests,[True])
    def test_stopped_does_not_trigger(self):
        r=self.runner()
        with patch.object(Stage2Runner,'gt_cb',return_value=None):r.gt_cb(self.msg(speed=0))
        self.assertEqual(r.requests,[])
    def test_too_late_does_not_trigger(self):
        r=self.runner()
        with patch.object(Stage2Runner,'gt_cb',return_value=None):r.gt_cb(self.msg(y=1.0))
        self.assertEqual(r.requests,[])
    def test_two_second_stop_required_before_resume(self):
        r=self.runner();r.phase='holding';r.hold_ack_sim=1.;r.final_command.linear.x=0.
        with patch.object(Stage2Runner,'gt_cb',return_value=None):
            r.gt_cb(self.msg(t=1.5,speed=0));r.gt_cb(self.msg(t=3.49,speed=0));self.assertEqual(r.requests,[])
            r.gt_cb(self.msg(t=3.51,speed=0))
        self.assertEqual(r.requests,[False])


if __name__=='__main__':unittest.main()
