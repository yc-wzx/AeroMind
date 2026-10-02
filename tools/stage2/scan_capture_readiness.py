"""Test-only receive evidence gate. It never publishes or modifies ROS data."""
import json
import math
from types import SimpleNamespace
from audit_localized import finite_tree
from stage2_wall_localization import validate_scan_metadata, SCAN_FRAME

SENSORS={'/clock':'gazebo_bridge','/gazebo/odometry':'gazebo_bridge',
    '/simulation/navigation_odometry':'stage2_simulated_odometry',
    '/scan':'gazebo_bridge','/guarded_scan':'stage2_scan_relay'}
NAVIGATION={'/ground/odometry':'gazebo_navigation_interface',
    '/model/omni_robot/cmd_vel':'gazebo_navigation_interface'}


class CaptureReadiness:
    def __init__(self):
        self.latest={};self.history={};self.accepted=[];self.faults=[]
        self.since={};self.last_report={}

    def observe(self,topic,data,receive_ns,stamp):
        if topic not in SENSORS and topic not in NAVIGATION and topic!='/localization/diagnostics':return
        try:
            if topic in ('/scan','/guarded_scan'):
                validate_scan_metadata(SimpleNamespace(**data))
                if data['header']['frame_id']!=SCAN_FRAME or any(math.isnan(v) for v in data['ranges']):
                    raise ValueError('Invalid scan frame or NaN beam')
            else:
                payload=json.loads(data['data']) if topic=='/localization/diagnostics' else data
                if not finite_tree(payload):raise ValueError('Nonfinite critical capture data')
            if topic=='/localization/diagnostics':
                if payload.get('accepted') is True:self.accepted.append(dict(data=payload,receive_ns=receive_ns))
            else:
                self.latest[topic]=dict(data=data,receive_ns=receive_ns,stamp=stamp)
                if topic in ('/scan','/guarded_scan','/simulation/navigation_odometry'):
                    if stamp is None or not math.isfinite(stamp):raise ValueError('Missing source stamp')
                    hist=self.history.setdefault(topic,{})
                    if stamp in hist:raise ValueError('Duplicate startup sensor stamp')
                    hist[stamp]=data
                    if len(hist)>4000:raise ValueError('Startup capture history bound exceeded')
        except (KeyError,TypeError,ValueError) as error:
            self.faults.append(topic+': '+str(error))

    def check(self,phase,now_ns,graph,writer_ok=True):
        required=dict(SENSORS)
        if phase=='goal':required.update(NAVIGATION)
        checks={'valid_writer':writer_ok,'finite_valid_observations':not self.faults}
        for topic,name in required.items():
            row=self.latest.get(topic)
            age=.5 if topic=='/ground/odometry' else .2
            checks['fresh:'+topic]=row is not None and 0 <= (now_ns-row['receive_ns'])*1e-9 <= age
            ep=graph.get(topic,[])
            checks['publisher:'+topic]=len(ep)==1 and ep[0].get('name')==name and any(ep[0].get('gid',[])) and any(ep[0].get('receiver_gid',[])) and ep[0].get('compatible') is True
        original=self.history.get('/scan',{});guarded=self.history.get('/guarded_scan',{})
        pairs=sorted(set(original)&set(guarded))
        recent=pairs[-3:]
        checks['three_identical_forwarded_scans']=len(recent)==3 and recent[-1]-recent[0]>=.2-1e-8 and all(original[t]==guarded[t] for t in recent)
        if phase=='goal':
            raw=self.history.get('/simulation/navigation_odometry',{})
            matches=[]
            for item in self.accepted:
                d=item['data'];t=d['stamp_s'];ot=d['associated_odom_stamp_s']
                matches.append(t in original and t in guarded and ot in raw and abs(t-ot)<=.025 and original[t]==guarded[t])
            checks['all_accepted_diagnostics_have_recorded_inputs']=len(matches)>=3 and all(matches)
            latest=self.accepted[-1] if self.accepted else None
            checks['fresh_accepted_localization']=latest is not None and 0 <= (now_ns-latest['receive_ns'])*1e-9 <= .2
        ready=all(checks.values())
        if not ready:self.since.pop(phase,None)
        else:self.since.setdefault(phase,now_ns)
        stable=(now_ns-self.since.get(phase,now_ns))*1e-9
        report=dict(ready=ready and stable>=.5,checks=checks,stable_s=stable,phase=phase,
            monotonic_ns=now_ns,graph=graph,paired_scan_stamps_s=recent,
            accepted_diagnostics_seen=len(self.accepted),faults=list(self.faults),
            scope='Receive evidence and actual publisher endpoints; not proof of publisher loss absence.')
        self.last_report[phase]=report
        return report
