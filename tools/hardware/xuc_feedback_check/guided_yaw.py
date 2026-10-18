"""Receive-only operator guidance and independent encoder turn/return analysis."""
import math,statistics
from inspect_wire import crc16,decode,ALGORITHMS,PROFILES
PHASES=['baseline','turn_left','turned_hold','return','final_hold']
PROMPTS=['原位静止5秒；底盘全程不动。',
    '现在单向向左慢转约45度，用约5秒；不要来回调整。',
    '现在停止，保持转后位置静止5秒。',
    '现在单向慢转回原位，用约5秒。',
    '现在停止，回原位静止5秒。']

class GuidedYaw:
    def __init__(self,profile):
        self.poly=ALGORITHMS[PROFILES[profile]];self.buffer=bytearray();self.count=0;self.last=None;self.invalid=False;self.phases=[];self.origin=None
    def start(self,now):
        self.origin=now;self.phases=[dict(phase=PHASES[0],start_monotonic_ns=now,end_monotonic_ns=None)]
        print(PROMPTS[0],flush=True)
    def feed(self,payload,received):
        self.buffer.extend(payload)
        while True:
            p=self.buffer.find(b'SP')
            if p<0:
                self.buffer[:]=b'S' if self.buffer[-1:]==b'S' else b'';return
            if p:del self.buffer[:p]
            if len(self.buffer)<39:return
            frame=bytes(self.buffer[:39])
            if crc16(frame[:-2],self.poly)!=int.from_bytes(frame[-2:],'little'):
                del self.buffer[0];continue
            del self.buffer[:39];fields=decode(frame)
            self.invalid=not (fields['relative_yaw_valid'] and fields['structural_fields_valid'])
            if not self.invalid:self.count+=1;self.last=received
    def tick(self,now):
        index=min(4,int((now-self.origin)//5_000_000_000))
        if index>=1 and (self.invalid or self.count<3 or self.last is None or not 0<=now-self.last<=200_000_000):
            raise RuntimeError('Fresh valid navigation yaw unavailable; stop manual turning')
        if index>=len(self.phases):
            if index!=len(self.phases):raise RuntimeError('Guidance loop stalled across a phase')
            self.phases[-1]['end_monotonic_ns']=now
            self.phases.append(dict(phase=PHASES[index],start_monotonic_ns=now,end_monotonic_ns=None))
            print(PROMPTS[index],flush=True)
    def finish(self,now):
        if self.phases:self.phases[-1]['end_monotonic_ns']=now

def summarize(frames,profile,phases,capture_complete,profile_matches):
    if not capture_complete or not profile_matches:raise ValueError('Capture incomplete or expected CRC not observed')
    if [p.get('phase') for p in phases]!=PHASES:raise ValueError('Incomplete guidance phases')
    previous=0
    for p in phases:
        a,b=p['start_monotonic_ns'],p['end_monotonic_ns']
        if not isinstance(a,int) or not isinstance(b,int) or a<previous or b<=a or (b-a)*1e-9<4.9:raise ValueError('Invalid/short phase coverage')
        previous=b
    start=phases[0]['start_monotonic_ns'];end=phases[-1]['end_monotonic_ns']
    selected=sorted([dict(f) for f in frames if f['crc_algorithm']==PROFILES[profile]],key=lambda r:r['offset'])
    if len(selected)<3:raise ValueError('Missing encoder frames')
    previous_angle=None;unwrapped=None;previous_receive=0
    for row in selected:
        t=row['host_completion_receive_monotonic_ns']
        if t<previous_receive:raise ValueError('Encoder receive clock regressed')
        previous_receive=t
        valid=row['relative_yaw_valid'] and row['structural_fields_valid']
        if start<=t<=end and not valid:raise ValueError('Invalid angle/flags during guided phases')
        if not valid:continue
        angle=row['gimbal_relative_angle_rad']
        if not math.isfinite(angle):raise ValueError('Nonfinite encoder angle')
        unwrapped=angle if previous_angle is None else unwrapped+math.atan2(math.sin(angle-previous_angle),math.cos(angle-previous_angle))
        row['unwrapped_rad']=unwrapped;previous_angle=angle
    mission=[r for r in selected if start<=r['host_completion_receive_monotonic_ns']<=end and 'unwrapped_rad' in r]
    times=[r['host_completion_receive_monotonic_ns'] for r in mission]
    if len(times)<3 or times[0]-start>200_000_000 or end-times[-1]>200_000_000 or max((b-a for a,b in zip(times,times[1:])),default=10**12)>200_000_000:
        raise ValueError('Encoder receive coverage/gap exceeds 0.2 seconds')
    holds={};motion={}
    for p in phases:
        a,b=p['start_monotonic_ns'],p['end_monotonic_ns']
        values=[r for r in mission if a<=r['host_completion_receive_monotonic_ns']<=b]
        if p['phase'] in ['baseline','turned_hold','final_hold']:
            values=[r for r in values if r['host_completion_receive_monotonic_ns']>=b-2_000_000_000]
            if len(values)<3 or values[-1]['host_completion_receive_monotonic_ns']-values[0]['host_completion_receive_monotonic_ns']<1_500_000_000:raise ValueError('Incomplete final two-second encoder hold')
            holds[p['phase']]=dict(median_unwrapped_rad=statistics.median(r['unwrapped_rad'] for r in values),samples=len(values))
        else:
            if len(values)<3:raise ValueError('Missing encoder motion interval')
            increments=[y['unwrapped_rad']-x['unwrapped_rad'] for x,y in zip(values,values[1:])]
            motion[p['phase']]=dict(net_deg=math.degrees(values[-1]['unwrapped_rad']-values[0]['unwrapped_rad']),
                travel_deg=math.degrees(sum(abs(v) for v in increments)),samples=len(values))
    a,b,c=(holds[p]['median_unwrapped_rad'] for p in ['baseline','turned_hold','final_hold'])
    outward=math.degrees(b-a);returned=math.degrees(c-b)
    left=outward>=5 and motion['turn_left']['net_deg']>=5
    back=returned<=-5 and motion['return']['net_deg']<=-5
    return dict(scope='RECEIVE-ONLY ENCODER DIRECTION/RETURN EXERCISE; NOT BODY HEADING ACCEPTANCE',
        evidence_complete=True,motion_check_complete=bool(left and back),left_positive_exercised=left,return_negative_exercised=back,
        hold_to_hold_left_deg=outward,hold_to_hold_return_deg=returned,encoder_return_closure_deg=math.degrees(c-a),
        operator_declared_forward_encoder_candidate_rad=math.atan2(math.sin(a),math.cos(a)),
        hold_windows=holds,motion_intervals=motion,minimum_exercised_angle_deg=5.,
        body_yaw_computed=False,navigation_ready=False,
        limitations=['Operator must align mechanical forward before starting and declare fixed chassis',
            'Only direction/exercise checked; approximate 45 degrees is not independent angular truth',
            'Unwrap assumes less than pi rotation between received samples; no angle samples interpolated',
            'Host receipt freshness is not a device measurement timestamp or latency guarantee',
            'Return closure reported without invented accuracy threshold; no encoder zero written to configuration'])
