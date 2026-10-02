#!/usr/bin/env python3
"""Reconstruct callback input history, including unpublished initialization poses.

Diagnostics identify callback dispositions, never supply expected velocities.
Independent algebra uses original raw poses and the sealed adapter contract.
No ROS publisher or executable production callback is run.
"""
import argparse,collections,hashlib,json,math
from pathlib import Path
import numpy as np

RAW='/lio/odometry';EVENT='/lio/adapter_diagnostics';OUT='/localization/lio_navigation_odometry'
def finite(value):
    if isinstance(value,float):return math.isfinite(value)
    if isinstance(value,dict):return all(finite(v) for v in value.values())
    if isinstance(value,(list,tuple)):return all(finite(v) for v in value)
    return True
def transform(data):
    p=data['pose']['pose'];r=p['position'];q=p['orientation'];x,y,z,w=[q[k] for k in 'xyzw']
    pos=[4.7-r['y']+.25*(2*(y*z-w*x)),.5+r['x']-.25*(2*(x*z+w*y)),.4+r['z']-.25*(1-2*(x*x+y*y))]
    qq=[(x-y)/math.sqrt(2),(x+y)/math.sqrt(2),(z+w)/math.sqrt(2),(w-z)/math.sqrt(2)]
    a=math.atan2(2*(qq[3]*qq[2]+qq[0]*qq[1]),1-2*(qq[1]**2+qq[2]**2))
    return pos,qq,a

def audit_rows(rows):
    raw=[r for r in rows if r['topic']==RAW];events=[r for r in rows if r['topic']==EVENT];outputs=[r for r in rows if r['topic']==OUT]
    faults=[];stats=collections.Counter();history=collections.deque();last=None;index=0
    pose_errors=[];velocity_errors=[];init_count=0;resets=[]
    def fail(message):faults.append(message)
    if not raw or len(raw)!=len(events):
        return {'pass':False,'faults':['raw-input/callback-disposition count mismatch or empty'],
            'counts':{'raw':len(raw),'events':len(events),'outputs':len(outputs)}}
    for sequence,(r,drow) in enumerate(zip(raw,events)):
        try:
            event=json.loads(drow['data']['data']);kind=event['event'];stats[kind]+=1
            if not finite(event):raise ValueError('nonfinite callback evidence')
            t=r['message_stamp_s'];data=r['data']
            if not finite(data) or not isinstance(t,(float,int)) or not math.isfinite(t):raise ValueError('nonfinite raw input')
            stamp=data['header']['stamp'];actual=stamp['sec']+stamp['nanosec']*1e-9
            if abs(actual-t)>1e-8:raise ValueError('raw payload/header stamp inconsistent')
            if 'stamp_s' in event and abs(event['stamp_s']-t)>1e-8:raise ValueError('callback/raw sequence stamp mismatch')
            if kind=='rejected':
                if event.get('reason')!='frame or stale/future LIO odometry':raise ValueError('unsupported rejection; no waiver')
                # Production rejects before updating its input history. Callback
                # clock is not recorded; rejection freshness is not independently proven.
                continue
            if data['header']['frame_id']!='lio_odom' or data['child_frame_id']!='lio_imu':raise ValueError('invalid input frames')
            q=data['pose']['pose']['orientation']
            if abs(sum(q[k]**2 for k in 'xyzw')-1)>.01:raise ValueError('invalid input quaternion')
            if kind=='duplicate_or_older':
                if last is None or t>last:raise ValueError('inconsistent duplicate disposition')
                continue
            if last is not None and t<=last:raise ValueError('nonmonotonic input accepted into history')
            if last is not None and t-last>.2:history.clear();resets.append(t)
            pos,quat,a=transform(data);history.append((t,pos[0],pos[1],a));last=t
            while history and t-history[0][0]>.2+1e-9:history.popleft()
            ready=len(history)>=3 and history[-1][0]-history[0][0]>=.10-1e-9
            if kind=='velocity_initializing':
                init_count+=1
                if ready:raise ValueError('initialization disposition despite complete window')
                continue
            if kind!='accepted' or not ready:raise ValueError('missing/unknown event or incomplete accepted window')
            if index>=len(outputs):raise ValueError('accepted callback missing output')
            out=outputs[index];index+=1
            if not math.isfinite(out['message_stamp_s']) or abs(out['message_stamp_s']-t)>1e-8 or not finite(out['data']):raise ValueError('output stamp/finite failure')
            op=out['data']['pose']['pose'];oq=op['orientation'];ov=out['data']['twist']['twist']
            if out['data']['header']['frame_id']!='odom' or out['data']['child_frame_id']!='base_link':raise ValueError('invalid output frames')
            residual=max(*(abs(pos[i]-op['position'][k]) for i,k in enumerate('xyz')),
                *(abs(quat[i]-oq[k]) for i,k in enumerate('xyzw')))
            pose_errors.append(residual)
            if residual>=1e-9:raise ValueError('raw/output transform mismatch')
            h=np.array(history);dt=h[:,0]-h[:,0].mean();den=dt@dt
            vx=dt@h[:,1]/den;vy=dt@h[:,2]/den;wz=dt@np.unwrap(h[:,3])/den;c,s=math.cos(a),math.sin(a)
            expected=[c*vx+s*vy,-s*vx+c*vy,wz];actual=[ov['linear']['x'],ov['linear']['y'],ov['angular']['z']]
            verr=float(max(abs(x-y) for x,y in zip(expected,actual)));velocity_errors.append(verr)
            if verr>=1e-7:raise ValueError('causal velocity mismatch')
            # Cross-check metadata but never use its supplied twist as expectation.
            if event['velocity_window_samples']!=len(history) or abs(event['velocity_window_s']-(t-history[0][0]))>1e-8:
                raise ValueError('reported window metadata inconsistent')
        except (KeyError,ValueError,TypeError,IndexError) as error:
            fail({'sequence':sequence,'reason':str(error)})
    if index!=len(outputs):fail('unmatched extra outputs')
    return {'pass':not faults and len(velocity_errors)>100,'faults':faults[:30],'fault_count':len(faults),
        'counts':{'raw':len(raw),'events':len(events),'outputs':len(outputs),'dispositions':dict(stats)},
        'initialization_inputs_retained':init_count,'gap_resets':resets,
        'max_pose_residual':max(pose_errors,default=None),'max_velocity_residual':max(velocity_errors,default=None),
        'thresholds':{'pose_residual':1e-9,'velocity_residual':1e-7,'history_s':.20,'minimum_history_s':.10},
        'scope':'Input/output algebra, full callback sequence and causal history; not completion, receive continuity or callback-clock freshness audit.',
        'limitations':['Callback dispositions identify omitted/rejected input; expected pose/velocity calculated solely from raw pose inputs.',
            'Same recorded publisher and stamp/event sequence association; no publication-loss absence proof.',
            'Rejected callbacks do not include source stamps or callback clock; exact freshness rejection cause remains not independently observable.']}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--native',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    rows=[json.loads(l) for l in a.native.open()];result=audit_rows(rows)
    result.update(native=str(a.native),native_sha256=hashlib.sha256(a.native.read_bytes()).hexdigest(),evaluator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    with a.output.open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:result.get(k) for k in ['pass','faults','counts','max_velocity_residual']},indent=2))
    raise SystemExit(0 if result['pass'] else 2)
