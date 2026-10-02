"""Independent algebraic source/velocity and physical IMU measurement evidence."""
import json,math,base64
from collections import defaultdict,deque
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

def audit_lio_source(directory,name):
    raw=defaultdict(list);outputs=[];kinematics=[]
    for line in (directory/(name+'.native.jsonl')).open():
        r=json.loads(line)
        if r['topic']=='/lio/odometry':raw[round(r['message_stamp_s'],8)].append(r['data'])
        elif r['topic']=='/localization/lio_navigation_odometry':outputs.append(r)
        elif r['topic']=='/simulation/imu_kinematics':kinematics.append(json.loads(r['data']['data']))
    errors=[];slope_errors=[];history=deque();missing=[]
    for row in outputs:
        t=row['message_stamp_s'];out=row['data'];p=out['pose']['pose']['position'];q=out['pose']['pose']['orientation']
        expected=[]
        for candidate in raw.get(round(t,8),[]):
            cp=candidate['pose']['pose'];rp=cp['position'];a=cp['orientation'];x,y,z,w=[a[k] for k in 'xyzw']
            # Compose map yaw90 and local pose, then subtract rotated .25m lever.
            px=4.7-rp['y']+.25*(2*(y*z-w*x));py=.5+rp['x']-.25*(2*(x*z+w*y))
            pz=.4+rp['z']-.25*(1-2*(x*x+y*y))
            eq=[(x-y)/math.sqrt(2),(x+y)/math.sqrt(2),(z+w)/math.sqrt(2),(w-z)/math.sqrt(2)]
            residual=max(abs(px-p['x']),abs(py-p['y']),abs(pz-p['z']),*(abs(v-q[k]) for v,k in zip(eq,'xyzw')))
            expected.append(residual)
        if not expected:missing.append(t)
        else:errors.append(min(expected))
        yaw=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
        history.append((t,p['x'],p['y'],yaw))
        while history and t-history[0][0]>.2+1e-9:history.popleft()
        if outputs and t-outputs[0]['message_stamp_s']>=.21 and len(history)>3:
            a=np.asarray(history);dt=a[:,0]-a[:,0].mean();den=float(dt@dt)
            vx=float(dt@a[:,1]/den);vy=float(dt@a[:,2]/den);wz=float(dt@np.unwrap(a[:,3])/den)
            v=out['twist']['twist'];c,s=math.cos(yaw),math.sin(yaw)
            slope_errors.append(max(abs(c*vx+s*vy-v['linear']['x']),abs(-s*vx+c*vy-v['linear']['y']),abs(wz-v['angular']['z'])))
    imu={}
    for line in (directory/(name+'.sensors.jsonl')).open():
        r=json.loads(line)
        if r['topic']=='/simulation/imu3d':
            m=deserialize_message(base64.b64decode(r['cdr']),Imu);imu[round(r['message_stamp_s'],8)]=m
    acceleration_residual=[];measurement_residual=[];max_accel=0.;pairs=0
    for previous,current in zip(kinematics,kinematics[1:]):
        dt=current['stamp_s']-previous['stamp_s']
        if abs(dt-current['dt_s'])>1e-8:continue  # Missing diagnostics are reported by coverage count, not filled.
        expected=[(b-a)/dt for a,b in zip(previous['world_velocity'],current['world_velocity'])]
        acceleration_residual.append(max(abs(a-b) for a,b in zip(expected,current['world_acceleration'])))
        key=round(current['stamp_s'],8)
        if key in imu:
            m=imu[key];a=current['body_acceleration']
            measurement_residual.append(max(abs(m.linear_acceleration.x-a[0]),abs(m.linear_acceleration.y-a[1]),abs(m.linear_acceleration.z-a[2]-9.81)))
            pairs+=1
        max_accel=max(max_accel,math.hypot(*current['body_acceleration'][:2]))
    checks={'raw_LIO_to_base_pose_exact':bool(errors) and not missing and max(errors)<1e-9,
        'actual_output_causal_velocity_recomputed':len(slope_errors)>100 and max(slope_errors)<1e-7,
        'physical_kinematics_recorded':len(kinematics)>100 and len(acceleration_residual)>100,
        'physical_velocity_derivative_consistent':bool(acceleration_residual) and max(acceleration_residual)<1e-5,
        'IMU_specific_force_matches_kinematics_and_declared_noise':pairs>100 and max(measurement_residual)<.01,
        'nonzero_dynamic_acceleration_exercised':max_accel>.05}
    return {'pass':all(checks.values()),'checks':checks,'raw_LIO_samples':sum(map(len,raw.values())),
        'aligned_pose_samples':len(outputs),'unmatched_LIO_stamps':missing[:20],
        'max_pose_algebra_residual':max(errors) if errors else None,'max_velocity_slope_residual':max(slope_errors) if slope_errors else None,
        'kinematic_diagnostic_samples':len(kinematics),'consecutive_acceleration_recurrence_pairs':len(acceleration_residual),
        'IMU_measurement_pairs':pairs,'max_IMU_measurement_residual':max(measurement_residual) if measurement_residual else None,
        'max_dynamic_horizontal_acceleration':max_accel,'acceleration_from_physics_not_ROS_GT':True,
        'measurement_model':'Flat yaw-only physical chassis; lever velocity differentiated at every4ms; noise from Gazebo ImuSensor',
        'covariance_calibrated':False}
