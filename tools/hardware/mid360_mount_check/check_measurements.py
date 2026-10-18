"""Offline measured-mount worksheet checker. No ROS, SDK, network or motor imports."""
import argparse,hashlib,json,math
from pathlib import Path

OFFICIAL_OPTICAL_HEIGHT_MM=47.0
OFFICIAL_IMU_IN_LIDAR_MM=(11.0,23.29,-44.12)
INVERTED_MIN_FACE_HEIGHT_MM=500.0


def number(value):return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)
def source_ok(value):return isinstance(value,str) and bool(value.strip())
def vector_rotated_for_inverted_mount(v,yaw_deg):
    # R_G_L = Rz(yaw) Rx(pi). Never apply this to the raw sensor stream.
    c=math.cos(math.radians(yaw_deg));s=math.sin(math.radians(yaw_deg));x,y,z=v
    return [c*x+s*y,s*x-c*y,-z]


def inspect(data):
    missing=[];invalid=[]
    if data.get('schema')!='mid360_yaw_mount_measurements_v1':invalid.append('schema')
    if data.get('units')!='mm':invalid.append('units must be mm')
    required={'base_to_yaw_axis_mm':['x','y'],
              'yaw_axis_to_mount_face_at_zero_mm':['x','y','height_above_ground'],
              'native_lidar_x_yaw_from_gimbal_forward_deg':['value']}
    for group,fields in required.items():
        record=data.get(group,{})
        if not isinstance(record,dict):invalid.append(group);continue
        for field in fields:
            value=record.get(field)
            if value is None:missing.append(group+'.'+field)
            elif not number(value):invalid.append(group+'.'+field+' must be finite')
        if not source_ok(record.get('source')):missing.append(group+'.source (measurement/explicit alignment confirmation)')
    declaration=data.get('mount_declaration',{})
    if not isinstance(declaration,dict):declaration={};invalid.append('mount_declaration')
    for field in ['inverted','yaw_only','native_z_points_down']:
        if declaration.get(field) is None:missing.append('mount_declaration.'+field)
        elif declaration.get(field) is not True:invalid.append('Only declared inverted yaw-only Z-down mounting supported: '+field)
    if not source_ok(declaration.get('source')):missing.append('mount_declaration.source')
    feedback=data.get('yaw_feedback',{})
    if not isinstance(feedback,dict):feedback={};invalid.append('yaw_feedback')
    available=feedback.get('available')
    if available is not None and not isinstance(available,bool):invalid.append('yaw_feedback.available must be true/false/null')
    unresolved_feedback=[]
    if available is not True:unresolved_feedback.append('No confirmed dynamic yaw-feedback interface')
    else:
        for key in ['interface','unit','positive_direction','zero_reference','timestamp_basis']:
            if not source_ok(feedback.get(key)):unresolved_feedback.append('yaw_feedback.'+key)
    result=dict(scope='OFFLINE MOUNT WORKSHEET; NOT CALIBRATION OR NAVIGATION ACCEPTANCE',
        measurement_data_complete=not missing and not invalid,missing=missing,invalid=invalid,
        yaw_feedback_declared=available,yaw_feedback_contract_complete=not unresolved_feedback,
        yaw_feedback_unresolved=unresolved_feedback,live_feedback_verified=False,
        navigation_ready=False,ros_publication=False,production_configuration_changed=False,
        geometry_candidate=None,installation_height_requirement='UNASSESSED',
        official_constants=dict(optical_height_mm=OFFICIAL_OPTICAL_HEIGHT_MM,imu_in_lidar_mm=list(OFFICIAL_IMU_IN_LIDAR_MM)),
        limitations=['Input values/provenance are operator declarations, not independently measured by this tool',
            'Ideal upside-down/yaw-only orientation model; small roll/pitch errors not calibrated',
            'Internal LiDAR/IMU nominal dimensions are separate from measured vehicle installation',
            'No live encoder identity, timestamp, direction or zero validation',
            'No local-odom gravity/map alignment or chassis-centre odometry qualification'])
    if not result['measurement_data_complete']:return result
    base=data['base_to_yaw_axis_mm'];mount=data['yaw_axis_to_mount_face_at_zero_mm'];yaw=data['native_lidar_x_yaw_from_gimbal_forward_deg']['value']
    optical_offset=vector_rotated_for_inverted_mount([0.,0.,OFFICIAL_OPTICAL_HEIGHT_MM],yaw)
    imu_from_face=vector_rotated_for_inverted_mount([OFFICIAL_IMU_IN_LIDAR_MM[0],OFFICIAL_IMU_IN_LIDAR_MM[1],
        OFFICIAL_OPTICAL_HEIGHT_MM+OFFICIAL_IMU_IN_LIDAR_MM[2]],yaw)
    face=[mount['x'],mount['y'],mount['height_above_ground']]
    result['geometry_candidate']=dict(base_to_axis_xy_m=[base['x']/1000.,base['y']/1000.],
        axis_ground_to_mount_face_at_zero_m=[v/1000. for v in face],
        axis_ground_to_lidar_at_zero_m=[(a+b)/1000. for a,b in zip(face,optical_offset)],
        axis_ground_to_imu_at_zero_m=[(a+b)/1000. for a,b in zip(face,imu_from_face)],
        native_x_yaw_at_zero_deg=yaw,meaning='Derived candidate only; yaw-axis point is its ground-plane intersection')
    result['installation_height_requirement']='MET_BY_DECLARED_MEASUREMENT' if face[2]>=INVERTED_MIN_FACE_HEIGHT_MM else 'NOT_MET_BY_DECLARED_MEASUREMENT'
    result['inverted_min_mount_face_height_mm']=INVERTED_MIN_FACE_HEIGHT_MM
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('measurements');parser.add_argument('--output');args=parser.parse_args()
    path=Path(args.measurements);raw=path.read_bytes();data=json.loads(raw);result=inspect(data)
    result['input_path']=str(path.resolve());result['input_sha256']=hashlib.sha256(raw).hexdigest()
    result['tool_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    if args.output:
        with Path(args.output).open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(result,indent=2,allow_nan=False))
    return 0 if result['measurement_data_complete'] else 2

if __name__=='__main__':raise SystemExit(main())
