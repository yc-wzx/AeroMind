"""Offline candidate frame/CRC inspection; does not choose or activate a control codec."""
import argparse,bisect,hashlib,json,math,struct
from pathlib import Path
ALGORITHMS={'modbus_a001_ffff':0xa001,'diagnostic_8408_ffff':0x8408}
LENGTHS=(39,)
PROFILES={'0xa001':'modbus_a001_ffff','0x8408':'diagnostic_8408_ffff'}

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def crc16(data,poly):
    crc=0xffff
    for byte in data:
        crc^=byte
        for _ in range(8):crc=(crc>>1)^poly if crc&1 else crc>>1
    return crc

def decode(frame):
    # Navigation fields only, from the operator-supplied 39-byte xuc_host layout.
    # Bytes belonging to other applications are not interpreted.
    if len(frame)!=39:raise ValueError('Expected navigation feedback frame length 39')
    flags=struct.unpack_from('<H',frame,23)[0];vx,vy,angle=struct.unpack_from('<fff',frame,25)
    allowed=frame[22]
    valid_x=bool(flags&1) and math.isfinite(vx)
    valid_y=bool(flags&2) and math.isfinite(vy)
    valid_allowed=bool(flags&4) and allowed in (0,1)
    valid_angle=bool(flags&8) and math.isfinite(angle)
    structural=all(not flags&bit or ok for bit,ok in [(1,valid_x),(2,valid_y),(4,valid_allowed),(8,valid_angle)])
    return dict(nav_allowed_raw=allowed,nav_allowed=allowed if valid_allowed else None,nav_allowed_valid=valid_allowed,
        nav_valid_flags=flags,chassis_vx_m_s=vx if valid_x else None,chassis_vx_valid=valid_x,
        chassis_vy_m_s=vy if valid_y else None,chassis_vy_valid=valid_y,
        gimbal_relative_angle_rad=angle if valid_angle else None,relative_yaw_valid=valid_angle,
        all_navigation_fields_valid=valid_x and valid_y and valid_allowed and valid_angle,
        unknown_valid_flag_bits=flags&~15,structural_fields_valid=structural)

def inspect_bytes(raw,reads,crc_profile='0xa001'):
    if crc_profile not in PROFILES:raise ValueError('Unsupported explicit CRC profile')
    if raw and not reads:raise ValueError('Missing receipt metadata')
    offset=0;previous=0;ends=[]
    for i,row in enumerate(reads,1):
        if row['sequence']!=i or row['offset']!=offset or not isinstance(row['length'],int) or row['length']<=0:raise ValueError('Invalid read sequence/byte coverage')
        enter=row['read_enter_monotonic_ns'];returned=row['read_return_monotonic_ns']
        if any(not isinstance(v,int) or isinstance(v,bool) for v in [row['sequence'],row['offset'],row['length'],enter,returned,row['receive_wall_ns']]):raise ValueError('Receipt fields must be finite integer counters/times')
        if enter<previous or returned<enter:raise ValueError('Read receive time regressed')
        previous=returned;offset+=row['length'];ends.append(offset)
    if offset!=len(raw):raise ValueError('Raw bytes/receipt counts disagree')
    headers=[];start=0
    while True:
        pos=raw.find(b'SP',start)
        if pos<0:break
        headers.append(pos);start=pos+1
    reports=[];frames=[]
    for length in LENGTHS:
        for name,poly in ALGORITHMS.items():
            matched=[];run=longest=0;last=None
            for pos in headers:
                if pos+length>len(raw):continue
                frame=raw[pos:pos+length]
                if crc16(frame[:-2],poly)!=int.from_bytes(frame[-2:],'little'):continue
                fields=decode(frame);index=bisect.bisect_left(ends,pos+length);receipt=reads[index]
                item=dict(offset=pos,length=length,crc_algorithm=name,completion_read_sequence=receipt['sequence'],
                    host_completion_receive_monotonic_ns=receipt['read_return_monotonic_ns'],device_measurement_stamp=None,**fields)
                matched.append(item);frames.append(item)
                if fields['structural_fields_valid']:
                    run=run+1 if last is not None and pos==last+length else 1;last=pos;longest=max(longest,run)
                else:run=0;last=None
            times=[r['host_completion_receive_monotonic_ns'] for r in matched]
            span=(times[-1]-times[0])*1e-9 if len(times)>1 else 0.
            valid_angles=[f['gimbal_relative_angle_rad'] for f in matched if f['relative_yaw_valid'] and f['structural_fields_valid']]
            last_fields=next((f for f in reversed(matched) if f['structural_fields_valid']),None)
            reports.append(dict(frame_bytes=length,crc_algorithm=name,crc_valid_frames=len(matched),
                relative_yaw_rad_range=[min(valid_angles),max(valid_angles)] if valid_angles else None,
                latest_navigation_fields={k:last_fields[k] for k in ['nav_valid_flags','nav_allowed','nav_allowed_valid','chassis_vx_m_s','chassis_vy_m_s','gimbal_relative_angle_rad','relative_yaw_valid']} if last_fields else None,
                structural_valid_frames=sum(f['structural_fields_valid'] for f in matched),longest_consecutive_valid_frames=longest,
                format_candidate_exercised=longest>=3,
                relative_yaw_valid_frames=sum(f['relative_yaw_valid'] and f['structural_fields_valid'] for f in matched),
                observed_completion_receive_hz=(len(times)-1)/span if span>0 else None,
                max_completion_receive_gap_s=max(((b-a)*1e-9 for a,b in zip(times,times[1:])),default=None)))
    candidates=[r for r in reports if r['format_candidate_exercised']]
    return dict(scope='RECEIVE-ONLY NAVIGATION FEEDBACK INSPECTION; NOT NAVIGATION ACCEPTANCE',
        raw_bytes=len(raw),read_chunks=len(reads),header_candidates=len(headers),candidate_reports=reports,
        format_status=(('NAV39_MODBUS_OBSERVED' if crc_profile=='0xa001' else 'NAV39_CRC8408_OBSERVED') if candidates[0]['crc_algorithm']==PROFILES[crc_profile] else 'NAV39_DIAGNOSTIC_CRC_ONLY') if len(candidates)==1 else 'AMBIGUOUS_CANDIDATES' if candidates else 'NAVIGATION_FEEDBACK_NOT_OBSERVED',
        expected_crc_profile=crc_profile,receive_profile_matches=bool(len(candidates)==1 and candidates[0]['crc_algorithm']==PROFILES[crc_profile]),
        selected_for_control=None,navigation_ready=False,validity_rule='At least 3 adjacent CRC+structurally valid frames is a diagnostic format candidate, not link/angle qualification',
        limitations=['No decoder or actuator path is enabled by this inspection',
            'Only the supplied 39-byte SP navigation feedback layout examined; a non-selected CRC remains diagnostic only',
            'Multiple candidates remain ambiguous; no fallback selects one silently',
            'Receive times describe host reads; frames within a batch share completion time, no device measurement timestamp invented',
            'No generation frequency, publisher-side losslessness or sensor/encoder clock alignment proven',
            'A valid relative-yaw flag and finite value do not prove direction/zero correctness']),frames

def audit(directory,crc_profile=None):
    root=Path(directory);meta=json.loads((root/'capture_meta.json').read_text())
    if not meta['writer_health']['closed']:raise ValueError('Raw files not closed')
    if set(meta['data_sha256'])!={'raw.bin','reads.jsonl'}:raise ValueError('Missing required original file hashes')
    for name,value in meta['data_sha256'].items():
        if sha(root/name)!=value:raise ValueError('Original input hash mismatch: '+name)
    raw=(root/'raw.bin').read_bytes();reads=[json.loads(s) for s in (root/'reads.jsonl').read_text().splitlines()]
    effective_profile=crc_profile or meta.get('expected_crc_profile','0xa001')
    result,frames=inspect_bytes(raw,reads,effective_profile);health=meta['writer_health']
    ok=health['error'] is None and health['overflow']==0 and health['received_bytes']==health['written_bytes']==len(raw) and health['received_chunks']==health['written_chunks']==len(reads)
    result.update(capture_complete=bool(meta['failure'] is None and not meta['interrupted'] and ok and meta['input_tools_unchanged']),writer_health=health,
        capture_failure=meta['failure'],interrupted=meta['interrupted'],device=meta['device'],baud=meta['baud'],
        actual_termios=meta.get('actual_termios'),original_data_sha256=meta['data_sha256'],
        serial_transmit_payload_bytes=0,input_tools_unchanged=meta['input_tools_unchanged'],modem_line_note='No DTR/RTS/reset ioctl is issued; actual line transitions not recorded')
    result['crc_profile_source']='explicit offline override' if crc_profile else 'recorded capture profile' if 'expected_crc_profile' in meta else 'legacy MODBUS default'
    result['capture_meta_sha256']=sha(root/'capture_meta.json');result['auditor_sha256']=sha(Path(__file__))
    if meta.get('guided_turn'):
        from guided_yaw import summarize
        try:result['guided_turn_check']=summarize(frames,effective_profile,meta.get('guided_phases',[]),result['capture_complete'],result['receive_profile_matches'])
        except Exception as error:result['guided_turn_check']=dict(evidence_complete=False,motion_check_complete=False,analysis_error=str(error))
    return result,frames

def main():
    p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('--output-name',default='independent_wire_review_v1.json');p.add_argument('--crc-profile',choices=list(PROFILES));a=p.parse_args()
    if Path(a.output_name).name!=a.output_name:p.error('Use a new filename')
    target=Path(a.directory)/a.output_name
    if target.exists():raise FileExistsError('Preserving existing output')
    result,_=audit(a.directory,a.crc_profile)
    with target.open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(result,indent=2,allow_nan=False));print('WIRE REVIEW:',target)
    return 0 if result['capture_complete'] else 2
if __name__=='__main__':raise SystemExit(main())
