#!/usr/bin/env python3
"""Read-only timing review of sealed native odometry. Never changes old verdicts."""
import argparse
import hashlib
import json
import math
import statistics
import sys
import time
from pathlib import Path


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_json(text):
    def bad_constant(value):
        raise ValueError('Nonfinite JSON constant: '+value)
    return json.loads(text, parse_constant=bad_constant)


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def integer(value):
    return isinstance(value, int) and not isinstance(value, bool)


def distribution(values):
    if not values:
        return dict(count=0, min=None, median=None, max=None)
    return dict(count=len(values), min=min(values), median=statistics.median(values), max=max(values))


def translation(a, b):
    return math.sqrt(sum((x-y)**2 for x, y in zip(a, b)))


def rotation_degrees(a, b):
    norm = math.sqrt(sum(x*x for x in a)*sum(x*x for x in b))
    cosine = min(1., max(0., abs(sum(x*y for x, y in zip(a, b)))/norm))
    return math.degrees(2*math.acos(cosine))


def analyze_rows(rows):
    if len(rows) < 2:
        raise ValueError('Insufficient native odometry records')
    sizes = dict(position=3, quaternion=4, velocity=6, pose_covariance=36, twist_covariance=36)
    for index, row in enumerate(rows, 1):
        if row.get('sequence') != index or row.get('error') is not None:
            raise ValueError('Invalid sequence/error at row '+str(index))
        for key in ['stamp_ns', 'receive_wall_ns', 'receive_monotonic_ns']:
            if not integer(row.get(key)) or row[key] <= 0:
                raise ValueError('Missing/invalid '+key+' at row '+str(index))
        for key, size in sizes.items():
            values = row.get(key)
            if not isinstance(values, list) or len(values) != size or not all(number(x) for x in values):
                raise ValueError('Missing/nonfinite '+key+' at row '+str(index))
        if abs(sum(q*q for q in row['quaternion'])-1.) > 1e-3:
            raise ValueError('Invalid quaternion at row '+str(index))
    receive = [(b['receive_monotonic_ns']-a['receive_monotonic_ns'])*1e-9 for a,b in zip(rows, rows[1:])]
    sensor = [(b['stamp_ns']-a['stamp_ns'])*1e-9 for a,b in zip(rows, rows[1:])]
    if any(d < 0 for d in receive):
        raise ValueError('Receive monotonic order regressed')
    span = (rows[-1]['receive_monotonic_ns']-rows[0]['receive_monotonic_ns'])*1e-9
    if span <= 0:
        raise ValueError('No positive receive span')
    regressions = []
    high_water = rows[0]['stamp_ns']
    below_watermark = 0
    for index in range(1,len(rows)):
        previous,current = rows[index-1],rows[index]
        below_watermark += current['stamp_ns'] < high_water
        high_water = max(high_water,current['stamp_ns'])
        if current['stamp_ns'] < previous['stamp_ns']:
            regressions.append(dict(sequence=current['sequence'],
                previous_stamp_ns=previous['stamp_ns'], stamp_ns=current['stamp_ns'],
                previous_receive_monotonic_ns=previous['receive_monotonic_ns'],
                receive_monotonic_ns=current['receive_monotonic_ns'],
                backward_jump_s=(previous['stamp_ns']-current['stamp_ns'])*1e-9,
                receive_gap_s=receive[index-1],
                following_sensor_step_s=sensor[index] if index<len(sensor) else None,
                position_step_m=translation(previous['position'],current['position']),
                rotation_step_deg=rotation_degrees(previous['quaternion'],current['quaternion'])))
    intervals = [(b['receive_monotonic_ns']-a['receive_monotonic_ns'])*1e-9
                 for a,b in zip(regressions,regressions[1:])]
    first,last=rows[0],rows[-1]
    return dict(count=len(rows), receive_span_s=span, receive_hz=(len(rows)-1)/span,
        max_receive_gap_s=max(receive), max_positive_sensor_gap_s=max([d for d in sensor if d>0],default=None),
        duplicate_stamps=sum(d==0 for d in sensor), adjacent_regressed_stamps=len(regressions),
        stamps_below_previous_high_watermark=below_watermark,
        regressions_per_receive_s=len(regressions)/span,
        backward_jump_s=distribution([r['backward_jump_s'] for r in regressions]),
        regression_receive_spacing_s=distribution(intervals),
        regression_position_step_m=distribution([r['position_step_m'] for r in regressions]),
        regression_rotation_step_deg=distribution([r['rotation_step_deg'] for r in regressions]),
        timestamp_strictly_increasing=all(d>0 for d in sensor),
        original_receipt_order_preserved=True,
        all_recorded_twist_components_zero=all(all(v==0 for v in r['velocity']) for r in rows),
        drift=dict(max_translation_from_first_m=max(translation(first['position'],r['position']) for r in rows),
                   end_translation_from_first_m=translation(first['position'],last['position']),
                   max_rotation_from_first_deg=max(rotation_degrees(first['quaternion'],r['quaternion']) for r in rows),
                   end_rotation_from_first_deg=rotation_degrees(first['quaternion'],last['quaternion']),
                   meaning='Estimated sensor motion under operator-declared stationary condition; no truth'),
        regression_details=regressions)


def producer_review(root, saved):
    manifest = root/'install_mid360/mid360_input_manifest.json'
    relative = 'src/third_party/spark-fast-lio/spark_fast_lio/src/spark_fast_lio.cpp'
    source = root/relative
    if not manifest.is_file() or not source.is_file():
        return dict(available=False, reason='Local captured manifest/source unavailable')
    data = load_json(manifest.read_text())
    expected = data.get('inputs',{}).get(relative,{})
    manifest_matches = sha(manifest)==saved.get('raw_build_manifest_sha256')
    source_matches = sha(source)==expected.get('sha256')
    code=source.read_text()
    calls={name:[i for i,line in enumerate(code.splitlines(),1) if text in line]
           for name,text in [('imu_prediction','publishOdometry(integrated_state, stamp)'),
                             ('lidar_correction','publishOdometry(latest_state_, stamp)')]}
    return dict(available=True, captured_manifest_hash_matches=manifest_matches,
        source_hash_matches_captured_manifest=source_matches, source_path=str(source),
        source_sha256=sha(source), output_callsite_lines=calls,
        dual_phase_publication_in_verified_source=manifest_matches and source_matches and all(calls.values()),
        limitation='Odometry records carry no callsite/phase ID; source mechanism does not independently identify each received message',
        twist_limitation='Current publishOdometry does not fill twist; recorded zero Twist cannot prove physical stationarity')


def inspect(summary_path, root=None):
    path=Path(summary_path).resolve()
    summary_sha=sha(path)
    saved=load_json(path.read_text())
    if saved.get('scope')!='REAL SENSOR-LOCAL STATIC LIO DIAGNOSTIC; NOT NAVIGATION ACCEPTANCE':
        raise ValueError('Unexpected capture scope')
    raw=path.parent/'odometry.jsonl'
    raw_sha=sha(raw)
    if saved.get('data_sha256',{}).get('odometry.jsonl') != raw_sha:
        raise ValueError('Native odometry SHA-256 mismatch')
    rows=[load_json(line) for line in raw.read_text().splitlines()]
    timing=analyze_rows(rows)
    stream=saved.get('stream',{})
    for key,actual in [('count',len(rows)),('regressed_stamps',timing['adjacent_regressed_stamps']),
                       ('duplicate_stamps',timing['duplicate_stamps'])]:
        if stream.get(key)!=actual:
            raise ValueError('Original summary/native mismatch: '+key)
    health=saved.get('health',{})
    if not (health.get('received')==health.get('written')==health.get('actual_lines')==len(rows)
            and health.get('overflow')==0 and health.get('error') is None and health.get('closed') is True):
        raise ValueError('Original writer health incomplete/inconsistent')
    if sha(raw)!=raw_sha or sha(path)!=summary_sha:
        raise ValueError('Original evidence changed during inspection')
    return dict(scope='READ-ONLY NATIVE LIO TIMESTAMP DIAGNOSIS; NOT ACCEPTANCE',
        source_summary=str(path),source_summary_sha256=summary_sha,raw_sha256=raw_sha,
        original_hashes_verified=True,original_evidence_complete=saved.get('evidence_complete'),
        original_verdict_unchanged=True,timing=timing,
        producer_source=producer_review(root,saved) if root else dict(available=False),
        status='TIMESTAMP ORDERING BLOCKED' if not timing['timestamp_strictly_increasing'] else 'DIAGNOSED; NO NEW ACCEPTANCE VERDICT',
        limitations=['No sorting, dropping, retimestamping, interpolation or timestamp gate relaxation',
                     'No navigation output or motor command',
                     'Physical stationarity is operator-declared; no ground truth',
                     'Mixed callsites are a source mechanism, not per-message provenance'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary');parser.add_argument('--output-dir')
    args=parser.parse_args()
    summary=Path(args.summary).resolve()
    root=Path(__file__).resolve().parents[3]
    report=inspect(summary,root)
    out=Path(args.output_dir).resolve() if args.output_dir else summary.parent/('timing_review_'+time.strftime('%Y%m%d_%H%M%S'))
    out.mkdir(parents=True,exist_ok=False)
    report['tool_sha256']=sha(__file__);report['command']=[sys.executable,*sys.argv]
    (out/'timing_review.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    compact=dict(report);compact['timing']=dict(report['timing'])
    details=compact['timing'].pop('regression_details')
    compact['timing']['regression_examples']=details[:5]
    print(json.dumps(compact,indent=2,allow_nan=False))
    print('REPORT:',out/'timing_review.json',flush=True)


if __name__=='__main__':
    main()
