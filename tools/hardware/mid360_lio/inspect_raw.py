#!/usr/bin/env python3
"""Read-only review of the exact sealed raw capture, without altering old hashes."""
import json
import statistics
import sys
from pathlib import Path

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'mid360'))
from preflight import verify_capture
from core import POINTS,IMU_RAW,IMU_SI

def inspect(path):
    path=Path(path);saved=verify_capture(path)
    rows=[json.loads(line) for line in (path.parent/'receipts.jsonl').read_text().splitlines()]
    by_topic={topic:[r for r in rows if r['topic']==topic] for topic in [POINTS,IMU_RAW,IMU_SI]}
    if any(len(by_topic[t])!=saved['streams'][t]['count'] for t in by_topic):
        raise RuntimeError('Original summary/receipt count mismatch')
    epochs={}
    for topic,items in by_topic.items():
        if not items or any(v.get('error') is not None or v.get('stamp_ns') is None for v in items):
            raise RuntimeError('Original raw evidence contains missing/invalid records')
        deltas=[(v['receive_wall_ns']-v['stamp_ns'])*1e-9 for v in items]
        epochs[topic]=dict(first_device_s=items[0]['stamp_ns']*1e-9,
                          last_device_s=items[-1]['stamp_ns']*1e-9,
                          wall_minus_device_s_min=min(deltas),wall_minus_device_s_median=statistics.median(deltas),
                          wall_minus_device_s_max=max(deltas))
    si={r['stamp_ns'] for r in by_topic[IMU_SI]}
    missing=[r['stamp_ns'] for r in by_topic[IMU_RAW] if r['stamp_ns'] not in si]
    first,last=min(si),max(si)
    return dict(scope='Read-only inspection of existing real raw evidence',original_hashes_verified=True,
                source=str(path),timestamp_epochs=epochs,raw_imu_without_si_in_capture=len(missing),
                unmatched_before_si_window=sum(t<first for t in missing),
                unmatched_after_si_window=sum(t>last for t in missing),
                unmatched_inside_si_window=sum(first<=t<=last for t in missing),
                note='No retimestamping; receipt counts do not prove no publisher-side packet loss')

if __name__=='__main__':print(json.dumps(inspect(sys.argv[1]),indent=2))
