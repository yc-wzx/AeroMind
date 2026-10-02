"""Buildable production copy with exactly one measurement traversal fix."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parents[3]


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    source=ROOT/'src/third_party/spark-fast-lio/spark_fast_lio'
    out=a.output.resolve()
    if out.exists():raise FileExistsError(out)
    shutil.copytree(source,out,ignore=shutil.ignore_patterns('__pycache__','Log','PCD'))
    (out/'Log').mkdir(exist_ok=True)
    header=out/'include/imu_processing.hpp';old=header.read_text()
    anchor='      if (it_pcl == pcl_out.points.begin()) break;'
    replacement='      // Every point has been consumed; exit all older IMU intervals.\n      if (it_pcl == pcl_out.points.begin()) return;'
    assert old.count(anchor)==1
    new=old.replace(anchor,replacement);header.write_text(new)
    (out/'point_boundary.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),
        fromfile='include/imu_processing.hpp',tofile='include/imu_processing.hpp')))
    changed=[]
    for original in source.rglob('*'):
        if not original.is_file() or not (out/original.relative_to(source)).is_file():continue
        if original.read_bytes() != (out/original.relative_to(source)).read_bytes():changed.append(str(original.relative_to(source)))
    assert changed==['include/imu_processing.hpp'],changed
    manifest={'original_source':str(source),'source_changes':changed,
        'original_header_sha256':hashlib.sha256((source/'include/imu_processing.hpp').read_bytes()).hexdigest(),
        'new_header_sha256':hashlib.sha256(header.read_bytes()).hexdigest(),
        'variance_changed':False,'full_3D_preserved':True,'GT_input_added':False,
        'scope':'Measurement traversal fix only; no diagnostics added to production, no planner, control or sensor changes.'}
    with (out/'production_manifest.json').open('x') as stream:json.dump(manifest,stream,indent=2)
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':main()
