"""Only fix measurement traversal and zero-duration cloud compensation."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parents[3]


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    source=ROOT/'src/third_party/spark-fast-lio/spark_fast_lio';out=a.output.resolve()
    if out.exists():raise FileExistsError(out)
    shutil.copytree(source,out,ignore=shutil.ignore_patterns('__pycache__','Log','PCD'))
    (out/'Log').mkdir(exist_ok=True)
    header=out/'include/imu_processing.hpp';old=header.read_text()
    first='      if (it_pcl == pcl_out.points.begin()) break;'
    empty='  if (pcl_out.points.begin() == pcl_out.points.end()) return;'
    assert old.count(first)==old.count(empty)==1
    new=old.replace(first,'      if (it_pcl == pcl_out.points.begin()) return;')
    new=new.replace(empty,empty+'''
  // IMU propagation and last-lidar bookkeeping above still execute. A truly
  // simultaneous point set needs no motion compensation at its own timestamp.
  // Do not extrapolate rays through an IMU interval boundary. Real nonzero
  // point-time scans continue through the original compensation algorithm.
  if (std::all_of(pcl_out.points.begin(), pcl_out.points.end(),
      [](const auto &point) { return point.curvature == 0.0f; })) return;
''')
    header.write_text(new)
    patch=''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),
        fromfile='include/imu_processing.hpp',tofile='include/imu_processing.hpp'))
    (out/'instantaneous_cloud.patch').write_text(patch)
    changes=[]
    for original in source.rglob('*'):
        target=out/original.relative_to(source)
        if original.is_file() and target.is_file() and original.read_bytes()!=target.read_bytes():
            changes.append(str(original.relative_to(source)))
    assert changes==['include/imu_processing.hpp']
    manifest={'original_source':str(source),'source_changes':changes,'full_3D':True,
        'observation_variance_changed':False,'sensor_timestamps_changed':False,
        'IMU_prediction_preserved':True,'GT_input_added':False,
        'files':{str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in out.rglob('*') if p.is_file()}}
    with (out/'production_manifest.json').open('x') as f:json.dump(manifest,f,indent=2)
    print(json.dumps({k:v for k,v in manifest.items() if k!='files'},indent=2))


if __name__=='__main__':main()
