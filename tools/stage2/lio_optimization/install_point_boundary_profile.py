"""Install only new profile names, refusing different existing bytes."""
import hashlib
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'tools/results/stage2_step4_lio_optimization_20261002'


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def install():
    # The complete candidate also bypasses compensation for zero-span scans.
    source=OUT/'instant_cloud_source';build=OUT/'instant_cloud_build'
    subprocess.run(['cmake','-S',str(source),'-B',str(build)],check=True)
    subprocess.run(['cmake','--build',str(build),'-j2'],check=True)
    target=ROOT/'install/stage2_lio_sim/lib/lio_point_boundary'
    target.mkdir(exist_ok=True)
    files=[]
    for name in ['spark_lio_mapping','libspark_lio_component.so']:
        src=build/name;dst=target/name
        if dst.exists() and sha(dst)!=sha(src):raise FileExistsError('Different installed candidate bytes: '+str(dst))
        if not dst.exists():dst.write_bytes(src.read_bytes());dst.chmod(0o755)
        files.append({'path':str(dst),'sha256':sha(dst)})
    for src,dst in [
        (ROOT/'src/stage2_lio_sim/scripts/lio_point_boundary_mapping.py',ROOT/'install/stage2_lio_sim/lib/stage2_lio_sim/lio_point_boundary_mapping.py'),
        (ROOT/'src/uav_bringup/launch/provincial_stage2_lio_point_boundary.launch.py',ROOT/'install/uav_bringup/share/uav_bringup/launch/provincial_stage2_lio_point_boundary.launch.py')]:
        src.chmod(0o755)
        if dst.exists() and (not dst.is_symlink() or dst.resolve()!=src.resolve()):raise FileExistsError(dst)
        if not dst.exists():dst.symlink_to(src)
        files.append({'path':str(dst),'resolved_path':str(dst.resolve()),'sha256':sha(dst)})
    result={'pass':True,'new_profile_only':True,'files':files,'source_manifest':str(source/'production_manifest.json')}
    print(json.dumps(result,indent=2))


if __name__=='__main__':install()
