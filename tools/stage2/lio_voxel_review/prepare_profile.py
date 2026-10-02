"""Install a new optional launch name only after two-recording offline checks."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'tools/results/stage2_step4_lio_voxel_review_20261002'


def main():
    gate=json.loads((OUT/'offline_candidate_gate.json').read_text())
    assert gate['offline_candidate_accepted'] and all(all(v.values()) for v in gate['checks'].values())
    original=ROOT/'src/uav_bringup/launch/provincial_stage2_lio_point_boundary.launch.py'
    new=ROOT/'src/uav_bringup/launch/provincial_stage2_lio_fine_map.launch.py'
    pattern="parameters=[os.path.join(bringup,'config','spark_provincial_noise_model_sim.yaml'),sim], output='log',"
    text=original.read_text();assert text.count(pattern)==1
    text=text.replace(pattern,"parameters=[os.path.join(bringup,'config','spark_provincial_noise_model_sim.yaml'),sim,{'filter_size_map':0.05}], output='log',")
    with new.open('x') as f:f.write(text)
    new.chmod(0o755)
    target=ROOT/'install/uav_bringup/share/uav_bringup/launch/provincial_stage2_lio_fine_map.launch.py'
    if target.exists() or target.is_symlink():raise FileExistsError(target)
    target.symlink_to(new)
    print(json.dumps({'prepared':True,'old_launch_unchanged':True,'new_launch':str(new),
        'status':'OFFLINE_ACCEPTED; CLOSED_LOOP_NOT_YET_VERIFIED'},indent=2))


if __name__=='__main__':main()
