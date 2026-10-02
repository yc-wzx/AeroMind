"""Nominal Full guard and semantic preservation of the frozen world."""
import sys,json,xml.etree.ElementTree as ET,argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
from run_provincial_forward_integration import offline_leg,GridRoute,MAP,FIELD,sdf_walls

def canonical(element):
    return (element.tag,sorted(element.attrib.items()),(element.text or '').strip(),
            [canonical(child) for child in element])

def check():
    old=ET.parse(ROOT/'src/uav_bringup/worlds/provincial_2025_training.sdf').getroot()
    new=ET.parse(ROOT/'src/uav_bringup/worlds/provincial_stage2_lio.sdf').getroot()
    removed=[]
    for parent in list(new.iter()):
        for child in list(parent):
            if (child.tag=='sensor' and child.get('name') in ['imu3d','lidar3d']) or (child.tag=='plugin' and child.get('name')=='stage2::KinematicImu'):
                removed.append([child.tag,child.get('name')]);parent.remove(child)
    identical=canonical(old)==canonical(new)
    if not identical:raise RuntimeError('Existing arena/robot/physics changed')
    route=offline_leg((4.7,.5),(8.7,4.25),GridRoute(MAP,clearance=.40),sdf_walls(),json.loads(FIELD.read_text())['reference_route'])
    return {'pass':True,'frozen_world_semantically_identical_after_new_sensor_removal':identical,
            'removed_new_elements':removed,'full':route,'classification':'TRAINING-ONLY / PROVISIONAL'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();r=check()
    with a.output.open('x') as f:json.dump(r,f,indent=2)
    print(json.dumps(r,indent=2))
