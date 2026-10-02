"""Build the RMUC arena world using the existing AeroMind robot and sensors."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
source = ROOT/'src/uav_bringup/worlds/competition_2025.sdf'
world_file = ROOT/'src/uav_bringup/worlds/rmuc_2025.sdf'
tree = ET.parse(source)
world = tree.getroot().find('world')
world.set('name', 'rmuc_2025')
for model in list(world.findall('model')):
    if model.get('name') != 'omni_robot':
        world.remove(model)
robot = world.find("model[@name='omni_robot']")
robot.find('pose').text = '3 3 0.15 0 0 0'
arena = ET.SubElement(world, 'include')
ET.SubElement(arena, 'uri').text = 'model://rmuc_2025'
ET.SubElement(arena, 'name').text = 'rmuc_2025_arena'
ET.indent(tree)
tree.write(world_file, encoding='unicode', xml_declaration=True)
print(world_file)
