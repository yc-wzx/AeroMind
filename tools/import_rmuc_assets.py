"""Import only the 2025 RMUC arena assets from the local source archives."""
from pathlib import Path
import tarfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ARCHIVES = Path('/mnt/d/RMNav/source-archives')
MODEL_ARCHIVE = ARCHIVES / 'SMBU-PolarBear-Robotics-Team__rmu_gazebo_simulator-dfc42a41eb2a.tar.gz'
MAP_ARCHIVE = ARCHIVES / 'SMBU-PolarBear-Robotics-Team__pb2025_sentry_nav-c291550ee555.tar.gz'


def copy_member(archive, suffix, destination):
    with tarfile.open(archive, 'r:gz') as bundle:
        matches = [entry for entry in bundle.getmembers()
                   if entry.isfile() and entry.name.endswith(suffix)]
        if len(matches) != 1:
            raise RuntimeError(f'Expected exactly one {suffix}: {len(matches)}')
        source = bundle.extractfile(matches[0])
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(source.read())
        print(destination)


model = ROOT / 'src/uav_bringup/models/rmuc_2025'
maps = ROOT / 'src/uav_bringup/maps'
licenses = ROOT / 'docs/third_party_licenses'
copy_member(MODEL_ARCHIVE, '/resource/models/rmuc_2025/meshes/rmuc_2025.stl',
            model/'meshes/rmuc_2025.stl')
copy_member(MODEL_ARCHIVE, '/resource/models/rmuc_2025/model.sdf', model/'model.sdf')
copy_member(MODEL_ARCHIVE, '/resource/models/rmuc_2025/model.config', model/'model.config')
copy_member(MODEL_ARCHIVE, '/LICENSE', licenses/'rmu_gazebo_simulator-LICENSE')
copy_member(MAP_ARCHIVE, '/map/simulation/rmuc_2025.pgm', maps/'rmuc_2025_source.pgm')
copy_member(MAP_ARCHIVE, '/map/simulation/rmuc_2025.yaml', maps/'rmuc_2025.yaml')
copy_member(MAP_ARCHIVE, '/LICENSE', licenses/'pb2025_sentry_nav-LICENSE')

# Preserve the visual arena mesh and generate a matching 3D collision mesh
# with only the walkable support surface excluded from physics collision.
model_sdf = model/'model.sdf'
tree = ET.parse(model_sdf)
visual_uri = tree.getroot().find('./model/link/visual/geometry/mesh/uri')
collision_uri = tree.getroot().find('./model/link/collision/geometry/mesh/uri')
collision_uri.text = 'model://rmuc_2025/meshes/rmuc_2025_obstacles_collision.stl'
ET.indent(tree)
tree.write(model_sdf, encoding='unicode', xml_declaration=True)
config = model/'model.config'
config.write_text(config.read_text().replace('<name>rmuc_2024</name>',
                                            '<name>rmuc_2025</name>'))

from generate_rmuc_navigation_map import main as generate_navigation_map
from generate_rmuc_collision import main as generate_collision
generate_navigation_map()
generate_collision()
