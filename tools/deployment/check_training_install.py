#!/usr/bin/env python3
"""Read-only install/launch/linker check; never starts a simulator or sends goals."""
import ast
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from ament_index_python.packages import get_package_prefix, get_package_share_directory

ROOT = Path(__file__).resolve().parents[2]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    from launch import LaunchDescription
    import importlib.util
    launch = Path(get_package_share_directory('uav_bringup')) / 'launch/provincial_stage2_lio_fine_map.launch.py'
    source = ROOT / 'src/uav_bringup/launch' / launch.name
    if sha(launch) != sha(source):
        raise RuntimeError('Installed launch differs from source')
    spec = importlib.util.spec_from_file_location('transfer_launch', launch)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert isinstance(module.generate_launch_description(), LaunchDescription)
    prefix = Path(get_package_prefix('stage2_lio_sim'))
    libraries = [prefix / 'lib/libstage2_position_imu.so',
                 prefix / 'lib/lio_point_boundary/libspark_lio_component.so',
                 prefix / 'lib/lio_point_boundary/spark_lio_mapping']
    import os
    env = dict(os.environ)
    env['LD_LIBRARY_PATH'] = str(prefix / 'lib/lio_point_boundary') + ':' + env.get('LD_LIBRARY_PATH', '')
    for path in libraries:
        result = subprocess.run(['ldd', str(path)], env=env, text=True, capture_output=True, check=True)
        if 'not found' in result.stdout:
            raise RuntimeError(f'Unresolved library: {path}\n{result.stdout}')
    for name in ['lio_safe_navigation_interface', 'lio_point_boundary_mapping', 'lio_sensor_adapter', 'lio_odometry_adapter']:
        path = prefix / 'lib/stage2_lio_sim' / (name + '.py')
        ast.parse(path.read_text())
    sys.path[:0] = [str(prefix / 'lib/stage2_lio_sim'),
                   str(Path(get_package_prefix('uav_planning')) / 'lib/uav_planning')]
    import lio_safe_navigation_interface
    assert hasattr(lio_safe_navigation_interface, 'LioSafeNavigationInterface')
    print(json.dumps({'pass': True, 'launch': str(launch),
                      'scope': 'read-only launch/import/linker checks; no new navigation validation',
                      'libraries': {str(p): sha(p) for p in libraries}}, indent=2))


if __name__ == '__main__':
    main()
