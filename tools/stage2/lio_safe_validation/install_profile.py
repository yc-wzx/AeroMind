#!/usr/bin/env python3
"""Reproducibly install only the new opt-in profile; frozen entries untouched."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path
ROOT = Path(__file__).resolve().parents[3]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(install=False):
    pairs = []
    for name in ['lio_safe_navigation_interface.py', 'lio_safety_geometry.py']:
        pairs.append((ROOT/'src/stage2_lio_sim/scripts'/name,
                      ROOT/'install/stage2_lio_sim/lib/stage2_lio_sim'/name))
    for subdir, name in [('launch', 'provincial_stage2_lio_safe.launch.py'),
                         ('worlds', 'provincial_stage2_lio_position_imu.sdf')]:
        pairs.append((ROOT/'src/uav_bringup'/subdir/name,
                      ROOT/'install/uav_bringup/share/uav_bringup'/subdir/name))
    if install:
        build = ROOT/'build/stage2_position_imu'
        subprocess.run(['cmake', '-S', str(Path(__file__).parent/'imu_physics'),
                        '-B', str(build)], check=True)
        subprocess.run(['cmake', '--build', str(build), '-j2'], check=True)
        library = ROOT/'install/stage2_lio_sim/lib/libstage2_position_imu.so'
        built = build/'libstage2_position_imu.so'
        # Never overwrite an existing installed artifact with different bytes.
        if library.exists() and sha(library) != sha(built):
            raise RuntimeError('Installed new-plugin bytes differ; inspect before replacing')
        if library.is_symlink():
            library.unlink()  # Only this exact new-profile link; target remains.
        if not library.exists(): library.write_bytes(built.read_bytes())
        for source, target in pairs:
            source.chmod(0o755)
            if target.exists() and (not target.is_symlink() or target.resolve() != source.resolve()):
                raise RuntimeError('Unexpected installed entry: '+str(target))
            if not target.exists(): target.symlink_to(source)
    checks = {str(target.relative_to(ROOT)): target.is_file() and sha(source) == sha(target)
              for source, target in pairs}
    library = ROOT/'install/stage2_lio_sim/lib/libstage2_position_imu.so'
    checks['position_imu_library_present'] = library.is_file()
    if not all(checks.values()): raise RuntimeError('Profile verification failed '+repr(checks))
    return {'pass': True, 'checks': checks, 'plugin_sha256': sha(library),
            'plugin_resolved_path': str(library.resolve()),
            'scope': 'New profile only; no planner, original world/plugin, map, navigation or control overwrite'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--install', action='store_true')
    args = parser.parse_args()
    print(json.dumps(run(args.install), indent=2))
