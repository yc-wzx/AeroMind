#!/usr/bin/env python3
"""Rebuild the accepted opt-in plugins from versioned sources, no results input."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/stage2/lio_optimization'))
from prepare_instantaneous_cloud import main as prepare_cloud


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def copy_verified(source, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        if digest(source) != digest(target):
            raise RuntimeError(f'Existing installed bytes differ: {target}. '
                               'Use a fresh checkout/build; no existing baseline is overwritten.')
    else:
        shutil.copy2(source, target)
    target.chmod(0o755)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--jobs', type=int, default=2)
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    package_prefix = ROOT / 'install/stage2_lio_sim'
    if not package_prefix.is_dir():
        raise RuntimeError('Run build_training_sim.sh first (isolated colcon install).')
    build = ROOT / 'build/training_profile'
    build.mkdir(parents=True, exist_ok=True)
    source = build / 'instant_cloud_source'
    if not source.exists():
        old_argv = sys.argv
        try:
            sys.argv = [str(ROOT / 'tools/stage2/lio_optimization/prepare_instantaneous_cloud.py'),
                        '--output', str(source)]
            prepare_cloud()
        finally:
            sys.argv = old_argv
    manifest = json.loads((source / 'production_manifest.json').read_text())
    for name, expected in manifest['files'].items():
        if digest(source / name) != expected:
            raise RuntimeError(f'Prepared LIO source changed: {name}')
    for source_dir, build_dir in [
        (source, build / 'lio'),
        (ROOT / 'tools/stage2/lio_safe_validation/imu_physics', build / 'position_imu'),
    ]:
        subprocess.run(['cmake', '-S', str(source_dir), '-B', str(build_dir)], check=True)
        subprocess.run(['cmake', '--build', str(build_dir), '-j', str(args.jobs)], check=True)
    outputs = []
    for name in ['spark_lio_mapping', 'libspark_lio_component.so']:
        target = package_prefix / 'lib/lio_point_boundary' / name
        copy_verified(build / 'lio' / name, target)
        outputs.append(target)
    target = package_prefix / 'lib/libstage2_position_imu.so'
    copy_verified(build / 'position_imu/libstage2_position_imu.so', target)
    outputs.append(target)
    # These optional programs were installed manually in the original trial.
    # Use the exact same versioned script bytes, including their adjacent imports.
    for path in sorted((ROOT / 'src/stage2_lio_sim/scripts').glob('*.py')):
        target = package_prefix / 'lib/stage2_lio_sim' / path.name
        if target.exists() or target.is_symlink():
            if not target.is_file() or digest(path) != digest(target):
                raise RuntimeError(f'Unexpected installed script: {target}')
        else:
            target.symlink_to(path)
        path.chmod(0o755)
        outputs.append(target)
    result = {'scope': 'simulation-only, same validated source; rebuilt binary hashes are machine-specific',
              'files': {str(p.relative_to(ROOT)): digest(p) for p in outputs}}
    (build / 'installation_manifest.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
