#!/usr/bin/env python3
"""One isolated provisional Full run with pre-goal input attestation.

This launcher never retries. The only goal publication is made by the existing
low-speed runner after nominal and measured-pose route checks.
"""

import argparse
import ast
import hashlib
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy

from run_provincial_forward_integration import (FIELD, FULL, MAP, ROOT,
    PoseObserver, offline_leg, server_processes)
from run_provincial_safety_fix_matrix import existing_servers, wait_for_launch
from summarize_provincial_stage15_terminal_v3 import sdf_walls

sys.path.insert(0, str(ROOT / 'src/uav_planning/scripts'))
from grid_route import GridRoute

OUT = ROOT / 'tools/results/provincial_stage15_full_observed_20260928_preflight2'
NAME = 'trial_full_observed_01'
LAUNCH = ['ros2', 'launch', 'uav_bringup',
          'provincial_2025_provisional.launch.py',
          'gui:=false', 'rviz:=false', 'auto_goal:=false']
INPUTS = (
    'tools/run_provincial_full_observed.py',
    'tools/run_provincial_low_speed_validation.py',
    'tools/provincial_native_trace.py',
    'tools/run_provincial_forward_integration.py',
    'tools/run_provincial_safety_fix_matrix.py',
    'tools/summarize_provincial_full_observed.py',
    'tools/summarize_provincial_forward_integration.py',
    'tools/summarize_provincial_stage15_terminal_v3.py',
    'tools/summarize_provincial_stage15_terminal_v2.py',
    'tools/provincial_terminal_gate.py',
    'src/uav_planning/scripts/gazebo_navigation_interface.py',
    'src/uav_planning/scripts/ego_trajectory_executor.py',
    'src/uav_planning/scripts/ego_goal_adapter.py',
    'src/uav_planning/scripts/grid_route.py',
    'src/uav_planning/scripts/provincial_safety_geometry.py',
    'src/uav_planning/config/ego_trajectory_executor.yaml',
    'src/uav_planning/config/ego_goal_adapter.yaml',
    'src/uav_planning/config/provincial_2025_provisional.json',
    'src/uav_bringup/launch/provincial_2025_provisional.launch.py',
    'src/uav_bringup/config/ego_provincial_2025_provisional.yaml',
    'src/uav_bringup/maps/provincial_2025_provisional.pgm',
    'src/uav_bringup/maps/provincial_2025_provisional.yaml',
    'src/uav_bringup/worlds/provincial_2025_training.sdf',
    'install/uav_planning/lib/uav_planning/gazebo_navigation_interface.py',
    'install/uav_planning/lib/uav_planning/ego_trajectory_executor.py',
    'install/uav_planning/lib/uav_planning/ego_goal_adapter.py',
    'install/uav_planning/lib/uav_planning/grid_route.py',
    'install/uav_planning/lib/uav_planning/provincial_safety_geometry.py',
    'install/uav_planning/share/uav_planning/config/ego_trajectory_executor.yaml',
    'install/uav_planning/share/uav_planning/config/ego_goal_adapter.yaml',
    'install/uav_planning/share/uav_planning/config/provincial_2025_provisional.json',
    'install/uav_bringup/share/uav_bringup/launch/provincial_2025_provisional.launch.py',
    'install/uav_bringup/share/uav_bringup/config/ego_provincial_2025_provisional.yaml',
    'install/uav_bringup/share/uav_bringup/maps/provincial_2025_provisional.pgm',
    'install/uav_bringup/share/uav_bringup/maps/provincial_2025_provisional.yaml',
    'install/uav_bringup/share/uav_bringup/worlds/provincial_2025_training.sdf',
    'install/ego_planner/lib/ego_planner/ego_planner_node',
)
PARAM_NODES = ('/gazebo_navigation_interface', '/ego_planner_node',
               '/ego_goal_adapter', '/ego_trajectory_executor')


def local_input_closure(paths):
    """Capture recursive local Python imports, including installed siblings.

    Third-party ROS/Python shared libraries remain versioned system dependencies;
    this is not a complete operating-system/container image.
    """
    found = set(paths)
    pending = list(paths)
    while pending:
        relative = pending.pop()
        source = ROOT / relative
        if source.suffix != '.py':
            continue
        tree = ast.parse(source.read_text(), filename=str(source))
        modules = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                modules.extend(item.name.split('.')[0] for item in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                modules.append(node.module.split('.')[0])
        for module in modules:
            for parent in (source.parent, ROOT/'tools', ROOT/'src/uav_planning/scripts'):
                candidate = parent / (module + '.py')
                if candidate.is_file():
                    name = str(candidate.relative_to(ROOT))
                    if name not in found:
                        found.add(name)
                        pending.append(name)
                    break
    return sorted(found)


def run_exit_code(progress):
    return 0 if (progress.get('status') == 'RUN_FINISHED_PENDING_INDEPENDENT_AUDIT'
                 and progress.get('runner_returncode') == 0
                 and not progress.get('failure')
                 and progress.get('remaining_gazebo_servers') == []) else 2


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def capture_inputs():
    snapshot = OUT / 'input_snapshot'
    records = {}
    local_files = local_input_closure(INPUTS)
    # ldd resolves the linked ELF dependency closure in the sourced runtime.
    # Keep system paths as metadata; seal every resolved repository-local ELF.
    elf = subprocess.run(['ldd', str(ROOT/'install/ego_planner/lib/ego_planner/ego_planner_node')],
                         capture_output=True, text=True, timeout=15, check=True)
    if 'not found' in elf.stdout:
        raise RuntimeError('Unresolved planner ELF dependency; source runtime first')
    elf_paths = []
    for line in elf.stdout.splitlines():
        if '=>' in line:
            candidate = line.split('=>', 1)[1].strip().split(' ', 1)[0]
        else:
            candidate = line.strip().split(' ', 1)[0]
        if not candidate.startswith('/'):
            continue
        path = Path(candidate)
        elf_paths.append({'path': str(path), 'resolved_path': str(path.resolve(strict=True))})
        if path.is_relative_to(ROOT):
            local_files.append(str(path.relative_to(ROOT)))
    # Include installed local plugins/extensions, even when loaded via dlopen.
    local_files.extend(str(path.relative_to(ROOT)) for path in (ROOT/'install').rglob('*.so*') if path.is_file())
    write_json(OUT/'planner_linked_dependencies.json', {
        'command': ['ldd', str(ROOT/'install/ego_planner/lib/ego_planner/ego_planner_node')],
        'resolved': elf_paths, 'raw_ldd_output': elf.stdout,
        'local_linked_library_contents_captured_before_goal': True,
        'dynamic_dlopen_plugins_complete': False,
        'system_library_contents_captured': False})
    for relative in sorted(set(local_files)):
        path = ROOT / relative
        if not path.is_file():
            raise FileNotFoundError(path)
        resolved = path.resolve(strict=True)
        item = {'path': str(path), 'is_symlink': path.is_symlink(),
                'resolved_path': str(resolved), 'sha256': digest(resolved),
                'size_bytes': resolved.stat().st_size}
        records[relative] = item
        target = snapshot / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(resolved, target)
        if digest(target) != item['sha256']:
            raise RuntimeError(f'input snapshot mismatch: {relative}')
    manifest = {
        'classification': 'TRAINING-ONLY / PROVISIONAL',
        'competition_arena_verified': False,
        'captured_before_goal': True,
        'trial_id': NAME,
        'local_python_import_closure_captured': True,
        'local_linked_elf_closure_captured': True,
        'all_local_installed_shared_libraries_snapshot': True,
        'dynamic_dlopen_plugins_complete': False,
        'system_dependency_content_snapshot_complete': False,
        'git_head': subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'launch_argv': LAUNCH,
        'runner_argv': [sys.executable,
            str(ROOT / 'tools/run_provincial_low_speed_validation.py'),
            '--route', 'full', '--name', NAME, '--yaw-mode', 'hold-start',
            '--terminal-validation', '--raw-every-message', '--timeout', '240',
            '--output-dir', str(OUT)],
        'inputs': records,
    }
    write_json(OUT / 'input_manifest.json', manifest)
    (OUT / 'git_status_before.txt').write_text(subprocess.check_output(
        ['git', 'status', '--short'], cwd=ROOT, text=True))
    (OUT / 'git_diff_before.patch').write_text(subprocess.check_output(
        ['git', 'diff', '--binary'], cwd=ROOT, text=True))
    versions = {}
    for label, command in {
        'ros_distro': ['printenv', 'ROS_DISTRO'],
        'ros_packages': ['dpkg-query', '-W', 'ros-humble-ros-base',
                         'ros-humble-ros-gz-bridge'],
        'gazebo': ['ign', 'gazebo', '--version'],
    }.items():
        result = subprocess.run(command, capture_output=True, text=True, timeout=15)
        versions[label] = {'command': command, 'exit_code': result.returncode,
                           'stdout': result.stdout, 'stderr': result.stderr}
    write_json(OUT / 'runtime_versions.json', versions)
    return manifest


def capture_runtime_params():
    directory = OUT / 'runtime_parameters'
    directory.mkdir()
    deadline = time.monotonic() + 25
    nodes = []
    while time.monotonic() < deadline:
        nodes = subprocess.run(['ros2', 'node', 'list'], capture_output=True,
                               text=True, timeout=15, check=True).stdout.splitlines()
        if all(node in nodes for node in PARAM_NODES):
            break
        time.sleep(.5)
    (directory / 'node_list.txt').write_text('\n'.join(nodes) + '\n')
    for node in PARAM_NODES:
        if node not in nodes:
            raise RuntimeError(f'parameter node missing before goal: {node}')
        result = subprocess.run(['ros2', 'param', 'dump', node],
                                capture_output=True, text=True, timeout=15)
        if result.returncode or 'ros__parameters:' not in result.stdout:
            raise RuntimeError(f'parameter dump unavailable: {node}: {result.stderr}')
        (directory / (node.lstrip('/') + '.yaml')).write_text(result.stdout)
    nav = (directory / 'gazebo_navigation_interface.yaml').read_text()
    executor = (directory / 'ego_trajectory_executor.yaml').read_text()
    planner = (directory / 'ego_planner_node.yaml').read_text()
    if not all(value in nav for value in ('imperfect_sensors: false',
                                         'grid_route_clearance: 0.4')):
        raise RuntimeError('live navigation parameters differ from training baseline')
    if '0.22' not in planner and '0.22' not in executor:
        raise RuntimeError('EGO inflation 0.22 not confirmed in live parameters')
    write_json(OUT / 'runtime_parameter_sha256.json', {
        path.name: digest(path) for path in directory.iterdir() if path.is_file()})


def final_hashes(manifest):
    checks = {name: {'before': item['sha256'],
                     'after': digest(Path(item['path'])),
                     'resolved_path_before': item['resolved_path'],
                     'resolved_path_after': str(Path(item['path']).resolve(strict=True))}
              for name, item in manifest['inputs'].items()}
    write_json(OUT / 'input_sha256_after.json', checks)
    files = sorted(path for path in OUT.iterdir() if path.is_file() and
                   path.name != 'raw_data_sha256.json' and
                   path.suffix in ('.csv', '.jsonl', '.log', '.json', '.yaml', '.txt', '.patch'))
    write_json(OUT / 'raw_data_sha256.json', {path.name: digest(path) for path in files})


def main():
    global OUT, NAME
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--trial-id', required=True)
    args = parser.parse_args()
    OUT = args.output_dir.resolve()
    NAME = args.trial_id
    if not NAME or any(character not in 'abcdefghijklmnopqrstuvwxyz0123456789_'
                       for character in NAME):
        raise ValueError('trial ID must use lowercase letters, digits and underscores')
    if OUT.exists():
        raise FileExistsError(OUT)
    if existing_servers():
        raise RuntimeError('Gazebo already active; isolated Full refused')
    old = ROOT / 'tools/results/provincial_stage15_forward_integration_20260928'
    for branch in ('continuous_abc', 'single_full'):
        previous = json.loads((old / branch / 'mission_verification_v4.json').read_text())
        if not previous['all_pass']:
            raise RuntimeError(f'prior {branch} audit does not pass')
    grid = GridRoute(MAP, clearance=0.40)
    walls = sdf_walls()
    reference = json.loads(FIELD.read_text())['reference_route']
    nominal = offline_leg(FULL[2], FULL[3], grid, walls, reference)
    OUT.mkdir(parents=True)
    write_json(OUT / 'nominal_offline_preflight.json', nominal)
    manifest = capture_inputs()
    progress = {'status': 'INPUTS_CAPTURED_NO_GOAL_SENT', 'launches': 0,
                'goals_requested': 0, 'name': NAME}
    write_json(OUT / 'progress.json', progress)
    launch = None
    node = None
    launch_log = None
    try:
        launch_log = (OUT / 'launch.log').open('x')
        launch = subprocess.Popen(LAUNCH, stdout=launch_log,
                                  stderr=subprocess.STDOUT,
                                  start_new_session=True)
        progress['launches'] = 1
        progress['launch_pid'] = launch.pid
        wait_for_launch()
        progress['gazebo_server_processes'] = server_processes()
        if len(progress['gazebo_server_processes']) != 1:
            raise RuntimeError('expected exactly one Gazebo server')
        capture_runtime_params()
        rclpy.init()
        node = PoseObserver()
        actual = node.fresh_pose()
        if math.dist(actual[:2], FULL[2]) > 0.20 or abs(
                math.atan2(math.sin(actual[2]-math.pi/2),
                           math.cos(actual[2]-math.pi/2))) > 0.05:
            raise RuntimeError(f'unexpected unmodified Gazebo spawn: {actual}')
        measured = offline_leg(actual[:2], FULL[3], grid, walls, reference)
        write_json(OUT / f'{NAME}.offline_preflight.json',
                   {'measured_start_gt': actual, **measured})
        progress['status'] = 'MEASURED_PREFLIGHT_PASSED'
        write_json(OUT / 'progress.json', progress)
        progress['goals_requested'] = 1
        with (OUT / f'{NAME}.runner.log').open('x') as logfile:
            try:
                result = subprocess.run(manifest['runner_argv'], stdout=logfile,
                                        stderr=subprocess.STDOUT, timeout=265)
                progress['runner_returncode'] = result.returncode
            except subprocess.TimeoutExpired:
                progress['runner_returncode'] = None
                progress['failure'] = 'runner process exceeded outer timeout'
        progress['status'] = 'RUN_FINISHED_PENDING_INDEPENDENT_AUDIT'
    except Exception as error:
        progress['status'] = 'FAILED_PRESERVED'
        progress['failure'] = f'{type(error).__name__}: {error}'
    finally:
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        if launch is not None:
            try:
                os.killpg(launch.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                launch.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGTERM)
                launch.wait(timeout=8)
        if launch_log is not None:
            launch_log.close()
        progress['remaining_gazebo_servers'] = server_processes()
        write_json(OUT / 'progress.json', progress)
        final_hashes(manifest)
    print(json.dumps(progress, ensure_ascii=False), flush=True)
    raise SystemExit(run_exit_code(progress))


if __name__ == '__main__':
    main()
