"""Read-only follow-up inventory. Does not build, source ROS, or launch nodes."""
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[7]
OUT = Path(__file__).resolve().parent
SOURCES = (
    'src/luggage_planning/CMakeLists.txt',
    'src/luggage_planning/launch/hardware_pick.launch.py',
    'src/luggage_planning/launch/joint_operation_prepare.launch.py',
    'src/luggage_planning/luggage_planning/hardware_placement_launch.py',
    'src/luggage_planning/scripts/hardware_pick_driver.py',
    'src/luggage_planning/scripts/motion_planner_node.py',
    'src/luggage_planning/luggage_planning/hardware_place_planning.py',
    'src/luggage_planning/luggage_planning/typed_driver_adapter.py',
    'src/luggage_planning/luggage_planning/joint_operation_prepare_node.py',
    'src/luggage_planning/luggage_planning/joint_operation_runtime.py',
    'src/luggage_planning/luggage_planning/joint_operation_runtime_ports.py',
    'src/luggage_planning/luggage_planning/joint_operation_observer.py',
    'src/luggage_planning/luggage_planning/placement_prior_requests.py',
    'src/luggage_packing/scripts/placement_planner_node.py',
    'src/luggage_packing/luggage_packing/placement_solver.py',
    'src/luggage_description/config/cargo_grid.yaml',
    'src/uld_occ_observer/config/observer.yaml',
    'src/uld_occ_observer/launch/standalone.launch.py',
    'src/uld_occ_observer/srv/ReadCells.srv',
    'src/uld_occ_observer/srv/ReadSurface.srv',
    'src/luggage_msgs/srv/GetCargoVolumeV2.srv',
    'build/luggage_planning/CMakeCache.txt',
    'build/luggage_planning/cmake_install.cmake',
    'build/luggage_planning/install_manifest.txt',
)


def identity(path):
    path = Path(path)
    result = {'path': str(path), 'exists': path.exists(), 'symlink': path.is_symlink()}
    if path.exists():
        content = path.read_bytes()
        result.update(sha256=hashlib.sha256(content).hexdigest(),
                      bytes=len(content), mtime_ns=path.stat().st_mtime_ns,
                      resolved=str(path.resolve()))
    return result


def main():
    # Capture immutable boundary inputs, including generated build rules.
    inputs = []
    for relative in SOURCES:
        path = ROOT / relative
        entry = identity(path)
        if path.exists():
            destination = OUT / 'snapshots' / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(path.read_bytes())
            entry['snapshot'] = str(destination.relative_to(ROOT))
            if identity(destination)['sha256'] != entry['sha256']:
                raise RuntimeError('INPUT_CHANGED_DURING_CAPTURE: ' + relative)
        inputs.append(entry)
    entries = []
    for prefix in ('install', 'install-mo', '/tmp/joint_operation_native/install'):
        for suffix in (
            'lib/luggage_planning/motion_planner_node.py',
            'lib/luggage_planning/joint_operation_prepare_node.py',
            'lib/luggage_planning/snapshot_joint_path_node',
            'local/lib/python3.10/dist-packages/luggage_planning/joint_operation_runtime_ports.py',
        ):
            entries.append(identity(ROOT / prefix / 'luggage_planning' / suffix))
    running = []
    names = {'hardware_pick_driver.py', 'motion_planner_node.py',
             'joint_operation_prepare_node.py', 'placement_planner_node.py',
             'observer_node', 'cargo_volume_mapper_node.py'}
    for process in Path('/proc').iterdir():
        if not process.name.isdigit() or int(process.name) in (os.getpid(), os.getppid()):
            continue
        try:
            args = (process / 'cmdline').read_bytes().split(b'\0')[:3]
        except OSError:
            continue
        matched = [Path(a.decode(errors='replace')).name for a in args
                   if Path(a.decode(errors='replace')).name in names]
        if matched:
            running.append({'pid': process.name, 'executables': matched})
    changed = [i['path'] for i in inputs if i.get('sha256') != identity(i['path']).get('sha256')]
    manifest = {
        'captured_at': datetime.now().astimezone().isoformat(),
        'revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'dirty_entries': len(subprocess.check_output(
            ['git', 'status', '--porcelain'], cwd=ROOT, text=True).splitlines()),
        'case': 'production entry/source/install read-only inventory',
        'capture_complete': not changed, 'replay_possible': not changed,
        'inputs': inputs, 'install_entries': entries, 'concurrent_changes': changed,
        'matching_processes': running,
        'scope_limit': 'First three argv entries and named prefixes only; no live ROS graph or import test.',
        'environment': {k: os.environ.get(k) for k in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'PYTHONPATH')},
        'robot_actions': 0, 'nodes_started': 0, 'product_code_changes': 0,
    }
    (OUT / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({'inputs': len(inputs), 'install_entries': len(entries),
                      'concurrent_changes': changed, 'matching_processes': running}))


if __name__ == '__main__':
    main()
