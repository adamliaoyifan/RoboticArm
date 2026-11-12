"""Behavioral installed-runtime checks; run through production/run.sh."""
import argparse
import hashlib
import importlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MODULES = (
    'luggage_planning.joint_operation_prepare_node',
    'luggage_planning.joint_operation_runtime',
    'luggage_planning.joint_operation_runtime_ports',
    'luggage_planning.joint_operation_observer',
    'luggage_description.cargo_grid_parameters',
    'luggage_packing.placement_solver',
    'luggage_perception.suction_patch_evaluator',
    'st09_contracts',
    'luggage_msgs.srv._prepare_joint_operation_v2',
    'luggage_msgs.action._execute_joint_operation_v2',
    'uld_occ_observer.srv._read_cells',
)
EXECUTABLES = {
    'luggage_planning': ('joint_operation_prepare_node.py', 'motion_planner_node.py',
                         'grasp_local_check_node', 'snapshot_joint_path_node',
                         'validate_route_node', 'trajectory_selector_node.py'),
    'luggage_packing': ('placement_planner_node.py',),
    'luggage_perception': ('cargo_volume_mapper_node.py',),
    'uld_occ_observer': ('observer_node',),
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_module(name, prefix, *, source=None):
    """Import the actual module, then admit its resolved origin and source bytes."""
    try:
        module = importlib.import_module(name)
        path = Path(module.__file__).resolve()
        if not path.is_relative_to(prefix.resolve()):
            return {'name': name, 'passed': False, 'reason': 'WRONG_IMPORT_PREFIX', 'path': str(path)}
        result = {'name': name, 'path': str(path), 'sha256': digest(path), 'passed': True}
        if source is not None and (not source.is_file() or digest(source) != result['sha256']):
            result.update(passed=False, reason='SOURCE_INSTALL_MISMATCH')
        return result
    except Exception as error:
        return {'name': name, 'passed': False, 'reason': 'IMPORT_FAILED',
                'error': f'{type(error).__name__}: {error}'}


def check_executable(prefix, package, name, *, source=None):
    import os
    path = prefix / package / 'lib' / package / name
    result = {'name': f'{package}/{name}', 'path': str(path), 'passed': False}
    if not path.is_file() or not os.access(path, os.X_OK):
        return dict(result, reason='EXECUTABLE_MISSING')
    if not path.resolve().is_relative_to(prefix.resolve()):
        return dict(result, reason='EXECUTABLE_OUTSIDE_PREFIX')
    result.update(passed=True, sha256=digest(path))
    if source is not None and (not source.is_file() or digest(source) != result['sha256']):
        result.update(passed=False, reason='SOURCE_INSTALL_MISMATCH')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, default=ROOT/'install-production')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists; preserve previous result')
    prefix = args.prefix.resolve()
    checks = []
    for name in MODULES:
        package = name.split('.')[0]
        candidate = ROOT/'src'/package/Path(*name.split('.')).with_suffix('.py')
        if name == 'st09_contracts':
            candidate = ROOT/'st09_contracts/st09_contracts/__init__.py'
        source = candidate if candidate.is_file() else None
        checks.append(check_module(name, prefix, source=source))
    for package, names in EXECUTABLES.items():
        for name in names:
            source = ROOT/'src'/package/'scripts'/name if name.endswith('.py') else None
            checks.append(check_executable(prefix, package, name, source=source))
    try:
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        for package in ('luggage_description', 'luggage_planning', 'luggage_packing',
                        'luggage_perception', 'uld_occ_observer', 'luggage_msgs',
                        'luggage_gazebo', 'st09_contracts'):
            found = Path(get_package_prefix(package)).resolve()
            checks.append({'name': package, 'path': str(found),
                           'passed': found == prefix/package, 'kind': 'ament_package'})
        for package, relative in (
            ('luggage_planning', 'launch/hardware_pick.launch.py'),
            ('luggage_planning', 'launch/joint_operation_prepare.launch.py'),
            ('luggage_description', 'config/scene_tf_deploy_update.yaml'),
            ('uld_occ_observer', 'config/observer.yaml'),
        ):
            installed = Path(get_package_share_directory(package))/relative
            source = ROOT/'src'/package/relative
            checks.append({'name': f'{package}/{relative}', 'kind': 'resource',
                           'passed': installed.is_file() and digest(installed) == digest(source)})
    except Exception as error:
        checks.append({'name': 'ament_lookup', 'passed': False, 'error': str(error)})
    result = {'prefix': str(prefix), 'passed': all(item['passed'] for item in checks),
              'checks': checks, 'scope': 'installed imports/bytes/lookup; no nodes or model qualification'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'passed': result['passed'], 'checks': len(checks),
                      'failures': [c for c in checks if not c['passed']]}))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
