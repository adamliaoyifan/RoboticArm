"""Behavioral installed-runtime checks; run through production/run.sh."""
import argparse
import hashlib
import importlib
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MODULES = (
    'luggage_mapping.observer',
    'luggage_mapping.configuration',
    'luggage_mapping.regions',
    'luggage_planning.joint_operation.prepare_node',
    'luggage_planning.joint_operation.local_branch_search',
    'luggage_planning.joint_operation.aperture_geometry',
    'luggage_planning.joint_operation.aperture_tool_geometry',
    'luggage_planning.joint_operation.aperture',
    'luggage_planning.joint_operation.scene_admission',
    'luggage_planning.joint_operation.preparation_profile',
    'luggage_planning.joint_operation.preparation_refresh',
    'luggage_planning.joint_operation.search_evidence',
    'luggage_planning.joint_operation.contact_observation',
    'luggage_planning.joint_operation.pickup_observation',
    'luggage_planning.joint_operation.view_proposals',
    'luggage_planning.joint_operation.view_scene',
    'luggage_planning.joint_operation.view_motion',
    'luggage_planning.joint_operation.view_context',
    'luggage_planning.joint_operation.view_budget',
    'luggage_planning.joint_operation.view_execution',
    'luggage_planning.joint_operation.view_execution_ports',
    'luggage_planning.joint_operation.view_owner',
    'luggage_planning.joint_operation.view_certification',
    'luggage_planning.joint_operation.rpc',
    'luggage_planning.joint_operation.runtime',
    'luggage_planning.joint_operation.runtime_ports',
    'luggage_planning.joint_operation.observer',
    'luggage_description.cargo_grid_parameters',
    'luggage_packing.placement_solver',
    'luggage_packing.placement_query',
    'luggage_packing.placement_volume_query',
    'luggage_packing.placement_recheck',
    'luggage_perception.suction_patch_evaluator',
    'luggage_perception.suction_acquisition_history',
    'luggage_perception.directed_suction_service',
    'luggage_perception.ros_suction_domain',
    'luggage_perception.pickup_view_evidence',
    'luggage_perception.pickup_view_service',
    'luggage_perception.pickup_view_association',
    'luggage_perception.ros_pickup_association',
    'luggage_perception.stationary_pickup_association',
    'luggage_planning.joint_operation.view_information',
    'luggage_planning.joint_operation.view_reprepare',
    'luggage_planning.joint_operation.view_continuation',
    'luggage_msgs.srv._reprepare_after_pickup_view_v2',
    'luggage_msgs.msg._stationary_pickup_policy_v2',
    'luggage_perception.observer_depth',
    'luggage_perception.observer_lidar',
    'luggage_gazebo.operation_launch',
    'luggage_gazebo.operation_readiness',
    'luggage_gazebo.startup_admission',
    'luggage_planning.planning_contract',
    'luggage_msgs.srv._prepare_joint_operation_v2',
    'luggage_msgs.srv._get_pickup_view_evidence_v2',
    'luggage_msgs.srv._prepare_pickup_view_v2',
    'luggage_msgs.msg._pickup_view_plan_v2',
    'luggage_msgs.msg._prepare_pickup_view_request_v2',
    'luggage_msgs.srv._check_prepared_pickup_view_v2',
    'luggage_msgs.action._execute_pickup_view_v2',
    'luggage_msgs.action._execute_joint_operation_v2',
    'uld_occ_observer.srv._read_cells',
    'uld_occ_observer.srv._initialize_empty',
)
EXECUTABLES = {
    'luggage_planning': ('joint_operation_prepare_node.py', 'motion_planner_node.py',
                         'grasp_local_check_node', 'snapshot_joint_path_node',
                         'validate_route_node', 'trajectory_selector_node.py'),
    'luggage_packing': ('placement_planner_node.py',),
    'luggage_perception': ('cargo_volume_mapper_node.py','observer_depth_node.py','observer_lidar_node.py',
                           'observer_self_mask_node'),
    'luggage_gazebo': ('joint_operation_sim_driver.py','wait_controller_manager.py'),
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
    from module_layout import load_layout
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, default=ROOT/'install-production')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists; preserve previous result')
    prefix = args.prefix.resolve()
    checks = []
    relocated = load_layout(ROOT)
    for name in dict.fromkeys((*MODULES, *relocated.values())):
        package = name.split('.')[0]
        candidate = ROOT/'src'/package/Path(*name.split('.')).with_suffix('.py')
        if name == 'luggage_planning.planning_contract':
            candidate = ROOT/'src/luggage_planning/luggage_planning/planning_contract/__init__.py'
        source = candidate if candidate.is_file() else None
        checks.append(check_module(name, prefix, source=source))
    package_names = {new.rsplit('.', 1)[0] for new in relocated.values()}
    for old in relocated:
        spec = importlib.util.find_spec(old)
        if old in package_names:
            # placement_prior.py became placement_prior/. This name now denotes
            # a package; require that package and absence of the old flat file.
            path = Path(spec.origin).resolve() if spec and spec.origin else None
            passed = bool(path and path.name == '__init__.py' and path.is_relative_to(prefix)
                          and not (path.parent.parent/(path.parent.name+'.py')).exists())
        else:
            passed = spec is None
        checks.append(dict(name=old, check='retired_flat_module_absent', passed=passed))
    for package, names in EXECUTABLES.items():
        for name in names:
            source = ROOT/'src'/package/'scripts'/name if name.endswith('.py') else None
            checks.append(check_executable(prefix, package, name, source=source))
    try:
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        for package in ('luggage_description', 'luggage_planning', 'luggage_packing',
                        'luggage_perception', 'uld_occ_observer', 'luggage_msgs',
                        'luggage_gazebo', 'luggage_mapping'):
            found = Path(get_package_prefix(package)).resolve()
            checks.append({'name': package, 'path': str(found),
                           'passed': found == prefix/package, 'kind': 'ament_package'})
        for package, relative in (
            ('luggage_planning', 'launch/hardware_pick.launch.py'),
            ('luggage_planning', 'launch/joint_operation_prepare.launch.py'),
            ('luggage_planning', 'config/planning_contract/resolved_planning_contract.json'),
            ('luggage_planning', 'config/atlas_runtime_gazebo_operation.yaml'),
            ('luggage_planning', 'data/reachability_atlas_deploy_update_gazebo/operation_atlas_qualification.json'),
            ('luggage_planning', 'data/reachability_atlas_deploy_update_gazebo/s20_container_collision_aware.npz'),
            ('luggage_planning', 'data/reachability_atlas_deploy_update_gazebo/s20_container_collision_aware.yaml'),
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
