import pytest
from luggage_gazebo.operation_launch import validate_operation_selection


def test_every_default_can_actually_resolve_from_declared_launch_arguments():
    import importlib.util
    from pathlib import Path
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    path=Path(__file__).resolve().parents[1]/'launch/sim_world.launch.py'
    spec=importlib.util.spec_from_file_location('operation_sim_world_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    context=LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    resolved=module._resolved_launch_config(context)
    assert set(resolved)==set(module._default_launch_values())
    assert resolved['planning_pipeline']=='legacy_segments'


def config(tmp_path):
    return dict(planning_pipeline='joint_operation',operation_run_dir=str(tmp_path/'fresh'),
        operation_prepare_budget_s='300.0',
        use_moveit='true',use_motion='true',use_vacuum='true',use_perception='true',use_semantic='true',
        use_cargo_map='false',use_packing='false',start_placement_prior='false',frame_runtime='false',
        top_surface_mode='dynamic')


def test_new_selection_accepts_single_explicit_owner(tmp_path):
    assert validate_operation_selection(config(tmp_path)) is True


@pytest.mark.parametrize('key,value',[
    ('planning_pipeline','v4'),('use_cargo_map','true'),('use_packing','true'),
    ('start_placement_prior','true'),('frame_runtime','true'),('use_moveit','false'),
    ('use_semantic','false'),('use_vacuum','false'),('top_surface_mode','legacy'),
    ('operation_run_dir',''),('operation_run_dir','relative')])
def test_bad_or_mixed_selection_refuses_before_startup(tmp_path,key,value):
    settings=config(tmp_path);settings[key]=value
    with pytest.raises(ValueError):validate_operation_selection(settings)


def test_existing_evidence_never_overwritten(tmp_path):
    settings=config(tmp_path);settings['operation_run_dir']=str(tmp_path)
    with pytest.raises(ValueError):validate_operation_selection(settings)


def test_explicit_old_pipeline_retains_compatibility(tmp_path):
    settings=config(tmp_path);settings.update(planning_pipeline='legacy_segments',use_cargo_map='true')
    assert validate_operation_selection(settings) is False


def test_composition_binds_one_owner_and_identical_native_model(tmp_path,monkeypatch):
    import hashlib
    import json
    from pathlib import Path
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters, normalize_parameters
    from luggage_gazebo import operation_launch
    monkeypatch.setattr(operation_launch,'Node',lambda **kwargs:kwargs)
    settings=config(tmp_path)
    settings.update(scene_tf_config=str(Path(get_package_share_directory('luggage_description'))/
        'config/scene_tf_deploy_update.yaml'),cargo_map_resolution_m='',
        operation_connection_max_edge_s='5.0',operation_max_branch_attempts='8',
        operation_max_branch_attempts_per_candidate='2')
    settings['operation_source_refresh_attempts']='3'
    xml,semantic='<robot name="test"><link name="world"/></robot>','<robot name="test"/>'
    nodes,runtime=operation_launch.operation_composition(settings,xml,semantic)
    assert [node['package'] for node in nodes].count('uld_occ_observer')==1
    assert all(node['executable']!='cargo_volume_mapper_node.py' for node in nodes)
    consumers=[node for node in nodes if node['package']=='luggage_planning']
    assert {node['executable'] for node in consumers}=={
        'grasp_local_check_node','snapshot_joint_path_node','joint_operation_prepare_node.py'}
    for node in consumers:
        evaluated=evaluate_parameters(LaunchContext(),normalize_parameters(node['parameters']))[0]
        assert evaluated['robot_description']==xml
        assert evaluated['robot_description_semantic']==semantic
        assert evaluated['observer_geometry_file']==runtime['observer_geometry_file']
        assert evaluated['observer_config_file']==runtime['observer_config_file']
    manifest=json.loads((Path(settings['operation_run_dir'])/'launch.json').read_text())
    assert manifest['model_sha256']==hashlib.sha256((xml+semantic).encode()).hexdigest()
    assert manifest['startup_authorizes_motion'] is False
    assert runtime['enable_joint_operation'] is True
    assert runtime['collision_margin_m'] == .02
    assert runtime['operation_source_refresh_attempts'] == 3
    prepare=next(node for node in consumers if node['executable']=='joint_operation_prepare_node.py')
    prepare_parameters={}
    for group in evaluate_parameters(LaunchContext(),normalize_parameters(prepare['parameters'])):
        prepare_parameters.update(group)
    assert prepare_parameters['connection_max_edge_s']==5.
    assert prepare_parameters['max_branch_attempts']==8
    assert prepare_parameters['max_branch_attempts_per_candidate']==2
    placement=next(node for node in nodes if node['package']=='luggage_packing')
    settings=evaluate_parameters(LaunchContext(),normalize_parameters(placement['parameters']))[0]
    assert settings['atlas_runtime_config']==operation_launch.operation_atlas_config(config(tmp_path))


def test_full_gazebo_uses_reviewed_config_and_explicit_override_wins():
    from luggage_gazebo.operation_launch import operation_atlas_config
    from ament_index_python.packages import get_package_share_directory
    from pathlib import Path
    expected=str(Path(get_package_share_directory('luggage_planning'))/'config/atlas_runtime_gazebo_operation.yaml')
    assert operation_atlas_config({})==expected
    assert operation_atlas_config({'atlas_runtime_config':'/reviewed/custom.yaml'})=='/reviewed/custom.yaml'


def test_installed_gazebo_atlas_exact_model_is_admitted():
    import hashlib,json
    from pathlib import Path
    from ament_index_python.packages import get_package_share_directory
    from luggage_gazebo.operation_launch import operation_atlas_config
    from luggage_planning.atlas.runtime import load_atlas_runtime,resolve_atlas_files
    from luggage_planning.joint_operation.atlas import load_operation_atlas
    from luggage_description.scene_tf_config_utils import world_transform,load_scene_tf_config
    from luggage_planning.placement_prior.contracts import Transform
    from scipy.spatial.transform import Rotation
    npz,meta=resolve_atlas_files(load_atlas_runtime(operation_atlas_config({})))
    folder=Path(npz).parent
    xml=(folder/'robot_description.xml').read_text()
    srdf=(folder/'robot_description_semantic.xml').read_text()
    receipt=folder/'operation_atlas_qualification.json'
    scene=Path(get_package_share_directory('luggage_description'))/'config/scene_tf_deploy_update.yaml'
    xyz,rpy=world_transform(load_scene_tf_config(str(scene)),'container_link')
    reader=load_operation_atlas(npz,meta,receipt,scene,robot_description=xml,
        model_sha256=hashlib.sha256((xml+srdf).encode()).hexdigest(),
        scene_revision=hashlib.md5(scene.read_bytes()).hexdigest(),
        world_from_container=Transform(tuple(xyz),tuple(Rotation.from_euler('xyz',rpy).as_quat())))
    assert reader.reason=='QUALIFIED' and reader.atlas is not None
