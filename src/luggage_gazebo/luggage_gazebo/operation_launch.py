"""Full-operation simulation composition; no client and no startup motion.

The caller supplies the exact already-expanded Gazebo model used by RSP and
move_group. This module owns the O map and new plan-only native services.
"""
import hashlib
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from scipy.spatial.transform import Rotation
import yaml

from luggage_description.runtime_identity import scene_revision_from_path
from luggage_description.scene_tf_config_utils import load_scene_tf_config, world_transform
from luggage_mapping.configuration import owner_configuration
from luggage_planning.atlas.runtime import load_atlas_runtime_for_scene, resolve_atlas_files


def operation_atlas_config(config):
    """Explicit overrides win; complete Gazebo uses its offline reviewed model."""
    explicit = str(config.get('atlas_runtime_config', '')).strip()
    return explicit or str(Path(get_package_share_directory('luggage_planning'))/
                           'config/atlas_runtime_gazebo_operation.yaml')


def validate_operation_selection(config):
    pipeline = config['planning_pipeline']
    if pipeline not in ('legacy_segments', 'joint_operation'):
        raise ValueError('planning_pipeline must be legacy_segments or joint_operation')
    if pipeline == 'legacy_segments':
        return False
    required = ('use_moveit', 'use_motion', 'use_vacuum', 'use_perception', 'use_semantic')
    enabled = lambda name: str(config[name]).lower() == 'true'
    if any(not enabled(name) for name in required):
        raise ValueError('joint_operation requires MoveIt, motion, vacuum and measured perception')
    if any(enabled(name) for name in ('use_cargo_map', 'use_packing', 'start_placement_prior', 'frame_runtime')):
        raise ValueError('joint_operation composes its own O/placement/native owners; disable legacy owners')
    if config['top_surface_mode'] != 'dynamic':
        raise ValueError('joint_operation requires measured dynamic suction support')
    folder = Path(config['operation_run_dir'])
    if not folder.is_absolute() or folder.exists():
        raise ValueError('operation_run_dir must be an absolute fresh evidence directory')
    return True


def operation_composition(config, robot_xml, semantic_xml):
    folder = Path(config['operation_run_dir'])
    folder.mkdir(parents=True, exist_ok=False)
    scene = config['scene_tf_config']
    source = load_scene_tf_config(scene)
    translation, rpy = world_transform(source, 'container_link')
    owner = owner_configuration(backend='observer', scene=scene,
        resolution_override=config['cargo_map_resolution_m'],
        observer_config=str(Path(get_package_share_directory('uld_occ_observer'))/'config/observer.yaml'),
        artifact_root=folder/'map_owner')
    revision = scene_revision_from_path(scene)
    # Never mint a replacement qualification when model bytes differ.
    atlas_files = resolve_atlas_files(load_atlas_runtime_for_scene(scene,
        operation_atlas_config(config)))
    atlas_parameters = {}
    atlas_status = 'ATLAS_UNAVAILABLE_ONLINE_IK'
    if atlas_files:
        receipt = Path(atlas_files[0]).parent/'operation_atlas_qualification.json'
        if receipt.is_file():
            atlas_parameters = dict(atlas_npz=atlas_files[0],atlas_yaml=atlas_files[1],
                atlas_qualification_file=str(receipt),atlas_scene_tf_config=scene)
            reviewed = json.loads(receipt.read_text())
            current_model = hashlib.sha256((robot_xml+semantic_xml).encode()).hexdigest()
            atlas_status = ('REVIEWED_IDENTITY_MATCH_REQUIRES_ONLINE_VALIDATION'
                if reviewed['model_sha256']==current_model and reviewed['scene_revision']==revision
                else 'REVIEWED_IDENTITY_MISMATCH_ONLINE_IK')
    common = dict(use_sim_time=True, frame_scene_revision=revision,
        robot_description=ParameterValue(robot_xml, value_type=str),
        robot_description_semantic=ParameterValue(semantic_xml, value_type=str),
        container_translation=list(translation),
        container_quaternion_xyzw=Rotation.from_euler('xyz', rpy).as_quat().tolist(),
        observer_geometry_file=owner.geometry_file, observer_config_file=owner.observer_config_file,
        scene_tf_config=scene, collision_margin_m=0.05)
    kinematics_path = Path(get_package_share_directory('elfin_moveit_config'))/'config/kinematics.yaml'
    kinematics = {'robot_description_kinematics': yaml.safe_load(kinematics_path.read_text())}
    nodes = [Node(package='uld_occ_observer', executable='observer_node', name='uld_occ_observer',
        output='screen', parameters=[dict(use_sim_time=True, geometry_file=owner.geometry_file,
            config_file=owner.observer_config_file, input_mode='subscription',
            cloud_topic='/luggage/observation/optical_points', optical_frame='camera_depth_optical_frame',
            lidar_cloud_topic='/luggage/observation/lidar_points',
            lidar_frame=config.get('operation_lidar_frame','livox_frame'))]),
        Node(package='luggage_perception', executable='observer_depth_node.py', name='observer_depth',
            output='screen', parameters=[dict(use_sim_time=True)]),
        Node(package='luggage_perception', executable='observer_lidar_node.py', name='observer_lidar',
            output='screen', parameters=[dict(use_sim_time=True,
                lidar_topic=config.get('operation_lidar_topic','/livox/lidar'),
                lidar_frame=config.get('operation_lidar_frame','livox_frame'))]),
        Node(package='luggage_packing', executable='placement_planner_node.py', name='placement_planner',
            output='screen', parameters=[dict(use_sim_time=True, scene_tf_config=scene,
                atlas_runtime_config=operation_atlas_config(config),
                volume_evidence_source='observer', validity_backend='observer_o',
                validity_topic='/uld_occ_observer/validity', validity_status_service='/uld_occ_observer/get_validity',
                volume_observer_geometry_file=owner.geometry_file,
                volume_observer_config_file=owner.observer_config_file,
                floor_prior_resolution=owner.resolution_m)])]
    for executable, name in (('grasp_local_check_node','grasp_local_check'),
                              ('snapshot_joint_path_node','snapshot_joint_path')):
        nodes.append(Node(package='luggage_planning', executable=executable, name=name,
                          output='screen', parameters=[common,kinematics]))
    nodes.append(Node(package='luggage_planning', executable='joint_operation_prepare_node.py',
        name='joint_operation_prepare', output='screen', parameters=[common,dict(
            map_backend='observer', evidence_dir=str(folder/'prepare'),
            max_preparation_budget_s=float(config['operation_prepare_budget_s'])),atlas_parameters]))
    (folder/'robot.urdf').write_text(robot_xml)
    (folder/'robot.srdf').write_text(semantic_xml)
    manifest = dict(pipeline='joint_operation', resolved=config,
        scene_revision=revision, model_sha256=hashlib.sha256((robot_xml+semantic_xml).encode()).hexdigest(),
        map_owner=dict(backend=owner.backend, resolution_m=owner.resolution_m,
                       geometry_file=owner.geometry_file, config_file=owner.observer_config_file),
        package_shares={name:get_package_share_directory(name) for name in
                        ('luggage_gazebo','luggage_planning','luggage_perception','luggage_description')},
        startup_authorizes_motion=False)
    manifest['operation_atlas'] = dict(status=atlas_status, parameters=atlas_parameters)
    (folder/'launch.json').write_text(json.dumps(manifest, indent=2)+'\n')
    runtime = dict(common, enable_joint_operation=True,
        operation_state_file=str(folder/'operation_state.json'),
        operation_evidence_dir=str(folder/'execution'))
    return nodes, runtime
