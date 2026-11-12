"""Replay exact archived S20 model/static scene against current source admission."""
from pathlib import Path
import hashlib
import json
import sys

sys.path[:0] = ['src/luggage_planning', 'src/luggage_description']
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from moveit_msgs.msg import PlanningScene
from luggage_description.scene_tf_config_utils import load_scene_tf_config, world_transform
from luggage_planning.aperture_geometry import ApertureGeometry
from luggage_planning.aperture_tool_geometry import fixed_tool_vertices
from luggage_planning.joint_operation_certification import EndpointModel
from luggage_planning.joint_operation_scene_admission import StaticWorldAdmission
from luggage_planning.placement_prior_contracts import Transform

root = Path(__file__).resolve().parents[2]/'ST-4-GAZEBO-OPERATION'
files = dict(urdf=root/'cycle_04/runtime/robot.urdf', srdf=root/'cycle_04/runtime/robot.srdf',
    scene=root/'atlas_requalification_01/static_scene.cdr',
    config=Path('src/luggage_description/config/scene_tf_deploy_update.yaml'))
model = EndpointModel(files['urdf'].read_text(), files['srdf'].read_text(),
    position_tolerance_m=.002, orientation_tolerance_rad=.02)
vertices = fixed_tool_vertices(model)
config = load_scene_tf_config(str(files['config']))
xyz, rpy = world_transform(config, 'container_link')
guard = StaticWorldAdmission(config, Transform(xyz, tuple(Rotation.from_euler('xyz', rpy).as_quat())))
scene = deserialize_message(files['scene'].read_bytes(), PlanningScene)
guard.admit(scene)
result = dict(scope='Full S20 model/static scene offline replay; no native motion, sensors or controller run',
    model_sha256=model.model_sha256, rigid_proposal_vertices=len(vertices),
    static_world='exact qualified ULD mesh/pose and pedestal accepted',
    aperture_origin=ApertureGeometry.from_scene(config).origin.tolist(),
    inputs={k:dict(path=str(v),sha256=hashlib.sha256(v.read_bytes()).hexdigest()) for k,v in files.items()})
Path(__file__).with_name('full_model_geometry_final.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
