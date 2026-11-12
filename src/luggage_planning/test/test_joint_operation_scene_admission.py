"""Actual ROS scene geometry, including MoveIt object-pose normalization."""
import copy

import pytest
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene

from luggage_planning.joint_operation.scene_admission import StaticWorldAdmission
from luggage_planning.planning_scene_client import build_mesh_collision_object
from luggage_planning.placement_prior.contracts import Transform


def scene_config():
    return dict(world_frame='world',gazebo=dict(container_model='fixture_uld'),
        container=dict(inner=dict(length=1.6,width=1.6,floor_z=.6,ceiling_z=1.6),
            opening=dict(side='negative_x',aperture=dict(corners=[[-.8,-.8,.6],[-.8,.8,.6],[-.8,.8,1.6],[-.8,-.8,1.6]]))),
        static_transforms=[dict(parent='world',child='container_link',
            translation=[0.,0.,0.],rotation_rpy=[0.,0.,0.])])


def fixture_scene_file(directory):
    import yaml
    mesh_file=directory/'fixture_static.stl'
    # Deliberately isolated fixture geometry, not a deployment ULD assertion.
    mesh_file.write_text('solid fixture\nfacet normal 0 0 1\nouter loop\n'
        'vertex 100 100 100\nvertex 101 100 100\nvertex 100 101 100\n'
        'endloop\nendfacet\nendsolid fixture\n')
    config_file=directory/'fixture_scene.yaml'
    config_file.write_text(yaml.safe_dump(scene_config()))
    from luggage_planning.planning_scene_client import load_stl_mesh_msg
    obj=build_mesh_collision_object('fixture_uld',(0.,0.,0.),(0.,0.,0.,1.),load_stl_mesh_msg(str(mesh_file)))
    return config_file,mesh_file,obj


def test_static_geometry_admission_normalized_pose_and_unrelated_world(tmp_path):
    config_file,mesh_file,obj=fixture_scene_file(tmp_path)
    guard=StaticWorldAdmission(scene_config(),Transform((0.,0.,0.),(0.,0.,0.,1.)),mesh_path=str(mesh_file))
    scene=PlanningScene();scene.world.collision_objects=[obj]
    assert guard.admit(scene) is scene
    # MoveIt may lift the shape pose into CollisionObject.pose.
    scene.world.collision_objects[0].pose=copy.deepcopy(obj.mesh_poses[0])
    scene.world.collision_objects[0].mesh_poses=[Pose()]
    scene.world.collision_objects[0].mesh_poses[0].orientation.w=1.
    assert guard.admit(scene) is scene


@pytest.mark.parametrize('damage',['empty','pose','geometry','diff','duplicate'])
def test_static_admission_rejects_missing_or_changed_environment(tmp_path,damage):
    _,mesh_file,obj=fixture_scene_file(tmp_path)
    guard=StaticWorldAdmission(scene_config(),Transform((0.,0.,0.),(0.,0.,0.,1.)),mesh_path=str(mesh_file))
    scene=PlanningScene();scene.world.collision_objects=[obj]
    if damage=='empty':scene.world.collision_objects=[]
    elif damage=='pose':scene.world.collision_objects[0].mesh_poses[0].position.x=.1
    elif damage=='geometry':scene.world.collision_objects[0].meshes[0].vertices[0].x+=.1
    elif damage=='diff':scene.is_diff=True
    else:scene.world.collision_objects.append(copy.deepcopy(obj))
    with pytest.raises(ValueError):guard.admit(scene)
