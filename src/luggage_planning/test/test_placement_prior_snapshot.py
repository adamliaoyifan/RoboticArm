"""Actual ROS message boundary test, independent of live services."""
import copy
import json
import math
import pytest
pytest.importorskip('moveit_msgs.msg')
from moveit_msgs.msg import PlanningScene, CollisionObject
from rclpy.serialization import serialize_message, deserialize_message
from luggage_planning.placement_prior_contracts import Transform
from luggage_planning.placement_prior_surface import ConfirmedSurface
from luggage_planning.placement_prior_snapshot import build_private_snapshot


def test_world_copy_keeps_static_geometry_and_rotates_unknown_and_occupied_columns():
    surface=dict(frame_id='container_link',scene_revision='cal',map_revision=4,
        geometry_hash='hull',floor_z_container=.2,ceiling_z_container=1.2,
        resolution_m=.1,origin_xy_container=[0,0],
        state=[['occupied','unknown','free']],surface_z_container=[[.5,None,.2]])
    frozen=ConfirmedSurface(json.dumps(surface).encode(),b'surface',b'confirmation')
    scene=PlanningScene();scene.is_diff=False
    hull=CollisionObject();hull.id='container';scene.world.collision_objects=[hull]
    before=serialize_message(scene)
    transform=Transform((1,2,0),(0,0,math.sqrt(.5),math.sqrt(.5)))
    result=build_private_snapshot(frozen,scene,'model',transform,collision_margin_m=.05)
    assert serialize_message(scene)==before
    decoded=deserialize_message(result.scene_cdr,PlanningScene)
    assert [o.id for o in decoded.world.collision_objects]==[
        'container','placement-prior-occ-0-0','placement-prior-occ-0-1']
    occupied,unknown=decoded.world.collision_objects[1:]
    p=occupied.primitive_poses[0].position
    assert (p.x,p.y,p.z)==pytest.approx((.95,2.05,.35))
    assert occupied.primitives[0].dimensions==pytest.approx((.2,.2,.3))
    assert unknown.primitives[0].dimensions[2]==pytest.approx(1.)
    assert result.map_revision==4
    # Serialized CDR padding is not semantic identity; fresh composition of
    # exactly the same message fields must hit the same cache key.
    again=build_private_snapshot(frozen,scene,'model',transform,collision_margin_m=.05)
    assert result.identity==again.identity


def test_scene_diff_and_missing_occupied_height_fail_closed():
    surface=dict(frame_id='container_link',scene_revision='cal',map_revision=4,
        geometry_hash='hull',floor_z_container=.2,ceiling_z_container=1.2,
        resolution_m=.1,origin_xy_container=[0,0],state=[['occupied']],surface_z_container=[[None]])
    frozen=ConfirmedSurface(json.dumps(surface).encode(),b'surface',b'confirmation')
    scene=PlanningScene();scene.is_diff=True
    args=(frozen,scene,'model',Transform((0,0,0),(0,0,0,1)))
    with pytest.raises(ValueError,match='full'):build_private_snapshot(*args,collision_margin_m=.05)
    scene.is_diff=False
    with pytest.raises(ValueError,match='valid measured top'):build_private_snapshot(*args,collision_margin_m=.05)


def ordering_fixture():
    from geometry_msgs.msg import Pose
    from shape_msgs.msg import SolidPrimitive
    surface=dict(frame_id='container_link',scene_revision='cal',map_revision=4,
        geometry_hash='hull',floor_z_container=.2,ceiling_z_container=1.2,
        resolution_m=.1,origin_xy_container=[0,0],state=[['free']],surface_z_container=[[.2]])
    frozen=ConfirmedSurface(json.dumps(surface).encode(),b'surface',b'confirmation')
    scene=PlanningScene();scene.is_diff=False
    scene.robot_state.joint_state.name=['elfin_joint1']
    scene.robot_state.joint_state.position=[0.]
    box=CollisionObject();box.id='wall';box.header.frame_id='world'
    shape=SolidPrimitive();shape.type=SolidPrimitive.BOX;shape.dimensions=[.1,1.,1.]
    pose=Pose();pose.orientation.w=1.;box.primitives=[shape];box.primitive_poses=[pose]
    scene.world.collision_objects=[box]
    def snapshot(message):
        return build_private_snapshot(frozen,message,'model',Transform((0,0,0),(0,0,0,1)),
                                      collision_margin_m=.05)
    return scene,snapshot


def test_moving_joint_samples_preserve_order_only_not_cached_collision_result():
    from dataclasses import replace
    from luggage_planning.placement_prior_prefetch import rank_regenerated_placements,reusable_placements
    from luggage_planning.placement_prior_contracts import PlacementCandidate
    from luggage_planning.placement_prior import compute_prior
    from test_placement_prior import request,SensorWall
    scene,snapshot=ordering_fixture();before=snapshot(scene)
    moved=copy.deepcopy(scene)
    moved.robot_state.joint_state.position=[.3]
    moved.robot_state.joint_state.velocity=[.1]
    moved.robot_state.joint_state.effort=[.2]
    moved.robot_state.joint_state.header.stamp.sec=10
    from rosidl_runtime_py.convert import message_to_ordereddict
    original_fields=message_to_ordereddict(moved);after=snapshot(moved)
    assert message_to_ordereddict(moved)==original_fields
    assert before.identity!=after.identity
    assert before.proposal_identity==after.proposal_identity
    measured=replace(request(),snapshot=before)
    old_result=compute_prior(measured,SensorWall())
    assert old_result.adopt(replace(measured,snapshot=after))==()
    hypothetical=replace(measured,measured=False)
    template_result=compute_prior(hypothetical,SensorWall())
    prefetch=(hypothetical,template_result)
    # Full geometry reuse is still forbidden after arm movement.
    assert reusable_placements(prefetch,after,measured.box_wdh,1)==()
    fallback=PlacementCandidate('fallback',Transform((.1,0,0),(0,0,0,1)))
    fresh=(fallback,*measured.placements)
    assert rank_regenerated_placements(prefetch,after,fresh)==tuple(reversed(fresh))


@pytest.mark.parametrize('field',['world','acm','padding','attached','base','joint_name','joint_frame'])
def test_ordering_context_invalidates_every_non_joint_sample_change(field):
    from moveit_msgs.msg import AttachedCollisionObject,LinkPadding
    from geometry_msgs.msg import Transform as RosTransform
    scene,snapshot=ordering_fixture();before=snapshot(scene)
    if field=='world':scene.world.collision_objects[0].primitive_poses[0].position.x=.01
    elif field=='acm':
        scene.allowed_collision_matrix.default_entry_names=['camera_link']
        scene.allowed_collision_matrix.default_entry_values=[True]
    elif field=='padding':
        padding=LinkPadding();padding.link_name='camera_link';padding.padding=.01
        scene.link_padding=[padding]
    elif field=='attached':
        attached=AttachedCollisionObject();attached.link_name='suction_contact_frame';attached.object.id='payload'
        scene.robot_state.attached_collision_objects=[attached]
    elif field=='base':
        pose=RosTransform();pose.rotation.w=1.;pose.translation.x=.1
        scene.robot_state.multi_dof_joint_state.joint_names=['world_joint']
        scene.robot_state.multi_dof_joint_state.transforms=[pose]
    elif field=='joint_name':scene.robot_state.joint_state.name=['different_joint']
    elif field=='joint_frame':scene.robot_state.joint_state.header.frame_id='different_frame'
    after=snapshot(scene)
    assert before.proposal_identity!=after.proposal_identity
