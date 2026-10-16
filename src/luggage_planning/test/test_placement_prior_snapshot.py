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
