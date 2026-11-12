"""Private phase composition: owned target only, envelope once, narrow ACM."""
import hashlib
import math

import pytest
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AllowedCollisionEntry, AttachedCollisionObject, CollisionObject, PlanningScene
from rclpy.serialization import deserialize_message, serialize_message
from shape_msgs.msg import SolidPrimitive

from luggage_planning.joint_operation.scenes import build_operation_scenes, owned_payload_diff
from luggage_planning.placement_prior.contracts import Snapshot, Transform
from st09_contracts.attachment_geometry import freeze_geometry
from st09_contracts.payload_envelope import EnvelopeLimits
from test_joint_operation_contracts import example_plan


@pytest.mark.parametrize('before,after',[
    ('absent','pick'),('pick','pick'),('pick','carry'),('carry','carry'),('carry','retreat')])
def test_live_phase_diff_removes_only_present_payload_and_keeps_other_objects(before,after):
    phases=compose(baseline())
    desired=deserialize_message(phases.for_phase(after).scene_cdr,PlanningScene)
    live=PlanningScene() if before=='absent' else deserialize_message(
        phases.for_phase(before).scene_cdr,PlanningScene)
    diff=owned_payload_diff(live,desired,'pickup_box')
    assert diff.is_diff and diff.robot_state.is_diff
    assert all(obj.id=='pickup_box' for obj in diff.world.collision_objects)
    assert all(obj.object.id=='pickup_box' for obj in diff.robot_state.attached_collision_objects)
    world_removes=[obj for obj in diff.world.collision_objects if obj.operation==CollisionObject.REMOVE]
    detach=[obj for obj in diff.robot_state.attached_collision_objects
            if obj.object.operation==CollisionObject.REMOVE]
    assert bool(world_removes)==(before=='pick' and after=='carry')
    assert bool(detach)==(before=='carry' and after=='retreat')
    assert diff.allowed_collision_matrix==desired.allowed_collision_matrix


def baseline():
    scene = PlanningScene()
    scene.robot_model_name = 'fixture'
    for name in ('pickup_box', 'pickup_box_neighbor', 'container', 'robot_pedestal'):
        obj = CollisionObject()
        obj.id, obj.header.frame_id, obj.operation = name, 'world', CollisionObject.ADD
        pose = Pose()
        pose.orientation.w, pose.position.x = 1., 2.
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.1]*3)]
        obj.primitive_poses = [pose]
        scene.world.collision_objects.append(obj)
    scene.allowed_collision_matrix.entry_names = ['pickup_box', 'container', 'elfin_base', 'robot_pedestal']
    scene.allowed_collision_matrix.entry_values = [AllowedCollisionEntry(enabled=row) for row in (
        [True, True, False, False], [True, False, False, False],
        [False, False, False, True], [False, False, True, False])]
    scene.allowed_collision_matrix.default_entry_names = ['pickup_box']
    scene.allowed_collision_matrix.default_entry_values = [True]
    scene.link_padding = []
    return Snapshot('scene', 7, 'hull', 'model', Transform((0., 0., 0.), (0., 0., 0., 1.)),
                    b'measured-surface', serialize_message(scene))


def compose(snapshot, geometry=None):
    plan = example_plan()
    return build_operation_scenes(snapshot, pickup_object_id='pickup_box',
        geometry=geometry or plan.predicted_attachment, world_from_pick_box=plan.world_from_pick_box,
        world_from_settled_box=plan.world_from_settled_box)


def test_box_changes_world_to_attachment_to_settled_world_without_owner_mutation():
    source = baseline()
    before = bytes(source.scene_cdr)
    result = compose(source)
    assert result.base_identity == source.identity
    assert source.scene_cdr == before
    for phase in ('pick', 'carry', 'retreat'):
        snapshot = result.for_phase(phase)
        scene = deserialize_message(snapshot.scene_cdr, PlanningScene)
        assert not scene.is_diff and not scene.robot_state.is_diff
        assert snapshot.surface_cdr == source.surface_cdr
        assert snapshot.identity != source.identity
        assert snapshot.scene_content_hash == hashlib.sha256(snapshot.scene_cdr).hexdigest()
        objects = {obj.id: obj for obj in scene.world.collision_objects}
        assert {'pickup_box_neighbor', 'container', 'robot_pedestal'} <= objects.keys()
        assert objects['pickup_box_neighbor'].primitive_poses[0].position.x == 2.
        if phase == 'carry':
            assert 'pickup_box' not in objects
            attachment, = scene.robot_state.attached_collision_objects
            assert attachment.object.id == 'pickup_box'
            assert attachment.link_name == 'suction_contact_frame'
            assert set(attachment.touch_links) == {'suction_panel', 'suction_contact_frame'}
        else:
            assert not scene.robot_state.attached_collision_objects
            pose = objects['pickup_box'].primitive_poses[0]
            expected = example_plan().world_from_pick_box if phase == 'pick' else example_plan().world_from_settled_box
            # Frozen geometry retains a numerical envelope epsilon even when
            # all configured uncertainty terms are zero.
            assert (pose.position.x, pose.position.y, pose.position.z) == pytest.approx(expected.translation, abs=1e-10)
        acm = scene.allowed_collision_matrix
        def allowed(a, b): return acm.entry_values[acm.entry_names.index(a)].enabled[acm.entry_names.index(b)]
        assert not allowed('pickup_box', 'container')
        assert allowed('pickup_box', 'suction_panel')
        assert allowed('elfin_base', 'robot_pedestal')  # unrelated declared structure survives
        assert not acm.default_entry_values[acm.default_entry_names.index('pickup_box')]


def test_conservative_geometry_is_transformed_once_and_contact_face_preserved():
    plan = example_plan()
    original = plan.predicted_attachment
    grown = freeze_geometry(original.measured_dimensions_wdh, original.t_tool_box_translation,
        original.t_tool_box_quaternion_xyzw, EnvelopeLimits(5., (.1, .1, .1), (0., 0.), .005))
    result = compose(baseline(), grown)
    for phase in ('pick', 'retreat'):
        scene = deserialize_message(result.for_phase(phase).scene_cdr, PlanningScene)
        box = next(o for o in scene.world.collision_objects if o.id == 'pickup_box')
        assert tuple(box.primitives[0].dimensions) == grown.conservative.dimensions_wdh
        nominal = plan.world_from_pick_box if phase == 'pick' else plan.world_from_settled_box
        xyz, _ = grown.conservative_pose_at_box(nominal.translation, nominal.quaternion_xyzw)
        p = box.primitive_poses[0].position
        assert (p.x, p.y, p.z) == xyz
        assert xyz != nominal.translation  # away-from-contact growth shifts centre


def test_tilted_phase_geometry_preserves_full_pose_and_rejects_flattening():
    from luggage_planning.joint_operation.scenes import validate_phase_payloads

    plan = example_plan()
    geometry = freeze_geometry((.5, .3, .2), (.08, -.03, .1),
        (math.cos(.02), 0., 0., math.sin(.02)),
        EnvelopeLimits(5., (.1, .1, .1), (.01, .02), .005))
    result = compose(baseline(), geometry)
    scenes = [deserialize_message(result.for_phase(phase).scene_cdr, PlanningScene)
              for phase in ('pick', 'carry', 'retreat')]
    arguments = dict(pickup_object_id='pickup_box', geometry=geometry,
        world_from_pick_box=plan.world_from_pick_box,
        world_from_settled_box=plan.world_from_settled_box)
    validate_phase_payloads(scenes, **arguments)
    for scene, nominal in ((scenes[0], plan.world_from_pick_box),
                           (scenes[2], plan.world_from_settled_box)):
        obj = next(o for o in scene.world.collision_objects if o.id == 'pickup_box')
        pose = obj.primitive_poses[0]
        expected = geometry.conservative_pose_at_box(nominal.translation, nominal.quaternion_xyzw)
        assert (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) == expected[1]
        assert tuple(obj.primitives[0].dimensions) == geometry.conservative.dimensions_wdh
    obj = next(o for o in scenes[2].world.collision_objects if o.id == 'pickup_box')
    q = obj.primitive_poses[0].orientation
    q.x, q.y, q.z, q.w = plan.world_from_settled_box.quaternion_xyzw
    with pytest.raises(ValueError, match='exact predicted box'):
        validate_phase_payloads(scenes, **arguments)


@pytest.mark.parametrize('kind', ['scene_diff', 'state_diff', 'attached', 'bad_acm'])
def test_incomplete_or_already_loaded_baseline_refuses(kind):
    source = baseline()
    scene = deserialize_message(source.scene_cdr, PlanningScene)
    if kind == 'scene_diff': scene.is_diff = True
    elif kind == 'state_diff': scene.robot_state.is_diff = True
    elif kind == 'attached': scene.robot_state.attached_collision_objects = [AttachedCollisionObject()]
    else: scene.allowed_collision_matrix.entry_values = []
    from dataclasses import replace
    with pytest.raises(ValueError):
        compose(replace(source, scene_cdr=serialize_message(scene)))
