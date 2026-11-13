"""Three private phase scenes for one predicted grasp; no scene-owner writes.

The caller binds the pickup object ID to its fresh measured detection. Only
that owned object is replaced/removed. Other cargo, meshes, octomap, padding,
robot state and ACM pairs survive. Historical payload contact allowances are
reset for this object; only the existing suction interface is exempted.
"""
from dataclasses import dataclass, replace
import copy
import hashlib

from luggage_planning.placement_prior.contracts import Snapshot, Transform
from luggage_planning.planning_contract.attachment_geometry import AttachmentGeometry


PHASES = ("pick", "carry", "retreat")
SUCTION_TOUCH_LINKS = ("suction_panel", "suction_contact_frame")


@dataclass(frozen=True)
class OperationScenes:
    base_identity: tuple
    pickup_object_id: str
    pick: Snapshot
    carry: Snapshot
    retreat: Snapshot

    @property
    def digests(self):
        return tuple(hashlib.sha256(getattr(self, p).scene_cdr).hexdigest() for p in PHASES)

    def for_phase(self, phase):
        if phase not in PHASES:
            raise ValueError("unknown operation phase")
        return getattr(self, phase)


def _pose(transform):
    from geometry_msgs.msg import Pose
    result = Pose()
    result.position.x, result.position.y, result.position.z = transform.translation
    (result.orientation.x, result.orientation.y, result.orientation.z,
     result.orientation.w) = transform.quaternion_xyzw
    return result


def _reset_payload_contacts(scene, object_id, *, allow_suction_touch=True):
    """Preserve unrelated ACM entries, overriding every payload allowance."""
    from moveit_msgs.msg import AllowedCollisionEntry
    acm = scene.allowed_collision_matrix
    names = list(acm.entry_names)
    if len(set(names)) != len(names) or len(acm.entry_values) != len(names):
        raise ValueError("invalid baseline ACM")
    rows = [list(row.enabled) for row in acm.entry_values]
    if any(len(row) != len(names) for row in rows):
        raise ValueError("invalid baseline ACM dimensions")
    for name in (object_id,)+SUCTION_TOUCH_LINKS:
        if name not in names:
            for row in rows:
                row.append(False)
            names.append(name)
            rows.append([False]*len(names))
    payload_index = names.index(object_id)
    for index, name in enumerate(names):
        rows[index][payload_index] = rows[payload_index][index] = (
            allow_suction_touch and name in SUCTION_TOUCH_LINKS)
    acm.entry_names = names
    acm.entry_values = [AllowedCollisionEntry(enabled=row) for row in rows]
    if len(acm.default_entry_names) != len(acm.default_entry_values):
        raise ValueError("invalid baseline default ACM")
    defaults = list(acm.default_entry_values)
    for index, name in enumerate(acm.default_entry_names):
        if name == object_id:
            defaults[index] = False
    if object_id not in acm.default_entry_names:
        acm.default_entry_names = list(acm.default_entry_names)+[object_id]
        defaults.append(False)
    acm.default_entry_values = defaults


def _box(object_id, dimensions, transform, frame):
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    result = CollisionObject()
    result.id, result.header.frame_id, result.operation = object_id, frame, CollisionObject.ADD
    result.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(dimensions))]
    result.primitive_poses = [_pose(transform)]
    return result


def owned_payload_diff(live, desired, pickup_object_id):
    """Change only this payload; never remove an absent MoveIt object.

    MoveIt reports a failed scene update for a missing world-object removal.
    Adding a complete replacement already replaces an existing world object.
    Attachment removal is needed only when the live payload is attached.
    """
    from moveit_msgs.msg import PlanningScene, AttachedCollisionObject, CollisionObject
    diff = PlanningScene(is_diff=True)
    diff.robot_state.is_diff = True
    world = [copy.deepcopy(obj) for obj in desired.world.collision_objects
             if obj.id == pickup_object_id]
    attached = [copy.deepcopy(obj) for obj in desired.robot_state.attached_collision_objects
                if obj.object.id == pickup_object_id]
    if len(world) + len(attached) != 1:
        raise ValueError('EXACTLY_ONE_DESIRED_PAYLOAD_REQUIRED')
    live_world = any(obj.id == pickup_object_id for obj in live.world.collision_objects)
    live_attached = [obj for obj in live.robot_state.attached_collision_objects
                     if obj.object.id == pickup_object_id]
    if live_world and not world:
        diff.world.collision_objects = [CollisionObject(id=pickup_object_id,
                                                       operation=CollisionObject.REMOVE)]
    else:
        diff.world.collision_objects = world
    if live_attached and not attached:
        detached = AttachedCollisionObject(link_name=live_attached[0].link_name)
        detached.object = CollisionObject(id=pickup_object_id, operation=CollisionObject.REMOVE)
        diff.robot_state.attached_collision_objects = [detached]
    else:
        diff.robot_state.attached_collision_objects = attached
    diff.allowed_collision_matrix = copy.deepcopy(desired.allowed_collision_matrix)
    return diff


def same_payload_object(left, right):
    """Compare box geometry through MoveIt's object/shape pose round trip.

    MoveIt may move the first primitive pose into CollisionObject.pose and
    normalize quaternion sign. These are equivalent encodings, not changes to
    owned geometry. Preserve dimensions, frame and all shape-count restrictions.
    """
    from luggage_planning.joint_operation.contracts import _same_pose
    from shape_msgs.msg import SolidPrimitive

    def geometry(obj):
        if (len(obj.primitives) != 1 or len(obj.primitive_poses) != 1
                or obj.primitives[0].type != SolidPrimitive.BOX
                or obj.meshes or obj.mesh_poses or obj.planes or obj.plane_poses
                or obj.subframe_names or obj.subframe_poses):
            raise ValueError('ONE_PAYLOAD_BOX_REQUIRED')
        def transform(pose, *, legacy_object_pose=False):
            xyz = (pose.position.x, pose.position.y, pose.position.z)
            quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            # Legacy CollisionObject writers leave its new object-pose field
            # entirely default and store the full transform on the primitive.
            if legacy_object_pose and xyz == (0., 0., 0.) and quat == (0., 0., 0., 0.):
                quat = (0., 0., 0., 1.)
            return Transform(xyz, quat)
        pose = transform(obj.pose,legacy_object_pose=True).compose(transform(obj.primitive_poses[0]))
        return tuple(obj.primitives[0].dimensions), pose
    try:
        left_size, left_pose = geometry(left)
        right_size, right_pose = geometry(right)
        return (left.id == right.id and left.header.frame_id == right.header.frame_id
                and left_size == right_size and _same_pose(left_pose,right_pose))
    except (TypeError, ValueError):
        return False


def validate_phase_payloads(decoded_scenes, *, pickup_object_id, geometry,
                            world_from_pick_box, world_from_settled_box):
    """Check the actual native inputs contain precisely the frozen envelope.

    Phase labels and a geometry digest alone do not prove an attachment exists.
    This inspects the decoded scenes which will be sent to the native checker.
    Unrelated world objects and declared structure allowances remain untouched.
    """
    if len(decoded_scenes) != len(PHASES):
        raise ValueError('three phase scenes required')
    envelope = geometry.conservative
    for phase, scene in zip(PHASES, decoded_scenes):
        if scene.is_diff or scene.robot_state.is_diff:
            raise ValueError('full phase scene required')
        # Resetting this object's allowances must be a no-op. Otherwise a
        # historical broad ACM exemption could silently bypass certification.
        from types import SimpleNamespace
        checked_acm = SimpleNamespace(allowed_collision_matrix=copy.deepcopy(scene.allowed_collision_matrix))
        _reset_payload_contacts(checked_acm, pickup_object_id)
        if checked_acm.allowed_collision_matrix != scene.allowed_collision_matrix:
            raise ValueError('payload contact policy differs from frozen phase policy')
        objects = [obj for obj in scene.world.collision_objects if obj.id == pickup_object_id]
        attached = scene.robot_state.attached_collision_objects
        if phase == 'carry':
            expected = _box(pickup_object_id, envelope.dimensions_wdh,
                Transform(envelope.t_tcp_box_translation, envelope.t_tcp_box_quaternion_xyzw),
                'suction_contact_frame')
            if (objects or len(attached) != 1 or not same_payload_object(attached[0].object,expected)
                    or attached[0].link_name != 'suction_contact_frame'
                    or set(attached[0].touch_links) != set(SUCTION_TOUCH_LINKS)):
                raise ValueError('carrying phase lacks the exact predicted attachment')
        else:
            nominal = world_from_pick_box if phase == 'pick' else world_from_settled_box
            xyz, quat = geometry.conservative_pose_at_box(nominal.translation, nominal.quaternion_xyzw)
            expected = _box(pickup_object_id, envelope.dimensions_wdh, Transform(xyz, quat), 'world')
            if attached or len(objects) != 1 or not same_payload_object(objects[0],expected):
                raise ValueError('world phase lacks the exact predicted box obstacle')


def build_operation_scenes(snapshot, *, pickup_object_id, geometry,
                           world_from_pick_box, world_from_settled_box):
    """Nominal box poses in world; uncertainty is already frozen by its owner.

    Existing attachments reject: preparing another complete pick while carrying
    is not an authorized transition. Generated scenes never reach ApplyScene.
    """
    from moveit_msgs.msg import AttachedCollisionObject, PlanningScene
    from rclpy.serialization import deserialize_message, serialize_message
    if not isinstance(snapshot, Snapshot) or not isinstance(geometry, AttachmentGeometry):
        raise ValueError("bound snapshot and frozen predicted geometry required")
    if not isinstance(pickup_object_id, str) or not pickup_object_id.strip():
        raise ValueError("explicit measured pickup object association required")
    if not all(isinstance(p, Transform) for p in (world_from_pick_box, world_from_settled_box)):
        raise ValueError("world-frame nominal box poses required")
    scene = deserialize_message(snapshot.scene_cdr, PlanningScene)
    if scene.is_diff or scene.robot_state.is_diff or scene.robot_state.attached_collision_objects:
        raise ValueError("full unloaded baseline scene required")
    # This identity is owned by pickup, never a prefix-based purge of obstacles.
    scene.world.collision_objects = [obj for obj in scene.world.collision_objects
                                     if obj.id != pickup_object_id]
    _reset_payload_contacts(scene, pickup_object_id)
    envelope = geometry.conservative
    snapshots = {}
    for phase in PHASES:
        private = copy.deepcopy(scene)
        private.name = "joint-operation:"+phase
        if phase == "carry":
            attachment = AttachedCollisionObject()
            attachment.link_name = "suction_contact_frame"
            attachment.touch_links = list(SUCTION_TOUCH_LINKS)
            attachment.object = _box(pickup_object_id, envelope.dimensions_wdh,
                Transform(envelope.t_tcp_box_translation, envelope.t_tcp_box_quaternion_xyzw),
                "suction_contact_frame")
            private.robot_state.attached_collision_objects = [attachment]
        else:
            nominal = world_from_pick_box if phase == "pick" else world_from_settled_box
            xyz, quat = geometry.conservative_pose_at_box(nominal.translation, nominal.quaternion_xyzw)
            private.world.collision_objects.append(_box(pickup_object_id,
                envelope.dimensions_wdh, Transform(xyz, quat), "world"))
        raw = serialize_message(private)
        snapshots[phase] = replace(snapshot, scene_cdr=raw,
            scene_content_hash=hashlib.sha256(raw).hexdigest(), proposal_scene_hash="")
    return OperationScenes(snapshot.identity, pickup_object_id,
                           snapshots["pick"], snapshots["carry"], snapshots["retreat"])
