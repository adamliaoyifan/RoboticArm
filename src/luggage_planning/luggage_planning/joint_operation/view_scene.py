"""Empty-arm view scene preserves O/static contents and forbids box contact."""
from dataclasses import replace

from luggage_planning.joint_operation.contracts import OPERATION_JOINTS
from luggage_planning.joint_operation.scenes import _box, _reset_payload_contacts, same_payload_object
from luggage_planning.placement_prior.contracts import Transform


def fixed_tool_from_optical(model, optical_frame_id, joints):
    """Derive the rigid mount from the exact model, never a guessed offset.

    Matching moving-joint paths ensure the relative camera/tool pose is fixed,
    including sensors on a sibling fixed branch of the final robot link.
    """
    path = model.urdf.path_to(optical_frame_id)
    moving = tuple(name for name in path if model.urdf.joints[name]['type'] != 'fixed')
    if not path or moving != OPERATION_JOINTS:
        raise ValueError('CAMERA_SAME_RIGID_WRIST_BRANCH_REQUIRED')
    tool = Transform.from_matrix(model.urdf.tcp(joints,model.path))
    camera = Transform.from_matrix(model.urdf.tcp(joints,path))
    return tool.inverse().compose(camera)


def pickup_view_scene(snapshot, domain, pickup_object_id, *, admit_static_scene):
    """Add only the measured target to a full scene; no map or scene-owner write."""
    from moveit_msgs.msg import PlanningScene
    from rclpy.serialization import deserialize_message, serialize_message
    if not callable(admit_static_scene) or not isinstance(pickup_object_id,str) or not pickup_object_id.strip():
        raise ValueError('STATIC_ADMISSION_AND_MEASURED_PICKUP_ID_REQUIRED')
    scene = deserialize_message(snapshot.scene_cdr,PlanningScene)
    if scene.is_diff or scene.robot_state.is_diff:
        raise ValueError('FULL_PICKUP_VIEW_SCENE_REQUIRED')
    if scene.robot_state.attached_collision_objects:
        raise ValueError('PICKUP_VIEW_REQUIRES_EMPTY_ARM')
    if admit_static_scene(scene) is not scene:  # Configured owner returns the admitted scene or raises.
        raise ValueError('STATIC_VIEW_SCENE_ADMISSION_REFUSED')
    target = _box(pickup_object_id,domain.box_wdh,
        Transform(domain.box_xyz_world,domain.box_quaternion_xyzw),'world')
    matches = [obj for obj in scene.world.collision_objects if obj.id == pickup_object_id]
    if len(matches)>1 or (matches and not same_payload_object(matches[0],target)):
        raise ValueError('PICKUP_VIEW_OBJECT_ASSOCIATION_CONFLICT')
    if not matches:
        scene.world.collision_objects.append(target)
    _reset_payload_contacts(scene,pickup_object_id,allow_suction_touch=False)
    return replace(snapshot,scene_cdr=serialize_message(scene),scene_content_hash='',proposal_scene_hash='')
