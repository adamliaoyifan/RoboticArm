"""Replay captured hardware planning inputs without synthesizing attachment.

The recorded Select request supplies the start/map context. Attachment comes
from its separate captured controller response, never from a proposed grasp.
The live graph must independently provide the matching scene and map.
"""
from luggage_planning.hardware_place_planning import attachment_from_response, plan_attached_box
from luggage_planning.placement_prior.adoption import _pose_equal
from luggage_planning.typed_driver_adapter import tcp_record


def replay_place(adapter, request, attachment_response, *, attachment_request, scene_revision,
                 prior=None, choice_index=None):
    if (request.surface.identity.scene_revision != scene_revision
            or request.grasp.identity.scene_revision != scene_revision
            or request.start_tcp.identity.scene_revision != scene_revision):
        raise ValueError('capture does not match live scene identity')
    if (request.start_tcp.identity.frame_id != 'container_link'
            or request.start_tcp.reference != 'suction_contact_frame'
            or request.start_tcp.orientation_semantics != 'tool_quaternion'
            or request.container_source_frame != 'container_link'
            or request.container_target_frame != 'world'
            or not request.container_stamp_is_null):
        raise ValueError('unsupported captured start/transform convention')
    if (not attachment_request.enable
            or attachment_request.scene_revision != scene_revision
            or attachment_request.box_id != request.box_id
            or attachment_request.generation != request.generation):
        raise ValueError('captured attachment request belongs to another object/session')
    attachment = attachment_from_response(attachment_response,
        request_id=attachment_request.request_id,
        box_id=request.box_id, generation=request.generation,
        scene_revision=scene_revision)
    if (tuple(attachment.box_wdh) != tuple(request.box_wdh)
            or not _pose_equal(attachment_response.measured_grasp.pose, request.grasp.pose)
            or attachment.stamp_ns != request.grasp.identity.stamp_ns):
        raise ValueError('captured Select and measured attachment disagree')
    if choice_index is not None:
        if prior is None or not 0 <= choice_index < len(prior.choices):
            raise ValueError('selected prior choice is absent')
        choice = prior.choices[choice_index]
    else:
        choice = None
    p, q = request.start_tcp.pose.position, request.start_tcp.pose.orientation
    return plan_attached_box(adapter, attachment, request_id=request.request_id,
        surface=request.surface, inner_size=request.inner_size,
        start_tcp=tcp_record(request.request_id,scene_revision,
                             [p.x,p.y,p.z],[q.x,q.y,q.z,q.w]),
        start_joints=request.start_joints,
        container_transform=(request.container_source_frame,request.container_target_frame,
                             request.container_translation,request.container_quaternion_xyzw),
        prior=prior, selected_choice=choice, timeout_sec=60.)
