"""Place the frozen tool-frame envelope without altering nominal box targets."""
from st09_contracts.attachment import compose_transform, invert_transform, transform_point
from st09_contracts.payload_envelope import collision_payload


def geometry_at_box_pose(attachment, box_position, box_quaternion):
    """Envelope pose in the same frame as a nominal box pose."""
    geometry = collision_payload(attachment)
    if getattr(attachment, 'collision_envelope', None) is None:
        return tuple(box_position), tuple(box_quaternion), geometry.dimensions_wdh
    box_tool = invert_transform(attachment.t_tcp_box_translation,
                                attachment.t_tcp_box_quaternion_xyzw)
    frame_tool = compose_transform(box_position, box_quaternion, *box_tool)
    position, quaternion = compose_transform(*frame_tool,
        geometry.t_tcp_box_translation, geometry.t_tcp_box_quaternion_xyzw)
    return position, quaternion, geometry.dimensions_wdh
