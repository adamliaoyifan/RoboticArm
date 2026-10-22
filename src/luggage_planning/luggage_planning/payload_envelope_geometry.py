"""Place the frozen tool-frame envelope without altering nominal box targets."""
from itertools import product
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


def slot_envelope_check(attachment, candidate, container_geometry):
    """Closed-hull containment, including floor; no implicit support exemption."""
    from luggage_description.container_geometry import contains_point
    position, quaternion, dimensions = geometry_at_box_pose(
        attachment, candidate.position_container, candidate.quaternion_xyzw_container)
    corners = [transform_point(position, quaternion,
        tuple(s*d/2 for s,d in zip(signs, dimensions)))
        for signs in product((-1,1), repeat=3)]
    outside = [p for p in corners if not contains_point(container_geometry, p)]
    return dict(inside=not outside, outside_corners=outside,
                envelope_center_container=position, envelope_dimensions=dimensions,
                envelope_bottom_container=min(p[2] for p in corners),
                container_floor_z=container_geometry.floor_z,
                envelope_digest=attachment.collision_envelope.digest())
