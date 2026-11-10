"""Lossless transport of Attachment output; no tolerance calculation here."""
from st09_contracts._base import ContractViolation, finite_tuple, positive_dimensions, unit_quaternion
from st09_contracts.attachment_geometry import AttachmentGeometry
from st09_contracts.payload_envelope import EnvelopeLimits, PayloadEnvelope


def geometry_to_message(geometry):
    from luggage_msgs.msg import AttachmentGeometryV2
    result = AttachmentGeometryV2()
    envelope = geometry.conservative
    result.measured_dimensions_wdh = list(geometry.measured_dimensions_wdh)
    result.tool_box_translation = list(geometry.t_tool_box_translation)
    result.tool_box_quaternion_xyzw = list(geometry.t_tool_box_quaternion_xyzw)
    result.conservative_dimensions_wdh = list(envelope.dimensions_wdh)
    result.tool_conservative_translation = list(envelope.t_tcp_box_translation)
    result.tool_conservative_quaternion_xyzw = list(envelope.t_tcp_box_quaternion_xyzw)
    limits = envelope.limits
    result.yaw_half_range_deg = float(limits.yaw_half_range_deg)
    result.size_growth_fraction_wdh = list(limits.size_growth_fraction_wdh)
    result.position_half_range_tool_xy_m = list(limits.position_half_range_tool_xy_m)
    result.away_from_contact_position_error_m = float(limits.away_from_contact_position_error_m)
    result.nominal_digest = envelope.nominal_digest
    result.geometry_digest = geometry.digest()
    return result


def geometry_from_message(message):
    if not message.geometry_digest:
        raise ContractViolation('ATTACHMENT_GEOMETRY_MISSING')
    limits = EnvelopeLimits(float(message.yaw_half_range_deg),
        tuple(message.size_growth_fraction_wdh),
        tuple(message.position_half_range_tool_xy_m),
        float(message.away_from_contact_position_error_m))
    envelope = PayloadEnvelope(
        positive_dimensions(message.conservative_dimensions_wdh, 'conservative_dimensions'),
        finite_tuple(message.tool_conservative_translation, 3, 'tool_conservative_translation'),
        unit_quaternion(message.tool_conservative_quaternion_xyzw, 'tool_conservative_quaternion'),
        limits, str(message.nominal_digest))
    result = AttachmentGeometry(tuple(message.measured_dimensions_wdh),
        tuple(message.tool_box_translation), tuple(message.tool_box_quaternion_xyzw), envelope)
    if result.digest() != message.geometry_digest:
        raise ContractViolation('ATTACHMENT_GEOMETRY_DIGEST_MISMATCH')
    return result
