"""ROS boundary for optional exact acquisition matching inputs.

Read only the supplied same-stamp RGB history entry and full received message;
never search a newer frame or remove competing candidates.
"""
from luggage_perception.pickup_view_association import (
    MAX_REPORTED_CANDIDATES, BufferedPickupRgb, PickupAssociationInputs,
    ReportedPickupCandidate, selected_candidate_index,
)


def association_inputs_from_detection(message, rgb, *, stamp_ns, optical_frame_id,
                                      image_shape, selected_bbox):
    try:
        if rgb is None:
            raise ValueError('PICKUP_ASSOCIATION_EXACT_RGB_UNAVAILABLE')
        expected = (stamp_ns, optical_frame_id)
        observed = (int(message.header.stamp.sec)*10**9+int(message.header.stamp.nanosec),
                    message.header.frame_id)
        if (type(rgb) is not BufferedPickupRgb or observed != expected
                or (rgb.stamp_ns, rgb.optical_frame_id) != expected
                or tuple(rgb.shape[:2]) != tuple(image_shape)
                or (int(message.image_height), int(message.image_width)) != tuple(image_shape)):
            raise ValueError('PICKUP_ASSOCIATION_SOURCE_MISMATCH')
        if not 0 < len(message.detections) <= MAX_REPORTED_CANDIDATES:
            raise ValueError('PICKUP_ASSOCIATION_POOL_EMPTY_OR_OVER_BUDGET')
        pool = tuple(ReportedPickupCandidate(int(c.label), c.prompt, float(c.confidence),
            tuple(int(v) for v in c.bbox), bool(c.held)) for c in message.detections)
        selected = selected_candidate_index(pool)
        if tuple(selected_bbox) != pool[selected].bbox:
            raise ValueError('PICKUP_ASSOCIATION_SELECTED_ROI_MISMATCH')
        return PickupAssociationInputs(rgb, pool, selected, int(message.frame_seq),
            int(message.generation), message.instance_id, message.backend), 'PICKUP_ASSOCIATION_INPUTS_AVAILABLE'
    except (AttributeError, TypeError, ValueError, OverflowError) as error:
        return None, 'PICKUP_ASSOCIATION_UNAVAILABLE:'+str(error)
