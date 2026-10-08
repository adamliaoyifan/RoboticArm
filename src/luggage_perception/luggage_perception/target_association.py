"""Same-frame rule: a cargo cloud authorizes a pick only with one target.

One detection on this stamp is that target. Several detections are one
target only when this stamp's instance mask contains a positive id.
A missing mask and an all-zero mask are the same refusal in that case:
both leave ``filter_depth`` with every cargo-labelled pixel. A detection
count, instance map, or size from another stamp is not association, and
the latest statistics are not a substitute. An older frame is never used.
"""

DETECT_TARGET_ASSOCIATION_AMBIGUOUS = "DETECT_TARGET_ASSOCIATION_AMBIGUOUS"


def association_inputs_aligned(detection_stamp, cloud_stamp, size_stamp,
                               instance_stamp=None):
    """Detection count, cloud, and size share one stamp.

    ``instance_stamp is None`` means this stamp has no mask message.
    A mask message from another stamp does not count as this frame.
    """
    if (detection_stamp is None or cloud_stamp is None
            or size_stamp is None):
        return False
    if detection_stamp != cloud_stamp or size_stamp != cloud_stamp:
        return False
    if instance_stamp is not None and instance_stamp != cloud_stamp:
        return False
    return True


def target_association_ambiguous(n_detections, instance_positive_pixels,
                                 stamps_aligned=True):
    """True when this stamp does not name one target.

    ``instance_positive_pixels`` is None when this stamp has no mask.
    ``stamps_aligned`` is False when the count, mask, or size is from
    another stamp. ``n_detections is None`` means the count was not taken
    from this stamp.
    """
    if not stamps_aligned or n_detections is None:
        return True
    if int(n_detections) < 2:
        return False
    if instance_positive_pixels is None:
        return True
    return int(instance_positive_pixels) <= 0


def admit_pick_frame(current_ambiguous, current_box):
    """The current frame only. ``current_box`` is not replaced by an older one."""
    if current_ambiguous:
        return None
    return current_box
