"""Same-frame rule: an unbound cargo cloud is not one suitcase.

Multiple detections plus an instance mask that was observed and contains
no positive id means ``filter_depth`` kept every cargo-labelled pixel.
That measurement cannot authorize a pick, even when the support check
passes. A missing mask is not this verdict. A mask that binds an instance
stays usable when other detections are also present. An older frame is
never substituted.
"""

DETECT_TARGET_ASSOCIATION_AMBIGUOUS = "DETECT_TARGET_ASSOCIATION_AMBIGUOUS"


def target_association_ambiguous(n_detections, instance_positive_pixels):
    """``instance_positive_pixels`` is None when this stamp had no mask."""
    if instance_positive_pixels is None:
        return False
    return int(n_detections) >= 2 and int(instance_positive_pixels) <= 0


def admit_pick_frame(current_ambiguous, current_box):
    """The current frame only. ``current_box`` is not replaced by an older one."""
    if current_ambiguous:
        return None
    return current_box
