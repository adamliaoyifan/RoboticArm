"""Bounded open-air TCP proposals; edges still require full certification."""

from luggage_description.frame_transforms import rotate_vector

# Search extents, not safety clearances. Never lower the baseline anchor.
HORIZONTAL_SEARCH_M = 0.15
UPWARD_SEARCH_M = 0.20


def staging_positions(baseline, container_quaternion):
    """Baseline first, then four container-axis offsets and one world lift.

    Only the transit anchor changes. Placement, attachment, portal and
    collision margins remain the caller's original values. This is a finite
    proposal set, not a claim that its endpoints or connecting paths are safe.
    """
    if baseline is None:
        return ()
    origin = tuple(float(v) for v in baseline)
    positions = [origin]
    for delta in ((HORIZONTAL_SEARCH_M, 0, 0),
                  (-HORIZONTAL_SEARCH_M, 0, 0),
                  (0, HORIZONTAL_SEARCH_M, 0),
                  (0, -HORIZONTAL_SEARCH_M, 0)):
        offset = rotate_vector(container_quaternion, delta)
        point = tuple(origin[i] + offset[i] for i in range(3))
        # A tilted container must not lower the world transit floor.
        if point[2] >= origin[2] - 1e-9:
            positions.append(point)
    positions.append((origin[0], origin[1], origin[2] + UPWARD_SEARCH_M))
    return tuple(dict.fromkeys(positions))
