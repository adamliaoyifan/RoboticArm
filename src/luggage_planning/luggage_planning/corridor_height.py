"""Measured obstacle height under a conservative swept payload footprint."""

from luggage_description.frame_transforms import quaternion_product, rotate_vector


def half_extents(dimensions, quaternion):
    axes = [rotate_vector(quaternion, axis) for axis in
            ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    return tuple(sum(abs(axes[j][i]) * dimensions[j] / 2
                     for j in range(3)) for i in range(3))


def required_corridor_height(portal, approach, tcp_quaternion, attachment,
                             obstacles, margin):
    """Return top/TCP height and obstacle ids, or None for no overlap.

    Sweep the attached payload's world AABB along the straight transit.
    AABB overlap is conservative for rotated footprints; final certification
    remains authoritative. Obstacles already include the configured padding.
    This function neither changes the box goal nor writes any scene.
    """
    offset = rotate_vector(tcp_quaternion, attachment.t_tcp_box_translation)
    orientation = quaternion_product(
        tcp_quaternion, attachment.t_tcp_box_quaternion_xyzw)
    half = half_extents(attachment.dimensions_wdh, orientation)
    low = [min(portal[i], approach[i]) + offset[i] - half[i] for i in range(2)]
    high = [max(portal[i], approach[i]) + offset[i] + half[i] for i in range(2)]
    matches = []
    for box in obstacles:
        extent = half_extents(box.size_whd, box.quaternion_xyzw)
        if all(box.center[i] + extent[i] >= low[i]
               and box.center[i] - extent[i] <= high[i] for i in range(2)):
            matches.append((box.object_id, box.center[2] + extent[2]))
    if not matches:
        return None
    top = max(value for _, value in matches)
    return {"obstacle_ids": [key for key, _ in matches], "top_world_z": top,
            "required_tcp_world_z": top - offset[2] + half[2] + margin}
