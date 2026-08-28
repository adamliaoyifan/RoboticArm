#!/usr/bin/env python3
"""Load scene_tf.yaml and compute world-frame static transforms."""

from __future__ import division

import math
import os

import yaml

from luggage_description._share import ENV_SCENE_TF, description_config_path


def default_scene_tf_config_path():
    return description_config_path("scene_tf.yaml.example")


def load_scene_tf_config(path=None):
    path = path or default_scene_tf_config_path()
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def resolve_scene_tf_config_path(path=None):
    if path:
        return path
    env = os.environ.get(ENV_SCENE_TF, "")
    if env:
        return env
    return default_scene_tf_config_path()


def resolve_scene_config(scene_config=None, scene_config_path=None):
    if scene_config is not None:
        return scene_config
    return load_scene_tf_config(scene_config_path or resolve_scene_tf_config_path())


def world_frame(config):
    return str(config.get("world_frame", "world"))


def robot_base_frame(config):
    robot = config.get("robot", {})
    return str(robot.get("base_frame", "elfin_base_link"))


def pedestal_link_frame(_config=None):
    return "pedestal_link"


def pedestal_config(config):
    pedestal = config.get("pedestal", {})
    return {
        "enabled": bool(pedestal.get("enabled", False)),
        "size": [float(v) for v in pedestal.get("size", [1.05, 0.74, 0.86])],
        "translation": [float(v) for v in pedestal.get("translation", [0.0, 0.0, 0.0])],
        "rotation_rpy": [float(v) for v in pedestal.get("rotation_rpy", [0.0, 0.0, 0.0])],
        "gazebo_model": str(pedestal.get("gazebo_model", "robot_pedestal")),
    }


def pedestal_enabled(config):
    return pedestal_config(config)["enabled"]


def pedestal_dimensions(config):
    length, width, height = pedestal_config(config)["size"]
    return length, width, height


def robot_base_height(config):
    """World-frame Z of elfin_base_link (pedestal top or robot.base_height fallback)."""
    if pedestal_enabled(config):
        _length, _width, height = pedestal_dimensions(config)
        return height
    robot = config.get("robot", {})
    return float(robot.get("base_height", 0.1))


def robot_base_rotation_rpy(config):
    robot = config.get("robot", {})
    return [float(v) for v in robot.get("rotation_rpy", [0.0, 0.0, 0.0])]


def urdf_world_base_pose(config):
    """Return the URDF world_base pose using the same math as planning.

    robot_base_in_world() is the single source of truth for the robot base pose.
    Keeping URDF/Gazebo and planning on this transform prevents RViz/Gazebo
    drift when the pedestal has translation or yaw.
    """
    return robot_base_in_world(config)


def pedestal_in_world(config):
    ped = pedestal_config(config)
    return list(ped["translation"]), list(ped["rotation_rpy"])


def robot_base_on_pedestal(config):
    _length, _width, height = pedestal_dimensions(config)
    return [0.0, 0.0, height], robot_base_rotation_rpy(config)


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _mat_vec(mat, vec):
    return [
        mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2],
        mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2],
        mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2],
    ]


def _matmul(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def _matrix_to_rpy(rot):
    sy = math.sqrt(rot[0][0] * rot[0][0] + rot[1][0] * rot[1][0])
    if sy > 1e-6:
        roll = math.atan2(rot[2][1], rot[2][2])
        pitch = math.atan2(-rot[2][0], sy)
        yaw = math.atan2(rot[1][0], rot[0][0])
    else:
        roll = math.atan2(-rot[1][2], rot[1][1])
        pitch = math.atan2(-rot[2][0], sy)
        yaw = 0.0
    return [roll, pitch, yaw]


def _invert_transform(xyz, rpy):
    rot = _rpy_matrix(*rpy)
    inv_rot = [
        [rot[0][0], rot[1][0], rot[2][0]],
        [rot[0][1], rot[1][1], rot[2][1]],
        [rot[0][2], rot[1][2], rot[2][2]],
    ]
    t_inv = _mat_vec(inv_rot, [-xyz[0], -xyz[1], -xyz[2]])
    return t_inv, _matrix_to_rpy(inv_rot)


def _compose(xyz_a, rpy_a, xyz_b, rpy_b):
    rot_a = _rpy_matrix(*rpy_a)
    rot_b = _rpy_matrix(*rpy_b)
    rot = _matmul(rot_a, rot_b)
    t = _mat_vec(rot_a, xyz_b)
    t = [t[0] + xyz_a[0], t[1] + xyz_a[1], t[2] + xyz_a[2]]
    return t, _matrix_to_rpy(rot)


def robot_base_in_world(config):
    if pedestal_enabled(config):
        ped_xyz, ped_rpy = pedestal_in_world(config)
        base_xyz, base_rpy = robot_base_on_pedestal(config)
        return _compose(ped_xyz, ped_rpy, base_xyz, base_rpy)
    height = robot_base_height(config)
    return [0.0, 0.0, height], robot_base_rotation_rpy(config)


def _pose_entry(parent, child, translation, rotation_rpy):
    return {
        "parent": parent,
        "child": child,
        "translation": [float(v) for v in translation],
        "rotation_rpy": [float(v) for v in rotation_rpy],
    }


def static_transforms(config):
    """Static TF entries published by container_tf_publisher (not robot base).

    Robot base pose is owned by URDF world_base via robot_state_publisher.
    Use robot_base_in_world() for planning math; do not duplicate base TF here.
    """
    world = world_frame(config)
    transforms = []

    if pedestal_enabled(config):
        ped_xyz, ped_rpy = pedestal_in_world(config)
        transforms.append(
            _pose_entry(world, pedestal_link_frame(config), ped_xyz, ped_rpy)
        )

    if pickup_platform_enabled(config):
        plat_xyz, plat_rpy = pickup_platform_in_world(config)
        transforms.append(
            _pose_entry(world, "pickup_platform_link", plat_xyz, plat_rpy)
        )
        plat = pickup_platform_config(config)
        _length, _width, height = plat["size"]
        transforms.append(
            _pose_entry("pickup_platform_link", "pickup_platform_top", [0.0, 0.0, height], [0.0, 0.0, 0.0])
        )

    for item in config.get("static_transforms", []):
        transforms.append(
            _pose_entry(
                str(item.get("parent", world)),
                str(item.get("child", "")),
                item.get("translation", [0.0, 0.0, 0.0]),
                item.get("rotation_rpy", [0.0, 0.0, 0.0]),
            )
        )
    return transforms


def world_transform(config, child_frame):
    """Return translation/rpy of child_frame expressed in world."""
    world = world_frame(config)
    for item in static_transforms(config):
        if item["child"] == child_frame and item["parent"] == world:
            return item["translation"], item["rotation_rpy"]
    raise KeyError("No world transform for frame '%s'" % child_frame)


def origin_in_world(config):
    """Container link pose in world (alias for world_transform container_link)."""
    return world_transform(config, "container_link")


def base_in_world(config):
    return robot_base_in_world(config)


def container_semantic_config(config):
    return config.get("container", {})


def container_outer_dimensions(config):
    outer = container_semantic_config(config).get("outer", {})
    return (
        float(outer.get("length", 2.4)),
        float(outer.get("width", 2.0)),
        float(outer.get("height", 2.2)),
    )


def container_inner_dimensions(config):
    """Return legacy inner X/Y extents and upper-Z coordinate.

    ``inner.height`` historically meant the upper container-link Z coordinate
    because the semantic volume started at Z=0. New code that needs the usable
    cargo span must use :func:`container_usable_dimensions`.
    """
    inner = container_semantic_config(config).get("inner", {})
    return (
        float(inner.get("length", 2.3)),
        float(inner.get("width", 1.9)),
        float(inner.get("height", 2.1)),
    )


def container_inner_floor_z(config):
    """Usable cargo-floor elevation in ``container_link``.

    Legacy configurations omit ``floor_z`` and retain the old Z=0 behavior.
    """
    inner = container_semantic_config(config).get("inner", {})
    return float(inner.get("floor_z", 0.0))


def container_inner_ceiling_z(config):
    """Usable cargo ceiling elevation in ``container_link``."""
    inner = container_semantic_config(config).get("inner", {})
    return float(inner.get("ceiling_z", inner.get("height", 2.1)))


def container_usable_dimensions(config):
    """Return usable cargo length, width and vertical span."""
    inner_l, inner_w, _legacy_height = container_inner_dimensions(config)
    floor_z = container_inner_floor_z(config)
    ceiling_z = container_inner_ceiling_z(config)
    if ceiling_z <= floor_z:
        raise ValueError(
            "container inner ceiling_z must be greater than floor_z")
    return inner_l, inner_w, ceiling_z - floor_z


def _cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _normalize(vec):
    norm = math.sqrt(sum(v * v for v in vec))
    if norm < 1e-9:
        return list(vec)
    return [v / norm for v in vec]


def _parse_aperture_corners_config(config):
    """Return four aperture corners in container_link, or None for legacy mode."""
    aperture = container_semantic_config(config).get("opening", {}).get("aperture", {})
    raw = aperture.get("corners")
    if not raw:
        return None
    if len(raw) != 4:
        raise ValueError("opening.aperture.corners must contain exactly four points")
    return [[float(v) for v in corner] for corner in raw]


def _legacy_opening_width_height(config):
    opening = container_semantic_config(config).get("opening", {})
    inner_l, _inner_w, inner_h = container_inner_dimensions(config)
    return (
        float(opening.get("width", inner_l)),
        float(opening.get("height", inner_h)),
    )


def container_opening_axes_in_container(config):
    """Return orthonormal (normal, lateral, vertical) opening axes in container_link."""
    axis, sign = container_opening_side(config)
    normal = [0.0, 0.0, 0.0]
    normal[axis] = sign
    vertical = [0.0, 0.0, 1.0]
    lateral = _normalize(_cross(vertical, normal))
    if all(abs(v) < 1e-6 for v in lateral):
        lateral = _normalize(_cross([0.0, 1.0, 0.0], normal))
    vertical = _normalize(_cross(normal, lateral))
    if vertical[2] < 0.0:
        lateral = [-v for v in lateral]
        vertical = [-v for v in vertical]
    return normal, lateral, vertical


def _aperture_bounds_from_corners(corners, lateral, vertical):
    """Return centroid and lateral/vertical min/max offsets for aperture corners."""
    centroid = [sum(corner[i] for corner in corners) / 4.0 for i in range(3)]
    lats = [
        sum((corner[i] - centroid[i]) * lateral[i] for i in range(3))
        for corner in corners
    ]
    verts = [
        sum((corner[i] - centroid[i]) * vertical[i] for i in range(3))
        for corner in corners
    ]
    return centroid, min(lats), max(lats), min(verts), max(verts)


def _corners_from_aperture_bounds(
        centroid, lat_min, lat_max, vert_min, vert_max, lateral, vertical):
    """Rebuild four CCW corners from tangent bounds relative to centroid."""
    return [
        [
            centroid[i] + lat * lateral[i] + vert * vertical[i]
            for i in range(3)
        ]
        for lat, vert in (
            (lat_min, vert_min),
            (lat_max, vert_min),
            (lat_max, vert_max),
            (lat_min, vert_max),
        )
    ]


def _legacy_aperture_corners_in_container(config):
    open_xyz, _ = container_opening_in_container(config)
    width, height = _legacy_opening_width_height(config)
    _normal, lateral, vertical = container_opening_axes_in_container(config)
    half_w = width * 0.5
    half_h = height * 0.5
    return _corners_from_aperture_bounds(
        open_xyz, -half_w, half_w, -half_h, half_h, lateral, vertical
    )


def _inset_aperture_corners_local(corners, lateral, vertical, margin):
    """Inset a planar rectangular aperture uniformly along lateral/vertical axes."""
    clearance = max(0.0, float(margin))
    centroid, lat_min, lat_max, vert_min, vert_max = _aperture_bounds_from_corners(
        corners, lateral, vertical
    )
    lat_min += clearance
    lat_max -= clearance
    vert_min += clearance
    vert_max -= clearance
    if lat_min > lat_max or vert_min > vert_max:
        return []
    return _corners_from_aperture_bounds(
        centroid, lat_min, lat_max, vert_min, vert_max, lateral, vertical
    )


def container_opening_aperture_corners_in_container(config, margin=0.0):
    """Return four aperture corners in container_link, inset by margin."""
    corners = _parse_aperture_corners_config(config)
    if corners is None:
        corners = _legacy_aperture_corners_in_container(config)
    _normal, lateral, vertical = container_opening_axes_in_container(config)
    return _inset_aperture_corners_local(corners, lateral, vertical, margin)


def _point_in_container_link(point, config):
    base_xyz, base_rpy = container_in_base_link(config)
    rotation = _rpy_matrix(*base_rpy)
    delta = [float(point[i]) - base_xyz[i] for i in range(3)]
    return [
        sum(rotation[row][axis] * delta[row] for row in range(3))
        for axis in range(3)
    ]


def _local_point_to_base_link(local_xyz, config):
    base_xyz, base_rpy = container_in_base_link(config)
    offset, _ = _compose([0.0, 0.0, 0.0], base_rpy, local_xyz, [0.0, 0.0, 0.0])
    return [base_xyz[i] + offset[i] for i in range(3)]


def container_opening_aperture_lateral_offsets(config, num_lateral, margin=0.0):
    """Return lateral offsets from opening target for interior probe sampling."""
    opening = container_opening_target_point(config)
    _normal, lateral, _vertical = container_opening_axes_in_base_link(config)
    corners_local = container_opening_aperture_corners_in_container(config, margin)
    if not corners_local:
        return [0.0]
    corners_base = [_local_point_to_base_link(corner, config) for corner in corners_local]
    centroid_base = [sum(c[i] for c in corners_base) / 4.0 for i in range(3)]
    lats = [
        sum((corner[i] - centroid_base[i]) * lateral[i] for i in range(3))
        for corner in corners_base
    ]
    lat_min, lat_max = min(lats), max(lats)
    ref_lat = sum((opening[i] - centroid_base[i]) * lateral[i] for i in range(3))
    if num_lateral <= 1:
        return [0.0]
    return [
        lat_min + (lat_max - lat_min) * float(i) / float(num_lateral - 1) - ref_lat
        for i in range(num_lateral)
    ]


def container_opening_dimensions(config):
    opening = container_semantic_config(config).get("opening", {})
    inner_l, _inner_w, inner_h = container_inner_dimensions(config)
    corners = _parse_aperture_corners_config(config)
    if corners:
        _normal, lateral, vertical = container_opening_axes_in_container(config)
        _centroid, lat_min, lat_max, vert_min, vert_max = _aperture_bounds_from_corners(
            corners, lateral, vertical
        )
        return (lat_max - lat_min, vert_max - vert_min)
    return (
        float(opening.get("width", inner_l)),
        float(opening.get("height", inner_h)),
    )


def container_opening_in_container(config):
    """Return container_opening_frame pose in container_link."""
    for item in static_transforms(config):
        if item["parent"] == "container_link" and item["child"] == "container_opening_frame":
            return item["translation"], item["rotation_rpy"]
    opening = container_semantic_config(config).get("opening", {})
    frame = opening.get("frame", {})
    return (
        [float(v) for v in frame.get("xyz", [0.0, 1.0, 1.0])],
        [float(v) for v in frame.get("rpy", [0.0, 0.0, 0.0])],
    )


def container_in_base_link(config):
    """Return xyz/rpy of container_link expressed in elfin_base_link."""
    world_t, world_r = origin_in_world(config)
    base_t, base_r = robot_base_in_world(config)
    base_inv_t, base_inv_r = _invert_transform(base_t, base_r)
    return _compose(base_inv_t, base_inv_r, world_t, world_r)


def container_opening_in_base_link(config):
    """Return xyz/rpy of container_opening_frame in elfin_base_link."""
    base_c_t, base_c_r = container_in_base_link(config)
    open_t, open_r = container_opening_in_container(config)
    return _compose(base_c_t, base_c_r, open_t, open_r)


def container_opening_target_point(config):
    """Opening center as [x,y,z] in elfin_base_link."""
    xyz, _ = container_opening_in_base_link(config)
    return xyz


def container_opening_side(config):
    """Return the axis sign of the opening face in container_link frame.

    Looks up ``container.opening.side`` (e.g. ``positive_y`` / ``negative_y``).
    Returns a dict with ``axis`` (0=x, 1=y, 2=z) and ``sign`` (+1 or -1).
    Defaults to +Y to match the example scene.
    """
    side = str(
        container_semantic_config(config).get("opening", {}).get("side", "positive_y")
    ).strip().lower()
    mapping = {
        "positive_x": (0, 1.0),
        "negative_x": (0, -1.0),
        "positive_y": (1, 1.0),
        "negative_y": (1, -1.0),
        "positive_z": (2, 1.0),
        "negative_z": (2, -1.0),
    }
    return mapping.get(side, (1, 1.0))


def container_interior_center_in_base_link(config):
    """Legacy Z=0 semantic-volume center as [x,y,z] in elfin_base_link.

    Kept for the existing ``surface_map_2d`` contract, whose height values are
    absolute elevations above container-link Z=0. New spatial bounds should
    use :func:`container_usable_center_in_base_link`.
    """
    base_xyz, base_rpy = container_in_base_link(config)
    _inner_l, _inner_w, inner_h = container_inner_dimensions(config)
    # Apply the container_link rotation to the local [0, 0, inner_h/2] offset.
    offset_local = [0.0, 0.0, inner_h * 0.5]
    offset_world, _ = _compose([0.0, 0.0, 0.0], base_rpy, offset_local, [0.0, 0.0, 0.0])
    return [base_xyz[i] + offset_world[i] for i in range(3)]


def container_usable_center_in_base_link(config):
    """Usable cargo-volume center as [x,y,z] in ``elfin_base_link``."""
    base_xyz, base_rpy = container_in_base_link(config)
    center_z = 0.5 * (
        container_inner_floor_z(config) + container_inner_ceiling_z(config))
    offset_world, _ = _compose(
        [0.0, 0.0, 0.0], base_rpy,
        [0.0, 0.0, center_z], [0.0, 0.0, 0.0])
    return [base_xyz[i] + offset_world[i] for i in range(3)]


def container_inner_box_in_base_link(config):
    """Return (min_corner, max_corner) of the interior AABB in base_link.

    The interior box is axis-aligned in container_link (which has zero
    rotation in the example scene, so it is also axis-aligned in base_link
    when container_link rotation_rpy is zero). For non-zero container
    rotation, the AABB is a tight bounding box of the rotated interior.
    """
    base_xyz, base_rpy = container_in_base_link(config)
    inner_l, inner_w, _legacy_height = container_inner_dimensions(config)
    floor_z = container_inner_floor_z(config)
    ceiling_z = container_inner_ceiling_z(config)
    # Sample 8 corners of the interior box in container_link, transform each
    # to base_link, then take the per-axis min/max.
    corners_local = []
    for sx in (-0.5, 0.5):
        for sy in (-0.5, 0.5):
            for z in (floor_z, ceiling_z):
                corners_local.append([sx * inner_l, sy * inner_w, z])
    corners_base = []
    for cl in corners_local:
        offset, _ = _compose([0.0, 0.0, 0.0], base_rpy, cl, [0.0, 0.0, 0.0])
        corners_base.append([base_xyz[i] + offset[i] for i in range(3)])
    mins = [min(c[i] for c in corners_base) for i in range(3)]
    maxs = [max(c[i] for c in corners_base) for i in range(3)]
    return mins, maxs


def point_inside_container_inner_box(point, config, margin=0.0):
    """Return whether a base-link point is inside the oriented inner box.

    ``margin`` insets every face and is used as the conservative camera/link
    wall clearance. A margin that consumes an axis makes the box empty.
    """
    clearance = max(0.0, float(margin))
    base_xyz, base_rpy = container_in_base_link(config)
    rotation = _rpy_matrix(*base_rpy)
    delta = [float(point[i]) - base_xyz[i] for i in range(3)]
    # R maps container-local vectors into base_link; R^T performs the inverse.
    local = [
        sum(rotation[row][axis] * delta[row] for row in range(3))
        for axis in range(3)
    ]
    inner_l, inner_w, _legacy_height = container_inner_dimensions(config)
    floor_z = container_inner_floor_z(config)
    ceiling_z = container_inner_ceiling_z(config)
    limits = (
        (-inner_l * 0.5 + clearance, inner_l * 0.5 - clearance),
        (-inner_w * 0.5 + clearance, inner_w * 0.5 - clearance),
        (floor_z + clearance, ceiling_z - clearance),
    )
    for axis, (low, high) in enumerate(limits):
        if (
                low > high
                or local[axis] < low - 1e-9
                or local[axis] > high + 1e-9):
            return False
    return True


def container_opening_normal_in_base_link(config):
    """Unit vector pointing from the container interior toward the opening.

    In container_link, the opening face normal is +/-X, +/-Y, or +/-Z based
    on ``container.opening.side``. This is rotated into base_link using the
    container_link rotation_rpy.
    """
    axis, sign = container_opening_side(config)
    local_normal = [0.0, 0.0, 0.0]
    local_normal[axis] = sign
    _base_xyz, base_rpy = container_in_base_link(config)
    rotated, _ = _compose([0.0, 0.0, 0.0], base_rpy, local_normal, [0.0, 0.0, 0.0])
    norm = math.sqrt(sum(v * v for v in rotated))
    if norm < 1e-9:
        return [0.0, 0.0, 0.0]
    return [v / norm for v in rotated]


def task_roi_from_scene(config):
    """Derive task-ROI geometry in elfin_base_link from scene_tf.

    These used to be transcribed into launch files and injected into other
    nodes by ``description_params_node``. Keep the numbers here so callers
    (this package's TF node, later Phase 5) can read them without a global
    parameter server.
    """
    _base_xyz, base_rpy = container_in_base_link(config)
    aperture_width, aperture_height = container_opening_dimensions(config)
    return {
        "container_center": [
            float(v) for v in container_usable_center_in_base_link(config)],
        "container_dims": [
            float(v) for v in container_usable_dimensions(config)],
        "container_yaw": float(base_rpy[2]),
        "opening_center": [
            float(v) for v in container_opening_target_point(config)],
        "opening_normal": [
            float(v) for v in container_opening_normal_in_base_link(config)],
        "aperture_width": float(aperture_width),
        "aperture_height": float(aperture_height),
    }


def container_opening_axes_in_base_link(config):
    """Return orthonormal (normal, lateral, vertical) opening axes in base_link."""
    normal = container_opening_normal_in_base_link(config)
    vertical = [0.0, 0.0, 1.0]
    lateral = [
        vertical[1] * normal[2] - vertical[2] * normal[1],
        vertical[2] * normal[0] - vertical[0] * normal[2],
        vertical[0] * normal[1] - vertical[1] * normal[0],
    ]
    norm = math.sqrt(sum(v * v for v in lateral))
    if norm < 1e-9:
        lateral = [1.0, 0.0, 0.0]
    else:
        lateral = [v / norm for v in lateral]
    vertical = [
        normal[1] * lateral[2] - normal[2] * lateral[1],
        normal[2] * lateral[0] - normal[0] * lateral[2],
        normal[0] * lateral[1] - normal[1] * lateral[0],
    ]
    vnorm = math.sqrt(sum(v * v for v in vertical))
    vertical = [v / vnorm for v in vertical] if vnorm > 1e-9 else [0.0, 0.0, 1.0]
    if vertical[2] < 0.0:
        lateral = [-v for v in lateral]
        vertical = [-v for v in vertical]
    return normal, lateral, vertical


def container_opening_aperture_corners(config, margin=0.0):
    """Return the four safe opening corners in base_link, inset by margin."""
    corners_local = container_opening_aperture_corners_in_container(config, margin)
    return [_local_point_to_base_link(corner, config) for corner in corners_local]


def point_inside_opening_aperture(point, config, margin=0.0):
    """Return whether point projects inside the inset opening rectangle."""
    corners_local = container_opening_aperture_corners_in_container(config, margin)
    if not corners_local:
        return False
    local = _point_in_container_link(point, config)
    _normal, lateral, vertical = container_opening_axes_in_container(config)
    centroid, lat_min, lat_max, vert_min, vert_max = _aperture_bounds_from_corners(
        corners_local, lateral, vertical
    )
    lat = sum((local[i] - centroid[i]) * lateral[i] for i in range(3))
    vert = sum((local[i] - centroid[i]) * vertical[i] for i in range(3))
    return (
        lat_min - 1e-9 <= lat <= lat_max + 1e-9
        and vert_min - 1e-9 <= vert <= vert_max + 1e-9
    )


def pickup_platform_config(config):
    platform = config.get("pickup_platform", {})
    return {
        "enabled": bool(platform.get("enabled", False)),
        "size": [float(v) for v in platform.get("size", [1.0, 1.0, 0.86])],
        "translation": [float(v) for v in platform.get("translation", [0.0, -0.8, 0.0])],
        "rotation_rpy": [float(v) for v in platform.get("rotation_rpy", [0.0, 0.0, 0.0])],
        "gazebo_model": str(platform.get("gazebo_model", "pickup_platform")),
    }


def pickup_platform_enabled(config):
    return pickup_platform_config(config)["enabled"]


def pickup_platform_in_world(config):
    """Return (xyz, rpy) of pickup_platform_link in world frame."""
    plat = pickup_platform_config(config)
    return list(plat["translation"]), list(plat["rotation_rpy"])


def pickup_platform_top_in_world(config):
    """Return (xyz, rpy) of the platform top surface center in world frame."""
    plat = pickup_platform_config(config)
    _length, _width, height = plat["size"]
    plat_xyz, plat_rpy = list(plat["translation"]), list(plat["rotation_rpy"])
    top_local = [0.0, 0.0, height]
    return _compose(plat_xyz, plat_rpy, top_local, [0.0, 0.0, 0.0])


def gazebo_pickup_platform_spawn_pose(config):
    plat = pickup_platform_config(config)
    tx, ty, tz = plat["translation"]
    roll, pitch, yaw = plat["rotation_rpy"]
    return {
        "x": tx,
        "y": ty,
        "z": tz,
        "R": roll,
        "P": pitch,
        "Y": yaw,
        "model": plat["gazebo_model"],
    }


def pickup_source_in_world(config):
    """Resolve pickup source to world frame.

    If pickup_platform is enabled and pickup_source.parent references it,
    compose the platform top pose with the local pickup_source offset.
    Otherwise fall back to interpreting translation as world-frame (legacy).
    """
    source = config.get("pickup_source", {})
    local_xyz = [float(v) for v in source.get("translation", [0.0, 0.0, 0.0])]
    local_rpy = [float(v) for v in source.get("rotation_rpy", [0.0, 0.0, 0.0])]
    parent = str(source.get("parent", "")).strip()

    if parent in ("pickup_platform_top", "pickup_platform_top_frame") and pickup_platform_enabled(config):
        top_xyz, top_rpy = pickup_platform_top_in_world(config)
        return _compose(top_xyz, top_rpy, local_xyz, local_rpy)

    return local_xyz, local_rpy


def gazebo_container_model(config):
    gazebo = config.get("gazebo", {})
    return str(gazebo.get("container_model", "airport_container_real"))


def gazebo_container_spawn_pose(config):
    xyz, rpy = origin_in_world(config)
    return {
        "x": xyz[0],
        "y": xyz[1],
        "z": xyz[2],
        "R": rpy[0],
        "P": rpy[1],
        "Y": rpy[2],
        "model": gazebo_container_model(config),
    }


def gazebo_robot_spawn_z(config):
    """Extra Z offset for a floating (unwelded) Gazebo spawn.

    Welded spawn (sim_world) puts the scene pose on URDF world_base and
    creates the model at the origin, so this offset is unused there.
    """
    return 0.0


def pedestal_center_in_base_link(config):
    _length, _width, height = pedestal_dimensions(config)
    return [0.0, 0.0, -height * 0.5]


def gazebo_pedestal_spawn_pose(config):
    ped = pedestal_config(config)
    length, width, height = ped["size"]
    tx, ty, tz = ped["translation"]
    roll, pitch, yaw = ped["rotation_rpy"]
    return {
        "x": tx,
        "y": ty,
        "z": tz,
        "R": roll,
        "P": pitch,
        "Y": yaw,
        "model": ped["gazebo_model"],
        "size": (length, width, height),
    }
