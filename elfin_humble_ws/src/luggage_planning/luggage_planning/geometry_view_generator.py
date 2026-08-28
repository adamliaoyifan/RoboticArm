#!/usr/bin/env python3
"""Generate candidate exploration viewpoints from container geometry.

Pure-Python module (no ROS dependency). Produces two families of camera
viewpoints for the smart explore pipeline:

- Phase 0 ("opening"): a small arc of viewpoints above and around the
  container opening, camera tilted at most ``max_tilt_deg`` from
  straight-down. Used to build the opening / exterior mesh before
  probing the interior.

- Phase 1 ("interior"): a grid of viewpoints just outside the opening
  plane, at multiple lateral offsets, heights, and standoffs, looking
  into the container interior center. Used for NBV-style exploration
  driven by frontier coverage.

All viewpoints are returned in ``elfin_base_link`` frame. Each view is a
dict with ``name``, ``stage``, ``camera_xyz``, ``look_at``,
``orientation_quat`` (x, y, z, w for camera_depth_optical_frame), and
``tilt_deg`` (angle between optical +Z and world -Z).

The module depends only on ``scene_tf_config_utils`` for container
geometry and ``container_aim_utils.look_at_quaternion`` for orientation
math, so it is unit-testable without a ROS master.
"""

from __future__ import division

import math

from luggage_description.scene_tf_config_utils import (
    container_in_base_link,
    container_usable_center_in_base_link,
    container_usable_dimensions,
    container_opening_dimensions,
    container_opening_aperture_lateral_offsets,
    container_opening_axes_in_base_link,
    container_opening_in_container,
    container_opening_normal_in_base_link,
    container_opening_side,
    container_opening_target_point,
    point_inside_opening_aperture,
    point_inside_container_inner_box,
)
from luggage_planning.container_aim_utils import look_at_quaternion


def _normalize(vec):
    norm = math.sqrt(sum(v * v for v in vec))
    if norm < 1e-9:
        return [0.0, 0.0, 0.0]
    return [v / norm for v in vec]


def _sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def _add(a, b):
    return [a[i] + b[i] for i in range(3)]


def _scale(a, s):
    return [a[i] * s for i in range(3)]


def validate_tilt(camera_xyz, look_at, max_tilt_deg):
    """Return (is_valid, tilt_deg, optical_z_axis).

    ``optical_z_axis`` is the unit vector from camera toward look_at (the
    optical +Z direction). ``tilt_deg`` is the angle between that axis and
    world -Z (straight down). A view is valid when ``tilt_deg <= max_tilt_deg``.
    """
    view_dir = _normalize(_sub(look_at, camera_xyz))
    down = [0.0, 0.0, -1.0]
    cos_tilt = max(-1.0, min(1.0, sum(view_dir[i] * down[i] for i in range(3))))
    tilt_deg = math.degrees(math.acos(cos_tilt))
    return tilt_deg <= max_tilt_deg, tilt_deg, view_dir


def _build_view(name, stage, camera_xyz, look_at, max_tilt_deg):
    """Build a view dict with orientation quaternion and tilt validation."""
    is_valid, tilt_deg, _ = validate_tilt(camera_xyz, look_at, max_tilt_deg)
    q = look_at_quaternion(camera_xyz, look_at)
    return {
        "name": name,
        "stage": stage,
        "camera_xyz": [float(v) for v in camera_xyz],
        "look_at": [float(v) for v in look_at],
        "orientation_quat": [float(v) for v in q],
        "tilt_deg": round(tilt_deg, 3),
        "valid_tilt": bool(is_valid),
    }


def _opening_local_offset(config):
    """Return the opening center offset in container_link as [x, y, z]."""
    open_xyz, _ = container_opening_in_container(config)
    return [float(v) for v in open_xyz]


def generate_opening_views(config, num_views=5, arc_radius=0.30,
                           height_above_opening=0.60, max_tilt_deg=45.0):
    """Phase 0: viewpoints above and around the container opening.

    The viewpoints form an arc in a plane parallel to the opening face,
    offset outward by ``arc_radius`` along the opening normal and upward
    by ``height_above_opening``. The arc spans the opening width and is
    centered on the opening center. Each view looks at the opening center.

    Returns a list of view dicts (see module docstring) in base_link frame.
    """
    opening_center = container_opening_target_point(config)
    open_w, _open_h = container_opening_dimensions(config)
    # Rotation-aware arc plane: use the container's own vertical/lateral axes
    # (from container_opening_axes_in_base_link) instead of a fixed world +Z,
    # so the arc stays in the opening plane when the container has a
    # rotation_rpy. Fallback to world +Z for a top opening where the helper's
    # vertical degenerates (normal parallel to world +Z).
    normal, lateral, up = container_opening_axes_in_base_link(config)
    _inner_l, inner_w, _inner_h = container_usable_dimensions(config)
    if all(abs(v) < 1e-6 for v in up):
        up = [0.0, 0.0, 1.0]
        lateral = _normalize(_cross(up, normal))
        if all(abs(v) < 1e-6 for v in lateral):
            lateral = _normalize(_cross([0.0, 1.0, 0.0], normal))

    # Camera arc center: opening center + normal*arc_radius + up*height_above_opening
    arc_center = _add(
        _add(opening_center, _scale(normal, arc_radius)),
        _scale(up, height_above_opening),
    )

    # Spread viewpoints laterally across the opening width, plus the center.
    # Use num_views points: if odd, include center; spread symmetrically.
    half_span = min(open_w, inner_w) * 0.4  # stay slightly inside opening edges
    if num_views <= 1:
        offsets = [0.0]
    else:
        offsets = [
            -half_span + 2.0 * half_span * i / (num_views - 1)
            for i in range(num_views)
        ]

    views = []
    for i, off in enumerate(offsets):
        camera_xyz = _add(arc_center, _scale(lateral, off))
        look_at = opening_center
        view = _build_view(
            "opening_%02d" % i, "opening", camera_xyz, look_at, max_tilt_deg
        )
        view["lateral_offset"] = round(off, 3)
        views.append(view)
    return views


def generate_interior_views(config, num_lateral=3, num_height=3,
                            standoff_values=(0.05, 0.15, 0.30),
                            max_tilt_deg=60.0, look_depth_ratio=0.3):
    """Phase 1: viewpoints just outside the opening, looking into the interior.

    Candidates are generated on a grid: ``num_lateral`` positions across
    the opening width, ``num_height`` positions from the opening bottom
    to top, and one entry per ``standoff_values`` distance along the
    opening normal. Each view looks at a point ``look_depth_ratio`` of
    the inner width into the container from the opening, at the same
    height as the camera (so the optical axis tilts down only by the
    geometry of being above the look_at point).

    Returns a list of view dicts in base_link frame.
    """
    opening_center = container_opening_target_point(config)
    open_w, open_h = container_opening_dimensions(config)
    normal = container_opening_normal_in_base_link(config)
    _inner_l, inner_w, _inner_h = container_usable_dimensions(config)
    interior_center = container_usable_center_in_base_link(config)

    up = [0.0, 0.0, 1.0]
    lateral = _normalize(_cross(up, normal))
    if all(abs(v) < 1e-6 for v in lateral):
        lateral = _normalize(_cross([0.0, 1.0, 0.0], normal))

    # Lateral offsets across opening width (centered).
    if num_lateral <= 1:
        lat_offsets = [0.0]
    else:
        lat_span = open_w * 0.4
        lat_offsets = [
            -lat_span + 2.0 * lat_span * i / (num_lateral - 1)
            for i in range(num_lateral)
        ]

    # Vertical offsets from opening bottom to top, centered on opening center.
    if num_height <= 1:
        z_offsets = [0.0]
    else:
        z_span = open_h * 0.4
        z_offsets = [
            -z_span + 2.0 * z_span * i / (num_height - 1)
            for i in range(num_height)
        ]

    # look_at depth: look_depth_ratio of inner_width INTO the container
    # from the opening, along -normal. Use interior_center as a stable
    # target but pull it back toward the opening by (1 - look_depth_ratio).
    look_at = _add(
        opening_center,
        _scale(normal, -inner_w * look_depth_ratio),
    )

    views = []
    idx = 0
    for si, standoff in enumerate(standoff_values):
        for li, lat in enumerate(lat_offsets):
            for zi, z_off in enumerate(z_offsets):
                # Camera just outside the opening, offset laterally and
                # vertically from the opening center.
                base = _add(opening_center, _scale(normal, standoff))
                camera_xyz = _add(_add(base, _scale(lateral, lat)), _scale(up, z_off))
                view = _build_view(
                    "interior_%02d" % idx, "interior",
                    camera_xyz, look_at, max_tilt_deg,
                )
                view["standoff"] = round(float(standoff), 3)
                view["lateral_offset"] = round(float(lat), 3)
                view["z_offset"] = round(float(z_off), 3)
                views.append(view)
                idx += 1
    return views


def _linspace_centered(span, count):
    if count <= 1:
        return [0.0]
    return [
        -span * 0.5 + span * float(i) / float(count - 1)
        for i in range(count)
    ]


def generate_interior_downward_views(
        config, num_lateral=3, num_depth=3, camera_z=None,
        depth_min_from_opening=0.20, depth_max_ratio=0.75,
        wall_clearance=0.15, aperture_margin=0.12, look_down=0.80):
    """Generate camera-down probe views whose optical origin is inside the box.

    Candidates are expressed in ``elfin_base_link``.  The camera enters along
    the inward opening normal, while its optical +Z axis remains world -Z.
    Geometry-invalid candidates are retained with ``valid_geometry=False`` and
    a stable ``reject_reason`` so RViz can explain why they were rejected.
    """
    opening = container_opening_target_point(config)
    normal, lateral, _vertical = container_opening_axes_in_base_link(config)
    inner_l, inner_w, _inner_h = container_usable_dimensions(config)

    lateral_offsets = container_opening_aperture_lateral_offsets(
        config, int(num_lateral), margin=float(aperture_margin)
    )
    opening_axis, _opening_sign = container_opening_side(config)
    inward_extent = (
        inner_l if opening_axis == 0
        else inner_w if opening_axis == 1
        else _inner_h
    )
    min_depth = max(0.01, float(depth_min_from_opening))
    max_depth = max(min_depth, inward_extent * float(depth_max_ratio))
    if num_depth <= 1:
        depths = [min_depth]
    else:
        depths = [
            min_depth + (max_depth - min_depth) * float(i) / float(num_depth - 1)
            for i in range(num_depth)
        ]

    z = float(camera_z) if camera_z is not None else float(opening[2])
    down_q = [1.0, 0.0, 0.0, 0.0]
    views = []
    idx = 0
    for depth in depths:
        for lane_index, lateral_offset in enumerate(lateral_offsets):
            camera = [
                opening[i] - normal[i] * depth + lateral[i] * lateral_offset
                for i in range(3)
            ]
            camera[2] = z
            look_at = [camera[0], camera[1], camera[2] - float(look_down)]
            # Intersect the straight insertion line (parallel to the opening
            # normal) with the opening plane. This remains correct when the
            # container is rotated, unlike copying world X/Y/Z coordinates.
            camera_from_opening = [camera[i] - opening[i] for i in range(3)]
            distance_to_plane = sum(
                camera_from_opening[i] * normal[i] for i in range(3)
            )
            aperture_point = [
                camera[i] - normal[i] * distance_to_plane for i in range(3)
            ]

            reason = ""
            if not point_inside_container_inner_box(camera, config):
                reason = "outside_inner_box"
            elif not point_inside_container_inner_box(
                    camera, config, margin=wall_clearance):
                reason = "wall_clearance"
            if not reason and not point_inside_opening_aperture(
                    aperture_point, config, margin=aperture_margin):
                reason = "aperture_blocked"

            views.append({
                "name": "interior_probe_%02d" % idx,
                "stage": "interior_probe",
                "camera_xyz": [float(v) for v in camera],
                "look_at": [float(v) for v in look_at],
                "orientation_quat": list(down_q),
                "tilt_deg": 0.0,
                "valid_tilt": True,
                "valid_geometry": not bool(reason),
                "reject_reason": reason,
                "depth": round(float(depth), 3),
                "lateral_offset": round(float(lateral_offset), 3),
                "lane_id": "lane_%02d" % lane_index,
                "aperture_xyz": [float(v) for v in aperture_point],
            })
            idx += 1
    return views


def filter_geometry_valid_views(views):
    """Return candidates that satisfy inner-wall and opening constraints."""
    return [view for view in views if view.get("valid_geometry", True)]


def _cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def views_to_candidates(views):
    """Format views for IK solving / frontier scoring.

    Returns a list of dicts with ``name``, ``camera_xyz``, ``look_at``,
    and ``orientation_quat`` keys (the subset consumed by
    ``cargo_exploration_planner_node._solve_view_ik`` and
    ``constrained_view_planner.coverage_score``).
    """
    return [
        {
            "name": v["name"],
            "stage": v["stage"],
            "camera_xyz": v["camera_xyz"],
            "look_at": v["look_at"],
            "orientation_quat": v["orientation_quat"],
            "tilt_deg": v["tilt_deg"],
            "valid_tilt": v["valid_tilt"],
        }
        for v in views
    ]


def filter_valid_views(views):
    """Return only views whose tilt is within the constraint."""
    return [v for v in views if v.get("valid_tilt", False)]


def distance_from_base(camera_xyz, base_origin=(0.0, 0.0, 0.0)):
    """Euclidean distance from the arm base origin to a camera viewpoint."""
    return math.sqrt(
        sum((camera_xyz[i] - base_origin[i]) ** 2 for i in range(3))
    )


def filter_reachable_views(views, max_reach=1.6, base_origin=(0.0, 0.0, 0.0)):
    """Return only views within the arm reach radius from base_origin."""
    return [
        v for v in views
        if distance_from_base(v["camera_xyz"], base_origin) <= max_reach
    ]


def generate_uncertainty_aware_corridor_views(
        opening_geometry, orientation_quat, observed_free_depth,
        num_lateral=3, min_depth=0.10, depth_step=0.15, max_depth=None,
        camera_half_width=0.08, camera_half_height=0.05,
        physical_clearance=0.05, uncertainty_margin=0.0):
    """Generate sensor-contract candidates inside an observed-free corridor."""
    opening = [float(v) for v in opening_geometry["opening_xyz"]]
    normal = _normalize([float(v) for v in opening_geometry["normal"]])
    lateral = _normalize([float(v) for v in opening_geometry["lateral"]])
    up = _normalize([float(v) for v in opening_geometry["up"]])
    if any(not any(abs(v) > 1e-9 for v in axis)
           for axis in (normal, lateral, up)):
        return []

    uncertainty = max(
        float(uncertainty_margin),
        float(opening_geometry.get("uncertainty_margin", 0.0)),
    )
    clearance = max(0.0, float(physical_clearance)) + uncertainty
    safe_half_width = (
        0.5 * float(opening_geometry["aperture_width"])
        - float(camera_half_width) - clearance
    )
    safe_half_height = (
        0.5 * float(opening_geometry["aperture_height"])
        - float(camera_half_height) - clearance
    )
    if safe_half_width <= 0.0 or safe_half_height <= 0.0:
        return []

    observed_depth = max(0.0, float(observed_free_depth))
    geometry_depth = max(
        0.0, float(opening_geometry.get("inner_depth", observed_depth)))
    depth_limit = min(observed_depth, geometry_depth)
    if max_depth is not None:
        depth_limit = min(depth_limit, max(0.0, float(max_depth)))
    min_depth = max(0.01, float(min_depth))
    depth_step = max(0.01, float(depth_step))
    if depth_limit + 1e-9 < min_depth:
        return []

    if int(num_lateral) <= 1:
        lateral_offsets = [0.0]
    else:
        lateral_offsets = [
            -safe_half_width
            + 2.0 * safe_half_width * i / float(int(num_lateral) - 1)
            for i in range(int(num_lateral))
        ]
    depths = []
    depth = min_depth
    while depth <= depth_limit + 1e-9:
        depths.append(min(depth, depth_limit))
        depth += depth_step
    if not depths or depth_limit - depths[-1] > 1e-6:
        depths.append(depth_limit)

    version = int(opening_geometry.get("geometry_version", 0))
    source = str(opening_geometry.get("source", "unknown"))
    age = float(opening_geometry.get("age", 0.0))
    views = []
    for lane_index, lateral_offset in enumerate(lateral_offsets):
        lane_id = "lane_%02d" % lane_index
        aperture = _add(opening, _scale(lateral, lateral_offset))
        for depth_index, insertion_depth in enumerate(depths):
            camera = _add(
                aperture, _scale(normal, -float(insertion_depth)))
            look_at = _add(camera, _scale(up, -0.80))
            views.append({
                "name": "corridor_%02d_%02d" % (
                    lane_index, depth_index),
                "candidate_id": "g%d:%s:d%02d" % (
                    version, lane_id, depth_index),
                "stage": "interior_probe",
                "camera_xyz": camera,
                "aperture_xyz": aperture,
                "opening_normal": list(normal),
                "look_at": look_at,
                "orientation_quat": [float(v) for v in orientation_quat],
                "tilt_deg": 0.0,
                "valid_tilt": True,
                "valid_geometry": True,
                "reject_reason": "",
                "lane_id": lane_id,
                "lateral_offset": float(lateral_offset),
                "depth": float(insertion_depth),
                "aperture_clearance": min(
                    safe_half_width - abs(lateral_offset),
                    safe_half_height,
                ),
                "wall_clearance": clearance,
                "uncertainty_margin": uncertainty,
                "geometry_age": age,
                "geometry_source": source,
                "geometry_version": version,
                "corridor_free_confidence": 1.0,
            })
    return views
