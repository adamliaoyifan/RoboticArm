#!/usr/bin/env python3
"""Pick waypoints and frozen clearance helpers.

[ST09-I] the fixed-clearance place legs were deleted with the geometric
execution bypass (routes own place motion); pick waypoints remain
candidate generators refined into certified edges, and the clearance
helpers feed route graph anchors and coverage checks.
"""

import math

from luggage_description.frame_transforms import transform_pose
from luggage_planning.geometry.pose import MotionSegment, Point, Pose, Quaternion

# Single source of truth for the motion clearances. These used to be declared
# three times each (launch arg, node param default, module constant) with
# different values, so the effective clearance depended on how the node was
# started. waypoint_generator_node reads its defaults from here.
DEFAULT_PLACE_CLEARANCE_Z = 0.15

# Extra post-release exit proposal height above the carrying approach.
# This seeds planning; the native checks decide whether the motion is legal.
EXIT_EXTRA_LIFT_M = 0.10

# Descend TCP sits this far along container +Z above the true rest top
# (slot center + half height). SlotSpec.place_pose stays the rest pose.
# 50 mm keeps the physical mesh bottom above the cargo slab at detach
# (hang + mesh-vs-measured); the box then drops onto OCC.
PLACE_RELEASE_GAP_M = 0.05

DEFAULT_PICK_CLEARANCES = {
    "pre_grasp": 0.30,
    "approach": 0.25,
    # attach = 0 so suction_contact_frame lands on the box top (contact). The
    # suction_panel<->current_pickup_box ACM (scene_manager) permits the cup
    # touching the box so this goal is not flagged GOAL_IN_COLLISION.
    "attach": 0.0,
    "pick_retreat": 0.35,
    "pre_grasp_min": 0.20,
    "approach_min": 0.08,
}

# Insertion descends to this fraction of the box height above the contact
# point, bounded by the transit clearance. A fixed 0.10 m was tuned for the
# three catalog heights; with continuously sized boxes it is 40% of a short box
# and 31% of a tall one, so the insertion depth silently changed with the box.
INSERT_HEIGHT_FRACTION = 0.35
INSERT_CLEARANCE_MIN = 0.06

# Corridor margin (G1, docs/plans/corridor_constraints.md): when the caller
# supplies the highest occupied surface along the opening corridor, the
# traverse/extract heights are raised so the payload bottom clears that
# surface by this margin. The payload hangs a full box height below the
# suction frame (box top = suction frame during tool-down carry), so the
# required suction height is surface_max + box_height + margin, not half.
DEFAULT_CORRIDOR_MARGIN = 0.05

# Pick waypoint generation needs a measured pickup contact Z (E0/E4,
# docs/plans/platform_free_height_eng_todo.md): either the measured
# top_surface_pose or a measured height to derive it from. An unmeasured
# height (including a catalog prior) must never drive the arm down.
FULL_GEOMETRY_REQUIRED = "DETECT_FULL_GEOMETRY_REQUIRED"


def pick_contact_top_z(pick):
    """Measured box-top Z from the platform-free detection contract.

    ``top_surface_pose.position.z`` is the sole Z input when valid; the
    legacy center+height/2 derivation is accepted only when the height is
    a measurement (``height_valid``). Raises ValueError otherwise.
    """
    if getattr(pick, "top_surface_valid", False):
        return float(pick.top_surface_pose.position.z)
    if getattr(pick, "height_valid", False):
        return float(pick.pose.position.z) + max(0.0, float(pick.height)) * 0.5
    raise ValueError(
        "%s: pick Z needs top_surface_pose or measured height"
        % FULL_GEOMETRY_REQUIRED)


def corridor_clearance(corridor_surface_max, box_height, contact_z,
                       place_clearance_z, margin=DEFAULT_CORRIDOR_MARGIN):
    """Effective place clearance honoring the corridor surface height.

    Returns ``place_clearance_z`` when ``corridor_surface_max`` is None
    (empty corridor / single-box slice-A behavior, backward compatible).
    Otherwise the clearance above the contact pose is raised so that
    ``contact_z + clearance >= corridor_surface_max + box_height + margin``
    i.e. the payload bottom clears the tallest neighbor by ``margin``.
    """
    if corridor_surface_max is None:
        return float(place_clearance_z)
    required_suction_z = (
        float(corridor_surface_max) + max(0.0, float(box_height))
        + float(margin))
    return max(float(place_clearance_z), required_suction_z - float(contact_z))


def insertion_clearance(box_height, place_clearance_z):
    """Height above the contact pose where the insertion segment begins."""
    scaled = INSERT_HEIGHT_FRACTION * max(0.0, float(box_height))
    return max(INSERT_CLEARANCE_MIN, min(scaled, float(place_clearance_z)))


def staging_offset(normal, stage_distance):
    """World-frame offset of the stage waypoint along the opening normal.

    The previous implementation only added ``normal[1] * distance`` (Y), which
    is a no-op when the opening faces -X in world.
    """
    distance = float(stage_distance)
    return [float(normal[i]) * distance for i in range(3)]


def _clearance(clearances, name):
    if clearances is None:
        clearances = DEFAULT_PICK_CLEARANCES
    return float(clearances.get(name, DEFAULT_PICK_CLEARANCES[name]))


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def nearest_box_yaw(reference, target):
    """Choose *target* or *target ± π* closest to *reference*.

    A rectangular payload is unchanged under 180° yaw. Independently folding
    both angles into (-π/2, π/2] would rewrite staging to the slot yaw and
    flip the wrist on the cartesian stage hop. Keep the carry heading and
    pick the equivalent slot yaw that does not demand that hop.

    Do not wrap ±π onto a single representative: ``_tool_down_quaternion(π)``
    is ``[0,1,0,0]`` and ``_tool_down_quaternion(-π)`` is ``[0,-1,0,0]``.
    IK treats those as a 180° wrist change even though they are the same
    heading. Stay on the reference sign of π.
    """
    ref = float(reference)
    candidates = [float(target) + k * math.pi for k in (-2, -1, 0, 1, 2)]
    best = min(candidates, key=lambda yaw: abs(yaw - ref))
    wrapped = abs(math.atan2(math.sin(best - ref), math.cos(best - ref)))
    if wrapped < 1e-3:
        return ref
    return best


def pick_tool_yaw(detected_yaw, yaw_valid, fallback_yaw=0.0):
    """Azimuth for tool-down pick poses.

    Near-square PCA heading is noise: when *yaw_valid* is false, keep
    *fallback_yaw* (current wrist or container heading from the node TF)
    instead of rotating to the detection yaw.
    """
    if yaw_valid:
        return float(detected_yaw)
    return float(fallback_yaw)


def _luggage_yaw_valid(pick):
    if not hasattr(pick, "yaw_valid"):
        return True
    return bool(pick.yaw_valid)


def _tool_down_quaternion(yaw):
    """Quaternion with tool z-axis pointing down (-Z) at the requested yaw."""
    c = math.cos(yaw * 0.5)
    s = math.sin(yaw * 0.5)
    return Quaternion(x=c, y=s, z=0.0, w=0.0)


def tool_down_yaw(x, y):
    """Yaw carried by a tool-down quaternion's ``x``/``y`` components.

    Inverse of ``_tool_down_quaternion``. Keeps the sign of π so a caller can
    feed the result to ``nearest_box_yaw`` as the reference wrist: ``[0,1,0,0]``
    reads +π and ``[0,-1,0,0]`` reads -π, which IK treats as a 180° apart.
    """
    return 2.0 * math.atan2(float(y), float(x))


def _pose_at(pose, z, yaw):
    return Pose(
        position=Point(x=pose.position.x, y=pose.position.y, z=z),
        orientation=_tool_down_quaternion(yaw),
    )


def slot_from_contract(record, container_in_world):
    """World slot pose from a v2 slot record (ST05 F08 seam).

    ``record`` is the slot-profile pose contract (container box center,
    box-axis yaw); ``container_in_world`` is the declared container
    transform as ``(translation_xyz, quaternion_xyzw)``. Returns the box
    center in world with the COMPLETE orientation (container rotation
    composed with the box-axis yaw) — never an xyz-only conversion. The
    box top is center + h/2 by contract. ST07 wires this into the
    BuildMotionSequenceV2/SelectPlacePathV2 endpoints; it is the declared
    meaning those services will carry.
    """
    translation, rotation = container_in_world
    world = transform_pose(record, {
        "source_frame": record.get("frame_id"),
        "target_frame": "world",
        "translation": [float(v) for v in translation],
        "quaternion_xyzw": [float(v) for v in rotation],
        "stamp_ns": record.get("stamp_ns"),
        "scene_revision": record.get("scene_revision"),
    })
    pos = world["position"]
    quat = world["quaternion_xyzw"]
    return Pose(
        position=Point(x=pos[0], y=pos[1], z=pos[2]),
        orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
    )


def _perception_clearances(perception_info, pick_clearances):
    """Compute adaptive clearances from perception data.

    When ``perception_info`` carries ``suction_z`` (current suction-contact
    frame Z in world) and ``box_top_z`` (perceived box-top surface Z), the
    pre-grasp and approach heights are scaled as fractions of the
    suction-to-box-top gap, with hard safety floors.
    """
    box_top_z = perception_info.get("box_top_z")
    suction_z = perception_info.get("suction_z")
    if box_top_z is None or suction_z is None:
        return pick_clearances or DEFAULT_PICK_CLEARANCES

    gap = max(0.05, suction_z - box_top_z)
    pre_grasp_min = float((pick_clearances or {}).get("pre_grasp_min", 0.20))
    approach_min = float((pick_clearances or {}).get("approach_min", 0.08))

    return {
        "pre_grasp": max(pre_grasp_min, 0.6 * gap),
        "approach": max(approach_min, 0.3 * gap),
        "attach": 0.0,
        "pick_retreat": _clearance(pick_clearances, "pick_retreat"),
    }


def build_sequence(pick, place_slot, phase, pick_clearances=None,
                   place_clearance_z=None, perception_info=None,
                   opening_info=None, fallback_yaw=0.0,
                   corridor_surface_max=None,
                   corridor_margin=DEFAULT_CORRIDOR_MARGIN,
                   retreat_safe_tool_z=None):
    """Build MotionSegment list for pick or place phase.

    Args:
        pick: DetectedLuggage
        place_slot: SlotSpec
        phase: 'pick' or 'place'
        pick_clearances: optional dict of per-pick-segment Z clearances above
            the box top for the pose target link/contact frame.
        place_clearance_z: optional Z clearance above the slot for transit.
        perception_info: optional dict with ``box_top_z`` and ``suction_z``
            for adaptive approach.  When *None* the fixed-clearance path is
            used (backward compatible).
        opening_info: optional dict describing the container opening.
        fallback_yaw: tool-down azimuth used when ``pick.yaw_valid`` is false.
            The node should pass current wrist heading from TF; this module
            does not look up TF.
        corridor_surface_max: optional float, highest occupied surface Z
            (world) along the opening corridor between the portal and this
            slot's near face. When given, traverse/retreat(extract) heights
            are raised so the payload bottom clears it by
            ``corridor_margin`` (G1 in docs/plans/corridor_constraints.md).
            None keeps the single-slot fixed-clearance behavior.
        corridor_margin: clearance kept between the payload bottom and the
            corridor surface when raising heights.
        retreat_safe_tool_z: optional tool-z cap in the same frame as
            ``place_slot.place_pose``. When set, the place retreat stops at
            this height instead of lifting into the lid. Traverse is not
            capped. None keeps the uncapped retreat (tests and callers that
            have not converted container_link's safe tool z).

    Returns:
        list[MotionSegment]
    """
    if phase == "pick":
        yaw = pick_tool_yaw(
            _yaw_from_quaternion(pick.pose.orientation),
            _luggage_yaw_valid(pick),
            fallback_yaw,
        )
        top_z = pick_contact_top_z(pick)
        if perception_info is not None and "box_top_z" in perception_info:
            top_z = perception_info["box_top_z"]

        effective = pick_clearances
        if perception_info is not None:
            effective = _perception_clearances(perception_info, pick_clearances)

        pre_grasp = _pose_at(pick.pose, top_z + _clearance(effective, "pre_grasp"), yaw)
        approach = _pose_at(pick.pose, top_z + _clearance(effective, "approach"), yaw)
        attach = _pose_at(pick.pose, top_z + _clearance(effective, "attach"), yaw)
        pick_retreat = _pose_at(
            pick.pose, top_z + _clearance(effective, "pick_retreat"), yaw)
        # Transitional pick segments (pre_grasp, approach, attach, pick_retreat)
        # must NOT carry tool_down/camera_down/wrist_lock path constraints.
        # The arm needs to rotate the wrist freely from the observe pose to
        # the tool-down grasp pose; locking j4/j5/j6 to the observe values
        # while demanding a tool-down orientation makes the start state
        # violate the path constraints (START_STATE_VIOLATES_PATH_CONSTRAINTS)
        # and no amount of constraint relaxation can recover, because the
        # start state itself is infeasible under the combined constraints.
        # The tool-down orientation is baked into each target_pose instead,
        # so the planner reaches it at the goal without forcing it mid-path.
        # pre_grasp is a large observe->above-box reposition, so it stays a
        # free-space (OMPL) pose_target. approach/attach/pick_retreat are
        # vertical descents/lifts directly over the box: a cartesian
        # straight-line path keeps the panel footprint fixed over the box
        # and the wrist tool-down, so the forearm cannot swing through the
        # box volume between waypoints (Bug #1 root cause). The motion
        # planner falls back to OMPL pose_target if the cartesian path
        # cannot reach the target fraction.
        return [
            MotionSegment(name="pre_grasp", type="pose_target", target_pose=pre_grasp),
            MotionSegment(
                name="approach", type="cartesian", target_pose=approach,
                allow_ompl_fallback=True,
            ),
            MotionSegment(
                name="attach", type="cartesian", target_pose=attach,
                allow_ompl_fallback=True,
            ),
            MotionSegment(
                name="pick_retreat", type="cartesian", target_pose=pick_retreat,
                keep_tool_down=True, keep_camera_down=False,
                allow_ompl_fallback=True,
            ),
        ]
    if phase == "place":
        # [ST09-I] the fixed-clearance place legs were the geometric
        # execution bypass: certified route legs come from
        # route_producer -> SelectPlacePathV2. The frozen clearance
        # helpers stay as graph-anchor generators.
        raise ValueError(
            "PLACE_LEGS_ROUTE_OWNED: place motion comes from the "
            "certified route (SelectPlacePathV2), not build_sequence")
    return []


def segment_names_for_phase(phase):
    """Ordered segment names. Pick lift-off is pick_retreat; place lift-off is retreat."""
    if phase == "pick":
        return ["pre_grasp", "approach", "attach", "pick_retreat"]
    if phase == "place":
        return ["transit", "traverse", "insert", "descend", "retreat"]
    return []
