#!/usr/bin/env python3
"""Build conservative pick/place motion segments."""

import math

from luggage_planning.pose import MotionSegment, Point, Pose, Quaternion

# Single source of truth for the motion clearances. These used to be declared
# three times each (launch arg, node param default, module constant) with
# different values, so the effective clearance depended on how the node was
# started. waypoint_generator_node reads its defaults from here.
DEFAULT_PLACE_CLEARANCE_Z = 0.15

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


def _pose_at(pose, z, yaw):
    return Pose(
        position=Point(x=pose.position.x, y=pose.position.y, z=z),
        orientation=_tool_down_quaternion(yaw),
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
                   corridor_margin=DEFAULT_CORRIDOR_MARGIN):
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

    Returns:
        list[MotionSegment]
    """
    if phase == "pick":
        yaw = pick_tool_yaw(
            _yaw_from_quaternion(pick.pose.orientation),
            _luggage_yaw_valid(pick),
            fallback_yaw,
        )
        top_z = pick.pose.position.z + max(0.0, pick.height) * 0.5
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
        slot_pose = place_slot.place_pose
        # SlotSpec.place_pose is the suitcase CENTER. Motion planning targets
        # suction_contact_frame, which must finish on the suitcase top.
        slot_yaw = _yaw_from_quaternion(slot_pose.orientation)
        box_height = max(
            0.0,
            float(getattr(place_slot, "height", 0.0)
                  or getattr(pick, "height", 0.0)),
        )
        contact_z = slot_pose.position.z + box_height * 0.5
        if place_clearance_z is None:
            place_clearance_z = DEFAULT_PLACE_CLEARANCE_Z
        # G1: raise the carry height when neighbors along the corridor are
        # taller than this slot's own top. The payload hangs a full box
        # height below the suction frame, so the required height keeps the
        # payload bottom above corridor_surface_max + margin.
        clearance = corridor_clearance(
            corridor_surface_max, box_height, contact_z, place_clearance_z,
            margin=corridor_margin)
        above_pose = _pose_at(slot_pose, contact_z + clearance, slot_yaw)
        transit_pose = above_pose
        stage_pose = None
        stage_mid_pose = None
        stage_late_pose = None
        if opening_info:
            opening = opening_info["point"]
            normal = opening_info["normal"]
            outward = float(opening_info.get("outward_clearance", 0.15))
            current = [
                above_pose.position.x,
                above_pose.position.y,
                above_pose.position.z,
            ]
            signed = sum(
                (current[i] - opening[i]) * normal[i]
                for i in range(3))
            portal = [
                current[i] + normal[i] * (outward - signed)
                for i in range(3)
            ]
            # Entry at the portal must already satisfy the corridor height:
            # a straight portal->above traverse crosses the opening plane at
            # (or near) portal height, so a tall box just inside the door
            # would clip an ascending entry that starts too low.
            portal[2] = max(
                portal[2],
                float(opening[2])
                + float(opening_info.get("min_height_above_opening", 0.35)),
            )
            if corridor_surface_max is not None:
                portal[2] = max(
                    portal[2],
                    float(corridor_surface_max) + box_height
                    + float(corridor_margin))
            transit_pose = Pose(
                position=Point(
                    x=portal[0], y=portal[1], z=portal[2]),
                orientation=_tool_down_quaternion(slot_yaw),
            )
            pick_yaw = pick_tool_yaw(
                _yaw_from_quaternion(pick.pose.orientation),
                _luggage_yaw_valid(pick),
                fallback_yaw,
            )
            stage_distance = float(
                opening_info.get("stage_outward_clearance", 0.65))
            offset = staging_offset(normal, stage_distance)
            stage_pose = Pose(
                position=Point(
                    x=portal[0] + offset[0],
                    y=portal[1] + offset[1],
                    z=max(
                        portal[2] + offset[2],
                        float(opening[2]) + float(
                            opening_info.get(
                                "stage_height_above_opening", 0.65))),
                ),
                orientation=_tool_down_quaternion(pick_yaw),
            )
            start = opening_info.get("start_point")
            if start is not None:
                stage_mid_pose = Pose(
                    position=Point(
                        x=(float(start[0]) + stage_pose.position.x) * 0.5,
                        y=(float(start[1]) + stage_pose.position.y) * 0.5,
                        z=max(
                            float(start[2]),
                            (float(start[2]) + stage_pose.position.z) * 0.5),
                    ),
                    orientation=_tool_down_quaternion(pick_yaw),
                )
                stage_late_pose = Pose(
                    position=Point(
                        x=(stage_mid_pose.position.x
                           + stage_pose.position.x) * 0.5,
                        y=(stage_mid_pose.position.y
                           + stage_pose.position.y) * 0.5,
                        z=(stage_mid_pose.position.z
                           + stage_pose.position.z) * 0.5,
                    ),
                    orientation=_tool_down_quaternion(pick_yaw),
                )
        insert_pose = _pose_at(
            slot_pose,
            contact_z + insertion_clearance(box_height, clearance),
            slot_yaw)
        contact_pose = _pose_at(slot_pose, contact_z, slot_yaw)
        retreat_pose = _pose_at(slot_pose, contact_z + clearance, slot_yaw)
        # Transit keeps the suction normal tool-down during payload carry.
        # Do not also lock J4/J5/J6: that redundant joint-space constraint can
        # exclude every IK branch for an otherwise valid tool-down goal
        # (E12 one-box smoke). insert/descend/retreat follow the same safe
        # orientation through Cartesian motion.
        segments = []
        if stage_mid_pose is not None:
            segments.append(MotionSegment(
                name="stage_mid", type="cartesian",
                target_pose=stage_mid_pose,
                keep_tool_down=True, keep_camera_down=False,
                allow_ompl_fallback=True,
                lock_wrist=False,
            ))
        if stage_late_pose is not None:
            segments.append(MotionSegment(
                name="stage_late", type="cartesian",
                target_pose=stage_late_pose,
                keep_tool_down=True, keep_camera_down=False,
                allow_ompl_fallback=True,
                lock_wrist=False,
            ))
        if stage_pose is not None and stage_late_pose is None:
            segments.append(MotionSegment(
                name="stage", type="cartesian", target_pose=stage_pose,
                keep_tool_down=True, keep_camera_down=False,
                allow_ompl_fallback=True,
                lock_wrist=False,
            ))
        segments.extend([
            MotionSegment(
                name="transit", type="pose_target", target_pose=transit_pose,
                keep_tool_down=True, keep_camera_down=False, lock_wrist=False,
            ),
            MotionSegment(
                name="traverse", type="cartesian", target_pose=above_pose,
                keep_tool_down=True, keep_camera_down=False,
                allow_ompl_fallback=True,
            ),
            MotionSegment(
                name="insert", type="cartesian", target_pose=insert_pose,
                keep_tool_down=True, keep_camera_down=True,
                allow_ompl_fallback=False,
            ),
            MotionSegment(
                name="descend", type="cartesian", target_pose=contact_pose,
                keep_tool_down=True, keep_camera_down=True,
                allow_ompl_fallback=False,
            ),
            MotionSegment(
                name="retreat", type="cartesian", target_pose=retreat_pose,
                keep_tool_down=True, keep_camera_down=True,
                allow_ompl_fallback=False,
            ),
        ])
        return segments
    return []


def segment_names_for_phase(phase):
    """Ordered segment names. Pick lift-off is pick_retreat; place lift-off is retreat."""
    if phase == "pick":
        return ["pre_grasp", "approach", "attach", "pick_retreat"]
    if phase == "place":
        return ["transit", "traverse", "insert", "descend", "retreat"]
    return []
