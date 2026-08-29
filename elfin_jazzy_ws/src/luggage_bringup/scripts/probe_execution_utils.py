#!/usr/bin/env python3
"""Build the execution chain for an interior camera probe."""


def build_interior_probe_segments(waypoints, motion_segment_type,
                                  require_tool_down=False,
                                  align_before_probe=True):
    """Return pre-opening, entry, and probe MotionSegments.

    Exploration is perception-only, so these segments carry NO downward path
    constraints (keep_camera_down / keep_tool_down are False) -- those belong
    to box placement, not exploration. ``require_tool_down`` now only affects
    segment structure: smart strict-down probes (require_tool_down=True) let
    align_down replace pre_opening, while plain interior_probe prepends
    align_down before pre_opening.
    """
    poses = list(waypoints or [])
    if len(poses) < 3:
        raise ValueError("probe plan requires pre-opening, aperture, and internal waypoints")
    segments = []

    def _align_down():
        # Reach the pre-opening goal without applying a path constraint to an
        # arbitrary observe/phase0 start state. Orchestrator executes this via
        # go_to_joint_values(plan.joint_values) so OMPL does not pick a
        # wrist-spinning pose-equivalent branch. The following constrained
        # segments start on the camera-down manifold.
        return motion_segment_type(
            name="align_down",
            type="pose_target",
            target_pose=poses[0],
            keep_camera_down=False,
            keep_tool_down=False,
            allow_ompl_fallback=False,
        )

    if require_tool_down and align_before_probe:
        # Smart strict-down: align_down reaches the suction-down goal; it
        # replaces pre_opening because the goal is already on the strict-down
        # manifold and a constrained hop from observe is unnecessary.
        segments.append(_align_down())
    else:
        if align_before_probe:
            # Plain interior_probe: prepend an unconstrained align_down so the
            # constrained pre_opening starts on the camera-down branch instead
            # of an arbitrary observe state. Without this, applying the
            # camera_down path constraint directly from a misaligned observe
            # (e.g. S30 observe camera yaw ~86deg off) forces OMPL into a
            # wrist-spinning branch that fails the wrist-quality gate.
            segments.append(_align_down())
        segments.append(motion_segment_type(
            name="pre_opening",
            type="pose_target",
            target_pose=poses[0],
            keep_camera_down=False,
            keep_tool_down=False,
            allow_ompl_fallback=False,
        ))
    segments.extend([
        motion_segment_type(
            name="enter_opening",
            type="cartesian",
            target_pose=poses[1],
            keep_camera_down=False,
            keep_tool_down=False,
            allow_ompl_fallback=False,
        ),
        motion_segment_type(
            name="probe_inside",
            type="cartesian",
            target_pose=poses[2],
            keep_camera_down=False,
            keep_tool_down=False,
            allow_ompl_fallback=False,
        ),
        motion_segment_type(
            name="retreat_opening",
            type="cartesian",
            target_pose=poses[0],
            keep_camera_down=False,
            keep_tool_down=False,
            allow_ompl_fallback=False,
        ),
    ])
    return segments
