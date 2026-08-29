#!/usr/bin/env python3
"""Build the MotionSegment for a smart-explore (pose_target) view."""


def build_smart_explore_segment(camera_pose, motion_segment_type,
                                phase="phase1"):
    """Return a pose_target MotionSegment for one smart explore view.

    Exploration is perception-only: the camera just has to reach a pose that
    faces the container interior. The downward path constraints
    (keep_camera_down / keep_tool_down) are intentionally OFF -- those belong
    to box placement (the suction must stay down while placing), not
    exploration. Keeping them on exploration over-constrained the path (e.g. a
    strict-down manifold violated by the observe start state) and caused
    planning failures. ``allow_ompl_fallback`` is False so a view that cannot
    be planned is rejected and the caller moves to the next candidate.

    ``camera_pose`` is the camera_depth_optical_frame pose in elfin_base_link
    (the orientation produced by the planner, facing the interior). ``phase``
    selects the segment name the motion planner routes to the optical frame.
    """
    name = "smart_phase0" if phase == "phase0" else "smart_phase1"
    return motion_segment_type(
        name=name,
        type="pose_target",
        target_pose=camera_pose,
        keep_camera_down=False,
        keep_tool_down=False,
        allow_ompl_fallback=False,
    )
