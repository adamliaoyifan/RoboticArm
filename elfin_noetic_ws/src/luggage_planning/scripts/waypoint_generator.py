#!/usr/bin/env python3
"""Pure geometry helpers — Phase 0 returns empty segment lists."""

from luggage_msgs.msg import MotionSegment


def build_sequence(pick, place_slot, phase):
    """Build MotionSegment list for pick or place phase.

    Args:
        pick: DetectedLuggage
        place_slot: SlotSpec
        phase: 'pick' or 'place'

    Returns:
        list[MotionSegment]
    """
    if phase == "pick":
        target = pick.pose
        return [
            MotionSegment(name="pre_grasp", type="pose_target", target_pose=target),
            MotionSegment(name="approach", type="pose_target", target_pose=target),
            MotionSegment(name="retreat", type="pose_target", target_pose=target),
        ]
    if phase == "place":
        target = place_slot.place_pose
        return [
            MotionSegment(name="transit", type="pose_target", target_pose=target),
            MotionSegment(name="insert", type="pose_target", target_pose=target),
            MotionSegment(name="descend", type="pose_target", target_pose=target),
            MotionSegment(name="retreat", type="pose_target", target_pose=target),
        ]
    return []


def segment_names_for_phase(phase):
    """Ordered segment names for stub orchestrator logging."""
    if phase == "pick":
        return ["pre_grasp", "approach", "retreat"]
    if phase == "place":
        return ["transit", "insert", "descend", "retreat"]
    return []
