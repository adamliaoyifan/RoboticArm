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
    # TODO: compute pre_grasp, approach, insert waypoints from YAML configs
    _ = (pick, place_slot, phase)
    return []


def segment_names_for_phase(phase):
    """Ordered segment names for stub orchestrator logging."""
    if phase == "pick":
        return ["pre_grasp", "approach", "retreat"]
    if phase == "place":
        return ["transit", "insert", "descend", "retreat"]
    return []
