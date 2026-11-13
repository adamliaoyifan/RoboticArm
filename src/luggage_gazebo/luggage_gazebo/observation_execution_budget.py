"""Response wait for an already-certified trajectory, not a planning limit."""
from luggage_planning.motion.timed_execution_budget import timed_execution_wait


def _segment_points(segment):
    """Points of a certified segment, message- or dict-shaped.

    Production passes MotionSegment messages (attribute paths); offline
    stubs pass dicts ({"points": [...]} or {"trajectory": {"points":
    [...]}}). Both carry the same certified timing contract.
    """
    node = getattr(segment, "joint_trajectory", None)
    if node is None and isinstance(segment, dict):
        node = segment.get("joint_trajectory")
    if node is None:
        node = segment
    traj = getattr(node, "trajectory", None)
    if traj is None and isinstance(node, dict):
        traj = node.get("trajectory")
    if traj is not None:
        node = traj
    points = getattr(node, "points", None)
    if points is None and isinstance(node, dict):
        points = node.get("points")
    return points


def certified_execution_wait(segment, overhead_s):
    points = _segment_points(segment)
    if points is None:
        raise ValueError("segment carries no certified points")
    return timed_execution_wait(points, overhead_s)


observation_execution_wait = certified_execution_wait
