"""Bounded independent acquisitions for post-release occupancy confirmation."""

import time
from luggage_gazebo.cargo_view_sequence import next_expected_revision, view_stop

MAX_FRAMES = 4
WALL_BUDGET_S = 15.0


def observe_fresh_frames(acquire, integrate, revision, *, after_ns=-1,
                         clock=time.monotonic):
    """Ports: acquire(after_ns, deadline)->(stamp_ns, frame) or None;
    integrate(frame, ordinal, revision)->trace. Never replay an acquisition.
    Existing confirmation criteria and owner revision admission stay intact.
    The wall budget gates starting acquisitions/integrations; an in-flight
    integration finishes under its caller's existing service watchdog.
    """
    deadline = clock() + WALL_BUDGET_S
    traces = []
    reason = "frame_limit"
    for ordinal in range(MAX_FRAMES):
        if clock() >= deadline:
            reason = "deadline"
            break
        item = acquire(after_ns, deadline)
        if item is None:
            reason = "fresh_frame_unavailable"
            break
        stamp_ns, frame = item
        if stamp_ns <= after_ns:
            reason = "nonmonotonic_acquisition"
            break
        after_ns = stamp_ns
        if clock() >= deadline:
            reason = "deadline"
            break
        trace = integrate(frame, ordinal, revision)
        traces.append(dict(trace, depth_acquisition_stamp_ns=stamp_ns))
        revision = next_expected_revision(trace, revision)
        if view_stop(trace):
            reason = "confirmed"
            break
        if not trace.get("integrated"):
            reason = "integration_refused"
            break
    final = dict(traces[-1]) if traces else {
        "success": False, "integrated": False, "empty_reason": reason}
    final["acquisition_burst"] = {"reason": reason, "frames": traces,
                                  "max_frames": MAX_FRAMES,
                                  "start_budget_s": WALL_BUDGET_S}
    return final, after_ns
