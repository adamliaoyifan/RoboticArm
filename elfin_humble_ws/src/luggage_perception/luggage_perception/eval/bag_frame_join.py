"""Eval-only pure timestamp join for bag replay. No I/O, no ROS.

The pendant bags share one header stamp between the colour image and the
aligned depth image (same sensor clock, verified on the real bags), so the
primary join is an exact (sec, nanosec) key match; the tolerance pass only
rescues frames whose recorder dropped one side. Aux topics
(/joint_states, /elfin/tcp_pose, /livox/lidar) are joined by nearest
header stamp within a per-topic tolerance — a miss is reported, never
silently widened.
"""
from __future__ import division

import bisect
from dataclasses import dataclass, field

NS_PER_SEC = 1_000_000_000


def nearest_stamp(sorted_stamps_ns, t_ns, max_dt_ns):
    """Nearest entry to *t_ns* within ±*max_dt_ns* over an ascending list.

    Returns ``(index, dt_ns)`` with ``dt_ns = entry - t_ns`` (negative when
    the entry is older), or ``None`` outside the window. Ties prefer the
    earlier (smaller) stamp.
    """
    if not sorted_stamps_ns:
        return None
    idx = bisect.bisect_left(sorted_stamps_ns, t_ns)
    best_idx, best_dt = None, None
    for cand in (idx - 1, idx):
        if cand < 0 or cand >= len(sorted_stamps_ns):
            continue
        dt = int(sorted_stamps_ns[cand]) - int(t_ns)
        if abs(dt) > int(max_dt_ns):
            continue
        if best_dt is None or abs(dt) < abs(best_dt):
            best_idx, best_dt = cand, dt
    if best_idx is None:
        return None
    return best_idx, best_dt


@dataclass
class FrameJoinPair(object):
    stamp_ns: int
    depth_stamp_ns: int
    source: str  # "exact" | "tolerance"
    dt_ns: int


@dataclass
class FrameJoinPlan(object):
    pairs: list = field(default_factory=list)
    color_orphans: list = field(default_factory=list)
    depth_orphans: list = field(default_factory=list)
    duplicates: dict = field(default_factory=dict)
    stats: dict = field(default_factory=dict)


def dedupe_stamped_entries(entries):
    """Collapse same-stamp entries, keeping the newest log_time.

    ``entries`` is a list of ``(stamp_ns, log_time_ns, payload)``; returns
    ``(sorted_entries, duplicates)`` where duplicates maps stamp_ns to the
    dropped count. Payloads may be None for stamp-only indexing.
    """
    best = {}
    duplicates = {}
    for stamp, log_time, payload in entries:
        stamp = int(stamp)
        if stamp in best:
            prev_log, _prev_payload = best[stamp]
            duplicates[stamp] = duplicates.get(stamp, 0) + 1
            if int(log_time) >= int(prev_log):
                best[stamp] = (int(log_time), payload)
        else:
            best[stamp] = (int(log_time), payload)
    ordered = sorted(
        (stamp, log_time, payload) for stamp, (log_time, payload) in
        best.items())
    return ordered, duplicates


def plan_frame_join(color_stamps_ns, depth_stamps_ns,
                    tolerance_ns=1_000_000):
    """Plan the colour↔depth frame join (colour is the primary clock).

    Pass 1 pairs exact (sec, nanosec) matches; pass 2 greedily pairs each
    still-unmatched colour (ascending) with the nearest unmatched depth
    inside *tolerance_ns*, one-to-one; the rest are orphans on either
    side.
    """
    color_sorted = sorted(int(s) for s in color_stamps_ns)
    depth_sorted = sorted(int(s) for s in depth_stamps_ns)
    depth_available = set(depth_sorted)
    depth_pool = list(depth_sorted)
    pairs = []
    paired_color = set()

    for stamp in color_sorted:
        if stamp in depth_available:
            depth_available.discard(stamp)
            depth_pool.remove(stamp)
            paired_color.add(stamp)
            pairs.append(FrameJoinPair(
                stamp_ns=stamp, depth_stamp_ns=stamp,
                source="exact", dt_ns=0))

    color_orphans = []
    for stamp in color_sorted:
        if stamp in paired_color:
            continue
        hit = nearest_stamp(depth_pool, stamp, tolerance_ns)
        if hit is not None:
            idx, dt = hit
            depth_stamp = depth_pool.pop(idx)
            depth_available.discard(depth_stamp)
            pairs.append(FrameJoinPair(
                stamp_ns=stamp, depth_stamp_ns=depth_stamp,
                source="tolerance", dt_ns=int(dt)))
        else:
            color_orphans.append(stamp)

    pairs.sort(key=lambda p: p.stamp_ns)
    depth_orphans = sorted(depth_pool)
    stats = {
        "n_color": len(color_sorted),
        "n_depth": len(depth_sorted),
        "n_pairs": len(pairs),
        "n_exact": sum(1 for p in pairs if p.source == "exact"),
        "n_tolerance": sum(1 for p in pairs if p.source == "tolerance"),
        "n_color_orphans": len(color_orphans),
        "n_depth_orphans": len(depth_orphans),
    }
    return FrameJoinPlan(
        pairs=pairs, color_orphans=color_orphans,
        depth_orphans=depth_orphans, stats=stats)


def frame_dir_name(stamp_ns):
    """Zero-padded sortable '<sec>_<nsec>' directory name."""
    stamp_ns = int(stamp_ns)
    return "%d_%09d" % (stamp_ns // NS_PER_SEC, stamp_ns % NS_PER_SEC)


def frame_dir_name_to_ns(name):
    sec, _, nsec = str(name).partition("_")
    return int(sec) * NS_PER_SEC + int(nsec)
