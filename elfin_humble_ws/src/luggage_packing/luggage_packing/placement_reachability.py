#!/usr/bin/env python3
"""Reachability-atlas adapter for placement candidates (no ROS).

The atlas is a coarse pre-computed grid, so it is used as a cheap prior plus a
conservative hard reject; MoveIt IK in ``placement_motion_filter`` stays the
only authority on feasibility.

Two conventions must match ``reachability_atlas_builder`` and
``packing_replay.FreeSpaceModelStrategy``:

  - the queried point is the *suction contact* at the box top, in
    ``container_link``: ``(lx, ly, floor_z + peak + box_height)``;
  - yaw is reduced modulo pi. The grid stores yaw bins ``[0, pi/2]`` because a
    tool-down footprint is pi-symmetric, so a pi-equivalent wrist branch must
    reuse the same cell instead of falling off the grid as a yaw mismatch.
"""

from __future__ import division

import math
import os

# Mirrors reachability_atlas status codes without importing the ROS-side module.
#
# MARGINAL is deliberately given the same prior as REACHABLE. Both mean "IK
# exists"; MARGINAL only adds "joint margin is tight", which the MoveIt filter
# and the tilt/retention gates already judge far more precisely downstream.
# Ranking MARGINAL below REACHABLE instead pulls placements toward the opening
# and measurably wastes container volume (offline replay: 9.6 -> 8.8 items).
# The prior's job here is to push away from cells the atlas says are unknown or
# unreachable, not to grade the ones it already accepts.
ATLAS_PRIOR_BY_STATUS = {
    0: 0.0,    # UNKNOWN: out of grid or unsampled - let MoveIt decide
    1: -1.0,   # UNREACHABLE
    2: 1.0,    # MARGINAL
    3: 1.0,    # REACHABLE
}

# Prior used when no atlas could be loaded: neutral, rejects nothing.
NEUTRAL_PRIOR = 0.5


def atlas_contact_point(candidate, floor_z):
    """Container_link point the atlas was sampled at for this candidate."""
    local = candidate["center_local"]
    box_height = float(candidate["size"][2])
    peak = float(candidate.get("peak", 0.0))
    return float(local[0]), float(local[1]), float(floor_z) + peak + box_height


def atlas_query_yaw(candidate):
    """Container-relative yaw reduced to [0, pi) to match the atlas yaw bins."""
    return float(candidate.get("box_yaw", 0.0)) % math.pi


def payload_atlas_path(atlas_dir, basename, box_size):
    """Path of the payload-matched atlas for ``box_size`` (may not exist)."""
    suffix = "_payload_%.2fx%.2fx%.2f" % tuple(float(v) for v in box_size)
    return os.path.join(atlas_dir, basename + suffix + ".npz")


def available_payload_sizes(atlas_dir, basename):
    """Payload sizes that have a built atlas, as ``[(size, npz_path), ...]``."""
    prefix = basename + "_payload_"
    found = []
    try:
        names = os.listdir(atlas_dir)
    except OSError:
        return found
    for name in sorted(names):
        if not name.startswith(prefix) or not name.endswith(".npz"):
            continue
        token = name[len(prefix):-len(".npz")]
        parts = token.split("x")
        if len(parts) != 3:
            continue
        try:
            size = [float(part) for part in parts]
        except ValueError:
            continue
        found.append((size, os.path.join(atlas_dir, name)))
    return found


def select_payload_atlas(atlas_dir, basename, box_size):
    """Smallest built atlas whose payload envelops ``box_size``.

    Atlases exist only for the three reference sizes, so a continuously sized
    box will never match one exactly. Choosing the nearest atlas could pick a
    *smaller* payload than the box being carried, which makes the reachability
    prior optimistic in the one direction that matters. Enveloping keeps it
    conservative; when nothing envelops the box, the largest available atlas is
    the closest available upper bound.
    """
    available = available_payload_sizes(atlas_dir, basename)
    if not available:
        return None
    def volume(entry):
        return entry[0][0] * entry[0][1] * entry[0][2]
    enveloping = [
        entry for entry in available
        if all(entry[0][i] >= float(box_size[i]) - 1e-9 for i in range(3))
    ]
    if enveloping:
        return min(enveloping, key=volume)[1]
    return max(available, key=volume)[1]


def resolve_atlas_path(atlas_dir, basename, box_size):
    """Conservative payload atlas if one exists, else the empty-load atlas.

    Returns ``(npz_path, meta_path)`` or ``None``.
    """
    candidates = []
    if box_size and min(float(v) for v in box_size) > 0.0:
        exact = payload_atlas_path(atlas_dir, basename, box_size)
        if os.path.isfile(exact):
            candidates.append(exact)
        selected = select_payload_atlas(atlas_dir, basename, box_size)
        if selected:
            candidates.append(selected)
    candidates.append(os.path.join(atlas_dir, basename + ".npz"))
    for npz_path in candidates:
        meta_path = os.path.splitext(npz_path)[0] + ".yaml"
        if os.path.isfile(npz_path) and os.path.isfile(meta_path):
            return npz_path, meta_path
    return None


def annotate_with_atlas(candidates, atlas, floor_z):
    """Annotate ``reachability_prior`` and drop only high-confidence rejects.

    ``atlas`` must expose ``query(x, y, z, yaw)`` returning a result with
    ``status`` and ``hard_reject_safe`` (``reachability_atlas.ReachabilityAtlas``).
    Candidates the atlas cannot speak to (out of grid, unknown cell) are kept
    with a neutral-to-low prior so the MoveIt filter still gets a chance.

    Returns ``(kept, rejected_count)``.
    """
    if atlas is None:
        for candidate in candidates:
            candidate["reachability_prior"] = NEUTRAL_PRIOR
            candidate["atlas_status_name"] = "no_atlas"
        return candidates, 0

    kept = []
    rejected = 0
    for candidate in candidates:
        x, y, z = atlas_contact_point(candidate, floor_z)
        result = atlas.query(x, y, z, atlas_query_yaw(candidate))
        status = int(result.status)
        candidate["reachability_prior"] = ATLAS_PRIOR_BY_STATUS.get(status, 0.0)
        candidate["atlas_status"] = status
        candidate["atlas_contact_z"] = round(z, 4)
        if result.hard_reject_safe:
            candidate["feasible"] = False
            candidate["reason"] = "atlas_unreachable"
            rejected += 1
            continue
        kept.append(candidate)
    return kept, rejected
