#!/usr/bin/env python3
"""Reachability-atlas adapter for placement candidates (no ROS).

The atlas is a coarse pre-computed grid, so it is used as a cheap prior plus a
conservative hard reject; MoveIt IK in ``placement_motion_filter`` stays the
only authority on feasibility.

Two conventions must match ``reachability_atlas_builder`` and
``packing_replay.FreeSpaceModelStrategy``:

  - the queried point is the *suction contact* at the box top, in
    ``container_link``: ``(lx, ly, floor_z + peak + box_height)``;
  - yaw: a v2 atlas (ST06/F14) stores DIRECTED concrete-tool bins with 2pi
    period, so the hook queries BOTH pi-equivalent grasps (``box_yaw`` and
    ``box_yaw + pi``) and hard-rejects only when both branches are
    confidently unreachable interior cells -- a failed branch never
    certifies its pi twin. A v1 atlas (pi-folded bins, no contract block)
    keeps the legacy single folded query.
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
    """Container-relative yaw reduced to [0, pi) to match v1 atlas bins."""
    return float(candidate.get("box_yaw", 0.0)) % math.pi


def atlas_query_yaws(atlas, candidate):
    """Query yaws for one candidate grasp under the artifact's semantics.

    v2 directed artifact: the DIRECTED box yaw (no fold) and its pi twin,
    both snapped per ``atlas_runtime``. v1 artifact (or a bare test stub):
    the legacy folded yaw alone.
    """
    box_yaw = float(candidate.get("box_yaw", 0.0))
    try:
        from luggage_planning.atlas.runtime import directed_yaw_candidates
    except ImportError:  # pragma: no cover - planning package missing
        return [box_yaw % math.pi]
    return directed_yaw_candidates(atlas, box_yaw)


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
    On a v2 directed artifact both pi-equivalent grasps are queried and only
    a candidate whose BOTH branches are confidently unreachable is dropped.
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
        result, all_hard_reject = _query_grasp(atlas, candidate, x, y, z)
        status = int(result.status)
        candidate["reachability_prior"] = ATLAS_PRIOR_BY_STATUS.get(status, 0.0)
        candidate["atlas_status"] = status
        candidate["atlas_contact_z"] = round(z, 4)
        if all_hard_reject:
            candidate["feasible"] = False
            candidate["reason"] = "atlas_unreachable"
            rejected += 1
            continue
        kept.append(candidate)
    return kept, rejected


def _query_grasp(atlas, candidate, x, y, z):
    """One grasp query under the artifact's yaw semantics.

    Prefers the atlas' own both-branch query (v2 semantics layer); a bare
    stub with only ``query`` falls back to the single folded query.
    """
    pair_query = getattr(atlas, "query_pi_equivalents", None)
    if callable(pair_query):
        result, all_hard = pair_query(x, y, z, float(
            candidate.get("box_yaw", 0.0)))
        return result, all_hard
    result = atlas.query(x, y, z, atlas_query_yaw(candidate))
    return result, bool(result.hard_reject_safe)


def solver_contact_point(candidate, floor_z, inner_h):
    """Container_link contact point for a placement_solver candidate.

    ST05: solver candidates carry ``center_container`` (box center above
    the container floor), so the contact z is center + h/2 with no
    volume-center inversion. The legacy ``center_local`` path stays for
    pre-ST05 replay rows.
    """
    box_height = float(candidate["size"][2])
    center = candidate.get("center_container")
    if center is not None:
        return float(center[0]), float(center[1]), float(center[2]) + box_height / 2.0
    local = candidate["center_local"]
    peak = float(local[2]) + float(inner_h) / 2.0 - box_height / 2.0
    return float(local[0]), float(local[1]), float(floor_z) + peak + box_height


def make_atlas_annotate(atlas, floor_z, inner_h, score_weight=0.3,
                        hard_reject=True, dump_only=False, query_offset=None):
    """Build the runtime atlas hook for ``solve_placement``.

    Records ``reachability_prior`` / ``atlas_status`` / ``atlas_status_name``
    / ``atlas_contact_z`` on every candidate. Unless ``dump_only``:
    high-confidence unreachable cells (``hard_reject_safe``) flip to
    ``feasible=False, reason="atlas_unreachable"`` (kept in the list so the
    reject histogram sees them) and reachable/marginal candidates gain
    ``score += score_weight * prior`` so selection prefers covered cells.

    Returns ``hook(candidates) -> (candidates, report)``.
    """
    from luggage_planning.atlas.runtime import directed_yaw_candidates

    offset = [float(v) for v in (query_offset or (0.0, 0.0, 0.0))]

    def hook(candidates):
        report = {
            "atlas": atlas is not None,
            "n_candidates": len(candidates),
            "n_hard_rejected": 0,
            "status_histogram": {},
        }
        if atlas is None:
            for candidate in candidates:
                candidate["reachability_prior"] = NEUTRAL_PRIOR
                candidate["atlas_status"] = None
                candidate["atlas_status_name"] = "no_atlas"
            report["status_histogram"] = {"no_atlas": len(candidates)}
            return candidates, report

        def bump(status):
            report["status_histogram"][status] = (
                report["status_histogram"].get(status, 0) + 1)

        for candidate in candidates:
            x, y, z = solver_contact_point(candidate, floor_z, inner_h)
            # ST06: query the DIRECTED box yaw and, on a v2 artifact, its
            # pi twin too. The hard reject needs both branches confidently
            # unreachable; the best verdict annotates the candidate.
            yaws = directed_yaw_candidates(
                atlas, float(candidate.get("box_yaw", 0.0)))
            results = [
                atlas.query(
                    x + offset[0], y + offset[1], z + offset[2], bin_yaw)
                for bin_yaw in yaws]
            result = _best_result(results)
            all_hard_reject = bool(results) and all(
                r.hard_reject_safe for r in results)
            status = int(result.status)
            prior = ATLAS_PRIOR_BY_STATUS.get(status, 0.0)
            candidate["reachability_prior"] = prior
            candidate["atlas_status"] = status
            candidate["atlas_status_name"] = str(result.reason)
            candidate["atlas_contact_z"] = round(z, 4)
            candidate["atlas_yaw_bins_queried"] = [
                round(float(v), 6) for v in yaws]
            candidate["atlas_query"] = _atlas_query_record(
                candidate, x, y, z, offset, yaws, result, all_hard_reject,
                atlas)
            bump(str(result.reason))
            if dump_only:
                continue
            if hard_reject and all_hard_reject:
                candidate["feasible"] = False
                candidate["reason"] = "atlas_unreachable"
                report["n_hard_rejected"] += 1
                continue
            candidate["score"] = round(
                float(candidate.get("score", 0.0))
                + float(score_weight) * prior, 4)
        return candidates, report

    return hook


def _atlas_query_record(candidate, x, y, z, offset, yaws, result,
                        all_hard_reject, atlas):
    """Dump of the query that already ran. Does not query again."""
    center = candidate.get("center_container")
    if center is None:
        center = candidate.get("center_local") or [x, y, z]
    meta = getattr(atlas, "meta", None) or {}
    dependencies = meta.get("dependencies") or {}
    indices = getattr(result, "indices", None)
    return {
        "center": [round(float(v), 6) for v in center[:3]],
        "yaw": round(float(candidate.get("box_yaw", 0.0)), 6),
        "contact_xyz": [round(float(x), 6), round(float(y), 6),
                        round(float(z), 6)],
        "query_offset": [round(float(v), 6) for v in offset],
        "query_offset_applied": True,
        "query_xyz": [
            round(float(x) + float(offset[0]), 6),
            round(float(y) + float(offset[1]), 6),
            round(float(z) + float(offset[2]), 6)],
        "query_yaw": [round(float(v), 6) for v in yaws],
        "cell": None if indices is None else [int(v) for v in indices],
        "status": int(result.status),
        "hard_reject_safe": bool(getattr(result, "hard_reject_safe", False)),
        "all_hard_reject_safe": bool(all_hard_reject),
        "scene_tf_hash": dependencies.get("scene_tf_hash") or "",
        "urdf_hash": dependencies.get("urdf_hash") or "",
    }


def _best_result(results):
    """Best of the directed-branch results; ties keep the earlier branch."""
    def rank(result):
        return (
            1 if int(result.status) in (2, 3) else 0,
            1 if (result.contact_seeds or result.transit_seeds) else 0,
            int(result.status),
        )

    best = results[0]
    for result in results[1:]:
        if rank(result) > rank(best):
            best = result
    return best
