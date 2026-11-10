"""Cheap snug ranking for structural-first search (G3 §5).

``structural_snug_depth_v1``: support layer low-to-high -> adjacency A
high-to-low -> depth deep-to-shallow -> baseline score high-to-low ->
stable candidate id. No F/P, no next-box simulation, no second
reachability model.

A measures snugness against MEASURED neighbors: a neighbor counts when
its horizontal gap to the candidate footprint is within
``adjacency_max`` and the candidate's predicted top face matches the
neighbor's measured top within ``adjacency_top_tol`` (equal-height
side-by-side landing). The weight is the shared-edge projection length
times a linear gap decay, normalized by the candidate perimeter. In this
offline unit the neighbor tops come from map plateaus and the candidate
top from the envelope size it was evaluated with (envelope semantics);
the Attachment nominal/conservative split lands with integration.

Deterministic: neighbor order is the map scan order, the key is total
(stable id last), and identical inputs rank identically.
"""

DEFAULT_RANK_POLICY = {
    "adjacency_max": 0.08,
    "adjacency_top_tol": 0.01,
}


def adjacency_score(candidate, neighbors, policy=None, payload=None):
    """Snugness weight of one candidate against measured neighbors.

    ``payload`` (StructuralPayload) selects the ranking object: with it,
    the comparison top is the candidate's PREDICTED NOMINAL top (the
    measured box's top face once the envelope is placed); without it,
    the legacy envelope top. Gates always consume the envelope; only
    this comparison switches objects (user item 1).
    """
    pol = dict(DEFAULT_RANK_POLICY)
    if policy:
        pol.update(policy)
    a_max = float(pol["adjacency_max"])
    top_tol = float(pol["adjacency_top_tol"])
    if a_max <= 0.0 or not neighbors:
        return 0.0
    cx, cy, cz = candidate["center_container"]
    half_l = 0.5 * float(candidate["footprint"][0])
    half_w = 0.5 * float(candidate["footprint"][1])
    if payload is not None:
        cand_top = payload.predicted_nominal_top(
            candidate["center_container"])
    else:
        cand_top = cz + 0.5 * float(candidate["size"][2])
    perimeter = 2.0 * (2.0 * half_l + 2.0 * half_w)
    if perimeter <= 0.0:
        return 0.0
    weight = 0.0
    for nb in neighbors:
        if abs(cand_top - float(nb["top_z"])) > top_tol:
            continue
        nx0 = nb["cx"] - nb["half_l"]
        nx1 = nb["cx"] + nb["half_l"]
        ny0 = nb["cy"] - nb["half_w"]
        ny1 = nb["cy"] + nb["half_w"]
        dx = max(nx0 - (cx + half_l), (cx - half_l) - nx1, 0.0)
        dy = max(ny0 - (cy + half_w), (cy - half_w) - ny1, 0.0)
        if dx > 0.0 and dy > 0.0:
            continue  # diagonal neighbour: no shared edge projection
        gap = max(dx, dy)
        if gap > a_max:
            continue
        if dx > 0.0:
            shared = min(cy + half_w, ny1) - max(cy - half_w, ny0)
        elif dy > 0.0:
            shared = min(cx + half_l, nx1) - max(cx - half_l, nx0)
        else:
            continue  # footprint overlap: the overlap gate's business
        if shared <= 0.0:
            continue
        weight += shared * (1.0 - gap / a_max)
    return weight / perimeter


def snug_rank_key(neighbors, policy=None, payload=None):
    """Total-order key builder for ``structural_snug_depth_v1``."""

    def key(candidate):
        return (
            float(candidate.get("support_layer_z", 0.0)),
            -adjacency_score(candidate, neighbors, policy, payload=payload),
            -float(candidate.get("depth_m", 0.0)),
            -float(candidate.get("score", 0.0)),
            str(candidate.get("candidate_id", "")),
        )

    return key


def snug_rank(candidates, neighbors, policy=None, payload=None):
    """Rank feasible candidates; input order only breaks nothing (the
    key is total). Returns a new list, input untouched."""
    ranked = list(candidates)
    ranked.sort(key=snug_rank_key(neighbors, policy, payload=payload))
    for cand in ranked:
        cand["adjacency_a"] = round(
            adjacency_score(cand, neighbors, policy, payload=payload), 6)
    return ranked
