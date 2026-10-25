"""Structural candidate generation + lazy grid fallback (G3 §4-§5).

Structural-first is a SEARCH ORDER change, not a permanent deletion of
grid positions: proposals come from structure (container boundary,
placed-box edges, platform boundaries, their bounded intersections),
then share the exact solver gate semantics; the full grid stays
available as a lazy fallback evaluated in the solver's own order.

Completeness statement (plan §1.3, user-tightened): the discrete
candidate COVERAGE guarantee holds only after a full fallback traversal
under sufficient budget; a budget cut is reported as such and never
phrased as completeness.

ROS-free, deterministic, no cross-request state. Gate verdicts are the
solver's own helpers on the same inputs, so per-candidate equality with
the exhaustive chain is exact (oracle-tested), not approximated.
"""

import math
import time

from luggage_packing.placement_solver import (
    REASON_INSUFFICIENT_CLEARANCE,
    REASON_INSUFFICIENT_SUPPORT,
    REASON_OUTSIDE_HULL,
    REASON_SUPPORT_TILT_UNSUPPORTED,
    REASON_UNKNOWN_ABOVE_FLOOR,
    VERDICT_LEGAL,
    _REASON_TO_VERDICT,
    _depth_datum,
    _depth_m,
    _hull_legal,
    _support_layer_z,
    _support_ratio,
    _support_tilt_ratio,
    _surface_hull,
    _validator_verdict,
    v1_candidate_view,
)
from luggage_packing.placement_support_index import (
    SupportIndex,
    plateau_components,
    plateau_rect_m,
)

DEFAULT_STRUCTURAL_POLICY = {
    # Same-layer gap used when proposing snug offsets (mirrors the
    # frozen solver clearance when the caller does not override).
    "gap_min": 0.05,
    # Proposal cap per request: a search-policy bound, NOT a capacity
    # claim; overrun positions stay reachable via the lazy fallback.
    "max_structural_proposals": 512,
    "fallback_batch_size": 64,
    "fallback_budget_s": 20.0,
    "adjacency_max": 0.08,
    "adjacency_top_tol": 0.01,
}


# ----------------------------------------------------------------------
# Exact single-window evaluation (solver semantics, shared by every path)


def evaluate_window(index, cls, ix0, iy0, cfg, hull, datum,
                    candidate_validator=None):
    """One window through the exact ``generate_candidates`` semantics.

    Produces the same candidate dict (id, fields, verdict, reason) the
    exhaustive chain produces for that window, so structural and
    fallback candidates compare by identity.
    """
    surface_map = index._map
    box_l, box_w, box_h = index._box_size
    resolution = index.resolution
    inner_l, inner_w, inner_h = index.inner_size
    half_l, half_w = 0.5 * inner_l, 0.5 * inner_w
    half_h = 0.5 * inner_h
    floor_z = float(surface_map.get("floor_z", 0.0))
    center_base = surface_map["center_base"]

    footprint_l, footprint_w, box_yaw = index.footprint(cls)
    cells_x, cells_y = index.window_cells(cls)
    headings = [0.0 if cls == 0 else math.pi / 2.0]
    if str(cfg.get("payload_symmetry", "none")).lower() == "pi":
        headings.append(headings[0] + math.pi)

    peak, has_unknown, cell_heights, confidence_ratio = index.window_stats(
        cls, ix0, iy0)
    support_ratio = _support_ratio(cell_heights, peak, cfg["support_tol"])
    top_of_box = peak + box_h
    clearance_top = inner_h - top_of_box

    lx = -half_l + (ix0 + cells_x * 0.5) * resolution
    ly = -half_w + (iy0 + cells_y * 0.5) * resolution
    floor_container_z = center_base[2] - half_h
    center_container = [lx, ly, floor_container_z + peak + box_h * 0.5]
    depth = _depth_m(center_container, datum[0], datum[1])
    support_layer = _support_layer_z(peak, resolution)

    feasible = True
    reason = "ok"
    support_source = "sensor"
    clearance_ok = clearance_top >= cfg["clearance_margin"]
    if has_unknown:
        if abs(peak - floor_z) > cfg["support_tol"]:
            feasible = False
            reason = REASON_UNKNOWN_ABOVE_FLOOR
        else:
            support_source = "floor_prior"
    tilt_record = {
        "support_span_m": 0.0,
        "support_tilt_ratio": 0.0,
        "max_support_tilt_ratio": float(cfg["max_support_tilt_ratio"]),
    }
    if feasible and not clearance_ok:
        feasible = False
        reason = REASON_INSUFFICIENT_CLEARANCE
    elif feasible and support_ratio < cfg["min_support_ratio"]:
        feasible = False
        reason = REASON_INSUFFICIENT_SUPPORT
    elif feasible:
        tilt_span = _support_tilt_ratio(cell_heights, peak, cfg["support_tol"])
        tilt_ratio = tilt_span / max(footprint_l, footprint_w, 1e-6)
        if tilt_ratio > float(cfg["max_support_tilt_ratio"]):
            feasible = False
            reason = REASON_SUPPORT_TILT_UNSUPPORTED
        tilt_record = {
            "support_span_m": round(tilt_span, 6),
            "support_tilt_ratio": round(tilt_ratio, 6),
            "max_support_tilt_ratio": float(cfg["max_support_tilt_ratio"]),
        }

    clearance_score = max(0.0, min(1.0, clearance_top / max(box_h, 1e-6)))
    compactness = 1.0 - min(1.0, peak / max(inner_h, 1e-6))
    w_conf = float(cfg.get("w_confidence", 0.0))
    score = (cfg["w_support"] * support_ratio
             + cfg["w_clearance"] * clearance_score
             + cfg["w_compactness"] * compactness
             + w_conf * confidence_ratio)

    hull_ok = True
    hull_margin_detail = {}
    if hull is not None:
        hull_ok, hull_margin_detail = _hull_legal(
            center_container, (footprint_l, footprint_w), box_h, hull,
            cfg["hull_margin"])

    out = []
    for heading_index, heading_theta in enumerate(headings):
        candidate = {
            "candidate_id": "c%03d_%03d_f%d_h%d" % (
                ix0, iy0, cls, heading_index),
            "center_container": center_container,
            "box_yaw": box_yaw,
            "heading_theta": heading_theta,
            "footprint_class": cls,
            "support_layer_z": support_layer,
            "depth_m": round(depth, 6),
            "ix0": ix0,
            "iy0": iy0,
            "cells_x": cells_x,
            "cells_y": cells_y,
            "footprint": [footprint_l, footprint_w],
            "size": [box_l, box_w, box_h],
            "support_score": round(support_ratio, 4),
            "clearance_score": round(clearance_score, 4),
            "clearance_top": round(clearance_top, 4),
            "collision_margin": round(clearance_top, 4),
            "confidence_ratio": round(confidence_ratio, 4),
            "support_source": support_source,
            "reachability_score": -1.0,
            "score": round(score, 4),
            "feasible": feasible,
            "reason": reason,
            "capacity_feasible": bool(clearance_ok and hull_ok),
            "detail": {
                "route_clearance_m": 0.0,
                "route_not_evaluated": True,
                "tilt": tilt_record,
                "margins": {
                    "hull_static_wall_and_chamfer_m": float(cfg["hull_margin"]),
                    "top_clearance_m": float(cfg["clearance_margin"]),
                    "support_contact_tol_m": float(cfg["support_tol"]),
                    **hull_margin_detail,
                },
                "hull_from_descriptor": hull is not None,
            },
        }
        v1_candidate_view(candidate, surface_map)
        if clearance_ok and candidate_validator is not None:
            constraint_reason, capacity_ok = _validator_verdict(
                candidate_validator, candidate)
            candidate["capacity_feasible"] = bool(capacity_ok and hull_ok)
            if feasible and constraint_reason:
                candidate["feasible"] = False
                candidate["reason"] = constraint_reason
        if feasible and not hull_ok:
            candidate["feasible"] = False
            candidate["reason"] = REASON_OUTSIDE_HULL
        candidate["verdict"] = _REASON_TO_VERDICT.get(
            candidate["reason"] if candidate["reason"] != "ok" else None,
            VERDICT_LEGAL)
        out.append(candidate)
    return out


# ----------------------------------------------------------------------
# Structural proposals (continuous center space -> window origins)


def _window_origin_candidates(index, cls, px, py):
    """Map one continuous box-center point to up to 4 window origins."""
    import math as _math
    cells_x, cells_y = index.window_cells(cls)
    half_l = 0.5 * index.inner_size[0]
    half_w = 0.5 * index.inner_size[1]
    res = index.resolution
    fx = (px + half_l) / res - cells_x * 0.5
    fy = (py + half_w) / res - cells_y * 0.5
    out = []
    for ix in {_math.floor(fx), _math.ceil(fx)}:
        if not 0 <= ix <= index.nx - cells_x:
            continue
        for iy in {_math.floor(fy), _math.ceil(fy)}:
            if not 0 <= iy <= index.ny - cells_y:
                continue
            out.append((ix, iy))
    return out


def _boundary_points(index, cls, floor_y_limit):
    footprint_l, footprint_w, _yaw = index.footprint(cls)
    half_l = 0.5 * index.inner_size[0]
    half_w = 0.5 * index.inner_size[1]
    fl, fw = 0.5 * footprint_l, 0.5 * footprint_w
    xs = [-half_l + fl, 0.0, half_l - fl]
    ys = [-half_w + fw, 0.0, half_w - fw]
    # Chamfer: at floor level the usable +Y boundary is floor_y_limit,
    # so the shallow-layer +Y snug line is inside it by half the width.
    ys.append(floor_y_limit - fw)
    return [(x, y, "container_boundary") for x in xs for y in ys]


def _plateau_edge_points(index, cls, surface_map, plateaus, gap):
    footprint_l, footprint_w, _yaw = index.footprint(cls)
    fl, fw = 0.5 * footprint_l, 0.5 * footprint_w
    points = []
    for plateau in plateaus:
        x0, y0, x1, y1 = plateau_rect_m(plateau, surface_map)
        cy = 0.5 * (y0 + y1)
        cx = 0.5 * (x0 + x1)
        # East / west offset lines (candidate long axis along X).
        east_x = x1 + gap + fl
        west_x = x0 - gap - fl
        for x in (east_x, west_x):
            for y in (y0 - gap, y0, cy, y1, y1 + gap):
                points.append((x, y, "plateau_edge"))
        # North / south offset lines.
        north_y = y1 + gap + fw
        south_y = y0 - gap - fw
        for y in (north_y, south_y):
            for x in (x0 - gap, x0, cx, x1, x1 + gap):
                points.append((x, y, "plateau_edge"))
    return points


def _edge_line_intersections(index, cls, surface_map, plateaus, gap):
    """Bounded intersections: only between facing offset lines whose
    plateaus overlap in projection; never a full cartesian product."""
    footprint_l, footprint_w, _yaw = index.footprint(cls)
    fl, fw = 0.5 * footprint_l, 0.5 * footprint_w
    vlines = []  # x of vertical offset lines, with owner y-range
    hlines = []
    for plateau in plateaus:
        x0, y0, x1, y1 = plateau_rect_m(plateau, surface_map)
        vlines.append((x1 + gap + fl, y0, y1, plateau))
        vlines.append((x0 - gap - fl, y0, y1, plateau))
        hlines.append((y1 + gap + fw, x0, x1, plateau))
        hlines.append((y0 - gap - fw, x0, x1, plateau))
    points = []
    for vx, vy0, vy1, _pv in vlines:
        for hy, hx0, hx1, _ph in hlines:
            if not (hx0 - gap <= vx <= hx1 + gap):
                continue
            if not (vy0 - gap <= hy <= vy1 + gap):
                continue
            points.append((vx, hy, "edge_intersection"))
    # Snug midlines between facing plateau edges.
    n = len(plateaus)
    for i in range(n):
        ri = plateau_rect_m(plateaus[i], surface_map)
        for j in range(i + 1, n):
            rj = plateau_rect_m(plateaus[j], surface_map)
            # Facing along X and overlapping in Y.
            if not (ri[1] - gap <= rj[3] and rj[1] - gap <= ri[3]):
                continue
            gap_x = rj[0] - ri[2]
            if 0.0 <= gap_x <= 2.0 * (gap + fl):
                mid = 0.5 * (ri[2] + rj[0])
                y = 0.5 * (max(ri[1], rj[1]) + min(ri[3], rj[3]))
                points.append((mid, y, "facing_midline"))
            gap_x = ri[0] - rj[2]
            if 0.0 <= gap_x <= 2.0 * (gap + fl):
                mid = 0.5 * (rj[2] + ri[0])
                y = 0.5 * (max(ri[1], rj[1]) + min(ri[3], rj[3]))
                points.append((mid, y, "facing_midline"))
    return points


def generate_proposals(index, surface_map, policy=None, floor_y_limit=None):
    """Structural proposals for one request.

    Returns ``(proposals, report)`` where proposals is a deduplicated
    list of ``(cls, ix0, iy0, provenance_tuple)`` in deterministic
    source-rotation order, and report carries the per-source counts and
    the cap accounting. ``floor_y_limit`` is the chamfer floor limit in
    container frame (deployment: chamfer.floor_y); None falls back to
    half width (cuboid).
    """
    pol = dict(DEFAULT_STRUCTURAL_POLICY)
    if policy:
        pol.update(policy)
    if floor_y_limit is None:
        hull = _surface_hull(surface_map)
        if hull is not None and hull.chamfer is not None:
            floor_y_limit = hull.chamfer.floor_y
        else:
            floor_y_limit = 0.5 * index.inner_size[1]

    gap = float(pol["gap_min"])
    plateaus = plateau_components(surface_map)
    proposals = {}
    report = {"sources": {}, "generated": 0, "emitted": 0,
              "capped": 0, "cap": int(pol["max_structural_proposals"])}
    for cls in index.classes():
        raw = []
        raw.extend(_boundary_points(index, cls, floor_y_limit))
        raw.extend(_plateau_edge_points(index, cls, surface_map, plateaus, gap))
        raw.extend(_edge_line_intersections(
            index, cls, surface_map, plateaus, gap))
        report["generated"] += len(raw)
        for px, py, source in raw:
            for ix0, iy0 in _window_origin_candidates(index, cls, px, py):
                key = (cls, ix0, iy0)
                entry = proposals.get(key)
                if entry is None:
                    if len(proposals) >= int(pol["max_structural_proposals"]):
                        report["capped"] += 1
                        continue
                    proposals[key] = (cls, ix0, iy0, (source,))
                    report["sources"][source] = (
                        report["sources"].get(source, 0) + 1)
                else:
                    if source not in entry[3]:
                        proposals[key] = (
                            cls, ix0, iy0, entry[3] + (source,))
    ordered = [proposals[key] for key in sorted(proposals)]
    report["emitted"] = len(ordered)
    return ordered, report


# ----------------------------------------------------------------------
# Lazy grid fallback


def fallback_batch(index, cfg, hull, datum, seen_ids, cursor=None,
                   batch_size=64, budget_s=20.0, candidate_validator=None):
    """Evaluate grid windows in the solver's own loop order.

    Skips ``seen_ids``, stops at ``batch_size`` feasible candidates or
    when ``budget_s`` elapses (reported, never silently), and marks
    ``coverage_complete`` only after every window of every class was
    visited. Deterministic; shares ``evaluate_window`` and the index
    stats memo with the structural path.
    """
    deadline = time.perf_counter() + float(budget_s)
    cursor = dict(cursor or {})
    out = []
    feasible_seen = 0
    exhausted = True
    budget_exhausted = False
    for cls in index.classes():
        position = cursor.get(cls, 0)
        rng_x, rng_y = index.window_origin_range(cls)
        row_x = list(rng_x)
        row_y = list(rng_y)
        total = len(row_x) * len(row_y)
        linear = position
        while linear < total:
            ix0 = row_x[linear // len(row_y)]
            iy0 = row_y[linear % len(row_y)]
            linear += 1
            cands = evaluate_window(
                index, cls, ix0, iy0, cfg, hull, datum,
                candidate_validator=candidate_validator)
            fresh = [c for c in cands
                     if c["candidate_id"] not in seen_ids]
            for cand in fresh:
                seen_ids.add(cand["candidate_id"])
            out.extend(fresh)
            feasible_seen += sum(1 for c in fresh if c["feasible"])
            if feasible_seen >= batch_size:
                cursor[cls] = linear
                exhausted = False
                break
            if time.perf_counter() > deadline:
                cursor[cls] = linear
                exhausted = False
                budget_exhausted = True
                break
        if exhausted:
            cursor[cls] = 0  # class complete
        else:
            break
    return {
        "candidates": out,
        "cursor": cursor,
        "coverage_complete": exhausted and not budget_exhausted,
        "budget_exhausted": budget_exhausted,
    }


# ----------------------------------------------------------------------
# Full structural-first search (offline comparison entry)


def structural_search(surface_map, box_size, allowed_yaws=None, params=None,
                      structural_policy=None, candidate_validator=None,
                      payload=None):
    """Structural-first placement search with lazy grid fallback.

    Returns a result dict with the ranked candidate list, the chosen
    candidate, and per-stage diagnostics. Gate semantics are identical
    to ``generate_candidates`` (oracle-tested); the ordering is the snug
    ranking (see placement_snug_ranking), NOT the old sort.

    ``payload`` (StructuralPayload) splits the two objects: the hard
    gates and window enumeration always consume the CONSERVATIVE
    ENVELOPE (``box_size`` must equal ``payload.envelope`` when both
    are given); the snug term compares the PREDICTED NOMINAL top
    against neighbour measured tops. Without a payload the legacy
    single-object behaviour is kept (envelope everywhere).
    """
    from luggage_packing.placement_snug_ranking import snug_rank
    from luggage_packing.placement_solver import _default_params

    cfg = _default_params()
    if params:
        cfg.update(params)
    cfg["top_n"] = 10 ** 9
    cfg["keep_rejected"] = 10 ** 9
    if payload is not None and tuple(
            float(v) for v in box_size) != tuple(payload.envelope):
        raise ValueError(
            "PAYLOAD_ENVELOPE_MISMATCH: box_size must be the payload's "
            "conservative envelope (gates consume the envelope; got %s "
            "vs %s)" % (list(box_size), list(payload.envelope)))

    stages = {}
    t0 = time.perf_counter()
    index = SupportIndex(surface_map, box_size,
                         allowed_yaws or [0.0], cfg)
    hull = _surface_hull(surface_map)
    datum = _depth_datum(surface_map, hull)
    neighbors = _neighbor_records(surface_map)
    stages["index_build_s"] = round(time.perf_counter() - t0, 6)

    t_gen = time.perf_counter()
    proposals, proposal_report = generate_proposals(
        index, surface_map, structural_policy)
    stages["structural_gen_s"] = round(time.perf_counter() - t_gen, 6)

    t_gate = time.perf_counter()
    wmax = {}
    structural_candidates = []
    prefilter_rejected = 0
    for cls, ix0, iy0, _prov in proposals:
        if cls not in wmax:
            wmax[cls] = index.window_peak_prefilter(cls)
        cells_y = index.window_cells(cls)[1]
        peak = float(wmax[cls][ix0, iy0])
        if (index.inner_size[2] - (peak + index._box_size[2])
                < cfg["clearance_margin"]):
            # Cheap necessary condition from the precomputed windowed
            # max: no cell scan for this window at all.
            prefilter_rejected += 1
            continue
        structural_candidates.extend(evaluate_window(
            index, cls, ix0, iy0, cfg, hull, datum,
            candidate_validator=candidate_validator))
    stages["structural_gates_s"] = round(time.perf_counter() - t_gate, 6)

    feasible = [c for c in structural_candidates if c["feasible"]]
    fallback_info = None
    if not feasible:
        t_fb = time.perf_counter()
        seen = {c["candidate_id"] for c in structural_candidates}
        pol = dict(DEFAULT_STRUCTURAL_POLICY)
        if structural_policy:
            pol.update(structural_policy)
        batch = fallback_batch(
            index, cfg, hull, datum, seen,
            batch_size=int(pol["fallback_batch_size"]),
            budget_s=float(pol["fallback_budget_s"]),
            candidate_validator=candidate_validator)
        stages["fallback_s"] = round(time.perf_counter() - t_fb, 6)
        feasible = [c for c in batch["candidates"] if c["feasible"]]
        fallback_info = batch

    t_rank = time.perf_counter()
    ranked = snug_rank(feasible, neighbors, structural_policy, payload=payload)
    stages["rank_s"] = round(time.perf_counter() - t_rank, 6)

    chosen = ranked[0] if ranked else None
    return {
        "success": chosen is not None,
        "selected": chosen,
        "ranked": ranked,
        "structural_candidates": structural_candidates,
        "fallback": fallback_info,
        "neighbors": neighbors,
        "diagnostics": {
            "stages": stages,
            "proposals": proposal_report,
            "structural_feasible": len(feasible) if not fallback_info else 0,
            "structural_evaluated": len(structural_candidates),
            "prefilter_rejected": prefilter_rejected,
            "stats_memo": index.stats_memo_size(),
        },
    }


def _neighbor_records(surface_map):
    from luggage_packing.placement_support_index import (
        structural_neighbor_records)
    return structural_neighbor_records(surface_map)
