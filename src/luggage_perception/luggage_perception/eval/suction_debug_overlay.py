#!/usr/bin/env python3
"""Suction debug overlay geometry (B7, plan section B).

Projects accepted candidate footprints and a bounded set of rejected
candidate ids into image pixels with a stable colour per rejection
reason. This module produces the GEOMETRY (corners, ids, colours) that
the on-node debug overlay and RViz marker adapter render; nothing here
feeds back into online selection (plan B7). Eval-layer: numpy/stdlib
only.
"""

from __future__ import division

import numpy as np

#: stable reason -> colour map (the B7 golden fixture compares these).
REJECTION_COLOURS = {
    "SUCTION_REJECT_ADJACENT_STEP": "red",
    "SUCTION_REJECT_BIMODAL": "red",
    "SUCTION_REJECT_PEAK_TO_VALLEY": "red",
    "SUCTION_REJECT_NORMAL_DEVIATION": "red",
    "SUCTION_REJECT_NORMAL_TILT": "red",
    "SUCTION_REJECT_RMS": "crimson",
    "SUCTION_REJECT_P95": "crimson",
    "SUCTION_REJECT_PLANE_COVERAGE": "orange",
    "SUCTION_REJECT_VALID_CELLS": "orange",
    "SUCTION_REJECT_MASK_COVERAGE": "orange",
    "SUCTION_REJECT_NORMAL_NO_SAMPLES": "orange",
}
ACCEPTED_COLOUR = "green"
#: rejected candidates rendered per case (bounded, golden-fixture scope).
MAX_REJECTED_SHOWN = 12


def _world_to_pixel(world_xyz, camera):
    mat = np.asarray(camera["optical_to_world"], dtype=np.float64)
    rot, trans = mat[:3, :3], mat[:3, 3]
    optical = rot.T @ (np.asarray(world_xyz, dtype=np.float64) - trans)
    if optical[2] <= 0.0:
        return None
    return (camera["fx"] * optical[0] / optical[2] + camera["cx"],
            camera["fy"] * optical[1] / optical[2] + camera["cy"])


def overlay_for_case(case, top, evaluation, contact_half_xy):
    """Overlay geometry for one evaluated case.

    ``top`` is the DynamicTopResult (its plane basis gives the in-plane
    axes); ``evaluation`` the SuctionPatchEvaluation or None;
    ``contact_half_xy`` the (half_u, half_v) footprint extents in
    metres. Footprint rectangles are axis-aligned in the top-plane
    frame; corners are counter-clockwise in pixels. Rejected candidates
    render their id, reason and colour (their footprint centre is not
    carried on the bounded diagnostic record).
    """
    camera = case.camera
    half_u, half_v = float(contact_half_xy[0]), float(contact_half_xy[1])
    basis = getattr(top, "plane_basis", None)
    if basis is not None and len(basis) == 3:
        u_axis, v_axis = np.asarray(basis[0]), np.asarray(basis[1])
    else:
        u_axis, v_axis = np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])
    out = {"case_id": case.case_id, "candidates": []}

    def corners_for(center_world):
        c = np.asarray(center_world, dtype=np.float64)
        quad = []
        for du, dv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            px = _world_to_pixel(
                c + u_axis * (du * half_u) + v_axis * (dv * half_v),
                camera)
            if px is None:
                return None
            quad.append([round(px[0], 2), round(px[1], 2)])
        return quad

    if evaluation is None:
        return out
    for record in evaluation.accepted:
        corners = corners_for(record.center_world)
        if corners is None:
            continue
        out["candidates"].append({
            "candidate_id": record.candidate_id,
            "rank": record.rank,
            "corner_pixels": corners,
            "colour": ACCEPTED_COLOUR,
            "metrics": {
                "valid_coverage": record.valid_coverage,
                "p95_residual": record.p95_residual,
                "boundary_clearance": record.boundary_clearance,
            },
        })
    for cid, reason, _metrics in evaluation.rejected[:MAX_REJECTED_SHOWN]:
        out["candidates"].append({
            "candidate_id": cid,
            "corner_pixels": None,
            "colour": REJECTION_COLOURS.get(reason, "grey"),
            "reason": reason,
        })
    return out
