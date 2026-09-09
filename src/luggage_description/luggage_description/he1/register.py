"""Enumerate D555-to-bar correspondences and compute the rigid seed."""

from __future__ import division

import itertools
import math

import numpy as np

from luggage_description.he1.geometry import (
    as_unit,
    invert_T,
    make_T,
    rotation_angle_deg,
    rotation_axis,
    rpy_to_R,
    transform_report,
    two_hole_plane_frame,
)

DATASHEET_SPACING_MM = 125.40
MAX_CENTRE_RMS_MM = 0.5
MAX_CENTRE_RESIDUAL_MM = 1.0
MAX_AXIS_DEG = 0.5
GUI_XYZ_M = np.array([0.013000, 0.097000, -0.021000])
GUI_RPY = np.array([0.03769911, 1.36345121, 1.57079633])


def _as_xyz(hole):
    return np.asarray(hole["centre"], dtype=np.float64)


def enumerate_bar_pairs(bar_holes, spacing_mm=DATASHEET_SPACING_MM, max_err_mm=2.0):
    """All unordered pairs among the four bar holes, with spacing verdict."""
    records = []
    for i, j in itertools.combinations(range(len(bar_holes)), 2):
        a = bar_holes[i]
        b = bar_holes[j]
        delta = float(np.linalg.norm(_as_xyz(a) - _as_xyz(b)))
        err = abs(delta - spacing_mm)
        records.append({
            "ids": [a["id"], b["id"]],
            "indices": [i, j],
            "spacing_mm": delta,
            "spacing_error_mm": err,
            "compatible": err <= max_err_mm,
            "reject_reason": (
                None if err <= max_err_mm
                else "spacing %.3f mm vs datasheet %.2f mm" % (delta, spacing_mm)
            ),
        })
    return records


def _candidate_transform(cad_a, cad_b, stl_a, stl_b, cad_normal, stl_normal):
    cad_T = two_hole_plane_frame(cad_a, cad_b, cad_normal)
    stl_T = two_hole_plane_frame(stl_a, stl_b, stl_normal)
    # p_adapter = T_adapter_cad * p_cad
    return stl_T.dot(invert_T(cad_T))


def _axis_angle_deg(a, b):
    value = float(np.clip(abs(as_unit(a).dot(as_unit(b))), 0.0, 1.0))
    return math.degrees(math.acos(value))


def _point_in_aabb(point, mins, maxs):
    point = np.asarray(point, dtype=np.float64)
    return bool(np.all(point >= mins) and np.all(point <= maxs))


def evaluate_candidate(
    transform_mm,
    cad_holes,
    stl_pair,
    cad_axis,
    stl_axes,
    mid360_pocket,
    eef_centroid,
    mount_mesh=None,
    d555_aabb=None,
    seating_normal=None,
    mid360_samples=None,
):
    reasons = []
    cad_pts = np.vstack([h["centre"] for h in cad_holes])
    stl_pts = np.vstack([h["centre"] for h in stl_pair])
    mapped = cad_pts.dot(transform_mm[:3, :3].T) + transform_mm[:3, 3]
    residuals = np.linalg.norm(mapped - stl_pts, axis=1)
    rms = float(np.sqrt(np.mean(residuals ** 2)))
    max_res = float(residuals.max())
    if rms > MAX_CENTRE_RMS_MM:
        reasons.append("hole centre RMS %.3f mm > 0.5" % rms)
    if max_res > MAX_CENTRE_RESIDUAL_MM:
        reasons.append("hole centre max residual %.3f mm > 1.0" % max_res)

    mapped_axis = transform_mm[:3, :3].dot(as_unit(cad_axis))
    axis_err = min(_axis_angle_deg(mapped_axis, ax) for ax in stl_axes)
    if axis_err > MAX_AXIS_DEG:
        reasons.append("hole-axis disagreement %.3f deg > 0.5" % axis_err)

    optics = as_unit(transform_mm[:3, :3].dot(np.array([0.0, 0.0, 1.0])))
    if seating_normal is not None:
        align = float(optics.dot(as_unit(seating_normal)))
        if align < 0.85:
            reasons.append(
                "optical +Z anti-aligned with outward seating normal (dot=%.3f)" % align)

    cam_origin = transform_mm[:3, 3]
    to_eef = as_unit(np.asarray(eef_centroid) - cam_origin)
    if float(optics.dot(to_eef)) > 0.15:
        reasons.append("optical windows face toward EEF/robot body")

    if d555_aabb is None:
        mins = np.array([-83.55, -20.86, -48.0])
        maxs = np.array([83.55, 20.86, 0.0])
    else:
        mins, maxs = d555_aabb
    inv = invert_T(transform_mm)
    pocket = np.asarray(mid360_pocket, dtype=np.float64)
    samples = [pocket]
    if mid360_samples is not None:
        samples.extend(np.asarray(mid360_samples, dtype=np.float64).reshape(-1, 3))
    occupied = 0
    for sample in samples:
        cad_pt = inv[:3, :3].dot(sample) + inv[:3, 3]
        if _point_in_aabb(cad_pt, mins - 1.0, maxs + 1.0):
            occupied += 1
    if occupied:
        reasons.append("housing occupies Mid360 square opening (%d samples)" % occupied)

    # Housing samples that land in the Mid360 tower, without rtree.
    corners = np.array([
        [x, y, z]
        for x in (mins[0], maxs[0])
        for y in (mins[1], maxs[1])
        for z in (mins[2], maxs[2])
    ], dtype=np.float64)
    grid = [corners]
    for x in np.linspace(mins[0], maxs[0], 5):
        for y in np.linspace(mins[1], maxs[1], 3):
            for z in np.linspace(mins[2], maxs[2], 4):
                grid.append(np.array([[x, y, z]]))
    housing = np.vstack(grid)
    mapped_h = housing.dot(transform_mm[:3, :3].T) + transform_mm[:3, 3]
    tower = (
        (mapped_h[:, 0] >= -5.0) & (mapped_h[:, 0] <= 50.0)
        & (mapped_h[:, 1] >= 95.0)
        & (mapped_h[:, 2] >= 8.0) & (mapped_h[:, 2] <= 70.0)
    )
    penetration_mm = 0.0
    if np.any(tower):
        penetration_mm = float(
            np.max(mapped_h[tower, 1] - 95.0)
        )
        reasons.append("housing samples occupy Mid360 tower (penetration proxy %.3f mm)" % penetration_mm)

    return {
        "centre_residuals_mm": residuals.tolist(),
        "centre_rms_mm": rms,
        "centre_max_mm": max_res,
        "axis_err_deg": axis_err,
        "optics_adapter": optics.tolist(),
        "penetration_mm": penetration_mm,
        "reject_reasons": reasons,
        "accepted": not reasons,
    }


def solve_correspondences(cad_features, mount_features, mount_mesh=None):
    cad_holes = cad_features["holes"]
    bar = mount_features["bar_holes"]
    pairs = enumerate_bar_pairs(bar)
    cad_a = cad_holes[0]["centre"]
    cad_b = cad_holes[1]["centre"]
    cad_n = cad_features["seating"]["normal"]
    cad_axis = cad_holes[0]["axis"]
    stl_n = mount_features["bar_seating"]["normal"]
    eef_c = np.mean([h["centre"] for h in mount_features["eef_holes"]], axis=0)
    pocket = mount_features["mid360_pocket"]["centroid"]
    mid_samples = [h["centre"] for h in mount_features["mid360_holes"]]

    evaluations = []
    for rec in pairs:
        if not rec["compatible"]:
            evaluations.append({
                "pair": rec,
                "flips": [],
                "accepted": False,
                "reject_reasons": [rec["reject_reason"]],
            })
            continue
        i, j = rec["indices"]
        stl_axes = [bar[i]["axis"], bar[j]["axis"]]
        flips = []
        for swap in (False, True):
            sa, sb = (bar[j], bar[i]) if swap else (bar[i], bar[j])
            for nsign in (1.0, -1.0):
                T = _candidate_transform(
                    cad_a, cad_b, sa["centre"], sb["centre"],
                    cad_n, nsign * stl_n,
                )
                metrics = evaluate_candidate(
                    T, cad_holes, [sa, sb], cad_axis, stl_axes,
                    pocket, eef_c, mount_mesh=mount_mesh,
                    seating_normal=stl_n,
                    mid360_samples=mid_samples,
                )
                flips.append({
                    "swap": swap,
                    "normal_sign": nsign,
                    "stl_ids": [sa["id"], sb["id"]],
                    "transform_mm": T,
                    "metrics": metrics,
                })
        accepted_flips = [f for f in flips if f["metrics"]["accepted"]]
        evaluations.append({
            "pair": rec,
            "flips": [
                {
                    "swap": f["swap"],
                    "normal_sign": f["normal_sign"],
                    "stl_ids": f["stl_ids"],
                    "metrics": f["metrics"],
                    "accepted": f["metrics"]["accepted"],
                }
                for f in flips
            ],
            "accepted": bool(accepted_flips),
            "reject_reasons": (
                [] if accepted_flips
                else sorted({r for f in flips for r in f["metrics"]["reject_reasons"]})
            ),
            "_accepted_full": accepted_flips,
        })

    winners = []
    for item in evaluations:
        winners.extend(item.get("_accepted_full") or [])
    for item in evaluations:
        item.pop("_accepted_full", None)

    yaw_symmetry = False
    canonical = winners
    if len(winners) == 2:
        signs = {w["normal_sign"] for w in winners}
        swaps = {w["swap"] for w in winners}
        origins = [w["transform_mm"][:3, 3] for w in winners]
        origin_delta = float(np.linalg.norm(origins[0] - origins[1]))
        rel = invert_T(winners[0]["transform_mm"]).dot(winners[1]["transform_mm"])
        ang = rotation_angle_deg(rel[:3, :3])
        if signs == {1.0} and swaps == {False, True} and origin_delta < 0.5 and abs(ang - 180.0) < 1.0:
            yaw_symmetry = True
            canonical = [w for w in winners if not w["swap"]]

    result = {
        "pair_enumeration": evaluations,
        "winner_count": len(winners),
        "yaw_symmetry_180_deg": yaw_symmetry,
    }
    if len(canonical) != 1:
        result["unique"] = False
        result["transform_mm"] = None
        result["ambiguity"] = [
            {"stl_ids": w["stl_ids"], "swap": w["swap"], "normal_sign": w["normal_sign"]}
            for w in winners
        ]
        return result

    winner = canonical[0]
    T_mm = winner["transform_mm"]
    T_m = np.array(T_mm, dtype=np.float64)
    T_m[:3, 3] = T_m[:3, 3] / 1000.0
    result.update({
        "unique": True,
        "stl_ids": winner["stl_ids"],
        "swap": winner["swap"],
        "normal_sign": winner["normal_sign"],
        "metrics": winner["metrics"],
        "transform_mm": T_mm,
        "transform_m": T_m,
        "report_m": transform_report(
            T_m, "eef_mount_adapter", "D555-mechanical", units="m"),
        "canonical_yaw_note": (
            "2x M4 pattern has a 180 deg yaw symmetry about the seating normal; "
            "canonical seed maps CAD_M4_NEG to STL_BAR_H0 (adapter +X along the bar)"
            if yaw_symmetry else None
        ),
    })
    return result


def gui_delta(T_adapter_child_m):
    """Compare a metres transform against the GUI camera_link value.

    GUI is comparison-only. If child is not d555_link, the delta is not a
    calibration residual.
    """
    T_gui = make_T(rpy_to_R(GUI_RPY), GUI_XYZ_M)
    delta = invert_T(T_adapter_child_m).dot(T_gui)
    rot = delta[:3, :3]
    return {
        "T_seed_inv_T_gui": delta.tolist(),
        "translation_norm_m": float(np.linalg.norm(delta[:3, 3])),
        "translation_norm_mm": float(np.linalg.norm(delta[:3, 3]) * 1000.0),
        "rotation_angle_deg": rotation_angle_deg(rot),
        "rotation_axis": rotation_axis(rot).tolist(),
        "gui_xyz_m": GUI_XYZ_M.tolist(),
        "gui_rpy_urdf": GUI_RPY.tolist(),
    }


def roundtrip_points(matrix, points):
    matrix = np.asarray(matrix, dtype=np.float64)
    points = np.asarray(points, dtype=np.float64)
    mapped = points.dot(matrix[:3, :3].T) + matrix[:3, 3]
    back = mapped.dot(matrix[:3, :3]) + invert_T(matrix)[:3, 3]
    # p' = R p + t; p = R^T (p' - t)
    recovered = (mapped - matrix[:3, 3]).dot(matrix[:3, :3])
    err = np.linalg.norm(recovered - points, axis=1)
    return float(err.max())


def monte_carlo_uncertainty(cad_a, cad_b, stl_a, stl_b, cad_n, stl_n, sigma_mm=0.25, n=400, seed=7):
    rng = np.random.default_rng(seed)
    base = _candidate_transform(cad_a, cad_b, stl_a, stl_b, cad_n, stl_n)
    trans = []
    rots = []
    for _ in range(n):
        ja = stl_a + rng.normal(0.0, sigma_mm, 3)
        jb = stl_b + rng.normal(0.0, sigma_mm, 3)
        T = _candidate_transform(cad_a, cad_b, ja, jb, cad_n, stl_n)
        d = invert_T(base).dot(T)
        trans.append(np.linalg.norm(d[:3, 3]))
        rots.append(rotation_angle_deg(d[:3, :3]))
    trans = np.asarray(trans)
    rots = np.asarray(rots)
    return {
        "sigma_mm": sigma_mm,
        "n": n,
        "translation_p95_mm": float(np.percentile(trans, 95)),
        "rotation_p95_deg": float(np.percentile(rots, 95)),
    }
