"""Error metrics, buckets, bootstrap intervals, and L1 gate mapping."""

from __future__ import division

import math

import numpy as np


def yaw_abs_err(pred, gt):
    delta = math.atan2(math.sin(pred - gt), math.cos(pred - gt))
    err = abs(delta)
    return min(err, math.pi - err)


def component_errors(pred, gt):
    if pred is None:
        return None
    pc = pred["center_xyz"]
    gc = gt["center_xyz"]
    return {
        "width": abs(pred["width"] - gt["width"]),
        "depth": abs(pred["depth"] - gt["depth"]),
        "height": abs(pred["height"] - gt["height"]),
        "xy": math.hypot(pc[0] - gc[0], pc[1] - gc[1]),
        "z": abs(pc[2] - gc[2]),
        "yaw": yaw_abs_err(pred["yaw"], gt["yaw"]),
        "top_z": abs((pc[2] + pred["height"] * 0.5) - (gc[2] + gt["height"] * 0.5)),
        "support_z": abs((pc[2] - pred["height"] * 0.5) - (gc[2] - gt["height"] * 0.5)),
    }


def normalized_composite(errors, gt):
    if errors is None:
        return None
    parts = [
        errors["width"] / max(gt["width"], 1e-3),
        errors["depth"] / max(gt["depth"], 1e-3),
        errors["height"] / max(gt["height"], 1e-3),
        errors["xy"] / 0.05,
        errors["z"] / 0.02,
        errors["yaw"] / 0.20,
    ]
    return float(sum(parts) / len(parts))


def underbound_faces(pred, gt, limit=0.030):
    if pred is None:
        return True
    return any((
        pred["width"] < gt["width"] - limit,
        pred["depth"] < gt["depth"] - limit,
        pred["height"] < gt["height"] - limit,
    ))


def percentile(values, q):
    arr = np.asarray(list(values), dtype=np.float64)
    if arr.size == 0:
        return None
    return float(np.percentile(arr, q))


def bootstrap_ci(values, stat_fn, rng, n_boot=400):
    arr = np.asarray(list(values), dtype=np.float64)
    if arr.size == 0:
        return {"median": None, "p95": None, "ci95": [None, None], "n": 0}
    stats = []
    n = len(arr)
    for _ in range(int(n_boot)):
        idx = rng.randint(0, n, size=n)
        stats.append(stat_fn(arr[idx]))
    stats = np.asarray(stats, dtype=np.float64)
    return {
        "median": float(np.median(arr)) if stat_fn is np.median else float(stat_fn(arr)),
        "p95": float(np.percentile(arr, 95)),
        "ci95": [float(np.percentile(stats, 2.5)), float(np.percentile(stats, 97.5))],
        "n": int(n),
    }


def rate_ci(flags, rng, n_boot=400):
    arr = np.asarray(list(flags), dtype=np.float64)
    if arr.size == 0:
        return {"rate": None, "ci95": [None, None], "n": 0}
    def _mean(x):
        return float(np.mean(x))
    boot = []
    n = len(arr)
    for _ in range(int(n_boot)):
        boot.append(_mean(arr[rng.randint(0, n, size=n)]))
    return {
        "rate": _mean(arr),
        "ci95": [float(np.percentile(boot, 2.5)), float(np.percentile(boot, 97.5))],
        "n": int(n),
    }


def summarize_rows(rows, rng):
    by_bucket = {}
    for row in rows:
        by_bucket.setdefault(row["occlusion_bucket"], []).append(row)
        by_bucket.setdefault("all", []).append(row)
        if row.get("unoccluded_control"):
            by_bucket.setdefault("unoccluded_control", []).append(row)

    out = {}
    for name, group in sorted(by_bucket.items()):
        valid = [1.0 if r["ok"] else 0.0 for r in group]
        errs = [r["errors"] for r in group if r["ok"] and r["errors"]]
        composites = [r["composite"] for r in group if r["ok"] and r["composite"] is not None]
        under = [1.0 if r["underbound"] else 0.0 for r in group]
        contain = [1.0 if r.get("contained") else 0.0 for r in group if r["ok"]]
        cover = [1.0 if r.get("interval_hit") else 0.0 for r in group if r["ok"]]
        lat = [r["latency_ms"] for r in group]
        keys = ("width", "depth", "height", "xy", "z", "yaw", "top_z", "support_z")
        err_summary = {}
        for key in keys:
            vals = [e[key] for e in errs]
            err_summary[key] = {
                "median": percentile(vals, 50) if vals else None,
                "p95": percentile(vals, 95) if vals else None,
                "n": len(vals),
            }
        fail_counts = {}
        for row in group:
            if not row["ok"]:
                fail_counts[row["reason"]] = fail_counts.get(row["reason"], 0) + 1
        out[name] = {
            "n": len(group),
            "valid_rate": rate_ci(valid, rng),
            "errors": err_summary,
            "composite_p95": percentile(composites, 95) if composites else None,
            "composite_median": percentile(composites, 50) if composites else None,
            "underbound_rate": rate_ci(under, rng),
            "containment_rate": rate_ci(contain, rng) if contain else {"rate": None, "ci95": [None, None], "n": 0},
            "interval_coverage": rate_ci(cover, rng) if cover else {"rate": None, "ci95": [None, None], "n": 0},
            "latency_ms": {
                "p50": percentile(lat, 50),
                "p95": percentile(lat, 95),
            },
            "failure_reasons": fail_counts,
        }
    return out


def relative_change(new, old):
    if old is None or new is None or abs(old) < 1e-12:
        return None
    return (new - old) / abs(old)


def map_gates(baseline, candidate):
    """Map each LRF-P1 L1 gate to pass/fail/not-measurable."""
    heavy_b = baseline.get("heavy") or {}
    heavy_c = candidate.get("heavy") or {}
    nom_b = baseline.get("unoccluded_control") or baseline.get("light") or {}
    nom_c = candidate.get("unoccluded_control") or candidate.get("light") or {}
    gates = {}

    def _rate(block, key):
        node = (block or {}).get(key) or {}
        return node.get("rate")

    vb = _rate(heavy_b, "valid_rate")
    vc = _rate(heavy_c, "valid_rate")
    if vb is None or vc is None:
        gates["heavy_valid_rate_plus_10pp"] = "not-measurable"
    else:
        gates["heavy_valid_rate_plus_10pp"] = "pass" if (vc - vb) >= 0.10 else "fail"

    cb = heavy_b.get("composite_p95")
    cc = heavy_c.get("composite_p95")
    rel = relative_change(cc, cb)
    if rel is None:
        gates["heavy_composite_p95_plus_20pct"] = "not-measurable"
    else:
        gates["heavy_composite_p95_plus_20pct"] = "pass" if rel <= -0.20 else "fail"

    critical = True
    measurable = True
    for key in ("width", "depth", "height", "xy", "z", "top_z", "support_z"):
        pb = ((heavy_b.get("errors") or {}).get(key) or {}).get("p95")
        pc = ((heavy_c.get("errors") or {}).get(key) or {}).get("p95")
        ch = relative_change(pc, pb)
        if ch is None:
            measurable = False
        elif ch > 0.10:
            critical = False
    if not measurable:
        gates["no_critical_p95_plus_10pct"] = "not-measurable"
    else:
        gates["no_critical_p95_plus_10pct"] = "pass" if critical else "fail"

    contain = _rate(heavy_c, "containment_rate")
    if contain is None:
        gates["conservative_containment_99"] = "not-measurable"
    else:
        # Plan requires 99% on held-out samples (all test, not only heavy).
        gates["conservative_containment_99"] = "pending-all-test"

    ub = _rate(candidate.get("all"), "underbound_rate")
    if ub is None:
        gates["zero_underbound_30mm"] = "not-measurable"
    else:
        gates["zero_underbound_30mm"] = "pass" if ub == 0.0 else "fail"

    cover = _rate(candidate.get("all"), "interval_coverage")
    if cover is None:
        gates["interval_coverage_85_95"] = "not-measurable"
    else:
        gates["interval_coverage_85_95"] = (
            "pass" if 0.85 <= cover <= 0.95 else "fail")

    nom_ok = True
    nom_meas = True
    for key in ("width", "depth", "height", "xy", "z", "yaw", "top_z"):
        pb = ((nom_b.get("errors") or {}).get(key) or {}).get("p95")
        pc = ((nom_c.get("errors") or {}).get(key) or {}).get("p95")
        ch = relative_change(pc, pb)
        if ch is None:
            nom_meas = False
        elif ch > 0.10:
            nom_ok = False
    if not nom_meas:
        gates["nominal_no_10pct_regression"] = "not-measurable"
    else:
        gates["nominal_no_10pct_regression"] = "pass" if nom_ok else "fail"

    lat = ((candidate.get("all") or {}).get("latency_ms") or {}).get("p95")
    if lat is None:
        gates["p95_latency_200ms_per_frame"] = "not-measurable"
    else:
        gates["p95_latency_200ms_per_frame"] = "pass" if lat <= 200.0 else "fail"

    return gates
