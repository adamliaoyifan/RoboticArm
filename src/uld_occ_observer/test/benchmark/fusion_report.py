"""Fusion-window statistics and semantic comparison.

P95 is the 29th of 30 sorted samples. The median is the mean of the 15th and
16th. Uncollected diagnostics stay the string not_collected. Stage percentiles
are not added together. A failed window is not folded into a campaign mean.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

NOT_COLLECTED = "not_collected"
REFERENCE_GATES = {
    "integrate_30k": {"p95_ms": 500.0, "max_ms": 1000.0},
    "integrate_100k": {"p95_ms": 2000.0, "max_ms": 5000.0},
    "query": {"p95_ms": 100.0, "max_ms": 250.0},
}
SEMANTIC_RECEIPT = (
    "status", "raw_points", "accepted_points", "free_keys", "hit_keys",
    "changed_keys", "occupancy_changed_keys", "rebuilt_blocks",
)
SEMANTIC_IDENTITY = (
    "epoch", "map_revision", "observed_revision", "last_integrated_stamp_ns",
    "geometry_hash", "config_digest", "clock_domain",
)


@dataclass
class WindowResult:
    case: str
    repetition: int
    mode: str
    rows: list = field(default_factory=list)
    status: str = "ok"
    require_content: bool = False
    returncode: int = 0
    sample_path: str = ""
    stdout_path: str = ""
    stderr_path: str = ""
    last_complete_index: int | None = None
    timed_out: bool = False
    binary: str = ""
    argv: list = field(default_factory=list)
    error: str = ""


def official_stats(values, minimum=0.0):
    """Score exactly 30 finite samples. Fewer samples are incomplete and have no P95."""
    if len(values) != 30:
        return {"n": len(values), "status": "incomplete"}
    if any(not isinstance(value, (int, float)) or isinstance(value, bool) or
           not math.isfinite(value) or value < minimum for value in values):
        raise ValueError("invalid metric sample")
    ordered = sorted(float(value) for value in values)
    return {
        "n": 30,
        "status": "complete",
        "median": 0.5 * (ordered[14] + ordered[15]),
        "p95": ordered[28],
        "max": ordered[-1],
    }


def _profile(row):
    return row["receipt"]["profile"]


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("invalid metric sample")
    return float(value)


def stage_measurement(row, stage):
    profile = _profile(row)
    if not profile.get("enabled"):
        return NOT_COLLECTED
    if profile.get("stage_visited", {}).get(stage) is not True:
        return NOT_COLLECTED
    value = _number(profile["stage_ms"][stage])
    if value < 0:
        raise ValueError("invalid metric sample")
    return value


def breakdown_measurement(row, phase):
    fusion = _profile(row).get("fusion_breakdown") or {}
    if not fusion.get("collected") or fusion.get(phase + "_visited") is not True:
        return NOT_COLLECTED
    value = fusion.get(phase + "_ms")
    if value is None:
        return NOT_COLLECTED
    value = _number(value)
    if value < 0:
        raise ValueError("invalid metric sample")
    return value


def fusion_remainder(row):
    """Fusion minus ApplyBatch minus BlockBatch. Not an allocation measurement."""
    fusion = stage_measurement(row, "fusion")
    apply_ms = breakdown_measurement(row, "apply_batch")
    block_ms = breakdown_measurement(row, "block_batch")
    if NOT_COLLECTED in (fusion, apply_ms, block_ms):
        return NOT_COLLECTED
    return fusion - apply_ms - block_ms


def metric_value(row, name):
    receipt = row["receipt"]
    if name == "integrate_external_ms":
        return _number(row["integrate_external_ms"])
    if name == "receipt_total_ms":
        return _number(receipt["total_ms"])
    if name == "preparation_ms":
        return _number(receipt["preparation_ms"])
    if name == "fusion_ms":
        return stage_measurement(row, "fusion")
    if name == "apply_batch_ms":
        return breakdown_measurement(row, "apply_batch")
    if name == "block_batch_ms":
        return breakdown_measurement(row, "block_batch")
    if name == "fusion_remainder_ms":
        return fusion_remainder(row)
    if name == "finalization_ms":
        return stage_measurement(row, "finalization")
    if name == "legacy_receipt_fusion_ms":
        return _number(receipt["fusion_ms"])
    if name == "residual_ms":
        if not _profile(row).get("enabled"):
            return NOT_COLLECTED
        return _number(_profile(row)["residual_ms"])
    if name == "acquire_ms":
        return _number(row["acquire_ms"])
    if name == "roi_ms":
        return _number(row["roi_ms"])
    if name == "surface_first_ms":
        return _number(row["surface_first_call"]["external_ms"])
    if name == "surface_second_ms":
        return _number(row["surface_second_call"]["external_ms"])
    raise KeyError(name)


METRIC_NAMES = (
    "integrate_external_ms", "receipt_total_ms", "preparation_ms", "fusion_ms",
    "apply_batch_ms", "block_batch_ms", "fusion_remainder_ms", "finalization_ms",
    "legacy_receipt_fusion_ms", "residual_ms", "acquire_ms", "roi_ms",
    "surface_first_ms", "surface_second_ms",
)


def _profile_count(row, key):
    profile = _profile(row)
    if not profile.get("enabled"):
        return NOT_COLLECTED
    return profile[key]


def mutation_fields(row):
    fusion = _profile(row).get("fusion_breakdown") or {}
    if fusion.get("mutation_storage") != "collected":
        return {
            "mutation_size": NOT_COLLECTED,
            "mutation_capacity": NOT_COLLECTED,
            "mutation_element_bytes": NOT_COLLECTED,
            "mutation_effective_bytes": NOT_COLLECTED,
            "mutation_reserved_bytes": NOT_COLLECTED,
        }
    size = int(fusion["mutation_size"])
    capacity = int(fusion["mutation_capacity"])
    element = int(fusion["mutation_element_bytes"])
    return {
        "mutation_size": size,
        "mutation_capacity": capacity,
        "mutation_element_bytes": element,
        # Products of the recorded vector, not RSS and not cumulative allocation.
        "mutation_effective_bytes": size * element,
        "mutation_reserved_bytes": capacity * element,
    }


def extract_frame(row):
    receipt = row["receipt"]
    frame = {
        "index": row["index"],
        "warmup": bool(row.get("warmup", False)),
        "mode": row["mode"],
        "status": row["status"],
        "input_sha256": row.get("input_sha256"),
        "free_keys": receipt["free_keys"],
        "hit_keys": receipt["hit_keys"],
        "changed_keys": receipt["changed_keys"],
        "occupancy_changed_keys": receipt["occupancy_changed_keys"],
        "known_cells": row["known_cells"],
        "tree_nodes": row["tree_nodes"],
        "block_count": row["block_count"],
        "rebuilt_blocks": receipt["rebuilt_blocks"],
        "copied_blocks": _profile_count(row, "copied_blocks"),
        "copied_cells": _profile_count(row, "copied_cells"),
        "process_peak_rss_kib": row["process_peak_rss_kib"],
        "probability_tree_sha256": row.get("probability_tree_sha256"),
        "roi_surface_content_sha256": row.get("roi_surface_content_sha256"),
    }
    frame.update(mutation_fields(row))
    for name in METRIC_NAMES:
        frame[name] = metric_value(row, name)
    return frame


def _stats_for(samples, name):
    if any(value == NOT_COLLECTED for value in samples):
        if all(value == NOT_COLLECTED for value in samples):
            return {"status": NOT_COLLECTED, "n": len(samples)}
        raise ValueError(f"mixed collection for {name}")
    minimum = -0.1 if name == "fusion_remainder_ms" else 0.0
    return official_stats(samples, minimum)


def _share_stats(frames, numerator):
    ratios = []
    skipped = 0
    for frame in frames:
        fusion = frame["fusion_ms"]
        value = frame[numerator]
        if fusion == NOT_COLLECTED or value == NOT_COLLECTED:
            return {"status": NOT_COLLECTED, "n": 0}
        if fusion == 0:
            skipped += 1
            continue
        ratios.append(value / fusion)
    if len(ratios) != 30:
        return {"status": "incomplete", "n": len(ratios), "zero_fusion_frames": skipped}
    result = official_stats(ratios, minimum=-1.0)
    result["zero_fusion_frames"] = skipped
    result["note"] = "per-frame share of the same frame Fusion; not a sum of percentiles"
    return result


def summarize_window(window, expect_scored=30):
    if window.status != "ok":
        return {
            "case": window.case,
            "repetition": window.repetition,
            "mode": window.mode,
            "status": window.status,
            "error": window.error,
            "last_complete_index": window.last_complete_index,
            "timed_out": window.timed_out,
        }
    if not window.rows:
        return {"case": window.case, "repetition": window.repetition, "mode": window.mode,
                "status": "missing_summary"}
    frames = [extract_frame(row) for row in window.rows]
    scored = [frame for frame in frames if not frame["warmup"]]
    by_index = {frame["index"]: frame for frame in frames}
    status = "complete"
    if expect_scored is None:
        status = "not_a_performance_sample"
    elif len(scored) != expect_scored:
        status = "incomplete"
    metrics = {}
    if status == "complete":
        for name in METRIC_NAMES:
            metrics[name] = _stats_for([frame[name] for frame in scored], name)
        metrics["apply_share_of_fusion"] = _share_stats(scored, "apply_batch_ms")
        metrics["block_share_of_fusion"] = _share_stats(scored, "block_batch_ms")
        metrics["remainder_share_of_fusion"] = _share_stats(scored, "fusion_remainder_ms")
    return {
        "case": window.case,
        "repetition": window.repetition,
        "mode": window.mode,
        "status": status,
        "frames": len(frames),
        "scored": len(scored),
        "metrics": metrics,
        "early_late": {str(index): by_index.get(index) for index in (3, 4, 31, 32)},
        "frames_extracted": frames,
        "reference_gate": reference_gate(window.case, metrics) if status == "complete" else {
            "status": "not_evaluated", "reference_only": True},
        "dominant_phase": dominant_phase(metrics) if status == "complete" else NOT_COLLECTED,
        "notes": {
            "fusion_remainder_ms": "Fusion minus ApplyBatch minus BlockBatch; unclassified work and boundary overhead",
            "mutation_bytes": "size or capacity times element size; not RSS and not cumulative allocation",
            "process_peak_rss_kib": "process peak, not current RSS",
            "percentiles": "stage percentiles are not additive",
        },
    }


def dominant_phase(metrics):
    names = ("apply_batch_ms", "block_batch_ms", "fusion_remainder_ms")
    medians = []
    for name in names:
        item = metrics.get(name) or {}
        if item.get("status") != "complete":
            return NOT_COLLECTED
        medians.append((item["median"], name))
    medians.sort(reverse=True)
    if medians[0][0] < 1.25 * max(medians[1][0], 0.0) and medians[0][0] > 0:
        return "not_separated"
    if medians[0][0] == 0:
        return "not_separated"
    return medians[0][1]


def reference_gate(case, metrics):
    integrate = metrics.get("integrate_external_ms", {})
    if "c100000" in case:
        limit = REFERENCE_GATES["integrate_100k"]
    elif "c30000" in case:
        limit = REFERENCE_GATES["integrate_30k"]
    else:
        limit = None
    checks = []
    if limit and integrate.get("status") == "complete":
        checks.append({
            "metric": "integrate_external_ms",
            "p95_ms": integrate["p95"],
            "max_ms": integrate["max"],
            "limit_p95_ms": limit["p95_ms"],
            "limit_max_ms": limit["max_ms"],
            "within_reference": integrate["p95"] <= limit["p95_ms"] and integrate["max"] <= limit["max_ms"],
        })
    for name in ("acquire_ms", "roi_ms", "surface_first_ms", "surface_second_ms"):
        item = metrics.get(name, {})
        if item.get("status") != "complete":
            continue
        limit = REFERENCE_GATES["query"]
        checks.append({
            "metric": name,
            "p95_ms": item["p95"],
            "max_ms": item["max"],
            "limit_p95_ms": limit["p95_ms"],
            "limit_max_ms": limit["max_ms"],
            "within_reference": item["p95"] <= limit["p95_ms"] and item["max"] <= limit["max_ms"],
        })
    return {"status": "reference_only", "reference_only": True, "checks": checks}


def _same(left, right, fields, prefix):
    for key in fields:
        if left.get(key) != right.get(key):
            return {"status": "mismatch", "field": f"{prefix}.{key}", "left": left.get(key), "right": right.get(key)}
    return None


def compare_frames(left, right):
    if left.status != "ok" or right.status != "ok":
        return {"status": "not_comparable", "reason": "window_not_ok",
                "left_status": left.status, "right_status": right.status}
    if not left.rows or not right.rows:
        return {"status": "not_comparable", "reason": "missing_summary"}
    if len(left.rows) != len(right.rows):
        return {"status": "not_comparable", "reason": "fewer_frames",
                "left_frames": len(left.rows), "right_frames": len(right.rows)}
    content_compared = False
    for index, (lhs, rhs) in enumerate(zip(left.rows, right.rows)):
        if lhs.get("status") != "OK" or rhs.get("status") != "OK":
            return {"status": "not_comparable", "reason": "frame_status", "frame": index}
        if lhs.get("mode") != left.mode or rhs.get("mode") != right.mode:
            return {"status": "not_comparable", "reason": "mode", "frame": index,
                    "left_mode": lhs.get("mode"), "right_mode": rhs.get("mode")}
        if lhs.get("index") != rhs.get("index") or lhs.get("input_sha256") != rhs.get("input_sha256"):
            return {"status": "not_comparable", "reason": "input_hash", "frame": index}
        mismatch = _same(lhs["receipt"], rhs["receipt"], SEMANTIC_RECEIPT, "receipt")
        if mismatch:
            mismatch["frame"] = index
            return mismatch
        mismatch = _same(lhs["receipt"], rhs["receipt"], SEMANTIC_IDENTITY, "identity")
        if mismatch:
            mismatch["frame"] = index
            return mismatch
        for key in ("tree_nodes", "known_cells", "block_count"):
            if lhs.get(key) != rhs.get(key):
                return {"status": "mismatch", "field": key, "frame": index,
                        "left": lhs.get(key), "right": rhs.get(key)}
        if "probability_tree_sha256" not in lhs or "probability_tree_sha256" not in rhs:
            return {"status": "not_comparable", "reason": "missing_tree_digest", "frame": index}
        if lhs["probability_tree_sha256"] != rhs["probability_tree_sha256"]:
            return {"status": "mismatch", "field": "probability_tree_sha256", "frame": index,
                    "left": lhs["probability_tree_sha256"], "right": rhs["probability_tree_sha256"]}
        left_content = lhs.get("roi_surface_content_sha256")
        right_content = rhs.get("roi_surface_content_sha256")
        if left_content is None or right_content is None:
            if left.require_content or right.require_content:
                return {"status": "not_comparable", "reason": "missing_content_digest", "frame": index}
        else:
            content_compared = True
            if left_content != right_content:
                return {"status": "mismatch", "field": "roi_surface_content_sha256", "frame": index,
                        "left": left_content, "right": right_content,
                        "note": "cell or column identity is reported by FusionProfileChecks"}
    return {"status": "comparable", "frames": len(left.rows), "content_compared": content_compared}


def perturbation(off_summary, stages_summary):
    off_metric = (off_summary.get("metrics") or {}).get("integrate_external_ms") or {}
    stages_metric = (stages_summary.get("metrics") or {}).get("integrate_external_ms") or {}
    if off_metric.get("status") != "complete" or stages_metric.get("status") != "complete":
        return {"status": "not_evaluated"}
    if off_metric["p95"] == 0 or off_metric["max"] == 0:
        return {"status": "not_evaluated", "reason": "off_zero"}
    p95_ratio = stages_metric["p95"] / off_metric["p95"]
    max_ratio = stages_metric["max"] / off_metric["max"]
    low = p95_ratio <= 1.10 and max_ratio <= 1.20
    return {
        "status": "low_perturbation" if low else "perturbed",
        "p95_ratio": p95_ratio,
        "max_ratio": max_ratio,
        "attribution_use": "low_perturbation" if low else "mechanism_only",
    }


def pair_status(comparison, left_summary, right_summary):
    if comparison["status"] == "mismatch":
        return "mismatch"
    if comparison["status"] != "comparable":
        return "not_comparable"
    statuses = {left_summary["status"], right_summary["status"]}
    if "failed" in statuses or "timeout" in statuses:
        return "failed"
    if "incomplete" in statuses or "missing_summary" in statuses:
        return "incomplete"
    if "not_a_performance_sample" in statuses:
        return "not_a_performance_sample"
    if statuses == {"complete"}:
        return "complete"
    return "failed"


def campaign_status(pair_statuses):
    """No pooled mean. Any bad pair decides the campaign."""
    if not pair_statuses:
        return "failed"
    if any(status in {"failed", "timeout", "mismatch", "not_comparable", "validation_failed"}
           for status in pair_statuses):
        return "failed"
    if any(status == "not_evaluated" for status in pair_statuses):
        return "not_evaluated"
    if any(status == "incomplete" for status in pair_statuses):
        return "incomplete"
    if any(status == "not_a_performance_sample" for status in pair_statuses):
        return "not_a_performance_sample"
    if all(status == "complete" for status in pair_statuses):
        return "complete"
    return "failed"


def execution_order(case_index, repetition):
    if (case_index + repetition) % 2 == 0:
        return ("A", "B")
    return ("B", "A")


def budget_allows(already_started, elapsed_s, budget_s):
    if already_started and elapsed_s > budget_s:
        return False
    return True
