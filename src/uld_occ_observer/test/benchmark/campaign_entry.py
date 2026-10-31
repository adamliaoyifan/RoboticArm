"""Runnable O5–O8 entry. Preparing inputs does not score them.

Stage resolution is data the caller executes. A failed O5 result stops O6 and
O7. Missing samples and an unevaluated resource total do not pass.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import repair_s0_contract as contract
import repair_s0_manifest as manifest


def load_plan(fixtures):
    return manifest.build_manifest(fixtures)


def resolve_stage(plan, name):
    """Return the execution description for one stage. This does not launch it."""
    if name == "o5":
        cases = plan["o5_cases"]
        return {
            "stage": "o5",
            "runner": "profile_cli_groups",
            "groups": len(cases),
            "resolution_m": sorted({case["group"]["resolution"] for case in cases}),
            "hulls": sorted({case["group"]["hull"] for case in cases}),
            "boxes": sorted({case["group"]["boxes"] for case in cases}),
            "clouds": sorted({case["group"]["cloud"] for case in cases}),
            "outside": sorted({case["group"]["outside"] for case in cases}),
            "cross": sorted({case["group"]["cross"] for case in cases}),
            "warmup": contract.WARMUP_SAMPLES,
            "scored": contract.SCORED_SAMPLES,
            "roi_cells": 4096,
            "budget_s": 8 * 3600,
            "depends_on": [],
            "output": "o5/groups",
        }
    if name == "o6":
        groups = plan["o6_groups"]
        return {
            "stage": "o6",
            "runner": "profile_cli_groups",
            "groups": len(groups),
            "resolutions_m": sorted({group["resolution"] for group in groups}),
            "cold_starts_per_group": contract.SCORED_SAMPLES,
            "warmup": contract.WARMUP_SAMPLES,
            "scored": contract.SCORED_SAMPLES,
            "budget_s": 8 * 3600,
            "depends_on": ["o5"],
            "blocks_if_o5_not_pass": True,
            "output": "o6/groups",
        }
    if name == "o7":
        windows = plan["o7_windows"]
        normal = [window for window in windows if window["kind"] == "normal"]
        overload = next(window for window in windows if window["kind"] == "overload")
        recovery = next(window for window in windows if window["kind"] == "recovery")
        return {
            "stage": "o7",
            "runner": "ros_windows",
            "normal_windows": len(normal),
            "normal_inputs": contract.NORMAL_INPUTS,
            "normal_queries": contract.NORMAL_QUERIES,
            "normal_duration_s": contract.NORMAL_DURATION_S,
            "normal_input_hz": 2,
            "overload_inputs": overload["inputs"],
            "overload_input_hz": overload["input_hz"],
            "overload_duration_s": overload["duration_s"],
            "drain_s": overload["drain_s"],
            "recovery_inputs": recovery["inputs"],
            "recovery_queries": recovery["queries"],
            "recovery_duration_s": recovery["duration_s"],
            "first_query": recovery["first_query"],
            "later_queries": recovery["later_queries"],
            "budget_s": 8 * 3600,
            "depends_on": ["o5"],
            "blocks_if_o5_not_pass": True,
            "output": "o7/windows",
        }
    if name == "o8":
        item = plan["o8"]
        return {
            "stage": "o8",
            "runner": "ros_stability",
            "runs": list(item["runs"]),
            "duration_s": item["duration_s"],
            "warmup_s": item["warmup_s"],
            "saturation_frames": item["saturation_frames"],
            "no_input_s": item["no_input_s"],
            "no_input_repeats": item["no_input_repeats"],
            "occlusion_s": item["occlusion_s"],
            "occlusion_repeats": item["occlusion_repeats"],
            "budget_s": 8 * 3600,
            "depends_on": ["o5"],
            "blocks_if_o5_not_pass": False,
            "output": "o8/runs",
        }
    raise ValueError("unknown stage %s" % name)


def continue_after(stage_results):
    """O5 must pass before O6 or O7 starts. O8 is not used to hide an O5 failure."""
    o5 = stage_results.get("o5")
    if o5 != "pass":
        return {"continue_o6": False, "continue_o7": False, "stopped_at": "o5"}
    return {"continue_o6": True, "continue_o7": True, "stopped_at": None}


def judge_recorded_window(document):
    """Judge one saved window. Missing numbers stay missing."""
    reasons = []
    samples = document.get("integrate_ms") if isinstance(document, dict) else None
    if not isinstance(samples, list) or len(samples) != contract.SCORED_SAMPLES:
        reasons.append("missing_sample")
    elif any(not isinstance(value, (int, float)) or value < 0 for value in samples):
        reasons.append("bad_sample")
    if not isinstance(document, dict) or document.get("resource_certification") != "pass":
        reasons.append("resource")
    return {"pass": not reasons, "reasons": reasons}


def prepare_inputs(fixtures, out_dir):
    """Write lossless generation manifests and their hashes. No scored process starts."""
    plan = load_plan(fixtures)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    written = []

    def dump(relative, payload):
        path = out / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        written.append({"path": relative, "sha256": digest})

    for group in plan["o6_groups"]:
        dump("o6/%s.json" % group["id"], {
            "generator_version": contract.GENERATOR_VERSION,
            "group_id": group["id"],
            "resolution": group["resolution"],
            "boxes": group["boxes"],
            "hull": group["hull"],
            "cloud": group["cloud"],
            "outside": group["outside"],
            "cross": group["cross"],
            "geometry_sha256": group["geometry_sha256"],
            "config_sha256": group["config_sha256"],
            "quotas": group["quotas"],
            "frames": group["seed_schedule"],
            "point_bytes": "not_emitted",
        })
    for case in plan["o5_cases"]:
        dump("o5/%s.json" % case["id"], {
            "generator_version": contract.GENERATOR_VERSION,
            "group": case["group"],
            "warmup": case["warmup"],
            "scored": case["scored"],
            "first_build_seeds": case["first_build_seeds"],
            "performance_roi_cells": case["performance_roi_cells"],
            "point_bytes": "not_emitted",
        })
    for window in plan["o7_windows"]:
        dump("o7/%s.json" % window["id"], dict(window, point_bytes="not_emitted",
                                                generator_version=contract.GENERATOR_VERSION))
    dump("o8/plan.json", dict(plan["o8"], point_bytes="not_emitted",
                              generator_version=contract.GENERATOR_VERSION))
    index = {"files": written, "scoring": "not_run"}
    index_path = out / "INDEX.json"
    index_path.write_text(json.dumps(index, indent=2, sort_keys=True) + "\n")
    index["sha256"] = hashlib.sha256(index_path.read_bytes()).hexdigest()
    return index
