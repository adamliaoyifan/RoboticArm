"""Frozen case identities for the 2 cm repair matrix.

Point-cloud bytes are not emitted here. The seed schedule matches
occ_load::frame_seed in test/load_model.cpp, generator o6-load-v1.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import o6_matrix
import repair_s0_contract as contract

DEFAULT_FIXTURES = Path(
    "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures")
TARGET_RESOLUTIONS = (0.02, 0.05)
O5_BOXES = (0, 20)
O5_CLOUDS = (30000, 100000)
O5_OUTSIDE = 0.5
O5_CROSS = 0.0


def frame_seed(group_id, base_seed, round_index):
    """First four digest bytes, little-endian, of version|group|base|round."""
    text = "%s|%s|%s|%s" % (contract.GENERATOR_VERSION, group_id, base_seed, round_index)
    digest = hashlib.sha256(text.encode()).hexdigest()
    seed = 0
    for index in range(4):
        seed |= int(digest[index * 2:index * 2 + 2], 16) << (8 * index)
    return seed


def frame_schedule(group_id, warmup=contract.WARMUP_SAMPLES, scored=contract.SCORED_SAMPLES):
    frames = []
    for index in range(warmup + scored):
        is_warmup = index < warmup
        scored_index = index - warmup
        base = index if is_warmup else scored_index % 10
        round_index = -1 if is_warmup else scored_index // 10
        frames.append({
            "index": index,
            "warmup": is_warmup,
            "base_seed": base,
            "round": round_index,
            "seed": frame_seed(group_id, base, round_index),
            "input_file_sha256": "not_emitted",
        })
    return frames


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_scenes(fixtures):
    """Read the historical scene manifest and re-hash every referenced file."""
    fixtures = Path(fixtures)
    document = json.loads((fixtures / "TEST_MANIFEST.json").read_text())
    if document.get("generator_version") != contract.GENERATOR_VERSION:
        raise ValueError("fixture generator is not o6-load-v1")
    scenes = []
    for scene in document["scenes"]:
        geometry = Path(scene["geometry_file"])
        config = Path(scene["config_file"])
        geometry_sha = _sha256(geometry)
        config_sha = _sha256(config)
        if geometry_sha != scene["geometry_sha256"] or config_sha != scene["config_sha256"]:
            raise ValueError("fixture hash changed for %s" % scene["id"])
        scenes.append({
            "id": scene["id"],
            "resolution": scene["resolution"],
            "boxes": scene["boxes"],
            "hull": scene["hull"],
            "geometry_file": str(geometry),
            "config_file": str(config),
            "geometry_sha256": geometry_sha,
            "config_sha256": config_sha,
        })
    return scenes


def o6_groups(scenes):
    """96 groups. The 1 cm rows of the historical 144 stay out."""
    selected = [group for group in o6_matrix.official_matrix(scenes) if group["resolution"] in TARGET_RESOLUTIONS]
    if len(selected) != contract.O6_GROUPS:
        raise ValueError("O6 target matrix is not 96 groups")
    if any(group["id"].startswith("r01_") for group in selected):
        raise ValueError("1 cm group entered the 2/5 cm matrix")
    by_scene = {(scene["resolution"], scene["boxes"], scene["hull"]): scene for scene in scenes}
    frozen = []
    for group in selected:
        scene = by_scene[(group["resolution"], group["boxes"], group["hull"])]
        quotas = o6_matrix.quotas_for(group["cloud"], group["outside"], group["cross"])
        frozen.append({
            "id": group["id"],
            "resolution": group["resolution"],
            "boxes": group["boxes"],
            "hull": group["hull"],
            "cloud": group["cloud"],
            "outside": group["outside"],
            "cross": group["cross"],
            "geometry_file": scene["geometry_file"],
            "config_file": scene["config_file"],
            "geometry_sha256": scene["geometry_sha256"],
            "config_sha256": scene["config_sha256"],
            "quotas": quotas,
            "warmup": contract.WARMUP_SAMPLES,
            "scored": contract.SCORED_SAMPLES,
            "cold_starts": contract.SCORED_SAMPLES,
            "seed_schedule": frame_schedule(group["id"]),
        })
    return frozen


def o5_cases(groups):
    """Representative 2 cm chamfer loads. outside 0.5 and cross 0 match the old selector."""
    chosen = [group for group in groups
              if group["resolution"] == 0.02 and group["hull"] == "chamfer"
              and group["boxes"] in O5_BOXES and group["cloud"] in O5_CLOUDS
              and group["outside"] == O5_OUTSIDE and group["cross"] == O5_CROSS]
    if len(chosen) != contract.O5_GROUPS:
        raise ValueError("O5 did not select 4 groups")
    cases = []
    for group in chosen:
        scored_frames = [frame for frame in group["seed_schedule"] if not frame["warmup"]]
        cases.append({
            "id": group["id"],
            "group": {key: group[key] for key in (
                "id", "resolution", "boxes", "hull", "cloud", "outside", "cross",
                "geometry_file", "config_file", "geometry_sha256", "config_sha256", "quotas")},
            "warmup": contract.WARMUP_SAMPLES,
            "scored": contract.SCORED_SAMPLES,
            "first_build_samples": contract.SCORED_SAMPLES,
            "first_build_seeds": [frame["seed"] for frame in scored_frames],
            "performance_roi_cells": {"begin": [0, 0, 0], "end": [16, 16, 16], "count": 4096},
            "physical_roi_m": {"begin": [0.0, 0.0, 0.0], "end": [0.16, 0.16, 0.16], "cells_at_2cm": 512},
            "physical_roi_in_4096_stats": False,
        })
    return cases


def o7_windows(groups):
    def one(boxes):
        matches = [group for group in groups
                   if group["resolution"] == 0.02 and group["hull"] == "chamfer"
                   and group["boxes"] == boxes and group["cloud"] == 30000
                   and group["outside"] == 0.5 and group["cross"] == 0.0]
        if len(matches) != 1:
            raise ValueError("O7 scene missing for %s boxes" % boxes)
        return matches[0]["id"]

    windows = []
    for boxes in (0, 5, 20):
        for repeat in range(3):
            windows.append({
                "id": "normal_b%02d_r%d" % (boxes, repeat),
                "kind": "normal",
                "group_id": one(boxes),
                "inputs": contract.NORMAL_INPUTS,
                "input_hz": 2,
                "queries": contract.NORMAL_QUERIES,
                "query_hz": 1,
                "duration_s": contract.NORMAL_DURATION_S,
            })
    windows.append({
        "id": "overload",
        "kind": "overload",
        "group_id": one(20),
        "inputs": contract.OVERLOAD_INPUTS,
        "input_hz": 10,
        "duration_s": 60,
        "drain_s": contract.DRAIN_S,
        "query_hz": 1,
    })
    windows.append({
        "id": "recovery",
        "kind": "recovery",
        "group_id": one(20),
        "inputs": contract.RECOVERY_INPUTS,
        "input_hz": 2,
        "duration_s": 30,
        "queries": contract.RECOVERY_QUERIES,
        "first_query": "wait_snapshot",
        "later_queries": "get_snapshot",
        "wait_timeout_ms": contract.WAIT_TIMEOUT_MS,
        "wait_max_age_ms": contract.WAIT_MAX_AGE_MS,
        "legacy_immediate_query": "recorded_separately",
    })
    return windows


def o8_plan(groups):
    group_id = o7_windows(groups)[0]["group_id"]
    seed = frame_seed("o8_fixed_%s" % group_id, 0, 0)
    return {
        "runs": ["o8_r0", "o8_r1", "o8_r2"],
        "duration_s": 180,
        "warmup_s": 60,
        "ray_group": "o8_fixed_%s" % group_id,
        "ray_seed": seed,
        "stamp_step_ns": 1000000000,
        "saturation_frames": 30,
        "no_input_s": contract.O8_HOLD_S,
        "no_input_repeats": contract.O8_HOLDS,
        "occlusion_s": contract.O8_HOLD_S,
        "occlusion_repeats": contract.O8_HOLDS,
        "rss_growth_limit": contract.O8_RSS_GROWTH,
    }


def o9_plan():
    pairs = []
    for index in range(contract.O9_PAIRS):
        pairs.append({"index": index, "order": "AB" if index % 2 == 0 else "BA"})
    return {
        "operations": ["roi", "integrate_30k", "surface_first"],
        "pairs": pairs,
        "ratio_limit": contract.O9_RATIO,
        "trace_changes_product": False,
    }


def environment():
    return {
        "governor": "powersave",
        "cpu_pinning": "none",
        "logical_cpus_are_utilization": False,
        "client_executor_threads": contract.CLIENT_EXECUTOR_THREADS,
        "integrate_threads": contract.INTEGRATE_THREADS,
        "octomap": "1.9.8",
        "discretize": False,
        "lock_path": "/tmp/elfin-occ-backend-perf.lock",
    }


def build_manifest(fixtures):
    scenes = load_scenes(fixtures)
    groups = o6_groups(scenes)
    return {
        "schema": contract.SCHEMA,
        "plan_revision": contract.PLAN_REVISION,
        "product_base": contract.PRODUCT_BASE,
        "tool_base": contract.TOOL_BASE,
        "generator_version": contract.GENERATOR_VERSION,
        "input_bytes": "not_emitted",
        "scoring": "not_run",
        "environment": environment(),
        "o6_groups": groups,
        "o5_cases": o5_cases(groups),
        "o7_windows": o7_windows(groups),
        "o8": o8_plan(groups),
        "o9": o9_plan(),
        "excluded_historical_failure": {
            "id": contract.HISTORICAL_O6_FAIL_ID,
            "p95_ms": contract.HISTORICAL_O6_FAIL_P95_MS,
            "in_new_matrix": False,
        },
    }
