"""Judge for the F-gate artifacts (and the shared validation helpers).

Every rule here fails closed: unknown classes, missing responses, NaN/bool/
negative values, self-inconsistent counters, and any disagreement with the
independent oracle are findings. The judge never rounds, never filters rows,
and never treats a missing artifact as a pass.
"""
from __future__ import annotations

import math

import freespace_contract as contract
import freespace_oracle

COUNTER_KEYS = ("grid_total", "feasible_total", "rejected_out_of_domain",
                "rejected_collision", "rejected_clearance", "rejected_unknown",
                "rejected_support")


def _number_problem(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return "bad_type"
    if not math.isfinite(value):
        return "nonfinite_value"
    if value < 0:
        return "negative_value"
    return None


def _model_for(case, rays=None):
    manifest = dict(case)
    if rays is not None:
        manifest["rays"] = rays
    return freespace_oracle.model_from_manifest(manifest)


def _expected_status(model, box, yaw):
    known = bool(model.free.any() or model.occ.any())
    if not known:
        return "NO_DATA"
    candidates, _ = model.enumerate(box, (yaw,))
    return "OK" if candidates else "NO_KNOWN_FREE"


def _artifact_candidates(response):
    out = []
    for entry in response.get("candidates") or []:
        cell = entry["cell"]
        out.append((int(entry["yaw_deg"]), int(cell[0]), int(cell[1]), int(cell[2]),
                    int(entry["support_kind"])))
    return out


def _compare_freespace(reasons, response, model, case, page_note):
    expected_status = _expected_status(model, case["box"], case["yaw"])
    semantic = response.get("semantic_status")
    if semantic != expected_status:
        reasons.append("%ssemantic_status %r != oracle %r" % (page_note, semantic,
                                                              expected_status))
    for key in COUNTER_KEYS:
        value = response.get(key)
        problem = _number_problem(value)
        if problem:
            reasons.append("%s%s %s" % (page_note, key, problem))
            continue
        if not isinstance(value, int):
            reasons.append("%s%s not_integer" % (page_note, key))
    identity = response.get("identity") or {}
    if identity.get("geometry_hash") != _geometry_hash(case):
        reasons.append("%sgeometry_hash_mismatch" % page_note)
    if identity.get("epoch") != _expected_epoch(case):
        reasons.append("%secho_epoch_mismatch" % page_note)
    feasible = response.get("feasible_total")
    if isinstance(feasible, int) and feasible > 0 and response.get("done") is not True:
        reasons.append("%sfull_page_not_done" % page_note)
    if isinstance(feasible, int) and response.get("next_cursor") != feasible:
        reasons.append("%snext_cursor_mismatch" % page_note)
    total = 0
    for key in COUNTER_KEYS[2:]:
        value = response.get(key)
        total += value if isinstance(value, int) else 0
    if isinstance(feasible, int):
        if response.get("grid_total") != feasible + total:
            reasons.append("%scounter_identity" % page_note)
    expected_candidates, expected_counters = model.enumerate(case["box"], (case["yaw"],))
    got = _artifact_candidates(response)
    if got != expected_candidates:
        missing = [c for c in expected_candidates if c not in got][:3]
        extra = [c for c in got if c not in expected_candidates][:3]
        reasons.append("%scandidate_set_mismatch n=%d exp=%d missing=%s extra=%s"
                       % (page_note, len(got), len(expected_candidates), missing, extra))
    for key, value in expected_counters.items():
        got_value = response.get(key)
        if got_value != value:
            reasons.append("%s%s %r != oracle %d" % (page_note, key, got_value, value))
    box_cells = [int(math.ceil(dim / model.res - 1e-9)) for dim in case["box"]]
    fx, fy = (box_cells[0], box_cells[1]) if case["yaw"] == 0 else (box_cells[1], box_cells[0])
    fz = box_cells[2]
    for entry in response.get("candidates") or []:
        begin = entry["verify_begin"]
        end = entry["verify_end"]
        if [end[0] - begin[0], end[1] - begin[1], end[2] - begin[2]] != [fx, fy, fz]:
            reasons.append("%sverify_range_not_box" % page_note)
            break
        if entry["support_kind"] not in (1, 2):
            reasons.append("%ssupport_kind_invalid" % page_note)
            break
        ratio = entry["support_ratio"]
        if _number_problem(ratio) or ratio < contract.MIN_SUPPORT_RATIO - 1e-9 \
                or ratio > 1.0 + 1e-9:
            reasons.append("%ssupport_ratio_out_of_range" % page_note)
            break


def _geometry_hash(case):
    return case.get("geometry_hash")


def _expected_epoch(case):
    return 2 if case["class"] == "reset" else 1


def judge_f_case(case, responses, descriptor, resolution):
    """Judge one F case: artifact vs the independent oracle, fail-closed."""
    reasons = []
    if not isinstance(responses, list):
        return {"reasons": ["responses_missing"], "status": "fail"}
    case = dict(case)
    case.setdefault("geometry_hash", descriptor.get("geometry_hash"))
    if case.get("cli_returncode") != 0:
        reasons.append("cli_returncode_%s" % case.get("cli_returncode"))
    acquires = [r for r in responses if r.get("op") == "acquire"]
    frees = [r for r in responses if r.get("op") == "freespace"]
    if case["class"] == "empty_tree":
        if len(acquires) != 1 or acquires[0].get("status") != "NO_DATA":
            reasons.append("empty_tree_acquire_not_no_data")
        if frees:
            reasons.append("empty_tree_ran_freespace")
        return {"reasons": reasons, "status": "pass" if not reasons else "fail"}
    if not frees:
        reasons.append("no_freespace_response")
        return {"reasons": reasons, "status": "fail"}
    if case["class"] == "two_version":
        if len(frees) != 2:
            reasons.append("two_version_needs_two_views")
        else:
            split = case.get("meta", {}).get("old_ray_count")
            if not isinstance(split, int):
                reasons.append("old_ray_count_missing")
            else:
                old_model = _model_for(case, case["rays"][:split])
                new_model = _model_for(case)
                _compare_freespace(reasons, frees[0], old_model, case, "old:")
                _compare_freespace(reasons, frees[1], new_model, case, "new:")
                old_rev = (frees[0].get("identity") or {}).get("map_revision")
                new_rev = (frees[1].get("identity") or {}).get("map_revision")
                if not isinstance(old_rev, int) or not isinstance(new_rev, int) \
                        or new_rev <= old_rev:
                    reasons.append("revision_did_not_advance")
        return {"reasons": reasons, "status": "pass" if not reasons else "fail"}
    if case["class"] == "reset":
        statuses = [r.get("status") for r in acquires]
        if statuses[:2] != ["NO_DATA", "EPOCH_MISMATCH"]:
            reasons.append("reset_acquire_sequence_%r" % (statuses[:2],))
    model = _model_for(case)
    # The main page is the first freespace; later ones are paging probes.
    _compare_freespace(reasons, frees[0], model, case, "")
    for probe in frees[1:]:
        expected, _ = model.enumerate(case["box"], (case["yaw"],))
        got = _artifact_candidates(probe)
        cursor = probe.get("next_cursor")
        # Probe convention: cursor=3, limit=5. Past the end the page is empty
        # and the cursor stays put.
        expected_next = min(8, len(expected)) if len(expected) > 3 else 3
        if got != expected[3:8]:
            reasons.append("paging_probe_slice_mismatch")
        if cursor != expected_next:
            reasons.append("paging_probe_cursor_mismatch")
    return {"reasons": reasons, "status": "pass" if not reasons else "fail"}


def judge_f_summary(summary):
    """Whole-matrix verdict: every case judged, counts consistent."""
    reasons = []
    if not isinstance(summary, dict):
        return ["summary_missing"]
    total = summary.get("total")
    passed = summary.get("passed")
    if not isinstance(total, int) or not isinstance(passed, int):
        reasons.append("summary_counts_missing")
    elif total != contract.F_CASES:
        reasons.append("matrix_size_%s_not_%d" % (total, contract.F_CASES))
    elif passed != total:
        reasons.append("matrix_failures_%d" % (total - passed))
    return reasons
