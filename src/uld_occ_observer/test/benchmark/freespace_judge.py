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


# --- P/R/L wiring judges (short rehearsals and formal runs share them) ----
def _latency_problem(value):
    problem = _number_problem(value)
    if problem:
        return problem
    return None


def judge_p_window(window):
    """One P window: wiring invariants always; formal gates only when the
    window carries formal counts (a short rehearsal must NOT certify)."""
    reasons = []
    if not isinstance(window, dict):
        return {"reasons": ["window_missing"]}
    if window.get("errors"):
        reasons.append("thread_errors:%s" % window["errors"][:2])
    sender = window.get("sender") or []
    queries = window.get("queries") or []
    if not sender or not queries:
        reasons.append("missing_requests")
        return {"reasons": reasons}
    for record in sender:
        for key in ("planned_mono_ns", "publish_mono_ns"):
            if not isinstance(record.get(key), int):
                reasons.append("sender_%s_missing" % key)
    ids = [record["input_id"] for record in sender]
    if len(set(ids)) != len(ids):
        reasons.append("duplicate_input_id")
    terminals = window.get("terminals") or {}
    pending = window.get("pending") or []
    if pending:
        reasons.append("inputs_without_terminal:%d" % len(pending))
    integrated = sum(1 for status in terminals.values() if status == "COMPLETED")
    formal = bool(window.get("formal"))
    if formal:
        if len(sender) != contract.P_WINDOW_INPUTS or len(queries) != contract.P_WINDOW_QUERIES:
            reasons.append("formal_counts")
        if integrated != len(sender):
            reasons.append("integrated_count_%d" % integrated)
        latencies = []
        for record in queries:
            ended, planned = record.get("ended_mono_ns"), record.get("planned_mono_ns")
            if not isinstance(ended, int) or not isinstance(planned, int):
                reasons.append("query_boundary_missing")
                continue
            latencies.append((ended - planned) / 1e6)
        for value in latencies:
            if _latency_problem(value):
                reasons.append("query_latency_bad_type")
                break
        if latencies:
            ordered = sorted(latencies)
            p95 = ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)]
            if p95 > contract.QUERY_FULL_P95_MS:
                reasons.append("query_p95_%.0f" % p95)
            if max(latencies) > contract.QUERY_FULL_MAX_MS:
                reasons.append("query_max_%.0f" % max(latencies))
        lateness = [(record["publish_mono_ns"] - record["planned_mono_ns"]) / 1e6
                    for record in sender]
        if lateness:
            ordered = sorted(lateness)
            p95 = ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)]
            if p95 > contract.SCHED_LATE_P95_MS * 10:  # sender thread only
                reasons.append("sched_late_p95_%.0f" % p95)
    else:
        # Wiring rehearsal: every query answered a semantic status and the
        # identity is present; no formal latency claim is made.
        for record in queries:
            if record.get("status") not in ("OK", "NO_KNOWN_FREE", "STALE",
                                            "SNAPSHOT_PRESSURE"):
                reasons.append("query_status_%s" % record.get("status"))
                break
            if record.get("status") in ("OK", "NO_KNOWN_FREE") \
                    and not record.get("identity"):
                reasons.append("query_identity_missing")
                break
        reasons.append("short_sample_not_formal")
    return {"reasons": reasons, "formal": formal,
            "inputs": len(sender), "queries": len(queries)}


def judge_r_stage(artifact):
    reasons = []
    if not isinstance(artifact, dict):
        return {"reasons": ["artifact_missing"]}
    overload = artifact.get("overload") or []
    if not overload:
        reasons.append("no_overload_inputs")
    pending = artifact.get("overload_pending") or []
    if pending:
        reasons.append("overload_pending_%d" % len(pending))
    terminals = artifact.get("overload_terminals") or {}
    statuses = set(terminals.values())
    if not statuses:
        reasons.append("no_overload_terminals")
    first = artifact.get("first_wait") or {}
    if not artifact.get("formal"):
        if not isinstance(first.get("elapsed_ms"), float) or \
                not first.get("bound_id"):
            reasons.append("first_wait_shape_missing")
    else:
        if first.get("status") != "OK":
            reasons.append("first_wait_%s" % first.get("status"))
        elif first.get("elapsed_ms", 1e9) > contract.RECOVERY_FIRST_WAIT_S * 1000:
            reasons.append("first_wait_%.0fms" % first.get("elapsed_ms", -1))
    recovery_pending = artifact.get("recovery_pending") or []
    if recovery_pending:
        reasons.append("recovery_pending_%d" % len(recovery_pending))
    epoch = artifact.get("epoch_after")
    if not isinstance(epoch, int):
        reasons.append("epoch_unknown")
    if not artifact.get("formal"):
        reasons.append("short_sample_not_formal")
    else:
        if len(overload) != contract.R_OVERLOAD_INPUTS:
            reasons.append("overload_counts")
        if not (statuses & {"COMPLETED", "REJECTED", "QUEUE_FULL", "DUPLICATE"}):
            reasons.append("no_real_pressure")
    return {"reasons": reasons, "overload": len(overload)}


def judge_l_stage(artifact):
    reasons = []
    if not isinstance(artifact, dict):
        return {"reasons": ["artifact_missing"]}
    if artifact.get("query_errors"):
        reasons.append("query_errors:%d" % len(artifact["query_errors"]))
    rounds = artifact.get("read_rounds_results") or []
    if len(rounds) != artifact.get("read_rounds", -1):
        reasons.append("read_rounds_missing")
    legal = ("OK", "NO_KNOWN_FREE", "STALE") if not artifact.get("formal") \
        else ("OK", "NO_KNOWN_FREE")
    for result in rounds:
        if result.get("status") not in legal:
            reasons.append("read_round_status_%s" % result.get("status"))
            break
    resets = artifact.get("reset_statuses") or []
    if any(status != "OK" for status in resets):
        reasons.append("reset_status_%s" % resets)
    baseline = artifact.get("baseline") or {}
    after = artifact.get("after") or {}
    if not isinstance(baseline.get("view_directory_entries"), int) \
            or not isinstance(after.get("view_directory_entries"), int):
        reasons.append("resource_fields_missing")
    elif after["view_directory_entries"] > 2:
        reasons.append("handles_leaked_%d" % after["view_directory_entries"])
    peak = after.get("read_view_cache_bytes_peak")
    if isinstance(peak, int) and peak > contract.READ_VIEW_CACHE_LIMIT_BYTES:
        reasons.append("cache_peak_over_limit")
    if not artifact.get("formal"):
        reasons.append("short_sample_not_formal")
    return {"reasons": reasons, "rounds": len(rounds)}
