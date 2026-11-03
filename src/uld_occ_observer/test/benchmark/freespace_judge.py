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


# --- P/R/L judges: one fail-closed rule set for rehearsals and formal runs.
# EVERY rule examines EVERY record -- a loop that breaks after the first row
# is exactly how 14399 broken queries passed next to one good one. Statuses,
# identities, release results, numeric sanity, ID-set coverage, and content
# digests are recomputed from the raw records in BOTH paths; the formal
# branch adds the exact contract counts, latency/deadline gates, pressure
# evidence, duration, RSS buckets, and per-round ownership. Nothing passes on
# sample counts, self-reported aggregates, or a string being merely present.
import hashlib
import math

QUERY_SEMANTIC_OK = ("OK", "NO_KNOWN_FREE")
# The receipts topic stamps a completed integration INTEGRATED; the status
# service answers COMPLETED for the same id. Both are the one success
# terminal; everything else is a rejection or a non-terminal state.
INPUT_SUCCESS = ("INTEGRATED", "COMPLETED")
QUERY_PRESSURE_OK = QUERY_SEMANTIC_OK + ("STALE", "SNAPSHOT_PRESSURE")
# gen4: an ACQUIRE_* record (no lease) is legal ONLY inside these explicitly
# planned phases -- never by default in a scored window. NO_LEASE_WARMUP
# covers the just-reset gaps of the L warmup (the judge recomputes the gap
# intervals from the reset rounds and the first INTEGRATED arrival after
# each); NO_LEASE_PRESSURE covers exactly the R overload and drain loops.
NO_LEASE_WARMUP = frozenset({"ACQUIRE_NO_DATA"})
NO_LEASE_PRESSURE = frozenset({"ACQUIRE_STALE", "ACQUIRE_SNAPSHOT_PRESSURE",
                               "ACQUIRE_NO_DATA"})
LEGAL_QUERY_YAWS = tuple(contract.F_YAWS)


def canonical_candidate_bytes(yaw, x, y, z, kind):
    """Same canonical stream the O9 profile CLI hashes, so a Python-run chain
    and a C++-run chain agree on content identity."""
    return ("%d,%d,%d,%d,%d;" % (yaw, x, y, z, kind)).encode()


def candidate_digest(candidates):
    """Recompute the canonical digest from the raw candidate list. A cell
    that is not five numbers returns None -- content is verified, never
    trusted from a stored string."""
    digest = hashlib.sha256()
    for cell in candidates or []:
        try:
            if not isinstance(cell, (list, tuple)) or len(cell) != 5:
                return None
            digest.update(canonical_candidate_bytes(*[int(value) for value in cell]))
        except (TypeError, ValueError):
            return None
    return digest.hexdigest()


def _bad_number(value):
    """Missing, boolean, non-finite, or negative -- all fail closed."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return True
    if not math.isfinite(value):
        return True
    return value < 0


def _p95(values):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)]


def _median(values):
    ordered = sorted(values)
    count = len(ordered)
    if not count:
        return None
    middle = count // 2
    if count % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0


def _identity_problem(identity):
    """The complete five-field identity with honest types and value ranges:
    a plausible epoch/map_revision starts at 1, revisions never go negative,
    and a stamp is a non-negative nanosecond count (epoch=-999 dies here
    before the binding check names it again)."""
    if not isinstance(identity, dict):
        return "identity_missing"
    for key in ("epoch", "map_revision", "observed_revision",
                "last_integrated_stamp_ns"):
        value = identity.get(key)
        if isinstance(value, bool) or not isinstance(value, int):
            return "identity_%s_bad" % key
        if value < 0 or (key in ("epoch", "map_revision") and value < 1):
            return "identity_%s_out_of_range" % key
    if not isinstance(identity.get("geometry_hash"), str) \
            or not identity["geometry_hash"]:
        return "identity_geometry_hash_bad"
    return None


def _binding_problems(binding, prefix):
    """The stage artifact must carry the identity it started with: the epoch
    every lease in the stage must echo, the frozen geometry hash, and the
    config/clock binding. Missing here fails the stage once, not per record."""
    if not isinstance(binding, dict):
        return ["%sidentity_binding_missing" % prefix]
    problems = []
    epoch = binding.get("expected_epoch")
    if isinstance(epoch, bool) or not isinstance(epoch, int) or epoch < 1:
        problems.append("%sbinding_epoch_bad" % prefix)
    for key in ("geometry_hash", "config_digest", "clock_domain"):
        if not isinstance(binding.get(key), str) or not binding[key]:
            problems.append("%sbinding_%s_missing" % (prefix, key))
    return problems


def _identity_bind_problems(binding, identity, prefix):
    """Every lease must carry the identity the stage bound at its start: the
    epoch and the geometry hash are cross-checked record by record."""
    if not isinstance(binding, dict):
        return []
    problems = []
    want_epoch = binding.get("expected_epoch")
    got_epoch = (identity or {}).get("epoch")
    if isinstance(want_epoch, int) and not isinstance(want_epoch, bool) \
            and got_epoch != want_epoch:
        problems.append("%sepoch_not_bound:%s!=%s" % (prefix, got_epoch,
                                                      want_epoch))
    want_hash = binding.get("geometry_hash")
    got_hash = (identity or {}).get("geometry_hash")
    if isinstance(want_hash, str) and want_hash \
            and got_hash != want_hash:
        problems.append("%sgeometry_unbound" % prefix)
    return problems


def _revision_monotonicity_problems(queries, prefix):
    """Ordered by planned time, revisions never go backwards inside one
    epoch segment (a reset legitimately starts a new segment)."""
    problems = []
    ordered = sorted(queries,
                     key=lambda record: record.get("planned_mono_ns")
                     if isinstance(record.get("planned_mono_ns"), int) else 0)
    last = (None, None, None)
    for index, record in enumerate(ordered):
        status = record.get("status")
        if isinstance(status, str) and status.startswith("ACQUIRE_"):
            continue
        identity = record.get("lease_identity") or {}
        triple = tuple(identity.get(key) for key in
                       ("epoch", "map_revision", "observed_revision"))
        if not all(isinstance(value, int) and not isinstance(value, bool)
                   for value in triple):
            continue
        if last[0] == triple[0]:
            if triple[1] < last[1]:
                problems.append("%smap_revision_regressed_%d" % (prefix, index))
            if triple[2] < last[2]:
                problems.append("%sobserved_revision_regressed_%d"
                                % (prefix, index))
        last = triple
    return problems


def _result_completeness_problems(record, prefix):
    """A semantic answer must RECONCILE, not merely hash: the recorded page
    stream is a contiguous cursor chain from 0 whose final page is done, the
    page counts sum to the materialized candidate list AND to the reported
    feasible_total, candidates are unique 5-tuples with a legal yaw, and the
    status agrees with the content (an empty answer is NO_KNOWN_FREE)."""
    problems = []
    if record.get("status") not in QUERY_SEMANTIC_OK:
        return problems
    detail = record.get("pages_detail")
    candidates = record.get("candidates") or []
    feasible = record.get("feasible_total")
    if not isinstance(detail, list) or not detail:
        return problems + ["%spages_detail_missing" % prefix]
    if record.get("pages") != len(detail):
        problems.append("%spages_ne_pages_detail" % prefix)
    counts = 0
    expected_cursor = 0
    for index, page in enumerate(detail):
        count = page.get("count")
        cursor = page.get("cursor")
        nxt = page.get("next_cursor")
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            problems.append("%spage_count_bad_%d" % (prefix, index))
            expected_cursor = None
            continue
        counts += count
        if expected_cursor is None:
            continue
        if cursor != expected_cursor:
            problems.append("%spage_cursor_chain_%d:%s!=%s"
                            % (prefix, index, cursor, expected_cursor))
            expected_cursor = None
            continue
        if nxt != cursor + count:
            problems.append("%spage_cursor_progress_%d" % (prefix, index))
            expected_cursor = None
            continue
        expected_cursor = nxt
        if page.get("done") is not (index == len(detail) - 1):
            problems.append("%spage_done_flag_%d" % (prefix, index))
    if counts != len(candidates):
        problems.append("%sresult_count_%d_ne_candidates_%d"
                        % (prefix, counts, len(candidates)))
    if isinstance(feasible, bool) or not isinstance(feasible, int):
        problems.append("%sfeasible_total_bad" % prefix)
    elif feasible != len(candidates):
        problems.append("%sresult_count_%d_ne_feasible_total_%d"
                        % (prefix, len(candidates), feasible))
    seen_cells = set()
    for cell in candidates:
        if not isinstance(cell, (list, tuple)) or len(cell) != 5:
            problems.append("%sresult_cell_shape" % prefix)
            break
        key = tuple(cell)
        if key in seen_cells:
            problems.append("%sresult_duplicate_candidate_%s" % (prefix, key))
            break
        seen_cells.add(key)
        if key[0] not in LEGAL_QUERY_YAWS:
            problems.append("%sresult_yaw_out_of_domain_%s" % (prefix, key[0]))
            break
    if record.get("status") == "OK" and not candidates:
        problems.append("%sresult_status_content_mismatch" % prefix)
    if record.get("status") == "NO_KNOWN_FREE" and candidates:
        problems.append("%sresult_status_content_mismatch" % prefix)
    return problems


def _query_record_problems(record, legal_statuses, prefix,
                           allowed_no_lease=frozenset(), binding=None):
    """One full-query record, every field. A semantic answer must carry the
    materialized candidates, a digest that matches them, AND a reconciled
    page/count/candidate result; a no-lease ACQUIRE_* is legal only when the
    CALLING PHASE explicitly allows it (scored windows allow none); anything
    that DID take a lease must have released it, timeout included, and the
    lease must be fresh (age within the contract) and bound to the stage
    identity."""
    problems = []
    status = record.get("status")
    no_lease = isinstance(status, str) and status.startswith("ACQUIRE_")
    tolerated_no_lease = no_lease and status in allowed_no_lease
    if status not in legal_statuses and not tolerated_no_lease:
        problems.append("%sstatus_%s" % (prefix, status))
    if _bad_number(record.get("latency_ms")):
        problems.append("%slatency_ms_bad" % (prefix,))
    if no_lease:
        if record.get("release_status") is not None:
            problems.append("%srelease_%s_without_lease"
                            % (prefix, record.get("release_status")))
    else:
        if record.get("release_status") != "OK":
            problems.append("%srelease_%s" % (prefix, record.get("release_status")))
        identity_problem = _identity_problem(record.get("lease_identity"))
        if identity_problem:
            problems.append("%s%s" % (prefix, identity_problem))
        else:
            problems.extend(_identity_bind_problems(binding,
                                                    record.get("lease_identity"),
                                                    prefix))
        age = record.get("age_ns")
        if _bad_number(age) or age > int(contract.SNAPSHOT_AGE_MAX_S * 1e9):
            problems.append("%sage_%s" % (prefix, age))
    if status in QUERY_SEMANTIC_OK:
        if candidate_digest(record.get("candidates")) != record.get("content_sha256"):
            problems.append("%scontent_digest_mismatch" % prefix)
        problems.extend(_result_completeness_problems(record, prefix))
    return problems


def _input_row_problems(row, allowed_terminals, prefix):
    """One input row against its RAW receipts: the id must have exactly one
    terminal receipt (duplicates are kept, never collapsed), the E2E must
    recompute from the publish and receipt-arrival clocks, and the clocks
    must be present, ordered, and non-negative."""
    problems = []
    if row.get("receipt_count") != 1:
        problems.append("%sreceipt_count_%s" % (prefix, row.get("receipt_count")))
    status = row.get("receipt_status")
    if status not in allowed_terminals:
        problems.append("%snot_terminal_%s" % (prefix, status))
    if status in allowed_terminals:
        publish = row.get("publish_mono_ns")
        arrival = row.get("receipt_arrival_mono_ns")
        e2e = row.get("e2e_ms")
        if not isinstance(publish, int) or not isinstance(arrival, int):
            problems.append("%sboundaries_missing" % prefix)
        elif arrival < publish:
            problems.append("%se2e_negative" % prefix)
        elif not isinstance(e2e, (int, float)) or isinstance(e2e, bool) \
                or not math.isfinite(e2e) \
                or abs(e2e - (arrival - publish) / 1e6) > 1e-6:
            problems.append("%se2e_not_recomputed" % prefix)
    return problems


def _ph_boundaries(queries, sender, reasons):
    """Planned vs started/published boundaries for every record (no break):
    returns (query_late, send_late, latency) in ms; ordering sanity is a
    finding, never silently dropped."""
    query_late, send_late, latency = [], [], []
    for index, record in enumerate(queries):
        started, planned, ended = (record.get("started_mono_ns"),
                                   record.get("planned_mono_ns"),
                                   record.get("ended_mono_ns"))
        if not all(isinstance(value, int) for value in (started, planned, ended)):
            reasons.append("query_boundary_missing_%d" % index)
            continue
        if ended < started:
            reasons.append("q_ended_before_started_%d" % index)
            continue
        query_late.append((started - planned) / 1e6)
        value = (ended - planned) / 1e6
        if value < 0:
            reasons.append("q_latency_negative_%d" % index)
            continue
        latency.append(value)
    for index, record in enumerate(sender):
        published, planned = (record.get("publish_mono_ns"),
                              record.get("planned_mono_ns"))
        if not isinstance(published, int) or not isinstance(planned, int):
            reasons.append("send_boundary_missing_%d" % index)
            continue
        send_late.append((published - planned) / 1e6)
    return query_late, send_late, latency


def _latency_gate_problems(e2e, latency, query_late, send_late):
    """The P/H latency gates, applied to whatever list they must bind."""
    problems = []
    if _p95(e2e) is None:
        problems.append("e2e_samples_missing")
        return problems
    if _p95(e2e) > contract.INPUT_E2E_P95_MS:
        problems.append("input_e2e_p95_%.0f" % _p95(e2e))
    if max(e2e) > contract.INPUT_E2E_MAX_MS:
        problems.append("input_e2e_max_%.0f" % max(e2e))
    if latency:
        if _p95(latency) > contract.QUERY_FULL_P95_MS:
            problems.append("query_p95_%.0f" % _p95(latency))
        if max(latency) > contract.QUERY_FULL_MAX_MS:
            problems.append("query_max_%.0f" % max(latency))
    else:
        problems.append("latency_samples_missing")
    if query_late:
        if _p95(query_late) > contract.SCHED_LATE_P95_MS:
            problems.append("sched_late_p95_%.0f" % _p95(query_late))
        if max(query_late) > contract.SCHED_LATE_MAX_MS:
            problems.append("sched_late_max_%.0f" % max(query_late))
    if send_late:
        if _p95(send_late) > contract.SCHED_LATE_P95_MS:
            problems.append("send_late_p95_%.0f" % _p95(send_late))
        if max(send_late) > contract.SCHED_LATE_MAX_MS:
            problems.append("send_late_max_%.0f" % max(send_late))
    return problems


def _health_series_problems(series, prefix, window_s=None):
    """Liveness recomputed from the RAW health stamps -- never from a
    self-reported maximum. Missing series, misordered stamps, and any gap at
    or beyond the freshness bound are findings; a density floor (0.8x the
    0.5 s cadence) applies when the caller names the window (formal)."""
    if not isinstance(series, list) or not series:
        return ["%shealth_series_missing" % prefix]
    problems = []
    last = None
    for index, sample in enumerate(series):
        stamp = sample.get("stamp_ns") if isinstance(sample, dict) else None
        if not isinstance(stamp, int) or isinstance(stamp, bool):
            problems.append("%shealth_stamp_bad_%d" % (prefix, index))
            continue
        if last is not None:
            if stamp <= last:
                problems.append("%shealth_stamp_not_monotonic_%d"
                                % (prefix, index))
            elif (stamp - last) / 1e9 >= contract.SNAPSHOT_AGE_MAX_S:
                problems.append("%shealth_gap_%.2fs_%d"
                                % (prefix, (stamp - last) / 1e9, index))
        if last is None or stamp > last:
            last = stamp
    if window_s is not None and isinstance(window_s, (int, float)):
        expected = int(window_s / 0.5 * 0.8)
        if len(series) < expected:
            problems.append("%shealth_sample_thin_%d" % (prefix, len(series)))
    return problems


def judge_p_window(window):
    """One P window (H reads the same records: planned vs started/published).

    The timeline must cover EXACTLY the sender id set -- same count alone
    never proves it -- every id carries one terminal receipt, and every query
    record is checked in full (semantic status, released lease, complete
    identity, digest recomputed from the materialized candidates)."""
    reasons = []
    if not isinstance(window, dict):
        return {"reasons": ["window_missing"]}
    formal = bool(window.get("formal"))
    if formal and not window.get("window_id"):
        reasons.append("window_id_missing")
    if window.get("errors"):
        reasons.append("thread_errors:%s" % window["errors"][:2])
    sender = window.get("sender") or []
    queries = window.get("queries") or []
    timeline = window.get("input_timeline") or []
    if not sender or not queries or not timeline:
        reasons.append("missing_requests")
        return {"reasons": reasons, "formal": formal,
                "inputs": len(sender), "queries": len(queries)}

    ids = [record.get("input_id") for record in sender]
    if len(set(ids)) != len(ids):
        reasons.append("duplicate_input_id")
    if len(timeline) != len(sender):
        reasons.append("timeline_missing_rows:%d/%d" % (len(timeline), len(sender)))
    timeline_ids = [entry.get("input_id") for entry in timeline]
    if sorted(timeline_ids) != sorted(ids):
        reasons.append("timeline_id_set_mismatch")
    binding = window.get("identity_binding")
    reasons.extend(_binding_problems(binding, "p_"))
    e2e = []
    for entry in timeline:
        reasons.extend(_input_row_problems(entry, INPUT_SUCCESS, "i:"))
        if entry.get("receipt_status") in INPUT_SUCCESS \
                and isinstance(entry.get("e2e_ms"), (int, float)) \
                and not isinstance(entry.get("e2e_ms"), bool):
            e2e.append(entry["e2e_ms"])
    for record in queries:
        reasons.extend(_query_record_problems(record, QUERY_SEMANTIC_OK, "q:",
                                              binding=binding))
    reasons.extend(_revision_monotonicity_problems(queries, "q_"))
    reasons.extend(_health_series_problems(
        window.get("health_series"), "p_",
        window_s=window.get("window_planned_s") if formal else None))
    query_late, send_late, latency = _ph_boundaries(queries, sender, reasons)

    if formal:
        if len(sender) != contract.P_WINDOW_INPUTS \
                or len(queries) != contract.P_WINDOW_QUERIES:
            reasons.append("formal_counts")
        reasons.extend(_latency_gate_problems(e2e, latency, query_late, send_late))
        close = window.get("window_close_ms")
        planned_ms = window.get("window_planned_s")
        if _bad_number(close) or not isinstance(planned_ms, (int, float)) \
                or close > planned_ms * 1000 + 500:
            reasons.append("window_close_%s" % close)
        return {"reasons": reasons, "formal": True,
                "inputs": len(sender), "queries": len(queries),
                "measured": {
                    "inputs": len(sender), "queries": len(queries),
                    "e2e_p95_ms": _p95(e2e), "e2e_max_ms": max(e2e)
                    if e2e else None,
                    "query_p95_ms": _p95(latency), "query_max_ms": max(latency)
                    if latency else None,
                    "sched_late_p95_ms": _p95(query_late + send_late),
                    "sched_late_max_ms": max(query_late + send_late)
                    if query_late or send_late else None}}
    if not reasons:
        reasons.append("short_sample_not_formal")
    return {"reasons": reasons, "formal": False,
            "inputs": len(sender), "queries": len(queries)}


def judge_p_matrix(windows):
    """The nine formal windows: 3 box counts x 3 windows, every (box, window)
    id present exactly once and each window green on its own."""
    problems = []
    expected = {(box, index) for box in contract.P_BOX_COUNTS
                for index in range(contract.P_WINDOWS_PER_BOX)}
    seen = set()
    for window in windows:
        key = (window.get("box_count"), window.get("window_index"))
        if key in seen:
            problems.append("p_window_duplicate_%s" % (key,))
        seen.add(key)
    missing = expected - seen
    extra = seen - expected
    if missing:
        problems.append("p_window_missing_%s" % sorted(missing))
    if extra:
        problems.append("p_window_extra_%s" % sorted(extra))
    for window in windows:
        verdict = judge_p_window(window)
        if verdict.get("reasons"):
            problems.append("p_window_%s:%s"
                            % (window.get("window_id"), verdict["reasons"][:3]))
    return problems


LEGAL_TERMINAL = ("INTEGRATED", "COMPLETED", "REJECTED", "QUEUE_FULL",
                  "DUPLICATE", "EPOCH_MISMATCH", "OPTICAL_FRAME_MISMATCH",
                  "TF_STAMP_MISMATCH", "NO_VALID_POINTS", "OUT_OF_ORDER",
                  "CONFLICT")


def judge_r_stage(artifact):
    """Overload -> drain -> recovery, judged from raw records only.

    EVERY overload and recovery row keeps its receipts (duplicates counted,
    never collapsed to the last); the drain verdict recomputes from receipt
    ARRIVAL times against the exact deadline -- a success that landed after
    the deadline is a late drain, and any extra status polling is diagnostic
    only. Every overload/drain/recovery query record is checked in full; the
    first Wait must share the first recovery input's planned instant, run the
    candidate chain on ITS OWN lease, and release it."""
    reasons = []
    if not isinstance(artifact, dict):
        return {"reasons": ["artifact_missing"]}
    formal = bool(artifact.get("formal"))
    overload = artifact.get("overload") or []
    timeline = artifact.get("overload_timeline") or []
    recovery = artifact.get("recovery") or []
    recovery_timeline = artifact.get("recovery_timeline") or []
    first = artifact.get("first_wait") or {}
    if not overload:
        reasons.append("no_overload_inputs")
    if len(timeline) != len(overload):
        reasons.append("overload_timeline_missing_rows:%d/%d"
                       % (len(timeline), len(overload)))
    for entry in timeline:
        reasons.extend(_input_row_problems(entry, LEGAL_TERMINAL, "o:"))
    non_terminal = [entry.get("input_id") for entry in timeline
                    if entry.get("receipt_status") not in LEGAL_TERMINAL]
    if overload and non_terminal:
        reasons.append("overload_not_terminal:%d" % len(non_terminal))
    if not recovery:
        reasons.append("no_recovery_inputs")
    if len(recovery_timeline) != len(recovery):
        reasons.append("recovery_timeline_missing_rows:%d/%d"
                       % (len(recovery_timeline), len(recovery)))
    for entry in recovery_timeline:
        reasons.extend(_input_row_problems(entry, INPUT_SUCCESS, "r:"))

    if not isinstance(first.get("bound_id"), str) or not first.get("bound_id"):
        reasons.append("first_wait_shape_missing")
    elapsed = first.get("elapsed_ms")
    if _bad_number(elapsed):
        reasons.append("first_wait_elapsed_bad")
    if recovery and first.get("planned_mono_ns") != recovery[0].get("planned_mono_ns"):
        reasons.append("first_wait_not_concurrent")
    binding = artifact.get("identity_binding")
    reasons.extend(_binding_problems(binding, "r_"))
    chain = first.get("chain") or {}
    if not isinstance(chain, dict) or not chain:
        reasons.append("first_wait_chain_missing")
    else:
        reasons.extend(_query_record_problems(chain, QUERY_SEMANTIC_OK, "fw:",
                                              binding=binding))

    # No-lease ACQUIRE_* records are legal ONLY in the explicitly-planned
    # pressure loops (overload and drain); the recovery queries and the first
    # Wait run on a fresh, integrating map and must take real leases.
    for record in (artifact.get("overload_queries") or []):
        reasons.extend(_query_record_problems(
            record, QUERY_PRESSURE_OK, "oq:",
            allowed_no_lease=NO_LEASE_PRESSURE, binding=binding))
    for record in (artifact.get("drain_queries") or []):
        reasons.extend(_query_record_problems(
            record, QUERY_PRESSURE_OK, "dq:",
            allowed_no_lease=NO_LEASE_PRESSURE, binding=binding))
    for record in (artifact.get("recovery_queries") or []):
        reasons.extend(_query_record_problems(record, QUERY_SEMANTIC_OK, "rq:",
                                              binding=binding))
    reasons.extend(_revision_monotonicity_problems(
        list(artifact.get("overload_queries") or [])
        + list(artifact.get("drain_queries") or [])
        + list(artifact.get("recovery_queries") or []), "r_"))

    epoch = artifact.get("epoch_after")
    if not isinstance(epoch, int):
        reasons.append("epoch_unknown")
    elif formal and epoch != artifact.get("epoch_before", epoch):
        reasons.append("epoch_restarted")

    deadline = artifact.get("drain_deadline_mono_ns")
    if not isinstance(deadline, int):
        reasons.append("drain_deadline_missing")
    elif timeline:
        in_flight, late = [], []
        for entry in timeline:
            arrival = entry.get("receipt_arrival_mono_ns")
            status = entry.get("receipt_status")
            if status not in LEGAL_TERMINAL:
                in_flight.append(entry.get("input_id"))
            elif isinstance(arrival, int) and arrival > deadline:
                late.append(entry.get("input_id"))
        if in_flight:
            reasons.append("drain_in_flight_%d" % len(in_flight))
        if late:
            reasons.append("drain_late_%d" % len(late))
        reported = artifact.get("in_flight_at_drain_deadline") or []
        if sorted(x for x in reported if x is not None) != sorted(in_flight):
            reasons.append("in_flight_report_mismatch")

    if formal:
        if len(overload) != contract.R_OVERLOAD_INPUTS:
            reasons.append("overload_counts")
        if len(artifact.get("overload_queries") or []) != contract.R_OVERLOAD_QUERIES:
            reasons.append("overload_query_counts")
        if len(artifact.get("drain_queries") or []) != contract.R_DRAIN_QUERIES:
            reasons.append("drain_query_counts")
        if len(recovery) != contract.R_RECOVERY_INPUTS:
            reasons.append("recovery_counts")
        if len(artifact.get("recovery_queries") or []) != contract.R_RECOVERY_QUERIES - 1:
            reasons.append("recovery_query_counts")
        manifest = artifact.get("frames_manifest") or []
        if not manifest:
            reasons.append("frames_manifest_missing")
        elif any(entry.get("box_tag") != 20 for entry in manifest):
            reasons.append("frames_not_20box")
        pressure = sum(1 for entry in timeline
                       if entry.get("receipt_status") in ("REJECTED", "QUEUE_FULL"))
        if pressure < 1:
            # The S3 execution contract (section 7): when the fixed overload
            # load never materializes actual queueing or rejection, the
            # PRESSURE COVERAGE is not_evaluated -- never auto-pressurize,
            # never pick a heavier rerun, and never fail the product for
            # outperforming the frozen load. The evidence (every row
            # INTEGRATED, e2e below the 10 Hz period) is recorded in measured.
            reasons.append("overload_pressure_not_evaluated")
        if first.get("status") != "OK":
            reasons.append("first_wait_%s" % first.get("status"))
        elif elapsed > contract.RECOVERY_FIRST_WAIT_S * 1000:
            reasons.append("first_wait_%.0fms" % elapsed)
        age = first.get("age_ns")
        if _bad_number(age) or age > 2 * int(1e9):
            reasons.append("first_wait_age_%s" % age)
    if not reasons and not formal:
        reasons.append("short_sample_not_formal")
    pressure_count = sum(1 for entry in timeline
                         if entry.get("receipt_status") in ("REJECTED", "QUEUE_FULL"))
    e2e = sorted(entry["e2e_ms"] for entry in timeline
                 if isinstance(entry.get("e2e_ms"), (int, float)))
    return {"reasons": reasons, "overload": len(overload), "formal": formal,
            "measured": {
                "overload_inputs": len(overload),
                "overload_pressure_inputs": pressure_count,
                "overload_e2e_ms_p95": e2e[int(len(e2e) * 0.95)] if e2e else None,
                "overload_e2e_ms_max": e2e[-1] if e2e else None,
                "drain_in_flight": len(artifact.get("in_flight_at_drain_deadline")
                                       or []),
                "drain_late": len(artifact.get("late_at_drain_deadline") or []),
                "first_wait_ms": elapsed,
                "recovery_inputs": len(recovery)}}


# gen4 per-owner reconciliation: after a round's chain is released, every
# REQUEST-LIFECYCLE owner class must sit back at the post-warmup baseline --
# the lease views (open handles: blocks AND directories), the client-held
# views (released but still referenced), the externally held surface buffers
# (a cache leak detached from any snapshot lands exactly there), and
# in-flight response bytes. The map-content classes are ACCOUNTED in every
# snapshot and bounded by the 512 MiB composite-peak and RSS gates but NOT
# reconciled to equality: the retained split is the store's cached CURRENT
# snapshot and the aggregates (merged directories/blocks/caches) legitimately
# track the live map (pruning shrinks, new content grows); a leaked holder
# of any of them necessarily appears in one of the owner classes above.
OWNERSHIP_RECONCILE_FIELDS = (
    "lease_view_block_bytes", "client_held_view_block_bytes",
    "lease_view_directory_entries", "client_held_view_directory_entries",
    "external_surface_buffer_bytes", "inflight_response_bytes")


def _class_residual_problems(ownership, baseline, prefix):
    """Per-class byte reconciliation: the residual (minimum over the retained
    samples -- a leak survives every sample, a transient drops out) of every
    class must equal the post-warmup baseline value. All samples are kept by
    the runner with timestamps; a single count minimum never substitutes."""
    problems = []
    samples = ownership.get("resource_samples")
    if not isinstance(samples, list) or not samples:
        return ["%ssamples_missing" % prefix]
    for field in OWNERSHIP_RECONCILE_FIELDS:
        values = []
        for sample in samples:
            if not isinstance(sample, dict):
                values = None
                break
            values.append(sample.get(field))
        if values is None or any(value is None for value in values):
            problems.append("%sclass_%s_missing" % (prefix, field))
            continue
        # The baseline target is itself the minimum over its own samples
        # when it was measured adaptively (a transient in the baseline's
        # last sample must not forgive a real residual).
        base = baseline.get(field)
        base_samples = baseline.get("resource_samples")
        if isinstance(base_samples, list) and base_samples:
            candidates = [sample.get(field) for sample in base_samples
                          if isinstance(sample, dict)]
            if len(candidates) == len(base_samples) \
                    and None not in candidates:
                base = min(candidates)
        if not isinstance(base, int) or isinstance(base, bool):
            problems.append("%sclass_%s_baseline_missing" % (prefix, field))
            continue
        if min(values) != base:
            problems.append("%sclass_%s_residual:%s!=%s"
                            % (prefix, field, min(values), base))
    if any(sample.get("uncovered_allocations") for sample in samples
           if isinstance(sample, dict)):
        problems.append("%suncovered_allocations" % prefix)
    return problems


def _split_identity_problems(snapshot, prefix):
    """The per-owner split must sum to the merged fields -- shared objects
    counted exactly once, verified on every full snapshot."""
    if not isinstance(snapshot, dict):
        return []

    def _num(key):
        value = snapshot.get(key)
        return value if isinstance(value, int) and not isinstance(value, bool) \
            else None
    problems = []
    blocks = [_num(key) for key in ("lease_view_block_bytes",
                                    "retained_view_block_bytes",
                                    "client_held_view_block_bytes")]
    old = _num("old_view_block_bytes")
    if None not in blocks and old is not None and sum(blocks) != old:
        problems.append("%sblock_split_inconsistent" % prefix)
    dirs = [_num(key) for key in ("lease_view_directory_entries",
                                  "retained_view_directory_entries",
                                  "client_held_view_directory_entries")]
    merged = _num("view_directory_entries")
    if None not in dirs and merged is not None and sum(dirs) > merged:
        problems.append("%sdirectory_split_inconsistent" % prefix)
    return problems


def _reset_gap_intervals(artifact):
    """Warmup reset gaps: from each reset's start to the first terminal
    success arrival after it (the tree is empty until the next input
    integrates). Only inside these intervals -- and only during warmup --
    is a no-lease ACQUIRE_NO_DATA query a legal honest answer."""
    gaps = []
    arrivals = sorted(
        entry.get("receipt_arrival_mono_ns")
        for entry in (artifact.get("input_timeline") or [])
        if entry.get("receipt_status") in INPUT_SUCCESS
        and isinstance(entry.get("receipt_arrival_mono_ns"), int))
    for record in (artifact.get("reset_rounds_results") or []):
        # The gap opens at the RESET CALL (started_mono_ns predates it: the
        # before-sampling runs first, and inputs integrate during it).
        start = record.get("reset_called_mono_ns") \
            if isinstance(record.get("reset_called_mono_ns"), int) \
            else record.get("started_mono_ns")
        if not isinstance(start, int):
            continue
        refill = next((arrival for arrival in arrivals if arrival > start), None)
        if refill is not None:
            gaps.append((start, refill))
    return gaps


def _ownership_round_problems(index, round_record, baseline, kind):
    """One read or reset round. A READ round must REALLY acquire a lease and
    release it -- a lease-refusing acquire is an honest failed attempt, but
    it is never a completed lifecycle round (the runner lists it in
    read_failed_attempts and it does not count toward the 1000). A RESET
    round holds a real lease across the reset and releases it explicitly.
    Both kinds reconcile every ownership class back to the baseline."""
    problems = []
    if kind == "read":
        status = round_record.get("status")
        if status not in QUERY_SEMANTIC_OK:
            problems.append("read_round_no_real_lease_%d_%s" % (index, status))
        if round_record.get("release_status") != "OK":
            problems.append("read_round_release_%d_%s"
                            % (index, round_record.get("release_status")))
    else:
        # The product's release is idempotent-OK by design (an unknown
        # handle is a no-op), so the OBSERVABLE error lifecycle is a real
        # lease held ACROSS a reset and explicitly released in a finally.
        # The per-owner split proves the reset semantics: right after the
        # reset NOTHING but live leases survives (retained and client-held
        # old-view blocks are zero; our cross-reset lease sits in the lease
        # class), and after the explicit release the round reconciles back
        # to its own pre-round state (delta: the warmup map legitimately
        # grows between rounds, but a round must leave nothing behind).
        if round_record.get("acquire_status") != "OK":
            problems.append("reset_round_acquire_%d_%s"
                            % (index, round_record.get("acquire_status")))
        if round_record.get("real_release") != "OK":
            problems.append("reset_round_real_release_%d_%s"
                            % (index, round_record.get("real_release")))
        if round_record.get("reset_status") != "OK":
            problems.append("reset_round_reset_%d_%s"
                            % (index, round_record.get("reset_status")))
        before = round_record.get("ownership_before") or {}
        if not before.get("resource_samples"):
            problems.append("reset_round_%d_before_missing" % index)
            before = {}
        after_reset = round_record.get("ownership_after_reset") or {}
        for field in ("retained_view_block_bytes",
                      "client_held_view_block_bytes"):
            value = after_reset.get(field)
            if not isinstance(value, int) or isinstance(value, bool) \
                    or value != 0:
                problems.append("reset_round_%d_%s_survived_reset:%s"
                                % (index, field, value))
        problems.extend(_class_residual_problems(
            round_record.get("ownership_after") or {}, before,
            "reset_round_%d_" % index))
        return problems
    problems.extend(_class_residual_problems(
        round_record.get("ownership_after") or {}, baseline,
        "%s_round_%d_" % (kind, index)))
    return problems


def judge_l_stage(artifact):
    """The 120-min soak, judged per record: exact counts, per-120s-window
    P/H gates over the 110 scored minutes, RSS by REAL TIME (strictly
    monotonic stamps, all 12 planned buckets present, first vs last scored
    10-min medians), and per-round ownership back to the baseline."""
    reasons = []
    if not isinstance(artifact, dict):
        return {"reasons": ["artifact_missing"]}
    formal = bool(artifact.get("formal"))
    if artifact.get("send_errors"):
        reasons.append("send_errors:%s" % artifact["send_errors"][:2])
    queries = artifact.get("query_records") or []
    if not queries:
        reasons.append("no_query_records")
    inputs = artifact.get("input_records") or []
    timeline = artifact.get("input_timeline") or []
    start_ns = inputs[0].get("planned_mono_ns") if inputs else None
    warmup_s = contract.L_WARMUP_S if formal \
        else max(1.0, (artifact.get("planned_seconds") or 2) / 2.0)
    warmup_end = start_ns + int(warmup_s * 1e9) \
        if isinstance(start_ns, int) else None
    binding = artifact.get("identity_binding")
    reasons.extend(_binding_problems(binding, "l_"))
    gaps = _reset_gap_intervals(artifact)
    for record in queries:
        planned = record.get("planned_mono_ns")
        in_warmup = isinstance(warmup_end, int) and isinstance(planned, int) \
            and planned < warmup_end
        allowed = frozenset()
        if in_warmup and any(a <= planned < b for a, b in gaps):
            allowed = NO_LEASE_WARMUP
        reasons.extend(_query_record_problems(
            record, QUERY_SEMANTIC_OK, "lq:", allowed_no_lease=allowed,
            binding=None if in_warmup else binding))
    reasons.extend(_revision_monotonicity_problems(queries, "lq_"))
    reasons.extend(_health_series_problems(
        artifact.get("health_series"), "l_",
        window_s=artifact.get("planned_seconds") if formal else None))

    if not inputs:
        reasons.append("no_input_records")
    if len(timeline) != len(inputs):
        reasons.append("input_timeline_missing_rows:%d/%d"
                       % (len(timeline), len(inputs)))
    ids = [record.get("input_id") for record in inputs]
    timeline_ids = [entry.get("input_id") for entry in timeline]
    if len(set(ids)) != len(ids):
        reasons.append("duplicate_input_id")
    if sorted(timeline_ids) != sorted(ids):
        reasons.append("timeline_id_set_mismatch")
    for entry in timeline:
        reasons.extend(_input_row_problems(entry, INPUT_SUCCESS, "li:"))
    for record in queries:
        for key in ("planned_mono_ns", "started_mono_ns", "ended_mono_ns"):
            if not isinstance(record.get(key), int):
                reasons.append("lq_boundary_missing_%s" % key)
                break

    baseline = artifact.get("baseline") or {}
    after = artifact.get("after") or {}
    baseline_entries = baseline.get("view_directory_entries")
    if not isinstance(baseline_entries, int):
        reasons.append("resource_fields_missing")
    missing_baseline = [field for field in OWNERSHIP_RECONCILE_FIELDS
                        if not isinstance(baseline.get(field), int)]
    if missing_baseline:
        reasons.append("baseline_class_fields_missing:%s"
                       % missing_baseline[:3])
    reasons.extend(_split_identity_problems(baseline, "baseline"))
    rounds = artifact.get("read_rounds_results") or []
    if len(rounds) != artifact.get("read_rounds", -1):
        reasons.append("read_rounds_missing")
    if not isinstance(artifact.get("read_failed_attempts"), list):
        reasons.append("read_failed_attempts_missing")
    for index, round_record in enumerate(rounds):
        reasons.extend(_ownership_round_problems(index, round_record,
                                                 baseline, "read"))
    resets = artifact.get("reset_rounds_results") or []
    if len(resets) != artifact.get("reset_rounds", -1):
        reasons.append("reset_rounds_missing")
    for index, round_record in enumerate(resets):
        reasons.extend(_ownership_round_problems(index, round_record,
                                                 baseline, "reset"))

    if not isinstance(after.get("view_directory_entries"), int):
        reasons.append("resource_fields_missing")
    else:
        # Stage end: every worker joined, nothing is live -- every OWNER
        # class must sit exactly at the baseline. The merged directory
        # count itself tracks the live map (pruning shrinks it) and is not
        # an equality gate; a lingering handle shows in the owner classes.
        reasons.extend(_class_residual_problems(after, baseline, "after_"))
        reasons.extend(_split_identity_problems(after, "after_"))
    peak = after.get("read_view_cache_bytes_peak")
    if _bad_number(peak):
        reasons.append("cache_peak_bad")
    elif peak > contract.READ_VIEW_CACHE_LIMIT_BYTES:
        reasons.append("cache_peak_over_limit")

    rss_series = artifact.get("rss_series") or []
    if not rss_series:
        reasons.append("rss_series_missing")
    for index, sample in enumerate(rss_series):
        if _bad_number(sample.get("rss_kib")):
            reasons.append("rss_sample_bad_%d" % index)
        if not isinstance(sample.get("t_mono_ns"), int):
            reasons.append("rss_time_missing_%d" % index)
        elif index and sample["t_mono_ns"] <= rss_series[index - 1]["t_mono_ns"]:
            reasons.append("rss_time_not_monotonic_%d" % index)

    if formal:
        if artifact.get("planned_seconds") != contract.L_DURATION_S:
            reasons.append("l_duration_%s" % artifact.get("planned_seconds"))
        if artifact.get("sent_inputs") != contract.L_INPUTS:
            reasons.append("l_inputs_%s" % artifact.get("sent_inputs"))
        if len(inputs) != contract.L_INPUTS:
            reasons.append("l_input_records_%d" % len(inputs))
        if len(queries) != contract.L_QUERIES:
            reasons.append("l_queries_%d" % len(queries))
        if len(rounds) != contract.L_READ_RELEASE_ROUNDS:
            reasons.append("l_read_rounds_%d" % len(rounds))
        if len(resets) != contract.L_RESET_ROUNDS:
            reasons.append("l_reset_rounds_%d" % len(resets))
        manifest = artifact.get("frames_manifest") or []
        if not manifest:
            reasons.append("frames_manifest_missing")
        elif any(entry.get("box_tag") != 20 for entry in manifest):
            reasons.append("frames_not_20box")
        # 120-s windows AND over the 110 scored minutes (warmup is the first
        # L_WARMUP_S seconds; its shape is checked above but not gated).
        if inputs and queries and not any(
                reason.startswith(("lq_boundary_missing", "query_boundary"))
                for reason in reasons):
            start = inputs[0].get("planned_mono_ns")
            window_ns = int(contract.P_WINDOW_S * 1e9)
            warmup_windows = int(contract.L_WARMUP_S / contract.P_WINDOW_S)
            input_buckets, query_buckets = {}, {}
            for record in inputs:
                bucket = int((record["planned_mono_ns"] - start) // window_ns)
                input_buckets.setdefault(bucket, []).append(record)
            for record in queries:
                bucket = int((record["planned_mono_ns"] - start) // window_ns)
                query_buckets.setdefault(bucket, []).append(record)
            expected_windows = int(contract.L_DURATION_S / contract.P_WINDOW_S)
            for bucket in range(expected_windows):
                window_inputs = input_buckets.get(bucket, [])
                window_queries = query_buckets.get(bucket, [])
                if bucket < warmup_windows:
                    # Warmup: shape only (resets run here; NO_DATA answers are
                    # the honest observable on a freshly reset tree).
                    if len(window_inputs) != contract.P_WINDOW_INPUTS:
                        reasons.append("l_warmup_inputs_w%d:%d"
                                       % (bucket, len(window_inputs)))
                    continue
                if len(window_inputs) != contract.P_WINDOW_INPUTS:
                    reasons.append("l_window_inputs_w%d:%d"
                                   % (bucket, len(window_inputs)))
                if len(window_queries) != contract.P_WINDOW_QUERIES:
                    reasons.append("l_window_queries_w%d:%d"
                                   % (bucket, len(window_queries)))
                    continue
                rows = timeline_by_id(artifact, window_inputs)
                e2e = [row["e2e_ms"] for row in rows
                       if row.get("receipt_status") in INPUT_SUCCESS
                       and isinstance(row.get("e2e_ms"), (int, float))]
                window_reasons = []
                query_late, send_late, latency = _ph_boundaries(
                    window_queries, window_inputs, window_reasons)
                if window_reasons:
                    reasons.extend(["l_window_boundary_w%d:%s"
                                    % (bucket, window_reasons[:2])])
                else:
                    gate = _latency_gate_problems(e2e, latency,
                                                  query_late, send_late)
                    if gate:
                        reasons.extend(gate)
        # RSS by real time: 12 planned buckets, all present, all dense; growth
        # compares the first and last scored 10-min medians.
        if rss_series and isinstance(rss_series[0].get("t_mono_ns"), int):
            span_ns = rss_series[-1]["t_mono_ns"] - rss_series[0]["t_mono_ns"]
            if span_ns < int((contract.L_DURATION_S - 5) * 1e9):
                reasons.append("l_span_short_%.0fs" % (span_ns / 1e9))
            else:
                t0 = rss_series[0]["t_mono_ns"]
                bucket_ns = int(600 * 1e9)
                planned_buckets = int(contract.L_DURATION_S // 600)
                counts = {}
                for sample in rss_series:
                    bucket = int((sample["t_mono_ns"] - t0) // bucket_ns)
                    counts[bucket] = counts.get(bucket, 0) + 1
                missing = [bucket for bucket in range(planned_buckets)
                           if counts.get(bucket, 0) < 600]
                if missing:
                    reasons.append("rss_bucket_missing_or_sparse:%s" % missing)
                values = [sample["rss_kib"] for sample in rss_series]
                if max(values) > contract.L_RSS_LIMIT_KIB:
                    reasons.append("rss_over_limit_%d" % max(values))
                warmup_end = t0 + int(contract.L_WARMUP_S * 1e9)
                t_end = rss_series[-1]["t_mono_ns"]
                first_window = [sample["rss_kib"] for sample in rss_series
                                if warmup_end <= sample["t_mono_ns"]
                                < warmup_end + bucket_ns]
                last_window = [sample["rss_kib"] for sample in rss_series
                               if t_end - bucket_ns <= sample["t_mono_ns"] <= t_end]
                if len(first_window) < 600 or len(last_window) < 600:
                    reasons.append("rss_growth_windows_thin")
                else:
                    growth = _median(last_window) / max(1.0, _median(first_window))
                    if growth > 1 + contract.L_RSS_GROWTH:
                        reasons.append("rss_growth_%.3f" % growth)
    if not reasons and not formal:
        reasons.append("short_sample_not_formal")
    rss_values = [sample.get("rss_kib") for sample in (artifact.get("rss_series")
                                                       or [])
                  if isinstance(sample.get("rss_kib"), (int, float))]
    failed_attempts = artifact.get("read_failed_attempts")
    return {"reasons": reasons, "rounds": len(rounds), "formal": formal,
            "measured": {
                "inputs": len(inputs), "queries": len(queries),
                "read_rounds_real": len(rounds),
                "read_failed_attempts": len(failed_attempts)
                if isinstance(failed_attempts, list) else None,
                "reset_rounds": len(resets),
                "rss_max_kib": max(rss_values) if rss_values else None,
                "read_view_cache_bytes_peak": peak}}


def timeline_by_id(artifact, records):
    """Recompute the timeline rows for a subset of input records."""
    rows = {entry.get("input_id"): entry
            for entry in (artifact.get("input_timeline") or [])}
    return [rows.get(record.get("input_id")) or {} for record in records]


# --- O9 independent re-judge (the stored verdict is never trusted alone) ----
O9_OPERATIONS = ("roi", "integrate_30k", "surface_first", "freespace_full")


def o9_document_problems(document):
    """Independent re-implementation of the sample validation: status, cache
    state transition, ROI size, and content-hash presence."""
    problems = []
    if not isinstance(document, dict):
        return ["o9_sample_not_a_document"]
    if document.get("status") != "OK":
        problems.append("o9_sample_status_%s" % document.get("status"))
    first = document.get("surface_first_call") or {}
    second = document.get("surface_second_call") or {}
    if first.get("cache_hit") is not False or second.get("cache_hit") is not True:
        problems.append("o9_surface_cache_transition")
    if int(document.get("roi_cells") or 0) != 4096:
        problems.append("o9_roi_cells_%s" % document.get("roi_cells"))
    if not document.get("roi_surface_content_sha256"):
        problems.append("o9_content_hash_missing")
    for key in ("integrate_external_ms", "roi_ms", "freespace_full_external_ms"):
        value = document.get(key)
        if _bad_number(value):
            problems.append("o9_%s_bad" % key)
    return problems


def judge_o9_samples(pairs, result):
    """Recompute the P95 ratios and the per-pair content identity from the
    raw per-pair samples and compare with the reported result: a tampered
    verdict, ratio, or missing trace cannot stay green.

    pairs: [{"on": {op: ms}, "off": {op: ms}, "freespace_on": sha,
             "freespace_off": sha, "content_on": sha, "content_off": sha}]
    """
    problems = []
    if len(pairs) != contract.O9_FREESPACE_PAIRS:
        problems.append("o9_pair_count_%d" % len(pairs))
    ratios = {}
    for name in O9_OPERATIONS:
        on = sorted(pair["on"][name] for pair in pairs
                    if isinstance(pair.get("on"), dict)
                    and isinstance(pair["on"].get(name), (int, float)))
        off = sorted(pair["off"][name] for pair in pairs
                     if isinstance(pair.get("off"), dict)
                     and isinstance(pair["off"].get(name), (int, float)))
        if len(on) != len(pairs) or len(off) != len(pairs):
            problems.append("o9_%s_samples_missing" % name)
            continue
        index = math.ceil(0.95 * len(on)) - 1
        if off[index] <= 0:
            problems.append("o9_%s_off_nonpositive" % name)
            continue
        ratios[name] = on[index] / off[index]
        if ratios[name] > 1.10:
            problems.append("o9_%s_ratio_%.4f" % (name, ratios[name]))
    for index, pair in enumerate(pairs):
        if pair.get("freespace_on") != pair.get("freespace_off"):
            problems.append("o9_freespace_digest_pair_%d" % index)
        if pair.get("content_on") != pair.get("content_off"):
            problems.append("o9_content_digest_pair_%d" % index)
    reported = (result or {}).get("ratios") or {}
    for name, value in ratios.items():
        if not isinstance(reported.get(name), (int, float)) \
                or abs(reported[name] - value) > 1e-9:
            problems.append("o9_ratio_mismatch_%s" % name)
    verdict = (result or {}).get("verdict") or {}
    if verdict.get("pass") is not True:
        problems.append("o9_verdict_not_pass")
    if verdict.get("performance_certified") is not True:
        problems.append("o9_not_performance_certified")
    if (result or {}).get("failures"):
        problems.append("o9_failures_%d" % len(result["failures"]))
    return problems
