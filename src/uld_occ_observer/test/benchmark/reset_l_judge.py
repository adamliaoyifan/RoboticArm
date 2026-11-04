#!/usr/bin/env python3
"""Reset-contract L judge (OCTOMAP_RESET_VALIDITY_NEXT_20261103 §5, L row).

  reset_l_judge.py --run <campaign-dir-containing-l.json>

RETROSPECTIVE TOOL -- see PROVENANCE in the verdict: this judge was first
committed AFTER the run02 freeze (d10dffe1) and is an offline recompute of
preserved raw records under the already-frozen contract text. It never
rewrites artifacts; the scoring-time freeze (FREEZE_INTEG.json) and the S3
S3-rule view (campaign summary.json) stay untouched beside its verdict.

Rule set (frozen plan §5 + §1 amendments over the S3 L rules):

1. Planned-ID discipline: the 18000 input stamps and 14400 query slots are
   derived from the raw arithmetic grid (2.5 Hz / 2 Hz from t0). Every
   planned ID present exactly once, no duplicates/unknowns, one terminal per
   row. At most 2 extra inputs and 2 extra queries -- real, retained, judged
   with the same latency/status/identity rules, reported as boundary rows,
   never substituted for planned IDs and never counted as a 120 s window.
2. Warmup [t0, t0+600 s) and the scored half-open window [t0+600 s, t0+7200 s)
   are separate segments. All reset rounds declare their own event window
   [reset_called, round sampling end]; a non-normal outcome is exempted ONLY
   with an execution-interval overlap with that window AND row-level identity
   proof (server-side rejection receipt, lease epoch, or the store-empty
   interval for ACQUIRE_NO_DATA). Planned-time overlap alone never exempts.
3. Scored segment: per-120 s-window AND of the full S3 P/H gates (e2e, query
   latency, schedule lateness, snapshot age, health liveness) via the frozen
   freespace_judge helpers; whole-run all-ID closure and input/query/age
   gates include the exempted rows.
4. Ownership (§1): size or survival at the reset instant is recorded, never
   gated; the verdict is per-round final release (round reconciles to its own
   pre-round state) plus stage-end residual == post-warmup baseline for every
   owner class. Every external_surface_buffer byte must belong to the
   surface-cache entry (external <= cache at every sample) -- constancy of
   the cache counter alone is never the ownership proof.
5. Resources: RSS by real time (12 dense 10-min buckets, max <= 2 GiB,
   first-vs-last scored 10-min medians <= +5%), same-instant read view+cache
   peak <= 512 MiB, 1000 real read/release rounds and 100 reset rounds
   validated per round -- never by array length alone.

The helpers are the S3 frozen ones (freespace_judge): per-record field
checks, latency gates, quantile method, ownership reconciliation.
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import freespace_contract as contract  # noqa: E402
import freespace_judge as s3  # noqa: E402

#: V1/V2 status contract for reset-window rows (plan §5: legal only with the
#: execution + identity proof this judge demands per row).
RESET_LEGAL_QUERY = {"ACQUIRE_NO_DATA", "PAGE_INVALIDATED", "INVALIDATED",
                     "NO_DATA", "STALE", "REVISION_NOT_COVERED"}
RESET_LEGAL_INPUT = {"EPOCH_MISMATCH", "REJECTED", "QUEUE_FULL"}
NORMAL_QUERY_OK = set(s3.QUERY_SEMANTIC_OK)   # served answers ("OK", "NO_KNOWN_FREE")
NORMAL_INPUT_OK = set(s3.INPUT_SUCCESS)
#: Max extra rows per stream (plan §5 L row); extras are judged, not dropped.
MAX_EXTRA = 2
VIEW_CACHE_PEAK_MIB = contract.READ_VIEW_CACHE_LIMIT_BYTES / 1048576.0
RULE_SET = "OCTOMAP_RESET_VALIDITY_NEXT_20261103 §5 L (reset contract)"


def _p95(values):
    return s3._p95(values)


#: §1 live-owner classes: retained by a legitimate holder for the stage's
#: life (the surface-cache entry owns its external buffer), so a ROUND
#: reconciliation cannot demand they return to the round's pre-state. They
#: are gated separately: every external byte must sit inside the cache entry
#: (external <= cache at every sample) and the entry must exist at baseline
#: AND stage end. Every other class stays under the frozen S3 residual rule.
LIVE_OWNER_FIELDS = ("external_surface_buffer_bytes",)


def _class_residual_round(after, before, prefix):
    """Frozen S3 per-class residual, minus the §1 live-owner fields."""
    skip = set(LIVE_OWNER_FIELDS)
    fields = [f for f in s3.OWNERSHIP_RECONCILE_FIELDS if f not in skip]
    problems = []
    samples = (after or {}).get("resource_samples")
    if not isinstance(samples, list) or not samples:
        return ["%ssamples_missing" % prefix]
    for field in fields:
        values = []
        for sample in samples:
            if not isinstance(sample, dict):
                values = None
                break
            values.append(sample.get(field))
        if values is None or any(value is None for value in values):
            problems.append("%sclass_%s_missing" % (prefix, field))
            continue
        base = (before or {}).get(field)
        base_samples = (before or {}).get("resource_samples")
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


def _grid_ok(rows, key_planned, key_id, hz, t0):
    """Exact planned arithmetic grid: returns (expected_ids, seen_ids, extras).

    The sender stamps inputs at base + i/hz and queries share the same t0 at
    2 Hz; anything off the grid is an unknown ID, any repeat is a duplicate.
    """
    step = int(round(1e9 / hz))
    seen = [row.get(key_id) for row in rows]
    planned_ns = [(row.get(key_planned) or 0) for row in rows]
    return step, seen, planned_ns


def judge(run: Path):
    l_path = run / "l.json"
    if not l_path.is_file():
        return {"status": "fail", "problems": ["missing_artifact:l.json"]}
    d = json.loads(l_path.read_text())
    problems = []
    tl = d.get("input_timeline") or []
    inputs = d.get("input_records") or []
    qs = d.get("query_records") or []
    rr = d.get("reset_rounds_results") or []
    read_rounds = d.get("read_rounds_results") or []
    formal = bool(d.get("formal"))
    if not formal:
        return {"status": "fail", "problems": ["not_formal_artifact"]}
    if d.get("send_errors"):
        problems.append("send_errors:%s" % d["send_errors"][:2])
    binding = d.get("identity_binding")
    problems.extend(s3._binding_problems(binding, "l_"))

    if not inputs or not qs:
        problems.append("records_missing")
        return _finish(run, d, problems, None, [], [], rr, read_rounds, binding)
    t0 = min(e.get("planned_mono_ns") or 0 for e in inputs)
    warmup_end = t0 + int(contract.L_WARMUP_S * 1e9)
    score_end = t0 + int(contract.L_DURATION_S * 1e9)

    # ---- gen4: board transition trace (hard evidence requirement) ----------
    # The node's server ring keeps only 64 events; the collector subscribes
    # from stage start. A formal gen4 artifact MUST carry the full trace;
    # a gap or a missing trace is evidence-incomplete = fail, never a
    # degraded pass. NOTE the board's `generation` is a TRANSITION counter
    # (every state change +1: boot BUILDING=1, VALID=2, reset INVALID=3,
    # BUILDING=4, VALID=5, ...): 100 resets span generations 1..302 with
    # exactly 100 INVALID events whose epochs advance 2..101 in order. The
    # boot event's transition_mono_ns==0 is tolerated (flagged), never
    # rewritten.
    validity_trace = d.get("validity_trace")
    invalid_events = []
    if not isinstance(validity_trace, list) or not validity_trace:
        problems.append("validity_trace_missing")
    else:
        generations = []
        for event in validity_trace:
            generation = event.get("generation")
            state = event.get("state")
            if not isinstance(generation, int) or not isinstance(state, str):
                problems.append("validity_trace_row_bad")
                continue
            generations.append(generation)
            if state == "INVALID":
                invalid_events.append(event)
        generations.sort()
        if generations != list(range(1, len(generations) + 1)):
            problems.append("validity_trace_generation_gap:%d..%d(%d rows)"
                            % (generations[0] if generations else -1,
                               generations[-1] if generations else -1,
                               len(generations)))
        ordered = sorted(validity_trace,
                         key=lambda e: e.get("generation") or 0)
        for prev, nxt in zip(ordered, ordered[1:]):
            if nxt.get("epoch") < prev.get("epoch"):
                problems.append("validity_trace_epoch_regression:g%s"
                                % nxt.get("generation"))
                break
        if len(invalid_events) != len(rr):
            problems.append("validity_trace_invalid_count:%d!=%d"
                            % (len(invalid_events), len(rr)))
        for index, event in enumerate(invalid_events):
            if event.get("epoch") != index + 2:
                problems.append("validity_trace_invalid_epoch_%d:%s"
                                % (index, event.get("epoch")))
                break

    # ---- frames / workload identity (item 2) -------------------------------
    manifest = d.get("frames_manifest") or []
    if not manifest:
        problems.append("frames_manifest_missing")
    else:
        if any(entry.get("box_tag") != 20 for entry in manifest):
            problems.append("frames_not_20box")
        if any(not entry.get("sha256") for entry in manifest):
            problems.append("frames_hash_missing")

    # ---- reset rounds: linearization windows + per-round lifecycle ----------
    windows = []          # (round_index, called_ns, round_end_ns, epoch_pre, epoch_post)
    at_reset_view = []    # recorded, never gated (§1)
    round_linearizations = {}
    for index, record in enumerate(rr):
        epoch = record.get("epoch")
        if epoch != index + 2:
            problems.append("reset_round_%d_epoch_%s" % (index, epoch))
        for field in ("acquire_status", "real_release", "reset_status"):
            if record.get(field) != "OK":
                problems.append("reset_round_%d_%s_%s"
                                % (index, field, record.get(field)))
        called = record.get("reset_called_mono_ns")
        after_reset = record.get("ownership_after_reset") or {}
        after = record.get("ownership_after") or {}
        if not isinstance(called, int):
            problems.append("reset_round_%d_called_missing" % index)
            continue
        samples = (after.get("resource_samples")
                   or [{"t_mono_ns": after.get("t_mono_ns")}])
        end = max((s.get("t_mono_ns") or 0) for s in samples) \
            if samples else called
        if end < called:
            problems.append("reset_round_%d_window_inverted" % index)
        # gen4 linearization proof: this round's INVALID transition (node
        # steady clock == client CLOCK_MONOTONIC on the same machine) must
        # land between the client's reset CALL and the first post-reset
        # resource sample -- the raw timestamp the reset actually won.
        linearization = None
        if index < len(invalid_events):
            stamp = invalid_events[index].get("transition_mono_ns")
            if isinstance(stamp, int) and stamp > 0 \
                    and called - 100_000_000 <= stamp <= end:
                linearization = stamp
        if linearization is None and invalid_events:
            problems.append("reset_round_%d_linearization_unproven" % index)
        windows.append((index, called, end, index + 1, epoch))
        if linearization is not None:
            round_linearizations[index] = linearization
        # Recorded (not gated): the S3 at-instant survival view, superseded by
        # §1 final-release. Kept in the verdict for the dual-view report.
        at_reset_view.append({
            "round": index,
            "client_held_view_block_bytes":
                after_reset.get("client_held_view_block_bytes"),
            "external_surface_buffer_bytes":
                after_reset.get("external_surface_buffer_bytes")})
        # §1 verdict for the round: reconcile after final release against the
        # round's OWN pre-round state (the map grows legitimately between
        # rounds; a round must leave nothing behind) -- minus the live-owner
        # classes, which are gated by the external<=cache rule below.
        before = record.get("ownership_before") or {}
        if not before.get("resource_samples"):
            problems.append("reset_round_%d_before_missing" % index)
        problems.extend(_class_residual_round(
            after, before, "reset_round_%d_" % index))
    if len(rr) != contract.L_RESET_ROUNDS or len(rr) != d.get("reset_rounds", -1):
        problems.append("reset_rounds_%d" % len(rr))
    if not windows:
        problems.append("reset_windows_missing")
        return _finish(run, d, problems, None, qs, tl, rr, read_rounds, binding)

    def window_hit(started, ended):
        for index, called, end, _pre, _post in windows:
            if started is not None and started <= end and \
                    (ended is None or ended >= called):
                if ended is not None and ended < called:
                    continue
                return index
        return None

    # ---- planned-ID discipline (grid, duplicates, unknowns, extras) --------
    step_i = int(round(1e9 / (2.5)))
    step_q = int(round(1e9 / 2.0))
    input_ids = [e.get("input_id") for e in tl]
    if len(set(input_ids)) != len(input_ids):
        problems.append("duplicate_input_id_count_%d"
                        % (len(input_ids) - len(set(input_ids))))
    if len(tl) != len(inputs) or \
            sorted(e.get("input_id") for e in inputs) != sorted(input_ids):
        problems.append("timeline_records_mismatch")
    # Inputs: stamp grid; ID == str(stamp) by sender construction.
    base_stamp = inputs[0].get("stamp_ns")
    if not isinstance(base_stamp, int):
        problems.append("stamp_base_missing")
    else:
        seen_stamps = [e.get("stamp_ns") for e in inputs]
        if len(set(seen_stamps)) != len(seen_stamps):
            problems.append("duplicate_stamp")
        planned_stamps = {base_stamp + i * step_i
                          for i in range(contract.L_INPUTS)}
        unknown = [s for s in seen_stamps if s not in planned_stamps]
        missing = planned_stamps - set(seen_stamps)
        extra_inputs = [s for s in unknown
                        if not (base_stamp + contract.L_INPUTS * step_i
                                <= s
                                <= base_stamp + (contract.L_INPUTS + MAX_EXTRA)
                                * step_i)]
        if missing:
            problems.append("planned_inputs_missing:%d" % len(missing))
        if len(unknown) > MAX_EXTRA or extra_inputs:
            problems.append("input_overrun_or_offgrid:%d" % len(unknown))
        by_id = {e.get("input_id"): e for e in inputs}
        for row in tl:
            stamp = by_id.get(row.get("input_id"), {}).get("stamp_ns")
            if isinstance(stamp, int) and row.get("input_id") != str(stamp):
                problems.append("input_id_not_stamp:%s" % row.get("input_id"))
                break
    # Queries: planned grid on index/planned time.
    query_indices = [q.get("index") for q in qs]
    if len(set(query_indices)) != len(query_indices):
        problems.append("duplicate_query_index")
    missing_q = [i for i in range(contract.L_QUERIES) if i not in set(query_indices)]
    extra_q_idx = [i for i in query_indices
                   if not (0 <= i < contract.L_QUERIES + MAX_EXTRA)]
    if missing_q:
        problems.append("planned_queries_missing:%d" % len(missing_q))
    if len([i for i in query_indices if i >= contract.L_QUERIES]) > MAX_EXTRA \
            or extra_q_idx:
        problems.append("query_overrun_or_offgrid:%d"
                        % len([i for i in query_indices if i >= contract.L_QUERIES]))

    # ---- segments + per-ID classification -----------------------------------
    exempt_queries, exempt_inputs = [], []
    for q in qs:
        q["_segment"] = ("warmup" if q["planned_mono_ns"] < warmup_end
                         else "scored" if q["planned_mono_ns"] < score_end
                         else "overrun")
    # Recovery boundary per query index: the lease epoch of the first SERVED
    # query at or after each index (None when none follows) -- the identity
    # evidence for pre-recovery ACQUIRE_NO_DATA rows.
    next_served_epoch = {}
    _last = None
    for q in reversed(qs):
        next_served_epoch[q.get("index")] = _last
        if q.get("status") in NORMAL_QUERY_OK:
            _last = (q.get("lease_identity") or {}).get("epoch")
    for e in tl:
        e["_segment"] = ("warmup" if e["planned_mono_ns"] < warmup_end
                         else "scored" if e["planned_mono_ns"] < score_end
                         else "overrun")

    def query_exemption(q):
        """(round, proof) when the non-normal outcome is reset-proven."""
        status = q.get("status")
        if status in NORMAL_QUERY_OK:
            return None
        started, ended = q.get("started_mono_ns"), q.get("ended_mono_ns")
        round_i = window_hit(started, ended)
        if round_i is None:
            return False
        index, called, end, pre, post = windows[round_i]
        identity = q.get("lease_identity") or {}
        epoch = identity.get("epoch")
        if status == "ACQUIRE_NO_DATA":
            # No lease exists by definition, so the identity proof is the
            # RECOVERY boundary: this query ran inside the round window AND
            # the first served query after it answers on the round's post
            # epoch -- the board had not recovered when this read raced it.
            # (A receipt-side "first integrated arrival" is NOT usable here:
            # in-flight pre-wipe callbacks land INTEGRATED receipts into the
            # abandoned store while the new store is still empty.)
            if started > end or ended < called:
                return False
            nxt = next_served_epoch.get(q.get("index"))
            if nxt != post:
                return False
            proof = "pre_recovery[next_served_epoch=%s==post_%s]" % (nxt, post)
        elif status == "PAGE_INVALIDATED":
            # Page acquired on the old epoch, wiped mid-read.
            if epoch != pre and epoch is not None:
                return False
            if started > end or ended < called:
                return False
            proof = "page_epoch_%s_pre_%d[called..round_end]" % (epoch, pre)
        else:
            if started > end or ended < called:
                return False
            proof = "execution[called..round_end]"
        return (round_i, proof, {"epoch": epoch,
                                 "map_revision": identity.get("map_revision")})

    for q in qs:
        status = q.get("status")
        if status in NORMAL_QUERY_OK:
            continue
        verdict = query_exemption(q)
        if verdict is False or verdict is None:
            problems.append("nonnormal_query_without_reset_proof:%s:%s:%s"
                            % (q.get("index"), status, q.get("_segment")))
        else:
            exempt_queries.append({"index": q.get("index"), "status": status,
                                   "segment": q.get("_segment"),
                                   "round": verdict[0], "proof": verdict[1],
                                   "identity": verdict[2],
                                   "started_mono_ns": q.get("started_mono_ns"),
                                   "ended_mono_ns": q.get("ended_mono_ns")})
    for e in tl:
        status = e.get("receipt_status")
        if status in NORMAL_INPUT_OK:
            continue
        started = e.get("publish_mono_ns")
        ended = e.get("receipt_arrival_mono_ns")
        round_i = window_hit(started, ended)
        if round_i is None:
            problems.append("nonnormal_input_without_reset_proof:%s:%s"
                            % (e.get("input_id"), status))
            continue
        index, called, end, pre, _post = windows[round_i]
        # EPOCH_MISMATCH is itself the server-side rejection receipt; the
        # publish must race the epoch bump inside the round window.
        if started > end or (ended is not None and ended < called):
            problems.append("input_planned_only_overlap:%s" % e.get("input_id"))
            continue
        exempt_inputs.append({"input_id": e.get("input_id"), "status": status,
                              "round": round_i,
                              "proof": "server_rejection[%s]" % status,
                              "publish_mono_ns": started,
                              "receipt_arrival_mono_ns": ended})

    # ---- per-row strict checks (frozen helpers) ------------------------------
    for q in qs:
        legal = NORMAL_QUERY_OK if q["status"] in NORMAL_QUERY_OK \
            else RESET_LEGAL_QUERY
        allowed_no_lease = {"ACQUIRE_NO_DATA"} \
            if q["status"] == "ACQUIRE_NO_DATA" else frozenset()
        # Epoch binding holds only for the scored segment: warmup rows echo
        # the pre-reset epochs by design; overrun rows echo the final epoch.
        bind = binding if q["_segment"] == "scored" else None
        problems.extend(s3._query_record_problems(
            q, tuple(legal), "lq:", allowed_no_lease=allowed_no_lease,
            binding=bind))
    problems.extend(s3._revision_monotonicity_problems(qs, "lq_"))
    for e in tl:
        legal = NORMAL_INPUT_OK if e.get("receipt_status") in NORMAL_INPUT_OK \
            else NORMAL_INPUT_OK | RESET_LEGAL_INPUT
        problems.extend(s3._input_row_problems(e, tuple(legal), "li:"))

    # ---- health liveness + epoch continuity ----------------------------------
    problems.extend(s3._health_series_problems(
        d.get("health_series"), "l_",
        window_s=contract.L_DURATION_S if formal else None))
    health_epochs = [h.get("epoch") for h in (d.get("health_series") or [])]
    if not health_epochs or any(
            not isinstance(e, int) for e in health_epochs) or \
            list(health_epochs) != sorted(health_epochs) or \
            set(health_epochs) != set(range(1, len(rr) + 2)):
        problems.append("health_epoch_continuity_broken")

    # ---- scored 120 s windows: full P/H gates via frozen helpers -------------
    window_ns = int(contract.P_WINDOW_S * 1e9)
    warmup_windows = int(contract.L_WARMUP_S / contract.P_WINDOW_S)
    expected_windows = int(contract.L_DURATION_S / contract.P_WINDOW_S)
    buckets_i, buckets_q = {}, {}
    rows_by_id = {e.get("input_id"): e for e in tl}
    for e in tl:
        buckets_i.setdefault(
            int((e["planned_mono_ns"] - t0) // window_ns), []).append(e)
    for q in qs:
        buckets_q.setdefault(
            int((q["planned_mono_ns"] - t0) // window_ns), []).append(q)
    window_fail = []
    for bucket in range(expected_windows):
        rows_i = buckets_i.get(bucket, [])
        rows_q = buckets_q.get(bucket, [])
        # Warmup: shape only (resets run here by design; the exempted rows
        # carry their own proof above).
        if bucket < warmup_windows:
            if len(rows_i) != contract.P_WINDOW_INPUTS:
                window_fail.append("warmup_w%d_inputs_%d"
                                   % (bucket, len(rows_i)))
            continue
        if not rows_i or not rows_q:
            window_fail.append("w%d_empty" % bucket)
            continue
        exempt_ids = {r["index"] for r in exempt_queries}
        normal_q = [q for q in rows_q if q["index"] not in exempt_ids]
        normal_i = [e for e in rows_i
                    if e.get("receipt_status") in NORMAL_INPUT_OK]
        if len(rows_i) != contract.P_WINDOW_INPUTS:
            window_fail.append("w%d_inputs_%d" % (bucket, len(rows_i)))
        if len(rows_q) != contract.P_WINDOW_QUERIES:
            window_fail.append("w%d_queries_%d" % (bucket, len(rows_q)))
            continue
        sender = [{"input_id": e["input_id"],
                   "planned_mono_ns": e["planned_mono_ns"],
                   "publish_mono_ns": e["publish_mono_ns"]} for e in rows_i]
        reasons = []
        query_late, send_late, latency = s3._ph_boundaries(
            normal_q, sender, reasons)
        e2e = [e["e2e_ms"] for e in normal_i
               if isinstance(e.get("e2e_ms"), (int, float))]
        gate = s3._latency_gate_problems(e2e, latency, query_late, send_late)
        if reasons or gate:
            window_fail.append("w%d:%s" % (bucket, (reasons + gate)[:3]))
    if window_fail:
        problems.extend(["window_and:%s" % f for f in window_fail[:8]])
        problems.append("window_and_count:%d" % len(window_fail))
    window_stats = {"expected": int(contract.L_DURATION_S
                                    / contract.P_WINDOW_S) - warmup_windows,
                    "failed": len(window_fail)}

    # ---- whole-run all-ID closure/latency (exempted rows stay counted) -------
    q_lat = [(q.get("ended_mono_ns", 0) - q.get("planned_mono_ns", 0)) / 1e6
             for q in qs
             if isinstance(q.get("ended_mono_ns"), int)
             and isinstance(q.get("planned_mono_ns"), int)]
    i_e2e = [e["e2e_ms"] for e in tl
             if e.get("receipt_status") in NORMAL_INPUT_OK
             and isinstance(e.get("e2e_ms"), (int, float))]
    no_terminal = [e["input_id"] for e in tl if not e.get("receipt_status")]
    if no_terminal:
        problems.append("inputs_without_terminal:%d" % len(no_terminal))
    if _p95(q_lat) is None or _p95(q_lat) > contract.QUERY_FULL_P95_MS:
        problems.append("allid_query_p95:%s" % _p95(q_lat))
    if q_lat and max(q_lat) > contract.QUERY_FULL_MAX_MS:
        problems.append("allid_query_max:%.1f" % max(q_lat))
    if _p95(i_e2e) is None or _p95(i_e2e) > contract.INPUT_E2E_P95_MS:
        problems.append("allid_input_p95:%s" % _p95(i_e2e))
    if i_e2e and max(i_e2e) > contract.INPUT_E2E_MAX_MS:
        problems.append("allid_input_max:%.1f" % max(i_e2e))
    ages = [q["age_ns"] / 1e9 for q in qs
            if isinstance(q.get("age_ns"), int)]
    if ages and max(ages) > contract.SNAPSHOT_AGE_MAX_S:
        problems.append("allid_age_max:%.3f" % max(ages))

    # ---- read/release rounds + external buffer ownership ----------------------
    if len(read_rounds) != contract.L_READ_RELEASE_ROUNDS or \
            len(read_rounds) != d.get("read_rounds", -1):
        problems.append("read_rounds_%d" % len(read_rounds))
    if d.get("read_failed_attempts") != []:
        problems.append("read_failed_attempts:%s"
                        % d.get("read_failed_attempts"))
    baseline = d.get("baseline") or {}
    for index, record in enumerate(read_rounds):
        problems.extend(s3._ownership_round_problems(
            index, record, baseline, "read"))
    external_series, cache_series = [], []

    def collect(snapshot):
        samples = snapshot.get("resource_samples") or [snapshot]
        for sample in samples:
            if isinstance(sample, dict):
                external_series.append(
                    sample.get("external_surface_buffer_bytes"))
                cache_series.append(sample.get("surface_cache_bytes"))
    for record in rr:
        collect(record.get("ownership_before") or {})
        collect(record.get("ownership_after_reset") or {})
        collect(record.get("ownership_after") or {})
    for record in read_rounds:
        collect(record.get("ownership_after") or {})
    collect(baseline)
    collect(d.get("after") or {})
    if not external_series or any(v is None for v in external_series):
        problems.append("external_surface_series_missing")
    else:
        # §1 ownership proof for the external buffer class: every external
        # byte must sit inside the surface-cache entry (its live, certified
        # owner); the cache entry itself must exist at baseline AND stage end.
        for ext, cache in zip(external_series, cache_series):
            if not isinstance(cache, int) or ext > cache:
                problems.append("external_byte_without_owner:%s>%s"
                                % (ext, cache))
                break
    if not isinstance(baseline.get("external_surface_buffer_bytes"), int):
        problems.append("baseline_external_missing")
    after_all = d.get("after") or {}

    # ---- gen4: per-object ledger no-loss proof --------------------------------
    # Every resource sample carries the monotone loss counters; the on-demand
    # tables (baseline / reset rounds / stage end) prove
    # allocated_total - released_total == live table size. A drop anywhere,
    # a failed balance, or missing counters = evidence incomplete = fail.
    ledger_stats = {"samples_with_counters": 0, "dropped_live_max": 0,
                    "table_checks": 0, "table_balance_ok": 0,
                    "end_live_views": None, "end_live_surface_buffers": None}

    def _ledger_check(snapshot, label):
        dropped = snapshot.get("ownership_dropped_live")
        allocated = snapshot.get("ownership_allocated_total")
        released = snapshot.get("ownership_released_total")
        if not all(isinstance(v, int) for v in (dropped, allocated, released)):
            return  # pre-gen4 sample: counted below as missing evidence
        ledger_stats["samples_with_counters"] += 1
        ledger_stats["dropped_live_max"] = max(
            ledger_stats["dropped_live_max"], dropped)
        table = snapshot.get("ownership_trace")
        if isinstance(table, list):
            ledger_stats["table_checks"] += 1
            live = [record for record in table
                    if isinstance(record, dict)
                    and not record.get("released_mono_ns")]
            if allocated - released == len(live):
                ledger_stats["table_balance_ok"] += 1
            else:
                problems.append("%s_ledger_balance:%d-%d!=%d"
                                % (label, allocated, released, len(live)))
            return live

    any_counter_sample = [False]

    def walk_samples(obj, label):
        if isinstance(obj, dict):
            if isinstance(obj.get("ownership_dropped_live"), int):
                any_counter_sample[0] = True
                _ledger_check(obj, label)
            for key, child in obj.items():
                if key in ("ownership_trace",):
                    continue
                walk_samples(child, label)
        elif isinstance(obj, list):
            for child in obj:
                walk_samples(child, label)

    walk_samples(d, "sample")
    if not any_counter_sample[0]:
        problems.append("ledger_counters_missing")
    if ledger_stats["dropped_live_max"]:
        problems.append("ledger_dropped_live:%d"
                        % ledger_stats["dropped_live_max"])
    # Stage-end semantics: disappearance != release -- only
    # released_mono_ns counts as released. Live VIEWS at stage end (all
    # workers joined) are a residual; a live SURFACE BUFFER is the cache
    # entry's own retained object and is legal ONLY while the cache entry
    # itself is present baseline..end (§1 live owner) -- never auto-leak,
    # never auto-legal.
    end_table = after_all.get("ownership_trace")
    if isinstance(end_table, list):
        live_views = [r for r in end_table if isinstance(r, dict)
                      and r.get("object_class") == "view"
                      and not r.get("released_mono_ns")]
        live_buffers = [r for r in end_table if isinstance(r, dict)
                        and r.get("object_class") == "surface_buffer"
                        and not r.get("released_mono_ns")]
        ledger_stats["end_live_views"] = len(live_views)
        ledger_stats["end_live_surface_buffers"] = len(live_buffers)
        if live_views:
            problems.append("ledger_end_live_views:%d" % len(live_views))
        cache_base = baseline.get("surface_cache_bytes")
        cache_end = after_all.get("surface_cache_bytes")
        if live_buffers and not (isinstance(cache_base, int)
                                 and isinstance(cache_end, int)
                                 and cache_base == cache_end and cache_end > 0):
            problems.append("ledger_end_surface_buffer_without_cache_owner")
    else:
        problems.append("ledger_end_table_missing")
    problems.extend(s3._class_residual_problems(after_all, baseline, "after_"))
    problems.extend(s3._split_identity_problems(after_all, "after_"))
    problems.extend(s3._split_identity_problems(baseline, "baseline_"))

    # ---- resources -------------------------------------------------------------
    rss = d.get("rss_series") or []
    for index, sample in enumerate(rss):
        if s3._bad_number(sample.get("rss_kib")):
            problems.append("rss_sample_bad_%d" % index)
        if not isinstance(sample.get("t_mono_ns"), int):
            problems.append("rss_time_missing_%d" % index)
        elif index and sample["t_mono_ns"] <= rss[index - 1]["t_mono_ns"]:
            problems.append("rss_time_not_monotonic_%d" % index)
    rss_stats = {}
    if rss:
        span = rss[-1]["t_mono_ns"] - rss[0]["t_mono_ns"]
        bucket_ns = int(600 * 1e9)
        counts = {}
        for sample in rss:
            counts[int((sample["t_mono_ns"] - rss[0]["t_mono_ns"])
                       // bucket_ns)] = \
                counts.get(int((sample["t_mono_ns"] - rss[0]["t_mono_ns"])
                               // bucket_ns), 0) + 1
        planned_buckets = int(contract.L_DURATION_S // 600)
        missing_buckets = [b for b in range(planned_buckets)
                           if counts.get(b, 0) < 600]
        if missing_buckets:
            problems.append("rss_bucket_missing_or_sparse:%s" % missing_buckets)
        values = [s["rss_kib"] for s in rss]
        rss_stats["rss_max_gib"] = round(max(values) / 1048576, 3)
        if max(values) > contract.L_RSS_LIMIT_KIB:
            problems.append("rss_over_limit_%d" % max(values))
        t_rss0 = rss[0]["t_mono_ns"]
        first_window = [s["rss_kib"] for s in rss
                        if t_rss0 + warmup_end - t0 <= s["t_mono_ns"]
                        < t_rss0 + warmup_end - t0 + bucket_ns]
        last_window = [s["rss_kib"] for s in rss
                       if rss[-1]["t_mono_ns"] - bucket_ns
                       <= s["t_mono_ns"] <= rss[-1]["t_mono_ns"]]
        if len(first_window) < 600 or len(last_window) < 600:
            problems.append("rss_growth_windows_thin")
        else:
            growth = 100.0 * (statistics.median(last_window)
                              - statistics.median(first_window)) \
                / max(1.0, statistics.median(first_window))
            rss_stats["rss_growth_pct"] = round(growth, 2)
            if growth > contract.L_RSS_GROWTH * 100:
                problems.append("rss_growth_%.2f%%" % growth)
        rss_stats["span_s"] = round(span / 1e9, 1)
    else:
        problems.append("rss_series_missing")
    peak = 0

    def walk(obj):
        nonlocal peak
        if isinstance(obj, dict):
            value = obj.get("read_view_cache_bytes_peak")
            if isinstance(value, int):
                peak = max(peak, value)
            for child in obj.values():
                walk(child)
        elif isinstance(obj, list):
            for child in obj:
                walk(child)
    walk(d)
    rss_stats["view_cache_peak_mib"] = round(peak / 1048576, 1)
    if peak > contract.READ_VIEW_CACHE_LIMIT_BYTES:
        problems.append("view_cache_peak:%.1fMiB" % (peak / 1048576))

    # ---- overrun rows report (judged above, never a window) --------------------
    overrun_rows = [{
        "kind": "query", "index": q.get("index"), "status": q.get("status"),
        "latency_ms": q.get("latency_ms")}
        for q in qs if q["_segment"] == "overrun"] + [{
        "kind": "input", "input_id": e.get("input_id"),
        "status": e.get("receipt_status"), "e2e_ms": e.get("e2e_ms")}
        for e in tl if e["_segment"] == "overrun"]
    return _finish(run, d, problems, rss_stats, qs, tl, rr, read_rounds,
                   binding, exempt_queries, exempt_inputs, at_reset_view,
                   overrun_rows, windows, peak, window_stats,
                   round_linearizations, ledger_stats)


def _finish(run, d, problems, rss_stats, qs, tl, rr, read_rounds, binding,
            exempt_queries=None, exempt_inputs=None, at_reset_view=None,
            overrun_rows=None, windows=None, peak=None, window_stats=None,
            round_linearizations=None, ledger_stats=None):
    exempt_queries = exempt_queries or []
    exempt_inputs = exempt_inputs or []
    verdict = {
        "rule_set": RULE_SET,
        "provenance": {
            "tool": "reset_l_judge.py (retrospective offline contract judge)",
            "note": "first committed after the run02 freeze; recomputes "
                    "preserved raw records under the frozen §5/§1 text; the "
                    "scoring-time freeze and the S3-rule view are unchanged "
                    "beside this verdict"},
        "counts": {"inputs": len(tl), "queries": len(qs or []),
                   "reset_rounds": len(rr), "read_rounds": len(read_rounds),
                   "extra_inputs": max(0, len(tl) - contract.L_INPUTS),
                   "extra_queries": max(0, len(qs or []) - contract.L_QUERIES)},
        "exemptions": {"queries": exempt_queries,
                       "inputs": exempt_inputs,
                       "at_reset_view_recorded_not_gated": at_reset_view,
                       "reset_windows": [
                           {"round": i, "called_mono_ns": c,
                            "round_end_mono_ns": e, "epoch_pre": p,
                            "epoch_post": q,
                            "linearization_mono_ns": (
                                round_linearizations or {}).get(i)}
                           for i, c, e, p, q in (windows or [])]},
        "overrun_rows": overrun_rows,
        "windows_120s": window_stats or {},
        "ledger": ledger_stats or {},
        "resources": rss_stats or {},
        "s3_rule_view": "see campaign summary.json stage 'l' (unchanged)",
    }
    verdict["problems"] = problems
    verdict["status"] = "fail" if problems else "pass"
    if run is not None:
        (run / "l_plan_verdict.json").write_text(
            json.dumps(verdict, indent=1, sort_keys=True, default=str))
    return verdict


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True)
    args = parser.parse_args()
    verdict = judge(Path(args.run))
    print(json.dumps({k: verdict[k] for k in
                      ("status", "problems", "counts", "resources",
                       "windows_120s", "exemptions", "overrun_rows")
                      if k in verdict}, indent=1, default=str))
    return 0 if verdict["status"] == "pass" else 5


if __name__ == "__main__":
    sys.exit(main())
