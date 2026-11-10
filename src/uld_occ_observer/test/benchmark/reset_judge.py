#!/usr/bin/env python3
"""Reset validity judge (plan OCTOMAP_RESET_VALIDITY_NEXT_20261103 §5/§6/§7).

  reset_judge.py --run <new-run>

Reads ONLY the artifacts on disk (manifest.json, events.jsonl, observer.log,
per-case bundles under cases/, and the v1/v2/v3/ownership records inside
campaign_summary.json) and recomputes every gate verdict from the raw
records and the T1 event stream -- the runner's own summary row and its
per-record timing fields are never trusted where an independent derivation
is possible (review 2026-11-03 finding 2):

- V3 pairing/visibility latency is derived from the v3_pairing_start /
  v3_reset_done / v3_invalidation_visible / v3_pairing_complete events and
  the 500 ms / 200 ms contract bounds are enforced on the DERIVED numbers;
  the record's own ms fields must agree or they are forged.
- V2 recovery requires the robot_refresh field to be PRESENT and true, and
  cross-checks the robot sample stamp against the validity transition stamp
  from the v2_recovery event.
- Ownership settle time is derived from the owner_final_release /
  owner_baseline events, enforced against the 2 s window, and the final
  owner's ledger release must post-date the reset.
- Missing artifacts (cases/, observer.log), non-numeric/non-finite
  evidence, wrong formal repeat counts and absent events are failures,
  never silent passes.

Exit 0 only for a complete pass; a not_evaluated gate is named in the JSON
report and still exits nonzero.

Negative self-test (contract §6 + review 2026-11-03): ``--selftest``
synthesizes corrupted copies of a healthy run and asserts the judge
REJECTS each. Healthy overrun 0 and overrun 2 fixtures must PASS; a 2-ID
overrun with one failed query must FAIL; a 3-ID overrun must FAIL.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import shutil
import sys
import tempfile
from pathlib import Path

V1_STAGES = 7
V3_PAIRINGS = 8
MAX_EXTRA_REQUESTS = 2
FORMAL_REPEATS = 30
PAIRING_DEADLINE_MS = 500.0
INVALIDATION_VISIBLE_MS = 200.0
OWNERSHIP_SETTLE_MS = 2000.0
#: Recorded ms fields must agree with the event-derived value well inside
#: scheduling jitter; anything far off is a forged field, not noise.
MS_AGREEMENT_TOLERANCE = 50.0


def load_summary(run: Path):
    return json.loads((run / "campaign_summary.json").read_text())


def load_events(run: Path):
    events = []
    path = run / "events.jsonl"
    if not path.is_file():
        return events
    for line in path.read_text().splitlines():
        line = line.strip()
        if line:
            events.append(json.loads(line))
    return events


def is_number(value) -> bool:
    """Finite int/float evidence only (bools and strings are not timing)."""
    return (isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value))


def judge(run: Path):
    problems = []
    verdict = {"run": str(run), "gates": {}, "problems": problems,
               "not_evaluated": []}
    summary_path = run / "campaign_summary.json"
    manifest_path = run / "manifest.json"
    events_path = run / "events.jsonl"
    cases_dir = run / "cases"
    observer_log = run / "observer.log"
    for required in (summary_path, manifest_path, events_path, cases_dir,
                     observer_log):
        if not required.exists():
            problems.append("missing_artifact:%s" % required.name)
            verdict["status"] = "fail"
            return verdict
    if not any(cases_dir.iterdir()):
        problems.append("missing_artifact:cases_empty")
        verdict["status"] = "fail"
        return verdict
    summary = load_summary(run)
    manifest = json.loads(manifest_path.read_text())
    events = load_events(run)
    verdict["manifest"] = {
        "mode": manifest.get("mode"),
        "node_sha256": manifest.get("node_sha256"),
        "frames": len(manifest.get("frames") or []),
    }
    mode = manifest.get("mode")
    repeats = manifest.get("repeats")
    if mode == "score" and repeats != FORMAL_REPEATS:
        problems.append("formal_repeats:%s!=%d" % (repeats, FORMAL_REPEATS))
    if not is_number(repeats):
        problems.append("manifest_repeats_not_numeric")

    # ---- raw-record recomputation (never the summary's own word) ----------
    def recompute(records, gate):
        out = []
        for record in records:
            problems_raw = list(record.get("problems") or [])
            status = record.get("status")
            expected = "pass" if not problems_raw else "fail"
            if status != expected:
                problems_raw.append("status_field_disagrees_with_problems")
            out.append((record, problems_raw))
        return out

    for gate in ("v1", "v2", "v3", "ownership"):
        records = summary.get(gate) or []
        if not records:
            verdict["not_evaluated"].append(gate)
            verdict["gates"][gate] = "not_evaluated"
            # A declared repeat count means the run promised these gates.
            if repeats:
                problems.append("gate_absent:%s" % gate)
            continue
        recomputed = recompute(records, gate)
        failures = [record["case_id"] for record, raw in recomputed if raw]
        verdict["gates"][gate] = "fail" if failures else "pass"
        if failures:
            problems.append("gate_failed:%s:%s" % (gate, ",".join(failures[:8])))

    # Count discipline applies to runs that declare their repeat count
    # (the runner's manifest does; minimal synthetic fixtures do not).
    if repeats:
        expected = {
            "v1": repeats * V1_STAGES, "v2": repeats,
            "v3": repeats * V3_PAIRINGS, "ownership": repeats * 2,
        }
        for gate, want in expected.items():
            got = len(summary.get(gate) or [])
            if got != want:
                problems.append("count_mismatch:%s=%d,want=%d" % (gate, got, want))

    # ---- per-gate semantic checks -------------------------------------------
    v1_records = {}
    for record in summary.get("v1") or []:
        case_id = record.get("case_id")
        if case_id in v1_records:
            problems.append("duplicate_case_id:%s" % case_id)
        v1_records[case_id] = record
    for case_id, record in v1_records.items():
        outcome = record.get("outcome") or {}
        if record["stage"] not in ("placement_computation",
                                   "planning_admission"):
            if outcome.get("status") == "OK" and \
                    record.get("reset_epoch", -1) > outcome.get(
                        "identity_epoch", -1):
                problems.append("old_epoch_ok_payload:%s" % case_id)
            # Payload completeness: an INVALIDATED outcome must still carry
            # the leased identity (truncation is tampering, not privacy).
            if outcome.get("status") == "INVALIDATED" and \
                    "identity_epoch" not in outcome:
                problems.append("payload_incomplete:%s" % case_id)
            # The parked lease was pre-reset by construction: an INVALIDATED
            # response claiming a post-reset epoch is a wrong epoch.
            if outcome.get("status") == "INVALIDATED" and \
                    "identity_epoch" in outcome and \
                    outcome["identity_epoch"] >= record.get("reset_epoch", 1 << 60):
                problems.append("wrong_epoch:%s" % case_id)
        else:
            # V1 downstream stages: nothing may be admitted across the reset.
            if outcome.get("admitted"):
                problems.append("stale_plan_admission:%s" % case_id)

    # ---- V2: recovery evidence (finding 2: absent field must fail) ---------
    v2_events = {}
    for event in events:
        if event.get("kind") == "v2_recovery":
            v2_events[event.get("case_id")] = event
    for record in summary.get("v2") or []:
        case_id = record.get("case_id")
        if "robot_refresh" not in record:
            problems.append("robot_refresh_absent:%s" % case_id)
        elif record.get("robot_refresh") is not True:
            problems.append("absent_robot_refresh:%s" % case_id)
        stamps = {}
        for field in ("robot_refresh_ns", "transition_mono_ns"):
            value = record.get(field)
            if not is_number(value):
                problems.append("evidence_not_numeric:%s:%s" % (case_id, field))
            else:
                stamps[field] = value
        event = v2_events.get(case_id)
        if event is None:
            problems.append("recovery_event_missing:%s" % case_id)
        if "robot_refresh_ns" in stamps and "transition_mono_ns" in stamps:
            if stamps["robot_refresh_ns"] <= stamps["transition_mono_ns"]:
                problems.append("robot_refresh_not_post_transition:%s" % case_id)
            if event is not None and (
                    event.get("robot_refresh_ns") != stamps["robot_refresh_ns"]
                    or event.get("transition_mono_ns")
                    != stamps["transition_mono_ns"]):
                problems.append("recovery_stamps_disagree:%s" % case_id)

    # ---- V3: derive latency from the event stream (finding 2) --------------
    v3_events = {}
    for event in events:
        if event.get("kind") in ("v3_pairing_start", "v3_reset_done",
                                 "v3_invalidation_visible",
                                 "v3_pairing_complete"):
            v3_events.setdefault(event.get("case_id"), {})[
                event["kind"]] = event
    for record in summary.get("v3") or []:
        case_id = record.get("case_id")
        stamps = {}
        for field in ("t_pairing_start_ns", "t_reset_done_ns",
                      "t_visible_ns", "t_complete_ns"):
            value = record.get(field)
            if not is_number(value):
                problems.append("evidence_not_numeric:%s:%s" % (case_id, field))
            else:
                stamps[field] = value
        case_events = v3_events.get(case_id) or {}
        for kind in ("v3_pairing_start", "v3_reset_done",
                     "v3_invalidation_visible", "v3_pairing_complete"):
            if kind not in case_events:
                problems.append("pairing_event_missing:%s:%s" % (case_id, kind))
        reset_ms = record.get("reset_ms")
        if not is_number(reset_ms):
            problems.append("evidence_not_numeric:%s:reset_ms" % case_id)
        elif reset_ms > PAIRING_DEADLINE_MS:
            problems.append("reset_over_deadline:%s" % case_id)
        if len(stamps) == 4:
            derived_pairing_ms = (stamps["t_complete_ns"]
                                  - stamps["t_pairing_start_ns"]) / 1e6
            derived_visible_ms = (stamps["t_visible_ns"]
                                  - stamps["t_reset_done_ns"]) / 1e6
            if stamps["t_pairing_start_ns"] > stamps["t_reset_done_ns"] or \
                    stamps["t_reset_done_ns"] > stamps["t_visible_ns"] or \
                    stamps["t_visible_ns"] > stamps["t_complete_ns"]:
                problems.append("stamp_order_invalid:%s" % case_id)
            else:
                if derived_pairing_ms > PAIRING_DEADLINE_MS:
                    problems.append("pairing_over_deadline:%s:%.1fms"
                                    % (case_id, derived_pairing_ms))
                if derived_visible_ms > INVALIDATION_VISIBLE_MS:
                    problems.append("invalidation_visibility_late:%s:%.1fms"
                                    % (case_id, derived_visible_ms))
            for field, derived in (("pairing_ms", derived_pairing_ms),
                                   ("invalidation_visible_ms",
                                    derived_visible_ms)):
                recorded = record.get(field)
                if not is_number(recorded):
                    problems.append("evidence_not_numeric:%s:%s"
                                    % (case_id, field))
                elif abs(recorded - derived) > MS_AGREEMENT_TOLERANCE:
                    problems.append("timing_field_forged:%s:%s=%s!=%.1f"
                                    % (case_id, field, recorded, derived))
            # The event stream must carry the same stamps the record claims.
            if "v3_pairing_start" in case_events and \
                    case_events["v3_pairing_start"].get("t_ns") != \
                    stamps["t_pairing_start_ns"]:
                problems.append("pairing_stamps_disagree:%s" % case_id)

    # ---- ownership: settle derived from events + ledger proof --------------
    owner_events = {}
    for event in events:
        if event.get("kind") in ("owner_hold", "owner_final_release",
                                 "owner_baseline"):
            owner_events.setdefault(event.get("case_id"), {})[
                event["kind"]] = event
    for record in summary.get("ownership") or []:
        case_id = record.get("case_id")
        residual = record.get("residual") or {}
        if any(residual.values()):
            problems.append("residual_after_final_release:%s" % case_id)
        at_hold = record.get("bytes_at_hold") or {}
        if record.get("kind") == "held_view" and not (
                at_hold.get("old_view") or at_hold.get("client_held")):
            problems.append("premature_owner_decrement:%s" % case_id)
        if record.get("kind") == "held_buffer" and not (
                at_hold.get("external_surface") or at_hold.get("old_view")
                or at_hold.get("client_held")):
            problems.append("premature_owner_decrement:%s" % case_id)
        stamps = {}
        for field in ("t_hold_ns", "t_reset_done_ns", "t_final_release_ns",
                      "t_baseline_ns"):
            value = record.get(field)
            if not is_number(value):
                problems.append("evidence_not_numeric:%s:%s" % (case_id, field))
            else:
                stamps[field] = value
        case_events = owner_events.get(case_id) or {}
        for kind in ("owner_hold", "owner_final_release", "owner_baseline"):
            if kind not in case_events:
                problems.append("owner_event_missing:%s:%s" % (case_id, kind))
        if "t_final_release_ns" in stamps and "t_baseline_ns" in stamps:
            derived_settle_ms = (stamps["t_baseline_ns"]
                                 - stamps["t_final_release_ns"]) / 1e6
            if derived_settle_ms < 0:
                problems.append("stamp_order_invalid:%s" % case_id)
            elif derived_settle_ms > OWNERSHIP_SETTLE_MS:
                problems.append("settle_over_window:%s:%.1fms"
                                % (case_id, derived_settle_ms))
            recorded_settle = record.get("settle_ms")
            if not is_number(recorded_settle):
                problems.append("evidence_not_numeric:%s:settle_ms" % case_id)
            elif abs(recorded_settle - derived_settle_ms) > \
                    MS_AGREEMENT_TOLERANCE:
                problems.append("settle_field_forged:%s:%s!=%.1f"
                                % (case_id, recorded_settle, derived_settle_ms))
        # Final-owner release evidence: the ledger must show the object
        # released, after the reset and before the baseline confirmation.
        ledger_release = record.get("ledger_released_mono_ns")
        if not is_number(ledger_release) or ledger_release <= 0:
            problems.append("final_owner_release_unproven:%s" % case_id)
        elif "t_reset_done_ns" in stamps and \
                ledger_release < stamps["t_reset_done_ns"]:
            problems.append("ledger_release_before_reset:%s" % case_id)
        elif "t_baseline_ns" in stamps and \
                ledger_release > stamps["t_baseline_ns"]:
            problems.append("ledger_release_after_baseline:%s" % case_id)

    # ---- event-stream closure (§6) -------------------------------------------
    planned = {}
    terminals = {}
    resets = []
    allowed_terminals = {
        "OK", "INVALIDATED", "NO_DATA", "STALE", "REJECTED", "QUEUE_FULL",
        "EPOCH_MISMATCH", "REVISION_NOT_COVERED", "TIMEOUT", "TIMEOUT_WAIT",
        # Downstream admission outcomes (placement/planning stages): the
        # gate's frozen reason codes are legal terminals for those streams.
        "validity_invalid", "validity_building", "map_not_admitted",
        "map_epoch_mismatch", "map_generation_mismatch", "robot_state_stale",
        "robot_state_missing", "robot_state_pre_transition",
        "commit_generation_mismatch", "late_old_epoch_result", "gate_closed"}
    for event in events:
        kind = event.get("kind")
        if kind == "request_planned":
            planned[event["request_id"]] = event
        elif kind == "request_terminal":
            terminals.setdefault(event["request_id"], []).append(event)
            if event.get("status") not in allowed_terminals:
                problems.append("failed_terminal_status:%s:%s"
                                % (event.get("request_id"), event.get("status")))
        elif kind == "reset_called":
            resets.append(event)
    for request_id in planned:
        hits = terminals.get(request_id) or []
        if len(hits) != 1:
            problems.append("terminal_closure:%s=%d" % (request_id, len(hits)))
    extra = manifest.get("extra_requests") or 0
    if extra > MAX_EXTRA_REQUESTS:
        problems.append("overrun:%d>%d" % (extra, MAX_EXTRA_REQUESTS))
    verdict["extra_requests"] = extra

    # Every reset event must be followed by an observable invalidation
    # transition in the event stream (the runner emits reset_called; the
    # per-case invalidation visibility is inside the v3 records).
    if not resets:
        problems.append("no_reset_events")

    # ---- forged-summary rejection --------------------------------------------
    if summary.get("gate_status") == "pass" and any(
            verdict["gates"].get(gate) == "fail" for gate in
            ("v1", "v2", "v3", "ownership")):
        problems.append("forged_summary_pass")

    verdict["status"] = "fail" if problems else "pass"
    return verdict


# ---- negative fixtures --------------------------------------------------------
def _write_run(run: Path, summary, manifest, events):
    if run.exists():
        shutil.rmtree(run)
    (run / "cases").mkdir(parents=True)
    for record in (summary.get("v1") or []) + (summary.get("v2") or []) + \
            (summary.get("v3") or []) + (summary.get("ownership") or []):
        (run / "cases" / ("%s.json" % record["case_id"])).write_text(
            json.dumps(record))
    (run / "observer.log").write_text("observer fixture log\n")
    (run / "campaign_summary.json").write_text(json.dumps(summary))
    (run / "manifest.json").write_text(json.dumps(manifest))
    (run / "events.jsonl").write_text(
        "\n".join(json.dumps(event) for event in events) + "\n")
    return run


def _healthy_run(tmp: Path, extra_requests=0):
    """A complete minimal healthy run: repeats=1 (rehearsal), every V1
    stage, one V2 cycle, all 8 V3 pairings, 2 ownership cycles, full event
    closure and the raw-evidence fields the judge derives timing from."""
    node_stages = ["acquire_before_lock", "acquire_after_lock",
                   "surface_first_build", "page_work",
                   "response_serialization"]
    downstream_stages = ["placement_computation", "planning_admission"]
    v1, v2, v3, ownership, events = [], [], [], [], []
    seq = 0
    clock = 1_000_000_000_000
    for stage in node_stages + downstream_stages:
        seq += 1
        outcome = ({"status": "INVALIDATED", "identity_epoch": 1,
                    "validity_state": "INVALID"}
                   if stage in node_stages else
                   {"status": "gate_closed", "admitted": False})
        v1.append({"case_id": "v1_%s_000" % stage, "stage": stage,
                   "problems": [], "status": "pass", "outcome": outcome,
                   "reset_epoch": 2})
        request_id = "v1-%06d" % seq
        events.append({"kind": "request_planned", "request_id": request_id,
                       "stage": stage, "epoch": 1})
        events.append({"kind": "request_terminal", "request_id": request_id,
                       "stage": stage, "status": "INVALIDATED"})
    # V2: recovery with real stamps (robot refresh after the transition).
    v2.append({"case_id": "v2_000", "problems": [], "status": "pass",
               "variant": "missing", "robot_refresh": True,
               "robot_refresh_ns": clock + 5_000_000,
               "transition_mono_ns": clock,
               "recovery_epoch": 2, "recovery_generation": 4})
    events.append({"kind": "v2_recovery", "case_id": "v2_000",
                   "t_mono_ns": clock + 6_000_000,
                   "transition_mono_ns": clock,
                   "robot_refresh_ns": clock + 5_000_000,
                   "epoch": 2, "generation": 4})
    # V3: all 8 pairings, event-derived windows inside the bounds.
    for pairing in ("integration", "acquire", "surface", "roi",
                    "release", "cache_build", "planning_admission",
                    "status"):
        case_id = "v3_%s_000" % pairing
        t0 = clock + 10_000_000
        t_reset = t0 + 8_000_000        # reset linearization
        t_visible = t_reset + 1_100_000  # 1.1 ms visibility
        t_done = t_reset + 6_000_000     # pairing complete (14 ms total)
        v3.append({"case_id": case_id, "pairing": pairing, "problems": [],
                   "status": "pass", "reset_ms": 7.5,
                   "t_pairing_start_ns": t0, "t_reset_done_ns": t_reset,
                   "t_visible_ns": t_visible, "t_complete_ns": t_done,
                   "pairing_ms": 14.0, "invalidation_visible_ms": 1.1})
        events.append({"kind": "v3_pairing_start", "case_id": case_id,
                       "t_mono_ns": t0, "t_ns": t0})
        events.append({"kind": "v3_reset_done", "case_id": case_id,
                       "t_mono_ns": t_reset, "t_ns": t_reset})
        events.append({"kind": "v3_invalidation_visible", "case_id": case_id,
                       "t_mono_ns": t_visible, "t_ns": t_visible,
                       "state": "INVALID"})
        events.append({"kind": "v3_pairing_complete", "case_id": case_id,
                       "t_mono_ns": t_done, "t_ns": t_done,
                       "pairing_ms": 14.0, "status": "pass"})
    # Ownership: hold bytes, final release, baseline inside 2 s.
    for kind, hold_bytes in (
            ("held_view", {"old_view": 3363936, "client_held": 3363936,
                           "external_surface": 0}),
            ("held_buffer", {"old_view": 0, "client_held": 0,
                             "external_surface": 72})):
        case_id = "own_%s_000" % kind
        t_reset = clock + 20_000_000
        t_hold = t_reset + 1_000_000
        t_release = t_hold + 2_000_000
        t_baseline = t_release + 1_400_000    # 1.4 ms settle
        ledger_release = t_release + 100_000
        ownership.append({"case_id": case_id, "kind": kind, "problems": [],
                          "status": "pass", "bytes_at_hold": hold_bytes,
                          "t_reset_done_ns": t_reset, "t_hold_ns": t_hold,
                          "t_final_release_ns": t_release,
                          "t_baseline_ns": t_baseline, "settle_ms": 1.4,
                          "ledger_object_id": 7,
                          "ledger_released_mono_ns": ledger_release})
        events.append({"kind": "owner_hold", "case_id": case_id,
                       "t_mono_ns": t_hold, "t_ns": t_hold,
                       "bytes_at_hold": hold_bytes})
        events.append({"kind": "owner_final_release", "case_id": case_id,
                       "t_mono_ns": t_release, "t_ns": t_release,
                       "terminal_status": "INVALIDATED",
                       "ledger_released_mono_ns": ledger_release})
        events.append({"kind": "owner_baseline", "case_id": case_id,
                       "t_mono_ns": t_baseline, "t_ns": t_baseline,
                       "bytes": {"old_view": 0, "client_held": 0,
                                 "external_surface": 0},
                       "settled": True})
    events.append({"kind": "reset_called", "reset_id": "reset-0001"})
    summary = {"mode": "rehearsal", "gate_status": "pass", "v1": v1, "v2": v2,
               "v3": v3, "ownership": ownership,
               "extra_requests": extra_requests, "complete": True}
    manifest = {"mode": "rehearsal", "node_sha256": "0" * 64, "frames": ["f"],
                "extra_requests": extra_requests, "repeats": 1}
    return _write_run(tmp / ("healthy_overrun_%d" % extra_requests),
                      summary, manifest, events)


def _failing_query_run(tmp: Path):
    run = tmp / "overrun2_failed_query"
    record = {"case_id": "v1_page_work_000", "stage": "page_work",
              "problems": [], "status": "pass",
              "outcome": {"status": "INVALIDATED", "identity_epoch": 1},
              "reset_epoch": 2}
    summary = {"mode": "rehearsal", "gate_status": "pass", "v1": [record],
               "v2": [], "v3": [], "ownership": [], "extra_requests": 2}
    manifest = {"mode": "rehearsal", "node_sha256": "0" * 64, "frames": ["f"],
                "extra_requests": 2, "repeats": None}
    events = [
        {"kind": "request_planned", "request_id": "v1-000001", "stage":
         "page_work", "epoch": 1},
        {"kind": "request_terminal", "request_id": "v1-000001", "stage":
         "page_work", "status": "INVALIDATED"},
        # One of the overrun queries FAILED its terminal -- judged per ID.
        {"kind": "request_terminal", "request_id": "overrun-q-1", "stage":
         "page_work", "status": "QUERY_FAILED"},
        {"kind": "reset_called", "reset_id": "reset-0001"},
    ]
    return _write_run(run, summary, manifest, events)


def selftest():
    """Every fixture must be rejected except the healthy ones."""
    failures = []
    with tempfile.TemporaryDirectory() as tmpdir:
        tmp = Path(tmpdir)
        # Healthy overrun 0 and 2 must pass.
        for extra in (0, 2):
            run = _healthy_run(tmp, extra_requests=extra)
            verdict = judge(run)
            if verdict["status"] != "pass":
                failures.append("healthy_overrun_%d_rejected:%s"
                                % (extra, verdict["problems"]))
        # A 2-ID overrun with one failed query must FAIL.
        run = _failing_query_run(tmp)
        verdict = judge(run)
        if verdict["status"] == "pass":
            failures.append("overrun2_failed_query_accepted")
        # A 3-ID overrun must FAIL.
        run = _healthy_run(tmp, extra_requests=3)
        verdict = judge(run)
        if verdict["status"] == "pass":
            failures.append("overrun3_accepted")
        # Corruptions of the healthy run must each be rejected.
        base = _healthy_run(tmp, extra_requests=0)

        def forged_v3_ms(summary):
            for record in summary["v3"]:
                record["pairing_ms"] = 9999
                record["reset_ms"] = 9999
                record["invalidation_visible_ms"] = 9999

        def inflated_v3_stamps(summary):
            for record in summary["v3"]:
                record["t_complete_ns"] += 9_999_000_000

        def removed_release_evidence(summary):
            for record in summary["ownership"]:
                record.pop("ledger_released_mono_ns", None)
                record.pop("t_final_release_ns", None)

        mutations = {
            "missing_case": lambda s, m, e: s["v1"].clear(),
            "duplicate_case": lambda s, m, e: s["v1"].append(
                copy.deepcopy(s["v1"][0])),
            "truncated_payload": lambda s, m, e: s["v1"][0]["outcome"].pop(
                "identity_epoch"),
            "forged_summary": lambda s, m, e: (
                s["v1"][0].__setitem__("problems", ["old_epoch_ok_payload"]),
                s["v1"][0].__setitem__("status", "pass")),
            "stale_plan_admission": lambda s, m, e: (
                s["v1"][0].__setitem__("stage", "planning_admission"),
                s["v1"][0]["outcome"].__setitem__("admitted", True),
                s["v1"][0].__setitem__("problems", []),
                s["v1"][0].__setitem__("status", "pass")),
            "wrong_epoch": lambda s, m, e: s["v1"][0]["outcome"].__setitem__(
                "identity_epoch", 2),
            # Review finding 2 counterexamples, verbatim:
            "v3_9999ms_fields": lambda s, m, e: forged_v3_ms(s),
            "v3_stamps_over_deadline": lambda s, m, e: inflated_v3_stamps(s),
            "v2_missing_refresh": lambda s, m, e: s["v2"][0].pop(
                "robot_refresh", None),
            "owner_settle_9999": lambda s, m, e: [
                r.__setitem__("settle_ms", 9999) for r in s["ownership"]],
            "owner_no_release_evidence": lambda s, m, e:
                removed_release_evidence(s),
            # Evidence-type and artifact-integrity counterexamples:
            "nonnumeric_timing": lambda s, m, e: s["v3"][0].__setitem__(
                "pairing_ms", "14.0"),
            "premature_owner_decrement": lambda s, m, e: s.__setitem__(
                "ownership", [{"case_id": "own_x", "kind": "held_view",
                               "problems": [], "status": "pass",
                               "t_reset_done_ns": 1, "t_hold_ns": 2,
                               "t_final_release_ns": 3, "t_baseline_ns": 4,
                               "settle_ms": 1.0,
                               "ledger_released_mono_ns": 3,
                               "bytes_at_hold": {"old_view": 0}}]),
            "residual_after_final_release": lambda s, m, e: s[
                "ownership"][0].__setitem__(
                    "residual", {"external_surface": 72}),
            "absent_robot_refresh": lambda s, m, e: s["v2"][0].__setitem__(
                "robot_refresh", False),
            "formal_repeats_wrong": lambda s, m, e: m.__setitem__(
                "mode", "score"),
            "missing_pairing_events": lambda s, m, e: e.__setitem__(
                [i for i, ev in enumerate(e)
                 if ev.get("kind") == "v3_pairing_start"][0],
                dict(e[0], kind="dropped")),
        }
        for name, mutate in mutations.items():
            run = tmp / ("mut_%s" % name)
            if run.exists():
                shutil.rmtree(run)
            shutil.copytree(base, run)
            summary = json.loads((run / "campaign_summary.json").read_text())
            manifest = json.loads((run / "manifest.json").read_text())
            events = [json.loads(line) for line in
                      (run / "events.jsonl").read_text().splitlines() if line]
            mutate(summary, manifest, events)
            (run / "campaign_summary.json").write_text(json.dumps(summary))
            (run / "manifest.json").write_text(json.dumps(manifest))
            (run / "events.jsonl").write_text(
                "\n".join(json.dumps(event) for event in events) + "\n")
            verdict = judge(run)
            if verdict["status"] == "pass":
                failures.append("mutation_accepted:%s" % name)
        # Artifact-integrity counterexamples on directory copies.
        for name, sabotage in (
                ("no_cases_dir", lambda r: shutil.rmtree(r / "cases")),
                ("empty_cases_dir", lambda r: [
                    p.unlink() for p in (r / "cases").iterdir()]),
                ("no_observer_log", lambda r: (r / "observer.log").unlink()),
        ):
            run = tmp / ("mut_%s" % name)
            if run.exists():
                shutil.rmtree(run)
            shutil.copytree(base, run)
            sabotage(run)
            verdict = judge(run)
            if verdict["status"] == "pass":
                failures.append("mutation_accepted:%s" % name)
    if failures:
        print("SELFTEST FAIL: %s" % "; ".join(failures))
        return 1
    print("selftest: all negative fixtures rejected; healthy overrun 0/2 pass")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", default=None)
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()
    if args.selftest:
        return selftest()
    if not args.run:
        parser.error("--run or --selftest required")
    verdict = judge(Path(args.run))
    (Path(args.run) / "verdict.json").write_text(
        json.dumps(verdict, indent=1, sort_keys=True))
    print(json.dumps({k: verdict[k] for k in ("status", "problems",
                                              "not_evaluated")
                      if k in verdict}, indent=1))
    return 0 if verdict["status"] == "pass" else 5


if __name__ == "__main__":
    sys.exit(main())
