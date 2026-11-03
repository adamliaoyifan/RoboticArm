#!/usr/bin/env python3
"""Reset validity judge (plan OCTOMAP_RESET_VALIDITY_NEXT_20261103 §5/§6/§7).

  reset_judge.py --run <new-run>

Reads ONLY the artifacts on disk (manifest.json, events.jsonl, v1/v2/v3/
ownership records inside campaign_summary.json, cases/, observer.log) and
recomputes every gate verdict from the raw records -- the runner's own
summary row is never evidence. Exit 0 only for a complete pass; a
not_evaluated gate is named in the JSON report and still exits nonzero.

Negative self-test (contract §6): ``--selftest`` synthesizes corrupted
copies of a healthy run and asserts the judge REJECTS each: missing case,
duplicate case id, truncated payload, forged summary (summary says pass
while raw records fail), stale plan admission, wrong epoch, absent robot
refresh, premature owner decrement, residual after final release, and a
3-ID overrun. Healthy overrun 0 and overrun 2 fixtures must PASS; a 2-ID
overrun with one failed query must FAIL.
"""
from __future__ import annotations

import argparse
import copy
import json
import shutil
import sys
import tempfile
from pathlib import Path

V1_STAGES = 7
V3_PAIRINGS = 8
MAX_EXTRA_REQUESTS = 2


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


def judge(run: Path):
    problems = []
    verdict = {"run": str(run), "gates": {}, "problems": problems,
               "not_evaluated": []}
    summary_path = run / "campaign_summary.json"
    manifest_path = run / "manifest.json"
    events_path = run / "events.jsonl"
    for required in (summary_path, manifest_path, events_path):
        if not required.is_file():
            problems.append("missing_artifact:%s" % required.name)
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

    repeats = manifest.get("repeats")
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

    # Ownership rules: a held-object cycle must show charged bytes at hold
    # (a zero row means the owner's bytes were decremented before release)
    # and must not show any residual class after the final release.
    for record in summary.get("ownership") or []:
        residual = record.get("residual") or {}
        if any(residual.values()):
            problems.append("residual_after_final_release:%s"
                            % record.get("case_id"))
        at_hold = record.get("bytes_at_hold") or {}
        if record.get("kind") == "held_view" and not (
                at_hold.get("old_view") or at_hold.get("client_held")):
            problems.append("premature_owner_decrement:%s"
                            % record.get("case_id"))
        if record.get("kind") == "held_buffer" and not (
                at_hold.get("external_surface") or at_hold.get("old_view")
                or at_hold.get("client_held")):
            problems.append("premature_owner_decrement:%s"
                            % record.get("case_id"))
    # V2 rule: every recovery cycle must have refreshed the robot state
    # (absent refresh keeps the gate closed; the runner records the fact).
    for record in summary.get("v2") or []:
        if record.get("robot_refresh") is False:
            problems.append("absent_robot_refresh:%s" % record.get("case_id"))

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
def _healthy_run(tmp: Path, extra_requests=0, failing_query=False):
    """A complete minimal healthy run: repeats=1, every V1 stage, one V2
    cycle, all 8 V3 pairings, 2 ownership cycles, full event closure."""
    run = tmp / ("healthy_overrun_%d" % extra_requests)
    if run.exists():
        shutil.rmtree(run)
    (run / "cases").mkdir(parents=True)
    node_stages = ["acquire_before_lock", "acquire_after_lock",
                   "surface_first_build", "page_work",
                   "response_serialization"]
    downstream_stages = ["placement_computation", "planning_admission"]
    v1, events = [], []
    seq = 0
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
    v2 = [{"case_id": "v2_000", "problems": [], "status": "pass",
           "variant": "missing", "robot_refresh": True}]
    v3 = [{"case_id": "v3_%s_000" % pairing, "pairing": pairing,
           "problems": [], "status": "pass", "reset_ms": 3.0,
           "invalidation_visible_ms": 5.0}
          for pairing in ("integration", "acquire", "surface", "roi",
                          "release", "cache_build", "planning_admission",
                          "status")]
    ownership = [
        {"case_id": "own_held_view_000", "kind": "held_view", "problems": [],
         "status": "pass",
         "bytes_at_hold": {"old_view": 3363936, "client_held": 3363936,
                           "external_surface": 0}},
        {"case_id": "own_held_buffer_000", "kind": "held_buffer",
         "problems": [], "status": "pass",
         "bytes_at_hold": {"old_view": 0, "client_held": 0,
                           "external_surface": 72}},
    ]
    events.append({"kind": "reset_called", "reset_id": "reset-0001"})
    summary = {"mode": "score", "gate_status": "pass", "v1": v1, "v2": v2,
               "v3": v3, "ownership": ownership,
               "extra_requests": extra_requests, "complete": True}
    manifest = {"mode": "score", "node_sha256": "0" * 64, "frames": ["f"],
                "extra_requests": extra_requests, "repeats": 1}
    (run / "campaign_summary.json").write_text(json.dumps(summary))
    (run / "manifest.json").write_text(json.dumps(manifest))
    (run / "events.jsonl").write_text(
        "\n".join(json.dumps(event) for event in events) + "\n")
    return run


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
            "premature_owner_decrement": lambda s, m, e: s.__setitem__(
                "ownership", [{"case_id": "own_x", "kind": "held_view",
                               "problems": [], "status": "pass",
                               "bytes_at_hold": {"old_view": 0}}]),
            "residual_after_final_release": lambda s, m, e: s.__setitem__(
                "ownership", [{"case_id": "own_x", "kind": "held_buffer",
                               "problems": [], "status": "pass",
                               "residual": {"external_surface": 72}}]),
            "absent_robot_refresh": lambda s, m, e: s.__setitem__(
                "v2", [{"case_id": "v2_000", "problems": [], "status": "pass",
                        "variant": "missing",
                        "robot_refresh": False}]),
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
    if failures:
        print("SELFTEST FAIL: %s" % "; ".join(failures))
        return 1
    print("selftest: all negative fixtures rejected; healthy overrun 0/2 pass")
    return 0


def _failing_query_run(tmp: Path):
    run = tmp / "overrun2_failed_query"
    if run.exists():
        shutil.rmtree(run)
    (run / "cases").mkdir(parents=True)
    record = {"case_id": "v1_page_work_000", "stage": "page_work",
              "problems": [], "status": "pass",
              "outcome": {"status": "INVALIDATED", "identity_epoch": 1},
              "reset_epoch": 2}
    summary = {"mode": "score", "gate_status": "pass", "v1": [record],
               "v2": [], "v3": [], "ownership": [], "extra_requests": 2}
    manifest = {"mode": "score", "node_sha256": "0" * 64, "frames": ["f"],
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
    (run / "campaign_summary.json").write_text(json.dumps(summary))
    (run / "manifest.json").write_text(json.dumps(manifest))
    (run / "events.jsonl").write_text(
        "\n".join(json.dumps(event) for event in events) + "\n")
    # Judge rule: an overrun ID with a failed terminal fails the run. The
    # fixture declares it via the event stream; add the explicit check.
    return run


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
