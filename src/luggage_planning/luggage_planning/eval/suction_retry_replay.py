#!/usr/bin/env python3
"""Replay a recorded suction retry trace through the pure reducer.

Evidence-matrix entry point (plan ``Candidate to waypoints`` /
``Vacuum/retry state`` rows)::

    python3 -m luggage_planning.eval.suction_retry_replay \\
        --case <case-dir> [--stage vacuum]

The case directory contains ``events.jsonl`` — the session trace as
written by ``hardware_pick_driver --trace-out``. Every ``retry_event``
record is re-fed through :func:`reduce_retry_event` from the initial
model; the replayed final state must equal the last recorded state and
the recomputed audit counters must match ``audit.json`` when present.
Any disagreement exits non-zero (``evidence_invalid``).
"""

from __future__ import annotations

import argparse
import json
import os
import sys

from luggage_planning.suction_retry_contracts import (
    RetryEvent,
    RetryEventType,
    audit_trace,
    initial_retry_model,
    is_terminal,
    reduce_retry_event,
)


def load_events(case_dir):
    path = os.path.join(case_dir, "events.jsonl")
    records = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def replay(records):
    """Re-feed retry_event records; return (final_model, problems)."""
    model = initial_retry_model()
    problems = []
    for index, record in enumerate(records):
        if record.get("kind") != "retry_event":
            continue
        try:
            event_type = RetryEventType(str(record.get("name", "")))
        except ValueError:
            problems.append("record %d: unknown event %r"
                            % (index, record.get("name")))
            continue
        event = RetryEvent(
            event_type=event_type,
            candidate_id=str(record.get("candidate_id", "") or ""),
            segment=str(record.get("segment", "") or ""),
            ok=bool(record.get("ok", False)),
            reason_code=str(record.get("reason_code", "") or ""),
            detail=str(record.get("detail", "") or ""),
            t=float(record.get("t", 0.0) or 0.0),
            di0=record.get("di0", None),
            fraction=float(record.get("fraction", -1.0) or 0.0),
            ranked_ids=tuple(record.get("ranked_ids", ()) or ()),
        )
        try:
            model = reduce_retry_event(model, event).model
        except Exception as error:  # noqa: BLE001 - replay must report
            problems.append("record %d (%s): reducer raised %r"
                            % (index, event_type.value, error))
            break
    return model, problems


def replay_case(case_dir, stage="vacuum"):
    del stage   # single state-machine stream; accepted for interface parity
    records = load_events(case_dir)
    model, problems = replay(records)

    recorded_final = None
    for record in records:
        if record.get("kind") == "state":
            recorded_final = record.get("name")
    if recorded_final is not None and model.state.value != recorded_final:
        problems.append(
            "final state mismatch: replayed %s, trace ended in %s"
            % (model.state.value, recorded_final))
    if not is_terminal(model.state) and not problems:
        problems.append("replay ended in non-terminal state %s"
                        % model.state.value)

    counters = audit_trace(records)
    audit_path = os.path.join(case_dir, "audit.json")
    if os.path.exists(audit_path):
        with open(audit_path, "r", encoding="utf-8") as handle:
            expected = json.load(handle)
        actual = counters.__dict__
        for key, value in expected.items():
            if key in actual and actual[key] != value:
                problems.append(
                    "audit counter %s: recomputed %s, recorded %s"
                    % (key, actual[key], value))
    violations = [key for key, value in counters.__dict__.items()
                  if value]
    if violations:
        problems.append("audit violations: %s" % ", ".join(violations))
    return model, counters, problems


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", required=True,
                        help="case directory containing events.jsonl")
    parser.add_argument("--stage", default="vacuum",
                        choices=("vacuum",),
                        help="evidence-matrix stage selector")
    args = parser.parse_args(args=argv)

    model, counters, problems = replay_case(args.case, args.stage)

    summary = {
        "case": args.case,
        "stage": args.stage,
        "final_state": model.state.value,
        "reason_code": model.reason_code,
        "detail": model.detail,
        "audit": counters.__dict__,
        "problems": problems,
        "valid": not problems,
    }
    out_path = os.path.join(args.case, "replay_summary.json")
    with open(out_path, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
        handle.write("\n")
    for problem in problems:
        print("REPLAY PROBLEM: %s" % problem, file=sys.stderr)
    print("replay %s: final_state=%s reason=%s"
          % ("ok" if not problems else "INVALID", model.state.value,
             model.reason_code or "-"))
    return 0 if not problems else 1


if __name__ == "__main__":
    sys.exit(main())
