#!/usr/bin/env python3
"""Run the dynamic-suction planning gates C0-C6 and record evidence.

::

    python3 -m luggage_planning.eval.suction_c_acceptance --out <dir>

Each gate maps to the pytest classes of the two required D1 entry
points (no logic is duplicated here; the suites are the authority):

======  ================================================================
C0      test_suction_candidate_planning.py: TestCandidateWaypoints,
        TestIdentityGate, TestContactModelGate, TestSessionConfigValidation
C1      TestCandidateSelection, TestSessionSelection
C2      test_suction_retry_state_machine.py: TestBoundedAttempts
C3      TestSealSuccess
C4      TestRetryTrace, TestAuditCounters, TestNoNewDetection
C5      TestRecoveryFaultInjection
C6      TestCarryBoundary, TestBoundaryTable, TestReducerTransitionGrid,
        TestReducerReleaseWindow
support TestMessageAdapters, TestWiring, TestArchitectureIsolation
C7      deferred — no-motion hardware dry run; protocol implemented as
        ``hardware_pick_driver --dry-run`` (operator session pending,
        same deferral class as ST-1 A5)
======  ================================================================

Output mirrors the ST-2 convention: ``manifest_T0.json``,
``traces_T1.jsonl``, ``summary.json``, ``RESULT.md``. Exit code 0 iff
every executed gate passes with zero failure and zero skip.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PLANNING_TEST = os.path.normpath(os.path.join(
    HERE, "..", "..", "test"))

GATES = (
    ("C0", "candidate waypoints, identity/model gates, config validation",
     "test_suction_candidate_planning.py",
     ("TestCandidateWaypoints", "TestIdentityGate",
      "TestContactModelGate", "TestSessionConfigValidation")),
    ("C1", "rank-order selection skipping failed probes",
     "test_suction_candidate_planning.py",
     ("TestCandidateSelection", "TestSessionSelection")),
    ("C2", "bounded attempts and exhaustion contract",
     "test_suction_retry_state_machine.py", ("TestBoundedAttempts",)),
    ("C3", "seal success under fake DI0 rise delays",
     "test_suction_retry_state_machine.py", ("TestSealSuccess",)),
    ("C4", "exact first-fail/second-seal trace and audit zeros",
     "test_suction_retry_state_machine.py",
     ("TestRetryTrace", "TestAuditCounters", "TestNoNewDetection")),
    ("C5", "recovery fault injection stops before lateral motion",
     "test_suction_retry_state_machine.py",
     ("TestRecoveryFaultInjection",)),
    ("C6", "pre/carry failure boundary, 100% transition coverage",
     "test_suction_retry_state_machine.py",
     ("TestCarryBoundary", "TestBoundaryTable",
      "TestReducerTransitionGrid", "TestReducerReleaseWindow")),
    ("SUPPORT", "message adapters, ROS wiring scans, architecture isolation",
     "test_suction_candidate_planning.py",
     ("TestMessageAdapters", "TestWiring", "TestArchitectureIsolation")),
)

ENV_KEYS = ("PYTHONPATH", "AMENT_PREFIX_PATH", "ROS_DISTRO")


def _git_state():
    def run(*args):
        try:
            out = subprocess.run(
                ("git",) + args, capture_output=True, text=True,
                cwd=os.path.join(HERE, "..", "..", "..", ".."))
            return out.stdout.strip()
        except Exception:   # noqa: BLE001 - evidence context only
            return ""
    return {
        "commit": run("rev-parse", "HEAD"),
        "branch": run("rev-parse", "--abbrev-ref", "HEAD"),
        "dirty": len([line for line in run("status", "--porcelain")
                      .splitlines() if line.strip()]),
    }


def _run_gate(gate_id, test_file, classes):
    test_path = os.path.join(PLANNING_TEST, test_file)
    # each class selector must attach to its own path argument
    targets = ["%s::%s" % (test_path, name) for name in classes]
    command = [sys.executable, "-m", "pytest", "-q"] + targets
    started = time.time()
    completed = subprocess.run(
        command, capture_output=True, text=True, cwd=PLANNING_TEST,
        env=dict(os.environ))
    elapsed = time.time() - started
    tail = completed.stdout.strip().splitlines()
    summary_line = tail[-1] if tail else ""
    # parse the pytest tail ("X failed, Y passed, Z skipped" variants)
    import re
    def count(label):
        match = re.search(r"(\d+) %s" % label, summary_line)
        return int(match.group(1)) if match else 0
    passed, failed, skipped = (count("passed"), count("failed"),
                               count("skipped"))
    if completed.returncode != 0 and failed == 0:
        failed = 1   # collection error or crash: never a silent pass
    ok = completed.returncode == 0 and failed == 0 and skipped == 0
    return {
        "gate": gate_id,
        "pass": ok,
        "passed": passed,
        "failed": failed,
        "skipped": skipped,
        "elapsed_sec": round(elapsed, 2),
        "command": "python3 -m pytest -q %s" % " ".join(
            os.path.relpath(target, PLANNING_TEST) for target in targets),
        "summary": summary_line,
        "returncode": completed.returncode,
        "stdout_tail": "\n".join(completed.stdout.strip().splitlines()[-12:]),
        "stderr_tail": "\n".join(completed.stderr.strip().splitlines()[-6:]),
    }


def run_acceptance(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    git = _git_state()
    results = []
    traces = open(os.path.join(out_dir, "traces_T1.jsonl"), "w",
                  encoding="utf-8")
    for gate_id, description, test_file, classes in GATES:
        result = _run_gate(gate_id, test_file, classes)
        result["description"] = description
        results.append(result)
        traces.write(json.dumps(result, sort_keys=True) + "\n")
        traces.flush()
    results.append({
        "gate": "C7",
        "pass": None,
        "status": "deferred",
        "description": "no-motion hardware dry run: real D555/TF, motion "
                       "disabled, 10 detections x 5 box positions; protocol "
                       "implemented as hardware_pick_driver --dry-run; "
                       "requires an operator session (ST-1 A5 deferral "
                       "class); INTEGRATION runs it on the consolidated "
                       "revision",
    })
    traces.write(json.dumps(results[-1], sort_keys=True) + "\n")
    traces.close()

    executed = [row for row in results if row.get("pass") is not None]
    overall = all(row["pass"] for row in executed) and bool(executed)
    summary = {"pass": overall,
               "gates": [{k: row.get(k) for k in
                          ("gate", "pass", "passed", "failed", "skipped",
                           "status", "description")}
                         for row in results]}
    with open(os.path.join(out_dir, "summary.json"), "w",
              encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
        handle.write("\n")

    manifest = {
        "run_kind": "dynamic_suction_st3_c",
        "plan": "docs/plans/dynamic_top_surface_and_suction_patch.md",
        "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "git": git,
        "environment": {key: os.environ.get(key, "")
                        for key in ENV_KEYS},
        "python": sys.version.split()[0],
        "entry_points": [
            "python3 -m pytest -q "
            "src/luggage_planning/test/test_suction_candidate_planning.py",
            "python3 -m pytest -q "
            "src/luggage_planning/test/test_suction_retry_state_machine.py",
            "python3 -m luggage_planning.eval.suction_retry_replay "
            "--case <dir> --stage vacuum",
        ],
    }
    with open(os.path.join(out_dir, "manifest_T0.json"), "w",
              encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2, sort_keys=True)
        handle.write("\n")

    lines = [
        "# ST-3 gate C evidence", "",
        "- commit: `%s` (dirty: %d)" % (git["commit"][:12], git["dirty"]),
        "- overall: **%s**" % ("pass" if overall else "FAIL"), "",
        "| gate | pass | cases | note |",
        "|---|---|---|---|",
    ]
    for row in results:
        note = row.get("status") or "%s | %s" % (
            row.get("summary", ""), row.get("description", ""))
        lines.append("| %s | %s | %s | %s |" % (
            row["gate"],
            "yes" if row.get("pass") else (
                "deferred" if row.get("pass") is None else "NO"),
            "%d/%d/%d" % (row.get("passed", 0), row.get("failed", 0),
                          row.get("skipped", 0)),
            note.replace("|", "/")))
    with open(os.path.join(out_dir, "RESULT.md"), "w",
              encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")
    return overall, results


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True,
                        help="evidence output directory")
    args = parser.parse_args(args=argv)
    overall, results = run_acceptance(args.out)
    for row in results:
        print("%-6s %s" % (row["gate"],
                           "pass" if row.get("pass") else (
                               "deferred" if row.get("pass") is None
                               else "FAIL")))
    print("overall: %s" % ("pass" if overall else "FAIL"))
    return 0 if overall else 1


if __name__ == "__main__":
    sys.exit(main())
