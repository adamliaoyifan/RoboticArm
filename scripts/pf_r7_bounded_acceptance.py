#!/usr/bin/env python3
"""PF-R7 generation-4 bounded acceptance CLI.

Default path is the live campaign. It never starts a second Gazebo world:
an occupied sim slot exits 2. ``--fixture`` runs the ROS-free harness used
by PF-R7 tests. Live refuses unless the evaluator and overlay are the same
clean commit.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path


def _workspace_root():
    return Path(__file__).resolve().parents[1]


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="PF-R7 generation-4 steady-window bounded acceptance")
    parser.add_argument(
        "--out", required=True,
        help="evidence root, typically docs/status/evidence/platform_free_height/<run>/")
    parser.add_argument("--slots", type=int, default=3)
    parser.add_argument("--eligible-per-size", type=int, default=2)
    parser.add_argument("--max-attempts-per-slot", type=int, default=12)
    parser.add_argument("--max-exclusions-per-slot", type=int, default=6)
    parser.add_argument("--max-consecutive-size-exclusions", type=int, default=3)
    parser.add_argument("--max-stack-resets", type=int, default=2)
    parser.add_argument("--max-attempts-campaign", type=int, default=36)
    parser.add_argument("--campaign-timeout-sec", type=float, default=2700)
    parser.add_argument("--ros-domain-id", type=int, default=7)
    parser.add_argument(
        "--fixture", default="",
        help="JSON fixture with records_by_size or attempts; skips Gazebo")
    parser.add_argument(
        "--git-commit", default="",
        help="exact revision recorded in the verdict; default is git HEAD")
    parser.add_argument(
        "--pidfile", default="/tmp/elfin_humble_sim.pid")
    parser.add_argument(
        "--live", action="store_true",
        help="run the Gazebo campaign (refuses if the sim slot is busy)")
    parser.add_argument(
        "--wait-slot-sec", type=float, default=0,
        help="wait up to this many seconds for the exclusive sim slot")
    parser.add_argument(
        "--overlay", default="/tmp/pfr10_g6",
        help="colcon overlay with production nodes at the PF-R10 revision")
    parser.add_argument(
        "--production-commit",
        default="60dafb7deee50a6f3a76d48076b743bf3e3e1bc8",
        help="production acceptance anchor recorded in the verdict")
    parser.add_argument(
        "--steady-start", default="support-window-ready",
        help="G4 rate-window start: support-window-ready")
    parser.add_argument(
        "--steady-window-sec", type=float, default=8.0)
    parser.add_argument(
        "--recovery-limit-sec", type=float, default=1.4)
    parser.add_argument(
        "--observe-sec", type=float, default=20.0,
        help="campaign observe cap; backend also stops at ROS window end")
    parser.add_argument(
        "--seed-matrix-json", default="",
        help="JSON seed matrix; omitted sizes keep the G4 default")
    parser.add_argument(
        "--scan-standard", action="store_true",
        help="run the G5 standard detector-availability scan instead of live")
    parser.add_argument("--scan-start", default="standard_06")
    parser.add_argument("--stop-available", type=int, default=6)
    parser.add_argument("--max-scan", type=int, default=24)
    parser.add_argument(
        "--import-scan", default="",
        help="JSON with previously classified seeds (G4 00-05)")
    parser.add_argument(
        "--import-g4", action="store_true",
        help="import built-in G4 standard_00-05 classifications")
    return parser.parse_args(argv)


def _git_head(root):
    try:
        proc = subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            capture_output=True, text=True, check=False)
        if proc.returncode == 0:
            return proc.stdout.strip()
    except OSError:
        pass
    return ""


EVALUATOR_PATHS = (
    "scripts/pf_r7_bounded_acceptance.py",
    "scripts/pf_r7_live_backend.py",
    "scripts/platform_free_height_gate4_eval.py",
    "src/luggage_perception/luggage_perception/eval/pf_r7_classifier.py",
    "src/luggage_perception/luggage_perception/eval/pf_r7_campaign.py",
    "src/luggage_perception/luggage_perception/eval/gate4_scoring.py",
    "src/luggage_gazebo/scripts/pickup_box_spawner_node.py",
    "src/luggage_msgs/msg/DetectionFrame.msg",
    "src/luggage_perception/scripts/luggage_detector_node.py",
    "src/luggage_perception/luggage_perception/platform_free_pipeline.py",
    "src/luggage_perception/luggage_perception/top_support_estimator.py",
    "scripts/pf_r7_generation3_live.sh",
    "scripts/pf_r7_generation4_live.sh",
    "scripts/pf_r7_generation5_live.sh",
    "src/luggage_perception/test/eval/test_pf_r7_classifier.py",
    "src/luggage_perception/test/eval/test_pf_r7_campaign.py",
    "src/luggage_perception/test/eval/test_pf_r7_score_window.py",
    "src/luggage_perception/test/eval/test_pf_r7_g4_steady_window.py",
    "src/luggage_perception/test/eval/test_pf_r7_g5_scan.py",
    "src/luggage_perception/test/eval/pf_r7_fixtures.py",
    "src/luggage_perception/test/eval/data/pfr7_g3_carryon00_scores.jsonl",
    "src/luggage_perception/test/eval/data/pfr7_g4_standard00_scores.jsonl",
    "src/luggage_perception/test/test_platform_free_pipeline.py",
)


def _git_dirty_paths(root, paths):
    try:
        proc = subprocess.run(
            ["git", "-C", str(root), "status", "--porcelain",
             "--untracked-files=all", "--"] + list(paths),
            capture_output=True, text=True, check=False)
    except OSError:
        return ["git_status_unavailable"]
    lines = [line for line in (proc.stdout or "").splitlines() if line.strip()]
    return lines


def _overlay_revision(overlay):
    head = _git_head(overlay)
    dirty = _git_dirty_paths(overlay, ["."])
    return head, len(dirty)


def _load_fixture(path):
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if "records_by_size" in payload:
        return payload["records_by_size"]
    attempts = payload.get("attempts") or payload.get("records") or []
    by_size = {}
    for record in attempts:
        size = str(record.get("size") or "")
        by_size.setdefault(size, []).append(record)
    return by_size


def _stop_sim(root, pidfile):
    script = root / "scripts" / "stop_sim.sh"
    if not script.is_file():
        return
    env = os.environ.copy()
    env["ELFIN_SIM_PIDFILE"] = pidfile
    subprocess.run(
        [str(script)], cwd=str(root), env=env, check=False)


def _wait_slot(pidfile, timeout_sec):
    import time
    from luggage_perception.eval.pf_r7_campaign import sim_slot_busy
    t0 = time.monotonic()
    while True:
        if not sim_slot_busy(pidfile):
            return True
        if timeout_sec <= 0 or (time.monotonic() - t0) >= float(timeout_sec):
            return False
        time.sleep(5)


def _merge_c2(out, verdict):
    summary = Path(out) / "g6s" / "g6s_summary.json"
    if not summary.is_file():
        verdict["c2_pass"] = None
        verdict["c2_missing"] = True
        if verdict.get("outcome") == "pass":
            verdict["outcome"] = "fail"
            verdict["reason"] = "eligible_fail"
            verdict["c2_failures"] = ["g6s_summary_missing"]
        return verdict
    payload = json.loads(summary.read_text(encoding="utf-8"))
    failures = payload.get("c2_failures") or []
    only_unscorable = bool(failures) and all(
        "scorable=False" in str(item) for item in failures)
    verdict["c2_pass"] = bool(payload.get("c2_pass"))
    verdict["c2_failures"] = failures
    if only_unscorable:
        verdict["c2_unscorable"] = True
        return verdict
    if verdict.get("outcome") == "pass" and not payload.get("c2_pass"):
        verdict["outcome"] = "fail"
        verdict["reason"] = "eligible_fail"
    return verdict


def main(argv=None):
    args = _parse_args(argv)
    root = _workspace_root()
    sys.path.insert(0, str(root / "src" / "luggage_perception"))
    sys.path.insert(0, str(root / "scripts"))
    from luggage_perception.eval.pf_r7_campaign import (
        CampaignDriver,
        ScriptedBackend,
        g4_imported_standard_rows,
        load_seed_matrix_json,
        sim_slot_busy,
    )

    git_commit = args.git_commit or _git_head(root)
    seed_matrix = None
    if args.seed_matrix_json:
        seed_matrix = load_seed_matrix_json(args.seed_matrix_json)
    imported = []
    if args.import_g4:
        imported = g4_imported_standard_rows()
    if args.import_scan:
        payload = json.loads(Path(args.import_scan).read_text(encoding="utf-8"))
        imported = list(payload.get("seeds") or payload or [])
    budgets = {
        "slots": args.slots,
        "eligible_per_size": args.eligible_per_size,
        "max_attempts_per_slot": args.max_attempts_per_slot,
        "max_exclusions_per_slot": args.max_exclusions_per_slot,
        "max_consecutive_size_exclusions": args.max_consecutive_size_exclusions,
        "max_stack_resets": args.max_stack_resets,
        "max_attempts_campaign": args.max_attempts_campaign,
    }
    deadlines = {"campaign_sec": float(args.campaign_timeout_sec)}
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    if args.fixture:
        backend = ScriptedBackend(records_by_size=_load_fixture(args.fixture))
        driver = CampaignDriver(
            out_dir=str(out),
            backend=backend,
            budgets=budgets,
            deadlines=deadlines,
            seed_matrix=seed_matrix,
            git_commit=git_commit,
            install_signals=True,
        )
        if args.scan_standard:
            verdict = driver.run_standard_scan(
                imported=imported,
                scan_start=args.scan_start,
                stop_available=args.stop_available,
                max_scan=args.max_scan,
            )
        else:
            verdict = driver.run()
        print(json.dumps(verdict, sort_keys=True))
        if verdict.get("outcome") == "pass":
            return 0
        if verdict.get("outcome") == "fail":
            return 1
        return 3

    if args.wait_slot_sec:
        print("waiting for sim slot (timeout %ss)" % args.wait_slot_sec,
              file=sys.stderr)
        if not _wait_slot(args.pidfile, args.wait_slot_sec):
            print("sim slot busy; refusing second world", file=sys.stderr)
            return 2

    if sim_slot_busy(args.pidfile):
        print("sim slot busy; refusing second world", file=sys.stderr)
        return 2

    if not args.live:
        print(
            "PF-R7-H1 harness is ready. Pass --fixture or --live.",
            file=sys.stderr)
        return 4

    evaluator_dirty = _git_dirty_paths(root, EVALUATOR_PATHS)
    if evaluator_dirty:
        print("evaluator dirty=1; commit G4 before live:", file=sys.stderr)
        for line in evaluator_dirty:
            print("  %s" % line, file=sys.stderr)
        return 4

    overlay_head, overlay_dirty_n = _overlay_revision(args.overlay)
    if overlay_dirty_n:
        print("overlay dirty=%s; rebuild from evaluator HEAD" % overlay_dirty_n,
              file=sys.stderr)
        return 4
    if overlay_head and git_commit and overlay_head != git_commit:
        print(
            "overlay HEAD %s != evaluator %s; same clean commit required"
            % (overlay_head, git_commit),
            file=sys.stderr)
        return 4

    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    os.environ["ELFIN_SIM_PIDFILE"] = args.pidfile
    os.environ.setdefault("YOLO_OFFLINE", "1")
    os.environ.setdefault("ULTRALYTICS_OFFLINE", "1")
    from pf_r7_live_backend import LiveTrialBackend

    def stop():
        _stop_sim(root, args.pidfile)

    stop()
    time.sleep(3)
    if sim_slot_busy(args.pidfile):
        print("sim still busy after stop_sim; refusing", file=sys.stderr)
        return 2

    scan_mode = bool(args.scan_standard)
    observe_sec = float(args.observe_sec)
    if scan_mode and observe_sec >= 20.0:
        observe_sec = 5.0
    backend = LiveTrialBackend(
        root=root,
        out_dir=out,
        overlay=args.overlay,
        pidfile=args.pidfile,
        domain=args.ros_domain_id,
        observe_sec=observe_sec,
        stop_sim=stop,
        steady_start=args.steady_start,
        steady_window_sec=float(args.steady_window_sec),
        recovery_limit_sec=float(args.recovery_limit_sec),
        scan_mode=scan_mode,
        wall_watchdog_sec=5.0 if scan_mode else None,
    )
    launched = backend.launch_stack(out / "launch.log")
    if not launched.get("ok"):
        print("launch failed: %s" % launched, file=sys.stderr)
        stop()
        return 1
    ready = backend.wait_ready()
    (out / "preflight.json").write_text(json.dumps(ready, indent=2) + "\n")
    if not ready.get("ok"):
        print("preflight failed: %s" % ready, file=sys.stderr)
        backend.teardown()
        return 1
    stop_file = "/tmp/pfr7_g3_probe_stop"
    probe_started = False
    if not scan_mode:
        backend.start_probe(out / "g6s", stop_file)
        probe_started = True
        time.sleep(20)
        deadlines["observe_sec"] = max(20.0, float(args.observe_sec))
    else:
        deadlines["observe_sec"] = max(5.0, observe_sec)
        deadlines["no_progress_sec"] = 8.0
    driver = CampaignDriver(
        out_dir=str(out),
        backend=backend,
        budgets=budgets,
        deadlines=deadlines,
        seed_matrix=seed_matrix,
        git_commit=git_commit or args.production_commit,
        stop_sim_fn=stop,
        install_signals=True,
    )
    try:
        if scan_mode:
            verdict = driver.run_standard_scan(
                imported=imported,
                scan_start=args.scan_start,
                stop_available=args.stop_available,
                max_scan=args.max_scan,
            )
        else:
            verdict = driver.run()
    finally:
        if probe_started:
            backend.stop_probe(stop_file)
        stop()
    verdict["production_commit"] = args.production_commit
    overlay_head, overlay_dirty_n = _overlay_revision(args.overlay)
    verdict["production_overlay_commit"] = overlay_head
    verdict["production_overlay_dirty"] = overlay_dirty_n
    verdict["evaluator_commit"] = git_commit
    verdict["evaluator_dirty"] = 0
    if not scan_mode:
        verdict = _merge_c2(out, verdict)
    (out / "verdict.json").write_text(
        json.dumps(verdict, indent=2, sort_keys=True) + "\n")
    print(json.dumps(verdict, sort_keys=True))
    if verdict.get("outcome") == "pass":
        return 0
    if verdict.get("outcome") == "fail":
        return 1
    return 3


if __name__ == "__main__":
    sys.exit(main())
