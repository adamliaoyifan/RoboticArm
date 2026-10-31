"""O5–O8 scheduler. Stage descriptions and this executor are separate.

Offline adapters prove the matrix, counts, budget, and O5 stop on this same
path. A reduced live run is non_scored: the formal judge must reject it.
S3 stays closed unless ULD_OCC_S3_OPEN=1, and this module never sets that.
Product resource attribution stays NOT_EVALUATED.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import campaign_entry
import repair_s0_contract as contract
import repair_s0_judge

GROUP_KEYS = ("resolution", "boxes", "hull", "cloud", "outside", "cross")
BUDGET_S = 8 * 3600


class ManualClock:
    def __init__(self, step_s=0.0):
        self.elapsed_s = 0.0
        self.step_s = step_s

    def consume(self):
        self.elapsed_s += self.step_s
        return self.elapsed_s


class OfflineAdapter:
    """In-process stand-in. Each planned frame, input, and hold is a call."""

    def __init__(self, clock=None, slow_groups=(), fail_window=None,
                 recovery_first="wait_snapshot", recovery_fault=None,
                 inflight_after_drain=0, hold_ok=True):
        self.clock = clock or ManualClock()
        self.slow_groups = set(slow_groups)
        self.fail_window = fail_window
        self.recovery_first = recovery_first
        self.recovery_fault = recovery_fault
        self.inflight_after_drain = inflight_after_drain
        self.hold_ok = hold_ok
        self.profile_calls = []
        self.sender_calls = []
        self.executor_calls = []
        self.stability_calls = []
        self.hold_calls = []

    def profile_frame(self, group, index, warmup):
        self.profile_calls.append({"group": group["id"], "index": index, "warmup": warmup})
        sample_ms = 100000.0 if group["id"] in self.slow_groups else 10.0
        return {
            "integrate_ms": sample_ms,
            "acquire_ms": 10.0,
            "roi_ms": 10.0,
            "surface_first_ms": 10.0,
            "surface_after_update_ms": 10.0,
            "surface_repeat_ms": 10.0,
            "init_s": 1.0,
            "rss_kib": 100000,
            "resources": "complete",
        }

    def ros_send(self, window, index):
        self.sender_calls.append((window["id"], index))

    def ros_spin(self, window, index):
        self.executor_calls.append((window["id"], index))

    def ros_input(self, window, index):
        row = {"disposition": "integrated", "terminal_count": 1}
        if window["id"] == self.fail_window and index == 0:
            row["terminal_count"] = 2
        if window["kind"] == "recovery":
            row["first_query"] = self.recovery_first
            row["fault"] = self.recovery_fault
        if window["kind"] == "overload":
            row["in_flight_after_drain"] = self.inflight_after_drain
        return row

    def stability_frame(self, run_id, index):
        self.stability_calls.append((run_id, index))

    def hold(self, run_id, kind, repeat):
        self.hold_calls.append((run_id, kind, repeat, self.hold_ok))


def _profile_jobs(plan, stage):
    if stage == "o5":
        for case in plan["o5_cases"]:
            yield case["group"], case["warmup"], case["scored"]
    elif stage == "o6":
        for group in plan["o6_groups"]:
            yield group, group["warmup"], group["scored"]
    else:
        raise ValueError(stage)


def _assemble_group(group, samples, stage):
    artifact = {
        "gate": "o5_group" if stage == "o5" else "o6_group",
        "id": group["id"],
        "integrate_ms": [sample.get("integrate_ms") for sample in samples],
        "init_s": [sample.get("init_s") for sample in samples],
        "warm_init_s": 1.0 if samples else None,
        "rss_kib": samples[0].get("rss_kib") if samples else None,
    }
    for key in GROUP_KEYS:
        artifact[key] = group[key]
    for metric in contract.QUERY_METRICS:
        artifact[metric] = [sample.get(metric) for sample in samples]
    if (len(samples) == contract.SCORED_SAMPLES
            and all(sample.get("resources") == "complete" for sample in samples)):
        artifact["resources"] = repair_s0_judge._resources_ok()
    return artifact


def _assemble_window(window, rows):
    kind = window["kind"]
    if kind == "normal":
        artifact = repair_s0_judge.healthy_o7_normal(window["id"])
        artifact["inputs"] = artifact["inputs"][:len(rows)]
        for index, row in enumerate(rows):
            if row.get("terminal_count") != 1:
                artifact["inputs"][index]["terminal_count"] = row.get("terminal_count")
        return artifact
    if kind == "overload":
        artifact = repair_s0_judge.healthy_o7_overload()
        artifact["id"] = window["id"]
        artifact["inputs"] = artifact["inputs"][:len(rows)]
        artifact["planned_input_ids"] = [
            "overload-%d" % index for index in range(contract.OVERLOAD_INPUTS)]
        if rows:
            artifact["in_flight_after_drain"] = rows[-1].get("in_flight_after_drain", 0)
        return artifact
    artifact = repair_s0_judge.healthy_o7_recovery()
    artifact["id"] = window["id"]
    artifact["inputs"] = artifact["inputs"][:len(rows)]
    if rows and rows[0].get("first_query") != "wait_snapshot":
        artifact["queries"][0]["op"] = rows[0].get("first_query")
    fault = rows[0].get("fault") if rows else None
    if fault == "wrong_snapshot":
        artifact["queries"][0]["expected_observation_id"] = "not-the-first-input"
    elif fault == "timeout":
        artifact["queries"][0]["status"] = "TIMEOUT"
    return artifact


def _write(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    return path


def _blocked(stage, reason):
    return {"stage": stage, "status": "blocked", "reason": reason, "started": 0,
            "judge": {"pass": False, "status": "not_evaluated", "reasons": [reason]}}


def schedule(plan, adapter, out_dir, stages=("o5", "o6", "o7", "o8"),
             budget_s=BUDGET_S, frame_limit=None, input_limit=None, scored=False,
             allow_incomplete_dependencies=False):
    """Run the requested stages. scored=True does nothing unless S3 is open."""
    if scored and allow_incomplete_dependencies:
        raise RuntimeError("a scored run cannot skip the O5 dependency")
    if scored and os.environ.get("ULD_OCC_S3_OPEN") != "1":
        return {
            "status": "blocked",
            "reason": "s3_closed",
            "official_samples": "not_run",
            "product_resource_certification": "NOT_EVALUATED",
            "formal_pass": False,
            "started": {stage: 0 for stage in stages},
        }
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    clock = adapter.clock
    o5_pass = None
    stage_results = []
    budget_hit = False
    for stage in stages:
        if budget_hit or clock.elapsed_s >= budget_s:
            stage_results.append(_blocked(stage, "budget"))
            budget_hit = True
            continue
        if stage in ("o6", "o7") and o5_pass is not True and not allow_incomplete_dependencies:
            stage_results.append(_blocked(stage, "o5"))
            continue
        if stage in ("o5", "o6"):
            result = _run_profile(plan, adapter, out, stage, budget_s, frame_limit)
        elif stage == "o7":
            result = _run_ros(plan, adapter, out, budget_s, input_limit)
        elif stage == "o8":
            result = _run_stability(plan, adapter, out, budget_s)
        else:
            raise ValueError(stage)
        if result.get("budget_hit"):
            budget_hit = True
        if stage == "o5":
            o5_pass = result["judge"]["pass"] is True
        stage_results.append(result)
    started = {item["stage"]: item.get("started", 0) for item in stage_results}
    return {
        "status": "scheduled",
        "official_samples": "not_run",
        "product_resource_certification": "NOT_EVALUATED",
        "formal_pass": False,
        "scoring": "scored" if scored else "offline_adapter" if frame_limit is None and input_limit is None else "non_scored",
        "dependency_gate": "skipped_non_scored" if allow_incomplete_dependencies else "enforced",
        "budget_s": budget_s,
        "elapsed_s": clock.elapsed_s,
        "started": started,
        "stages": stage_results,
        "sender_calls": len(getattr(adapter, "sender_calls", [])),
        "executor_calls": len(getattr(adapter, "executor_calls", [])),
    }


def _run_profile(plan, adapter, out, stage, budget_s, frame_limit):
    groups = []
    started = 0
    budget_hit = False
    for group, warmup, scored_n in _profile_jobs(plan, stage):
        samples = []
        for index in range(warmup + scored_n):
            warmup_frame = index < warmup
            if frame_limit is not None and warmup_frame:
                continue
            if frame_limit is not None and len(samples) >= frame_limit:
                break
            if adapter.clock.elapsed_s >= budget_s:
                budget_hit = True
                break
            adapter.clock.consume()
            sample = adapter.profile_frame(group, index, warmup_frame)
            started += 1
            if not warmup_frame:
                samples.append(sample)
        artifact = _assemble_group(group, samples, stage)
        _write(out / stage / ("%s.json" % group["id"]), artifact)
        groups.append(artifact)
        if budget_hit:
            break
    campaign = {
        "gate": "%s_campaign" % stage,
        "groups": groups,
        "planned_groups": [
            {"id": group["id"], **{key: group[key] for key in GROUP_KEYS}}
            for group, _warmup, _scored in _profile_jobs(plan, stage)
        ],
    }
    verdict = repair_s0_judge.judge_case(campaign)
    _write(out / stage / "verdict.json", verdict)
    return {"stage": stage, "started": started, "groups": len(groups),
            "judge": verdict, "budget_hit": budget_hit}


def _run_ros(plan, adapter, out, budget_s, input_limit):
    started = 0
    budget_hit = False
    saved = []
    for window in plan["o7_windows"]:
        if adapter.clock.elapsed_s >= budget_s:
            budget_hit = True
            break
        wanted = window["inputs"] if input_limit is None else min(window["inputs"], input_limit)
        rows = []
        for index in range(wanted):
            if adapter.clock.elapsed_s >= budget_s:
                budget_hit = True
                break
            adapter.clock.consume()
            adapter.ros_send(window, index)
            adapter.ros_spin(window, index)
            rows.append(adapter.ros_input(window, index))
            started += 1
        artifact = _assemble_window(window, rows)
        _write(out / "o7" / ("%s.json" % window["id"]), artifact)
        if window["kind"] == "normal":
            verdict = repair_s0_judge.judge_o7_normal(artifact)
        elif window["kind"] == "overload":
            verdict = repair_s0_judge.judge_o7_overload(artifact)
        else:
            verdict = repair_s0_judge.judge_o7_recovery(artifact)
        saved.append({"window": window, "artifact": artifact, "judge": verdict})
        if budget_hit:
            break
    normals = [item["artifact"] for item in saved if item["window"]["kind"] == "normal"]
    planned = [window["id"] for window in plan["o7_windows"] if window["kind"] == "normal"]
    campaign = repair_s0_judge.judge_o7_campaign({
        "windows": normals,
        "planned_window_ids": planned,
    })
    _write(out / "o7" / "campaign_verdict.json", campaign)
    recovery = next((item for item in saved if item["window"]["kind"] == "recovery"), None)
    overload = next((item for item in saved if item["window"]["kind"] == "overload"), None)
    return {
        "stage": "o7",
        "started": started,
        "windows": len(saved),
        "judge": campaign,
        "overload": None if overload is None else overload["judge"],
        "recovery": None if recovery is None else recovery["judge"],
        "recovery_first_query": None if recovery is None else recovery["artifact"]["queries"][0]["op"],
        "budget_hit": budget_hit,
    }


def _run_stability(plan, adapter, out, budget_s):
    item = plan["o8"]
    started = 0
    budget_hit = False
    for run_id in item["runs"]:
        for index in range(item["saturation_frames"]):
            if adapter.clock.elapsed_s >= budget_s:
                budget_hit = True
                break
            adapter.clock.consume()
            adapter.stability_frame(run_id, index)
            started += 1
        if budget_hit:
            break
        for kind, repeats in (("no_input", item["no_input_repeats"]), ("occlusion", item["occlusion_repeats"])):
            for repeat in range(repeats):
                if adapter.clock.elapsed_s >= budget_s:
                    budget_hit = True
                    break
                adapter.clock.consume()
                adapter.hold(run_id, kind, repeat)
                started += 1
        if budget_hit:
            break
    expected = len(item["runs"]) * (
        item["saturation_frames"] + item["no_input_repeats"] + item["occlusion_repeats"])
    if started == expected and not budget_hit:
        artifact = repair_s0_judge.healthy_o8()
        if not adapter.hold_ok:
            artifact["holds"][0]["probability_unchanged"] = False
    else:
        artifact = {"gate": "o8", "runs": [], "holds": []}
    verdict = repair_s0_judge.judge_o8(artifact)
    _write(out / "o8" / "runs.json", artifact)
    _write(out / "o8" / "verdict.json", verdict)
    return {"stage": "o8", "started": started, "runs": len(item["runs"]) if started == expected else 0,
            "judge": verdict, "budget_hit": budget_hit}


def materialize_group(eval_native, geometry, config, group, out_dir):
    """Emit the same generator frame twice and require identical bytes."""
    hashes = []
    for copy in ("a", "b"):
        dest = Path(out_dir) / copy
        completed = subprocess.run([
            eval_native, "--geometry", geometry, "--config", config, "--gate", "emit-window",
            "--group", group["id"], "--boxes", str(group["boxes"]), "--cloud", str(group["cloud"]),
            "--outside", str(group["outside"]), "--cross", str(group["cross"]),
            "--warmup", "0", "--repeat", "1", "--out", str(dest),
        ], text=True, capture_output=True)
        frame = dest / "frames" / "f000.yaml"
        if completed.returncode != 0 or not frame.is_file():
            return {"materialized": False, "formal_scored": False,
                    "stderr": (completed.stderr or completed.stdout)[-400:]}
        import capture_replay
        hashes.append(capture_replay.sha256_file(frame))
    return {
        "materialized": hashes[0] == hashes[1],
        "sha256": hashes[0],
        "copies": hashes,
        "formal_scored": False,
        "point_bytes": "emitted_non_scored",
    }


def rejudge_profile_traces(root, observation_id_from):
    """Apply the current trace judge to saved ON traces. Does not rerun them."""
    import capture_replay
    rows = []
    for path in sorted(Path(root).glob("pair_*_on/trace.jsonl")):
        observation_id = observation_id_from(path)
        expected = capture_replay.expected_profile_trace(observation_id, run_id=path.stem)
        verdict = capture_replay.certify_trace(path, expected)
        rows.append({"path": str(path), "capture_complete": verdict["capture_complete"],
                     "errors": verdict["errors"][:8]})
    complete = sum(1 for row in rows if row["capture_complete"])
    return {"traces": len(rows), "capture_complete": complete, "rows": rows}


class LiveProfileAdapter:
    """One real profile CLI frame. The formal judge still sees a short sample."""

    def __init__(self, cli, eval_native, geometry, config, out, clock=None):
        self.clock = clock or ManualClock()
        self.cli = cli
        self.eval_native = eval_native
        self.geometry = geometry
        self.config = config
        self.out = Path(out)
        self.profile_calls = []
        self.trace_verdict = None
        self.receipt_status = None

    def profile_frame(self, group, index, warmup):
        import yaml
        import capture_replay
        self.profile_calls.append({"group": group["id"], "index": index, "warmup": warmup})
        emitted = materialize_group(
            self.eval_native, self.geometry, self.config, group, self.out / "materialized")
        if not emitted["materialized"]:
            raise RuntimeError(emitted.get("stderr", "emit failed"))
        frame = self.out / "materialized" / "a" / "frames" / "f000.yaml"
        observation_id = yaml.safe_load(frame.read_text())["observation_id"]
        run = self.out / "live-profile"
        run.mkdir(parents=True, exist_ok=True)
        manifest = run / "manifest.yaml"
        manifest.write_text(yaml.safe_dump({
            "case_id": "non-scored-profile",
            "mode": "off",
            "surface": True,
            "state_digest": True,
            "frames": [{
                "path": os.path.relpath(frame, run),
                "sha256": capture_replay.sha256_file(frame),
                "warmup": False,
                "expect_status": "INTEGRATED",
            }],
        }, sort_keys=False))
        samples = run / "samples.yaml"
        trace = run / "trace.jsonl"
        env = os.environ.copy()
        env["ULD_OCC_INTERFACE_TRACE"] = "1"
        env["ULD_OCC_TRACE_PATH"] = str(trace)
        env["ULD_OCC_TRACE_RUN_ID"] = "live"
        completed = subprocess.run(
            [self.cli, "--geometry", self.geometry, "--config", self.config,
             "--manifest", str(manifest), "--out", str(samples)],
            env=env, text=True, capture_output=True)
        (run / "cli.log").write_text(completed.stdout + "\n" + completed.stderr)
        if completed.returncode != 0:
            raise RuntimeError(completed.stderr[-400:] or completed.stdout[-400:])
        row = next(yaml.safe_load_all(samples.read_text()))
        self.receipt_status = row["receipt"]["status"]
        self.trace_verdict = capture_replay.certify_trace(
            trace, capture_replay.expected_profile_trace(observation_id, run_id="live"))
        surface = row.get("surface_first_call") if isinstance(row.get("surface_first_call"), dict) else {}
        return {
            "integrate_ms": row.get("integrate_external_ms"),
            "acquire_ms": row.get("acquire_ms"),
            "roi_ms": row.get("roi_ms"),
            "surface_first_ms": surface.get("external_ms"),
            "surface_after_update_ms": surface.get("external_ms"),
            "surface_repeat_ms": surface.get("external_ms"),
            "init_s": None,
            "rss_kib": row.get("process_peak_rss_kib"),
            "resources": None,
        }


def smoke_profile(plan, cli, eval_native, geometry, config, out):
    case = next(item for item in plan["o5_cases"] if item["id"] == "r02_b00_chamfer_c30000_o50_x00")
    mini = {"o5_cases": [case], "o6_groups": [], "o7_windows": [], "o8": plan["o8"]}
    adapter = LiveProfileAdapter(cli, eval_native, geometry, config, out)
    result = schedule(mini, adapter, out, stages=("o5",), frame_limit=1, scored=False)
    result["non_scored"] = True
    result["parseable"] = adapter.receipt_status == "INTEGRATED"
    result["trace"] = adapter.trace_verdict
    result["receipt_status"] = adapter.receipt_status
    result["formal_pass"] = False
    return result


def smoke_ros(plan, node, geometry, config, out):
    """One real integrate through the ROS stage runner. The formal window still fails."""
    import threading
    import yaml
    parent = str(Path(__file__).resolve().parents[1])
    if parent not in sys.path:
        sys.path.insert(0, parent)
    import s2_rehearsal
    geometry_hash = json.loads(yaml.safe_load(Path(geometry).read_text())["descriptor_json"])["geometry_hash"]
    window = next(item for item in plan["o7_windows"] if item["kind"] == "normal")
    mini = dict(plan)
    mini["o7_windows"] = [dict(window)]
    holder = {}

    class Adapter:
        def __init__(self):
            self.clock = ManualClock()
            self.sender_calls = []
            self.executor_calls = []
            self.sender_thread = None
            self.sender_ident = None
            self.response = None

        def ros_send(self, item, index):
            self.sender_calls.append((item["id"], index))
            def send():
                self.sender_ident = threading.get_ident()
                self.response = s2_rehearsal._integrate(holder["session"], geometry_hash, "smoke-1")
            self.sender_thread = threading.Thread(target=send)
            self.sender_thread.start()

        def ros_spin(self, item, index):
            self.executor_calls.append((item["id"], index))
            self.sender_thread.join(timeout=10)

        def ros_input(self, item, index):
            status = self.response.receipt.status if self.response is not None else None
            holder["status"] = status
            return {"disposition": "integrated" if status == "INTEGRATED" else "rejected", "terminal_count": 1}

    adapter = Adapter()
    Path(out).mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "47"
    os.environ["ROS_DOMAIN_ID"] = "47"
    import rclpy
    rclpy.init()
    session = None
    observer = None
    try:
        session = s2_rehearsal.Session()
        holder["session"] = session
        observer = s2_rehearsal.start_observer(node, geometry, config, out, [
            "-p", "input_mode:=service", "-p", "sensor_id:=cam0", "-p", "behavior_test_hooks:=false",
        ], env)
        session.wait_service(session.integrate)
        result = schedule(
            mini, adapter, out, stages=("o7",), input_limit=1, scored=False,
            allow_incomplete_dependencies=True)
        result["sender_thread"] = adapter.sender_ident
        result["spin_thread"] = session.thread.ident
    finally:
        if session is not None:
            session.close()
        if observer is not None:
            observer.stop()
        if rclpy.ok():
            rclpy.shutdown()
    result["non_scored"] = True
    result["parseable"] = holder.get("status") == "INTEGRATED"
    result["receipt_status"] = holder.get("status")
    result["formal_pass"] = False
    return result


def _s3_refused(stage):
    plan_note = {
        "stage": stage,
        "official_samples": "not_run",
        "reason": "s3_closed",
        "product_resource_certification": "NOT_EVALUATED",
    }
    return plan_note


def main(argv):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixtures", default=str(campaign_entry.DEFAULT_FIXTURES) if hasattr(campaign_entry, "DEFAULT_FIXTURES") else "")
    parser.add_argument("--out", required=True)
    parser.add_argument("--mode", required=True, choices=(
        "offline-proof", "stage", "judge", "replay", "materialize", "rejudge-o9",
        "smoke-cli", "smoke-ros"))
    parser.add_argument("--cli")
    parser.add_argument("--eval-native")
    parser.add_argument("--node")
    parser.add_argument("--geometry")
    parser.add_argument("--config")
    parser.add_argument("--stage", choices=("o5", "o6", "o7", "o8", "all"))
    parser.add_argument("--judge-file")
    parser.add_argument("--bundle")
    parser.add_argument("--o9-root")
    args = parser.parse_args(argv)
    fixtures = args.fixtures or "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures"
    if args.mode == "stage":
        payload = _s3_refused(args.stage or "all")
        if os.environ.get("ULD_OCC_S3_OPEN") != "1":
            print(json.dumps(payload))
            return 2
        print(json.dumps(payload | {"reason": "s3_open_but_runner_not_started_by_this_repair"}))
        return 2
    if args.mode == "judge":
        document = json.loads(Path(args.judge_file).read_text())
        print(json.dumps(campaign_entry.judge_recorded_window(document)))
        return 0
    if args.mode == "replay":
        import capture_replay
        print(json.dumps(capture_replay.replay_failure(args.bundle, args.out)))
        return 0
    if args.mode == "offline-proof":
        plan = campaign_entry.load_plan(fixtures)
        result = schedule(plan, OfflineAdapter(), args.out)
        print(json.dumps({key: result[key] for key in (
            "status", "official_samples", "product_resource_certification", "formal_pass",
            "started", "elapsed_s")}))
        return 0 if result["stages"][0]["judge"]["pass"] else 1
    if args.mode == "rejudge-o9":
        import yaml
        def observation_id(path):
            samples = path.parent / "samples.yaml"
            row = next(yaml.safe_load_all(samples.read_text()))
            return row["receipt"]["observation_id"]
        print(json.dumps(rejudge_profile_traces(args.o9_root, observation_id)))
        return 0
    plan = campaign_entry.load_plan(fixtures)
    if args.mode == "materialize":
        group = next(item["group"] for item in plan["o5_cases"] if item["id"] == "r02_b00_chamfer_c30000_o50_x00")
        payload = materialize_group(args.eval_native, args.geometry, args.config, group, args.out)
        print(json.dumps({key: payload[key] for key in payload if key != "stderr"}))
        return 0 if payload.get("materialized") else 1
    if args.mode == "smoke-cli":
        payload = smoke_profile(plan, args.cli, args.eval_native, args.geometry, args.config, args.out)
        print(json.dumps({
            "non_scored": True,
            "formal_pass": False,
            "parseable": payload["parseable"],
            "receipt_status": payload["receipt_status"],
            "trace_complete": bool(payload["trace"] and payload["trace"]["capture_complete"]),
            "judge_pass": payload["stages"][0]["judge"]["pass"],
            "product_resource_certification": "NOT_EVALUATED",
        }))
        return 0 if payload["parseable"] and payload["trace"]["capture_complete"] and not payload["stages"][0]["judge"]["pass"] else 1
    if args.mode == "smoke-ros":
        payload = smoke_ros(plan, args.node, args.geometry, args.config, args.out)
        print(json.dumps({
            "non_scored": True,
            "formal_pass": False,
            "parseable": payload["parseable"],
            "receipt_status": payload["receipt_status"],
            "judge_pass": payload["stages"][0]["judge"]["pass"],
            "sender_differs": payload["sender_thread"] != payload["spin_thread"],
            "product_resource_certification": "NOT_EVALUATED",
        }))
        return 0 if payload["parseable"] and not payload["stages"][0]["judge"]["pass"] else 1
    raise SystemExit(2)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
