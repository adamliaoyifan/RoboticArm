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
import time
from pathlib import Path

import campaign_entry
import repair_s0_contract as contract
import repair_s0_judge

GROUP_KEYS = ("resolution", "boxes", "hull", "cloud", "outside", "cross")
BUDGET_S = 8 * 3600


def _field(obj, name):
    if obj is None:
        return None
    if isinstance(obj, dict):
        return obj.get(name)
    return getattr(obj, name, None)


def interpret_observation_status(response, expected, subscribed_receipt=False):
    """Ledger status is COMPLETED. The integrate result is receipt.status.

    receipt_received is true only when an UpdateReceipt subscription observed
    the same identity. A status query does not count as that subscription.
    """
    top = _field(response, "status")
    receipt = _field(response, "receipt")
    inner = _field(receipt, "status")
    identity = _field(receipt, "identity")
    epoch = _field(identity, "epoch") if identity is not None else _field(receipt, "epoch")
    sensor = _field(receipt, "sensor_id")
    observation = _field(receipt, "observation_id")
    identity_ok = (
        sensor == expected.get("sensor_id") and observation == expected.get("observation_id")
        and (expected.get("epoch") is None or epoch in (None, expected.get("epoch")))
    )
    if top == "NOT_SEEN":
        disposition = "NOT_SEEN"
    elif top == "EXPIRED":
        disposition = "EXPIRED"
    elif top == "REJECTED":
        disposition = "REJECTED"
    elif not identity_ok:
        disposition = "identity_mismatch"
    elif top == "COMPLETED" and inner == "INTEGRATED":
        disposition = "integrated"
    else:
        disposition = "unknown"
    return {
        "disposition": disposition,
        "status_query": top,
        "receipt_status": inner,
        "identity_ok": identity_ok and disposition == "integrated",
        "receipt_received": bool(subscribed_receipt),
    }


def launch_process(argv, **kwargs):
    """The only process-start boundary for a live runner."""
    kwargs.setdefault("text", True)
    kwargs.setdefault("capture_output", True)
    return subprocess.run(argv, **kwargs)


class ManualClock:
    def __init__(self, step_s=0.0):
        self.elapsed_s = 0.0
        self.step_s = step_s

    def consume(self):
        self.elapsed_s += self.step_s
        return self.elapsed_s

    def advance(self, delta):
        self.elapsed_s += delta
        return self.elapsed_s

    def remaining(self, budget_s):
        return max(0.0, budget_s - self.elapsed_s)


class MonotonicClock:
    """Wall clock for a real runner. advance() does not invent virtual time."""

    def __init__(self):
        self._t0 = time.monotonic()

    @property
    def elapsed_s(self):
        return time.monotonic() - self._t0

    def consume(self):
        return self.elapsed_s

    def advance(self, delta):
        del delta
        return self.elapsed_s

    def remaining(self, budget_s):
        return max(0.0, budget_s - self.elapsed_s)


def _phase(window):
    return {"normal": "score", "overload": "overload", "recovery": "recovery"}[window["kind"]]


def _input_id(window, index):
    return "%s-%d" % (_phase(window), index)


def _health_times(duration):
    step = 0.5
    return [index * step for index in range(int(duration / step) + 1)]


def _protocol_resources():
    """Offline protocol fixture. The assembler never builds this."""
    return {
        "rss_kib": 100000,
        "read_view_bytes": 1024,
        "cache_bytes": 1024,
        "attribution_complete": True,
        "unmarked_allocations": False,
    }


class OfflineAdapter:
    """Protocol fixture. It returns observations; the assembler only copies them."""

    def __init__(self, clock=None, slow_groups=(), fail_window=None,
                 recovery_first="wait_snapshot", recovery_fault=None,
                 inflight_after_drain=0, hold_ok=True, input_disposition="integrated",
                 emit_queries=True, drop_health=False, omit_resources=False,
                 duplicate_input_id=False, rss_growth=False, node_stable=True):
        self.clock = clock or ManualClock()
        self.slow_groups = set(slow_groups)
        self.fail_window = fail_window
        self.recovery_first = recovery_first
        self.recovery_fault = recovery_fault
        self.inflight_after_drain = inflight_after_drain
        self.hold_ok = hold_ok
        self.input_disposition = input_disposition
        self.emit_queries = emit_queries
        self.drop_health = drop_health
        self.omit_resources = omit_resources
        self.duplicate_input_id = duplicate_input_id
        self.rss_growth = rss_growth
        self.node_stable = node_stable
        self.profile_calls = []
        self.sender_calls = []
        self.executor_calls = []
        self.query_calls = []
        self.stability_calls = []
        self.hold_calls = []
        self.order = []

    def profile_frame(self, group, index, warmup, dest):
        self.profile_calls.append({
            "group": group["id"], "index": index, "warmup": warmup,
            "cold_start": not warmup, "dest": str(dest),
        })
        sample = {
            "integrate_ms": 100000.0 if group["id"] in self.slow_groups else 10.0,
            "acquire_ms": 10.0,
            "roi_ms": 10.0,
            "surface_first_ms": 10.0,
            "surface_after_update_ms": 12.0,
            "surface_repeat_ms": 8.0,
            "init_s": 1.0,
            "warm_init_s": 1.0,
            "rss_kib": 100000,
        }
        if not self.omit_resources:
            sample["resources"] = _protocol_resources()
        return sample

    def ros_send(self, window, index, input_id, deadline=None, actual=None):
        planned = self.clock.elapsed_s if deadline is None else deadline
        actual_s = planned if actual is None else actual
        self.sender_calls.append((window["id"], index))
        self.order.append(("send", window["id"], index, planned, actual_s))

    def ros_collect(self, window, index, input_id, deadline=None):
        stamp = self.clock.elapsed_s if deadline is None else deadline
        self.executor_calls.append((window["id"], index))
        self.order.append(("collect", window["id"], index, stamp))
        phase = _phase(window)
        disposition = self.input_disposition
        if (window["kind"] == "overload" and disposition == "integrated"
                and index == window["inputs"] - 1):
            disposition = "rejected"
        if window["id"] == self.fail_window and index == 0 and disposition == "integrated":
            terminal = 2
        else:
            terminal = 1
        observed_id = _input_id(window, 0) if self.duplicate_input_id else input_id
        row = {
            "input_id": observed_id,
            "phase": phase,
            "publish_end_ns": 1000 + index,
            "callback_enter_ns": 1100 + index,
            "disposition": disposition,
            "terminal_count": terminal,
            "queue_age_ms": 0.2,
            "observed_revision": index + 1,
        }
        if disposition == "rejected":
            row["reject_reason"] = "QUEUE_FULL"
            row["receipt_received"] = False
        elif disposition == "unknown":
            row["receipt_received"] = False
            row["integrate_ms"] = 100000.0
        else:
            row["reject_reason"] = None
            row["receipt_received"] = True
            row["integrate_ms"] = 40.0
        return row

    def ros_query(self, window, index, deadline=None):
        stamp = self.clock.elapsed_s if deadline is None else deadline
        self.query_calls.append((window["id"], index, stamp))
        if not self.emit_queries:
            return None
        op = "immediate"
        status = "OK"
        latency = 20.0
        age = 20.0
        expected = None
        if window["kind"] == "recovery" and index == 0:
            op = self.recovery_first
            latency = 300.0
            age = 300.0
            expected = _input_id(window, 0)
            if self.recovery_fault == "wrong_snapshot":
                expected = "not-the-first-input"
            if self.recovery_fault == "timeout":
                status = "TIMEOUT"
            if self.recovery_fault == "stale":
                status = "STALE"
        return {
            "query_id": "q-%d" % index,
            "op": op,
            "status": status,
            "latency_ms": latency,
            "age_ms": age,
            "covers_observation": True,
            "expected_observation_id": expected,
            "acquire_ms": 5.0,
            "roi_ms": 8.0,
            "surface_ms": 15.0,
            "epoch": 1,
            "observed_revision": index + 1,
            "content_hash": "chain-%d" % index,
        }

    def ros_health(self, window):
        if self.drop_health or window["kind"] != "normal":
            return None
        return _health_times(window["duration_s"])

    def ros_drain(self, window, deadline=None):
        self.drain_time = self.clock.elapsed_s if deadline is None else deadline
        return self.inflight_after_drain

    def stability_frame(self, run_id, index):
        self.stability_calls.append((run_id, index))
        return {"run_id": run_id, "index": index}

    def stability_run(self, run_id):
        medians = [1000, 1000, 2000] if self.rss_growth else [1000, 1000, 1040]
        return {
            "run_id": run_id,
            "saturation_probability_keys": 0,
            "saturation_geometry_blocks": 0,
            "node_count_stable": self.node_stable,
            "rss_median_kib": medians,
        }

    def hold(self, run_id, kind, repeat):
        self.hold_calls.append((run_id, kind, repeat, self.hold_ok))
        return {
            "kind": kind,
            "probability_unchanged": self.hold_ok is True,
            "freshness_advanced": False,
            "duration_s": 10,
        }


def _profile_jobs(plan, stage):
    if stage == "o5":
        for case in plan["o5_cases"]:
            yield case["group"], case["warmup"], case["scored"]
    elif stage == "o6":
        for group in plan["o6_groups"]:
            yield group, group["warmup"], group["scored"]
    else:
        raise ValueError(stage)


def _column(samples, key):
    if not samples or any(key not in sample for sample in samples):
        return None
    return [sample[key] for sample in samples]


def _assemble_group(group, samples, stage):
    """Copy measured fields. Missing keys stay missing."""
    artifact = {"gate": "o5_group" if stage == "o5" else "o6_group", "id": group["id"]}
    for key in GROUP_KEYS:
        artifact[key] = group[key]
    for key in ("integrate_ms", "init_s", *contract.QUERY_METRICS):
        column = _column(samples, key)
        if column is not None:
            artifact[key] = column
    if samples and all("warm_init_s" in sample for sample in samples):
        artifact["warm_init_s"] = samples[0]["warm_init_s"]
    rss = _column(samples, "rss_kib")
    if rss is not None:
        artifact["rss_kib"] = rss[0]
    if samples and all(isinstance(sample.get("resources"), dict) for sample in samples):
        artifact["resources"] = samples[0]["resources"]
    return artifact


def _assemble_window(window, rows, queries, health, inflight):
    """Copy adapter rows and queries. This does not invent a healthy window."""
    artifact = {
        "id": window["id"],
        "inputs": [dict(row) for row in rows],
        "queries": [dict(query) for query in queries],
        "planned_input_ids": [_input_id(window, index) for index in range(window["inputs"])],
    }
    if window["kind"] == "normal":
        artifact["gate"] = "o7_normal"
        artifact["planned_query_ids"] = ["q-%d" % index for index in range(window["queries"])]
        if health is not None:
            artifact["health_times_s"] = health
    elif window["kind"] == "overload":
        artifact["gate"] = "o7_overload"
        if inflight is not None:
            artifact["in_flight_after_drain"] = inflight
    else:
        artifact["gate"] = "o7_recovery"
        artifact["planned_query_ids"] = ["q-%d" % index for index in range(window["queries"])]
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
            dest = out / stage / group["id"] / ("f%03d" % index)
            dest.mkdir(parents=True, exist_ok=True)
            sample = adapter.profile_frame(group, index, warmup_frame, dest)
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


def _advance(clock, budget_s, delta):
    if delta < 0:
        delta = 0.0
    if delta > 0 and clock.elapsed_s + delta >= budget_s:
        return False
    clock.advance(delta)
    return True


def _window_events(window, input_count, query_count):
    events = [("input", index, index / float(window["input_hz"])) for index in range(input_count)]
    if query_count:
        query_hz = float(window.get("query_hz") or 1)
        events.extend(("query", index, index / query_hz) for index in range(query_count))
    events.sort(key=lambda item: (item[2], 0 if item[0] == "input" else 1))
    return events


def _run_ros(plan, adapter, out, budget_s, input_limit):
    started = 0
    budget_hit = False
    saved = []
    for window in plan["o7_windows"]:
        if adapter.clock.elapsed_s >= budget_s:
            budget_hit = True
            break
        input_count = window["inputs"] if input_limit is None else min(window["inputs"], input_limit)
        full_queries = window.get("queries") or 0
        query_count = full_queries if input_limit is None else min(full_queries, input_limit)
        origin = adapter.clock.elapsed_s
        input_hz = float(window["input_hz"])
        sent = []
        # Send deadlines are fixed when the window opens. A query may advance
        # the clock afterwards; that duration is not added to the next send.
        for index in range(input_count):
            deadline = origin + index / input_hz
            if deadline > origin and adapter.clock.elapsed_s + (deadline - origin) >= budget_s:
                budget_hit = True
                break
            adapter.ros_send(window, index, _input_id(window, index), deadline, deadline)
            sent.append(index)
            started += 1
        queries = []
        if not budget_hit:
            query_hz = float(window.get("query_hz") or 1)
            for index in range(query_count):
                deadline = origin + index / query_hz
                if adapter.clock.elapsed_s < deadline:
                    adapter.clock.advance(deadline - adapter.clock.elapsed_s)
                query = adapter.ros_query(window, index)
                if query is not None:
                    queries.append(query)
        last_send = origin if not sent else origin + (sent[-1]) / input_hz
        inflight = None
        if not budget_hit and window["kind"] == "overload":
            drain_at = last_send + float(window.get("drain_s") or 0)
            if _advance(adapter.clock, budget_s, max(0.0, drain_at - adapter.clock.elapsed_s)):
                inflight = adapter.ros_drain(window, drain_at)
            else:
                budget_hit = True
        elif not budget_hit:
            planned_end = origin + float(window["duration_s"])
            _advance(adapter.clock, budget_s, max(0.0, planned_end - adapter.clock.elapsed_s))
        collect_at = origin + float(window["duration_s"])
        rows = [adapter.ros_collect(window, index, _input_id(window, index), collect_at) for index in sent]
        health = adapter.ros_health(window)
        artifact = _assemble_window(window, rows, queries, health, inflight)
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
    recovery_queries = [] if recovery is None else recovery["artifact"].get("queries") or []
    return {
        "stage": "o7",
        "started": started,
        "windows": len(saved),
        "judge": campaign,
        "overload": None if overload is None else overload["judge"],
        "recovery": None if recovery is None else recovery["judge"],
        "recovery_first_query": recovery_queries[0].get("op") if recovery_queries else None,
        "budget_hit": budget_hit,
    }


def _run_stability(plan, adapter, out, budget_s):
    item = plan["o8"]
    started = 0
    budget_hit = False
    runs = []
    for run_id in item["runs"]:
        origin = adapter.clock.elapsed_s
        frames = item["saturation_frames"]
        for index in range(frames):
            mark = origin + (index + 1) * (item["duration_s"] / frames)
            if not _advance(adapter.clock, budget_s, mark - adapter.clock.elapsed_s):
                budget_hit = True
                break
            adapter.stability_frame(run_id, index)
            started += 1
        if budget_hit:
            break
        observed = adapter.stability_run(run_id)
        if observed is not None:
            runs.append(observed)
    holds = []
    if not budget_hit:
        for kind, repeats, seconds in (
                ("no_input", item["no_input_repeats"], item["no_input_s"]),
                ("occlusion", item["occlusion_repeats"], item["occlusion_s"])):
            for repeat in range(repeats):
                if not _advance(adapter.clock, budget_s, seconds):
                    budget_hit = True
                    break
                observed = adapter.hold(item["runs"][0], kind, repeat)
                started += 1
                if observed is not None:
                    holds.append(observed)
            if budget_hit:
                break
    artifact = {"gate": "o8", "runs": runs, "holds": holds}
    verdict = repair_s0_judge.judge_o8(artifact)
    _write(out / "o8" / "runs.json", artifact)
    _write(out / "o8" / "verdict.json", verdict)
    return {"stage": "o8", "started": started, "runs": len(runs),
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
        self.processes = 0
        self.surfaces = {}

    def profile_frame(self, group, index, warmup, dest):
        import yaml
        import capture_replay
        self.profile_calls.append({
            "group": group["id"], "index": index, "warmup": warmup,
            "cold_start": not warmup, "dest": str(dest),
        })
        emitted = materialize_group(
            self.eval_native, self.geometry, self.config, group, self.out / "materialized")
        if not emitted["materialized"]:
            raise RuntimeError(emitted.get("stderr", "emit failed"))
        frame = self.out / "materialized" / "a" / "frames" / "f000.yaml"
        observation_id = yaml.safe_load(frame.read_text())["observation_id"]
        run = Path(dest)
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
        timeout = max(self.clock.remaining(BUDGET_S), 0.001)
        completed = launch_process(
            [self.cli, "--geometry", self.geometry, "--config", self.config,
             "--manifest", str(manifest), "--out", str(samples)],
            env=env, timeout=timeout)
        (run / "cli.log").write_text(completed.stdout + "\n" + completed.stderr)
        if completed.returncode != 0:
            raise RuntimeError(completed.stderr[-400:] or completed.stdout[-400:])
        row = next(yaml.safe_load_all(samples.read_text()))
        self.receipt_status = row["receipt"]["status"]
        self.trace_verdict = capture_replay.certify_trace(
            trace, capture_replay.expected_profile_trace(observation_id, run_id="live"))
        first = row.get("surface_first_call") if isinstance(row.get("surface_first_call"), dict) else {}
        second = row.get("surface_second_call") if isinstance(row.get("surface_second_call"), dict) else {}
        sample = {}
        for key, value in (
                ("integrate_ms", row.get("integrate_external_ms")),
                ("acquire_ms", row.get("acquire_ms")),
                ("roi_ms", row.get("roi_ms")),
                ("surface_first_ms", first.get("external_ms")),
                ("surface_repeat_ms", second.get("external_ms")),
                ("rss_kib", row.get("process_peak_rss_kib"))):
            if value is not None:
                sample[key] = value
        return sample

    def run_cumulative(self, group, frame_count, dest):
        """One profile process, several frames, three distinct surface calls."""
        import yaml
        import capture_replay
        geometry = group.get("geometry_file") or self.geometry
        config = group.get("config_file") or self.config
        dest = Path(dest)
        dest.mkdir(parents=True, exist_ok=True)
        emitted = dest / "emitted"
        completed = launch_process([
            self.eval_native, "--geometry", geometry, "--config", config, "--gate", "emit-window",
            "--group", group["id"], "--boxes", str(group["boxes"]), "--cloud", str(group["cloud"]),
            "--outside", str(group["outside"]), "--cross", str(group["cross"]),
            "--warmup", "0", "--repeat", str(frame_count), "--out", str(emitted),
        ], timeout=max(self.clock.remaining(BUDGET_S), 0.001))
        if completed.returncode != 0:
            raise RuntimeError((completed.stderr or completed.stdout)[-400:])
        frames = sorted((emitted / "frames").glob("f*.yaml"))
        if len(frames) < frame_count:
            raise RuntimeError("generator wrote %d frames" % len(frames))
        manifest = {
            "case_id": "cumulative-%s" % group["id"],
            "mode": "off",
            "surface": True,
            "state_digest": False,
            "frames": [{
                "path": os.path.relpath(frame, dest),
                "sha256": capture_replay.sha256_file(frame),
                "warmup": False,
                "expect_status": "INTEGRATED",
            } for frame in frames[:frame_count]],
        }
        (dest / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))
        samples = dest / "samples.yaml"
        timeout = max(self.clock.remaining(BUDGET_S), 0.001)
        completed = launch_process(
            [self.cli, "--geometry", geometry, "--config", config,
             "--manifest", str(dest / "manifest.yaml"), "--out", str(samples)],
            timeout=timeout)
        (dest / "cli.log").write_text((completed.stdout or "") + "\n" + (completed.stderr or ""))
        if completed.returncode != 0:
            raise RuntimeError((completed.stderr or completed.stdout)[-400:])
        self.processes += 1
        documents = [row for row in yaml.safe_load_all(samples.read_text()) if isinstance(row, dict)]
        self.receipt_status = documents[0]["receipt"]["status"]
        first = documents[0]["surface_first_call"]["external_ms"]
        repeat = documents[0]["surface_second_call"]["external_ms"]
        after = documents[1]["surface_first_call"]["external_ms"] if len(documents) > 1 else None
        self.surfaces = {
            "surface_first_ms": first,
            "surface_repeat_ms": repeat,
            "surface_after_update_ms": after,
        }
        scored = []
        for document in documents:
            call = document.get("surface_first_call") if isinstance(document.get("surface_first_call"), dict) else {}
            again = document.get("surface_second_call") if isinstance(document.get("surface_second_call"), dict) else {}
            sample = {"integrate_ms": document.get("integrate_external_ms")}
            if call.get("external_ms") is not None:
                sample["surface_first_ms"] = call["external_ms"]
            if again.get("external_ms") is not None:
                sample["surface_repeat_ms"] = again["external_ms"]
            if document.get("roi_ms") is not None:
                sample["roi_ms"] = document["roi_ms"]
            if document.get("acquire_ms") is not None:
                sample["acquire_ms"] = document["acquire_ms"]
            if document.get("process_peak_rss_kib") is not None:
                sample["rss_kib"] = document["process_peak_rss_kib"]
            scored.append(sample)
        if after is not None and scored:
            scored[0]["surface_after_update_ms"] = after
        return scored


def smoke_profile(plan, cli, eval_native, geometry, config, out):
    case = next(item for item in plan["o5_cases"] if item["id"] == "r02_b00_chamfer_c30000_o50_x00")
    group = dict(case["group"], geometry_file=geometry, config_file=config)
    adapter = LiveProfileAdapter(cli, eval_native, geometry, config, out, clock=MonotonicClock())
    samples = adapter.run_cumulative(group, 2, Path(out) / "cumulative")
    artifact = _assemble_group(group, samples, "o5")
    verdict = repair_s0_judge.judge_case(artifact)
    return {
        "non_scored": True,
        "formal_pass": False,
        "parseable": adapter.receipt_status == "INTEGRATED",
        "receipt_status": adapter.receipt_status,
        "processes": adapter.processes,
        "surfaces": adapter.surfaces,
        "judge": verdict,
        "stages": [{"judge": verdict}],
        "product_resource_certification": "NOT_EVALUATED",
    }


def _identity_tuple(identity):
    return (
        _field(identity, "epoch"),
        _field(identity, "observed_revision"),
        _field(identity, "geometry_hash"),
        _field(identity, "config_digest"),
    )


def _read_snapshot_chain(session, expected):
    """WaitSnapshot, full ReadSurface, one ReadCells page, then release."""
    from uld_occ_observer.srv import GetSnapshot, ReadCells, ReadSurface, ReleaseSnapshot
    wait = session.wait_for(expected["epoch"], expected["observation_id"], 2000)
    chain = {
        "wait_status": wait.status,
        "wait_reason": wait.reason,
        "age_ns": int(wait.age_ns),
        "wait_handle": wait.handle,
        "identity": _identity_tuple(wait.identity),
        "covered_stamp_ns": int(wait.identity.last_integrated_stamp_ns),
    }
    handle = ""
    snapshot = None
    if wait.status == "OK":
        acquire = session.node.create_client(GetSnapshot, "/uld_occ_observer/get_snapshot")
        session.wait_service(acquire, timeout=5)
        request = GetSnapshot.Request()
        request.expected_epoch = int(expected["epoch"])
        request.max_age_ms = 2000
        request.allow_stale = False
        request.exact_revision = False
        snapshot = session.call(acquire, request, 5)
        chain["snapshot_status"] = snapshot.status
        chain["snapshot_age_ns"] = int(snapshot.age_ns)
        chain["snapshot_identity"] = _identity_tuple(snapshot.identity)
        handle = snapshot.handle or ""
        if chain["snapshot_identity"] != chain["identity"]:
            chain["identity_stable"] = False
    pages = []
    cells = None
    release_status = None
    try:
        if wait.status != "OK" or snapshot is None or snapshot.status != "OK" or not handle:
            chain["handle"] = handle
            return chain
        chain["handle"] = handle
        surface = session.node.create_client(ReadSurface, "/uld_occ_observer/read_surface")
        roi = session.node.create_client(ReadCells, "/uld_occ_observer/read_cells")
        release = session.node.create_client(ReleaseSnapshot, "/uld_occ_observer/release_snapshot")
        session.wait_service(surface, timeout=5)
        cursor = 0
        while True:
            request = ReadSurface.Request()
            request.handle = handle
            request.cursor = int(cursor)
            request.limit = 512
            page = session.call(surface, request, 5)
            pages.append({
                "status": page.status,
                "done": bool(page.done),
                "identity": _identity_tuple(page.identity),
                "columns": len(page.x),
            })
            if page.status != "OK" or page.done:
                break
            cursor = page.next_cursor
        session.wait_service(roi, timeout=5)
        cell_request = ReadCells.Request()
        cell_request.handle = handle
        cell_request.begin = [0, 0, 0]
        cell_request.end = [16, 16, 16]
        cell_request.cursor = 0
        cell_request.limit = 4096
        cell_page = session.call(roi, cell_request, 5)
        cells = {"status": cell_page.status, "identity": _identity_tuple(cell_page.identity),
                 "count": len(cell_page.states)}
    finally:
        if handle:
            try:
                release = session.node.create_client(ReleaseSnapshot, "/uld_occ_observer/release_snapshot")
                session.wait_service(release, timeout=5)
                request = ReleaseSnapshot.Request()
                request.handle = handle
                release_status = session.call(release, request, 5).status
            except Exception as exc:
                release_status = "release_failed:%s" % exc
    identities = [page["identity"] for page in pages]
    if cells is not None:
        identities.append(cells["identity"])
    chain.update({
        "pages": len(pages),
        "surface_done": bool(pages) and pages[-1]["done"] and all(page["status"] == "OK" for page in pages),
        "roi": cells,
        "identity_stable": len(set(identities)) == 1 and identities[0][2] not in (None, ""),
        "release_status": release_status,
        "age_within_2s": int(wait.age_ns) <= 2_000_000_000,
    })
    return chain


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

        def ros_send(self, item, index, input_id, deadline=None, actual=None):
            del deadline, actual
            self.sender_calls.append((item["id"], index))
            def send():
                import time as time_mod
                self.sender_ident = threading.get_ident()
                session = holder["session"]
                session.broadcaster.sendTransform(s2_rehearsal.transform(session.next_stamp()))
                time_mod.sleep(0.4)
                stamp = session.next_stamp()
                holder["planned_s"] = time_mod.monotonic()
                holder["pub"].publish(s2_rehearsal.cloud_message(stamp))
                holder["actual_s"] = time_mod.monotonic()
                holder["observation_id"] = str(stamp)
                holder["stamp_ns"] = stamp
            self.sender_thread = threading.Thread(target=send)
            self.sender_thread.start()

        def ros_collect(self, item, index, input_id, deadline=None):
            del deadline
            self.executor_calls.append((item["id"], index))
            if self.sender_thread is not None:
                self.sender_thread.join(timeout=10)
            terminal = holder.get("terminal") or {}
            return {
                "input_id": input_id,
                "phase": "score",
                "disposition": terminal.get("disposition", "unknown"),
                "receipt_received": terminal.get("receipt_received", False),
                "status_query": terminal.get("status_query"),
                "receipt_status": terminal.get("receipt_status"),
            }

        def ros_query(self, item, index):
            import time as time_mod
            deadline = time_mod.monotonic() + 8
            while holder.get("observation_id") is None and time_mod.monotonic() < deadline:
                time_mod.sleep(0.02)
            observation_id = holder.get("observation_id")
            if not observation_id:
                return None
            last = None
            session = holder["session"]
            expected = {"epoch": 1, "sensor_id": "cam0", "observation_id": observation_id}
            while time_mod.monotonic() < deadline:
                last = session.query(1, observation_id)
                subscribed = any(
                    message.observation_id == observation_id and message.sensor_id == "cam0"
                    for message in session.receipts)
                terminal = interpret_observation_status(last, expected, subscribed)
                holder["terminal"] = terminal
                holder["query_status"] = terminal["disposition"]
                if terminal["disposition"] in ("integrated", "REJECTED", "EXPIRED", "NOT_SEEN"):
                    if terminal["disposition"] != "NOT_SEEN":
                        break
                time_mod.sleep(0.05)
            if holder.get("terminal", {}).get("disposition") == "integrated":
                holder["chain"] = _read_snapshot_chain(session, expected)
            if last is None:
                return None
            return {
                "query_id": "q-%d" % index,
                "op": "wait_snapshot" if holder.get("chain", {}).get("wait_status") == "OK" else "status_query",
                "status": holder.get("terminal", {}).get("status_query"),
            }

        def ros_health(self, item):
            return None

        def ros_drain(self, item, deadline=None):
            del item, deadline
            return None

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
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
        holder["pub"] = session.node.create_publisher(
            PointCloud2, "/s2_g6_cloud", QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        observer = s2_rehearsal.start_observer(node, geometry, config, out, [
            "-p", "input_mode:=subscription", "-p", "sensor_id:=cam0",
            "-p", "optical_frame:=d555_color_optical_frame",
            "-p", "cloud_topic:=/s2_g6_cloud", "-p", "behavior_test_hooks:=false",
        ], env)
        session.wait_service(session.status)
        result = schedule(
            mini, adapter, out, stages=("o7",), input_limit=1, scored=False,
            allow_incomplete_dependencies=True)
        result["sender_thread"] = adapter.sender_ident
        result["spin_thread"] = session.thread.ident
        import time as time_mod
        time_mod.sleep(1.2)
        samples = []
        for raw in session.health:
            parsed = yaml.safe_load(raw)
            if isinstance(parsed, dict) and "tree_nodes" in parsed:
                samples.append(parsed["tree_nodes"])
        result["stability_samples"] = samples
        result["stability_official_180s"] = "not_run"
        try:
            from uld_occ_observer.srv import GetResourceUsage
            session.wait_service(session.resource, timeout=5)
            usage = session.call(session.resource, GetResourceUsage.Request(), 5)
            result["resource_certification"] = usage.certification
        except Exception as exc:
            result["resource_certification"] = "NOT_EVALUATED"
            result["resource_error"] = str(exc)[-200:]
    finally:
        if session is not None:
            session.close()
        if observer is not None:
            observer.stop()
        if rclpy.ok():
            rclpy.shutdown()
    result["non_scored"] = True
    result["parseable"] = holder.get("query_status") == "integrated"
    result["receipt_status"] = (holder.get("terminal") or {}).get("receipt_status")
    result["terminal"] = holder.get("terminal")
    result["chain"] = holder.get("chain")
    if result["chain"] is not None:
        (Path(out) / "chain.json").write_text(json.dumps(result["chain"], indent=2) + "\n")
    result["planned_s"] = holder.get("planned_s")
    result["actual_s"] = holder.get("actual_s")
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
        adapter_name = os.environ.get("ULD_OCC_S2_ADAPTER", "live")
        selected = {"o5": "profile_cli_groups", "o6": "profile_cli_groups",
                    "o7": "ros_windows", "o8": "ros_stability", "all": "schedule"}
        payload["runner"] = selected.get(args.stage or "all", "schedule")
        payload["dispatched"] = True
        if adapter_name == "live" and os.environ.get("ULD_OCC_S2_CONFIRM_OFFICIAL") != "1":
            payload["reason"] = "official_samples_not_authorized"
            payload["adapter"] = "LiveCampaign"
            payload["started"] = {stage: 0 for stage in ("o5", "o6", "o7", "o8")}
            print(json.dumps(payload))
            return 2
        if adapter_name == "live":
            payload["adapter"] = "LiveCampaign"
            payload["launch_boundary"] = "launch_process"
            payload["reason"] = "official_long_window_not_started"
            payload["started"] = {stage: 0 for stage in ("o5", "o6", "o7", "o8")}
            print(json.dumps(payload))
            return 2
        if adapter_name != "injected":
            payload["reason"] = "unknown_adapter"
            print(json.dumps(payload))
            return 2
        plan = campaign_entry.load_plan(fixtures)
        stages = ("o5", "o6", "o7", "o8") if (args.stage or "all") == "all" else (args.stage,)
        result = schedule(plan, OfflineAdapter(), args.out, stages=stages, scored=True)
        result["runner"] = payload["runner"]
        result["dispatched"] = True
        print(json.dumps({key: result[key] for key in (
            "status", "official_samples", "product_resource_certification", "formal_pass",
            "started", "dispatched", "runner")}))
        failed = any(not item["judge"]["pass"] for item in result["stages"] if item.get("judge"))
        return 1 if failed else 0
    if args.mode == "judge":
        document = json.loads(Path(args.judge_file).read_text())
        verdict = campaign_entry.judge_recorded_window(document)
        print(json.dumps(verdict))
        return 0 if verdict.get("pass") else 1
    if args.mode == "replay":
        import capture_replay
        result = capture_replay.replay_failure(args.bundle, args.out)
        print(json.dumps(result))
        return 0 if result.get("replay_possible") else 1
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
            "processes": payload["processes"],
            "surfaces": payload["surfaces"],
            "judge_pass": payload["judge"]["pass"],
            "product_resource_certification": "NOT_EVALUATED",
        }))
        surfaces = payload["surfaces"]
        return 0 if payload["parseable"] and payload["processes"] == 1 and surfaces.get("surface_after_update_ms") is not None and not payload["judge"]["pass"] else 1
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
