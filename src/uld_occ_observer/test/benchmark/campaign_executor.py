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
WIRE_BUDGET_S = 180.0
WIRE_FRAMES = 2
WIRE_INPUTS = 2
_WIRED_SAMPLE_FIELDS = frozenset((
    "integrate_ms", "acquire_ms", "roi_ms", "surface_first_ms", "surface_repeat_ms",
    "surface_after_update_ms", "rss_kib", "init_s", "warm_init_s",
))


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
    return {
        "read_view_bytes": 1024,
        "cache_bytes": 1024,
        "read_view_bytes_peak": 2048,
        "cache_bytes_peak": 2048,
        "read_view_cache_bytes_peak": 4096,
        "store_tree_bytes": 65536,
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
        medians = [1000, 1000, 2000] if self.rss_growth else [1000, 1000, 1004]
        frames = contract.O8_SATURATION_FRAMES
        stamps = ["offline-sat-%d" % index for index in range(frames)]
        return {
            "run_id": run_id,
            "saturation_frames_expected": frames,
            "saturation_frames_verified": frames,
            "saturation_stamps": stamps,
            "saturation_receipts": [
                {"index": index, "observation_id": stamps[index], "events": 1,
                 "status": "COMPLETED", "disposition": "integrated", "epoch": 1,
                 "sensor_id": "cam0", "receipt_observation_id": stamps[index],
                 "changed_keys": 0, "rebuilt_blocks": 0}
                for index in range(frames)],
            "saturation_probability_keys": 0,
            "saturation_geometry_blocks": 0,
            "node_count_stable": self.node_stable,
            "node_health_times_s": [round(60.0 + index * 0.5, 3)
                                    for index in range(240)],
            "node_health_values": [4321 if self.node_stable else 4321 + (index % 2)
                                   for index in range(240)],
            "node_health_count": 240,
            "rss_median_kib": medians,
            "rss_window_counts": [30, 30, 30],
            "rss_samples": [[round(index * 2.0, 3),
                             medians[2] if index >= 60 else medians[0]]
                            for index in range(90)],
        }

    def _offline_probe(self, marker):
        import base64
        import hashlib
        import struct as _struct
        import zlib
        cells = contract.O8_ORACLE_CELLS
        states = bytearray(cells)
        for offset, value in ((0, 1), (1, 2), (2, 3)):
            states[offset] = value
        states = bytes(states)
        odds = [0.0] * cells
        odds[0], odds[1], odds[2] = -0.9, 0.4, 0.9
        odds_bytes = _struct.pack("<%df" % cells, *odds)
        return {
            "cells": cells,
            "region": {"begin": contract.O8_ORACLE_REGION["begin"],
                       "end": contract.O8_ORACLE_REGION["end"],
                       "shape": [75, 99, 74], "resolution_m": 0.02},
            "content": {
                "states_zb64": base64.b64encode(zlib.compress(states)).decode(),
                "odds_zb64": base64.b64encode(zlib.compress(odds_bytes)).decode(),
                "states_sha256": hashlib.sha256(states).hexdigest(),
                "odds_sha256": hashlib.sha256(odds_bytes).hexdigest(),
                "content_sha256": contract.probe_content_sha256(states, odds_bytes),
            },
            "surface_hash": "surface-%s" % marker,
            "surface_columns": 6534,
            "identity": {"epoch": 1, "map_revision": 2, "observed_revision": 2,
                         "last_integrated_stamp_ns": 1000},
            "release": "OK",
            "pages": 8,
            "pages_done": True,
            "probe_mono_ns": 1,
            "duration_ms": 100.0,
            "last_integrated_stamp_ns": 1000,
        }

    def hold(self, run_id, kind, repeat):
        self.hold_calls.append((run_id, kind, repeat, self.hold_ok))
        before = self._offline_probe("b")
        after = self._offline_probe("b")
        after["probe_mono_ns"] = 2
        row = {
            "kind": kind,
            "duration_s": 10,
            "duration_measured_s": 10.2,
            "probe_before": before,
            "probe_after": after,
        }
        if kind == "occlusion":
            row["occlusion_input"] = {
                "shape": "plane", "plane_uld_x_m": -0.62, "plane_max_cell_x": 7,
                "oracle_min_cell_x": 9, "point_count": 10,
                "points_sha256": "offline", "rays_file": "offline",
                "y_range_m": [-0.85, 0.52], "z_range_m": [0.75, 1.85],
                "sensor_uld_origin_m": [-1.15, 0.0, 1.2],
            }
            row["input_accepted"] = self.hold_ok is True
            row["occluder_receipt"] = {
                "status": "COMPLETED" if self.hold_ok is True else "REJECTED",
                "changed_keys": 700 if self.hold_ok is True else None,
            }
            row["region_probability_unchanged"] = self.hold_ok is True
            row["freshness_advanced"] = True if self.hold_ok is True else None
        elif kind == "duplicate_id":
            row["duplicate_receipt"] = {
                "status": "COMPLETED", "disposition":
                    "rejected" if self.hold_ok is True else "integrated",
            }
            row["duplicate_rejected"] = self.hold_ok is True
            row["probability_unchanged"] = self.hold_ok is True
            row["freshness_advanced"] = False
        else:
            row["probability_unchanged"] = self.hold_ok is True
            row["freshness_advanced"] = False
        return row


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


def _assemble_window(window, rows, queries, health, inflight, continuity=None):
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
        if continuity is not None:
            artifact["continuity"] = continuity
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
             allow_incomplete_dependencies=False, o5_evidence=None):
    """Run the requested stages. scored=True does nothing unless S3 is open.

    o5_evidence: a prior invocation's scoring/o5/verdict.json. The S3
    orchestrator resumes stages one at a time in contract order; a resumed
    O6 may carry the ALREADY-SCORED O5 verdict instead of re-running O5
    (re-running a scored unit is forbidden). The file must say pass:true --
    a failing or unreadable evidence file leaves the dependency blocked.
    """
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
    if o5_evidence and "o5" not in stages and ("o6" in stages or "o7" in stages):
        try:
            document = json.loads(Path(o5_evidence).read_text())
        except (OSError, ValueError):
            document = None
        if isinstance(document, dict) and document.get("pass") is True:
            o5_pass = True
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
        continuity = None
        if hasattr(adapter, "ros_continuity"):
            continuity = adapter.ros_continuity(window)
        artifact = _assemble_window(window, rows, queries, health, inflight, continuity)
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
                ("occlusion", item["occlusion_repeats"], item["occlusion_s"]),
                ("duplicate_id", item.get("duplicate_id_repeats", 0), item["no_input_s"])):
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
    artifact = {"gate": "o8", "runs": runs, "holds": holds,
                "artifact_dir": str(out / "o8")}
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
        self.budget_s = BUDGET_S

    def _timeout(self):
        remaining = self.clock.remaining(self.budget_s)
        if remaining <= 0:
            raise TimeoutError("runner budget exhausted")
        return remaining

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
        completed = launch_process(
            [self.cli, "--geometry", self.geometry, "--config", self.config,
             "--manifest", str(manifest), "--out", str(samples)],
            env=env, timeout=self._timeout())
        (run / "cli.log").write_text(completed.stdout + "\n" + completed.stderr)
        if completed.returncode != 0:
            raise RuntimeError(completed.stderr[-400:] or completed.stdout[-400:])
        row = next(yaml.safe_load_all(samples.read_text()))
        self.receipt_status = row["receipt"]["status"]
        self.frame_receipts = [row.get("receipt", {}).get("status")]
        self.trace_verdict = capture_replay.certify_trace(
            trace, capture_replay.expected_profile_trace(observation_id, run_id="live"))
        first = row.get("surface_first_call") if isinstance(row.get("surface_first_call"), dict) else {}
        second = row.get("surface_second_call") if isinstance(row.get("surface_second_call"), dict) else {}
        # Raw values only; poisoned or missing values stay visible to the judge.
        sample = {
            "integrate_ms": row.get("integrate_external_ms"),
            "acquire_ms": row.get("acquire_ms"),
            "roi_ms": row.get("roi_ms"),
            "surface_first_ms": first.get("external_ms"),
            "surface_repeat_ms": second.get("external_ms"),
            "rss_kib": row.get("process_peak_rss_kib"),
        }
        self.sample_problems = [dict(problem, frame=0) for problem in _wire_sample_problems(sample)]
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
        ], timeout=self._timeout())
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
        completed = launch_process(
            [self.cli, "--geometry", geometry, "--config", config,
             "--manifest", str(dest / "manifest.yaml"), "--out", str(samples)],
            timeout=self._timeout())
        (dest / "cli.log").write_text((completed.stdout or "") + "\n" + (completed.stderr or ""))
        if completed.returncode != 0:
            raise RuntimeError((completed.stderr or completed.stdout)[-400:])
        self.processes += 1
        documents = [row for row in yaml.safe_load_all(samples.read_text()) if isinstance(row, dict)]
        self.receipt_status = documents[0]["receipt"]["status"]
        self.frame_receipts = [document.get("receipt", {}).get("status") for document in documents]
        first = documents[0]["surface_first_call"]["external_ms"]
        repeat = documents[0]["surface_second_call"]["external_ms"]
        after = documents[1]["surface_first_call"]["external_ms"] if len(documents) > 1 else None
        self.surfaces = {
            "surface_first_ms": first,
            "surface_repeat_ms": repeat,
            "surface_after_update_ms": after,
        }
        self.surface_problems = _wire_sample_problems(self.surfaces)
        scored = []
        for document in documents:
            call = document.get("surface_first_call") if isinstance(document.get("surface_first_call"), dict) else {}
            again = document.get("surface_second_call") if isinstance(document.get("surface_second_call"), dict) else {}
            # Raw values only. A poisoned or missing value stays in its column so
            # the judge scores it as nonfinite/missing instead of silently
            # shrinking the sample; the wiring layer records its own problems.
            scored.append({
                "integrate_ms": document.get("integrate_external_ms"),
                "surface_first_ms": call.get("external_ms"),
                "surface_repeat_ms": again.get("external_ms"),
                "roi_ms": document.get("roi_ms"),
                "acquire_ms": document.get("acquire_ms"),
                "rss_kib": document.get("process_peak_rss_kib"),
            })
        if after is not None and scored:
            scored[0]["surface_after_update_ms"] = after
        # The after-update moment for sample i is the first surface call of
        # frame i+1; every sample with a successor carries its own value so
        # the judge scores a full column instead of a missing one.
        for index, sample in enumerate(scored):
            if index + 1 < len(documents):
                successor = documents[index + 1].get("surface_first_call")
                if isinstance(successor, dict):
                    sample["surface_after_update_ms"] = successor.get("external_ms")
        self.sample_problems = []
        for index, sample in enumerate(scored):
            for problem in _wire_sample_problems(sample):
                self.sample_problems.append(dict(problem, frame=index))
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
        _field(identity, "map_revision"),
        _field(identity, "observed_revision"),
        _field(identity, "last_integrated_stamp_ns"),
        _field(identity, "geometry_hash"),
        _field(identity, "config_digest"),
        _field(identity, "clock_domain"),
    )


def _finite_age(value, limit):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    return value >= 0 and value <= limit


def _wire_sample_problems(values):
    """Named problems for wiring-level samples: bool/NaN/Inf/negative/missing.

    This check is independent of the campaign judge. A missing_group rejection
    must not mask a poisoned sample, so the wiring layer keeps its own list and
    refuses to call itself executed while any problem stands.
    """
    problems = []
    for key in sorted(values):
        problem = repair_s0_judge._number_problem(values[key])
        if problem:
            problems.append({"field": key, "problem": problem})
    return problems


def _cloud_publisher_qos():
    """Frozen plan §2B: the cloud topic is Reliable + KeepLast(32)."""
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=32,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


def _receipt_events(session, observation_id, sensor_id="cam0"):
    """Raw same-ID receipt subscription events, oldest first.

    Counts and duplicate detection come from this list only; a boolean cannot
    discover a repeated terminal receipt.
    """
    events = getattr(session, "receipt_events", None)
    if events is None:
        # Compatibility: messages retained without receive times.
        events = [(None, message) for message in getattr(session, "receipts", [])]
    collected = []
    for received_mono_ns, message in events:
        if _field(message, "observation_id") != observation_id:
            continue
        if _field(message, "sensor_id") != sensor_id:
            continue
        collected.append({
            "received_mono_ns": received_mono_ns,
            "status": _field(message, "status"),
            "queue_ms": _field(message, "queue_ms"),
            "total_ms": _field(message, "total_ms"),
            "input_stamp_ns": _field(message, "input_stamp_ns"),
            # 0 is the product's not-recorded sentinel for this additive field.
            "callback_enter_ns": _field(message, "callback_enter_ns") or None,
        })
    return collected


def _receipt_event_problems(events, observation_id):
    problems = []
    for index, event in enumerate(events):
        for key in ("queue_ms", "total_ms"):
            problem = repair_s0_judge._number_problem(event.get(key))
            if problem:
                problems.append({"event": index, "field": key, "problem": problem,
                                 "observation_id": observation_id})
    if len(events) > 1:
        problems.append({"event": "duplicate", "problem": "duplicate_terminal",
                         "observation_id": observation_id, "count": len(events)})
    return problems


def _o8_health_valid(health, rss):
    """O8 wiring validity: parseable, distinct-stamp health and readable RSS.

    Missing samples, repeated timestamps, poisoned node counts, or an exited
    process (unreadable /proc) keep the wiring from claiming it executed.
    """
    health_dicts = [item["health"] for item in health if isinstance(item["health"], dict)]
    stamps = [item.get("stamp_ns") for item in health_dicts]
    return bool(
        len(health_dicts) == 2
        and len(stamps) == len(set(stamps)) == 2
        and all(repair_s0_judge._number_problem(item.get("tree_nodes")) is None
                for item in health_dicts)
        and len(rss) == 2 and all(value is not None for value in rss))


def _logged_call(session, client, request, timeout, service, request_id):
    """One service call with an independent request id and monotonic boundaries.

    The log entry keeps the CLOCK_MONOTONIC domain; the status is copied from
    the response verbatim and never synthesized.
    """
    started = time.monotonic_ns()
    response = session.call(client, request, timeout)
    status = _field(response, "status")
    return response, {
        "service": service,
        "request_id": request_id,
        "started_mono_ns": started,
        "ended_mono_ns": time.monotonic_ns(),
        "status": status if status is not None else "no_status_field",
    }


def _content_sha256(material):
    import hashlib
    import json as json_mod
    digest = hashlib.sha256()
    for item in material:
        digest.update(json_mod.dumps(item, sort_keys=True).encode())
    return digest.hexdigest()


def expected_map_contract(geometry_file, config_file):
    """Identity and surface shape taken from the fixture files, not from a response."""
    import hashlib
    import math
    import yaml
    geometry_bytes = Path(geometry_file).read_bytes()
    config_bytes = Path(config_file).read_bytes()
    descriptor = json.loads(yaml.safe_load(geometry_bytes.decode())["descriptor_json"])
    config = yaml.safe_load(config_bytes.decode())
    resolution = float(config["resolution_m"])
    return {
        "geometry_hash": descriptor["geometry_hash"],
        "config_digest": hashlib.sha256(
            geometry_bytes + b"\n" + config_bytes + b"\nprobability-profile-v1").hexdigest(),
        "clock_domain": config["clock_domain"],
        "surface_shape": [
            math.ceil(float(descriptor["length"]) / resolution),
            math.ceil(float(descriptor["width"]) / resolution),
        ],
    }


def _read_snapshot_chain(session, expected):
    """Use only the WaitSnapshot lease. A later GetSnapshot is not a substitute."""
    from uld_occ_observer.srv import ReadCells, ReadSurface, ReleaseSnapshot
    service_calls = []
    chain_started_ns = time.monotonic_ns()
    wait_started_ns = time.monotonic_ns()
    wait = session.wait_for(expected["epoch"], expected["observation_id"], 2000)
    wait_ms = (time.monotonic_ns() - wait_started_ns) / 1e6
    service_calls.append({
        "service": "wait_snapshot",
        "request_id": "wait-%s" % expected["observation_id"],
        "started_mono_ns": wait_started_ns,
        "ended_mono_ns": time.monotonic_ns(),
        "status": getattr(wait, "status", None),
    })
    handle = wait.handle or ""
    wait_identity = _identity_tuple(wait.identity)
    reasons = []
    if wait.status != "OK":
        reasons.append("wait_%s" % wait.status)
    if not handle:
        reasons.append("empty_handle")
    if expected.get("receipt_matches_contract") is False:
        reasons.append("receipt_contract")
    if not _finite_age(int(wait.age_ns), 2_000_000_000):
        reasons.append("age")
    if wait_identity[0] != expected.get("epoch"):
        reasons.append("epoch")
    minimum = expected.get("min_observed_revision")
    if minimum is not None and (wait_identity[2] is None or wait_identity[2] < minimum):
        reasons.append("target_not_covered")
    for name, index in (("geometry_hash", 4), ("config_digest", 5), ("clock_domain", 6)):
        if name in expected and wait_identity[index] != expected[name]:
            reasons.append(name)
    chain = {
        "wait_status": wait.status,
        "wait_reason": wait.reason,
        "age_ns": int(wait.age_ns),
        "handle": handle,
        "identity": wait_identity,
    }
    pages = []
    cells = None
    release_status = None
    material = []
    seen = set()
    try:
        if reasons:
            chain["identity_stable"] = False
            chain["reasons"] = reasons
            chain["content_material"] = material
            chain["content_sha256"] = _content_sha256(material)
            chain["wait_ms"] = wait_ms
            chain["service_calls"] = service_calls
            chain["total_ms"] = (time.monotonic_ns() - chain_started_ns) / 1e6
            return chain
        surface = session.node.create_client(ReadSurface, "/uld_occ_observer/read_surface")
        roi = session.node.create_client(ReadCells, "/uld_occ_observer/read_cells")
        session.wait_service(surface, timeout=5)
        cursor = 0
        shape = expected.get("surface_shape")
        surface_pages_ms = 0.0
        while True:
            request = ReadSurface.Request()
            request.handle = handle
            request.cursor = int(cursor)
            request.limit = 512
            page, page_call = _logged_call(
                session, surface, request, 5, "read_surface",
                "surface-%s-%d" % (handle or "none", int(cursor)))
            service_calls.append(page_call)
            surface_pages_ms += (page_call["ended_mono_ns"] - page_call["started_mono_ns"]) / 1e6
            columns = [_field(page, name) for name in (
                "x", "y", "active_count", "known_count", "has_occupied", "top_z")]
            width = None if columns[0] is None else len(columns[0])
            shape_ok = width is not None and all(item is not None and len(item) == width for item in columns)
            identity = _identity_tuple(page.identity)
            if page.status != "OK" or not shape_ok or identity != wait_identity:
                reasons.append("page")
            stored = [list(item) if item is not None else None for item in columns]
            material.append(stored)
            if shape_ok:
                for column in zip(columns[0], columns[1]):
                    x, y = int(column[0]), int(column[1])
                    if (x, y) in seen:
                        reasons.append("duplicate_column")
                    seen.add((x, y))
                    if shape is not None and not (0 <= x < int(shape[0]) and 0 <= y < int(shape[1])):
                        reasons.append("column_range")
            pages.append({"status": page.status, "done": bool(page.done), "identity": identity,
                          "columns": width, "cursor": int(cursor),
                          "next_cursor": int(page.next_cursor),
                          "call_ms": (page_call["ended_mono_ns"] - page_call["started_mono_ns"]) / 1e6})
            if page.status != "OK" or not shape_ok:
                break
            if page.done:
                if int(page.next_cursor) != int(cursor) + width:
                    reasons.append("cursor")
                break
            if int(page.next_cursor) <= int(cursor):
                reasons.append("cursor")
                break
            cursor = int(page.next_cursor)
        if not pages or not pages[-1]["done"]:
            reasons.append("surface_incomplete")
        if shape is not None and len(seen) != int(shape[0]) * int(shape[1]):
            reasons.append("missing_column")
        if "surface_columns" in expected and material != list(expected["surface_columns"]):
            reasons.append("content_oracle")
        session.wait_service(roi, timeout=5)
        cell_request = ReadCells.Request()
        cell_request.handle = handle
        cell_request.begin = [0, 0, 0]
        cell_request.end = [16, 16, 16]
        cell_request.cursor = 0
        cell_request.limit = 4096
        cell_page, cell_call = _logged_call(
            session, roi, cell_request, 5, "read_cells", "cells-%s" % (handle or "none"))
        service_calls.append(cell_call)
        cells_ms = (cell_call["ended_mono_ns"] - cell_call["started_mono_ns"]) / 1e6
        cell_identity = _identity_tuple(cell_page.identity)
        if cell_page.status != "OK" or cell_identity != wait_identity:
            reasons.append("roi")
        states = [int(item) for item in list(cell_page.states or [])]
        odds = [float(item) for item in list(_field(cell_page, "log_odds") or [])]
        material.append(states)
        material.append(odds)
        cells = {"status": cell_page.status, "identity": cell_identity, "count": len(states),
                 "done": bool(_field(cell_page, "done")),
                 "next_cursor": int(_field(cell_page, "next_cursor") or 0),
                 "occupied": any(state == 2 for state in states)}
        if (len(states) != 4096 or len(odds) != len(states) or not cells["done"]
                or cells["next_cursor"] != 4096):
            reasons.append("missing_roi")
        if "roi_states" in expected and states != [int(item) for item in expected["roi_states"]]:
            reasons.append("content_oracle")
        if "roi_log_odds" in expected and odds != [float(item) for item in expected["roi_log_odds"]]:
            reasons.append("content_oracle")
    finally:
        if handle:
            try:
                release = session.node.create_client(ReleaseSnapshot, "/uld_occ_observer/release_snapshot")
                session.wait_service(release, timeout=5)
                request = ReleaseSnapshot.Request()
                request.handle = handle
                release_started_ns = time.monotonic_ns()
                release_status = session.call(release, request, 5).status
                service_calls.append({
                    "service": "release_snapshot",
                    "request_id": "release-%s" % handle,
                    "started_mono_ns": release_started_ns,
                    "ended_mono_ns": time.monotonic_ns(),
                    "status": release_status,
                })
            except Exception as exc:
                release_status = "release_failed:%s" % exc
                reasons.append("release")
    if release_status != "OK":
        reasons.append("release")
    chain.update({
        "pages": len(pages),
        "page_log": pages,
        "surface_columns": len(seen),
        "surface_done": bool(pages) and pages[-1]["done"] and "surface_incomplete" not in reasons
        and "page" not in reasons,
        "roi": cells,
        "content_material": material,
        "content_sha256": _content_sha256(material),
        "identity_stable": not reasons,
        "reasons": reasons,
        "release_status": release_status,
        "age_within_2s": "age" not in reasons,
        "wait_ms": wait_ms,
        "surface_pages_ms": surface_pages_ms,
        "cells_ms": cells_ms,
        "service_calls": service_calls,
        "total_ms": (time.monotonic_ns() - chain_started_ns) / 1e6,
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
    holder = {"geometry": geometry, "config": config}

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
            events = _receipt_events(holder["session"], holder.get("observation_id") or "")
            return {
                "input_id": input_id,
                "phase": "score",
                "disposition": terminal.get("disposition", "unknown"),
                "receipt_received": terminal.get("receipt_received", False),
                "status_query": terminal.get("status_query"),
                "receipt_status": terminal.get("receipt_status"),
                "terminal_count": len(events),
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
                events = _receipt_events(session, observation_id)
                terminal = interpret_observation_status(last, expected, bool(events))
                terminal["terminal_count"] = len(events)
                terminal["receipt_events"] = events
                holder["terminal"] = terminal
                holder["query_status"] = terminal["disposition"]
                if terminal["disposition"] in ("integrated", "REJECTED", "EXPIRED", "NOT_SEEN"):
                    if terminal["disposition"] != "NOT_SEEN":
                        break
                time_mod.sleep(0.05)
            if holder.get("terminal", {}).get("disposition") == "integrated":
                receipt = _field(last, "receipt")
                receipt_identity = _field(receipt, "identity")
                covered = _field(receipt_identity, "observed_revision")
                if covered is not None:
                    expected["min_observed_revision"] = int(covered)
                if holder.get("geometry") and holder.get("config"):
                    contract = expected_map_contract(holder["geometry"], holder["config"])
                    expected.update(contract)
                    pinned = _identity_tuple(receipt_identity)
                    expected["receipt_matches_contract"] = pinned[4:] == (
                        contract["geometry_hash"], contract["config_digest"], contract["clock_domain"])
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
        from sensor_msgs.msg import PointCloud2
        holder["pub"] = session.node.create_publisher(
            PointCloud2, "/s2_g6_cloud", _cloud_publisher_qos())
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


_FAILURE_EXPECTED = {
    "o5": ("cumulative/samples.yaml", "cumulative/manifest.yaml", "cumulative/cli.log"),
    "o6": ("cumulative/samples.yaml", "cumulative/manifest.yaml", "cumulative/cli.log"),
    "o7": ("o7/window.json", "o7/campaign_verdict.json"),
    "o8": ("o8/runs.json", "o8/verdict.json"),
}


def _verify_replay_manifest(out, relative_path):
    """Verify every input a replay manifest references.

    An input counts only when the file exists, its real sha256 equals the
    declared one, the frame carries identity (sensor/observation) and a
    transform, and the frame set is a contiguous f000..fNNN prefix. A manifest
    that references a nonexistent file or a fake hash is not replay material,
    however complete the surrounding filenames look.
    """
    import yaml
    manifest_path = Path(out) / relative_path
    try:
        document = yaml.safe_load(manifest_path.read_text())
    except Exception:
        return {"ok": False, "problems": ["manifest_unparseable"], "verified": 0, "frames": 0}
    frames = document.get("frames") if isinstance(document, dict) else None
    if not isinstance(frames, list) or not frames:
        return {"ok": False, "problems": ["manifest_no_frames"], "verified": 0, "frames": 0}
    problems = []
    verified = 0
    names = []
    for entry in frames:
        name = entry.get("path") if isinstance(entry, dict) else None
        declared = entry.get("sha256") if isinstance(entry, dict) else None
        if not isinstance(name, str) or not isinstance(declared, str):
            problems.append("frame_fields")
            continue
        names.append(name)
        frame_path = manifest_path.parent / name
        if not frame_path.is_file():
            problems.append("input_missing:%s" % name)
            continue
        import capture_replay
        if capture_replay.sha256_file(frame_path) != declared:
            problems.append("input_sha_mismatch:%s" % name)
            continue
        try:
            frame = yaml.safe_load(frame_path.read_text())
        except Exception:
            problems.append("input_unreadable:%s" % name)
            continue
        if not isinstance(frame, dict) or not frame.get("observation_id") \
                or not frame.get("sensor_id"):
            problems.append("input_identity:%s" % name)
            continue
        if not isinstance(frame.get("transform"), dict):
            problems.append("input_tf:%s" % name)
            continue
        verified += 1
    # Contiguous complete prefix: f000, f001, ... with no gaps.
    import re
    numbers = sorted(int(match.group(1)) for name in names
                     for match in [re.fullmatch(r"f(\d{3})\.yaml", Path(name).name)]
                     if match)
    if numbers != list(range(len(numbers))):
        problems.append("input_prefix_gaps")
    return {"ok": not problems, "problems": problems, "verified": verified,
            "frames": len(frames)}


def _save_failure(out, stage, reason, details, stage_dir=None):
    """Persist the failure bundle before teardown.

    Nothing here is asserted: capture_complete is computed from the files that
    are actually on disk (with trace integrity and every referenced input
    present at its declared hash), replay_possible requires a manifest whose
    frames all verify (existence, sha256, identity, TF, complete prefix), and
    every file gets a sha256. Callers must invoke this before tearing sessions
    or observer processes so the inventory reflects what the failed run really
    captured.
    """
    import capture_replay
    out = Path(out)
    bundle = out / "failure" / stage
    bundle.mkdir(parents=True, exist_ok=True)
    roots = [Path(stage_dir)] if stage_dir else [out / stage]
    inventory = []
    for root in roots:
        if not root.is_dir():
            continue
        for item in sorted(root.rglob("*")):
            if item.is_file():
                entry = {
                    "path": str(item.relative_to(out)),
                    "bytes": item.stat().st_size,
                    "sha256": capture_replay.sha256_file(item),
                }
                if item.name == "trace.jsonl":
                    events, errors = capture_replay._load_events(item)
                    entry["trace_events"] = len(events)
                    entry["trace_errors"] = errors
                inventory.append(entry)
    present = {entry["path"] for entry in inventory}
    missing = [name for name in _FAILURE_EXPECTED.get(stage, ())
               if not any(path == name or path.endswith("/" + name) for path in present)]
    replay_possible = False
    replay_inputs = []
    input_problems = []
    for entry in inventory:
        if not entry["path"].endswith("manifest.yaml"):
            continue
        verdict = _verify_replay_manifest(out, entry["path"])
        if verdict["ok"]:
            replay_possible = True
            replay_inputs.append(entry["path"])
        else:
            input_problems.append({"manifest": entry["path"],
                                   "problems": verdict["problems"]})
            for problem in verdict["problems"]:
                token = problem.split(":", 1)
                if token[0] in ("input_missing", "input_sha_mismatch",
                                "input_unreadable", "input_identity", "input_tf"):
                    missing.append("%s!%s" % (entry["path"], token[1] if len(token) > 1 else ""))
    # Any referenced input that does not verify (missing file, wrong hash, no
    # identity or TF, broken prefix) makes the capture incomplete: the bundle
    # does not honestly contain what a replay needs.
    capture_complete = not missing and not input_problems and all(
        not entry.get("trace_errors") for entry in inventory if "trace_errors" in entry)
    bounded = {"text": str(details), "truncated": False}
    if len(bounded["text"]) > 8000:
        bounded = {"text": bounded["text"][:8000], "truncated": True}
    manifest = {
        "stage": stage,
        "reason": reason,
        "details": bounded,
        "official_samples": "not_run",
        "capture_complete": capture_complete,
        "replay_possible": replay_possible,
        "missing": missing,
        "replay_inputs": replay_inputs,
        "input_problems": input_problems,
        "file_count": len(inventory),
        "files": inventory,
    }
    (bundle / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return bundle


def _rss_kib(pid):
    status = Path("/proc/%s/status" % pid)
    if not status.is_file():
        return None
    for line in status.read_text().splitlines():
        if line.startswith("VmRSS:"):
            return int(line.split()[1])
    return None


def _wait_integrated(session, observation_id, deadline_s):
    end = time.monotonic() + deadline_s
    last = None
    terminal = {"disposition": "unknown", "receipt_received": False}
    expected = {"epoch": 1, "sensor_id": "cam0", "observation_id": observation_id}
    events = []
    while time.monotonic() < end:
        last = session.query(1, observation_id)
        events = _receipt_events(session, observation_id)
        terminal = interpret_observation_status(last, expected, bool(events))
        terminal["terminal_count"] = len(events)
        terminal["receipt_events"] = events
        if terminal["disposition"] in ("integrated", "REJECTED", "EXPIRED"):
            break
        time.sleep(0.05)
    return last, terminal


def _profile_group(plan, stage):
    if stage == "o5":
        case = next(item for item in plan["o5_cases"] if item["id"] == "r02_b00_chamfer_c30000_o50_x00")
        return dict(case["group"])
    return next(
        group for group in plan["o6_groups"]
        if group["resolution"] == 0.02 and group["hull"] == "chamfer"
        and group["boxes"] == 0 and group["cloud"] == 30000
        and group["outside"] == 0.5 and group["cross"] == 0.0)


class LiveCampaign:
    """Real O5–O8 adapters. wire() is the non-scored entry; official counts stay at zero."""

    def __init__(self, cli, eval_native, node, geometry, config, clock=None):
        self.clock = clock or MonotonicClock()
        self.cli = cli
        self.eval_native = eval_native
        self.node = node
        self.geometry = geometry
        self.config = config
        self.profile = None

    def wire(self, plan, out):
        out = Path(out)
        out.mkdir(parents=True, exist_ok=True)
        self.profile = LiveProfileAdapter(
            self.cli, self.eval_native, self.geometry, self.config, out, clock=self.clock)
        self.profile.budget_s = WIRE_BUDGET_S
        result = {
            "adapter": "LiveCampaign",
            "scoring": "non_scored",
            "official_samples": "not_run",
            "formal_pass": False,
            "product_resource_certification": "NOT_EVALUATED",
            "reason": "non_scored_wiring",
            "started": {stage: 0 for stage in ("o5", "o6", "o7", "o8")},
            "stages": [],
            "wiring_complete": False,
            "short_samples_rejected": False,
        }
        stage_name = "o5"
        try:
            for stage_name in ("o5", "o6"):
                result["stages"].append(self._wire_profile(plan, out, stage_name, result["started"]))
            stage_name = "o7"
            ros = self._wire_ros(plan, out, result["started"])
            result["stages"].extend(ros)
            result["sender"] = ros[0].get("sender")
        except Exception as exc:
            # Persist before the finally teardown inventories the stage dir.
            _save_failure(out, stage_name, type(exc).__name__,
                          {"error_type": type(exc).__name__, "error": str(exc)},
                          out / stage_name)
            result["reason"] = "wiring_failed"
            result["error"] = str(exc)[-400:]
            _write(out / "wiring.json", result)
            return result
        rejected = bool(result["stages"]) and all(
            item.get("judge", {}).get("pass") is False for item in result["stages"])
        result["short_samples_rejected"] = rejected
        result["formal_pass"] = any(item.get("judge", {}).get("pass") is True for item in result["stages"])
        result["wiring_complete"] = rejected and result["formal_pass"] is False and all(
            item.get("executed") is True for item in result["stages"])
        if result["formal_pass"]:
            result["reason"] = "short_sample_not_rejected"
        _write(out / "wiring.json", result)
        return result

    def _wire_profile(self, plan, out, stage, started):
        group = _profile_group(plan, stage)
        before = self.profile.processes
        samples = self.profile.run_cumulative(group, WIRE_FRAMES, out / stage / "cumulative")
        started[stage] = len(samples)
        artifact = _assemble_group(group, samples, stage)
        planned = [
            {"id": item[0]["id"], **{key: item[0][key] for key in GROUP_KEYS}}
            for item in _profile_jobs(plan, stage)
        ]
        campaign = {
            "gate": "%s_campaign" % stage,
            "groups": [artifact],
            "planned_groups": planned,
        }
        verdict = repair_s0_judge.judge_case(campaign)
        _write(out / stage / "verdict.json", verdict)
        surfaces = dict(self.profile.surfaces)
        # Wiring-level judgment: finite/non-negative/typed surfaces, every frame
        # receipt INTEGRATED, and no per-sample problem. These checks stand on
        # their own — a judge missing_group rejection must not mask bad samples,
        # and bad samples must keep executed false even when the judge rejects.
        problems = list(getattr(self.profile, "surface_problems", None) or _wire_sample_problems(surfaces))
        # Recompute per-frame problems from the samples themselves so an
        # adapter that does not self-report cannot mask a poisoned frame.
        for index, sample in enumerate(samples):
            present = {key: value for key, value in dict(sample).items()
                       if key in _WIRED_SAMPLE_FIELDS}
            for problem in _wire_sample_problems(present):
                problems.append(dict(problem, frame=index))
        frame_receipts = list(getattr(self.profile, "frame_receipts", None) or [])
        receipts_ok = bool(frame_receipts) and all(status == "INTEGRATED" for status in frame_receipts)
        return {
            "stage": stage,
            "started": len(samples),
            "processes": self.profile.processes - before,
            "surfaces": surfaces,
            "sample_problems": problems,
            "frame_receipts": frame_receipts,
            "judge": verdict,
            "executed": self.profile.processes - before == 1 and len(samples) == WIRE_FRAMES
            and not problems and receipts_ok
            and self.profile.receipt_status == "INTEGRATED" and verdict.get("pass") is False,
        }

    def _wire_ros(self, plan, out, started):
        import threading
        parent = str(Path(__file__).resolve().parents[1])
        if parent not in sys.path:
            sys.path.insert(0, parent)
        import s2_rehearsal
        window = next(item for item in plan["o7_windows"] if item["kind"] == "normal" and item["id"] == "normal_b00_r0")
        group = next(item for item in plan["o6_groups"] if item["id"] == window["group_id"])
        geometry = group["geometry_file"]
        config = group["config_file"]
        previous_domain = os.environ.get("ROS_DOMAIN_ID")
        os.environ["ROS_DOMAIN_ID"] = "48"
        import rclpy
        if not rclpy.ok():
            rclpy.init()
        session = None
        observer = None
        (out / "o7").mkdir(parents=True, exist_ok=True)
        try:
            session = s2_rehearsal.Session()
            env = os.environ.copy()
            observer = s2_rehearsal.start_observer(self.node, geometry, config, out / "o7", [
                "-p", "input_mode:=subscription", "-p", "sensor_id:=cam0",
                "-p", "optical_frame:=d555_color_optical_frame",
                "-p", "cloud_topic:=/s2_wire_cloud", "-p", "behavior_test_hooks:=false",
            ], env)
            from sensor_msgs.msg import PointCloud2
            publisher = session.node.create_publisher(
                PointCloud2, "/s2_wire_cloud", _cloud_publisher_qos())
            session.wait_service(session.status)
            session.broadcaster.sendTransform(s2_rehearsal.transform(session.next_stamp()))
            time.sleep(0.3)
            origin_ns = time.monotonic_ns()
            spacing_s = 1.0 / float(window["input_hz"])
            deadlines_ns = [
                origin_ns + int(round(index * spacing_s * 1e9)) for index in range(WIRE_INPUTS)]
            sent = []
            sender_ident = {}

            def publish():
                sender_ident["id"] = threading.get_ident()
                for index, deadline_ns in enumerate(deadlines_ns):
                    delay_s = (deadline_ns - time.monotonic_ns()) / 1e9
                    if delay_s > 0:
                        time.sleep(delay_s)
                    stamp = session.next_stamp()
                    # rclpy publish() returns None; the first clock read after
                    # the call returns is the observable publish boundary. All
                    # boundaries here are CLOCK_MONOTONIC; the cloud header
                    # stamp is kept separately and never stands in for them.
                    call_start_ns = time.monotonic_ns()
                    publisher.publish(s2_rehearsal.cloud_message(stamp))
                    publish_return_ns = time.monotonic_ns()
                    sent.append({
                        "index": index,
                        "planned_ns": deadline_ns,
                        "planned_s": (deadline_ns - origin_ns) / 1e9,
                        "call_start_mono_ns": call_start_ns,
                        "publish_return_mono_ns": publish_return_ns,
                        "actual_s": (call_start_ns - origin_ns) / 1e9,
                        "observation_id": str(stamp),
                        "input_stamp_ns": stamp,
                    })

            sender = threading.Thread(target=publish)
            sender.start()
            chains_started_ns = time.monotonic_ns()
            deadline = time.monotonic() + 8
            while len(sent) < 1 and time.monotonic() < deadline:
                time.sleep(0.01)
            if not sent:
                raise TimeoutError("sender published no cloud")
            chains = []
            rows = []
            queries = []
            for index in range(WIRE_INPUTS):
                q_started_ns = time.monotonic_ns()
                wait_end = time.monotonic() + 8
                while len(sent) <= index and time.monotonic() < wait_end:
                    time.sleep(0.01)
                if len(sent) <= index:
                    raise TimeoutError("sender missed input %d" % index)
                send_wait_ms = (time.monotonic_ns() - q_started_ns) / 1e6
                item = sent[index]
                last, terminal = _wait_integrated(session, item["observation_id"], 8)
                events = terminal.get("receipt_events") or []
                row = {
                    "input_id": "score-%d" % index,
                    "phase": "score",
                    "publish_planned_ns": item["planned_ns"],
                    "publish_end_ns": item["publish_return_mono_ns"],
                    "input_stamp_ns": item["input_stamp_ns"],
                    "clock_domain": "CLOCK_MONOTONIC",
                    "callback_enter_ns": events[0].get("callback_enter_ns") if events else None,
                    "disposition": terminal.get("disposition", "unknown"),
                    "receipt_received": terminal.get("receipt_received", False),
                    "receipt_status": terminal.get("receipt_status"),
                    "status_query": terminal.get("status_query"),
                    "terminal_count": terminal.get("terminal_count", len(events)),
                    "queue_age_ms": None,
                    "observed_revision": None,
                }
                row["receipt_problems"] = _receipt_event_problems(events, item["observation_id"])
                receipt = _field(last, "receipt")
                if receipt is not None:
                    row["queue_age_ms"] = _field(receipt, "queue_ms")
                    row["integrate_ms"] = _field(receipt, "total_ms")
                    identity = _field(receipt, "identity")
                    row["observed_revision"] = _field(identity, "observed_revision")
                rows.append(row)
                expected = {
                    "epoch": 1,
                    "sensor_id": "cam0",
                    "observation_id": item["observation_id"],
                }
                if terminal.get("disposition") == "integrated" and row["observed_revision"] is not None:
                    expected["min_observed_revision"] = int(row["observed_revision"])
                    expected.update(expected_map_contract(geometry, config))
                    pinned = _identity_tuple(_field(receipt, "identity"))
                    expected["receipt_matches_contract"] = pinned[4:] == (
                        expected["geometry_hash"], expected["config_digest"], expected["clock_domain"])
                    chain = _read_snapshot_chain(session, expected)
                else:
                    chain = {"identity_stable": False, "reasons": ["terminal_%s" % terminal.get("disposition")]}
                (out / "o7").mkdir(parents=True, exist_ok=True)
                (out / "o7" / ("chain-%d.json" % index)).write_text(json.dumps(chain, indent=2) + "\n")
                if events:
                    (out / "o7" / ("receipts-%d.json" % index)).write_text(
                        json.dumps(events, indent=2) + "\n")
                chains.append(chain)
                identity = chain.get("identity") or [None, None, None]
                q_ended_ns = time.monotonic_ns()
                queries.append({
                    "query_id": "q-%d" % index,
                    "op": "wait_snapshot" if chain.get("wait_status") == "OK" else "status_query",
                    "status": chain.get("wait_status"),
                    # Per-request boundaries in CLOCK_MONOTONIC. latency_ms is
                    # this request only; send_wait_ms is the share spent
                    # waiting for the sender to reach this input.
                    "started_mono_ns": q_started_ns,
                    "ended_mono_ns": q_ended_ns,
                    "latency_ms": (q_ended_ns - q_started_ns) / 1e6,
                    "send_wait_ms": send_wait_ms,
                    "chain_ms": chain.get("total_ms"),
                    "age_ms": None if chain.get("age_ns") is None else chain["age_ns"] / 1e6,
                    "covers_observation": chain.get("identity_stable") is True,
                    "expected_observation_id": item["observation_id"],
                    "epoch": identity[0],
                    "observed_revision": identity[2],
                    "content_hash": chain.get("content_sha256") or "",
                })
            sender.join(timeout=5)
            if sender.is_alive():
                raise TimeoutError("sender thread did not finish")
            # Aggregate wall time of every query chain plus the join. This is
            # NOT a single ROS round trip; per-request numbers are on each
            # query row, and chains_wall_s keeps the old query_s name honest.
            chains_wall_s = (time.monotonic_ns() - chains_started_ns) / 1e9
            intervals_s = [
                sent[index + 1]["actual_s"] - sent[index]["actual_s"]
                for index in range(len(sent) - 1)]
            interval = sent[-1]["actual_s"] - sent[0]["actual_s"] if len(sent) >= 2 else None
            send_query_timeline = sorted(
                [{"event": "publish_return", "mono_ns": item["publish_return_mono_ns"]}
                 for item in sent]
                + [{"event": "query_start", "mono_ns": query["started_mono_ns"]}
                   for query in queries]
                + [{"event": "query_end", "mono_ns": query["ended_mono_ns"]}
                   for query in queries],
                key=lambda event: event["mono_ns"])
            import yaml
            health = []
            sample_origin = time.monotonic()
            rss = []
            for _ in range(2):
                time.sleep(0.5)
                parsed = None
                for raw in reversed(session.health):
                    loaded = yaml.safe_load(raw)
                    if isinstance(loaded, dict) and "tree_nodes" in loaded:
                        parsed = loaded
                        break
                health.append({"t_s": time.monotonic() - sample_origin, "health": parsed})
                rss.append(_rss_kib(observer.proc.pid))
            certification = None
            try:
                session.wait_service(session.resource, timeout=5)
                from uld_occ_observer.srv import GetResourceUsage
                usage = session.call(session.resource, GetResourceUsage.Request(), 5)
                certification = usage.certification
            except Exception as exc:
                (out / "o7" / "resource_error.txt").write_text(str(exc)[-400:])
            artifact = _assemble_window(window, rows, queries, None, None)
            window_verdict = repair_s0_judge.judge_o7_normal(artifact)
            planned = [item["id"] for item in plan["o7_windows"] if item["kind"] == "normal"]
            campaign = repair_s0_judge.judge_o7_campaign({"windows": [artifact], "planned_window_ids": planned})
            _write(out / "o7" / "window.json", artifact)
            _write(out / "o7" / "campaign_verdict.json", campaign)
            o8 = {
                "gate": "o8",
                "official_duration_s": "not_run",
                "sample_window_s": time.monotonic() - sample_origin,
                "runs": [{
                    "run_id": "wire",
                    "health": health,
                    "rss_samples_kib": rss,
                    "rss_median_kib": [item for item in rss if item is not None],
                    "node_count_stable": len(health) == 2 and health[0]["health"] and health[1]["health"]
                    and health[0]["health"].get("tree_nodes") == health[1]["health"].get("tree_nodes"),
                }],
                "holds": [],
            }
            o8_verdict = repair_s0_judge.judge_o8(o8)
            _write(out / "o8" / "runs.json", o8)
            _write(out / "o8" / "verdict.json", o8_verdict)
            started["o7"] = len(rows)
            started["o8"] = len(o8["runs"])
            # O8 wiring validity: two parseable health samples with distinct
            # stamps and finite node counts, plus readable RSS. Missing
            # samples, repeated timestamps, or an exited process keep the
            # wiring from claiming it executed.
            o8_valid = _o8_health_valid(health, rss)
            chains_ok = all(chain.get("identity_stable") is True for chain in chains)
            if not chains_ok:
                _save_failure(out, "o7", "chain_not_stable",
                              {"chain_reasons": [chain.get("reasons") for chain in chains]},
                              out / "o7")
            receipt_rows_ok = all(not row.get("receipt_problems") for row in rows)
            sender_ok = (
                len(sent) == WIRE_INPUTS
                and all(item["publish_return_mono_ns"] is not None for item in sent)
                and all(abs(step - spacing_s) < 1e-6 for step in
                        (sent[index + 1]["planned_s"] - sent[index]["planned_s"]
                         for index in range(len(sent) - 1)))
                and sender_ident.get("id") not in (None, session.thread.ident))
            o7 = {
                "stage": "o7",
                "started": len(rows),
                "judge": campaign,
                "window_judge": window_verdict,
                # The resource service must have answered with a declared
                # value; which scope it certifies is the product's statement,
                # not something this wiring run evaluates.
                "executed": chains_ok and sender_ok and receipt_rows_ok
                and certification in ("NOT_EVALUATED", "READ_VIEW_CACHE_ATTRIBUTED"),
                "sender": {
                    "planned_s": [item["planned_s"] for item in sent],
                    "actual_s": [item["actual_s"] for item in sent],
                    "interval_s": interval,
                    "intervals_s": intervals_s,
                    # chains_wall_s: both query chains plus the sender join in
                    # wall time. query_s keeps the historical key, equal to it.
                    "chains_wall_s": chains_wall_s,
                    "query_s": chains_wall_s,
                    "send_query_timeline": send_query_timeline,
                    "clock_domain": "CLOCK_MONOTONIC",
                    "sender_thread": sender_ident.get("id"),
                    "spin_thread": session.thread.ident,
                },
                "chains_stable": chains_ok,
                "resource_certification": certification,
            }
            return [
                o7,
                {"stage": "o8", "started": 1, "judge": o8_verdict,
                 "health_valid": o8_valid,
                 "executed": o8_verdict.get("pass") is False
                 and o8["official_duration_s"] == "not_run" and o8_valid},
            ]
        finally:
            if session is not None:
                session.close()
            if observer is not None:
                observer.stop()
            if rclpy.ok():
                rclpy.shutdown()
            if previous_domain is None:
                os.environ.pop("ROS_DOMAIN_ID", None)
            else:
                os.environ["ROS_DOMAIN_ID"] = previous_domain


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
        "smoke-cli", "smoke-ros", "wire"))
    parser.add_argument("--cli")
    parser.add_argument("--eval-native")
    parser.add_argument("--node")
    parser.add_argument("--geometry")
    parser.add_argument("--config")
    parser.add_argument("--stage", choices=("o5", "o6", "o7", "o8", "all"))
    parser.add_argument("--frame-limit", type=int,
                        help="non-scored rehearsal: cap scored frames per group")
    parser.add_argument("--input-limit", type=int,
                        help="non-scored rehearsal: cap inputs/queries per window")
    parser.add_argument("--judge-file")
    parser.add_argument("--bundle")
    parser.add_argument("--o9-root")
    parser.add_argument("--budget-minutes", type=float, default=None,
                        help="stage-mode budget for the scheduler (the S3 "
                             "orchestrator passes the remaining shared budget)")
    parser.add_argument("--o5-evidence", default=None,
                        help="a prior scored O5 verdict.json; lets a resumed "
                             "O6 run without re-scoring O5 (pass:true only)")
    args = parser.parse_args(argv)
    fixtures = args.fixtures or "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures"
    if args.mode == "stage":
        payload = _s3_refused(args.stage or "all")
        adapter_name = os.environ.get("ULD_OCC_S2_ADAPTER", "live")
        selected = {"o5": "profile_cli_groups", "o6": "profile_cli_groups",
                    "o7": "ros_windows", "o8": "ros_stability", "all": "schedule"}
        payload["runner"] = selected.get(args.stage or "all", "schedule")
        payload["dispatched"] = True
        stages = ("o5", "o6", "o7", "o8") if (args.stage or "all") == "all" else (args.stage,)
        if adapter_name == "live":
            payload["adapter"] = "LiveScheduleAdapter"
            payload["launch_boundary"] = "launch_process"
            if not all((args.cli, args.eval_native, args.node, args.geometry, args.config)):
                payload["reason"] = "wiring_binaries_missing"
                payload["started"] = {stage: 0 for stage in ("o5", "o6", "o7", "o8")}
                payload["formal_pass"] = False
                print(json.dumps(payload))
                return 2
            rehearsal = args.frame_limit is not None or args.input_limit is not None
            if not rehearsal:
                # The formal entry keeps both gates: S3 open and an explicit
                # official-samples confirmation. Neither is ever set by tooling.
                if os.environ.get("ULD_OCC_S3_OPEN") != "1":
                    payload["reason"] = "s3_closed"
                    print(json.dumps(payload))
                    return 2
                if os.environ.get("ULD_OCC_S2_CONFIRM_OFFICIAL") != "1":
                    payload["reason"] = "official_samples_not_authorized"
                    print(json.dumps(payload))
                    return 2
            import live_schedule
            plan = campaign_entry.load_plan(fixtures)
            adapter = live_schedule.LiveScheduleAdapter(
                args.cli, args.eval_native, args.node, args.geometry, args.config, args.out,
                frame_limit=args.frame_limit, input_limit=args.input_limit)
            adapter.bind_plan(plan)
            result = None
            try:
                budget_s = (args.budget_minutes * 60.0
                            if args.budget_minutes is not None else BUDGET_S)
                result = schedule(plan, adapter, args.out, stages=stages,
                                  budget_s=budget_s,
                                  frame_limit=args.frame_limit, input_limit=args.input_limit,
                                  scored=not rehearsal,
                                  o5_evidence=args.o5_evidence,
                                  # A rehearsal exercises every stage on the live
                                  # adapter; the O5 dependency gate itself is
                                  # proven offline on the same schedule().
                                  allow_incomplete_dependencies=rehearsal)
                result["runner"] = payload["runner"]
                result["dispatched"] = True
                result["adapter"] = payload["adapter"]
                result["overruns"] = adapter.overruns
            except Exception as exc:
                # The failure bundle is written while the sessions, observer
                # processes, and partial artifacts still exist: teardown comes
                # after the inventory, never before.
                import traceback
                details = traceback.format_exc()
                for stage in stages:
                    _save_failure(args.out, stage, "%s:%s" % (type(exc).__name__, exc),
                                  details)
                payload["reason"] = "stage_exception:%s" % type(exc).__name__
                payload["started"] = {stage: 0 for stage in ("o5", "o6", "o7", "o8")}
                payload["formal_pass"] = False
                adapter.close()
                print(json.dumps(payload))
                return 3
            finally:
                adapter.close()
            print(json.dumps({key: result[key] for key in (
                "status", "official_samples", "product_resource_certification", "formal_pass",
                "scoring", "started", "dispatched", "runner", "adapter")}))
            if rehearsal:
                # Short samples must fail the judges; a completed run is the
                # pass condition for a non-scored rehearsal.
                return 0 if result["status"] == "scheduled" else 1
            failed = any(not item["judge"]["pass"] for item in result["stages"] if item.get("judge"))
            return 1 if failed else 0
        if os.environ.get("ULD_OCC_S3_OPEN") != "1":
            print(json.dumps(payload))
            return 2
        if adapter_name != "injected":
            payload["reason"] = "unknown_adapter"
            print(json.dumps(payload))
            return 2
        plan = campaign_entry.load_plan(fixtures)
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
    if args.mode == "wire":
        plan = campaign_entry.load_plan(fixtures)
        wired = LiveCampaign(
            args.cli, args.eval_native, args.node, args.geometry, args.config).wire(plan, args.out)
        print(json.dumps({
            "adapter": wired["adapter"],
            "scoring": wired["scoring"],
            "official_samples": wired["official_samples"],
            "formal_pass": wired["formal_pass"],
            "wiring_complete": wired["wiring_complete"],
            "short_samples_rejected": wired["short_samples_rejected"],
            "started": wired["started"],
            "reason": wired["reason"],
            "product_resource_certification": "NOT_EVALUATED",
        }))
        return 0 if wired["wiring_complete"] and not wired["formal_pass"] else 1
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
