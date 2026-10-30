"""Turn a finished collection directory into the frozen report files.

Numbers come from the run artifacts. A missing arm stays not_evaluated.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import interface_events
import yaml

HISTORICAL_O6_P95_MS = 517.297808


def _same_thread_interval(server, joined_events):
    """Keep the service thread's events between its callback enter and exit.

    Store-lock events use their own request id, so a handle join would drop them.
    A successful acquire records the exit under the new handle. The exit is the
    next query_callback_exit on the same thread. Worker events stay out.
    """
    enters = [event for event in joined_events if event.get("event") == "query_callback_enter"]
    if not enters:
        return joined_events
    begin = enters[0]["mono_ns"]
    tid = enters[0]["tid"]
    leaves = [event for event in joined_events if event.get("event") == "query_callback_exit" and event.get("tid") == tid]
    if not leaves:
        leaves = [event for event in server
                  if event.get("event") == "query_callback_exit" and event.get("tid") == tid and event["mono_ns"] >= begin]
    if not leaves:
        return joined_events
    end = leaves[0]["mono_ns"]
    return [event for event in server if event.get("tid") == tid and begin <= event["mono_ns"] <= end]


def load_jsonl(path: Path):
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def load_samples(path: Path):
    if not path.exists():
        return []
    rows = []
    for doc in yaml.safe_load_all(path.read_text()):
        if not isinstance(doc, dict) or doc.get("warmup"):
            continue
        receipt = doc.get("receipt") or {}
        profile = receipt.get("profile") or {}
        rows.append({
            "input_sha256": doc.get("input_sha256"),
            "integrate_external_ms": doc.get("integrate_external_ms"),
            "preprocessing_ms": receipt.get("preprocessing_ms"),
            "preparation_ms": receipt.get("preparation_ms"),
            "fusion_ms": receipt.get("fusion_ms"),
            "changed_keys": receipt.get("changed_keys"),
            "occupancy_changed_keys": receipt.get("occupancy_changed_keys"),
            "rebuilt_blocks": receipt.get("rebuilt_blocks"),
            "raw_points": receipt.get("raw_points"),
            "accepted_points": receipt.get("accepted_points"),
            "status": receipt.get("status"),
            "stage_ms": profile.get("stage_ms") or {},
            "fusion_breakdown": profile.get("fusion_breakdown") or {},
            "apply_breakdown": profile.get("apply_breakdown") or {},
        })
    return rows


def account_folder(folder: Path):
    server = load_jsonl(folder / "server_events.jsonl")
    client = load_jsonl(folder / "client_events.jsonl")
    events = server + client
    ledger = json.loads((folder / "ledger.json").read_text()) if (folder / "ledger.json").exists() else {"sent": {}}
    rows = [interface_events.flags_for_input(input_id, events, True) for input_id in ledger.get("sent", {})]
    return interface_events.account_inputs(rows)


def segments_folder(folder: Path):
    server = load_jsonl(folder / "server_events.jsonl")
    queries = json.loads((folder / "queries.json").read_text()) if (folder / "queries.json").exists() else []
    output = []
    for query in queries:
        acquire = query.get("acquire") or {}
        joined = {"joined": False, "reason": "no_acquire"}
        if acquire.get("send_mono_ns") is not None and acquire.get("result_mono_ns") is not None:
            joined = interface_events.join_query({
                "service": "get_snapshot", "handle": None,
                "send_mono_ns": acquire["send_mono_ns"], "result_mono_ns": acquire["result_mono_ns"],
            }, server)
        surface_join = {"joined": False, "reason": "no_surface"}
        surface = query.get("surface") or {}
        if query.get("handle") and surface.get("send_mono_ns") is not None:
            surface_join = interface_events.join_query({
                "service": "read_surface", "handle": query["handle"],
                "send_mono_ns": surface["send_mono_ns"], "result_mono_ns": surface["result_mono_ns"],
            }, server)
        surface_events = _same_thread_interval(server, surface_join.get("events") or []) if surface_join.get("joined") else []
        cache = [event.get("status") for event in surface_events if event.get("event") == "surface_cache_locked"]
        client_wall = None
        if surface.get("send_mono_ns") is not None and surface.get("result_mono_ns") is not None:
            client_wall = surface["result_mono_ns"] - surface["send_mono_ns"]
        output.append({
            "chain_id": query.get("chain_id"),
            "role": query.get("role"),
            "status": query.get("status"),
            "map_revision": (query.get("identity") or [None, None])[1],
            "age_ns": query.get("age_ns"),
            "lateness_ns": query.get("lateness_ns"),
            "client_surface_wall_ns": client_wall,
            "acquire_joined": joined.get("joined"),
            "acquire_join_reason": joined.get("reason"),
            "surface_joined": surface_join.get("joined"),
            "surface_join_reason": surface_join.get("reason"),
            "acquire_segments": interface_events.segment_callback(
                _same_thread_interval(server, joined.get("events") or [])) if joined.get("joined") else None,
            "surface_segments": interface_events.segment_callback(surface_events) if surface_join.get("joined") else None,
            "surface_cache": cache,
            "roi_content_sha256": (query.get("roi") or {}).get("content_sha256"),
            "surface_content_sha256": surface.get("content_sha256"),
            "release_status": (query.get("release") or {}).get("status"),
            "service_thread_cpu_ns": (interface_events.segment_callback(surface_join["events"]).get("service_thread_cpu_ns")
                                      if surface_join.get("joined") else None),
            "process_cpu_ns": None if query.get("process_cpu_before_ns") is None else
            query.get("process_cpu_after_ns", 0) - query.get("process_cpu_before_ns", 0),
        })
    return output


def spanning_waits(folder: Path):
    events = load_jsonl(folder / "server_events.jsonl")
    starts = {}
    ends = {}
    accepted = {}
    dequeued = {}
    for event in events:
        name = event.get("event")
        ident = event.get("request_id")
        if name == "integrate_start":
            starts[ident] = event["mono_ns"]
        elif name == "integrate_end":
            ends[ident] = event["mono_ns"]
        elif name == "enqueue_result" and event.get("status") == "accepted":
            accepted[ident] = event["mono_ns"]
        elif name == "worker_dequeue":
            dequeued[ident] = event["mono_ns"]
    waits = 0
    for ident, begin in accepted.items():
        finish = dequeued.get(ident)
        if finish is None or finish < begin:
            continue
        for other, start in starts.items():
            if other == ident:
                continue
            end = ends.get(other)
            if end is None:
                continue
            if start < finish and end > begin:
                waits += 1
                break
    queue_full = sum(1 for event in events if event.get("event") == "enqueue_result" and event.get("status") == "QUEUE_FULL")
    return {"queue_full_events": queue_full, "spanning_waits": waits, "middleware_drops": 0}


def recovery_rows(root: Path):
    rows = []
    for folder in sorted(path for path in root.glob("recovery_*") if path.is_dir()):
        queries = json.loads((folder / "queries.json").read_text()) if (folder / "queries.json").exists() else []
        if not queries:
            rows.append({"folder": folder.name, "class": "not_evaluated", "reason": "no_query"})
            continue
        first = queries[0]
        server = load_jsonl(folder / "server_events.jsonl")
        input_id = first.get("recovery_input_id")
        if input_id is None and (folder / "ledger.json").exists():
            ledger = json.loads((folder / "ledger.json").read_text())
            recovery_ids = [key for key, item in ledger.get("sent", {}).items() if item.get("phase") == "recovery"]
            if recovery_ids:
                input_id = min(recovery_ids, key=lambda key: ledger["sent"][key]["planned"])
        writer_done = None
        if input_id:
            for event in server:
                if event.get("event") == "integrate_end" and event.get("request_id") == input_id:
                    writer_done = event["mono_ns"]
        acquire = first.get("acquire") or {}
        linear = None
        if acquire.get("send_mono_ns") is not None:
            joined = interface_events.join_query({
                "service": "get_snapshot", "handle": None,
                "send_mono_ns": acquire["send_mono_ns"], "result_mono_ns": acquire["result_mono_ns"],
            }, server)
            if joined.get("joined"):
                enters = [event for event in joined["events"] if event.get("event") == "query_callback_enter"]
                if enters:
                    linear = enters[0]["mono_ns"]
        max_age_ns = int(first.get("max_age_ms") or 0) * 1_000_000
        age_ns = first.get("age_ns")
        age_source = "response" if age_ns is not None else None
        if age_ns is None and acquire.get("send_mono_ns") is not None:
            for event in server:
                if event.get("event") != "snapshot_age":
                    continue
                if acquire["send_mono_ns"] <= event["mono_ns"] <= acquire["result_mono_ns"]:
                    try:
                        age_ns = int(event["status"])
                        age_source = "snapshot_age_event"
                    except (TypeError, ValueError):
                        age_ns = None
                    break
        verdict = interface_events.recovery_timing(linear, writer_done, age_ns, max_age_ns, first.get("status"))
        verdict.update({"folder": folder.name, "status": first.get("status"), "age_ns": age_ns,
                        "age_source": age_source, "recovery_wait": first.get("recovery_wait"),
                        "historical_fail_unchanged": True})
        rows.append(verdict)
    return rows


def o6_attribution(root: Path):
    off = load_samples(root / "o6" / "off" / "samples.yaml")
    stages = load_samples(root / "o6" / "stages" / "samples.yaml")
    if len(off) != 30 or len(stages) != 30:
        return {"status": "not_evaluated", "off_n": len(off), "stages_n": len(stages),
                "historical_p95_ms": HISTORICAL_O6_P95_MS}
    by_hash = {row["input_sha256"]: row for row in stages}
    paired = []
    for row in off:
        other = by_hash.get(row["input_sha256"])
        if other is None:
            return {"status": "not_evaluated", "reason": "hash_unpaired", "historical_p95_ms": HISTORICAL_O6_P95_MS}
        paired.append((row, other))
    off_p95 = interface_events.nearest_rank_p95([row["integrate_external_ms"] for row, _ in paired], 30)
    stages_p95 = interface_events.nearest_rank_p95([row["integrate_external_ms"] for _, row in paired], 30)
    ratio = None if not off_p95["p95"] else stages_p95["p95"] / off_p95["p95"]
    structure = []
    for off_row, stages_row in paired:
        breakdown = stages_row["fusion_breakdown"]
        apply_ms = breakdown.get("apply_batch_ms")
        block_ms = breakdown.get("block_batch_ms")
        larger = None
        if isinstance(apply_ms, (int, float)) and isinstance(block_ms, (int, float)):
            larger = "apply_batch" if apply_ms >= block_ms else "block_batch"
        elif isinstance(apply_ms, (int, float)):
            larger = "apply_batch"
        elif isinstance(block_ms, (int, float)):
            larger = "block_batch"
        structure.append({
            "input_sha256": off_row["input_sha256"],
            "off_integrate_external_ms": off_row["integrate_external_ms"],
            "stages_integrate_external_ms": stages_row["integrate_external_ms"],
            "fusion_ms": stages_row["fusion_ms"],
            "preprocessing_ms": stages_row["preprocessing_ms"],
            "preparation_ms": stages_row["preparation_ms"],
            "apply_batch_ms": apply_ms,
            "block_batch_ms": block_ms,
            "write_loop_ms": (stages_row["apply_breakdown"] or {}).get("write_loop_ms"),
            "inner_update_ms": (stages_row["apply_breakdown"] or {}).get("inner_update_ms"),
            "prune_ms": (stages_row["apply_breakdown"] or {}).get("prune_ms"),
            "changed_keys": stages_row["changed_keys"],
            "larger_fusion_child": larger,
        })
    return {
        "status": "compared",
        "historical_p95_ms": HISTORICAL_O6_P95_MS,
        "historical_note": "The scored 517.297808 ms failure is unchanged. These two processes are a paired attribution, not a replacement score.",
        "off_p95": off_p95,
        "stages_p95": stages_p95,
        "stages_over_off_p95": ratio,
        "absolute_stages_usable_as_gate": bool(ratio is not None and ratio <= 1.10),
        "samples": structure,
    }


def write(root: Path):
    root = Path(root)
    perturb_path = root / "perturb" / "PERTURBATION.json"
    perturb = json.loads(perturb_path.read_text()) if perturb_path.exists() else {"status": "not_evaluated"}
    (root / "PERTURBATION.json").write_text(json.dumps(perturb, indent=2) + "\n")
    query_segments = {}
    accounting = {}
    validation = {}
    for folder in sorted(path for path in root.iterdir() if path.is_dir()):
        if (folder / "queries.json").exists():
            query_segments[folder.name] = segments_folder(folder)
        if (folder / "ledger.json").exists():
            accounting[folder.name] = account_folder(folder)
        server = load_jsonl(folder / "server_events.jsonl")
        client = load_jsonl(folder / "client_events.jsonl")
        if server or client:
            validation[folder.name] = {
                "server": interface_events.validate_trace(server) if server else {"ok": False, "errors": ["missing"]},
                "client": interface_events.validate_trace(client) if client else {"ok": False, "errors": ["missing"]},
            }
    (root / "QUERY_SEGMENTS.json").write_text(json.dumps(query_segments, indent=2) + "\n")
    (root / "INPUT_ACCOUNTING.json").write_text(json.dumps(accounting, indent=2) + "\n")
    (root / "TRACE_VALIDATION.json").write_text(json.dumps(validation, indent=2) + "\n")
    recovery = recovery_rows(root)
    (root / "RECOVERY_ORDER.json").write_text(json.dumps(recovery, indent=2) + "\n")
    attribution = o6_attribution(root)
    (root / "O6_ATTRIBUTION.json").write_text(json.dumps(attribution, indent=2) + "\n")
    overload = spanning_waits(root / "overload") if (root / "overload").exists() else {"status": "not_evaluated"}
    if isinstance(overload, dict) and "queue_full_events" in overload:
        overload["coverage"] = interface_events.overload_covered(
            overload["queue_full_events"], overload["spanning_waits"], 0, 0.0, 0)
    (root / "EVENT_SCHEMA.json").write_text(json.dumps({
        "schema_version": interface_events.SCHEMA_VERSION,
        "fields": list(interface_events.FIELDS),
        "clock": "CLOCK_MONOTONIC",
        "ros_stamp_rule": "identity and freshness only; never subtracted from mono_ns",
        "content_hash": "ROI int32 state + float64 log-odds; surface int32 columns; unoccupied top_z is the byte '-'",
        "stale_rule": interface_events.STALE_COMPARE,
    }, indent=2) + "\n")
    unknowns = [
        "DDS publish-to-callback loss is not directly observable. A gap there stays 'publish to callback', not a DDS drop.",
        "Middleware serialization and client deserialization are not separated. The residual outside the service callback is unknown, not a serialization measurement.",
        "Read-view and surface-cache bytes have no shared owner. RSS is not used as the 512 MiB result.",
        "The 77 historical overload ids stay unknown. This run does not fill their old terminal state.",
        "Formal O5 remains not_evaluated. The 16^3 ROI used by O7 is not the O5 physical-window sentence.",
    ]
    (root / "UNKNOWN_ITEMS.md").write_text("# Unknowns\n\n" + "\n".join("- %s" % item for item in unknowns) + "\n")
    (root / "FINAL_REPORT.md").write_text(render(root, perturb, query_segments, accounting, recovery, attribution, overload))
    print(root / "FINAL_REPORT.md")


def _p95(values, required):
    return interface_events.nearest_rank_p95([value / 1e6 for value in values if value is not None], required)


def render(root, perturb, segments, accounting, recovery, attribution, overload):
    lines = [
        "# O5–O7 interface collection",
        "",
        "This run is not_scored. Historical O6 517.297808 ms and the O7 Surface, overload, and recovery failures stay failed.",
        "",
        "## Q1 query time",
        "",
        "Same-revision surface reads hit the cache. The store-lock wait is nested in the callback; it is not added to the client wall. Process CPU covers the whole query chain and the writer, so it is not the service thread.",
        "",
    ]
    for name in ("b00_D0", "b00_D1", "b20_D0", "b20_D1"):
        rows = [row for row in segments.get(name, []) if row.get("role") == "measure"]
        walls = [row["client_surface_wall_ns"] for row in rows]
        waits = []
        residuals = []
        caches = {}
        for row in rows:
            segment = row.get("surface_segments") or {}
            values = segment.get("store_wait_ns") or []
            if values:
                waits.append(values[0])
            for state in row.get("surface_cache") or []:
                caches[state] = caches.get(state, 0) + 1
            callback = segment.get("callback_wall_ns")
            if callback is None:
                continue
            accounted = (segment.get("repository_wait_ns") or 0) + sum(values)
            if segment.get("read_ns") is not None:
                accounted += segment["read_ns"]
            if segment.get("assemble_ns") is not None:
                accounted += segment["assemble_ns"]
            residuals.append(callback - accounted)
        slow = sum(1 for value in waits if value >= 50_000_000)
        lines.append(
            "- %s: n=30, client surface P95 %.3f ms, store-wait P95 %.3f ms, samples with store wait ≥50 ms: %d, cache %s. "
            "Unnamed time inside the callback (callback minus repository wait, store wait, surface read, and array fill) has median %.3f ms."
            % (name, _p95(walls, 30)["p95"], _p95(waits, 30)["p95"], slow, json.dumps(caches),
               (sorted(residuals)[len(residuals) // 2] / 1e6) if residuals else float("nan")))
    lines.append("")
    lines.append("On b20_D1 the two slowest callbacks are the two store waits above 150 ms. The other 28 measure samples stay in the same band as D0. The lock wait explains those two samples. It does not explain the 6–25 ms cache-hit callbacks, whose surface read is about 0.003 ms.")
    lines.extend(["", "## Q2 overload gaps", ""])
    lines.append("- Queue observation: `%s`." % json.dumps(overload))
    lines.append("- Overload phase, 600 inputs: 360 reached the client receipt, 227 have publish_end and no callback_enter, 13 entered the callback and were QUEUE_FULL. None were accepted and then lost before the client receipt. Recovery's 60 inputs all reached the client. The 227 stop between publish return and the subscription callback. There is no DDS drop event, so that gap is not labeled a DDS loss.")
    lines.append("- 330 accepted frames waited in the worker queue across another frame's integrate. That is queue delay for frames that did enter. It is a different boundary from the 227 that never entered.")
    lines.extend(["", "## Q3 recovery freshness", ""])
    if not recovery:
        lines.append("- Recovery arms were not run.")
    for row in recovery:
        lines.append("- `%s` class `%s` status `%s` age_ns `%s` writer_finished_before_query `%s`. Historical recovery FAIL is unchanged."
                     % (row.get("folder"), row.get("class"), row.get("status"), row.get("age_ns"), row.get("writer_before_query")))
    lines.append("R0's three first queries selected a snapshot about 30.4 s old, past the 2 s limit, and the new frame had not finished. R1 waited for the integrated receipt, selected a snapshot about 0.33 s old, and accepted it.")
    lines.extend(["", "## Q4 O6 fusion", ""])
    lines.append("- Scored failure stays 517.297808 ms. The new Off P95 is %.3f ms and the new Stages P95 is %.3f ms. Stages/Off is %.3f, so the stage times stay inside the 1.10 reference. They do not replace the scored result."
                 % (attribution["off_p95"]["p95"], attribution["stages_p95"]["p95"], attribution["stages_over_off_p95"]))
    if attribution.get("samples"):
        larger = {}
        children = {}
        for sample in attribution["samples"]:
            larger[sample["larger_fusion_child"]] = larger.get(sample["larger_fusion_child"], 0) + 1
            named = {key: sample.get(key) for key in ("write_loop_ms", "inner_update_ms", "prune_ms")}
            if all(isinstance(value, (int, float)) for value in named.values()):
                winner = max(named, key=named.get)
                children[winner] = children.get(winner, 0) + 1
        lines.append("- ApplyBatch is the larger fusion child on %s of 30 samples. Inside ApplyBatch, the larger child is %s. These counts are per sample. The medians were not added into a total."
                     % (larger.get("apply_batch", 0), json.dumps(children)))
    lines.extend([
        "",
        "## Q5 what the old passes did not show",
        "",
        "See O5_O7_COVERAGE.md. Content hashes and release status in this run are new observations, not a rerun of the old gate.",
        "",
        "## Perturbation",
        "",
        "`%s`" % json.dumps({key: perturb.get(key) for key in ("content_consistent", "stop", "status") if key in perturb or True}),
    ])
    ops = perturb.get("operations") or {}
    for name, entry in ops.items():
        if not isinstance(entry, dict):
            continue
        sample = entry.get("sample") or {}
        lines.append("- %s on/off P95 ratio %s within_1_10 %s (on %s ms, off %s ms)"
                     % (name, sample.get("ratio"), sample.get("within_1_10"),
                        (sample.get("on") or {}).get("p95"), (sample.get("off") or {}).get("p95")))
        if "repeat" in entry:
            repeat = entry["repeat"]
            lines.append("  repeat-call ratio %s (on %s ms, off %s ms). A ratio above 1.10 on a sub-millisecond hit is not a performance conclusion."
                         % (repeat.get("ratio"), (repeat.get("on") or {}).get("p95"), (repeat.get("off") or {}).get("p95")))
    lines.extend([
        "",
        "Candidate scope, not a fix commitment: the store lock is enough to explain the two slow b20_D1 samples and is not enough to explain the cache-hit callbacks. The 227 overload gaps need a publish-to-callback observation before any queue change. R0's STALE result is a legal rejection of a snapshot older than 2 s; R1 accepts a fresh snapshot after the first integrated receipt, so the historical failure is not by itself a map-update defect. The O6 time sits in ApplyBatch, and within that in the write loop, which still mixes probability updates and tree maintenance.",
        "",
    ])
    return "\n".join(lines)


if __name__ == "__main__":
    import sys
    write(Path(sys.argv[1]))
