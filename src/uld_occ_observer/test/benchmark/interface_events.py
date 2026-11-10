"""Event contract for the O5–O7 interface collection.

Scores stay on the historical gates. These helpers only check that a trace can
be joined, that counters are not double-counted, and that a sample count is
not promoted into a percentile it did not earn.
"""
from __future__ import annotations

import hashlib
import math
import struct
from typing import Iterable, Mapping, Optional, Sequence

SCHEMA_VERSION = 1
FIELDS = (
    "schema_version", "run_id", "phase", "event_seq", "event",
    "mono_ns", "pid", "tid", "thread_cpu_ns",
    "request_id", "chain_id", "input_id", "status",
)
# age > max_age is stale. age == max_age is still inside the contract.
STALE_COMPARE = "age_ns > max_age_ns"


def make_event(run_id, phase, event, mono_ns, pid, tid, thread_cpu_ns,
               request_id, chain_id, input_id, status, event_seq, schema_version=SCHEMA_VERSION):
    """Build one event. Unused identity fields stay None; they are not omitted."""
    row = {
        "schema_version": schema_version,
        "run_id": run_id,
        "phase": phase,
        "event_seq": event_seq,
        "event": event,
        "mono_ns": mono_ns,
        "pid": pid,
        "tid": tid,
        "thread_cpu_ns": thread_cpu_ns,
        "request_id": request_id,
        "chain_id": chain_id,
        "input_id": input_id,
        "status": status,
    }
    missing = [name for name in FIELDS if name not in row]
    if missing:
        raise ValueError("missing event fields: %s" % ",".join(missing))
    return row


def validate_trace(events: Sequence[Mapping]):
    """Require a contiguous per-pid sequence and a non-decreasing monotonic clock.

    A missing sequence number, a repeated sequence number, or a backwards
    monotonic timestamp is a trace failure. ROS stamps are not part of this
    order.
    """
    errors = []
    if not events:
        errors.append("empty_trace")
        return {"ok": False, "errors": errors, "dropped": []}
    by_pid = {}
    seen_ids = {}
    for index, event in enumerate(events):
        for name in FIELDS:
            if name not in event:
                errors.append("missing_field:%d:%s" % (index, name))
        if event.get("schema_version") != SCHEMA_VERSION:
            errors.append("schema:%d" % index)
        pid = event.get("pid")
        seq = event.get("event_seq")
        by_pid.setdefault(pid, []).append(event)
        identity = (event.get("event"), event.get("request_id"), event.get("input_id"), event.get("mono_ns"), pid)
        if identity in seen_ids:
            errors.append("duplicate_event:%d" % index)
        seen_ids[identity] = index
    for pid, rows in by_pid.items():
        ordered = sorted(rows, key=lambda item: item.get("event_seq") if item.get("event_seq") is not None else -1)
        sequences = [item.get("event_seq") for item in ordered]
        if any(not isinstance(value, int) or value < 1 for value in sequences):
            errors.append("bad_seq:%s" % pid)
            continue
        if sequences != list(range(sequences[0], sequences[0] + len(sequences))):
            errors.append("gap_or_reorder:%s" % pid)
        by_tid = {}
        for item in ordered:
            by_tid.setdefault(item.get("tid"), []).append(item)
        for tid, tid_rows in by_tid.items():
            monos = [item.get("mono_ns") for item in tid_rows]
            if any(not isinstance(value, int) for value in monos):
                errors.append("bad_mono:%s:%s" % (pid, tid))
            elif any(later < earlier for earlier, later in zip(monos, monos[1:])):
                errors.append("mono_backwards:%s:%s" % (pid, tid))
    dropped = [event for event in events if event.get("event") == "trace_dropped"]
    return {"ok": not errors, "errors": errors, "dropped": dropped}


def publish_state(events: Sequence[Mapping], input_id: str):
    """publish returning is not business acceptance."""
    names = {event.get("event") for event in events if event.get("input_id") == input_id or event.get("request_id") == input_id}
    published = "publish_end" in names or "receipt_publish_end" in names
    accepted = any(event.get("event") == "enqueue_result" and event.get("status") == "accepted"
                   and (event.get("request_id") == input_id or event.get("input_id") == input_id)
                   for event in events)
    return {"published": published, "accepted": accepted, "publish_means_accepted": False}


def unique_rejections(queue_rejected: int, adapter_rejected: int):
    """QUEUE_FULL increments both counters. The unique count is the adapter total."""
    if queue_rejected < 0 or adapter_rejected < 0:
        raise ValueError("negative rejection counter")
    if adapter_rejected < queue_rejected:
        return {"consistent": False, "unique": None, "overlap_queue_full": None,
                "adapter_only": None, "forbidden_sum": queue_rejected + adapter_rejected}
    return {
        "consistent": True,
        "unique": adapter_rejected,
        "overlap_queue_full": queue_rejected,
        "adapter_only": adapter_rejected - queue_rejected,
        "forbidden_sum": queue_rejected + adapter_rejected,
    }


def flags_for_input(input_id: str, events: Sequence[Mapping], planned: bool):
    """Collapse one planned input into the exclusive-account flags.

    `sent` on the result means publish_end was observed. A planned input with
    no publish_end stays in planned_not_sent.
    """
    del planned
    related = [event for event in events
               if event.get("request_id") == input_id or event.get("input_id") == input_id]
    def seen(name, status=None):
        for event in related:
            if event.get("event") != name:
                continue
            if status is None or event.get("status") == status:
                return True
        return False
    accepted = seen("enqueue_result", "accepted")
    rejected = seen("adapter_rejected") or any(
        event.get("event") == "enqueue_result" and event.get("status") not in ("accepted", "", None)
        for event in related)
    row = {
        "input_id": input_id,
        "sent": seen("publish_end"),
        "callback_entered": seen("callback_enter"),
        "accepted": accepted,
        "rejected": rejected,
        "completed": seen("integrate_end"),
        "client_receipt": seen("client_receipt"),
    }
    if accepted and rejected:
        row["inconsistent"] = "accept_and_reject"
    elif row["completed"] and not accepted and not rejected:
        row["inconsistent"] = "complete_without_accept"
    return row


def input_boundaries(events: Sequence[Mapping], input_id: str):
    names = [event.get("event") for event in events
             if event.get("input_id") == input_id or event.get("request_id") == input_id]
    return {
        "callback_entered": "callback_enter" in names,
        "accepted": "enqueue_result" in names and any(
            event.get("event") == "enqueue_result" and event.get("status") == "accepted"
            and (event.get("request_id") == input_id or event.get("input_id") == input_id)
            for event in events),
        "completed": "integrate_end" in names,
        "receipt_published": "receipt_publish_end" in names,
        "client_receipt": "client_receipt" in names,
        "rejected": any(event.get("event") in ("enqueue_result", "adapter_rejected")
                        and event.get("status") not in (None, "accepted", "")
                        and (event.get("request_id") == input_id or event.get("input_id") == input_id)
                        for event in events),
    }


def account_inputs(rows: Sequence[Mapping]):
    """One exclusive bucket per planned input. Later stages do not erase earlier gaps."""
    buckets = {
        "planned_not_sent": [],
        "published_no_callback": [],
        "callback_not_accepted": [],
        "explicit_reject": [],
        "accepted_in_flight": [],
        "complete_no_client_receipt": [],
        "client_terminal": [],
        "unknown": [],
    }
    for row in rows:
        input_id = row["input_id"]
        if row.get("inconsistent"):
            buckets["unknown"].append({"input_id": input_id, "reason": row["inconsistent"]})
            continue
        if not row.get("sent"):
            buckets["planned_not_sent"].append(input_id)
        elif not row.get("callback_entered"):
            buckets["published_no_callback"].append(input_id)
        elif row.get("rejected"):
            buckets["explicit_reject"].append(input_id)
        elif not row.get("accepted"):
            buckets["callback_not_accepted"].append(input_id)
        elif not row.get("completed"):
            buckets["accepted_in_flight"].append(input_id)
        elif not row.get("client_receipt"):
            buckets["complete_no_client_receipt"].append(input_id)
        else:
            buckets["client_terminal"].append(input_id)
    assigned = sum(len(value) for value in buckets.values())
    return {"buckets": buckets, "assigned": assigned, "inputs": len(rows),
            "conserved": assigned == len(rows)}


def parse_health(text: Optional[str]):
    if text is None:
        return {"ok": False, "reason": "missing_payload"}
    try:
        import yaml
        payload = yaml.safe_load(text)
    except Exception as error:  # yaml parser errors are evidence, not a crash
        return {"ok": False, "reason": "parse_error", "detail": type(error).__name__}
    if not isinstance(payload, dict):
        return {"ok": False, "reason": "not_a_map"}
    required = ("waiting", "in_flight", "submitted", "queue_rejected", "adapter_rejected",
                "integrated", "rejected", "duplicates", "stamp_ns")
    missing = [name for name in required if name not in payload]
    if missing:
        return {"ok": False, "reason": "missing_endpoint_fields", "missing": missing, "payload": payload}
    return {"ok": True, "payload": payload}


def query_lateness_ns(planned_mono_ns: int, actual_mono_ns: int):
    return actual_mono_ns - planned_mono_ns


def release_observation(sent: bool, result_status: Optional[str], late: bool):
    if not sent:
        return "release_not_sent"
    if late and result_status is None:
        return "release_timeout"
    if result_status not in (None, "OK"):
        return "release_failed"
    if result_status is None:
        return "release_result_missing"
    return "release_ok"


def roi_content_bytes(states: Sequence[int], log_odds: Sequence[float]):
    """Little-endian int32 state then float64 log-odds, in response order.

    A missing log-odds is not stored as zero. The caller must pass one value
    per state. NaN keeps the IEEE pattern produced by struct.pack.
    """
    if len(states) != len(log_odds):
        raise ValueError("roi length mismatch")
    buffer = bytearray()
    for state, odds in zip(states, log_odds):
        buffer += struct.pack("<id", int(state), float(odds))
    return bytes(buffer)


def surface_content_bytes(columns: Sequence[Mapping]):
    """Column order is the response order. Unoccupied top_z is the byte '-'."""
    buffer = bytearray()
    for column in columns:
        occupied = 1 if column["has_occupied"] else 0
        buffer += struct.pack("<iiiii", int(column["x"]), int(column["y"]),
                              int(column["active_count"]), int(column["known_count"]), occupied)
        if column["has_occupied"]:
            buffer += struct.pack("<d", float(column["top_z"]))
        else:
            if column.get("top_z") not in (None, "-"):
                raise ValueError("unoccupied column must not invent a top_z")
            buffer += b"-"
    return bytes(buffer)


def content_sha256(blob: bytes):
    return hashlib.sha256(blob).hexdigest()


def same_identity_content(rows: Sequence[Mapping]):
    """Rows sharing identity must share content hash. Different identities are not pooled."""
    grouped = {}
    for row in rows:
        grouped.setdefault(tuple(row["identity"]), []).append(row["content_sha256"])
    mismatches = []
    for identity, hashes in grouped.items():
        if any(value != hashes[0] for value in hashes):
            mismatches.append({"identity": identity, "hashes": hashes})
    return {"consistent": not mismatches, "mismatches": mismatches}


def page_status(done: bool, next_cursor: int, returned: int, total: Optional[int]):
    if total is None:
        return {"ok": False, "reason": "total_unknown"}
    if returned < 0 or next_cursor < returned:
        return {"ok": False, "reason": "cursor_missing"}
    expected_done = next_cursor == total
    if done != expected_done:
        return {"ok": False, "reason": "done_mismatch"}
    return {"ok": True, "reason": "complete" if done else "partial"}


def stale_decision(age_ns: int, max_age_ns: int, allow_stale: bool):
    if age_ns < 0 or max_age_ns < 0:
        return "INVALID_QUERY_TIME"
    if age_ns > max_age_ns and not allow_stale:
        return "STALE"
    return "ACCEPT"


def cross_clock_delta(ros_stamp_ns, mono_ns):
    """ROS time and CLOCK_MONOTONIC are different domains. Never subtract them."""
    del ros_stamp_ns, mono_ns
    raise ValueError("clock_domain_mismatch")


def cpu_observation(thread_cpu_ns, process_cpu_ns):
    if thread_cpu_ns is None or process_cpu_ns is None:
        return {"usable": False, "reason": "cpu_channel_missing"}
    return {"usable": True, "thread_cpu_ns": thread_cpu_ns, "process_cpu_ns": process_cpu_ns,
            "substituted": False}


def cache_pools(samples: Sequence[Mapping]):
    pools = {"first": [], "same_revision": [], "revision_changed": []}
    previous = None
    for sample in samples:
        revision = sample["map_revision"]
        if previous is None:
            kind = "first"
        elif revision == previous:
            kind = "same_revision"
        else:
            kind = "revision_changed"
        previous = revision
        pools[kind].append(sample)
    return pools


def overload_covered(queue_full_events: int, spanning_waits: int, middleware_drops: int,
                     max_queue_ms: float, max_in_flight: int):
    """A microsecond handoff or a single in-flight frame is not saturation evidence."""
    real = queue_full_events > 0 or spanning_waits > 0 or middleware_drops > 0
    weak = max_queue_ms > 0 or max_in_flight > 0
    return {"covered": real, "weak_signal_ignored": weak and not real,
            "middleware_observed": middleware_drops > 0}


def nearest_rank_p95(samples: Sequence[float], required_n: Optional[int] = None):
    count = len(samples)
    if required_n is not None and count != required_n:
        return {"status": "descriptive", "n": count, "required_n": required_n, "p95": None,
                "rank": None}
    if count == 0:
        return {"status": "empty", "n": 0, "p95": None, "rank": None}
    ordered = sorted(samples)
    rank = math.ceil(0.95 * count) - 1
    return {"status": "complete", "n": count, "p95": ordered[rank], "rank": rank,
            "rank_1based": rank + 1}


def perturbation_ratio(on_samples: Sequence[float], off_samples: Sequence[float], required_n: int = 10):
    on = nearest_rank_p95(on_samples, required_n)
    off = nearest_rank_p95(off_samples, required_n)
    if on["p95"] is None or off["p95"] is None or off["p95"] == 0:
        return {"status": "not_comparable", "on": on, "off": off, "ratio": None, "within_1_10": False}
    ratio = on["p95"] / off["p95"]
    return {"status": "compared", "on": on, "off": off, "ratio": ratio, "within_1_10": ratio <= 1.10}


def join_query(chain: Mapping, server_events: Sequence[Mapping]):
    """Join a serial RPC by handle, or by a unique server callback inside the client interval.

    Nearest-timestamp matching is rejected. Zero or several callbacks inside
    the interval stay unknown.
    """
    handle = chain.get("handle")
    send = chain["send_mono_ns"]
    result = chain["result_mono_ns"]
    if result < send:
        return {"joined": False, "reason": "interval_inverted"}
    if handle:
        matched = [event for event in server_events
                   if event.get("request_id") == handle and send <= event.get("mono_ns", -1) <= result]
        enters = [event for event in matched if event.get("event") == "query_callback_enter"]
        if len(enters) != 1:
            return {"joined": False, "reason": "handle_callback_not_unique", "matches": len(enters)}
        return {"joined": True, "reason": "handle", "events": matched}
    inside = [event for event in server_events
              if send <= event.get("mono_ns", -1) <= result and event.get("event") == "query_callback_enter"
              and event.get("request_id") == chain.get("service")]
    if len(inside) != 1:
        return {"joined": False, "reason": "interval_not_unique", "matches": len(inside)}
    return {"joined": True, "reason": "unique_interval", "events": inside}


def segment_callback(events: Sequence[Mapping]):
    """Nested durations. Overlapping locks are not added into one wall sum."""
    def first(name, request_id=None):
        for event in events:
            if event.get("event") == name and (request_id is None or event.get("request_id") == request_id):
                return event
        return None

    def delta(begin, end):
        if begin is None or end is None:
            return None
        return end["mono_ns"] - begin["mono_ns"]

    enter = first("query_callback_enter")
    leave = first("query_callback_exit")
    repo_wait = delta(first("repository_lock_attempt"), first("repository_lock_acquired"))
    store_waits = []
    pending = None
    for event in events:
        if event.get("event") == "store_lock_attempt":
            pending = event
        elif event.get("event") == "store_lock_acquired" and pending is not None:
            store_waits.append(event["mono_ns"] - pending["mono_ns"])
            pending = None
    read = delta(first("roi_begin"), first("roi_end"))
    if read is None:
        read = delta(first("surface_begin"), first("surface_end"))
    assemble = delta(first("response_assemble_begin"), first("response_assemble_end"))
    callback = delta(enter, leave)
    thread_cpu = None
    if enter and leave and enter.get("tid") == leave.get("tid"):
        thread_cpu = leave["thread_cpu_ns"] - enter["thread_cpu_ns"]
    return {
        "callback_wall_ns": callback,
        "repository_wait_ns": repo_wait,
        "store_wait_ns": store_waits,
        "read_ns": read,
        "assemble_ns": assemble,
        "service_thread_cpu_ns": thread_cpu,
        "nested": True,
        "sum_forbidden": True,
    }


def recovery_timing(query_linear_mono_ns, writer_done_mono_ns, selected_age_ns, max_age_ns, status):
    """Separate a legal stale rejection from a fresh snapshot that was still refused."""
    if query_linear_mono_ns is None or writer_done_mono_ns is None or selected_age_ns is None:
        return {"class": "not_localized", "reason": "missing_boundary"}
    writer_before_query = writer_done_mono_ns <= query_linear_mono_ns
    decision = stale_decision(selected_age_ns, max_age_ns, False)
    if status == "STALE" and decision == "STALE":
        return {"class": "legal_stale_rejection", "writer_before_query": writer_before_query,
                "selected_was_stale": True}
    if status == "STALE" and decision == "ACCEPT":
        return {"class": "fresh_but_rejected", "writer_before_query": writer_before_query,
                "selected_was_stale": False}
    if status == "OK" and decision == "ACCEPT":
        return {"class": "accepted_fresh", "writer_before_query": writer_before_query,
                "selected_was_stale": False}
    return {"class": "not_localized", "reason": "status_age_mismatch", "decision": decision,
            "writer_before_query": writer_before_query}


def evidence_flushed(footer: Optional[Mapping], bytes_written: int):
    if not footer or footer.get("event") != "trace_footer":
        return {"flushed": False, "reason": "footer_missing"}
    if bytes_written <= 0:
        return {"flushed": False, "reason": "empty_file"}
    return {"flushed": True, "dropped": footer.get("status")}


def iter_pairs(count: int = 10):
    """Five AB pairs then five BA pairs."""
    order = [("off", "on")] * 5 + [("on", "off")] * 5
    if len(order) != count:
        raise ValueError("pair count")
    return order
