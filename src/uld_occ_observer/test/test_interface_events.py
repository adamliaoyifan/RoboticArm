"""Positive and negative controls for the interface-collection contract."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "test" / "benchmark"))

import interface_events as events


def _event(seq, name, mono, request_id="", status="", input_id=None, pid=7, tid=8):
    return events.make_event("run", "writer", name, mono, pid, tid, mono // 10,
                             request_id, None, input_id, status, seq)


def test_trace_accepts_ordered_events_and_rejects_gaps_duplicates_and_rewind():
    good = [_event(1, "callback_enter", 10, "a"), _event(2, "enqueue_result", 11, "a", "accepted")]
    assert events.validate_trace(good)["ok"]
    gap = [_event(1, "callback_enter", 10, "a"), _event(3, "enqueue_result", 11, "a", "accepted")]
    assert "gap_or_reorder:7" in events.validate_trace(gap)["errors"]
    rewind = [_event(1, "callback_enter", 12, "a"), _event(2, "enqueue_result", 11, "a", "accepted")]
    assert "mono_backwards:7:8" in events.validate_trace(rewind)["errors"]
    other = _event(2, "enqueue_result", 9, "b", tid=9)
    crossed = [_event(1, "callback_enter", 12, "a"), other]
    assert events.validate_trace(crossed)["ok"]
    duplicate = [_event(1, "callback_enter", 10, "a"), _event(1, "callback_enter", 10, "a")]
    report = events.validate_trace(duplicate)
    assert any(item.startswith("duplicate_event") or item.startswith("gap_or_reorder") for item in report["errors"])


def test_publish_is_not_acceptance_and_queue_full_is_not_counted_twice():
    published = [_event(1, "publish_end", 10, input_id="in-1")]
    state = events.publish_state(published, "in-1")
    assert state["published"] and not state["accepted"] and not state["publish_means_accepted"]
    unique = events.unique_rejections(4, 6)
    assert unique["unique"] == 6
    assert unique["overlap_queue_full"] == 4
    assert unique["forbidden_sum"] == 10
    assert unique["forbidden_sum"] != unique["unique"]
    assert events.unique_rejections(3, 2)["consistent"] is False


def test_joined_flags_keep_completion_and_receipt_apart():
    rows = [
        events.flags_for_input("a", [
            _event(1, "publish_end", 1, input_id="a"),
            _event(2, "callback_enter", 2, "a"),
            _event(3, "enqueue_result", 3, "a", "accepted"),
            _event(4, "integrate_end", 4, "a", "INTEGRATED"),
        ], True),
        events.flags_for_input("b", [
            _event(1, "publish_end", 1, input_id="b"),
            _event(2, "callback_enter", 2, "b"),
            _event(3, "enqueue_result", 3, "b", "accepted"),
        ], True),
    ]
    rows[0]["client_receipt"] = False
    account = events.account_inputs(rows)
    assert account["buckets"]["complete_no_client_receipt"] == ["a"]
    assert account["buckets"]["accepted_in_flight"] == ["b"]
    missing = events.flags_for_input("c", [], True)
    assert events.account_inputs([missing])["buckets"]["planned_not_sent"] == ["c"]


def test_completion_and_client_receipt_are_separate_buckets():
    rows = [
        {"input_id": "a", "sent": True, "callback_entered": True, "accepted": True,
         "completed": True, "client_receipt": False, "rejected": False},
        {"input_id": "b", "sent": True, "callback_entered": True, "accepted": True,
         "completed": False, "client_receipt": False, "rejected": False},
        {"input_id": "c", "sent": True, "callback_entered": False, "accepted": False,
         "completed": False, "client_receipt": False, "rejected": False},
        {"input_id": "d", "sent": False, "callback_entered": False, "accepted": False,
         "completed": False, "client_receipt": False, "rejected": False},
    ]
    account = events.account_inputs(rows)
    assert account["conserved"]
    assert account["buckets"]["complete_no_client_receipt"] == ["a"]
    assert account["buckets"]["accepted_in_flight"] == ["b"]
    assert account["buckets"]["published_no_callback"] == ["c"]
    assert account["buckets"]["planned_not_sent"] == ["d"]


def test_health_parse_and_missing_endpoint_fields():
    import yaml
    text = yaml.safe_dump({"waiting": 0, "in_flight": 0, "submitted": 1, "queue_rejected": 0,
                           "adapter_rejected": 0, "integrated": 1, "rejected": 0, "duplicates": 0,
                           "stamp_ns": 5})
    assert events.parse_health(text)["ok"]
    assert events.parse_health(None)["reason"] == "missing_payload"
    assert events.parse_health("waiting: [")["reason"] == "parse_error"
    partial = yaml.safe_dump({"waiting": 0})
    assert events.parse_health(partial)["reason"] == "missing_endpoint_fields"


def test_query_lateness_release_and_content_identity():
    assert events.query_lateness_ns(100, 150) == 50
    assert events.release_observation(True, None, True) == "release_timeout"
    assert events.release_observation(True, "ERROR", False) == "release_failed"
    assert events.release_observation(True, "OK", False) == "release_ok"
    blob = events.roi_content_bytes([2, 0], [0.5, float("nan")])
    assert events.content_sha256(blob) == events.content_sha256(blob)
    columns = [{"x": 1, "y": 2, "active_count": 3, "known_count": 1, "has_occupied": False, "top_z": None}]
    assert events.surface_content_bytes(columns)
    same = events.same_identity_content([
        {"identity": (1, 2), "content_sha256": "aa"},
        {"identity": (1, 2), "content_sha256": "bb"},
    ])
    assert not same["consistent"]
    assert events.same_identity_content([
        {"identity": (1, 2), "content_sha256": "aa"},
        {"identity": (1, 3), "content_sha256": "bb"},
    ])["consistent"]


def test_page_and_stale_boundaries():
    assert events.page_status(True, 4096, 4096, 4096)["ok"]
    assert events.page_status(False, 10, 10, None)["reason"] == "total_unknown"
    assert events.page_status(True, 10, 10, 12)["reason"] == "done_mismatch"
    assert events.stale_decision(2000, 2000, False) == "ACCEPT"
    assert events.stale_decision(2001, 2000, False) == "STALE"
    try:
        events.cross_clock_delta(10, 20)
    except ValueError as error:
        assert str(error) == "clock_domain_mismatch"
    else:
        raise AssertionError("cross clock subtraction was allowed")


def test_cpu_channels_cache_pools_and_overload_predicate():
    cpu = events.cpu_observation(10, 30)
    assert cpu["usable"] and cpu["thread_cpu_ns"] != cpu["process_cpu_ns"]
    assert not events.cpu_observation(None, 30)["usable"]
    pools = events.cache_pools([
        {"map_revision": 1, "ms": 20},
        {"map_revision": 1, "ms": 4},
        {"map_revision": 2, "ms": 18},
    ])
    assert [item["ms"] for item in pools["first"]] == [20]
    assert [item["ms"] for item in pools["same_revision"]] == [4]
    assert [item["ms"] for item in pools["revision_changed"]] == [18]
    assert not events.overload_covered(0, 0, 0, 0.01, 1)["covered"]
    assert events.overload_covered(1, 0, 0, 0.0, 0)["covered"]
    assert events.overload_covered(0, 0, 0, 0.01, 1)["weak_signal_ignored"]


def test_sample_count_and_perturbation_rank():
    short = events.nearest_rank_p95(list(range(29)), required_n=30)
    assert short["status"] == "descriptive" and short["p95"] is None
    ten = events.nearest_rank_p95(list(range(1, 11)), required_n=10)
    assert ten["rank_1based"] == 10 and ten["p95"] == 10
    ratio = events.perturbation_ratio([11] * 10, [10] * 10)
    assert ratio["within_1_10"] and abs(ratio["ratio"] - 1.1) < 1e-9
    over = events.perturbation_ratio([12] * 10, [10] * 10)
    assert not over["within_1_10"]


def test_query_join_requires_a_unique_interval_and_segments_stay_nested():
    server = [
        _event(1, "query_callback_enter", 100, "get_snapshot", pid=1, tid=2),
        _event(2, "repository_lock_attempt", 101, "acquire", pid=1, tid=2),
        _event(3, "repository_lock_acquired", 130, "acquire", pid=1, tid=2),
        _event(4, "store_lock_attempt", 131, "snapshot", pid=1, tid=2),
        _event(5, "store_lock_acquired", 180, "snapshot", pid=1, tid=2),
        _event(6, "store_lock_release", 190, "snapshot", "hit", pid=1, tid=2),
        _event(7, "query_callback_exit", 200, "get_snapshot", pid=1, tid=2),
    ]
    # Fix thread cpu so the delta is meaningful: rebuild last event's cpu via make_event path.
    for item, cpu in zip(server, (10, 11, 12, 13, 14, 15, 40)):
        item["thread_cpu_ns"] = cpu
    joined = events.join_query({"service": "get_snapshot", "handle": None, "send_mono_ns": 90,
                                 "result_mono_ns": 210}, server)
    assert joined["joined"] and joined["reason"] == "unique_interval"
    ambiguous = events.join_query({"service": "get_snapshot", "handle": None, "send_mono_ns": 0,
                                    "result_mono_ns": 1000}, server + [
                                        _event(8, "query_callback_enter", 300, "get_snapshot", pid=1, tid=3)])
    assert not ambiguous["joined"]
    nearest = events.join_query({"service": "get_snapshot", "handle": None, "send_mono_ns": 0,
                                  "result_mono_ns": 50}, server)
    assert not nearest["joined"]
    segments = events.segment_callback(server)
    assert segments["repository_wait_ns"] == 29
    assert segments["store_wait_ns"] == [49]
    assert segments["service_thread_cpu_ns"] == 30
    assert segments["sum_forbidden"]


def test_recovery_classes_and_flush():
    legal = events.recovery_timing(200, 100, 2001, 2000, "STALE")
    assert legal["class"] == "legal_stale_rejection"
    fresh = events.recovery_timing(200, 100, 1000, 2000, "STALE")
    assert fresh["class"] == "fresh_but_rejected"
    missing = events.recovery_timing(None, 100, 1000, 2000, "STALE")
    assert missing["class"] == "not_localized"
    assert not events.evidence_flushed(None, 10)["flushed"]
    footer = _event(1, "trace_footer", 10, status="0")
    footer["event"] = "trace_footer"
    assert events.evidence_flushed(footer, 12)["flushed"]
    assert events.iter_pairs()[:5] == [("off", "on")] * 5
    assert events.iter_pairs()[5:] == [("on", "off")] * 5
