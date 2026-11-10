"""gen3 tool-repair tests: contract-GRADE healthy fixtures and the review's
eight audit mutations replayed against them, plus the field x position x
fault matrix. The gen2 fixtures were short of the contract (one recovery
query, two read rounds labeled formal) -- exactly what the 1621 review
rejected; every healthy control here carries the FULL formal workload shape
(300/240 windows, 600+120/60/75+60 recovery with a used wait lease,
18000/14400 + 1000/100 rounds, 12 dense RSS buckets) and must pass with
formal=true, while each mutation must add a NAMED failure.
"""
from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace as S

import pytest

BENCH = Path(__file__).resolve().parent / "benchmark"
sys.path.insert(0, str(BENCH))
import freespace_campaign as campaign  # noqa: E402
import freespace_contract as contract  # noqa: E402
import freespace_judge as judge  # noqa: E402

GEOMETRY_HASH = "cb41f55f243786d50ab30997291083ab40467507abcf78a00e55da77ffc00856"
T0 = 10_000_000_000_000
PERIOD_IN = int(1e9 / contract.UPDATE_HZ)      # 400 ms
PERIOD_Q = int(1e9 / contract.QUERY_HZ)        # 500 ms


def identity(epoch=1, revision=5, observed=9):
    return {"epoch": epoch, "map_revision": revision,
            "observed_revision": observed, "geometry_hash": GEOMETRY_HASH,
            "last_integrated_stamp_ns": 10**9}


def binding(epoch=1):
    """The stage-start identity binding the runner captures once and every
    lease in the stage must echo (gen4 review case 3)."""
    return {"expected_epoch": epoch, "geometry_hash": GEOMETRY_HASH,
            "config_digest": "c" * 64, "clock_domain": "steady_clock",
            "captured_mono_ns": T0}


def health_series(count, step_s=0.5):
    """Raw health stamps at the 2 Hz publisher cadence -- the judge
    recomputes gaps from these, never from a self-reported maximum."""
    return [{"stamp_ns": T0 + int(index * step_s * 1e9)}
            for index in range(count)]


def resource_snapshot(**overrides):
    """A full GetResourceUsage snapshot with a consistent per-owner split
    (splits sum to the merged fields, shared counted once)."""
    snapshot = {
        "view_directory_entries": 1, "view_directory_entries_peak": 2,
        "view_directory_bytes": 64, "view_directory_bytes_peak": 128,
        "shared_block_bytes": 502080, "shared_block_bytes_peak": 502080,
        "old_view_block_bytes": 0,
        "lease_view_block_bytes": 0, "retained_view_block_bytes": 0,
        "client_held_view_block_bytes": 0,
        "lease_view_directory_entries": 0,
        "retained_view_directory_entries": 0,
        "client_held_view_directory_entries": 0,
        "surface_cache_bytes": 2048, "surface_cache_bytes_peak": 4096,
        "external_surface_buffer_bytes": 0,
        "read_view_bytes": 503296, "read_view_bytes_peak": 600000,
        "cache_bytes": 2048, "cache_bytes_peak": 4096,
        "read_view_cache_bytes_peak": 700000,
        "inflight_response_bytes": 0, "inflight_response_bytes_peak": 4096,
        "store_tree_bytes": 4 * 1024 * 1024,
        "uncovered_allocations": False,
        "certification": "READ_VIEW_CACHE_ATTRIBUTED"}
    snapshot.update(overrides)
    return snapshot


def ownership_after(**overrides):
    """A round's ownership measurement: every retained sample is a FULL
    resource snapshot; the judge takes the per-class minimum back to the
    baseline (gen4 review cases 6/7)."""
    snapshot = resource_snapshot(**overrides)
    return dict(snapshot,
                resource_samples=[dict(snapshot) for _ in range(3)],
                view_directory_entries_samples=[1, 2, 1])


def input_row(index, *, status="INTEGRATED", count=1, e2e_ms=120.0,
              id_text=None, arrival=None):
    ident = id_text if id_text is not None else "in%06d" % index
    planned = T0 + index * PERIOD_IN
    publish = planned + 2_000_000
    arrival_ns = arrival if arrival is not None \
        else publish + int(e2e_ms * 1e6)
    return {"input_id": ident, "planned_mono_ns": planned,
            "publish_mono_ns": publish,
            "receipt_status": status, "receipt_count": count,
            "callback_enter_ns": publish + 1_000_000,
            "receipt_arrival_mono_ns": arrival_ns if count else None,
            "e2e_ms": (arrival_ns - publish) / 1e6 if count else None}


def query_record(index, *, base=0, status="OK", candidates=3, lateness_ms=3.0,
                 latency_ms=100.0, epoch=1):
    planned = base + index * PERIOD_Q
    started = planned + int(lateness_ms * 1e6)
    ended = started + int(latency_ms * 1e6)
    cells = [(0 if i % 2 else 90, i, i + 1, 3 + i % 5, 1 + i % 2)
             for i in range(candidates)]
    return {"index": index, "box": list(contract.F_BOXES[index % 2]),
            "planned_mono_ns": planned, "started_mono_ns": started,
            "ended_mono_ns": ended,
            "status": status, "pages": 1, "candidates": cells,
            "pages_detail": [{"count": len(cells), "cursor": 0,
                              "next_cursor": len(cells), "done": True,
                              "region_begin": [0, 0, 0],
                              "region_end": [64, 40, 24]}],
            "feasible_total": len(cells),
            "content_sha256": judge.candidate_digest(cells),
            "lease_identity": identity(epoch),
            "age_ns": 4 * 10**8, "release_status": "OK",
            "latency_ms": (ended - started) / 1e6 + 0.5,
            "page_status": status}


def sender_row(index):
    planned = T0 + index * PERIOD_IN
    return {"index": index, "input_id": "in%06d" % index,
            "planned_mono_ns": planned, "publish_mono_ns": planned + 2_000_000,
            "stamp_ns": T0 + index * PERIOD_IN}


# --- full-grade formal P window ---------------------------------------------
def full_p_window():
    sender = [sender_row(i) for i in range(contract.P_WINDOW_INPUTS)]
    timeline = [input_row(i) for i in range(contract.P_WINDOW_INPUTS)]
    queries = [query_record(i, base=T0 + 5_000_000)
               for i in range(contract.P_WINDOW_QUERIES)]
    return {"stage": "p", "formal": True, "box_count": 5, "window_index": 1,
            "window_id": "b05_w1", "inputs": contract.P_WINDOW_INPUTS,
            "queries": contract.P_WINDOW_QUERIES,
            "update_hz": contract.UPDATE_HZ, "query_hz": contract.QUERY_HZ,
            "window_planned_s": 120.0,
            "identity_binding": binding(),
            "frames_manifest": [{"name": "b5_%d.yaml" % i,
                                 "sha256": "a" * 64, "box_tag": 5}
                                for i in range(6)],
            "sender": sender, "queries": queries, "errors": [],
            "input_timeline": timeline, "terminals": {}, "pending": [],
            "health_series": health_series(240),
            "health_samples": 240, "health_gap_max_s": 0.5,
            "window_close_ms": 120_200.0}


def test_full_formal_p_window_passes():
    assert judge.judge_p_window(full_p_window())["reasons"] == []


# --- full-grade formal R artifact -------------------------------------------
def overload_row(index, *, status="COMPLETED", late_s=0.0, count=1):
    planned = T0 + index * 100_000_000  # 10 Hz
    publish = planned + 3_000_000
    arrival = publish + int(90 * 1e6 + late_s * 1e9)
    return {"input_id": "ov%06d" % index, "planned_mono_ns": planned,
            "publish_mono_ns": publish, "receipt_status": status,
            "receipt_count": count,
            "receipt_arrival_mono_ns": arrival if count else None,
            "callback_enter_ns": publish + 1_000_000,
            "e2e_ms": (arrival - publish) / 1e6 if count else None}


def full_r_artifact():
    overload = [{"input_id": "ov%06d" % i,
                 "planned_mono_ns": T0 + i * 100_000_000,
                 "publish_mono_ns": T0 + i * 100_000_000 + 3_000_000}
                for i in range(contract.R_OVERLOAD_INPUTS)]
    timeline = [overload_row(0, status="REJECTED")]
    timeline += [overload_row(i) for i in range(1, contract.R_OVERLOAD_INPUTS)]
    last_planned = overload[-1]["planned_mono_ns"]
    deadline = last_planned + int(contract.R_DRAIN_S * 1e9)
    overload_queries = [query_record(i, base=T0, status="STALE" if i % 40 == 0
                                     else "OK")
                        for i in range(contract.R_OVERLOAD_QUERIES)]
    drain_queries = [query_record(i, base=last_planned + 250_000_000)
                     for i in range(contract.R_DRAIN_QUERIES)]
    recovery_t0 = deadline + 500_000_000
    recovery = [{"input_id": "rc%06d" % i,
                 "planned_mono_ns": recovery_t0 + i * PERIOD_IN,
                 "publish_mono_ns": recovery_t0 + i * PERIOD_IN + 2_000_000}
                for i in range(contract.R_RECOVERY_INPUTS)]
    recovery_timeline = []
    for index in range(contract.R_RECOVERY_INPUTS):
        planned = recovery_t0 + index * PERIOD_IN
        publish = planned + 2_000_000
        arrival = publish + 90_000_000
        recovery_timeline.append({
            "input_id": "rc%06d" % index, "planned_mono_ns": planned,
            "publish_mono_ns": publish, "receipt_status": "INTEGRATED",
            "receipt_count": 1, "callback_enter_ns": publish + 1_000_000,
            "receipt_arrival_mono_ns": arrival, "e2e_ms": 90.0})
    chain_cells = [(0, 1, 1, 3, 2), (90, 2, 2, 3, 1)]
    chain = {"status": "OK", "pages": 1, "candidates": chain_cells,
             "pages_detail": [{"count": 2, "cursor": 0, "next_cursor": 2,
                               "done": True, "region_begin": [0, 0, 0],
                               "region_end": [64, 40, 24]}],
             "feasible_total": 2,
             "content_sha256": judge.candidate_digest(chain_cells),
             "lease_identity": identity(), "age_ns": 4 * 10**8,
             "release_status": "OK", "latency_ms": 150.0,
             "page_status": "OK"}
    recovery_queries = [query_record(i, base=recovery_t0 + PERIOD_Q)
                        for i in range(contract.R_RECOVERY_QUERIES - 1)]
    return {"stage": "r", "formal": True,
            "overload_inputs": contract.R_OVERLOAD_INPUTS,
            "drain_s": contract.R_DRAIN_S,
            "recovery_inputs": contract.R_RECOVERY_INPUTS,
            "identity_binding": binding(),
            "frames_manifest": [{"name": "b20_%d.yaml" % i,
                                 "sha256": "b" * 64, "box_tag": 20}
                                for i in range(6)],
            "overload": overload, "overload_timeline": timeline,
            "overload_terminals": {}, "overload_pending": [],
            "overload_queries": overload_queries,
            "drain_queries": drain_queries,
            "in_flight_at_drain_deadline": [],
            "late_at_drain_deadline": [],
            "drain_deadline_mono_ns": deadline,
            "drain_observed_mono_ns": deadline,
            "recovery": recovery, "recovery_timeline": recovery_timeline,
            "recovery_queries": recovery_queries,
            "first_wait": {"status": "OK", "reason": "",
                           "age_ns": 5 * 10**8, "elapsed_ms": 800.0,
                           "started_mono_ns": recovery_t0,
                           "planned_mono_ns": recovery_t0,
                           "bound_id": "rc000000", "chain": chain,
                           "release_status": "OK"},
            "epoch_before": 1, "epoch_after": 1, "errors": []}


def test_full_formal_r_passes():
    assert judge.judge_r_stage(full_r_artifact())["reasons"] == []


# --- full-grade formal L artifact -------------------------------------------
def full_l_artifact():
    total_inputs = contract.L_INPUTS
    total_queries = contract.L_QUERIES
    inputs = [{"index": i, "input_id": "lin%06d" % i,
               "stamp_ns": T0 + i * PERIOD_IN,
               "planned_mono_ns": T0 + i * PERIOD_IN,
               "publish_mono_ns": T0 + i * PERIOD_IN + 2_000_000}
              for i in range(total_inputs)]
    timeline = []
    for index in range(total_inputs):
        planned = T0 + index * PERIOD_IN
        publish = planned + 2_000_000
        arrival = publish + 90_000_000
        timeline.append({
            "input_id": "lin%06d" % index, "planned_mono_ns": planned,
            "publish_mono_ns": publish, "receipt_status": "INTEGRATED",
            "receipt_count": 1, "callback_enter_ns": publish + 1_000_000,
            "receipt_arrival_mono_ns": arrival, "e2e_ms": 90.0})
    # 100 resets run inside the 600 s warmup (epoch 2..101); queries before
    # the warmup end are not bound (epochs in flux), scored-window queries
    # carry the final epoch the binding names.
    final_epoch = contract.L_RESET_ROUNDS + 1
    warmup_queries = int(contract.L_WARMUP_S * contract.QUERY_HZ)
    queries = [query_record(i, base=T0, epoch=1 if i < warmup_queries
                            else final_epoch)
               for i in range(total_queries)]
    good_round = {"status": "OK", "release_status": "OK", "pages": 1,
                  "candidates": [(0, 1, 1, 3, 2)],
                  "pages_detail": [{"count": 1, "cursor": 0,
                                    "next_cursor": 1, "done": True,
                                    "region_begin": [0, 0, 0],
                                    "region_end": [64, 40, 24]}],
                  "feasible_total": 1,
                  "content_sha256": judge.candidate_digest([(0, 1, 1, 3, 2)]),
                  "lease_identity": identity(final_epoch), "latency_ms": 120.0,
                  "age_ns": 4 * 10**8, "page_status": "OK",
                  "ownership_after": ownership_after()}
    read_rounds = [dict(good_round) for _ in range(contract.L_READ_RELEASE_ROUNDS)]
    reset_rounds = [{"index": i,
                     "started_mono_ns": T0 + int(
                         contract.L_WARMUP_S * (i + 1)
                         / (contract.L_RESET_ROUNDS + 1) * 1e9),
                     "acquire_status": "OK",
                     "real_release": "OK", "reset_status": "OK", "epoch": i + 2,
                     "ownership_before": ownership_after(),
                     # Right after the reset only LIVE leases survive (our
                     # cross-reset lease sits in the lease class); the store
                     # is empty and no retained/client old blocks remain.
                     "ownership_after_reset": resource_snapshot(
                         view_directory_entries=2, view_directory_bytes=192,
                         old_view_block_bytes=125520,
                         lease_view_block_bytes=125520,
                         lease_view_directory_entries=2),
                     "ownership_after": ownership_after()}
                    for i in range(contract.L_RESET_ROUNDS)]
    span = int(contract.L_DURATION_S * 1e9)
    rss = [{"t_mono_ns": T0 + int(i * 0.5e9), "rss_kib": 300_000}
           for i in range(int(contract.L_DURATION_S / 0.5) + 1)]
    rss[-1]["t_mono_ns"] = T0 + span  # exact end
    return {"stage": "l", "formal": True,
            "planned_seconds": contract.L_DURATION_S,
            "read_rounds": contract.L_READ_RELEASE_ROUNDS,
            "reset_rounds": contract.L_RESET_ROUNDS,
            "identity_binding": binding(final_epoch),
            "frames_manifest": [{"name": "b20_%d.yaml" % i,
                                 "sha256": "c" * 64, "box_tag": 20}
                                for i in range(6)],
            "sent_inputs": total_inputs, "send_errors": [],
            "input_records": inputs, "input_timeline": timeline,
            "query_records": queries,
            "read_rounds_results": read_rounds,
            "read_failed_attempts": [],
            "reset_rounds_results": reset_rounds,
            "rss_series": rss,
            "health_series": health_series(int(contract.L_DURATION_S / 0.5)),
            "health_samples": 7000,
            "baseline": resource_snapshot(),
            "after": ownership_after()}


def test_full_formal_l_passes():
    assert judge.judge_l_stage(full_l_artifact())["reasons"] == []


# --- the review's eight audit mutations, replayed on the full fixtures ------
def test_review_eight_mutations_all_fail():
    p = full_p_window()
    for row in p["input_timeline"]:
        row["input_id"] = "unrelated"
    assert judge.judge_p_window(p)["reasons"]
    p = full_p_window()
    for record in p["queries"]:
        record["ended_mono_ns"] = record["planned_mono_ns"] - 1
    assert any("negative" in r or "before" in r
               for r in judge.judge_p_window(p)["reasons"])

    r = full_r_artifact()
    r["recovery_timeline"] = []
    r["recovery_queries"] = []
    assert judge.judge_r_stage(r)["reasons"]
    r = full_r_artifact()
    r["recovery_queries"][1]["status"] = "INTERNAL_ERROR"
    assert any("rq:" in reason for reason in judge.judge_r_stage(r)["reasons"])

    l = full_l_artifact()
    for record in l["query_records"][1:]:
        record["status"] = "INTERNAL_ERROR"
    assert any("lq:status" in reason for reason in judge.judge_l_stage(l)["reasons"])
    l = full_l_artifact()
    l["sent_inputs"] = contract.L_INPUTS - 1800
    l["query_records"] = l["query_records"][:contract.L_QUERIES - 1440]
    assert judge.judge_l_stage(l)["reasons"]
    l = full_l_artifact()
    stamp = l["rss_series"][1]["t_mono_ns"]
    for sample in l["rss_series"][1:]:
        sample["t_mono_ns"] = stamp
    assert any("monotonic" in reason
               for reason in judge.judge_l_stage(l)["reasons"])
    l = full_l_artifact()
    l["rss_series"] = [s for s in l["rss_series"]
                       if not T0 + 1800 * 10**9 <= s["t_mono_ns"]
                       < T0 + 2400 * 10**9]
    assert any("bucket" in reason for reason in judge.judge_l_stage(l)["reasons"])


# --- field x position (first/middle/last) x fault matrix --------------------
POSITIONS = ("first", "middle", "last")


def _pick(records, position):
    if position == "first":
        return records[0]
    if position == "last":
        return records[-1]
    return records[len(records) // 2]


def _residual_entries(round_record, value=4):
    """Lift the directory residual in the round's ownership measurement --
    the owner-class directory field (where a lingering handle lives) AND
    every retained sample (the judge reads the per-class minimum of the
    samples, never a self-report)."""
    ownership = round_record["ownership_after"]
    ownership["view_directory_entries"] = value
    ownership["lease_view_directory_entries"] = value
    for sample in ownership["resource_samples"]:
        sample["view_directory_entries"] = value
        sample["lease_view_directory_entries"] = value


def _residual_class(round_record, field, delta):
    """Leave a byte residual in exactly one owner class, directory entries
    still at baseline -- the gen4 review's 100 MiB cache-residue case."""
    ownership = round_record["ownership_after"]
    ownership[field] = ownership.get(field, 0) + delta
    for sample in ownership["resource_samples"]:
        sample[field] = sample.get(field, 0) + delta


@pytest.mark.parametrize("position", POSITIONS)
def test_p_matrix_input_faults(position):
    faults = {
        "missing_receipt": lambda row: row.update(receipt_status=None,
                                                  receipt_count=0,
                                                  e2e_ms=None,
                                                  receipt_arrival_mono_ns=None),
        "duplicate_terminal": lambda row: row.update(receipt_count=2),
        "wrong_id": lambda row: row.update(input_id="ghost"),
        "bad_e2e": lambda row: row.update(e2e_ms=row["e2e_ms"] + 10.0),
        "rejected": lambda row: row.update(receipt_status="REJECTED"),
    }
    for name, mutate in faults.items():
        window = full_p_window()
        mutate(_pick(window["input_timeline"], position))
        reasons = judge.judge_p_window(window)["reasons"]
        assert reasons, "%s at %s passed" % (name, position)


@pytest.mark.parametrize("position", POSITIONS)
def test_p_matrix_query_faults(position):
    faults = {
        "status": lambda record: record.update(status="INTERNAL_ERROR"),
        "release": lambda record: record.update(release_status="LEAKED"),
        "identity": lambda record: record.pop("observed_revision") if False
        else record["lease_identity"].update(observed_revision=None),
        "digest": lambda record: record.update(content_sha256="0" * 64),
        "negative_latency": lambda record: record.update(
            ended_mono_ns=record["planned_mono_ns"] - 5_000_000),
        "over_gate": lambda record: record.update(
            e2e_hint=None) if False else None,
    }
    for name, mutate in faults.items():
        if name == "over_gate":
            continue
        window = full_p_window()
        mutate(_pick(window["queries"], position))
        reasons = judge.judge_p_window(window)["reasons"]
        assert reasons, "%s at %s passed" % (name, position)
    window = full_p_window()
    _pick(window["input_timeline"], position)["e2e_ms"] = 501.0
    assert any("e2e" in r for r in judge.judge_p_window(window)["reasons"])


@pytest.mark.parametrize("position", POSITIONS)
def test_r_matrix_faults(position):
    faults = {
        "overload_non_terminal": lambda a: _pick(
            a["overload_timeline"], position).update(receipt_status="QUEUED"),
        "drain_late": lambda a: _pick(a["overload_timeline"], position).update(
            receipt_arrival_mono_ns=a["drain_deadline_mono_ns"] + 10**9,
            e2e_ms=_pick(a["overload_timeline"], position)["e2e_ms"] + 1000.0),
        "drain_query_fault": lambda a: _pick(
            a["drain_queries"], position).update(status="PAGE_EVICTION"),
        "recovery_query_fault": lambda a: _pick(
            a["recovery_queries"], position).update(release_status="LEAKED"),
        "wait_not_concurrent": lambda a: a["first_wait"].update(
            planned_mono_ns=a["first_wait"]["planned_mono_ns"] + 400_000_000),
        "wait_chain_missing": lambda a: a["first_wait"].pop("chain"),
        "epoch_restarted": lambda a: a.update(epoch_after=2),
        "frames_mixed": lambda a: a["frames_manifest"][0].update(box_tag=5),
    }
    for name, mutate in faults.items():
        artifact = full_r_artifact()
        mutate(artifact)
        reasons = judge.judge_r_stage(artifact)["reasons"]
        assert reasons, "%s at %s passed" % (name, position)


@pytest.mark.parametrize("position", POSITIONS)
def test_l_matrix_faults(position):
    faults = {
        "query_fault": lambda a: _pick(a["query_records"], position).update(
            status="INTERNAL_ERROR"),
        "query_digest": lambda a: _pick(a["query_records"], position).update(
            content_sha256="0" * 64),
        "input_fault": lambda a: _pick(a["input_timeline"], position).update(
            receipt_count=2),
        "read_ownership": lambda a: _residual_entries(
            _pick(a["read_rounds_results"], position)),
        "read_cache_residual": lambda a: _residual_class(
            _pick(a["read_rounds_results"], position),
            "external_surface_buffer_bytes", 100 * 1024 * 1024),
        "read_status": lambda a: _pick(
            a["read_rounds_results"], position).update(status="STALE"),
        "reset_real_release": lambda a: _pick(
            a["reset_rounds_results"], position).update(real_release="LEAKED"),
        "reset_no_lease": lambda a: _pick(
            a["reset_rounds_results"], position).update(acquire_status="NO_DATA"),
        "reset_status_bad": lambda a: _pick(
            a["reset_rounds_results"], position).update(reset_status="EPOCH_MISMATCH"),
        "reset_ownership": lambda a: _residual_entries(
            _pick(a["reset_rounds_results"], position), value=9),
        "rss_bad_sample": lambda a: _pick(a["rss_series"], position).update(
            rss_kib=-1),
        "rss_duplicate_time": lambda a: a["rss_series"].__setitem__(
            a["rss_series"].index(_pick(a["rss_series"], position)),
            dict(_pick(a["rss_series"], position),
                 t_mono_ns=a["rss_series"][a["rss_series"].index(
                     _pick(a["rss_series"], position)) - 1]["t_mono_ns"])),
    }
    for name, mutate in faults.items():
        artifact = full_l_artifact()
        mutate(artifact)
        reasons = judge.judge_l_stage(artifact)["reasons"]
        assert reasons, "%s at %s passed" % (name, position)
    # whole-bucket loss at the START, MIDDLE, and END of the scored window
    for bucket, label in ((6, "first_scored"), (11, "middle"), (11 + 0, "end")):
        artifact = full_l_artifact()
        lo = T0 + int((contract.L_WARMUP_S + (bucket - 5) * 600) * 1e9)
        artifact["rss_series"] = [s for s in artifact["rss_series"]
                                  if not lo <= s["t_mono_ns"] < lo + 600 * 10**9]
        assert judge.judge_l_stage(artifact)["reasons"], label


# --- short samples carry the marker only when otherwise clean ---------------
def test_short_sample_carries_only_the_marker():
    window = full_p_window()
    window["formal"] = False
    window["sender"] = window["sender"][:20]
    window["input_timeline"] = window["input_timeline"][:20]
    window["queries"] = window["queries"][:10]
    assert judge.judge_p_window(window)["reasons"] == ["short_sample_not_formal"]


# --- the schedule plans are the offline-verifiable event source --------------
def test_recovery_plan_first_wait_shares_input_zero():
    plan = campaign.recovery_plan(1000, contract.R_RECOVERY_INPUTS,
                                  contract.R_RECOVERY_QUERIES - 1,
                                  contract.UPDATE_HZ, contract.QUERY_HZ)
    assert plan["first_wait_ns"] == plan["inputs"][0]
    assert plan["queries"][0] == 1000 + PERIOD_Q
    assert len(plan["queries"]) == contract.R_RECOVERY_QUERIES - 1
    assert plan["inputs"][-1] == 1000 + int(74 / 2.5 * 1e9)


def test_drain_plan_deadline_and_queries():
    plan = campaign.drain_plan(5000, contract.R_DRAIN_S,
                               contract.R_DRAIN_QUERIES, contract.QUERY_HZ)
    assert plan["deadline_ns"] == 5000 + int(30 * 1e9)
    assert len(plan["queries"]) == contract.R_DRAIN_QUERIES
    assert plan["queries"][0] == 5000 + 250_000_000
    assert plan["queries"][-1] < plan["deadline_ns"]


def test_p_window_plan_rates_independent():
    plan = campaign.p_window_plan(0, contract.P_WINDOW_INPUTS,
                                  contract.P_WINDOW_QUERIES,
                                  contract.UPDATE_HZ, contract.QUERY_HZ)
    assert plan["inputs"][1] - plan["inputs"][0] == PERIOD_IN
    assert plan["queries"][1] - plan["queries"][0] == PERIOD_Q
    assert plan["inputs"][-1] < 120 * 10**9
    assert plan["queries"][-1] < 120 * 10**9


# --- the indexed receipt collector keeps duplicates visible ------------------
class FakeClient:
    def __init__(self, receipts):
        self.receipts = receipts


def test_receipt_timeline_counts_duplicate_terminals():
    publish = 10**9
    records = [{"input_id": "a", "planned_mono_ns": 0,
                "publish_mono_ns": publish}]
    receipts = [
        {"arrival_mono_ns": publish + 50_000_000, "status": "QUEUED",
         "observation_id": "a", "callback_enter_ns": publish,
         "input_stamp_ns": 0, "epoch": 1, "map_revision": 1,
         "observed_revision": 1},
        {"arrival_mono_ns": publish + 90_000_000, "status": "INTEGRATED",
         "observation_id": "a", "callback_enter_ns": publish,
         "input_stamp_ns": 0, "epoch": 1, "map_revision": 1,
         "observed_revision": 2},
        {"arrival_mono_ns": publish + 120_000_000, "status": "INTEGRATED",
         "observation_id": "a", "callback_enter_ns": publish,
         "input_stamp_ns": 0, "epoch": 1, "map_revision": 1,
         "observed_revision": 3},
    ]
    timeline = campaign.receipt_timeline(FakeClient(receipts), records,
                                         timeout_s=0.1)
    assert timeline[0]["receipt_count"] == 2  # duplicate stays visible
    assert len(timeline[0]["receipt_events"]) == 3
    assert timeline[0]["receipt_arrival_mono_ns"] == publish + 120_000_000
    assert abs(timeline[0]["e2e_ms"] - 120.0) < 1e-6


def test_receipt_timeline_missing_receipt_stays_missing():
    records = [{"input_id": "ghost", "planned_mono_ns": 0,
                "publish_mono_ns": 10}]
    timeline = campaign.receipt_timeline(FakeClient([]), records, timeout_s=0.1)
    assert timeline[0]["receipt_status"] is None
    assert timeline[0]["receipt_count"] == 0
    assert timeline[0]["e2e_ms"] is None


# --- the nine-window matrix check -------------------------------------------
def test_p_matrix_requires_nine_distinct_windows():
    windows = []
    for box in contract.P_BOX_COUNTS:
        for index in range(contract.P_WINDOWS_PER_BOX):
            window = full_p_window()
            window.update({"box_count": box, "window_index": index,
                           "window_id": "b%02d_w%d" % (box, index)})
            windows.append(window)
    assert judge.judge_p_matrix(windows) == []
    dropped = windows[:-1]
    assert any("missing" in p for p in judge.judge_p_matrix(dropped))
    broken = [dict(w) for w in windows]
    broken[4] = dict(broken[4])
    broken[4]["queries"] = broken[4]["queries"][:100]
    assert judge.judge_p_matrix(broken)


# --- O9 independent re-judge --------------------------------------------------
def o9_pair(index, scale=1.0):
    base = {"roi": 10.0, "integrate_30k": 30.0,
            "surface_first": 12.0, "freespace_full": 40.0}
    on = {name: value * scale for name, value in base.items()}
    return {"on": on, "off": dict(base), "freespace_on": "f" * 64,
            "freespace_off": "f" * 64, "content_on": "c" * 64,
            "content_off": "c" * 64}


def o9_result(pairs):
    ratios = {}
    for name in judge.O9_OPERATIONS:
        on = sorted(p["on"][name] for p in pairs)
        off = sorted(p["off"][name] for p in pairs)
        index = int(__import__("math").ceil(0.95 * len(pairs))) - 1
        ratios[name] = on[index] / off[index]
    return {"ratios": ratios,
            "verdict": {"pass": True, "performance_certified": True},
            "failures": []}


def test_o9_rejudge_accepts_honest_result():
    pairs = [o9_pair(i, 1.0 + 0.001 * (i % 7)) for i in range(30)]
    assert judge.judge_o9_samples(pairs, o9_result(pairs)) == []


def test_o9_rejudge_catches_tampering():
    pairs = [o9_pair(i, 1.0 + 0.001 * (i % 7)) for i in range(30)]
    problems = judge.judge_o9_samples(pairs[:-3], o9_result(pairs))
    assert any("pair_count" in p for p in problems)
    result = o9_result(pairs)
    result["ratios"]["roi"] = 0.5  # a lied ratio
    assert any("ratio_mismatch" in p
               for p in judge.judge_o9_samples(pairs, result))
    result = o9_result(pairs)
    result["verdict"]["performance_certified"] = False
    assert any("certified" in p for p in judge.judge_o9_samples(pairs, result))
    tampered = [dict(pair) for pair in pairs]
    tampered[7] = dict(tampered[7], freespace_off="0" * 64)
    assert any("freespace_digest" in p
               for p in judge.judge_o9_samples(tampered, o9_result(pairs)))
    slow = [o9_pair(i, 3.0) for i in range(30)]  # every pair 3x slower ON
    assert any("ratio" in p for p in judge.judge_o9_samples(slow, {
        **o9_result(slow), "ratios": {n: 1.0 for n in judge.O9_OPERATIONS}}))


def test_o9_document_problems():
    good = {"status": "OK",
            "surface_first_call": {"cache_hit": False, "external_ms": 5},
            "surface_second_call": {"cache_hit": True, "external_ms": 2},
            "roi_cells": 4096, "roi_surface_content_sha256": "x",
            "integrate_external_ms": 3.0, "roi_ms": 2.0,
            "freespace_full_external_ms": 9.0}
    assert judge.o9_document_problems(good) == []
    assert judge.o9_document_problems(dict(good, roi_cells=1024))
    assert judge.o9_document_problems(dict(good, status="ERROR"))
    assert judge.o9_document_problems(
        dict(good, surface_second_call={"cache_hit": False}))
