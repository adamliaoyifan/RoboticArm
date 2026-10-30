"""Positive and negative controls for the gen5 judge. No campaign is run."""
import occ_grid_g5_judge as judge


def test_n29_is_not_scored_and_n30_uses_the_29th():
    values = list(range(29))
    assert judge.nearest_rank_p95(values) is None
    values = list(range(30))
    assert judge.nearest_rank_p95(values) == 28


def test_identity_match_and_empty_or_wrong():
    assert judge.judge_identity("map-a", "map-a")["status"] == judge.PASS
    assert judge.judge_identity("map-a", "")["reason"] == "identity_mismatch"
    assert judge.judge_identity("map-a", "map-b")["status"] == judge.FAIL


def test_plan_ids_exact_set_and_duplicate_or_missing():
    assert judge.judge_plan_ids(["a", "b"], ["b", "a"])["status"] == judge.PASS
    bad = judge.judge_plan_ids(["a", "b"], ["a", "a"])
    assert bad["status"] == judge.FAIL
    assert bad["missing"] == ["b"]
    assert bad["duplicates"] == ["a"]


def test_all_rejected_is_not_a_latency_pass_and_a_success_is_not_that_case():
    assert judge.judge_all_rejected([])["status"] == judge.NOT_EVALUATED
    assert judge.judge_all_rejected([1.0])["status"] == judge.FAIL


def test_late_success_and_on_time():
    assert judge.judge_late_success(1000, 1001)["reason"] == "late_success"
    assert judge.judge_late_success(1000, 1000)["status"] == judge.PASS


def test_current_single_start_and_early_retry():
    assert judge.judge_current_retry([("start", 0), ("retry", 50)])["status"] == judge.PASS
    early = judge.judge_current_retry([("start", 0), ("retry", 49)])
    assert early["reason"] == "early_retry"
    repeat = judge.judge_current_retry([("start", 0), ("start", 10)])
    assert repeat["reason"] == "current_start_count"


def test_batch_coverage_proof():
    assert judge.judge_batch_coverage(["c1", "c2"], ["c2", "c1"])["status"] == judge.PASS
    missing = judge.judge_batch_coverage(["c1", "c2"], ["c1"])
    assert missing["reason"] == "coverage_proof_missing"


def test_missing_completion_stays_in_the_denominator():
    done = judge.judge_completion(["p1", "p2"], ["p1", "p2"])
    assert done["status"] == judge.PASS
    gap = judge.judge_completion(["p1", "p2"], ["p1"])
    assert gap["denominator"] == 2
    assert gap["completed"] == 1
    assert gap["missing"] == ["p2"]


def test_fast_admit_and_named_capacity_reject_and_silent_drop():
    ok = judge.judge_bounded_queue(["a", "b", "c"], ["a", "b"], [
        {"id": "c", "reason": "not_admitted_capacity"}], capacity=2)
    assert ok["status"] == judge.PASS
    silent = judge.judge_bounded_queue(["a", "b"], ["a"], [], capacity=2)
    assert silent["reason"] == "silent_drop"


def test_recorder_off_passes_and_slow_recorder_fails():
    assert judge.judge_recorder([])["reason"] == "recorder_off"
    assert judge.judge_recorder([{"id": "x"}])["status"] == judge.FAIL
    assert judge.judge_recorder([], slow_ratio=1.0)["status"] == judge.PASS
    assert judge.judge_recorder([], slow_ratio=1.11)["reason"] == "slow_recorder"


def test_resource_dedup_limit_and_unknown():
    shared = [
        {"name": "a", "rss_bytes": 10, "derived_bytes": 40, "shared_id": "obj", "attribution": "rss"},
        {"name": "b", "rss_bytes": 10, "derived_bytes": 40, "shared_id": "obj", "attribution": "rss"},
        {"name": "buf", "rss_bytes": 5, "derived_bytes": 7, "shared_id": None, "attribution": "buffer"},
    ]
    ok = judge.judge_resources(shared, rss_limit=100, derived_limit=50)
    assert ok["status"] == judge.PASS
    assert ok["derived_bytes"] == 47
    over = judge.judge_resources(shared, rss_limit=100, derived_limit=40)
    assert over["status"] == judge.FAIL
    unknown = dict(shared[0])
    unknown["attribution"] = "unknown"
    assert judge.judge_resources([unknown], 100, 100)["status"] == judge.NOT_EVALUATED


def test_replay_fault_matches_and_a_success_does_not():
    assert judge.judge_replay("SNAPSHOT_IDENTITY_MISMATCH", "SNAPSHOT_IDENTITY_MISMATCH")["status"] == judge.PASS
    assert judge.judge_replay("SNAPSHOT_IDENTITY_MISMATCH", "OK")["status"] == judge.FAIL


def test_propagation_upper_pass_lower_fail_and_gap_unresolved():
    bounds = [(100, 200)] * 30
    assert judge.judge_propagation(bounds)["status"] == judge.PASS
    late = [(1500, 1600)] * 30
    assert judge.judge_propagation(late)["reason"] == "propagation_lower_bound_exceeds"
    gap = [(100, 5000)] * 30
    assert judge.judge_propagation(gap)["reason"] == "propagation_interval_unresolved"
    short = [(100, 200)] * 29
    assert judge.judge_propagation(short)["status"] == judge.NOT_EVALUATED
