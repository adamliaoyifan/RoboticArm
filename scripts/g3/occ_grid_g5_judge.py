"""Gen5 acceptance judge. Pure decisions, no product imports.

P95 is nearest-rank sorted[ceil(0.95*n)-1]. Fewer than 30 samples is
not_evaluated. These functions are the pre-score discriminators; they do
not start a campaign.
"""
import math


PASS = "pass"
FAIL = "fail"
NOT_EVALUATED = "not_evaluated"


def nearest_rank_p95(values, minimum=30):
    if len(values) < minimum:
        return None
    ordered = sorted(values)
    return ordered[math.ceil(0.95 * len(ordered)) - 1]


def judge_identity(expected, observed):
    if not expected or not observed or observed != expected:
        return {"status": FAIL, "reason": "identity_mismatch"}
    return {"status": PASS, "reason": "identity_match"}


def judge_plan_ids(expected, observed):
    expected = list(expected)
    observed = list(observed)
    duplicates = sorted({item for item in observed if observed.count(item) > 1})
    missing = sorted(set(expected) - set(observed))
    extra = sorted(set(observed) - set(expected))
    if duplicates or missing or extra:
        return {
            "status": FAIL,
            "reason": "plan_id_set",
            "duplicates": duplicates,
            "missing": missing,
            "extra": extra,
        }
    return {"status": PASS, "reason": "plan_id_set"}


def judge_all_rejected(latencies_ms):
    """An all-rejected window has no success latency to score."""
    if latencies_ms:
        return {"status": FAIL, "reason": "successes_present"}
    return {"status": NOT_EVALUATED, "reason": "all_rejected"}


def judge_late_success(deadline_ms, completed_ms):
    if completed_ms > deadline_ms:
        return {"status": FAIL, "reason": "late_success"}
    return {"status": PASS, "reason": "on_time"}


def judge_current_retry(events, minimum_gap_ms=50):
    """events are ('start', t_ms) or ('retry', t_ms) in order."""
    starts = [t for kind, t in events if kind == "start"]
    if len(starts) != 1:
        return {"status": FAIL, "reason": "current_start_count", "starts": len(starts)}
    previous = starts[0]
    for kind, stamp in events:
        if kind != "retry":
            continue
        if stamp - previous < minimum_gap_ms:
            return {"status": FAIL, "reason": "early_retry", "gap_ms": stamp - previous}
        previous = stamp
    return {"status": PASS, "reason": "retry_gap"}


def judge_batch_coverage(member_ids, proof_ids):
    missing = sorted(set(member_ids) - set(proof_ids or ()))
    if missing:
        return {"status": FAIL, "reason": "coverage_proof_missing", "missing": missing}
    return {"status": PASS, "reason": "coverage_proof"}


def judge_completion(planned_ids, completed_ids):
    missing = sorted(set(planned_ids) - set(completed_ids))
    if missing:
        return {
            "status": FAIL,
            "reason": "missing_completion",
            "missing": missing,
            "denominator": len(planned_ids),
            "completed": len(set(completed_ids) & set(planned_ids)),
        }
    return {"status": PASS, "reason": "complete", "denominator": len(planned_ids)}


def judge_bounded_queue(offered_ids, admitted_ids, rejected, capacity):
    """Excess must be named not_admitted_capacity. Silent loss fails."""
    offered = list(offered_ids)
    admitted = list(admitted_ids)
    named = {item["id"] for item in rejected if item.get("reason") == "not_admitted_capacity"}
    silent = [item for item in offered if item not in admitted and item not in named]
    if silent:
        return {"status": FAIL, "reason": "silent_drop", "ids": silent}
    if len(admitted) > capacity:
        return {"status": FAIL, "reason": "capacity_exceeded", "admitted": len(admitted)}
    return {"status": PASS, "reason": "queue_accounted", "not_admitted": sorted(named)}


def judge_recorder(records, slow_ratio=None, ratio_limit=1.10):
    if records:
        return {"status": FAIL, "reason": "recorder_not_off", "records": len(records)}
    if slow_ratio is None:
        return {"status": PASS, "reason": "recorder_off"}
    if slow_ratio > ratio_limit:
        return {"status": FAIL, "reason": "slow_recorder", "ratio": slow_ratio}
    return {"status": PASS, "reason": "recorder_within_limit", "ratio": slow_ratio}


def judge_resources(entries, rss_limit, derived_limit):
    """entries: name, rss_bytes, derived_bytes, shared_id, attribution."""
    if any(item.get("attribution") in (None, "", "unknown") for item in entries):
        return {"status": NOT_EVALUATED, "reason": "resource_attribution_unknown"}
    rss = 0
    derived = {}
    for item in entries:
        rss += int(item["rss_bytes"])
        key = item.get("shared_id") or item["name"]
        derived[key] = max(derived.get(key, 0), int(item["derived_bytes"]))
    derived_sum = sum(derived.values())
    if rss > rss_limit or derived_sum > derived_limit:
        return {
            "status": FAIL,
            "reason": "resource_limit",
            "rss_bytes": rss,
            "derived_bytes": derived_sum,
        }
    return {"status": PASS, "reason": "resource_within_limit", "rss_bytes": rss, "derived_bytes": derived_sum}


def judge_replay(expected_fault, observed_fault):
    if observed_fault != expected_fault:
        return {"status": FAIL, "reason": "replay_mismatch", "observed": observed_fault}
    return {"status": PASS, "reason": "replay_match"}


def judge_propagation(bounds, p95_limit_ms=1000, max_limit_ms=2000):
    """bounds are (lower_ms, upper_ms) per commit. None is not observed."""
    if any(item is None or item[0] is None or item[1] is None for item in bounds):
        return {"status": NOT_EVALUATED, "reason": "propagation_endpoint_missing"}
    uppers = [item[1] for item in bounds]
    lowers = [item[0] for item in bounds]
    if nearest_rank_p95(lowers) is not None and nearest_rank_p95(lowers) > p95_limit_ms:
        return {"status": FAIL, "reason": "propagation_lower_bound_exceeds"}
    if max(lowers) > max_limit_ms:
        return {"status": FAIL, "reason": "propagation_lower_max_exceeds"}
    upper_p95 = nearest_rank_p95(uppers)
    if upper_p95 is None:
        return {"status": NOT_EVALUATED, "reason": "propagation_sample_count"}
    if upper_p95 <= p95_limit_ms and max(uppers) <= max_limit_ms:
        return {"status": PASS, "reason": "propagation_upper_bounds"}
    if upper_p95 > p95_limit_ms or max(uppers) > max_limit_ms:
        # Upper over the limit does not by itself fail when the lower bound
        # still sits under the limit: the true commit may be inside the gap.
        if nearest_rank_p95(lowers) is not None and nearest_rank_p95(lowers) <= p95_limit_ms:
            return {"status": NOT_EVALUATED, "reason": "propagation_interval_unresolved"}
    return {"status": NOT_EVALUATED, "reason": "propagation_unresolved"}
