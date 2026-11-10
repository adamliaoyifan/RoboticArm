"""Bounded same-orientation IK continuation; never certifies a trajectory."""

import math

STEP_M = 0.01
MAX_STEPS = 128


def refine_retreat(start, target, quaternion, seed, refine, expired):
    """Follow local IK seeds so total travel is not mistaken for a branch jump.

    The returned endpoint still requires the caller's Cartesian edge and full
    collision/timing certification. Failure never returns a partial endpoint.
    """
    from luggage_planning.grid_path_search import (
        DESCEND_BRANCH_TOL_RAD, descend_pair_rank)

    distance = math.sqrt(sum((b - a) ** 2 for a, b in zip(start, target)))
    count = max(1, math.ceil(distance / STEP_M))
    evidence = {"distance_m": distance, "required_steps": count, "steps": []}
    if count > MAX_STEPS:
        evidence["reason"] = "step_limit"
        return None, evidence
    q = tuple(seed)
    for index in range(1, count + 1):
        if expired():
            evidence["reason"] = "deadline"
            return None, evidence
        position = tuple(a + (b - a) * index / count
                         for a, b in zip(start, target))
        solved = refine(position, quaternion, q)
        row = {"position": list(position), "seed_q": list(q),
               "result_q": None if solved is None else list(solved)}
        evidence["steps"].append(row)
        if solved is None:
            evidence["reason"] = "ik_failed"
            return None, evidence
        row["branch_rank"] = list(descend_pair_rank(q, solved))
        raw_delta = max(abs(a - b) for a, b in zip(q, solved))
        if row["branch_rank"][0] != 0 or raw_delta > DESCEND_BRANCH_TOL_RAD:
            evidence["reason"] = "branch_change"
            return None, evidence
        q = tuple(solved)
    if expired():
        evidence["reason"] = "deadline"
        return None, evidence
    evidence["reason"] = "refined"
    return q, evidence
