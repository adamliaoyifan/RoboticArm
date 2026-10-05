#!/usr/bin/env python3
"""MotionExecutor atlas seed chain: miss-retry, always-first, joint_goal."""

from __future__ import division

import math
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..")))

from luggage_planning.motion_executor import JOINTS, MotionExecutor  # noqa: E402
from luggage_planning.pose import Point, Pose, Quaternion  # noqa: E402

CURRENT = [2.956, -1.396, -1.197, 4.158, 1.570, 2.964]
ATLAS_SEEDS = [
    [0.936, -0.602, 0.496, 1.671, 1.567, 2.507],
    [0.808, -0.531, 0.924, 1.182, 1.566, 0.808],
]
TARGET = (1.03, -0.735, 0.777)


def _pose():
    return Pose(
        position=Point(x=1.03, y=-0.735, z=0.777),
        orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))


def _provider(seeds=None, error=None):
    calls = []

    def provider(pose):
        calls.append(pose)
        if error is not None:
            raise error
        return {
            "status": 3, "status_name": "reachable",
            "cell": [2, 2, 1, 0], "seed_cell": [2, 2, 1, 0],
            "n_seeds": len(seeds or []), "seeds": list(seeds or []),
        }

    provider.calls = calls
    return provider


def _executor(provider, mode="miss-retry", submit_mode="reseed",
              fk=None, fk_tol=0.03, ik_hits_seeds=True):
    executor = object.__new__(MotionExecutor)
    executor._atlas_provider = provider
    executor._atlas_mode = mode
    executor._atlas_max_seeds = 3
    executor._atlas_fk_tol = fk_tol
    executor._atlas_submit_mode = submit_mode
    executor._pose_target_max_delta = 1.50
    executor._j6_limit = 2.50
    executor._wrist_margin = 0.50
    executor._atlas_record = None
    executor._ik_calls = []

    def _query_ik(pose, seed):
        executor._ik_calls.append(list(seed or []))
        # the unseeded pass (seed == current joints) always misses; atlas
        # seeds hit only when ik_hits_seeds (False simulates the all-miss
        # case where the joint_goal fallback is the last resort)
        if ik_hits_seeds and seed and list(seed) != list(CURRENT):
            return [round(v, 6) for v in seed]
        return None

    executor._query_ik = _query_ik
    executor._fk_xyz = (
        (lambda names, positions: fk) if fk is not None
        else (lambda names, positions: None))
    executor._j6_check = lambda points, names=None: None
    executor._wrist_check = lambda points, names=None: None
    return executor


def test_miss_retry_seeds_after_unseeded_miss():
    provider = _provider(ATLAS_SEEDS)
    executor = _executor(provider)
    chosen = executor._ik_joints(_pose(), CURRENT, keep_tool_down=False)
    # unseeded (miss) then one call per seed
    assert executor._ik_calls[0] == CURRENT
    assert executor._ik_calls[1] == ATLAS_SEEDS[0]
    assert chosen == ATLAS_SEEDS[0]
    record = executor._atlas_record
    assert record["tried"] and record["outcome"] == "seeded"
    assert record["cell"] == [2, 2, 1, 0] and record["n_seeds"] == 2
    assert executor._pose_target_joint_delta["n_solutions"] == 2


def test_unseeded_hit_never_calls_provider():
    provider = _provider(ATLAS_SEEDS)
    executor = _executor(provider)
    # make the unseeded pass succeed
    executor._query_ik = lambda pose, seed: list(CURRENT)
    chosen = executor._ik_joints(_pose(), CURRENT)
    assert chosen == CURRENT
    assert provider.calls == []
    assert executor._atlas_record["outcome"] == "unseeded"
    assert not executor._atlas_record["tried"]


def test_always_first_skips_unseeded_when_seeds_hit():
    provider = _provider(ATLAS_SEEDS)
    executor = _executor(provider, mode="always-first")
    chosen = executor._ik_joints(_pose(), CURRENT)
    assert chosen == ATLAS_SEEDS[0]
    assert executor._ik_calls == [ATLAS_SEEDS[0], ATLAS_SEEDS[1]]
    assert executor._atlas_record["outcome"] == "always_first"


def test_joint_goal_fk_verified_submit():
    provider = _provider(ATLAS_SEEDS)
    executor = _executor(
        provider, submit_mode="joint_goal", ik_hits_seeds=False,
        fk=TARGET)  # every IK pass misses; FK of the seed hits the target
    chosen = executor._ik_joints(_pose(), CURRENT)
    assert chosen == ATLAS_SEEDS[0]
    record = executor._atlas_record
    assert record["outcome"] == "joint_goal"
    assert record["fk_error_m"] == 0.0
    assert executor._pose_target_joint_delta["n_solutions"] == 1  # FK-verified seed


def test_joint_goal_fk_out_of_tolerance_refuses():
    far = tuple(v + 0.05 for v in TARGET)
    provider = _provider(ATLAS_SEEDS)
    executor = _executor(
        provider, submit_mode="joint_goal", ik_hits_seeds=False,
        fk=far, fk_tol=0.03)
    chosen = executor._ik_joints(_pose(), CURRENT)
    assert chosen is None
    assert executor._atlas_record["outcome"] == "no_solution"


def test_reseed_mode_never_submits_without_ik():
    provider = _provider(ATLAS_SEEDS)
    executor = _executor(provider, submit_mode="reseed", ik_hits_seeds=False,
                         fk=TARGET)
    chosen = executor._ik_joints(_pose(), CURRENT)
    assert chosen is None
    assert executor._atlas_record["outcome"] == "no_solution"


def test_no_seeds_records_and_returns_none():
    provider = _provider([])
    executor = _executor(provider)
    chosen = executor._ik_joints(_pose(), CURRENT)
    assert chosen is None
    record = executor._atlas_record
    assert record["outcome"] == "no_seeds" and record["n_seeds"] == 0


def test_provider_exception_is_absorbed():
    provider = _provider(error=RuntimeError("atlas hiccup"))
    executor = _executor(provider)
    chosen = executor._ik_joints(_pose(), CURRENT)
    assert chosen is None
    assert executor._atlas_record["outcome"] == "no_seeds"


def test_keep_tool_down_tries_both_yaws_per_seed():
    provider = _provider(ATLAS_SEEDS[:1])
    executor = _executor(provider)
    chosen = executor._ik_joints(_pose(), CURRENT, keep_tool_down=True)
    # unseeded pass: current seed on pose + yaw+pi twin (2 misses);
    # seeded pass: the atlas seed on both twins (2 hits) -> chosen
    assert executor._ik_calls == [
        CURRENT, CURRENT, ATLAS_SEEDS[0], ATLAS_SEEDS[0]]
    assert chosen == ATLAS_SEEDS[0]
