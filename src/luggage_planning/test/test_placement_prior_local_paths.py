from dataclasses import replace
import math
import pytest
from test_placement_prior import request, I
from test_placement_prior_pick import detection
from luggage_planning.placement_prior import CollisionVerdict, compute_prior
from luggage_planning.placement_prior_contracts import Transform
from luggage_planning.placement_prior_pick import candidates_for_prior
from luggage_planning.placement_prior_local_paths import local_query


def measured():
    d = detection()
    return replace(
        request(),
        grasps=candidates_for_prior(d),
        world_from_pick_box=Transform(d.box_xyz, d.box_quat),
    )


def test_production_pick_rotation_and_place_offset_share_same_G():
    req = measured()
    g = req.grasps[0]
    slot = replace(
        req.placements[0], container_from_box=Transform((3, 4, 0.5), (0, 0, 1, 0))
    )
    q = local_query(req, slot, g)
    assert q["pick_box_world"].compose(q["box_from_tcp"]).translation == pytest.approx(
        (1, 2.04, 0.6)
    )
    assert q["place_box_world"].compose(q["box_from_tcp"]).translation == pytest.approx(
        (2.96, 4, 0.6)
    )
    assert q["pick_approach_world"].translation == pytest.approx((1, 2.04, 0.9))
    assert q["pick_lift_world"].translation == pytest.approx((1, 2.04, 0.95))
    assert q["place_entry_world"].translation == pytest.approx((2.96, 4, 0.7))


@pytest.mark.parametrize("valid", [False, True])
def test_rigid_clear_is_insufficient_and_witness_is_preserved(valid):
    class Port:
        def check(self, *args, **kwargs):
            return CollisionVerdict(True, "CLEAR")

        def check_local(self, *args, **kwargs):
            return CollisionVerdict(
                valid,
                "LOCAL_PATHS_VALID" if valid else "NO_PLACE_LOCAL_BRANCH",
                local_paths_json='{"witness":"fixture"}',
            )

    result = compute_prior(measured(), Port())
    assert bool(result.choices) == valid
    assert result.trace[-1].stage_index == -1
    assert result.trace[-1].verdict.local_paths_json == '{"witness":"fixture"}'


def test_changed_pickup_invalidates_cache_and_missing_references_refuse():
    req = measured()
    assert replace(req, world_from_pick_box=I).reuse_key != req.reuse_key
    with pytest.raises(ValueError, match="references required"):
        local_query(req, req.placements[0], replace(req.grasps[0], pick_references=()))
    with pytest.raises(ValueError, match="disagree"):
        local_query(
            replace(req, world_from_pick_box=I), req.placements[0], req.grasps[0]
        )


@pytest.mark.parametrize("damage", ["empty", "nan", "disconnect", "names", "none"])
def test_local_witness_rejects_truncation_and_discontinuity(damage):
    from copy import deepcopy
    from luggage_msgs.msg import GraspLocalBranch
    from trajectory_msgs.msg import JointTrajectoryPoint
    from luggage_planning.placement_prior_local_paths import valid_branch_paths

    branch = GraspLocalBranch()
    branch.incoming.joint_names = ["joint"]
    branch.incoming.points = [
        JointTrajectoryPoint(positions=[0.0]),
        JointTrajectoryPoint(positions=[0.1]),
    ]
    branch.outgoing.joint_names = ["joint"]
    branch.outgoing.points = [
        JointTrajectoryPoint(positions=[0.1]),
        JointTrajectoryPoint(positions=[0.2]),
    ]
    assert valid_branch_paths([branch])
    if damage == "empty":
        branch.incoming.points = []
    if damage == "nan":
        branch.outgoing.points[-1].positions = [float("nan")]
    if damage == "disconnect":
        branch.outgoing.points[0].positions = [0.5]
    if damage == "names":
        branch.outgoing.joint_names = ["other"]
    assert valid_branch_paths([branch]) == (damage == "none")


def test_measured_request_cannot_bypass_local_checks_by_omitting_pickup():
    result = compute_prior(replace(measured(), world_from_pick_box=None), object())
    assert result.status == "MISSING_PICK_CONTEXT"
    assert not result.choices
