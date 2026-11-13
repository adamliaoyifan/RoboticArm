"""Whole-path equivalent proposals, actual limits, and archived-model conformance."""
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from luggage_planning.joint_operation.contracts import OPERATION_JOINTS
from luggage_planning.joint_operation.local_branch_search import LocalBranchSearch
from luggage_planning.joint_operation.planner import LocalJointBranch


def model(limits=None):
    limits = limits or ((-7., 7.),)*6
    urdf = SimpleNamespace(joints={name: {'type': 'revolute'} for name in OPERATION_JOINTS},
                          joint_limits=lambda: limits)
    return SimpleNamespace(urdf=urdf, path=OPERATION_JOINTS)


def branch(name, rows):
    return LocalJointBranch(name, tuple(rows[:2]), tuple(rows[1:]))


def constant(name, q):
    return branch(name, (q, q, q))


@pytest.mark.parametrize('joint', range(6))
def test_uniform_legal_turns_for_each_joint_preserve_source_and_contact_join(joint):
    reference = tuple(-3. if i == joint else 0. for i in range(6))
    rows = tuple(tuple(3.+offset if i == joint else .1*offset for i in range(6))
                 for offset in (0., .01, .02))
    raw = branch('place', rows)
    original = asdict(raw)
    trace = []
    proposals = LocalBranchSearch(model()).placements(constant('pick', reference), (raw,), trace=trace)
    assert len(proposals) == 2 and proposals[1] is raw
    selected = proposals[0]
    assert selected.incoming[-1] == selected.outgoing[0]
    assert selected.branch_id != raw.branch_id and asdict(raw) == original
    for source, shifted in zip(raw.incoming+raw.outgoing, selected.incoming+selected.outgoing):
        for i, (before, after) in enumerate(zip(source, shifted)):
            assert after-before == pytest.approx(-math.tau if i == joint else 0.)
            assert -7. <= after <= 7.
    assert trace[0][0] == 'equivalent_local_branch'
    assert trace[0][-1] == tuple(-1 if i == joint else 0 for i in range(6))


def test_interior_or_outgoing_bound_blocks_endpoint_only_shift():
    pick = constant('pick', (-3.,)*6)
    # Entry could move down one turn, but contact/retreat could not.
    for rows in (((3.,)*6, (-2.,)*6, (3.,)*6),
                 ((3.,)*6, (3.,)*6, (-2.,)*6)):
        raw = branch('place', rows)
        trace = []
        assert LocalBranchSearch(model(((-3.5, 3.5),)*6)).placements(
            pick, (raw,), trace=trace) == (raw,)
        assert not trace


def test_each_native_family_precedes_raw_fallback_and_order_is_deterministic():
    pick = constant('pick', (-3.,)*6)
    sources = tuple(constant(name, (value,)*6) for name, value in (('a', 3.), ('b', 3.1)))
    policy = LocalBranchSearch(model())
    trace = []
    proposals = policy.placements(pick, sources, trace=trace)
    assert len(proposals) == 4
    assert [p.branch_id.split(':')[0] for p in proposals] == ['a', 'b', 'a', 'b']
    assert proposals[2:] == sources
    assert policy.placements(pick, sources, trace=[]) == proposals


def test_identity_and_invalid_raw_branches_are_not_silently_repaired():
    pick = constant('pick', (0.,)*6)
    for q in ((.1,)*6, (8.,)*6):
        raw = constant('raw', q)
        trace = []
        result = LocalBranchSearch(model()).placements(pick, (raw,), trace=trace)
        assert len(result) == 1 and result[0] is raw and not trace


@pytest.mark.parametrize('defect', ('order', 'prismatic', 'infinite', 'reversed', 'count'))
def test_noncanonical_or_unbounded_models_rejected(defect):
    candidate = model()
    if defect == 'order':
        candidate.path = tuple(reversed(candidate.path))
    elif defect == 'prismatic':
        candidate.urdf.joints[OPERATION_JOINTS[0]]['type'] = 'prismatic'
    else:
        limits = list(candidate.urdf.joint_limits())
        if defect == 'count': limits.pop()
        else: limits[0] = (0., float('inf')) if defect == 'infinite' else (1., -1.)
        candidate.urdf.joint_limits = lambda: limits
    with pytest.raises(ValueError, match='canonical bounded revolute'):
        LocalBranchSearch(candidate)


def test_archived_full_model_proposal_matches_independently_checked_endpoint():
    source = os.environ.get('LOCAL_BRANCH_REPLAY_DIR')
    if not source:
        pytest.skip('requires preserved full-model sensor-driven local receipt')
    from rclpy.serialization import deserialize_message
    from luggage_msgs.srv import CheckGraspLocalPaths, PlanSnapshotJointPathV2
    from luggage_planning.joint_operation.certification import EndpointModel
    directory = Path(source)
    inputs = ('robot.urdf', 'robot.srdf', 'original_local_response.cdr', '2_shifted_request.cdr')
    hashes = {name: hashlib.sha256((directory/name).read_bytes()).hexdigest() for name in inputs}
    response = deserialize_message((directory/inputs[2]).read_bytes(), CheckGraspLocalPaths.Response)
    target = deserialize_message((directory/inputs[3]).read_bytes(), PlanSnapshotJointPathV2.Request)
    actual_model = EndpointModel((directory/inputs[0]).read_text(), (directory/inputs[1]).read_text(),
        position_tolerance_m=.002, orientation_tolerance_rad=.02)
    # This receipt has two native families. Select by actual message order, as production does.
    def local(item):
        return LocalJointBranch('recorded-seed:'+str(item.seed_index),
            tuple(tuple(p.positions) for p in item.incoming.points),
            tuple(tuple(p.positions) for p in item.outgoing.points))
    picks = tuple(map(local, response.pick_branches))
    places = tuple(map(local, response.place_branches))
    trace = []
    proposals = LocalBranchSearch(actual_model).placements(picks[0], places, trace=trace)
    selected, raw = proposals[1], places[1]
    assert selected.incoming[0] == pytest.approx(tuple(target.goal_joints), abs=1e-9)
    paths = {link.get('name'): actual_model.urdf.path_to(link.get('name'))
             for link in actual_model.urdf.root.findall('link')}
    max_error = 0.
    rows = raw.incoming+raw.outgoing
    for before, after in zip(rows, selected.incoming+selected.outgoing):
        for q, (lower, upper) in zip(after, actual_model.urdf.joint_limits()):
            assert lower <= q <= upper
        for path in paths.values():
            max_error = max(max_error, float(np.max(np.abs(
                actual_model.urdf.tcp(before, path)-actual_model.urdf.tcp(after, path)))))
    assert len(rows) == 99 and len(paths) == 23 and max_error <= 1e-9
    evidence = Path(os.environ['LOCAL_BRANCH_REPLAY_EVIDENCE'])
    evidence.mkdir(parents=True, exist_ok=False)
    (evidence/'case.json').write_text(json.dumps(dict(source=str(directory), source_sha256=hashes,
        trace=trace, selected=asdict(selected), raw=asdict(raw), pick=asdict(picks[0]),
        rows=len(rows), links=len(paths), max_fk_error=max_error), indent=2))
