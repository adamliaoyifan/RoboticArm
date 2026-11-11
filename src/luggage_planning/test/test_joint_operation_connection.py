"""No geometric connection is executable; consume only bound complete evidence."""
from copy import deepcopy
import time
from types import SimpleNamespace as NS

import pytest

from luggage_planning.joint_operation_connection import (
    NativeSnapshotConnectionPort, validate_connection_response,
)
from luggage_planning.joint_operation_contracts import OPERATION_JOINTS


def response():
    def point(q):
        return NS(positions=q, velocities=[], accelerations=[], effort=[],
                  time_from_start=NS(sec=0, nanosec=0))
    return NS(request_id="req", snapshot_id="snapshot", model_sha256="model",
              scene_sha256="scene", success=True, complete=True, reason="CHECKED",
              checked_states=3, geometric_path=NS(
                  header=NS(frame_id="", stamp=NS(sec=0, nanosec=0)),
                  joint_names=list(OPERATION_JOINTS),
                  points=[point([0.]*6), point([.01]*6), point([.02]*6)]))


def check(message):
    return validate_connection_response(message, request_id="req", snapshot_id="snapshot",
        model_sha256="model", scene_sha256="scene", start=(0.,)*6, goal=(.02,)*6)


def test_valid_bound_geometric_path_keeps_original_knots():
    message = response()
    result = check(message)
    assert result.success and result.complete and result.checked_states == 3
    assert result.positions == tuple(tuple(p.positions) for p in message.geometric_path.points)
    message.geometric_path.points[0].positions[0] = 1.
    assert result.positions[0][0] == 0.


@pytest.mark.parametrize("field", ["request_id", "snapshot_id", "model_sha256", "scene_sha256"])
def test_stale_or_foreign_response_never_consumed(field):
    message = response()
    setattr(message, field, "other")
    assert check(message).reason == "CONNECTION_IDENTITY_MISMATCH"


@pytest.mark.parametrize("mutation,reason", [
    (lambda r: setattr(r, "complete", False), "INVALID_CONNECTION_PATH"),
    (lambda r: setattr(r, "success", False), "REFUSED_CONNECTION_HAS_PATH"),
    (lambda r: setattr(r.geometric_path.header, "frame_id", "world"), "INVALID_CONNECTION_PATH"),
    (lambda r: r.geometric_path.joint_names.reverse(), "INVALID_CONNECTION_PATH"),
    (lambda r: setattr(r.geometric_path.points[1], "velocities", [0.]*6), "INVALID_GEOMETRIC_CONNECTION"),
    (lambda r: setattr(r.geometric_path.points[1].time_from_start, "nanosec", 1), "INVALID_GEOMETRIC_CONNECTION"),
    (lambda r: setattr(r.geometric_path.points[1], "positions", [float("nan")]*6), "INVALID_GEOMETRIC_CONNECTION"),
    (lambda r: setattr(r.geometric_path.points[1], "positions", [6.28]*6), "UNDERSAMPLED_CONNECTION"),
    (lambda r: setattr(r.geometric_path.points[0], "positions", [.001]*6), "CONNECTION_ENDPOINT_MISMATCH"),
    (lambda r: setattr(r, "checked_states", 2), "INCOMPLETE_CONNECTION_CHECKS"),
])
def test_partial_untied_or_retimed_evidence_is_refused(mutation, reason):
    message = deepcopy(response())
    mutation(message)
    result = check(message)
    assert not result.success and not result.positions and result.reason == reason


def test_search_timeout_has_no_path_and_is_incomplete_not_unreachable():
    message = response()
    message.success = message.complete = False
    message.reason = "DEADLINE"
    message.geometric_path.points = []
    message.geometric_path.joint_names = []
    result = check(message)
    assert not result.success and not result.complete and not result.positions
    assert result.reason == "DEADLINE"


def test_cancelled_or_inflight_query_cannot_submit_another_native_request():
    port = object.__new__(NativeSnapshotConnectionPort)
    port.inflight = NS(done=lambda: False)
    # No client/service exists here: any attempted submission would fail.
    result = port.connect(None, (0.,)*6, (0.,)*6, request_id="req",
                          deadline=time.monotonic()+1., cancelled=lambda: False)
    assert result.reason == "CONNECTION_BACKEND_BUSY"
    result = port.connect(None, (0.,)*6, (0.,)*6, request_id="req",
                          deadline=time.monotonic()+1., cancelled=lambda: True)
    assert result.reason == "CANCELLED_OR_DEADLINE"
