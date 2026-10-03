#!/usr/bin/env python3
"""GoToRobotPoseV2 live service coverage (ST09-I S5.7-6).

Direct-service tests against the whole-chain mock stack (the I-suite
two-box loop drives the v1 action, not this service): a named pose
returns COMPLETE certified joint legs (executed through the adapter
with per-leg admission — certify A, execute A), a second call at the
pose is already_there (zero legs, nothing promoted), a pose the
fixture file does not carry is a named refusal, and the
EnsureCycleStartV2 adjudication answers from the selector's OWN
latched state.
"""
import json
import os
import sys
import time

import rclpy.action  # noqa: F401  (the v1 helper below needs it)

_ROOT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
for entry in ("src/luggage_planning",):
    path = os.path.join(_ROOT, entry)
    if path not in sys.path:
        sys.path.append(path)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_mock_whole_chain as _chain  # noqa: E402


def _wait_future(node, future, timeout):
    event = {"done": False}
    future.add_done_callback(lambda _f: event.__setitem__("done", True))
    deadline = time.monotonic() + float(timeout)
    while not event["done"] and time.monotonic() < deadline:
        time.sleep(0.05)
    return future.done(), (future.result() if future.done() else None)


def _stats_client(chain):
    from luggage_msgs.srv import GetCargoMapStats

    client = getattr(chain.node, "_stats_client", None)
    if client is None:
        client = chain.node.create_client(
            GetCargoMapStats, "/cargo_map/get_stats")
        chain.node._stats_client = client
    return client


def _settle_map(chain, timeout=15.0):
    """Wait until the mapper's revision stops moving.

    Right after stack boot the readiness integrations are still
    committing; a goto planned between two commits is honestly refused
    MAP_STALE by the revalidator. Settle first, then plan.
    """
    from luggage_msgs.srv import GetCargoMapStats

    client = _stats_client(chain)
    deadline = time.monotonic() + float(timeout)
    last = -1
    while time.monotonic() < deadline:
        done, stats = _wait_future(
            chain.node, client.call_async(GetCargoMapStats.Request()), 5.0)
        assert done and stats is not None and stats.success
        revision = int(stats.map_revision)
        if revision == last:
            return revision
        last = revision
        time.sleep(1.0)
    return last


def _goto(chain, pose_name, timeout=60.0):
    from luggage_msgs.srv import GoToRobotPoseV2

    client = getattr(chain.node, "_goto_v2_client", None)
    if client is None:
        client = chain.node.create_client(
            GoToRobotPoseV2,
            "/trajectory_selector/goto_robot_pose_v2")
        chain.node._goto_v2_client = client
    assert client.wait_for_service(timeout_sec=10.0)
    stats_client = _stats_client(chain)
    assert stats_client.wait_for_service(timeout_sec=10.0)
    from luggage_msgs.srv import GetCargoMapStats

    done, stats = _wait_future(
        chain.node, stats_client.call_async(GetCargoMapStats.Request()), 5.0)
    assert done and stats is not None and stats.success
    request = GoToRobotPoseV2.Request()
    request.request_id = "goto-live-%s" % pose_name
    request.pose_name = str(pose_name)
    request.current_joint_state = [
        float(v) for v in (chain.joint_state() or [])]
    request.scene_revision = str(_chain._loop_revision(chain))
    request.latest_map_revision = int(stats.map_revision)
    done, response = _wait_future(
        chain.node, client.call_async(request), timeout)
    return (response if done else None), request


def _execute_goto_legs(chain, response):
    """Run certified goto legs through the adapter with per-leg scopes."""
    adapter = chain.adapter()
    admission = {
        "route_digest": str(response.route_digest),
        "certification_id": str(response.certification_id),
        "admission_map_revision": int(response.admission_map_revision),
    }
    digests = [str(d) for d in response.leg_digests]
    states = [str(s) for s in response.leg_load_states]
    for index, segment in enumerate(response.motion_segments):
        leg_admission = dict(admission)
        if index < len(digests):
            leg_admission["leg_digest"] = digests[index]
            leg_admission["load_state"] = states[index]
        ok, message, _result = adapter.plan_motion(
            segment, 45.0,
            name="PlanMotionV2:goto_%d" % index,
            admission=leg_admission)
        assert ok, message


def test_goto_service_returns_certified_legs(whole_chain):
    chain = whole_chain
    _chain._goto_named_pose(chain, "pickup_observe")
    # The goto producer's map gate is fail-closed on the LATEST map —
    # ensure the latched surface + a settled revision exist even when
    # this file runs before the whole-chain loop. The goto target is
    # the PORTAL pose (the fixture's pickup_observe_sim equals the
    # home pose — that combination is the already_there case below).
    _chain._observe_fresh_map(chain, "goto-pre")
    _settle_map(chain)
    response, _request = _goto(chain, "portal_observe_sim")
    assert response is not None, "goto service timeout"
    assert response.success, "%s %s" % (
        response.reason_code, response.detail)
    assert not bool(response.already_there)
    assert response.motion_segments, "no typed legs"
    assert response.route_digest and response.certification_id
    assert list(response.leg_digests), "no per-leg digests"
    assert all(str(s) == "unloaded"
               for s in response.leg_load_states), list(
                   response.leg_load_states)
    # certify A, execute A
    _execute_goto_legs(chain, response)
    chain.out_dir.joinpath("goto_v2.json").write_text(json.dumps({
        "pose": "portal_observe_sim",
        "route_digest": str(response.route_digest),
        "certification_id": str(response.certification_id),
        "legs": [str(s.name) for s in response.motion_segments],
        "certify_ms": float(response.certify_ms)}, indent=2))


def test_goto_service_already_there_after_arrival(whole_chain):
    """At the portal from the previous test: zero legs, nothing
    promoted, nothing to submit; then a second certified motion back
    to the pickup pose exercises the other direction."""
    chain = whole_chain
    response, _request = _goto(chain, "portal_observe_sim")
    assert response is not None, "goto service timeout"
    assert response.success, "%s %s" % (
        response.reason_code, response.detail)
    assert bool(response.already_there)
    assert not response.motion_segments
    assert response.route_digest == ""
    chain.record["goto_already_there"] = True
    # a second certified route (portal -> pickup observe). The frozen
    # 10 s snapshot-age window applies: a fresh observation (and a
    # settled revision) before planning, exactly like the loop.
    _chain._observe_fresh_map(chain, "goto-return")
    _settle_map(chain)
    response, _request = _goto(chain, "pickup_observe_sim")
    assert response is not None, "goto service timeout"
    assert response.success, "%s %s" % (
        response.reason_code, response.detail)
    assert not bool(response.already_there)
    assert response.motion_segments
    _execute_goto_legs(chain, response)


def test_goto_refuses_pose_the_file_does_not_carry(whole_chain):
    chain = whole_chain
    response, _request = _goto(chain, "pickup_observe")
    assert response is not None, "goto service timeout"
    assert not response.success
    assert str(response.reason_code) == "POSE_NOT_VALID", (
        response.reason_code)


def test_ensure_cycle_start_live_verdict(whole_chain):
    """[ST09-I S5.7-6 §2] the adjudication service answers from the
    selector's OWN latched state (arm at the named pickup pose from the
    goto test above: outside; vacuum off: unloaded) -> ALREADY_READY."""
    from luggage_msgs.srv import EnsureCycleStartV2

    chain = whole_chain
    client = chain.node.create_client(
        EnsureCycleStartV2,
        "/trajectory_selector/ensure_cycle_start_v2")
    try:
        assert client.wait_for_service(timeout_sec=10.0)
        request = EnsureCycleStartV2.Request()
        request.scene_revision = str(_chain._loop_revision(chain))
        done, response = _wait_future(
            chain.node, client.call_async(request), 10.0)
        assert done and response is not None
        assert int(response.outcome) == int(
            EnsureCycleStartV2.Response.ALREADY_READY), (
            response.reason_code, response.zone,
            response.attachment_state, response.detail)
        assert str(response.zone) == "outside_container"
        assert str(response.attachment_state) == "unloaded"
        assert str(response.reason_code) == "OK"
        chain.record["ensure_cycle_start"] = True
    finally:
        chain.node.destroy_client(client)
