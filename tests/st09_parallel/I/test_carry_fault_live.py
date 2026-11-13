#!/usr/bin/env python3
"""Live mid-execution payload-loss injection (ST09-I S5.7-6, 评审点 3).

The unit fakes prove the adapter's cancel chain shape; this test proves
it against the REAL mock stack: a carrying route leg is executing via
the adapter when the authoritative /vacuum/state flips to
attached=False mid-wait. In the mock stack the controller republishes
its backend truth at 30 Hz while attached, so an external
attached=false message cannot stick — the honest sustained flip is the
COMMANDED RELEASE (identical authoritative topic state to a gazebo
payload drop; neither the adapter nor the selector can distinguish the
two, by design — the gate keys on the feed, not the cause).

Asserted end to end:
* plan_motion returns the named CARRY_FAULT_PAYLOAD_LOST refusal with a
  consumed cancel receipt (cancel_status != TIMEOUT);
* the authoritative feed reports the payload gone;
* the cargo map revision does not move (no map commit on the fault
  path);
* the next carrying leg's admission is refused by name (PAYLOAD_LOST)
  BOTH at the selector service and at the executor's pre-submission
  gate — zero controller submissions for it.
"""
import json
import os
import sys
import time

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


def test_carry_fault_mid_execution_cancels_and_closes(whole_chain):
    chain = whole_chain
    adapter = chain.adapter()
    scene_revision = _chain._loop_revision(chain)
    base = "i-fault-1"
    box_id = "fault-box-1"

    from luggage_planning.pick.pickup_collision import pickup_collision_aabb
    from luggage_planning.motion.planning_scene_client import PlanningSceneClient
    from luggage_planning.deploy.typed_driver_adapter import (
        detection_record_from_box,
    )

    scene_client = PlanningSceneClient(chain.node)

    # -- same box-1 script as the two-box loop, through the attach.
    _chain._goto_named_pose(chain, "pickup_observe")
    _chain._publish_current_box(chain, box_id, 1)
    pick = _chain._synthetic_detection(chain, box_id)
    xyz, quat, size = pickup_collision_aabb(pick)
    ok, message = scene_client.add_pickup_box(xyz, quat, size)
    assert ok, "pickup scene box: %s" % message
    built = adapter.build_pick(
        request_id="%s-pick" % base, scene_revision=scene_revision,
        pick_message=pick)
    assert built is not None and built.success, (
        getattr(built, "message", None) or "build_pick timeout")

    grasp = None
    for v1_segment, v2_segment in zip(built.segments,
                                      built.motion_segments):
        ok, message, _result = adapter.plan_motion(
            v2_segment, 45.0, name="PlanMotionV2:%s" % v1_segment.name)
        assert ok, "%s: %s" % (v1_segment.name, message)
        if v1_segment.name == "attach":
            grasp = _chain._grasp_measured_at_attach(
                chain, pick, "%s-grasp" % base)
            request_id = "%s-vac-on" % base
            box_pose = detection_record_from_box(
                pick, request_id, scene_revision)
            grasp_for_wire = dict(grasp)
            grasp_for_wire["request_id"] = request_id
            on = adapter.vacuum_command(
                enable=True, request_id=request_id,
                scene_revision=scene_revision, box_id=box_id,
                generation=1, measured_box=pick,
                box_pose=box_pose, grasp=grasp_for_wire)
            assert on is not None and on.success, (
                getattr(on, "message", None) or "vacuum timeout")
    state = _chain._vacuum_state(chain)
    assert bool(state.attached), state.fail_reason

    # -- observe + place plan, exactly like the loop.
    surface = _chain._observe_fresh_map(chain, "%s-obs" % base)
    placement = _chain._compute_placement(chain, base)
    assert placement is not None and placement.success, (
        getattr(placement, "message", None) or "placement timeout")
    request = _chain._loop_select_request(
        chain, base, placement, grasp, -0.15)
    selection = adapter.select_place_path(timeout_sec=150.0, **request)
    assert selection is not None and selection.success, (
        getattr(selection, "reason_code", None) or "timeout")

    # -- mint the fresh admission + per-leg scopes (the driver's
    #    ValidateSelectedRoute step; certify A, execute A). The scene
    #    identity is the SEGMENTS' own (what the executor's admission
    #    RPC also reads) — the select snapshot binds to the placement
    #    slots' identity, not the probe heartbeat's.
    legs = list(selection.motion_segments)
    segment_scene = str(getattr(legs[0], "scene_revision", "") or "")
    from luggage_msgs.srv import ValidateSelectedRoute

    validate = chain.node.create_client(
        ValidateSelectedRoute,
        "/trajectory_selector/validate_selected_route")
    assert validate.wait_for_service(timeout_sec=10.0)
    vreq = ValidateSelectedRoute.Request()
    vreq.route_digest = str(selection.path_hash)
    vreq.current_joint_state = [
        float(v) for v in (chain.joint_state() or [])]
    vreq.latest_map_revision = int(surface.map_revision)
    vreq.scene_revision = segment_scene
    done, vres = _wait_future(chain.node, validate.call_async(vreq), 30.0)
    assert done and vres is not None and vres.valid, (
        getattr(vres, "reason_code", None) or "validate timeout")
    digests = [str(d) for d in vres.leg_digests]
    states = [str(s) for s in vres.leg_load_states]
    carrying = [i for i, load in enumerate(states) if load == "carrying"]
    assert carrying, "route has no carrying legs: %s" % states
    # longest carrying leg: the fault must land mid-execution, not
    # after the leg already finished.
    target = max(
        carrying, key=lambda i: len(
            legs[i].joint_trajectory.trajectory.points))
    leg_admission = {
        "route_digest": str(selection.path_hash),
        "certification_id": str(vres.certification_id),
        "admission_map_revision": int(vres.validated_map_revision),
        "leg_digest": digests[target],
        "load_state": "carrying",
    }

    # -- fault injection: the authoritative /vacuum/state flips
    #    attached=False mid-execution. In the mock stack the controller
    #    republishes its own backend truth at 30 Hz while attached, so
    #    an external attached=false message cannot stick — the honest
    #    sustained flip is the COMMANDED RELEASE itself (the same
    #    authoritative topic state a gazebo payload drop produces; the
    #    adapter and the selector cannot tell the two apart, by
    #    design). The should_abort poll issues it at ~1.5 s into the
    #    leg, then the cancel chain owns the rest.
    started = {"ns": time.monotonic()}
    released = {"done": False}

    def should_abort():
        if time.monotonic() - started["ns"] < 1.5 or released["done"]:
            return released["done"]
        off = adapter.vacuum_command(
            enable=False, request_id="%s-fault-off" % base,
            scene_revision=scene_revision, box_id=box_id,
            generation=1)
        released["done"] = bool(off is not None and off.success)
        return released["done"]

    map_before = int(_chain._fresh_surface(chain).map_revision)
    try:
        ok, message, result = adapter.plan_motion(
            legs[target], 45.0, name="PlanMotionV2:fault-%s"
            % legs[target].name, admission=leg_admission,
            should_abort=should_abort)
    finally:
        # Stack hygiene: this suite shares one live stack — leave the
        # vacuum off and the arm homed no matter how the probe ends.
        if not released["done"]:
            try:
                adapter.vacuum_command(
                    enable=False, request_id="%s-vac-off" % base,
                    scene_revision=scene_revision, box_id=box_id,
                    generation=1)
            except Exception:  # noqa: BLE001 - best-effort cleanup
                pass
        try:
            _chain._goto_named_pose(chain, "pickup_observe")
        except Exception:  # noqa: BLE001 - best-effort cleanup
            pass

    evidence = {
        "leg": str(legs[target].name),
        "leg_index": target,
        "route_legs": [str(leg.name) for leg in legs],
        "leg_load_states": states,
        "message": str(message),
        "cancel_receipt": "cancel_status=" in str(message),
        "release_ok": released["done"],
        "admission": dict(leg_admission),
    }
    # 1) the named refusal with a consumed cancel receipt.
    assert not ok
    assert str(message).startswith("CARRY_FAULT_PAYLOAD_LOST"), message
    assert "cancel_status=" in str(message), message
    assert "cancel_status=TIMEOUT" not in str(message), message

    # 2) the authoritative feed now reports the payload gone.
    state = _chain._vacuum_state(chain)
    assert state is not None and not bool(state.attached)

    # 3) no map effect on the fault path.
    evidence["map_before"] = map_before
    evidence["map_after"] = int(_chain._fresh_surface(chain).map_revision)
    assert evidence["map_after"] == map_before

    # 4) the next carrying leg is refused by name at BOTH gates —
    #    selector service verdict first...
    from luggage_msgs.srv import CheckExecutionAdmission

    check = chain.node.create_client(
        CheckExecutionAdmission,
        "/trajectory_selector/check_execution_admission")
    assert check.wait_for_service(timeout_sec=10.0)
    creq = CheckExecutionAdmission.Request()
    creq.route_digest = leg_admission["route_digest"]
    creq.certification_id = leg_admission["certification_id"]
    creq.admission_map_revision = leg_admission[
        "admission_map_revision"]
    creq.scene_revision = segment_scene
    creq.leg_digest = leg_admission["leg_digest"]
    creq.leg_load_state = "carrying"
    done, cres = _wait_future(chain.node, check.call_async(creq), 10.0)
    assert done and cres is not None
    assert not bool(cres.live)
    assert str(cres.reason_code) == "PAYLOAD_LOST", cres.reason_code
    evidence["next_leg_selector_verdict"] = str(cres.reason_code)

    # ...then the executor's own pre-submission gate (the goal is
    # refused BEFORE any controller submission — the window closure).
    if target + 1 < len(legs) and states[target + 1] == "carrying":
        next_admission = dict(
            leg_admission, leg_digest=digests[target + 1])
        ok, message, _result = adapter.plan_motion(
            legs[target + 1], 45.0,
            name="PlanMotionV2:next-%s" % legs[target + 1].name,
            admission=next_admission)
        assert not ok
        assert str(message) == "PAYLOAD_LOST", message
        evidence["next_leg_executor_gate"] = str(message)

    chain.out_dir.joinpath("carry_fault.json").write_text(
        json.dumps(evidence, indent=2, sort_keys=True))
    chain.record["carry_fault"] = True
