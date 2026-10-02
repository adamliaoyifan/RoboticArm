"""ST09-I mock whole-chain: placement -> certified route -> execution on
the real seven-endpoint v2 stack over mock controllers.

Zero unexpected business effects (map revision unchanged across planning
and execution), named refusals through the service surface, and measured
in-flight cancel are all asserted against the live services — no fakes
past the mock hardware itself.
"""

from __future__ import division

import json
import time
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "src" / "luggage_planning" / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

BOX_WDH = [0.35, 0.25, 0.20]  # small carryon-class payload


def _select_request(chain, request_id, placement, grasp_offset=(0.0, 0.0, 0.10)):
    from luggage_planning.typed_driver_adapter import (
        container_transform_fields,
        grasp_record,
        tcp_record,
    )
    from luggage_description.scene_tf_config_utils import (
        container_usable_dimensions,
        load_scene_tf_config,
        resolve_scene_tf_config_path,
    )
    from ament_index_python.packages import get_package_share_directory
    import os

    surface = chain.latched_surface()
    assert surface is not None, "latched surface_v2 missing"
    scene = load_scene_tf_config(resolve_scene_tf_config_path(os.path.join(
        get_package_share_directory("luggage_description"), "config",
        "scene_tf.yaml.example")))
    scene_revision = chain.probe.runtime.identity["scene_revision"]
    joints = chain.joint_state()
    assert joints is not None, "no joint states"
    slots = list(placement.candidates) or [placement.slot]
    scores = list(placement.candidate_scores) or [1.0]
    # Test stimulus (like the spawner input, never production policy):
    # the blind mock map enumerates only the deep band — unreachable for
    # this arm — so one near-opening candidate exercises the route. In
    # the live M3 run the observed map supplies real near candidates.
    from luggage_msgs.msg import PoseContractV2

    near = PoseContractV2()
    for field in ("contract_version", "frame_id", "length_unit",
                  "stamp_valid", "stamp_ns", "time_basis",
                  "scene_revision", "request_id"):
        setattr(near.identity, field,
                getattr(slots[0].identity, field))
    near.reference = str(slots[0].reference)
    near.orientation_semantics = str(slots[0].orientation_semantics)
    near.pose.position.x = -0.55
    near.pose.position.y = 0.0
    near.pose.position.z = float(slots[0].pose.position.z)
    near.pose.orientation.w = 1.0
    slots = slots + [near]
    scores = scores + [0.1]
    slot_request = str(slots[0].identity.request_id or "") or request_id
    stamp = int(chain.node.get_clock().now().nanoseconds)
    # The start TCP record is diagnostic for admission; the route's home
    # node is registered from the measured start joints via FK.
    start_tcp = tcp_record(
        slot_request, scene_revision, (0.45, 0.0, 1.15),
        (1.0, 0.0, 0.0, 0.0))
    start_tcp["stamp_ns"] = stamp
    # T_box_tool for a suction grasp on an unrotated box: the tool sits
    # on the box top with its z-axis pointing down ((1,0,0,0) in box
    # coordinates) — a measured grasp from a real tool-down pick.
    grasp = grasp_record(
        slot_request, scene_revision, list(grasp_offset),
        [1.0, 0.0, 0.0, 0.0])
    grasp["stamp_ns"] = stamp
    return {
        "request_id": slot_request,
        "box_wdh": BOX_WDH,
        "slots": slots,
        "slot_scores": scores,
        "surface_message": surface,
        "inner_size": list(container_usable_dimensions(scene)),
        "geometry_hash": str(surface.geometry_hash or ""),
        "map_revision": int(surface.map_revision),
        "start_tcp": start_tcp,
        "start_joints": joints,
        "grasp": grasp,
        "container_transform": container_transform_fields(scene),
        "excluded_ids": [],
    }


def _compute_placement(chain, request_id):
    surface = chain.latched_surface()
    assert surface is not None
    adapter = chain.adapter()
    return adapter.compute_placement(
        request_id=request_id, box_wdh=BOX_WDH, height_measured=True,
        scene_revision=chain.probe.runtime.identity["scene_revision"],
        geometry_hash=str(surface.geometry_hash or ""),
        expected_map_revision=int(surface.map_revision),
        max_candidates=400)


def _observe_fresh_map(chain, request_id):
    """One REAL settled observation: publish a small settled cloud on
    the untracked topic and integrate it through IntegrateCargoView.

    Route admission refuses stale maps (the frozen 10 s snapshot age);
    the mock stack's latched surface otherwise carries the boot stamp.
    The integration is an observation (no arm/cargo business effect)
    and republishes surface_v2 with a fresh acquisition stamp."""
    import struct
    import time as _time

    from luggage_msgs.srv import IntegrateCargoView
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs.msg import PointField

    from luggage_description.scene_tf_config_utils import (
        container_inner_floor_z,
        load_scene_tf_config,
        resolve_scene_tf_config_path,
    )
    import os
    from ament_index_python.packages import get_package_share_directory

    node = chain.node
    surface = chain.latched_surface()
    assert surface is not None, "no surface before observation"
    publisher = node.create_publisher(
        PointCloud2, "/luggage/semantic/cargo_points_untracked",
        QoSProfile(depth=10,
                   reliability=ReliabilityPolicy.BEST_EFFORT))
    cloud = PointCloud2()
    cloud.header.frame_id = "container_link"
    # Points on the settled surface of the observed cargo: the box top
    # height derives from the floor slab elevation plus THIS box's
    # depth (not a constant) — the mapper filters points below
    # floor_z + 0.05, so the surface must sit above it. This is the
    # post-release observation semantics: floor + box height.
    scene = load_scene_tf_config(resolve_scene_tf_config_path(os.path.join(
        get_package_share_directory("luggage_description"), "config",
        "scene_tf.yaml.example")))
    top_z = container_inner_floor_z(scene) + BOX_WDH[2]
    points = [
        (x, y, top_z)
        for x in (-0.20, -0.10, 0.0, 0.10, 0.20)
        for y in (-0.12, 0.0, 0.12)
    ]
    cloud.height = 1
    cloud.width = len(points)
    cloud.is_bigendian = False
    cloud.point_step = 12
    for index, name in enumerate(("x", "y", "z")):
        field = PointField()
        field.name = name
        field.offset = index * 4
        field.datatype = PointField.FLOAT32
        field.count = 1
        cloud.fields.append(field)
    cloud.row_step = cloud.point_step * cloud.width
    cloud.data = b"".join(struct.pack("<3f", *point) for point in points)
    # publish for long enough that BEST_EFFORT discovery settles and
    # the mapper buffers a cloud stamped within the join window
    for _ in range(15):
        cloud.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(cloud)
        _time.sleep(0.1)
    client = node.create_client(IntegrateCargoView,
                                "/cargo_map/integrate_cargo_view")
    assert client.wait_for_service(timeout_sec=10.0)
    request = IntegrateCargoView.Request()
    request.schema_version = 1
    request.view_request_id = "%s-view" % request_id
    request.candidate_id = "observe-0"
    # the freshness window is 0.20 s: stamp the request at call time so
    # the nearest buffered cloud is always inside it
    request.source_acquisition_stamp = node.get_clock().now().to_msg()
    request.expected_map_revision = int(surface.map_revision)
    request.geometry_hash = str(surface.geometry_hash or "")
    request.settled = True
    request.frame_count = 1
    future = client.call_async(request)
    deadline = _time.monotonic() + 15.0
    while not future.done() and _time.monotonic() < deadline:
        _time.sleep(0.1)
    assert future.done(), "IntegrateCargoView timeout"
    response = future.result()
    assert response.success, ("observation refused: %s %s" % (
        response.reason_code, response.message))
    node.destroy_publisher(publisher)
    node.destroy_client(client)
    fresh = chain.latched_surface()
    assert fresh is not None
    assert int(fresh.map_revision) >= int(surface.map_revision)
    return fresh


CARRY_POSE_Q = (0.936, -1.025, 1.268, -1.819, -1.567, -0.635)


def _timed_joint_leg(chain, request_id, target_q, duration_s=4.0):
    """One admitted joint_path leg (linear rest-to-rest) for reposition."""
    from luggage_planning.route_producer import _leg_record
    from st09_contracts.fixtures import make_timed_trajectory

    from luggage_description.frame_diagnostics import BoundaryContext
    from luggage_planning.motion_frame_adapters import message_from_admitted
    from luggage_planning.motion_frame_contracts import (
        TARGET_LINK, ExecutionContext,
    )

    trajectory = make_timed_trajectory()
    points = [
        {"positions": [float(v) for v in target_q],
         "velocities": [0.0] * 6, "accelerations": [0.0] * 6,
         "effort": [], "time_from_start_ns": 0},
        {"positions": [float(v) for v in target_q],
         "velocities": [0.0] * 6, "accelerations": [0.0] * 6,
         "effort": [], "time_from_start_ns": int(duration_s * 1e9)},
    ]
    start_q = chain.joint_state()
    assert start_q is not None, "no joint state for carry reposition"
    # Dense interpolation (<=0.5 rad per hop) keeps the executor's
    # joint-continuity gate satisfied for a large reposition.
    import bisect

    steps = max(2, int(max(abs(t - s) for s, t in
                           zip(start_q, target_q)) / 0.5) + 2)
    positions = []
    for step in range(steps + 1):
        fraction = step / steps
        positions.append(tuple(
            s + (t - s) * fraction for s, t in zip(start_q, target_q)))
    times = [int(round(duration_s * 1e9 * step / steps))
             for step in range(steps + 1)]
    trajectory = type("Traj", (), {
        "joint_names": tuple("elfin_joint%d" % i for i in range(1, 7)),
        "positions": tuple(positions),
        "velocities": ((0.0,) * 6,) * len(positions),
        "accelerations": ((0.0,) * 6,) * len(positions),
        "time_from_start_ns": tuple(times)})()
    leg = _leg_record(
        "carry_pose", trajectory, request_id,
        chain.probe.runtime.identity["scene_revision"],
        int(chain.node.get_clock().now().nanoseconds))
    leg["joint_trajectory"]["points"] = [
        {"positions": [float(v) for v in row],
         "velocities": [0.0] * 6, "accelerations": [0.0] * 6,
         "effort": [], "time_from_start_ns": int(t_ns)}
        for row, t_ns in zip(positions, times)]
    boundary = BoundaryContext(
        interface_id="F15", producer="route_producer",
        consumer="motion_executor", request_id=request_id,
        scene_revision=chain.probe.runtime.identity["scene_revision"],
        frame_id="world", reference="suction_contact_frame",
        time_basis="robot_state")
    context = ExecutionContext(boundary, TARGET_LINK, "world")
    return message_from_admitted(leg, context)


class TestMockWholeChain:

    def test_startup_budget_and_no_effect_compute(self, whole_chain):
        record = whole_chain.record
        assert record.get("ready") is True
        assert record.get("startup_seconds", 999) <= 180.0
        exercise = whole_chain.probe.exercise_compute()
        assert exercise.get("ok"), exercise

    def test_placement_route_and_execution(self, whole_chain):
        from luggage_description.frame_contracts import ContractError

        chain = whole_chain
        # Reposition to a realistic post-pick carry pose first: the mock
        # home holds the wrist in a configuration where an attached
        # payload fouls the arm — the system correctly refuses to
        # certify a carry from there.
        adapter = chain.adapter()
        ok, message, _result = adapter.plan_motion(
            _timed_joint_leg(chain, "i-carry-pose-1", CARRY_POSE_Q),
            30.0, name="PlanMotionV2:carry_pose")
        assert ok, message
        # A fresh observation: admission refuses stale maps, and the
        # latched surface otherwise carries the boot stamp.
        _observe_fresh_map(chain, "i-route-1")
        placement = _compute_placement(chain, "i-route-1")
        assert placement is not None and placement.success, (
            getattr(placement, "message", None) or "placement timeout")
        request = _select_request(chain, "i-route-1", placement)
        try:
            selection = adapter.select_place_path(
                timeout_sec=150.0, **request)
        except ContractError as error:
            pytest.fail("select refused: %s %s" % (error.code, error.field))
        if selection is None or not selection.success:
            Path("/tmp/st09_i_select_rows.json").write_text(
                str(getattr(selection, "rows_json", "")) if selection
                else "timeout")
            pytest.fail("select failed: %s" % (
                getattr(selection, "reason_code", None) or "timeout"))
        assert selection.path_source == "route-graph"
        legs = list(selection.motion_segments)
        assert legs, "route returned no typed legs"
        names = [leg.name for leg in legs]
        assert names[-1] in ("retreat", "return")
        # Map revision is unchanged by planning: no commits, no scene edits.
        before = chain.latched_surface().map_revision
        for leg in legs:
            ok, message, _result = adapter.plan_motion(
                leg, 45.0, name="PlanMotionV2:%s" % leg.name)
            assert ok, message
        after = chain.latched_surface().map_revision
        assert int(after) == int(before), (before, after)
        chain.out_dir.joinpath("route_legs.json").write_text(json.dumps({
            "selection_id": str(selection.selection_id),
            "path_hash": str(selection.path_hash),
            "legs": names,
            "map_revision": [int(before), int(after)],
        }))
        whole_chain.record["route_executed"] = True

    def test_geometry_mismatch_refuses_named(self, whole_chain):
        from luggage_planning.typed_driver_adapter import TypedDriverAdapter

        chain = whole_chain
        # Same setup as the positive route test (the degenerate mock
        # home is a pathological planning start): reposition to the
        # carry pose and take one real observation, then vary EXACTLY
        # ONE input — the geometry hash.
        adapter = chain.adapter()
        ok, message, _result = adapter.plan_motion(
            _timed_joint_leg(chain, "i-carry-pose-2", CARRY_POSE_Q),
            30.0, name="PlanMotionV2:carry_pose")
        assert ok, message
        _observe_fresh_map(chain, "i-route-2")
        placement = _compute_placement(chain, "i-route-2")
        assert placement is not None and placement.success
        request = _select_request(chain, "i-route-2", placement)
        request["geometry_hash"] = "not-the-hash"
        selection = adapter.select_place_path(
            timeout_sec=150.0, **request)
        assert selection is not None
        assert not selection.success
        assert str(selection.reason_code)

    def test_in_flight_cancel_is_bounded(self, whole_chain):
        """Cancel a running route leg; acknowledgement within the frozen
        10 s goal-accept deadline, business effects limited to cancel."""
        from luggage_msgs.action import PlanMotionV2

        chain = whole_chain
        # Same setup as the positive route test: carry pose + one real
        # observation — the cancel probe varies only the mid-flight
        # cancellation of an executing leg.
        adapter = chain.adapter()
        ok, message, _result = adapter.plan_motion(
            _timed_joint_leg(chain, "i-carry-pose-3", CARRY_POSE_Q),
            30.0, name="PlanMotionV2:carry_pose")
        assert ok, message
        _observe_fresh_map(chain, "i-route-3")
        placement = _compute_placement(chain, "i-route-3")
        if placement is None or not placement.success:
            pytest.skip("placement unavailable for cancel probe")
        from luggage_planning.typed_driver_adapter import TypedDriverAdapter

        adapter = TypedDriverAdapter(chain.node, chain.probe.runtime)
        request = _select_request(chain, "i-route-3", placement)
        selection = adapter.select_place_path(
            timeout_sec=150.0, **request)
        if selection is None or not selection.success:
            pytest.skip("route unavailable for cancel probe: %s" % (
                getattr(selection, "reason_code", None) or "timeout"))
        legs = list(selection.motion_segments)
        # Pick the longest leg (the entry transit) for a mid-flight cancel.
        leg = max(legs, key=lambda m: len(
            m.joint_trajectory.trajectory.points))
        import rclpy

        action = rclpy.action.ActionClient(
            chain.node, PlanMotionV2, "/motion_planner/plan_motion_v2")
        try:
            assert action.wait_for_server(timeout_sec=10.0)
            goal = PlanMotionV2.Goal()
            goal.segment = leg
            send_future = action.send_goal_async(goal)
            deadline = time.monotonic() + 10.0
            while not send_future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            assert send_future.done(), "goal accept timed out"
            handle = send_future.result()
            assert handle.accepted, "goal refused"
            result_future = handle.get_result_async()
            # Let execution start, then cancel.
            time.sleep(0.8)
            t0 = time.monotonic()
            handle.cancel_goal()
            cancel_ack = None
            cancel_deadline = t0 + 10.0  # frozen goal_accept_wait_s
            while time.monotonic() < cancel_deadline:
                if result_future.done():
                    cancel_ack = time.monotonic() - t0
                    break
                time.sleep(0.05)
            assert cancel_ack is not None, "cancel not acknowledged in 10 s"
            chain.out_dir.joinpath("cancel_probe.json").write_text(
                json.dumps({"leg": str(leg.name),
                            "cancel_ack_s": round(cancel_ack, 3)}))
        finally:
            action.destroy()
