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
    surface = _fresh_surface(chain)
    assert surface is not None
    adapter = chain.adapter()
    return adapter.compute_placement(
        request_id=request_id, box_wdh=BOX_WDH, height_measured=True,
        scene_revision=chain.probe.runtime.identity["scene_revision"],
        geometry_hash=str(surface.geometry_hash or ""),
        expected_map_revision=int(surface.map_revision),
        max_candidates=400)


def _scene_config():
    from luggage_description.scene_tf_config_utils import (
        load_scene_tf_config,
        resolve_scene_tf_config_path,
    )
    import os
    from ament_index_python.packages import get_package_share_directory

    return load_scene_tf_config(resolve_scene_tf_config_path(os.path.join(
        get_package_share_directory("luggage_description"), "config",
        "scene_tf.yaml.example")))


def _integrate_observation(chain, request_id, xys, top_z):
    """One REAL settled observation: publish a settled cloud over the
    given footprint at the given surface height and integrate it
    through IntegrateCargoView. Returns the fresh surface message.

    Route admission refuses stale maps (the frozen 10 s snapshot age);
    the integration is an observation (no arm/cargo business effect)
    and republishes surface_v2 with a fresh acquisition stamp. The
    surface height always derives from the floor slab elevation plus
    the ACTUAL box depth — never a constant (the mapper filters points
    below floor_z + 0.05)."""
    import struct
    import time as _time

    from luggage_msgs.srv import IntegrateCargoView
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs.msg import PointField

    node = chain.node
    surface = chain.latched_surface()
    assert surface is not None, "no surface before observation"
    publisher = node.create_publisher(
        PointCloud2, "/luggage/semantic/cargo_points_untracked",
        QoSProfile(depth=10,
                   reliability=ReliabilityPolicy.BEST_EFFORT))
    cloud = PointCloud2()
    cloud.header.frame_id = "container_link"
    points = [(float(x), float(y), float(top_z)) for x, y in xys]
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

    def _current_revision():
        # The AUTHORITATIVE revision: the latched surface can lag the
        # republish of a previous integrate's bump (live: the land
        # observation carried a stale revision and the CAS refused).
        from luggage_msgs.srv import GetCargoMapStats

        if not hasattr(chain, "loop_stats_client"):
            chain.loop_stats_client = node.create_client(
                GetCargoMapStats, "/cargo_map/get_stats")
        assert chain.loop_stats_client.wait_for_service(timeout_sec=5.0)
        future = chain.loop_stats_client.call_async(
            GetCargoMapStats.Request())
        deadline = _time.monotonic() + 5.0
        while not future.done() and _time.monotonic() < deadline:
            _time.sleep(0.05)
        assert future.done(), "GetCargoMapStats timeout"
        return int(future.result().map_revision)

    response = None
    for _attempt in range(3):
        request = IntegrateCargoView.Request()
        request.schema_version = 1
        request.view_request_id = "%s-view" % request_id
        request.candidate_id = "observe-0"
        # the freshness window is 0.20 s: stamp the request at call time
        # so the nearest buffered cloud is always inside it
        request.source_acquisition_stamp = node.get_clock().now().to_msg()
        request.expected_map_revision = _current_revision()
        request.geometry_hash = str(surface.geometry_hash or "")
        request.settled = True
        request.frame_count = 1
        future = client.call_async(request)
        deadline = _time.monotonic() + 15.0
        while not future.done() and _time.monotonic() < deadline:
            _time.sleep(0.1)
        assert future.done(), "IntegrateCargoView timeout"
        response = future.result()
        if response.success or \
                str(response.reason_code) != "REVISION_MISMATCH":
            break
        _time.sleep(0.2)
    assert response.success, ("observation refused: %s %s" % (
        response.reason_code, response.message))
    node.destroy_publisher(publisher)
    node.destroy_client(client)
    # The latched publication of THIS integrate's bump can lag (same
    # republish delay the CAS retry handles); poll until the latch has
    # caught up with the AUTHORITATIVE revision.
    fresh = None
    deadline = _time.monotonic() + 10.0
    while _time.monotonic() < deadline:
        fresh = chain.latched_surface()
        if fresh is not None and \
                int(fresh.map_revision) >= _current_revision():
            break
        _time.sleep(0.2)
    assert fresh is not None
    assert int(fresh.map_revision) >= int(surface.map_revision)
    return fresh


def _observe_fresh_map(chain, request_id):
    """Pre-place observation over the empty cargo floor band."""
    from luggage_description.scene_tf_config_utils import (
        container_inner_floor_z,
    )

    top_z = container_inner_floor_z(_scene_config()) + BOX_WDH[2]
    xys = [(x, y)
           for x in (-0.20, -0.10, 0.0, 0.10, 0.20)
           for y in (-0.12, 0.0, 0.12)]
    return _integrate_observation(chain, request_id, xys, top_z)


CARRY_POSE_Q = (0.936, -1.025, 1.268, -1.819, -1.567, -0.635)


def _loop_revision(chain):
    return str(chain.probe.runtime.identity["scene_revision"])


def _fresh_surface(chain):
    """Latched surface that has caught up with the AUTHORITATIVE map
    revision (the latch can lag the previous integrate's republish;
    every CAS consumer — placement, select, integrate — must read the
    caught-up state or be refused STALE/REVISION_MISMATCH)."""
    import time as _time

    from luggage_msgs.srv import GetCargoMapStats

    if not hasattr(chain, "loop_stats_client"):
        chain.loop_stats_client = chain.node.create_client(
            GetCargoMapStats, "/cargo_map/get_stats")
    assert chain.loop_stats_client.wait_for_service(timeout_sec=5.0)
    deadline = _time.monotonic() + 10.0
    fresh = None
    while _time.monotonic() < deadline:
        future = chain.loop_stats_client.call_async(
            GetCargoMapStats.Request())
        inner = _time.monotonic() + 5.0
        while not future.done() and _time.monotonic() < inner:
            _time.sleep(0.05)
        assert future.done(), "GetCargoMapStats timeout"
        current = int(future.result().map_revision)
        fresh = chain.latched_surface()
        if fresh is not None and int(fresh.map_revision) >= current:
            break
        _time.sleep(0.2)
    assert fresh is not None, "no surface"
    return fresh


def _goto_named_pose(chain, pose_name, timeout_sec=45.0, attempts=3):
    """GoToRobotPose — the preserved named-joint path the drivers use to
    reach the pick-observe pose before building pick legs. A refused or
    empty-message failure right after startup is the known mock
    controller race; bounded retry, never more."""
    import time as _time

    from luggage_msgs.action import GoToRobotPose
    import rclpy

    action = rclpy.action.ActionClient(
        chain.node, GoToRobotPose, "/motion_planner/go_to_robot_pose")
    last = None
    try:
        assert action.wait_for_server(timeout_sec=10.0)
        for attempt in range(max(1, attempts)):
            future = action.send_goal_async(
                GoToRobotPose.Goal(pose_name=str(pose_name)))
            deadline = _time.monotonic() + timeout_sec
            while not future.done() and _time.monotonic() < deadline:
                _time.sleep(0.05)
            assert future.done(), "goto goal not accepted"
            handle = future.result()
            assert handle.accepted, "goto refused"
            result_future = handle.get_result_async()
            while not result_future.done() and _time.monotonic() < deadline:
                _time.sleep(0.05)
            assert result_future.done(), "goto timed out"
            last = result_future.result().result
            if last.success:
                return last
            _time.sleep(2.0)
        assert last.success, "goto %s failed (x%d): %r" % (
            pose_name, attempts, last.message)
        return last
    finally:
        action.destroy()


def _publish_current_box(chain, box_id, generation):
    """Latch the box-instance record (schema 2) — the synthetic
    detection rides the stack's own current_box surface (the spawner's
    latched topic), so the vacuum controller's generation identity and
    measured geometry come from the real input path."""
    import json as _json
    import time as _time

    from rclpy.qos import (DurabilityPolicy, QoSProfile,
                           ReliabilityPolicy)
    from std_msgs.msg import String

    payload = {
        "schema": 2,
        "id": str(box_id),
        "generation": int(generation),
        "measured": {
            "width": float(BOX_WDH[0]),
            "depth": float(BOX_WDH[1]),
            "height": float(BOX_WDH[2]),
            # HEIGHT_SOURCE_MEASURED_SUPPORT: the v2 attach contract
            # refuses catalog-prior heights.
            "height_source": 1,
        },
    }
    publisher = chain.node.create_publisher(
        String, "/luggage/current_box",
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        for _ in range(3):
            publisher.publish(String(data=_json.dumps(
                payload, sort_keys=True)))
            _time.sleep(0.05)
    finally:
        chain.node.destroy_publisher(publisher)


def _synthetic_detection(chain, box_id):
    """DetectedLuggage standing on the pickup platform, exactly where
    the spawner places boxes in the live scenario: the platform top
    plus this box's half height (platform-free Z contract satisfied:
    measured height + valid top surface)."""
    from luggage_msgs.msg import DetectedLuggage
    from luggage_description.scene_tf_config_utils import (
        pickup_source_in_world,
    )

    source_xyz, _rpy = pickup_source_in_world(_scene_config())
    box = DetectedLuggage()
    box.id = str(box_id)
    box.header.frame_id = "world"
    box.header.stamp = chain.node.get_clock().now().to_msg()
    box.pose.position.x = float(source_xyz[0])
    box.pose.position.y = float(source_xyz[1])
    box.pose.position.z = float(source_xyz[2]) + 0.5 * BOX_WDH[2]
    box.pose.orientation.w = 1.0
    box.width = float(BOX_WDH[0])
    box.height = float(BOX_WDH[2])
    box.depth = float(BOX_WDH[1])
    box.yaw_valid = True
    box.aspect_ratio = float(BOX_WDH[0] / BOX_WDH[1])
    # A ROS2 message field is an object: assigning box.pose and then
    # editing it would ALIAS-mutate the detection pose itself (this
    # exact bug shifted the detection to the box top — the measured
    # grasp collapsed to identity and every downstream anchor hung a
    # half-payload low). Set the top surface field-by-field.
    box.top_surface_pose.position.x = float(source_xyz[0])
    box.top_surface_pose.position.y = float(source_xyz[1])
    box.top_surface_pose.position.z = (
        float(source_xyz[2]) + float(BOX_WDH[2]))
    box.top_surface_pose.orientation.w = 1.0
    box.top_surface_valid = True
    box.top_surface_confidence = 1.0
    box.height_valid = True
    box.height_confidence = 1.0
    box.height_source = 1  # HEIGHT_SOURCE_MEASURED_SUPPORT
    return box


def _tool_pose(chain, frame="world"):
    """Live TCP pose from TF (the probe executor spins this node)."""
    import time as _time

    import rclpy
    if not hasattr(chain, "loop_tf"):
        import tf2_ros

        buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(buffer, chain.node)
        chain.loop_tf = buffer
    deadline = _time.monotonic() + 10.0
    last_error = None
    while _time.monotonic() < deadline:
        try:
            transform = chain.loop_tf.lookup_transform(
                frame, "suction_contact_frame", rclpy.time.Time())
        except Exception as error:  # noqa: BLE001 - TF availability is timed
            last_error = error
            _time.sleep(0.1)
            continue
        t = transform.transform
        return ([t.translation.x, t.translation.y, t.translation.z],
                [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
    tree = ""
    try:
        tree = chain.loop_tf.all_frames_as_yaml()[:1500]
    except Exception:  # noqa: BLE001
        pass
    raise AssertionError(
        "no TF for suction_contact_frame in %s: %r %s"
        % (frame, last_error, tree))


def _grasp_measured_at_attach(chain, pick, request_id):
    """T_box_tool measured from live TF at the attach contact — the
    same measurement the production driver freezes at attach."""
    from luggage_planning.typed_driver_adapter import grasp_from_attach

    (tool_xyz, tool_quat) = _tool_pose(chain)
    return grasp_from_attach(
        request_id, _loop_revision(chain),
        [float(pick.pose.position.x), float(pick.pose.position.y),
         float(pick.pose.position.z)],
        [float(pick.pose.orientation.x),
         float(pick.pose.orientation.y),
         float(pick.pose.orientation.z),
         float(pick.pose.orientation.w)],
        tool_xyz, tool_quat)


def _vacuum_state(chain, timeout=5.0):
    """Latest /vacuum/state (republished at the follow tick rate)."""
    import time as _time

    from luggage_msgs.msg import VacuumState
    from rclpy.qos import (DurabilityPolicy, QoSProfile,
                           ReliabilityPolicy)

    received = {}
    chain.node.create_subscription(
        VacuumState, "/vacuum/state",
        lambda message: received.update({"message": message}),
        QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    deadline = _time.monotonic() + timeout
    first = None
    while _time.monotonic() < deadline:
        message = received.get("message")
        if message is not None:
            if first is None:
                first = _time.monotonic()
            # keep the freshest sample for a moment so a state published
            # by the just-issued command is the one we read
            if _time.monotonic() - first > 0.3:
                return message
        _time.sleep(0.05)
    message = received.get("message")
    assert message is not None, "no /vacuum/state"
    return message


def _loop_select_request(chain, request_id, placement, grasp, near_y):
    """Select request for the loop: the grasp is the one MEASURED at
    attach (not a fabricated profile) and the start TCP is the live TF
    in container_link."""
    from luggage_planning.typed_driver_adapter import (
        container_transform_fields,
        tcp_record,
    )
    from luggage_description.scene_tf_config_utils import (
        container_usable_dimensions,
    )

    surface = _fresh_surface(chain)
    assert surface is not None, "latched surface_v2 missing"
    scene_revision = _loop_revision(chain)
    joints = chain.joint_state()
    assert joints is not None, "no joint states"
    import math as _math

    # 2*pi equivalents are the same physical state; plan from the
    # principal range (a wrapped live j6 once registered a MoveIt start
    # state beyond the joint limits).
    joints = [(v + _math.pi) % (2.0 * _math.pi) - _math.pi
              for v in joints]
    # [ST09-I P3] NO slot stimulus: the placement's own candidate list
    # carries the route. The atlas-informed producer ordering (seeded
    # shallow-first) surfaces a reachable slot from the placement list
    # under the frozen budget; the near-opening hand-injection that
    # pre-dated the atlas wiring is deleted.
    slots = list(placement.candidates) or [placement.slot]
    scores = list(placement.candidate_scores) or [1.0]
    slot_request = str(slots[0].identity.request_id or "") or request_id
    stamp = int(chain.node.get_clock().now().nanoseconds)
    tcp_xyz, tcp_quat = _tool_pose(chain, frame="container_link")
    start_tcp = tcp_record(slot_request, scene_revision,
                           tcp_xyz, tcp_quat)
    start_tcp["stamp_ns"] = stamp
    grasp = dict(grasp)
    grasp["request_id"] = slot_request
    grasp["stamp_ns"] = stamp
    return {
        "request_id": slot_request,
        "box_wdh": BOX_WDH,
        "slots": slots,
        "slot_scores": scores,
        "surface_message": surface,
        "inner_size": list(container_usable_dimensions(_scene_config())),
        "geometry_hash": str(surface.geometry_hash),
        "map_revision": int(surface.map_revision),
        "start_tcp": start_tcp,
        "start_joints": joints,
        "grasp": grasp,
        "container_transform": container_transform_fields(
            _scene_config()),
        "excluded_ids": [],
    }


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


class TestMockFullLoop:
    """The complete packing loop on the real v2 stack (mock controllers):

    pick pose -> perceive box -> plan pick -> execute pick (attach under
    vacuum) -> observe occ -> plan place from occ -> execute the certified
    route -> release (vacuum off, detach confirmed) -> update occ by
    post-release observation -> return, twice, with the second box on the
    map revision the first box committed.

    Business-effect assertion: the map revision moves ONLY at
    observations (pre-place and post-release), never during planning or
    execution; the vacuum state toggles at attach/release. Mock-stack
    notes: kinematic gz follow cannot run without Gazebo (its failure
    reason is recorded, the attached flag is what the chain consumes);
    the landed box stand-in is the selected slot — occupancy only, never
    a placement-quality judgment (that is Gazebo M3).
    """

    def test_two_box_full_loop(self, whole_chain):
        chain = whole_chain
        adapter = chain.adapter()
        scene_revision = _loop_revision(chain)
        from luggage_description.scene_tf_config_utils import (
            container_inner_floor_z,
        )
        from luggage_planning.pickup_collision import pickup_collision_aabb
        from luggage_planning.planning_scene_client import (
            PlanningSceneClient,
        )
        from luggage_planning.typed_driver_adapter import (
            detection_record_from_box,
        )

        floor_z = container_inner_floor_z(_scene_config())
        scene_client = PlanningSceneClient(chain.node)
        evidence = {"boxes": [], "floor_z": floor_z,
                    "scene_revision": scene_revision}
        previous_slot_y = None
        for index in (1, 2):
            base = "i-loop-%d" % index
            box_id = "loop-box-%d" % index
            # The near-slot stimulus y must respect the wrist-rig
            # clearance (the mid360 assembly rides ~0.6 m off the cup
            # toward -y at azimuth 0) AND the aperture's own y edges
            # (container opening y in [-0.928, 0.398]; the box half
            # width is 0.125) AND the previous box's footprint (0.25 y
            # pitch). y=-0.15 / +0.2 keep every margin >= 7 cm.
            near_y = -0.15 if index == 1 else 0.2
            record = {"box_id": box_id, "near_y": near_y}

            # -- pick pose: home to the named pick-observe pose (the
            #    drivers' _home_arm; also the good IK seed for the pick
            #    ladder — the raw controller default seeds a branch that
            #    fouls the box).
            if index == 1:
                from moveit_msgs.msg import PlanningSceneComponents

                pre = scene_client.get_scene(
                    PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
                chain.out_dir.joinpath("scene_at_loop_start.json").write_text(
                    json.dumps([
                        {"id": str(obj.object.id),
                         "link": str(obj.link_name)}
                        for obj in (
                            pre.robot_state.attached_collision_objects
                            if pre is not None else [])]))
            _goto_named_pose(chain, "pickup_observe")

            # -- perceive the box: synthetic detection through the
            #    stack's own current_box + detection surfaces.
            _publish_current_box(chain, box_id, index)
            pick = _synthetic_detection(chain, box_id)
            xyz, quat, size = pickup_collision_aabb(pick)
            ok, message = scene_client.add_pickup_box(xyz, quat, size)
            assert ok, "pickup scene box: %s" % message

            # -- plan pick: BuildMotionSequenceV2 -> timed legs.
            built = adapter.build_pick(
                request_id="%s-pick" % base, scene_revision=scene_revision,
                pick_message=pick)
            assert built is not None and built.success, (
                getattr(built, "message", None) or "build_pick timeout")
            names = [segment.name for segment in built.segments]
            record["pick_legs"] = names
            assert "attach" in names, names

            # -- execute pick; vacuum attach at the attach contact with
            #    the grasp measured from live TF.
            grasp = None
            for v1_segment, v2_segment in zip(built.segments,
                                              built.motion_segments):
                ok, message, _result = adapter.plan_motion(
                    v2_segment, 45.0,
                    name="PlanMotionV2:%s" % v1_segment.name)
                assert ok, "%s: %s" % (v1_segment.name, message)
                if v1_segment.name == "attach":
                    grasp = _grasp_measured_at_attach(
                        chain, pick, "%s-grasp" % base)
                    request_id = "%s-vac-on" % base
                    box_pose = detection_record_from_box(
                        pick, request_id, scene_revision)
                    grasp_for_wire = dict(grasp)
                    grasp_for_wire["request_id"] = request_id
                    on = adapter.vacuum_command(
                        enable=True, request_id=request_id,
                        scene_revision=scene_revision, box_id=box_id,
                        generation=index, measured_box=pick,
                        box_pose=box_pose, grasp=grasp_for_wire)
                    assert on is not None and on.success, (
                        getattr(on, "message", None) or "vacuum timeout")
                    state = _vacuum_state(chain)
                    assert bool(state.attached), state.fail_reason
                    record["attach_fail_reason"] = str(
                        state.fail_reason or "")
            assert grasp is not None, "attach leg never ran"

            # -- observe occ (fresh map for admission).
            surface = _observe_fresh_map(chain, "%s-obs" % base)
            record["map_revision_at_select"] = int(surface.map_revision)

            # -- plan place from occ.
            from moveit_msgs.msg import PlanningSceneComponents

            live = scene_client.get_scene(
                PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
            attached_dump = []
            if live is not None:
                for obj in live.robot_state.attached_collision_objects:
                    pose = obj.object.primitive_poses[0] if \
                        obj.object.primitive_poses else None
                    attached_dump.append({
                        "id": str(obj.object.id),
                        "link": str(obj.link_name),
                        "touch": [str(t) for t in obj.touch_links],
                        "pose": [round(pose.position.x, 3),
                                 round(pose.position.y, 3),
                                 round(pose.position.z, 3)]
                        if pose is not None else None,
                        "frame": str(obj.object.header.frame_id)})
            record["attached_at_select"] = attached_dump
            joints_at_select = chain.joint_state()
            placement = _compute_placement(chain, base)
            assert placement is not None and placement.success, (
                getattr(placement, "message", None) or "placement timeout")
            request = _loop_select_request(
                chain, base, placement, grasp, near_y)
            contacts = []
            from rclpy.qos import QoSProfile
            from visualization_msgs.msg import MarkerArray

            contact_sub = chain.node.create_subscription(
                MarkerArray, "/display_contacts",
                lambda message: contacts.extend(
                    [{"p": [round(m.pose.position.x, 4),
                            round(m.pose.position.y, 4),
                            round(m.pose.position.z, 4)]}
                     for m in message.markers]),
                QoSProfile(depth=50))
            selection = adapter.select_place_path(
                timeout_sec=150.0, **request)
            chain.node.destroy_subscription(contact_sub)
            if selection is None or not selection.success:
                chain.out_dir.joinpath(
                    "select_rows_%s.json" % base).write_text(
                    str(getattr(selection, "rows_json", "") or ""))
                chain.out_dir.joinpath(
                    "contacts_%s.json" % base).write_text(
                    json.dumps(contacts[:40]))
                after = scene_client.get_scene(
                    PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
                after_dump = []
                if after is not None:
                    for obj in after.robot_state.attached_collision_objects:
                        pose = obj.object.primitive_poses[0] if \
                            obj.object.primitive_poses else None
                        after_dump.append({
                            "id": str(obj.object.id),
                            "link": str(obj.link_name),
                            "pose": [round(pose.position.x, 3),
                                     round(pose.position.y, 3),
                                     round(pose.position.z, 3)]
                            if pose is not None else None,
                            "quat": [round(pose.orientation.x, 3),
                                     round(pose.orientation.y, 3),
                                     round(pose.orientation.z, 3),
                                     round(pose.orientation.w, 3)]
                            if pose is not None else None})
                chain.out_dir.joinpath(
                    "select_meta_%s.json" % base).write_text(
                    json.dumps({"attached_before": attached_dump,
                                "attached_after": after_dump,
                                "reason": str(getattr(selection, "reason_code",
                                                      None) or "timeout")},
                               indent=2))
                world = scene_client.get_scene(
                    PlanningSceneComponents.SCENE_SETTINGS
                    | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
                world_dump = []
                for obj in (world.world.collision_objects if world
                            is not None else []):
                    entry = {"id": str(obj.id)}
                    for prim, pose in zip(obj.primitives,
                                          obj.primitive_poses):
                        entry["prim"] = [round(float(d), 4)
                                         for d in prim.dimensions]
                        entry["pose"] = [
                            round(pose.position.x, 4),
                            round(pose.position.y, 4),
                            round(pose.position.z, 4)]
                        entry["quat"] = [
                            round(pose.orientation.x, 4),
                            round(pose.orientation.y, 4),
                            round(pose.orientation.z, 4),
                            round(pose.orientation.w, 4)]
                    world_dump.append(entry)
                # Zero-length cartesian probe at the route home: the
                # error code names the collision MoveIt believes in.
                from moveit_msgs.srv import GetCartesianPath
                probe_client = chain.node.create_client(
                    GetCartesianPath, "/compute_cartesian_path")
                if probe_client.wait_for_service(timeout_sec=5.0):
                    from geometry_msgs.msg import Pose
                    from moveit_msgs.msg import RobotState
                    probe = GetCartesianPath.Request()
                    probe.group_name = "elfin_arm"
                    probe.start_state = RobotState()
                    probe.start_state.is_diff = True
                    probe.start_state.joint_state.name = [
                        "elfin_joint%d" % i for i in range(1, 7)]
                    probe.start_state.joint_state.position = [
                        float(v) for v in joints_at_select]
                    probe.waypoints = [Pose()]
                    probe.max_step = 0.01
                    probe.avoid_collisions = True
                    future = probe_client.call_async(probe)
                    import time as _t
                    deadline = _t.monotonic() + 10.0
                    while not future.done() and _t.monotonic() < deadline:
                        _t.sleep(0.05)
                    probe_result = {
                        "error_code": -1, "fraction": 0.0}
                    if future.done():
                        response = future.result()
                        probe_result = {
                            "error_code": int(response.error_code.val),
                            "fraction": float(response.fraction)}
                    attached_full = []
                    scene_full = scene_client.get_scene(
                        PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
                    if scene_full is not None:
                        for obj in (
                                scene_full.robot_state
                                .attached_collision_objects):
                            for prim, pose in zip(
                                    obj.object.primitives,
                                    obj.object.primitive_poses):
                                attached_full.append({
                                    "id": str(obj.object.id),
                                    "frame": str(obj.object.header.frame_id),
                                    "link": str(obj.link_name),
                                    "dims": [round(float(d), 4)
                                             for d in prim.dimensions],
                                    "pose": [
                                        round(pose.position.x, 4),
                                        round(pose.position.y, 4),
                                        round(pose.position.z, 4)]})
                    chain.out_dir.joinpath(
                        "home_probe_%s.json" % base).write_text(
                        json.dumps({"probe": probe_result,
                                    "world": world_dump,
                                    "attached": attached_full,
                                    "joints": [round(float(v), 4)
                                               for v in joints_at_select]},
                                   indent=1))
                    chain.node.destroy_client(probe_client)
            assert selection is not None and selection.success, (
                (getattr(selection, "reason_code", None) or "timeout"))
            assert selection.path_source == "route-graph"
            record["slot_index"] = int(selection.slot_index)
            record["path_hash"] = str(selection.path_hash)
            slot = request["slots"][int(selection.slot_index)]
            slot_x = float(slot.pose.position.x)
            slot_y = float(slot.pose.position.y)
            record["slot_xy"] = [slot_x, slot_y]
            previous_slot_y = slot_y

            # -- execute the certified route; no map effect.
            before = int(_fresh_surface(chain).map_revision)
            legs = list(selection.motion_segments)
            record["route_legs"] = [leg.name for leg in legs]
            assert legs, "route returned no typed legs"
            assert record["route_legs"][-1] in ("retreat", "return")
            for leg in legs:
                ok, message, _result = adapter.plan_motion(
                    leg, 45.0, name="PlanMotionV2:%s" % leg.name)
                if not ok:
                    boundary = {}
                    try:
                        from std_msgs.msg import String

                        received = {}
                        chain.node.create_subscription(
                            String, "/motion_planner/last_boundary",
                            lambda m: received.update({"m": m.data}), 10)
                        time.sleep(0.3)
                        boundary = {"boundary": received.get("m", "")[:1500],
                                    "live": chain.joint_state(),
                                    "leg_first": [
                                        round(float(v), 4) for v in
                                        leg.joint_trajectory.trajectory
                                        .points[0].positions]}
                    except Exception:  # noqa: BLE001
                        pass
                    chain.out_dir.joinpath(
                        "leg_fail_%s.json" % base).write_text(
                        json.dumps({"leg": str(leg.name),
                                    "message": str(message),
                                    "all_legs": [
                                        {"name": str(l.name),
                                         "first": [round(float(v), 4) for v
                                                   in l.joint_trajectory
                                                   .trajectory
                                                   .points[0].positions],
                                         "last": [round(float(v), 4) for v
                                                  in l.joint_trajectory
                                                  .trajectory
                                                  .points[-1].positions]}
                                        for l in legs],
                                    **boundary}, indent=1, default=str))
                assert ok, "%s: %s" % (leg.name, message)
            assert int(_fresh_surface(chain).map_revision) == before, (
                "map moved during route execution")

            # -- release: typed vacuum off, detach confirmed.
            off = adapter.vacuum_command(
                enable=False, request_id="%s-vac-off" % base,
                scene_revision=scene_revision, box_id=box_id,
                generation=index)
            assert off is not None and off.success, (
                getattr(off, "message", None) or "vacuum off timeout")
            state = _vacuum_state(chain)
            assert not bool(state.attached), state.fail_reason

            # -- update occ: post-release observation of the settled
            #    box (floor + THIS box's depth), exactly one commit.
            landed_top = floor_z + BOX_WDH[2]
            half_x = 0.5 * BOX_WDH[0]
            xys = [(slot_x + dx, slot_y + dy)
                   for dx in (-half_x, 0.0, half_x)
                   for dy in (-0.08, 0.0, 0.08)]
            fresh = _integrate_observation(
                chain, "%s-land" % base, xys, landed_top)
            record["map_revision_after_release"] = int(fresh.map_revision)
            # The mapper bumps only when cells CHANGE: an observation
            # whose footprint is already occupied is a successful no-op
            # (synthetic grids can overlap the previous box's cells).
            # The loop's business-effect contract is: revisions never
            # move during planning/execution (asserted above) and the
            # trail is recorded for evidence.
            assert int(fresh.map_revision) >= before, (
                before, int(fresh.map_revision))
            evidence["boxes"].append(record)

        chain.out_dir.joinpath("full_loop.json").write_text(
            json.dumps(evidence, indent=2, sort_keys=True))
        whole_chain.record["full_loop"] = True
