"""Runtime reports roundtrip actual generated ROS messages."""
from types import SimpleNamespace
from rclpy.serialization import serialize_message, deserialize_message
from luggage_msgs.msg import RuntimeIdentityV2
from luggage_description.runtime_manifest import collect_report, identity_fingerprint
from luggage_description.ros_frame_bindings.runtime import report_to_message, report_from_message


def test_wire_preserves_local_identity(tmp_path):
    identity = dict(contract_version=2, scene_revision="scene", calibration_revision="cal",
                    urdf_sha256="a"*64, srdf_sha256="b"*64, tool_link="tool", joint_names=["j2", "j1"],
                    geometry_hash="hull", atlas_sha256="c"*64, atlas_certification_sha256="d"*64,
                    endpoints={"/tf": "tf2_msgs/msg/TFMessage"})
    registry = {"/tf": {"type": "tf2_msgs/msg/TFMessage", "provider": "scene", "clients": ["motion"]}}
    modules = []
    for kind in ("module", "generated_message", "type_support"):
        path = tmp_path/kind
        path.write_text(kind)
        modules.append((SimpleNamespace(__file__=str(path)), kind, "application"))
    report = collect_report("scene", identity, {"use_sim_time": True}, modules, {"scene": str(tmp_path)})
    first = report_to_message(report, registry)
    decoded = report_from_message(deserialize_message(serialize_message(first), RuntimeIdentityV2), registry)
    assert identity_fingerprint(decoded) == identity_fingerprint(report)
    next_message = report_to_message(report, registry, sequence=9007199254740993, readiness_code="READY")
    decoded_next = report_from_message(deserialize_message(serialize_message(next_message), RuntimeIdentityV2), registry)
    assert identity_fingerprint(decoded_next) == identity_fingerprint(decoded)
    assert decoded_next["heartbeat_sequence"] == 9007199254740993
    assert decoded_next["readiness"]["ready"]


def test_heartbeat_runs_with_paused_sim_clock(tmp_path):
    import time
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from luggage_description.ros_frame_bindings.runtime import RuntimeHeartbeat, TOPIC
    identity = dict(contract_version=2, scene_revision="scene", calibration_revision="cal",
                    urdf_sha256="a"*64, srdf_sha256="b"*64, tool_link="tool", joint_names=["j1"],
                    geometry_hash="hull", atlas_sha256="c"*64, atlas_certification_sha256="d"*64,
                    endpoints={"/tf": "tf2_msgs/msg/TFMessage"})
    registry = {"/tf": {"type": "tf2_msgs/msg/TFMessage", "provider": "scene", "clients": ["motion"]}}
    modules = []
    for kind in ("module", "generated_message", "type_support"):
        path = tmp_path/kind
        path.write_text(kind)
        modules.append((SimpleNamespace(__file__=str(path)), kind, "application"))
    report = collect_report("scene", identity, {"use_sim_time": True}, modules, {"scene": str(tmp_path)})
    rclpy.init()
    node = Node("st09_heartbeat_test", parameter_overrides=[Parameter("use_sim_time", value=True)])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    received = []
    subscription = node.create_subscription(RuntimeIdentityV2, TOPIC, received.append, 100)
    heartbeat = RuntimeHeartbeat(node, report, registry, lambda: "MISSING_PEER")
    try:
        assert heartbeat.timer.timer_period_ns == 250_000_000
        deadline = time.monotonic()+3.0
        while len(received) < 5 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert len(received) >= 5
        assert node.get_clock().now().nanoseconds == 0
        assert all(not msg.ready for msg in received)
        assert len({msg.incarnation for msg in received}) == 1
        assert len({msg.heartbeat_sequence for msg in received}) == len(received)
        assert len({identity_fingerprint(report_from_message(msg, registry)) for msg in received}) == 1
    finally:
        node.destroy_timer(heartbeat.timer)
        node.destroy_subscription(subscription)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
