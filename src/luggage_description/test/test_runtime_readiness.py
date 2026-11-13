"""Behavioral lease/admission/provenance regressions; no ROS or Gazebo needed."""
from copy import deepcopy
from types import SimpleNamespace
from pathlib import Path

import pytest

from luggage_description.frame_contracts import ContractError
from luggage_description.runtime_profile import validate_identity, canonical_hash
from luggage_description.runtime_manifest import collect_report, validate_manifest, SCHEMA
from luggage_description.runtime_readiness import ReadinessGate


def identity():
    return dict(contract_version=2, scene_revision="scene", calibration_revision="cal",
                urdf_sha256="a"*64, srdf_sha256="b"*64, tool_link="tool",
                joint_names=["j1"], geometry_hash="hull", atlas_sha256="c"*64,
                atlas_certification_sha256="d"*64,
                endpoints={"/motion": "luggage_msgs/action/PlanMotionV2"})


@pytest.fixture
def runtime(tmp_path):
    root = tmp_path / "install"
    system = tmp_path / "ros"
    root.mkdir(); system.mkdir()
    modules = []
    for kind, origin, directory in (("module", "application", root),
                                    ("generated_message", "application", root),
                                    ("type_support", "system", system)):
        path = directory / kind
        path.write_text(kind)
        modules.append((SimpleNamespace(__file__=str(path)), kind, origin))
    report = collect_report("motion", identity(), {"speed": 1}, modules,
                            {"app": str(root), "ros": {"origin": "system", "path": str(system)}})
    expected = {key: deepcopy(report[key]) for key in ("resolved_config", "artifacts", "package_prefixes")}
    manifest = dict(schema=SCHEMA, identity=identity(), install_root=str(root), system_root=str(system),
                    processes={"motion": expected}, endpoints={"/motion": {
                        "type": identity()["endpoints"]["/motion"], "provider": "motion", "clients": ["motion"]}})
    now = [0.0]
    gate = ReadinessGate(identity(), ["motion"], root, clock=lambda: now[0], manifest=manifest)
    providers = {"/motion": [{"peer": "motion", "type": identity()["endpoints"]["/motion"]}]}
    return gate, report, providers, now, manifest


def test_standard_interface_registry():
    record = identity()
    record["endpoints"]["/tf"] = "tf2_msgs/msg/TFMessage"
    assert validate_identity(record) == record
    record["endpoints"]["/tf"] = "anything/msg/UnregisteredV2"
    with pytest.raises(ContractError, match="ENDPOINT_TYPE_MISMATCH"):
        validate_identity(record)


def test_mutable_heartbeats_and_explicit_readmission(runtime):
    gate, report, providers, now, _ = runtime
    gate.observe(report)
    token = gate.admit(providers, "request", endpoint="/motion", client="motion")
    now[0] = 1
    report.update(heartbeat_stamp_ns=1234, readiness={"state": "READY"})
    gate.observe(report)
    gate.require_admission(token, providers, "request")
    now[0] = 4
    with pytest.raises(ContractError, match="PEER_LEASE_EXPIRED"):
        gate.observe(report)
    effects = []
    with pytest.raises(ContractError):
        gate.require_admission(token, providers, "request")
        effects.append("motion")
    gate.readmit()
    gate.observe(report)
    with pytest.raises(ContractError, match="ADMISSION_REVOKED"):
        gate.require_admission(token, providers, "request")
        effects.append("motion")
    assert effects == []
    fresh = gate.admit(providers, "new", endpoint="/motion", client="motion")
    gate.require_admission(fresh, providers, "new")


@pytest.mark.parametrize("fault,code", [
    ("artifact", "STALE_PROCESS_ARTIFACT"), ("overlay", "WRONG_OVERLAY"),
    ("digest", "PEER_IDENTITY_MISMATCH"), ("restart", "PEER_RESTART"),
    ("provider", "ENDPOINT_PROVIDER_MISMATCH"), ("client", "ENDPOINT_CLIENT_MISMATCH"),
    ("duplicate", "DUPLICATE_PROVIDER"), ("type", "ENDPOINT_TYPE_MISMATCH")])
def test_faults_have_zero_effects(runtime, fault, code):
    gate, report, providers, now, _ = runtime
    gate.observe(report)
    token = gate.admit(providers, "r", endpoint="/motion", client="motion")
    effects = []
    with pytest.raises(ContractError, match=code):
        if fault == "artifact": report["artifacts"][0]["sha256"] = "f"*64
        if fault == "overlay": report["artifacts"][0]["path"] = "/wrong/module"
        if fault == "digest": report["contract_digest"] = "f"*64
        if fault == "restart": report["incarnation"] = "new"
        gate.observe(report)
        if fault == "provider": providers["/motion"][0]["peer"] = "impostor"
        if fault == "duplicate": providers["/motion"] *= 2
        if fault == "type": providers["/motion"][0]["type"] = "luggage_msgs/action/PlanMotion"
        if fault == "client": gate.admit(providers, "r", endpoint="/motion", client="impostor")
        gate.require_admission(token, providers, "r")
        effects.append("motion")
    assert effects == []


def test_manifest_unknown_and_local_hash(runtime):
    gate, report, _, _, manifest = runtime
    report["artifacts"][0]["sha256"] = canonical_hash("expected value")
    with pytest.raises(ContractError, match="STALE_PROCESS_ARTIFACT"):
        gate.observe(report)
    manifest["unknown"] = 1
    with pytest.raises(ContractError, match="RUNTIME_FIELD_MISMATCH"):
        validate_manifest(manifest)


def test_explicit_shared_tf_providers(runtime):
    gate, report, _, _, manifest = runtime
    manifest["processes"]["robot_state"] = deepcopy(manifest["processes"]["motion"])
    manifest["identity"]["endpoints"] = {"/tf": "tf2_msgs/msg/TFMessage"}
    manifest["endpoints"] = {"/tf": dict(type="tf2_msgs/msg/TFMessage", provider=["motion", "robot_state"], clients=["motion"])}
    gate = ReadinessGate(manifest["identity"], ["motion", "robot_state"], manifest["install_root"], manifest=manifest)
    report["identity"] = manifest["identity"]
    report["contract_digest"] = canonical_hash(manifest["identity"])
    gate.observe(report)
    report["peer"] = "robot_state"
    gate.observe(report)
    providers = {"/tf": [dict(peer=peer, type="tf2_msgs/msg/TFMessage") for peer in ["motion", "robot_state"]]}
    gate.require_ready(providers)
    providers["/tf"].append(providers["/tf"][0])
    with pytest.raises(ContractError, match="DUPLICATE_PROVIDER"):
        gate.require_ready(providers)


@pytest.fixture
def action_runtime(runtime):
    from luggage_description.runtime_action_readiness import ActionReadiness
    from luggage_description.runtime_graph import endpoint_registry, identity_endpoints, resolve_peers
    _gate, template, _providers, now, manifest = runtime
    peers = resolve_peers()
    registry = endpoint_registry(peers)
    value = identity()
    value["endpoints"] = identity_endpoints(registry)
    gate = ReadinessGate(value, peers[:-1], manifest["install_root"], clock=lambda: now[0])
    actions = ActionReadiness(gate, registry, peers[-1])
    reports = {}
    for peer in peers[:-1]:
        report = deepcopy(template)
        report.update(peer=peer, identity=value, contract_digest=canonical_hash(value))
        for item in report["artifacts"]:
            if item["origin"] == "system":
                path = Path(manifest["install_root"]) / "local_type_support"
                path.write_text("type_support")
                item.update(origin="application", path=str(path))
        report["package_prefixes"] = {"app": manifest["install_root"]}
        reports[peer] = report
    from luggage_description.runtime_graph import live_providers
    providers = live_providers(registry, set(peers))
    return actions, reports, providers, now


def test_unloaded_motion_has_no_placement_or_waypoint_dependency(action_runtime):
    actions, reports, providers, _now = action_runtime
    for peer, report in reports.items():
        if peer not in ("placement_planner", "waypoint_generator"):
            actions.observe(report)
    with pytest.raises(ContractError, match="MISSING_PEER"):
        actions.full.require_ready(providers)
    gate = actions.gate("unloaded_motion")
    token = gate.admit(providers, "return")
    gate.require_admission(token, providers, "return")
    assert gate.identity == actions.full.identity


@pytest.mark.parametrize("fault", ["expiry", "restart"])
def test_unrelated_revocation_cannot_poison_motion_scope(action_runtime, fault):
    actions, reports, providers, now = action_runtime
    for report in reports.values():
        actions.observe(report)
    gate = actions.gate("unloaded_motion")
    token = gate.admit(providers, "return")
    if fault == "expiry":
        now[0] = 1.5
        for peer, report in reports.items():
            if peer != "placement_planner":
                actions.observe(report)
        now[0] = 2.1
    else:
        reports["placement_planner"]["incarnation"] = "restarted"
    with pytest.raises(ContractError):
        actions.observe(reports["placement_planner"])
    with pytest.raises(ContractError):
        actions.full.require_ready(providers)
    gate.require_admission(token, providers, "return")


@pytest.mark.parametrize("peer", ["trajectory_selector", "motion_planner", "vacuum_controller",
                                  "scene_manager", "cargo_volume_mapper"])
@pytest.mark.parametrize("fault", ["expiry", "restart", "identity", "type"])
def test_required_dependency_failure_blocks_unloaded_motion(action_runtime, peer, fault):
    actions, reports, providers, now = action_runtime
    for report in reports.values():
        actions.observe(report)
    gate = actions.gate("unloaded_motion")
    token = gate.admit(providers, "return")
    if fault == "expiry":
        now[0] = 1.5
        for other, report in reports.items():
            if other != peer:
                actions.observe(report)
        now[0] = 2.1
    elif fault == "type":
        from luggage_description.runtime_graph import OWNED_ENDPOINTS
        endpoint = next(name for name, entry in OWNED_ENDPOINTS.items() if entry[1] == peer)
        providers[endpoint] = ["wrong/type"]
    else:
        reports[peer] = deepcopy(reports[peer])
        if fault == "restart":
            reports[peer]["incarnation"] = "new"
        else:
            reports[peer]["identity"]["scene_revision"] = "changed"
        with pytest.raises(ContractError):
            actions.observe(reports[peer])
    with pytest.raises(ContractError):
        gate.require_admission(token, providers, "return")


def test_renewable_planning_lease_invalidates_old_request_not_new_request(runtime):
    old, report, providers, now, manifest = runtime
    gate = ReadinessGate(old.identity, ["motion"], manifest["install_root"],
                         clock=lambda: now[0], manifest=manifest, renewable_peers=["motion"])
    gate.observe(report)
    token = gate.admit(providers, "old", endpoint="/motion", client="motion")
    now[0] = 2.1
    with pytest.raises(ContractError, match="PEER_LEASE_EXPIRED"):
        gate.require_ready(providers)
    gate.observe(report)
    with pytest.raises(ContractError, match="ADMISSION_REVOKED"):
        gate.require_admission(token, providers, "old")
    fresh = gate.admit(providers, "new", endpoint="/motion", client="motion")
    gate.require_admission(fresh, providers, "new")
    report["incarnation"] = "new"
    with pytest.raises(ContractError, match="PEER_RESTART"):
        gate.observe(report)


def test_incomplete_or_carrying_certificate_never_selects_unloaded_scope():
    from luggage_description.runtime_action_readiness import certified_motion_scope
    good = dict(load_state="unloaded", route_digest="r", certification_id="c", leg_digest="l")
    assert certified_motion_scope(good) == "unloaded_motion"
    for key in good:
        partial = dict(good)
        partial.pop(key)
        assert certified_motion_scope(partial) is None
    assert certified_motion_scope(dict(good, load_state="carrying")) is None
    assert certified_motion_scope(None) is None


def test_node_runtime_scope_survives_async_effect_boundary(action_runtime):
    import threading
    from luggage_description.ros_frame_bindings.runtime import NodeRuntime
    actions, reports, providers, _now = action_runtime
    for peer, report in reports.items():
        if peer not in ("placement_planner", "waypoint_generator"):
            actions.observe(report)
    runtime = object.__new__(NodeRuntime)
    runtime.action_readiness = actions
    runtime._providers = lambda: providers
    runtime._lock = threading.Lock()
    runtime.counters = {}
    token = runtime.admit_request("return", scope="unloaded_motion")
    runtime.authorize_effect(token, "return")
    with pytest.raises(ContractError, match="MISSING_PEER"):
        runtime.require_graph_ready()
    wrong_scope = dict(token, scope=None)
    with pytest.raises(ContractError):
        runtime.authorize_effect(wrong_scope, "return")
    with pytest.raises(ContractError, match="UNKNOWN_ACTION_SCOPE"):
        runtime.require_graph_ready(scope="skip_checks")


def test_typed_client_uses_scope_only_for_complete_unloaded_certificate(action_runtime):
    import threading
    from luggage_description.ros_frame_bindings.runtime import NodeRuntime
    from luggage_planning.deploy.typed_driver_adapter import TypedDriverAdapter
    from luggage_msgs.msg import MotionSegmentV2
    actions, reports, providers, _now = action_runtime
    for peer, report in reports.items():
        if peer not in ("placement_planner", "waypoint_generator"):
            actions.observe(report)
    runtime = object.__new__(NodeRuntime)
    runtime.action_readiness = actions
    runtime._providers = lambda: providers
    runtime._lock = threading.Lock()
    runtime.counters = {}
    submitted = []
    def send(goal):
        submitted.append(goal)
        return SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(accepted=False))
    adapter = SimpleNamespace(runtime=runtime, _plan=SimpleNamespace(
        wait_for_server=lambda **kwargs: True, send_goal_async=send))
    good = dict(load_state="unloaded", route_digest="route", certification_id="cert", leg_digest="leg")
    TypedDriverAdapter.plan_motion(adapter, MotionSegmentV2(), 1.0, admission=good)
    assert len(submitted) == 1
    assert submitted[0].admission_load_state == "unloaded"
    for admission in (None, dict(good, load_state="carrying"), dict(good, leg_digest="")):
        with pytest.raises(ContractError, match="MISSING_PEER"):
            TypedDriverAdapter.plan_motion(adapter, MotionSegmentV2(), 1.0, admission=admission)
    assert len(submitted) == 1



def test_occupancy_commit_does_not_require_placement_but_requires_mapper(action_runtime):
    actions, reports, providers, _now = action_runtime
    gate = actions.gate("occupancy_commit")
    assert gate.peers == ("cargo_volume_mapper",)
    with pytest.raises(ContractError, match="MISSING_PEER"):
        gate.require_ready(providers)
    actions.observe(reports["cargo_volume_mapper"])
    token = gate.admit(providers, "commit-verified-box")
    gate.require_admission(token, providers, "commit-verified-box")
    with pytest.raises(ContractError, match="MISSING_PEER"):
        actions.full.require_ready(providers)
    reports["cargo_volume_mapper"]["incarnation"] = "restarted"
    with pytest.raises(ContractError, match="PEER_RESTART"):
        actions.observe(reports["cargo_volume_mapper"])
    with pytest.raises(ContractError, match="PEER_RESTART"):
        gate.require_admission(token, providers, "commit-verified-box")
