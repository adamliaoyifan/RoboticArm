"""Behavioral lease/admission/provenance regressions; no ROS or Gazebo needed."""
from copy import deepcopy
from types import SimpleNamespace

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
