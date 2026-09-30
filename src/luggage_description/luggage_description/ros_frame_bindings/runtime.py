"""RuntimeIdentityV2 wire binding and non-authorizing heartbeat publisher."""
import json
from copy import deepcopy

from luggage_msgs.msg import RuntimeArtifactV2, RuntimeEndpointV2, RuntimeIdentityV2, RuntimePackageV2
from luggage_description.runtime_profile import canonical_hash, reject, validate_identity
from luggage_description.runtime_manifest import HEARTBEAT_PERIOD_SECONDS
from .identity import require_message

TOPIC = "/frame_contracts/v2/runtime_identity"
SCALAR_IDENTITY_FIELDS = (
    "scene_revision", "calibration_revision", "urdf_sha256", "srdf_sha256",
    "tool_link", "geometry_hash", "atlas_sha256", "atlas_certification_sha256",
)


def report_to_message(report, endpoint_registry, *, sequence=0, readiness_code="MISSING_PEER"):
    identity = validate_identity(report["identity"])
    if report.get("contract_digest") != canonical_hash(identity):
        reject("PEER_IDENTITY_MISMATCH", "contract_digest", canonical_hash(identity), report.get("contract_digest"))
    message = RuntimeIdentityV2()
    message.contract_version = 2
    message.peer = report["peer"]
    message.incarnation = report["incarnation"]
    message.contract_digest = report["contract_digest"]
    for field in SCALAR_IDENTITY_FIELDS:
        setattr(message, field, identity[field])
    message.joint_names = identity["joint_names"]
    for name, type_name in identity["endpoints"].items():
        endpoint = endpoint_registry[name]
        if endpoint["type"] != type_name:
            reject("ENDPOINT_TYPE_MISMATCH", name, type_name, endpoint["type"])
        message.endpoints.append(RuntimeEndpointV2(
            name=name, type_name=type_name, providers=([endpoint["provider"]] if isinstance(endpoint["provider"], str) else endpoint["provider"]), clients=endpoint["clients"]))
    message.resolved_config_json = json.dumps(report["resolved_config"], sort_keys=True, allow_nan=False)
    message.artifacts = [RuntimeArtifactV2(kind=item["kind"], origin=item["origin"],
                                         path=item["path"], sha256=item["sha256"])
                         for item in report["artifacts"]]
    for name, entry in report["package_prefixes"].items():
        if isinstance(entry, str):
            entry = {"path": entry, "origin": "application"}
        message.package_prefixes.append(RuntimePackageV2(name=name, origin=entry["origin"], prefix=entry["path"]))
    message.heartbeat_sequence = sequence
    message.ready = readiness_code == "READY"
    message.readiness_code = readiness_code
    return message


def report_from_message(message, endpoint_registry):
    require_message(message, RuntimeIdentityV2)
    identity = {field: getattr(message, field) for field in SCALAR_IDENTITY_FIELDS}
    endpoints = {}
    for item in message.endpoints:
        if item.name in endpoints:
            reject("DUPLICATE_PROVIDER", "endpoint", "unique endpoint", item.name)
        endpoints[item.name] = dict(type=item.type_name, provider=list(item.providers), clients=list(item.clients))
    normalized_registry = {
        name: dict(entry, provider=([entry["provider"]] if isinstance(entry["provider"], str)
                                   else list(entry["provider"])))
        for name, entry in endpoint_registry.items()}
    if endpoints != normalized_registry:
        reject("ENDPOINT_PROVIDER_MISMATCH", "registry", endpoint_registry, endpoints)
    identity.update(contract_version=int(message.contract_version), joint_names=list(message.joint_names),
                    endpoints={name: entry["type"] for name, entry in endpoints.items()})
    validate_identity(identity)
    prefixes = {}
    for item in message.package_prefixes:
        if item.name in prefixes:
            reject("INVALID_PROCESS_REPORT", "package_prefixes", "unique package", item.name)
        prefixes[item.name] = dict(origin=item.origin, path=item.prefix)
    try:
        config = json.loads(message.resolved_config_json)
    except (ValueError, TypeError):
        reject("INVALID_PROCESS_REPORT", "resolved_config", "JSON mapping", message.resolved_config_json)
    if not isinstance(config, dict) or not config:
        reject("INVALID_PROCESS_REPORT", "resolved_config", "nonempty mapping", config)
    return dict(peer=message.peer, incarnation=message.incarnation, identity=identity,
                contract_digest=message.contract_digest, resolved_config=config,
                artifacts=[dict(kind=item.kind, origin=item.origin, path=item.path, sha256=item.sha256)
                           for item in message.artifacts], package_prefixes=prefixes,
                heartbeat_sequence=int(message.heartbeat_sequence),
                readiness={"ready": message.ready, "code": message.readiness_code})


class RuntimeHeartbeat:
    """Advertise measured identity before readiness; never grant operator start.

    The caller supplies collect_report's locally measured result. Use a steady
    timer so paused simulation time cannot conceal a dead process lease.
    """
    def __init__(self, node, report, endpoint_registry, readiness):
        from rclpy.clock import Clock, ClockType
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        self.report = deepcopy(report)
        self.registry = deepcopy(endpoint_registry)
        self.readiness = readiness
        self.sequence = 0
        self.publisher = node.create_publisher(RuntimeIdentityV2, TOPIC, QoSProfile(
            depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))
        self.timer = node.create_timer(HEARTBEAT_PERIOD_SECONDS, self.publish,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish()

    def publish(self):
        self.publisher.publish(report_to_message(self.report, self.registry,
            sequence=self.sequence, readiness_code=self.readiness()))
        self.sequence += 1
