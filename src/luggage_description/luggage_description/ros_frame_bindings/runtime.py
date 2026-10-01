"""RuntimeIdentityV2 wire binding and non-authorizing heartbeat publisher."""
import json
from copy import deepcopy

from luggage_msgs.msg import RuntimeArtifactV2, RuntimeEndpointV2, RuntimeIdentityV2, RuntimePackageV2
from luggage_description.frame_contracts import ContractError
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


def _generated_message_module():
    import luggage_msgs.msg as message_package
    return message_package


def _type_support_module(install_root):
    """The compiled type-support artifact of the generated messages."""
    import importlib
    from pathlib import Path
    for module_name in ("luggage_msgs.luggage_msgs_s__rosidl_typesupport_c",
                        "luggage_msgs_s.rosidl_typesupport_c"):
        try:
            return importlib.import_module(module_name)
        except ImportError:
            continue
    for pattern in ("luggage_msgs/lib/libluggage_msgs__rosidl_typesupport_c.so",
                    "lib/libluggage_msgs__rosidl_typesupport_c.so"):
        candidate = Path(install_root) / pattern
        if candidate.is_file():
            return type("_TypeSupport", (), {"__file__": str(candidate)})()
    reject("MISSING_PROCESS_PROVENANCE", "type_support", "luggage_msgs typesupport", pattern)


class NodeRuntime:
    """Production-node bundle: measured report, heartbeat, peers, readiness.

    One instance per process. Publishes this node's locally measured identity
    at 4 Hz on a steady clock, observes peer reports into a :class:
    ``ReadinessGate``, and admits requests / authorizes effects. v1 effect
    paths route through :meth:`authorize_legacy_effect`; nothing bypasses the
    gate. All rejections increment ``counters`` by contract code for evidence.
    """

    def __init__(self, node, *, peer, identity, resolved_config, peers,
                 modules=None, package_names=None, lease_seconds=2.0,
                 transitions_path=None):
        from ament_index_python.packages import get_package_prefix
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from luggage_description.runtime_graph import (
            endpoint_registry, identity_endpoints, live_providers)
        from luggage_description.runtime_manifest import collect_report
        from luggage_description.runtime_readiness import ReadinessGate

        self.node = node
        self.peer = str(peer).lstrip("/")
        self.identity = validate_identity(identity)
        self.peers = [str(name).lstrip("/") for name in peers]
        if self.peer not in self.peers:
            reject("INVALID_PEER_SET", "peer", self.peers, self.peer)
        self.registry = endpoint_registry(self.peers)
        if identity_endpoints(self.registry) != self.identity["endpoints"]:
            reject("PEER_IDENTITY_MISMATCH", "endpoints",
                   identity_endpoints(self.registry), self.identity["endpoints"])
        self.counters = {}
        self.transitions_path = transitions_path
        # Isolated colcon installs give every package its own prefix
        # directory; the application install root that contains all of them
        # (and that artifact paths are validated against) is its parent.
        from pathlib import Path
        install_root = str(Path(get_package_prefix("luggage_msgs")).parent)
        package_names = list(package_names or ("luggage_description",))
        prefixes = {}
        for name in dict.fromkeys((*package_names, "luggage_msgs")):
            prefixes[name] = get_package_prefix(name)
        loaded = list(modules or [])
        loaded.append((_generated_message_module(), "generated_message", "application"))
        loaded.append((_type_support_module(install_root), "type_support", "application"))
        self.report = collect_report(self.peer, self.identity, resolved_config,
                                     loaded, prefixes)
        # Readiness tracks the OTHER participants: this process is alive by
        # construction, and self-echo delivery must not be load-bearing.
        self.gate = ReadinessGate(
            self.identity,
            [peer for peer in self.peers if peer != self.peer],
            install_root, lease_seconds=lease_seconds)
        self._live_providers = live_providers
        import threading
        self._lock = threading.Lock()
        self._heartbeat = RuntimeHeartbeat(node, self.report, self.registry,
                                           self._readiness)
        self.subscription = node.create_subscription(
            RuntimeIdentityV2, TOPIC, self._on_report,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))

    # -- observation -----------------------------------------------------
    def _on_report(self, message):
        try:
            report = report_from_message(message, self.registry)
            if report["peer"] == self.peer:
                return
            with self._lock:
                self.gate.observe(report)
        except ContractError as error:
            self._count(error.code)
            self.node.get_logger().warning(
                "runtime identity report rejected (%s %s)" % (error.code, error.field))
        except Exception as error:  # noqa: BLE001 - a bad peer must not kill the node
            self._count("INVALID_PROCESS_REPORT")
            self.node.get_logger().warning(
                "runtime identity report failed to decode: %s" % (error,))

    def _count(self, code):
        self.counters[code] = self.counters.get(code, 0) + 1

    def _providers(self):
        # This process backs its own endpoints by construction.
        observed = set(self.gate.reports) | {self.peer}
        return self._live_providers(self.registry, observed)

    def _readiness(self):
        try:
            with self._lock:
                return self.gate.reason(self._providers())
        except ContractError as error:
            return error.code

    # -- admission -------------------------------------------------------
    def admit_request(self, request_id, endpoint=None, client=None):
        """Admit a request under the current graph; returns the admission token."""
        with self._lock:
            token = self.gate.admit(self._providers(), request_id,
                                    endpoint=endpoint, client=client)
        self._count("ADMITTED")
        return token

    def authorize_effect(self, admission, request_id):
        """Recheck readiness and the token immediately before an effect."""
        with self._lock:
            self.gate.require_admission(admission, self._providers(), request_id)
        self._count("EFFECT_AUTHORIZED")

    def authorize_legacy_effect(self):
        """v1 handlers cannot bypass profile admission: readiness still holds."""
        with self._lock:
            self.gate.require_ready(self._providers())
        self._count("LEGACY_EFFECT_AUTHORIZED")

    def require_graph_ready(self):
        """Client-side enforcement: a driver requests only against a live graph."""
        with self._lock:
            self.gate.require_ready(self._providers())
        self._count("GRAPH_READY_CHECK")

    def readiness(self):
        """Current readiness code without draining the transition log."""
        return self._readiness()

    def _write_transitions(self, transitions):
        if not self.transitions_path or not transitions:
            return
        try:
            import json as _json
            with open(self.transitions_path, "a", encoding="utf-8") as stream:
                for entry in transitions:
                    stream.write(_json.dumps(
                        {"peer": self.peer, **entry}, allow_nan=False) + "\n")
        except OSError:
            pass

    def snapshot(self):
        """Evidence view: readiness, observed peers, counters, transitions."""
        with self._lock:
            transitions = self.gate.transitions
            self.gate.transitions = []
        self._write_transitions(transitions)
        return {
            "peer": self.peer,
            "readiness": self._readiness(),
            "observed_peers": sorted(self.gate.reports),
            "counters": dict(self.counters),
            "transitions": transitions,
            "heartbeat_sequence": self._heartbeat.sequence,
        }


def node_runtime_from_params(node, *, resolved_config, modules=None,
                             package_names=None, transitions_path=None):
    """Build a NodeRuntime from the standard ``frame_*`` node parameters.

    Returns ``None`` when ``frame_runtime`` is false, leaving legacy behavior
    byte-identical. ``frame_identity_json`` carries the launch-measured
    identity; ``frame_peers`` the session peer set (csv).
    """
    if not node.has_parameter("frame_runtime") or not node.get_parameter(
            "frame_runtime").value:
        return None
    import json as _json
    from luggage_description.runtime_graph import endpoint_registry, identity_endpoints
    identity = _json.loads(str(node.get_parameter("frame_identity_json").value))
    peers = [part.strip() for part in str(
        node.get_parameter("frame_peers").value).split(",") if part.strip()]
    registry = endpoint_registry(peers)
    if identity_endpoints(registry) != identity.get("endpoints"):
        reject("PEER_IDENTITY_MISMATCH", "endpoints",
               identity_endpoints(registry), identity.get("endpoints"))
    return NodeRuntime(node, peer=node.get_name(), identity=identity,
                       resolved_config=resolved_config, peers=peers,
                       modules=modules, package_names=package_names,
                       transitions_path=transitions_path)
