"""Versioned runtime intent and measured local provenance, without ROS imports.

Manifest expectations are build inputs. Reports must be collected from loaded
modules and resolved configuration, never manufactured from those expectations.
"""
from copy import deepcopy
from pathlib import Path
import uuid

from .runtime_profile import canonical_hash, reject, sha256_file, validate_identity

SCHEMA = "luggage.runtime_manifest/2"
HEARTBEAT_PERIOD_SECONDS = 0.25
REPORT_IDENTITY_FIELDS = (
    "peer", "incarnation", "identity", "contract_digest", "resolved_config",
    "artifacts", "package_prefixes",
)

# Standard interfaces retain their standard types. Application boundaries are
# explicitly enumerated; a V2 suffix alone is not evidence of migration.
APPLICATION_TYPES = frozenset(
    "luggage_msgs/" + kind + "/" + name
    for kind, names in (
        ("srv", ("ComputePlacementV2", "SelectPlacePathV2", "BuildMotionSequenceV2",
                 "ProbeMotionSegmentV2", "AddSceneBoxV2", "RemoveSceneBoxV2",
                 "VacuumCommandV2", "AddPlacedBoxV2", "RemovePlacedBoxV2",
                 "SyncStaticSceneV2", "SetPlaceSupportTouchV2", "ResetCargoMapV2",
                 "IntegrateCargoViewV2", "ValidatePoseV2")),
        ("action", ("PlanMotionV2", "GoToRobotPoseV2")),
        ("msg", ("SurfaceContractV2", "RuntimeIdentityV2")),
    ) for name in names
)
STANDARD_TYPES = frozenset((
    "tf2_msgs/msg/TFMessage", "sensor_msgs/msg/JointState",
    "sensor_msgs/msg/Image", "sensor_msgs/msg/CameraInfo",
    "sensor_msgs/msg/PointCloud2", "rosgraph_msgs/msg/Clock",
    "moveit_msgs/srv/GetPositionFK", "moveit_msgs/srv/GetPositionIK",
    "moveit_msgs/srv/GetPlanningScene", "moveit_msgs/srv/ApplyPlanningScene",
    "moveit_msgs/action/MoveGroup", "moveit_msgs/action/ExecuteTrajectory",
    "control_msgs/action/FollowJointTrajectory",
    "controller_manager_msgs/srv/ListControllers",
))


def validate_endpoint_type(name, type_name):
    if (not isinstance(name, str) or not name.startswith("/")
            or not isinstance(type_name, str)
            or type_name not in APPLICATION_TYPES | STANDARD_TYPES):
        reject("ENDPOINT_TYPE_MISMATCH", str(name), "registered exact interface type", type_name)


def identity_fingerprint(report):
    """Readiness and heartbeat clocks are mutable, not process identity."""
    return canonical_hash({key: report.get(key) for key in REPORT_IDENTITY_FIELDS})


def validate_manifest(manifest):
    if not isinstance(manifest, dict) or manifest.get("schema") != SCHEMA:
        reject("UNKNOWN_RUNTIME_SCHEMA", "schema", SCHEMA,
               manifest.get("schema") if isinstance(manifest, dict) else None)
    required = {"schema", "identity", "endpoints", "processes", "install_root", "system_root"}
    if set(manifest) != required:
        reject("RUNTIME_FIELD_MISMATCH", "fields", sorted(required), sorted(manifest))
    identity = validate_identity(manifest["identity"])
    for root in ("install_root", "system_root"):
        if not isinstance(manifest[root], str) or not Path(manifest[root]).is_absolute():
            reject("WRONG_OVERLAY", root, "absolute root", manifest[root])
    processes = manifest["processes"]
    if not isinstance(processes, dict) or not processes:
        reject("MISSING_PROCESS_PROVENANCE", "processes", "nonempty mapping", processes)
    endpoints = manifest["endpoints"]
    if not isinstance(endpoints, dict) or set(endpoints) != set(identity["endpoints"]):
        reject("ENDPOINT_TYPE_MISMATCH", "endpoints", identity["endpoints"], endpoints)
    for name, endpoint in endpoints.items():
        if not isinstance(endpoint, dict) or set(endpoint) != {"type", "provider", "clients"}:
            reject("INVALID_ENDPOINT_REGISTRY", name, "type/provider/clients", endpoint)
        if endpoint["type"] != identity["endpoints"][name]:
            reject("ENDPOINT_TYPE_MISMATCH", name, identity["endpoints"][name], endpoint["type"])
        clients = endpoint["clients"]
        providers = endpoint["provider"]
        providers = [providers] if isinstance(providers, str) else providers
        if (not isinstance(providers, list) or not providers
                or any(not isinstance(provider, str) or provider not in processes for provider in providers)
                or len(set(providers)) != len(providers) or not isinstance(clients, list)
                or not clients or any(not isinstance(client, str) or client not in processes for client in clients)
                or len(set(clients)) != len(clients)):
            reject("INVALID_ENDPOINT_OWNERSHIP", name, "known provider and unique clients", endpoint)
    for peer, expected in processes.items():
        if (not isinstance(peer, str) or not peer.strip() or not isinstance(expected, dict)
                or set(expected) != {"resolved_config", "artifacts", "package_prefixes"}
                or not all(expected.values())
                or not isinstance(expected["resolved_config"], dict)):
            reject("MISSING_PROCESS_PROVENANCE", peer, "config/artifacts/prefixes", expected)
        validate_artifact_locations(expected, manifest["install_root"], manifest["system_root"])
    result = deepcopy(manifest)
    for expected in result["processes"].values():
        expected["package_prefixes"] = canonical_prefixes(expected["package_prefixes"])
    return result


def canonical_prefixes(prefixes):
    return {name: (dict(origin="application", path=entry) if isinstance(entry, str)
                   else deepcopy(entry)) for name, entry in prefixes.items()}


def validate_artifact_locations(report, install_root, system_root=None):
    """System paths are allowed only for explicitly labelled pinned dependencies."""
    roots = {"application": Path(install_root).resolve()}
    if system_root:
        roots["system"] = Path(system_root).resolve()
    kinds = set()
    artifacts = report.get("artifacts")
    if not isinstance(artifacts, list) or not artifacts:
        reject("MISSING_PROCESS_PROVENANCE", "artifacts", "nonempty list", artifacts)
    paths = set()
    for artifact in artifacts:
        if not isinstance(artifact, dict) or not isinstance(artifact.get("path"), str):
            reject("INVALID_PROCESS_REPORT", "artifact", "mapping with path", artifact)
        path = Path(artifact["path"])
        root = roots.get(artifact.get("origin", "application"))
        if not path.is_absolute() or root is None or not path.resolve().is_relative_to(root):
            reject("WRONG_OVERLAY", "path", str(root), str(path))
        if str(path.resolve()) in paths:
            reject("INVALID_PROCESS_REPORT", "artifacts", "unique paths", str(path))
        paths.add(str(path.resolve()))
        digest = artifact.get("sha256")
        if (not isinstance(digest, str) or len(digest) != 64
                or any(c not in "0123456789abcdef" for c in digest)):
            reject("MISSING_PROCESS_PROVENANCE", "sha256", "SHA256", digest)
        kinds.add(artifact.get("kind"))
    if not {"module", "generated_message", "type_support"}.issubset(kinds):
        reject("MISSING_PROCESS_PROVENANCE", "artifact kinds", "module/message/type_support", sorted(str(kind) for kind in kinds))
    prefixes = report.get("package_prefixes")
    if not isinstance(prefixes, dict) or not prefixes:
        reject("MISSING_PROCESS_PROVENANCE", "package_prefixes", "nonempty mapping", prefixes)
    for package, entry in prefixes.items():
        # String prefixes are application-only for the existing primitive API.
        origin, prefix = (entry.get("origin"), entry.get("path")) if isinstance(entry, dict) else ("application", entry)
        root = roots.get(origin)
        if (not isinstance(prefix, str) or not Path(prefix).is_absolute()
                or root is None or not Path(prefix).resolve().is_relative_to(root)):
            reject("WRONG_OVERLAY", package, str(root), prefix)


def collect_report(peer, identity, resolved_config, modules, package_prefixes, *, incarnation=None):
    """Hash actual imported module files (module, kind, origin triples).

    The node supplies its own resolved settings and ament-resolved prefixes.
    No manifest artifact path or expected digest is accepted by this function.
    Reuse the returned report for heartbeats, keeping the incarnation stable.
    """
    artifacts = []
    for module, kind, origin in modules:
        path = getattr(module, "__file__", None)
        if not path:
            reject("MISSING_PROCESS_PROVENANCE", "module.__file__", "loaded file", path)
        path = str(Path(path).resolve(strict=True))
        artifacts.append(dict(kind=kind, origin=origin, path=path, sha256=sha256_file(path)))
    local_identity = validate_identity(identity)
    return dict(peer=peer, incarnation=incarnation or str(uuid.uuid4()),
                identity=local_identity, contract_digest=canonical_hash(local_identity),
                resolved_config=deepcopy(resolved_config), artifacts=artifacts,
                package_prefixes=canonical_prefixes(package_prefixes))
