"""Strict shared launch resolution and immutable v2 process contracts.

No ROS dependency. A resolved profile is intent, not proof of running peers.
Consumer admission is implemented separately in runtime_readiness.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

from .frame_contracts import ContractError

PROFILE_SENTINEL = "__profile__"
IDENTITY_FIELDS = (
    "scene_revision", "calibration_revision", "urdf_sha256", "srdf_sha256",
    "tool_link", "joint_names", "geometry_hash", "atlas_sha256",
    "atlas_certification_sha256", "endpoints",
)


def reject(code, field, expected, received):
    raise ContractError(code, field, expected, received)


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def canonical_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def flatten_profile(data):
    """Groups organize YAML only; duplicate leaf names are always ambiguous."""
    if not isinstance(data, dict):
        reject("INVALID_PROFILE", "profile", "mapping", data)
    result = {}

    def visit(group):
        for key, value in group.items():
            if not isinstance(key, str) or not key:
                reject("INVALID_PROFILE", "key", "nonempty string", key)
            if isinstance(value, dict):
                visit(value)
            elif key in result:
                reject("DUPLICATE_PROFILE_KEY", key, "one declaration", value)
            else:
                result[key] = value
    visit(data)
    return result


def load_profile(path):
    if not path:
        return {}
    import yaml
    with open(path, encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    return flatten_profile({} if value is None else value)


def resolve_profile(defaults, profile, overrides):
    """CLI > profile > defaults. Reject unknowns even when CLI masks them."""
    for source, values in (("profile", profile), ("CLI", overrides)):
        unknown = sorted(set(values) - set(defaults))
        if unknown:
            reject("UNKNOWN_PROFILE_SETTING", source, sorted(defaults), unknown)
    resolved = deepcopy(defaults)
    resolved.update(deepcopy(profile))
    resolved.update({key: deepcopy(value) for key, value in overrides.items()
                     if value != PROFILE_SENTINEL})
    return resolved


def launch_text(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, tuple)):
        return ",".join(str(item) for item in value)
    return str(value)


def validate_identity(identity):
    if not isinstance(identity, dict):
        reject("MISSING_IDENTITY", "identity", "mapping", identity)
    if type(identity.get("contract_version")) is not int or identity["contract_version"] != 2:
        reject("CONTRACT_VERSION_MISMATCH", "contract_version", 2, identity.get("contract_version"))
    for key in IDENTITY_FIELDS:
        value = identity.get(key)
        if not value or isinstance(value, str) and not value.strip():
            reject("MISSING_IDENTITY", key, "nonempty identity", value)
        if key not in ("joint_names", "endpoints") and not isinstance(value, str):
            reject("INVALID_IDENTITY", key, "string", value)
        if key.endswith("sha256") and (not isinstance(value, str) or len(value) != 64
                or any(c not in "0123456789abcdef" for c in value)):
            reject("INVALID_IDENTITY_HASH", key, "SHA256 hex", value)
    joints = identity["joint_names"]
    if (not isinstance(joints, list) or any(not isinstance(j, str) or not j.strip() for j in joints)
            or len(set(joints)) != len(joints)):
        reject("JOINT_ORDER_MISMATCH", "joint_names", "unique ordered names", joints)
    endpoints = identity["endpoints"]
    if not isinstance(endpoints, dict):
        reject("ENDPOINT_TYPE_MISMATCH", "endpoints", "endpoint/type mapping", endpoints)
    from .runtime_manifest import validate_endpoint_type
    for name, message_type in endpoints.items():
        validate_endpoint_type(name, message_type)
    return deepcopy(identity)
