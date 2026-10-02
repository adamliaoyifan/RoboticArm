"""Versioned identity manifest for the ST09-PD deploy assets.

ROS-free. The manifest is the machine-readable inventory of everything
the deploy cell freezes: scene file, seven-face container geometry,
expanded hardware URDF, SRDF, planning group / tool link / ordered
joints / tool transform, and one entry per payload atlas with its
directed route graph. Every hash is MEASURED from the bytes of the file
the entry names -- nothing is copied from a launch argument or from an
expected value a caller declared. ``validate_deploy_manifest`` repeats
those measurements on the consumer side, so a tampered asset is a named
identity rejection instead of a silent mismatch.

File references are package-anchored (``package`` + package-relative
POSIX ``path``) and resolved by a caller-supplied ``resolve`` callable,
so the same record validates from the source tree and from a fresh
install prefix without embedding a workspace layout.
"""

from __future__ import division

import hashlib
import math
import xml.etree.ElementTree as ElementTree

from .runtime_profile import canonical_hash, reject

#: Schema identity of this manifest format.
DEPLOY_MANIFEST_SCHEMA = "luggage.deploy_manifest/1"

#: Payload envelopes frozen by ST09-PD (length, width, height metres),
#: matching the atlas basenames built by the campaign.
DEPLOY_PAYLOAD_ENVELOPES = (
    (0.55, 0.40, 0.25),
    (0.70, 0.45, 0.28),
    (0.80, 0.50, 0.32),
)

#: Semantic block fixed by the v2 atlas contract (frame_contracts.md).
DEPLOY_SEMANTICS = {
    "angle_unit": "rad",
    "length_unit": "m",
    "height_datum": "container_origin",
    "orientation_semantics": "concrete_tool_pose",
    "tool_down_reference": "container_negative_z",
    "position_frame": "container_link",
    "orientation_frame": "container_link",
    "reference": "suction_contact_frame",
}

_HASH_LENGTHS = {"sha256": 64, "md5": 32}


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(4096), b""):
            digest.update(chunk)
    return digest.hexdigest()


def md5_file(path):
    digest = hashlib.md5()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(4096), b""):
            digest.update(chunk)
    return digest.hexdigest()


def semantic_srdf_text(srdf_text):
    """Final robot_description_semantic string the hardware stack loads.

    Byte-identical to ``hardware_pick._semantic_srdf``: the
    ``elfin_s20_with_camera`` robot name becomes MoveIt's ``S20``.
    """
    return srdf_text.replace('name="elfin_s20_with_camera"', 'name="S20"', 1)


def fixed_link_transform(urdf_text, source_link, target_link):
    """Constant SE(3) source->target transform with every joint at zero.

    The tool-identity transform (elfin_base_link -> suction_contact_frame
    at q = 0) is exactly this composition: parse the joint chain once and
    compose the joint origins child-first. Pure stdlib, no kinematics
    dependency.
    """
    root = ElementTree.fromstring(
        urdf_text.encode("utf-8") if isinstance(urdf_text, str) else urdf_text)
    joints = {}
    child_of = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        joints[child] = joint
        child_of[child] = parent
    chain = []
    link = target_link
    while link != source_link:
        if link not in joints:
            reject("INVALID_IDENTITY", "urdf_chain",
                   "%s reachable from %s" % (target_link, source_link), link)
        joint = joints[link]
        chain.append(joint)
        link = child_of[link]
        if len(chain) > len(joints) + 1:
            reject("INVALID_IDENTITY", "urdf_chain", "acyclic chain", chain)
    # p_source = R_acc @ p_frame + t_acc with the frame walking from the
    # target up to the source one joint at a time (child-first order):
    # each joint origin is the child link frame expressed in its parent.
    translation = (0.0, 0.0, 0.0)
    rotation = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    for joint in chain:
        origin = joint.find("origin")
        xyz = (0.0, 0.0, 0.0)
        rpy = (0.0, 0.0, 0.0)
        if origin is not None:
            if origin.get("xyz"):
                xyz = tuple(float(v) for v in origin.get("xyz").split())
            if origin.get("rpy"):
                rpy = tuple(float(v) for v in origin.get("rpy").split())
        joint_rotation = _rpy_matrix(rpy)
        rotation = _mat_mul(joint_rotation, rotation)
        rotated = _mat_vec(joint_rotation, translation)
        translation = tuple(rotated[i] + xyz[i] for i in range(3))
    return {
        "translation": list(translation),
        "quaternion_xyzw": list(_matrix_to_quat(rotation)),
    }


def _rpy_matrix(rpy):
    cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
    cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
    cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _mat_mul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3))


def _mat_vec(a, v):
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def _matrix_to_quat(m):
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quat = ((m[2][1] - m[1][2]) / scale, (m[0][2] - m[2][0]) / scale,
                (m[1][0] - m[0][1]) / scale, 0.25 * scale)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        scale = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        quat = (0.25 * scale, (m[0][1] + m[1][0]) / scale,
                (m[0][2] + m[2][0]) / scale, (m[2][1] - m[1][2]) / scale)
    elif m[1][1] > m[2][2]:
        scale = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        quat = ((m[0][1] + m[1][0]) / scale, 0.25 * scale,
                (m[1][2] + m[2][1]) / scale, (m[0][2] - m[2][0]) / scale)
    else:
        scale = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        quat = ((m[0][2] + m[2][0]) / scale, (m[1][2] + m[2][1]) / scale,
                0.25 * scale, (m[1][0] - m[0][1]) / scale)
    norm = math.sqrt(sum(v * v for v in quat))
    return tuple(v / norm for v in quat)


def file_entry_for(package, rel_path, abs_path):
    """One package-anchored file record with freshly measured sha256."""
    return {
        "package": str(package),
        "path": str(rel_path),
        "sha256": sha256_file(abs_path),
    }


def build_deploy_manifest(*, scene, urdf, srdf, atlas_entries, resolve,
                          provenance, payload_envelopes=None):
    """Assemble the manifest record, measuring every hash on the way in.

    ``scene``/``urdf``/``srdf`` are ``(abs_path, package_rel_path)``
    pairs; ``atlas_entries`` items are dicts with ``payload`` (None or
    [l, w, h]), ``basename``, ``npz_path``/``npz_rel``,
    ``meta_path``/``meta_rel`` and optionally ``graph``
    ({graph_id, path, rel, digest}). ``resolve(package, rel) -> abs
    path`` maps package-anchored references for validation.
    """
    from .runtime_identity import actuated_joint_names
    from .runtime_identity import scene_revision_from_path as scene_md5

    scene_path, scene_rel = scene
    urdf_path, urdf_rel = urdf
    srdf_path, srdf_rel = srdf
    with open(srdf_path, "r", encoding="utf-8") as handle:
        srdf_text = handle.read()
    semantic_text = semantic_srdf_text(srdf_text)
    with open(urdf_path, "rb") as handle:
        urdf_raw = handle.read()
    urdf_text = urdf_raw.decode("utf-8")
    tool_transform = fixed_link_transform(
        urdf_text, "elfin_base_link", "suction_contact_frame")

    from .container_geometry import descriptor_from_scene_config
    import yaml
    with open(scene_path, encoding="utf-8") as stream:
        scene_config = yaml.safe_load(stream) or {}
    geometry = descriptor_from_scene_config(scene_config)

    entries = []
    for entry in atlas_entries:
        record = {
            "payload": (None if entry.get("payload") is None
                        else [float(v) for v in entry["payload"]]),
            "basename": str(entry["basename"]),
            "npz": file_entry_for(
                "luggage_planning", entry["npz_rel"], entry["npz_path"]),
            "meta": file_entry_for(
                "luggage_planning", entry["meta_rel"], entry["meta_path"]),
        }
        if entry.get("graph"):
            graph = dict(entry["graph"])
            record["graph"] = {
                "graph_id": str(graph["graph_id"]),
                "package": "luggage_planning",
                "path": str(graph["rel"]),
                "sha256": sha256_file(graph["path"]),
                "digest": str(graph.get("digest") or ""),
            }
        entries.append(record)

    manifest = {
        "schema": DEPLOY_MANIFEST_SCHEMA,
        "generated": dict(provenance),
        "scene": dict(
            file_entry_for("luggage_description", scene_rel, scene_path),
            scene_revision_md5=scene_md5(scene_path)),
        "geometry": geometry.descriptor(include_hash=True),
        "urdf": dict(
            file_entry_for("luggage_planning", urdf_rel, urdf_path),
            # md5 is the atlas-contract identity; sha256 the runtime one.
            md5=hashlib.md5(urdf_raw).hexdigest(),
            joint_names=actuated_joint_names(urdf_text)),
        "srdf": dict(
            file_entry_for("luggage_description", srdf_rel, srdf_path),
            semantic_sha256=hashlib.sha256(
                semantic_text.encode("utf-8")).hexdigest()),
        "planning": {
            "group": "elfin_arm",
            "tool_link": "suction_contact_frame",
            "base_link": "elfin_base_link",
            "tool_transform_identity": tool_transform,
        },
        "semantics": dict(DEPLOY_SEMANTICS),
        "payload_envelopes": [
            [float(v) for v in size]
            for size in (DEPLOY_PAYLOAD_ENVELOPES
                         if payload_envelopes is None
                         else payload_envelopes)],
        "atlas": entries,
    }
    return validate_deploy_manifest(manifest, resolve)


def validate_deploy_manifest(manifest, resolve):
    """Re-measure every recorded hash through ``resolve``.

    Returns a copy of the manifest on success and raises
    ``ContractError`` with a named code on any drift. The check is
    deliberately structural-plus-byte-level: it does not load the atlas
    (that is the strict atlas loader's job) but it does pin every file
    the deploy cell freezes to the exact bytes recorded at freeze time.
    """
    if not isinstance(manifest, dict):
        reject("INVALID_MANIFEST", "manifest", "mapping", manifest)
    if manifest.get("schema") != DEPLOY_MANIFEST_SCHEMA:
        reject("MANIFEST_SCHEMA_MISMATCH", "schema",
               DEPLOY_MANIFEST_SCHEMA, manifest.get("schema"))
    for section in ("generated", "scene", "geometry", "urdf", "srdf",
                    "planning", "semantics"):
        if not isinstance(manifest.get(section), dict):
            reject("MISSING_MANIFEST_SECTION", section, "mapping",
                   manifest.get(section))
    if not isinstance(manifest.get("atlas"), list):
        reject("MISSING_MANIFEST_SECTION", "atlas", "entry list",
               manifest.get("atlas"))
    if not isinstance(manifest.get("payload_envelopes"), list):
        reject("MISSING_MANIFEST_SECTION", "payload_envelopes", "list",
               manifest.get("payload_envelopes"))

    _check_file(manifest["scene"], resolve, "scene")
    _check_file(manifest["urdf"], resolve, "urdf", extra_hashes=("md5",))
    _check_file(manifest["srdf"], resolve, "srdf")

    joints = manifest["urdf"].get("joint_names")
    if (not isinstance(joints, list) or len(joints) != 6
            or any(not isinstance(v, str) or not v.strip() for v in joints)
            or len(set(joints)) != len(joints)):
        reject("JOINT_ORDER_MISMATCH", "urdf.joint_names",
               "six unique ordered names", joints)
    planning = manifest["planning"]
    if planning.get("group") != "elfin_arm":
        reject("INVALID_MANIFEST", "planning.group", "elfin_arm",
               planning.get("group"))
    if planning.get("tool_link") != "suction_contact_frame":
        reject("INVALID_MANIFEST", "planning.tool_link",
               "suction_contact_frame", planning.get("tool_link"))

    expected_entries = 1 + len(manifest["payload_envelopes"])
    if len(manifest["atlas"]) != expected_entries:
        reject("ATLAS_ENTRY_COUNT_MISMATCH", "atlas",
               "%d entries (empty + %d payloads)" % (
                   expected_entries, len(manifest["payload_envelopes"])),
               len(manifest["atlas"]))
    for entry in manifest["atlas"]:
        label = "atlas[%s]" % (entry.get("basename") or "?")
        if not isinstance(entry, dict) or not entry.get("basename"):
            reject("INVALID_MANIFEST", label, "basename entry", entry)
        _check_file(entry.get("npz") or {}, resolve, label + ".npz")
        _check_file(entry.get("meta") or {}, resolve, label + ".meta")
        graph = entry.get("graph")
        if not isinstance(graph, dict) or not graph.get("graph_id"):
            reject("MISSING_MANIFEST_SECTION", label + ".graph",
                   "graph_id/path/sha256 mapping", graph)
        _check_file(graph, resolve, label + ".graph")
        if not str(graph.get("digest") or ""):
            reject("MISSING_MANIFEST_SECTION", label + ".graph.digest",
                   "nonempty graph digest", graph.get("digest"))

    semantics = manifest["semantics"]
    for key, value in DEPLOY_SEMANTICS.items():
        if semantics.get(key) != value:
            reject("MANIFEST_SEMANTICS_MISMATCH", "semantics." + key,
                   value, semantics.get(key))
    return dict(manifest)


def _check_file(section, resolve, label, extra_hashes=()):
    if not isinstance(section, dict):
        reject("MISSING_MANIFEST_SECTION", label, "path mapping", section)
    package = section.get("package")
    rel = section.get("path")
    if not package or not rel or not isinstance(rel, str):
        reject("MISSING_MANIFEST_SECTION", label + ".path",
               "package + relative path", section)
    try:
        path = resolve(str(package), str(rel))
    except Exception as error:  # noqa: BLE001 - resolver reports missing roots
        reject("MANIFEST_FILE_MISSING", label + ".path", rel, str(error))
    if not path or not isinstance(path, str) or not _is_file(path):
        reject("MANIFEST_FILE_MISSING", label + ".path", rel, path)
    fields = ("sha256",) + tuple(extra_hashes)
    for field in fields:
        recorded = section.get(field)
        if (not isinstance(recorded, str)
                or len(recorded) != _HASH_LENGTHS[field]
                or any(c not in "0123456789abcdef" for c in recorded)):
            reject("INVALID_MANIFEST_HASH", label + "." + field,
                   "%s hex" % field, recorded)
        actual = (md5_file(path) if field == "md5" else sha256_file(path))
        if actual != recorded:
            reject("HASH_MISMATCH", label + "." + field, recorded, actual)


def _is_file(path):
    import os
    return os.path.isfile(path)


def manifest_identity(manifest):
    """Stable content identity of a validated manifest record."""
    return canonical_hash(manifest)
