"""Measured runtime identity assembly for the v2 production chain.

Identity inputs are settings the deployment resolves (scene file, calibration
label, tool link, atlas selection); every hash in the identity is MEASURED
from the resolved bytes at launch time. Nothing here trusts a declared hash.
ROS-free so launches, nodes and offline tools share one implementation.
"""
import hashlib
import xml.etree.ElementTree as ElementTree

from .runtime_profile import reject, sha256_file, validate_identity

#: The atlas builder and its runtime cells identify the scene by the MD5 of
#: the scene_tf file bytes (``atlas_runtime.compute_scene_tf_hash``). The v2
#: chain reuses that value as ``scene_revision`` so the surface contract, the
#: atlas contract block and the runtime identity name one scene identically.
def scene_revision_from_path(path):
    if not path:
        reject("MISSING_IDENTITY", "scene_tf_config", "scene file", path)
    digest = hashlib.md5()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(4096), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_text(text):
    raw = text.encode("utf-8") if isinstance(text, str) else bytes(text)
    if not raw:
        reject("MISSING_IDENTITY", "robot_description", "nonempty text", "")
    return hashlib.sha256(raw).hexdigest()


def actuated_joint_names(urdf_text):
    """Ordered actuated joint names from the expanded URDF document.

    Document order matches the xacro ``initial_jointN`` expansion the
    controllers and the atlas builder were built against.
    """
    try:
        root = ElementTree.fromstring(
            urdf_text.encode("utf-8") if isinstance(urdf_text, str) else urdf_text)
    except ElementTree.ParseError as error:
        reject("INVALID_IDENTITY", "urdf", "parseable XML", str(error))
    joints = [joint.get("name") for joint in root.findall("joint")
              if joint.get("type") != "fixed" and joint.get("name")]
    if not joints:
        reject("MISSING_IDENTITY", "joint_names", "actuated joints", joints)
    return joints


def atlas_identity(atlas_files):
    """(atlas_sha256, certification_sha256) from resolved (npz, meta) paths.

    The certified artifact pair is the certification record: hashing both
    files binds the identity to the exact bytes a placement query reads.
    """
    if not atlas_files or len(atlas_files) != 2:
        reject("MISSING_IDENTITY", "atlas_files", "(npz, meta) pair", atlas_files)
    npz_path, meta_path = atlas_files
    return sha256_file(npz_path), sha256_file(meta_path)


def build_identity(*, scene_tf_config, calibration_revision, urdf_text,
                   srdf_text, tool_link, atlas_files, endpoints,
                   geometry_hash=None):
    """Assemble and validate the shared runtime identity.

    ``geometry_hash`` may be supplied by a caller that already normalized the
    container descriptor; otherwise it is derived from the scene config.
    """
    if geometry_hash is None:
        from .container_geometry import descriptor_from_scene_config
        import yaml
        with open(scene_tf_config, encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}
        geometry_hash = descriptor_from_scene_config(config).geometry_hash
    npz_sha, meta_sha = atlas_identity(atlas_files)
    identity = {
        "contract_version": 2,
        "scene_revision": scene_revision_from_path(scene_tf_config),
        "calibration_revision": str(calibration_revision),
        "urdf_sha256": sha256_text(urdf_text),
        "srdf_sha256": sha256_text(srdf_text),
        "tool_link": str(tool_link),
        "joint_names": actuated_joint_names(urdf_text),
        "geometry_hash": str(geometry_hash),
        "atlas_sha256": npz_sha,
        "atlas_certification_sha256": meta_sha,
        "endpoints": dict(endpoints),
    }
    return validate_identity(identity)
