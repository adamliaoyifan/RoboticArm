"""Strict deploy-profile resolution for the hardware cell (ST09-PD).

The hardware launch must never obtain a simulation scene, simulation
atlas, or simulation pose file through an empty or default argument.
This module resolves, fail closed, the installed deploy scene
(``scene_tf_deploy.yaml``), the deploy atlas runtime cell
(``atlas_runtime_deploy.yaml``), the empty-load atlas pair it selects,
and the deploy asset manifest -- then re-measures every hash the
manifest records. Any missing, ambiguous, sim-named, or hash-mismatched
asset is a named ``ContractError`` rejection, never a silent fallback
to the simulation cell.

ROS-free. Package roots resolve an injected share map first
(fresh-install tests), then the source tree, then the ament install
prefix. When both an injected share and the source tree hold a
different copy of the same named asset, the profile is ambiguous and
rejected; a symlink install resolves both spellings to one inode and
stays unambiguous.
"""

from __future__ import division

import os

from luggage_planning.atlas_runtime import (
    compute_scene_tf_hash,
    compute_urdf_hash,
    identity_verdict,
    load_atlas_runtime,
    resolve_atlas_files,
)

#: Scene basenames that mark simulation assets. A hardware profile that
#: resolves to any of these -- by default or by explicit argument -- is
#: a configuration error, not a deploy cell.
SIM_SCENE_BASENAMES = frozenset((
    "scene_tf.yaml",
    "scene_tf.yaml.example",
))

DEPLOY_SCENE_BASENAME = "scene_tf_deploy.yaml"
DEPLOY_ATLAS_CONFIG_BASENAME = "atlas_runtime_deploy.yaml"
DEPLOY_MANIFEST_BASENAME = "deploy_asset_manifest.yaml"
DEPLOY_ATLAS_DIR_NAME = "reachability_atlas_deploy"

DESCRIPTION_PACKAGE = "luggage_description"
PLANNING_PACKAGE = "luggage_planning"

_SRC_ROOTS = {
    DESCRIPTION_PACKAGE: os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..",
        "luggage_description")),
    PLANNING_PACKAGE: os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..")),
}


def default_resolver(shares=None):
    """Build ``resolve(package, rel_path) -> abs path or None``.

    ``shares`` maps package name -> package root (the directory holding
    ``config``/``data``). A package present there is authoritative and
    unambiguous by declaration (fresh-install testing); otherwise the
    source tree wins and the ament prefix is the fallback.
    """

    def resolve(package, rel_path):
        rel_path = str(rel_path).replace("/", os.sep)
        injected = (shares or {}).get(package)
        if injected:
            return os.path.join(str(injected), rel_path)
        src_root = _SRC_ROOTS.get(package)
        if src_root:
            candidate = os.path.join(src_root, rel_path)
            if os.path.exists(candidate):
                return candidate
        try:
            from ament_index_python.packages import (
                get_package_share_directory)
            return os.path.join(
                get_package_share_directory(package), rel_path)
        except Exception:  # noqa: BLE001 - not installed / no ament
            return None

    resolve.shares = dict(shares or {})
    return resolve


def reject_asset(code, field, expected, received):
    from luggage_description.frame_contracts import ContractError
    raise ContractError(code, field, expected, received)


def _read_bytes(path):
    with open(path, "rb") as handle:
        return handle.read()


def _resolve_unique(resolver, package, rel_path, label):
    """Resolve one package file, rejecting absence and ambiguity.

    Production resolution (no injected share) checks source tree against
    the ament prefix: two different bytes claiming one deploy asset name
    is a drifted install, not a cell. An injected share is authoritative
    by declaration and is never cross-checked.
    """
    primary = resolver(package, rel_path)
    if not primary or not os.path.isfile(primary):
        reject_asset("DEPLOY_ASSET_MISSING", label, rel_path, primary)
    injected = (getattr(resolver, "shares", None) or {}).get(package)
    if not injected:
        alternatives = []
        src_root = _SRC_ROOTS.get(package)
        if src_root:
            candidate = os.path.join(
                src_root, str(rel_path).replace("/", os.sep))
            if os.path.isfile(candidate):
                alternatives.append(candidate)
        try:
            from ament_index_python.packages import (
                get_package_share_directory)
            candidate = os.path.join(
                get_package_share_directory(package),
                str(rel_path).replace("/", os.sep))
            if os.path.isfile(candidate):
                alternatives.append(candidate)
        except Exception:  # noqa: BLE001 - not installed / no ament
            pass
        reject_ambiguous(primary, alternatives, label)
    return os.path.abspath(primary)


def reject_ambiguous(primary, alternatives, label):
    """Raise when another root holds different bytes under the same name."""
    for alternative in alternatives:
        if os.path.realpath(alternative) == os.path.realpath(primary):
            continue
        if _read_bytes(alternative) != _read_bytes(primary):
            reject_asset(
                "AMBIGUOUS_DEPLOY_ASSET", label,
                "one file (source tree and install prefix differ)",
                sorted([primary, alternative]))


def resolve_deploy_profile(scene_arg="", atlas_config_arg="",
                           manifest_arg="", *, resolver=None,
                           check_atlas=True, check_manifest=True):
    """Resolve the deploy profile; return a record or raise ContractError.

    ``scene_arg`` / ``atlas_config_arg`` / ``manifest_arg`` are the
    launch argument values: empty means the named deploy default, a
    nonempty value is an explicit override that must still be a deploy
    asset (a sim-named scene or atlas config is rejected on the
    hardware path).

    ``check_atlas`` gates the atlas-pair/URDF/manifest identity checks:
    a detect-only bringup (no planning) still gets strict scene and
    config resolution but is not refused for missing generated assets.
    """
    from luggage_description.deploy_manifest import validate_deploy_manifest

    resolver = resolver or default_resolver()

    scene = _resolve_scene(scene_arg, resolver)
    atlas_config = _resolve_atlas_config(atlas_config_arg, resolver)
    record = {
        "scene_tf_path": scene,
        "atlas_runtime_config": atlas_config,
        "scene_tf_hash": compute_scene_tf_hash(scene),
    }
    if not check_atlas:
        return record
    cfg = load_atlas_runtime(atlas_config)
    if str(cfg.get("hash_policy") or "") != "strict":
        reject_asset(
            "DEPLOY_ATLAS_NOT_STRICT", "atlas_runtime_deploy.hash_policy",
            "strict (the deploy cell never runs warn)", cfg.get("hash_policy"))
    atlas_dir = _deploy_atlas_dir(cfg, resolver)
    files = resolve_atlas_files(cfg, None, atlas_dir=atlas_dir)
    if files is None:
        reject_asset(
            "ATLAS_FILES_NOT_FOUND", "atlas_runtime_deploy.atlas_dir",
            "%s/(%s.npz|yaml) under the planning package data tree"
            % (str(cfg.get("atlas_dir") or "?"),
               cfg.get("basename") or "s20_container_collision_aware"),
            atlas_dir)
    npz_path, meta_path = files
    meta = _load_yaml(meta_path)

    urdf_path = os.path.join(atlas_dir, "robot_description.xml")
    if not os.path.isfile(urdf_path):
        reject_asset("DEPLOY_URDF_MISSING", "robot_description.xml",
                     "expanded deploy URDF captured at freeze time",
                     urdf_path)
    ok, message, _scene_ok, _urdf_ok = identity_verdict(
        meta.get("dependencies") or {},
        scene_tf_hash=compute_scene_tf_hash(scene),
        urdf_hash=compute_urdf_hash(_read_bytes(urdf_path)),
        policy="strict")
    if not ok:
        reject_asset("ATLAS_IDENTITY_MISMATCH",
                     "atlas meta dependencies", "ok", message)

    record = {
        "scene_tf_path": scene,
        "atlas_runtime_config": atlas_config,
        "atlas_npz": os.path.abspath(npz_path),
        "atlas_meta": os.path.abspath(meta_path),
        "urdf_path": os.path.abspath(urdf_path),
        "scene_tf_hash": compute_scene_tf_hash(scene),
        "urdf_hash": compute_urdf_hash(_read_bytes(urdf_path)),
    }
    if not check_manifest:
        return record

    manifest_path = _resolve_manifest(manifest_arg, resolver)
    manifest = _load_yaml(manifest_path)
    validate_deploy_manifest(manifest, resolver)
    # The manifest must freeze exactly the cell being launched. An
    # explicit override at a different path is legal only when its bytes
    # are the frozen identity (same file copied elsewhere).
    manifest_scene = _manifest_file(manifest, "scene", resolver)
    if _read_bytes(manifest_scene) != _read_bytes(scene):
        reject_asset(
            "MANIFEST_SCENE_MISMATCH", "manifest.scene.path",
            os.path.abspath(scene), manifest_scene)
    manifest_urdf = _manifest_file(manifest, "urdf", resolver)
    if _read_bytes(manifest_urdf) != _read_bytes(urdf_path):
        reject_asset(
            "MANIFEST_URDF_MISMATCH", "manifest.urdf.path",
            os.path.abspath(urdf_path), manifest_urdf)
    record["manifest_path"] = os.path.abspath(manifest_path)
    record["manifest"] = manifest
    return record


def _resolve_scene(scene_arg, resolver):
    text = str(scene_arg or "").strip()
    if text:
        path = os.path.abspath(os.path.expanduser(text))
        if not os.path.isfile(path):
            reject_asset("DEPLOY_SCENE_MISSING", "scene_tf_config",
                         "existing deploy scene file", path)
    else:
        path = _resolve_unique(
            resolver, DESCRIPTION_PACKAGE,
            os.path.join("config", DEPLOY_SCENE_BASENAME),
            "scene_tf_config")
    if os.path.basename(path) in SIM_SCENE_BASENAMES:
        reject_asset(
            "SIM_SCENE_ON_HARDWARE", "scene_tf_config",
            "a deploy scene (%s), never %s" % (
                DEPLOY_SCENE_BASENAME, sorted(SIM_SCENE_BASENAMES)),
            path)
    return path


def _resolve_atlas_config(atlas_config_arg, resolver):
    text = str(atlas_config_arg or "").strip()
    if text:
        path = os.path.abspath(os.path.expanduser(text))
        if not os.path.isfile(path):
            reject_asset("DEPLOY_ATLAS_CONFIG_MISSING",
                         "atlas_runtime_config",
                         "existing deploy atlas runtime yaml", path)
        if os.path.basename(path) == "atlas_runtime.yaml":
            reject_asset(
                "SIM_ATLAS_ON_HARDWARE", "atlas_runtime_config",
                "atlas_runtime_deploy.yaml, never the simulation "
                "atlas_runtime.yaml", path)
        return path
    return _resolve_unique(
        resolver, PLANNING_PACKAGE,
        os.path.join("config", DEPLOY_ATLAS_CONFIG_BASENAME),
        "atlas_runtime_config")


def _resolve_manifest(manifest_arg, resolver):
    text = str(manifest_arg or "").strip()
    if text:
        path = os.path.abspath(os.path.expanduser(text))
        if not os.path.isfile(path):
            reject_asset("DEPLOY_MANIFEST_MISSING", "deploy_asset_manifest",
                         "existing deploy manifest yaml", path)
        return path
    return _resolve_unique(
        resolver, DESCRIPTION_PACKAGE,
        os.path.join("config", DEPLOY_MANIFEST_BASENAME),
        "deploy_asset_manifest")


def _deploy_atlas_dir(cfg, resolver):
    """Absolute atlas dir for the deploy cell under the planning package."""
    rel = str(cfg.get("atlas_dir") or "").strip()
    if not rel:
        reject_asset(
            "DEPLOY_ATLAS_DIR_EMPTY", "atlas_runtime_deploy.atlas_dir",
            DEPLOY_ATLAS_DIR_NAME, repr(cfg.get("atlas_dir")))
    if os.path.isabs(rel):
        reject_asset(
            "DEPLOY_ATLAS_DIR_ABSOLUTE", "atlas_runtime_deploy.atlas_dir",
            "package data-relative directory name", rel)
    if DEPLOY_ATLAS_DIR_NAME not in rel:
        reject_asset(
            "SIM_ATLAS_ON_HARDWARE", "atlas_runtime_deploy.atlas_dir",
            "a directory containing %s" % DEPLOY_ATLAS_DIR_NAME, rel)
    rel_path = os.path.join("data", rel)
    primary = resolver(PLANNING_PACKAGE, rel_path)
    if not primary or not os.path.isdir(primary):
        reject_asset("DEPLOY_ASSET_MISSING", "atlas_runtime_deploy.atlas_dir",
                     rel_path, primary)
    if not (getattr(resolver, "shares", None) or {}).get(PLANNING_PACKAGE):
        alternatives = []
        src_root = _SRC_ROOTS.get(PLANNING_PACKAGE)
        if src_root and os.path.isdir(os.path.join(src_root, rel_path)):
            alternatives.append(os.path.join(src_root, rel_path))
        try:
            from ament_index_python.packages import (
                get_package_share_directory)
            candidate = os.path.join(
                get_package_share_directory(PLANNING_PACKAGE),
                rel_path.replace("/", os.sep))
            if os.path.isdir(candidate):
                alternatives.append(candidate)
        except Exception:  # noqa: BLE001 - not installed / no ament
            pass
        # Directory drift is judged by the marker files inside it (the
        # atlas pair bytes), not by directory mtimes.
        primary_npz = os.path.join(
            primary, "%s.npz" % (cfg.get("basename")
                                 or "s20_container_collision_aware"))
        for alternative in alternatives:
            if os.path.realpath(alternative) == os.path.realpath(primary):
                continue
            alternative_npz = os.path.join(
                alternative, "%s.npz" % (cfg.get("basename")
                                         or "s20_container_collision_aware"))
            if not os.path.isfile(alternative_npz):
                continue
            if not os.path.isfile(primary_npz):
                continue
            if _read_bytes(primary_npz) != _read_bytes(alternative_npz):
                reject_asset(
                    "AMBIGUOUS_DEPLOY_ASSET", "atlas_runtime_deploy.atlas_dir",
                    "one atlas pair (source tree and install prefix differ)",
                    sorted([primary, alternative]))
    return os.path.abspath(primary)


def _manifest_file(manifest, section, resolver):
    entry = manifest.get(section) or {}
    path = resolver(str(entry.get("package") or ""),
                    str(entry.get("path") or ""))
    if not path or not os.path.isfile(path):
        reject_asset("MANIFEST_FILE_MISSING", "manifest.%s.path" % section,
                     entry.get("path"), path)
    return path


def _load_yaml(path):
    import yaml
    with open(path, "r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream)
    if not isinstance(loaded, dict):
        reject_asset("INVALID_DEPLOY_ASSET", os.path.basename(path),
                     "yaml mapping", type(loaded).__name__)
    return loaded
