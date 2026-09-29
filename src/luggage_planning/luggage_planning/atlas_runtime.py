"""Runtime wiring for the pre-computed reachability atlas.

Shared by placement_planner_node (candidate gate + rank) and
motion_planner_node (IK seed chain). Pure functions, no ROS imports, so the
same helpers serve pytest and the nodes.

Config resolution mirrors occupancy_place_paths.selector_weights_path: src
tree first (symlink install), installed share second, DEFAULTS with a loud
warning last.

The payload-atlas selection duplicates the ~30 lines in
luggage_packing.placement_reachability on purpose: the dependency direction
is packing -> planning, and this module must not import luggage_packing.
Keep the enveloping semantics identical when editing either copy.
"""

from __future__ import division

import hashlib
import math
import os
import sys

from luggage_planning.reachability_atlas import (
    ReachabilityAtlas,
    STATUS_NAMES,
)
from luggage_planning.atlas_frame_contracts import (
    geometry_hash as _contract_geometry_hash,
    payload_hash as _contract_payload_hash,
    snap_yaw_directed,
    wrapped_yaw_error,
)

CONFIG_NAME = "atlas_runtime.yaml"

DEFAULT_ATLAS_RUNTIME = {
    "enabled": True,
    "atlas_dir": "",
    "basename": "s20_container_collision_aware",
    "atlas_file": "",
    "hash_policy": "strict",    # off | warn | strict
    "query_offset": [0.0, 0.0, 0.0],
    "payload": {"policy": "auto", "explicit_size": [0.0, 0.0, 0.0]},
    "query_semantics": {
        # Query-layer correctness rules over the discrete cell verdicts; see
        # QuerySemantics. All are query-side only -- no re-sampling.
        "enabled": True,
        "floor_clamp": True,
        "neighbor_promote": True,
        "strict_interior_reject": True,
    },
    "placement": {
        "hard_reject": True, "w_reachability": 0.3, "dump_only": False,
    },
    "motion": {
        "enabled": True, "seed_set": "transit", "max_seeds": 3,
        "mode": "miss-retry", "fk_verify_tol_m": 0.03,
        "submit_mode": "reseed",
    },
}

# (ros param name, config path tuple, cast)
PARAM_SPEC = (
    ("atlas_enabled", ("enabled",), bool),
    ("atlas_basename", ("basename",), str),
    ("atlas_dir", ("atlas_dir",), str),
    ("atlas_file", ("atlas_file",), str),
    ("atlas_hash_policy", ("hash_policy",), str),
    ("atlas_query_offset", ("query_offset",), lambda v: [float(x) for x in v]),
    ("atlas_payload_policy", ("payload", "policy"), str),
    ("atlas_payload_size", ("payload", "explicit_size"),
     lambda v: [float(x) for x in v]),
    ("atlas_query_semantics", ("query_semantics", "enabled"), bool),
    ("atlas_qsem_floor_clamp", ("query_semantics", "floor_clamp"), bool),
    ("atlas_qsem_neighbor_promote", ("query_semantics", "neighbor_promote"), bool),
    ("atlas_qsem_strict_interior", ("query_semantics", "strict_interior_reject"), bool),
    ("atlas_placement_hard_reject", ("placement", "hard_reject"), bool),
    ("atlas_w_reachability", ("placement", "w_reachability"), float),
    ("atlas_placement_dump_only", ("placement", "dump_only"), bool),
    ("atlas_motion_enabled", ("motion", "enabled"), bool),
    ("atlas_seed_set", ("motion", "seed_set"), str),
    ("atlas_max_seeds", ("motion", "max_seeds"), int),
    ("atlas_mode", ("motion", "mode"), str),
    ("atlas_fk_verify_tol_m", ("motion", "fk_verify_tol_m"), float),
    ("atlas_submit_mode", ("motion", "submit_mode"), str),
)


def _module_dir(module_file=None):
    return os.path.dirname(os.path.abspath(module_file or __file__))


def atlas_runtime_config_path(module_file=None, name=CONFIG_NAME):
    """Config path: src tree first, installed share second, else None."""
    src_path = os.path.normpath(os.path.join(
        _module_dir(module_file), "..", "config", name))
    if os.path.isfile(src_path):
        return src_path
    try:
        from ament_index_python.packages import (
            get_package_share_directory)
        share_path = os.path.join(
            get_package_share_directory("luggage_planning"),
            "config", name)
    except Exception:  # noqa: BLE001 - not installed / no ament
        return None
    if os.path.isfile(share_path):
        return share_path
    return None


def package_data_dir(module_file=None):
    """Package data dir: src tree first, installed share second."""
    src_path = os.path.normpath(os.path.join(
        _module_dir(module_file), "..", "data"))
    if os.path.isdir(src_path):
        return src_path
    try:
        from ament_index_python.packages import (
            get_package_share_directory)
        return os.path.join(
            get_package_share_directory("luggage_planning"), "data")
    except Exception:  # noqa: BLE001
        return src_path


def default_atlas_dir(module_file=None):
    """Sim-scene atlas dir: src tree first, installed share second."""
    return os.path.join(package_data_dir(module_file), "reachability_atlas")


def resolve_atlas_dir(atlas_dir=None, module_file=None):
    """Resolve an atlas directory.

    Empty means the sim atlas (``reachability_atlas``). An absolute path is
    used as given. Any other value is a directory name under the package
    ``data/`` tree, so a deploy yaml can say ``reachability_atlas_deploy``
    without a machine-specific path.
    """
    text = str(atlas_dir or "").strip()
    if not text:
        return default_atlas_dir(module_file)
    if os.path.isabs(text):
        return text
    return os.path.normpath(os.path.join(package_data_dir(module_file), text))


def _deep_merge(base, override):
    out = dict(base)
    for key, value in (override or {}).items():
        if key in out and isinstance(out[key], dict) and isinstance(value, dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def load_atlas_runtime(path=None):
    """Load the runtime config over DEFAULT_ATLAS_RUNTIME (never silent)."""
    if path is None:
        path = atlas_runtime_config_path()
        if path is None:
            sys.stderr.write(
                "luggage_planning: %s not found (src tree or installed "
                "share); using DEFAULT_ATLAS_RUNTIME\n" % CONFIG_NAME)
            return _deep_merge(DEFAULT_ATLAS_RUNTIME, {})
    if not path or not os.path.isfile(path):
        sys.stderr.write(
            "luggage_planning: atlas runtime config %r missing; using "
            "DEFAULT_ATLAS_RUNTIME\n" % (path,))
        return _deep_merge(DEFAULT_ATLAS_RUNTIME, {})
    try:
        import yaml
        loaded = yaml.safe_load(open(path, "r", encoding="utf-8")) or {}
    except Exception as exc:  # noqa: BLE001 - unreadable yaml
        sys.stderr.write(
            "luggage_planning: atlas runtime config %r unreadable (%s); "
            "using DEFAULT_ATLAS_RUNTIME\n" % (path, exc))
        return _deep_merge(DEFAULT_ATLAS_RUNTIME, {})
    if not isinstance(loaded, dict):
        return _deep_merge(DEFAULT_ATLAS_RUNTIME, {})
    return _deep_merge(DEFAULT_ATLAS_RUNTIME, loaded)


def compute_scene_tf_hash(path):
    """MD5 of file bytes, byte-compatible with the atlas builder."""
    if not path or not os.path.isfile(path):
        return ""
    digest = hashlib.md5()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(4096), b""):
            digest.update(chunk)
    return digest.hexdigest()


def compute_urdf_hash(urdf_text):
    """MD5 of the robot_description string, matching the atlas builder.

    The builder stores ``hashlib.md5(urdf.encode()).hexdigest()`` of the
    expanded URDF that MoveIt collided against. An empty description is
    not a hash: callers must treat it as identity-not-proven.
    """
    if urdf_text is None:
        return ""
    if isinstance(urdf_text, bytes):
        raw = urdf_text
    else:
        raw = str(urdf_text).encode("utf-8")
    if not raw:
        return ""
    return hashlib.md5(raw).hexdigest()


ATLAS_IDENTITY_UNVERIFIED = "ATLAS_IDENTITY_UNVERIFIED"
ATLAS_SEMANTICS_MISMATCH = "ATLAS_SEMANTICS_MISMATCH"


def atlas_contract_record(atlas):
    """Contract block of a (possibly wrapped) atlas, or None for v1.

    Accepts ``ReachabilityAtlas`` and ``QuerySemantics`` alike; a plain test
    stub without the attribute is treated as v1.
    """
    record = getattr(atlas, "contract_record", None)
    return dict(record) if isinstance(record, dict) else None


def snap_yaw_for_atlas(atlas, yaw):
    """Snap ``yaw`` per the artifact's own semantics.

    v2 directed artifacts (contract block present) snap over 2pi without a
    pi fold: 0 and pi stay distinct branches. v1 artifacts keep the legacy
    pi-fold onto their bins. Returns ``(bin_yaw, true_error_rad)``.
    """
    if atlas_contract_record(atlas) is not None:
        return snap_yaw_directed(yaw, atlas.yaw_bins or [0.0])
    return snap_yaw_to_bin(atlas, yaw)


def directed_yaw_candidates(atlas, yaw):
    """Query yaws for one grasp: the directed yaw and its pi twin (v2).

    v1 keeps the single folded yaw. Both are already snapped per the
    artifact's semantics, so ``query`` sees no yaw mismatch.
    """
    if atlas_contract_record(atlas) is None:
        folded, _err = snap_yaw_to_bin(atlas, yaw)
        return [folded]
    first, _err = snap_yaw_directed(yaw, atlas.yaw_bins or [0.0])
    second, _err2 = snap_yaw_directed(
        float(yaw) + math.pi, atlas.yaw_bins or [0.0])
    if abs(wrapped_yaw_error(first, second)) < 1e-9:
        return [first]
    return [first, second]


def identity_verdict(dependencies, scene_tf_hash=None, urdf_hash=None,
                     policy="warn"):
    """Return ``(ok, message, scene_ok, urdf_ok)``.

    ``off`` skips the check. ``warn`` reports a supplied hash that differs
    from a stored hash. ``strict`` also fails when the caller or the atlas
    is missing either hash: an unverified atlas must not be used.
    """
    mode = str(policy or "warn")
    if mode == "off":
        return True, "ok", True, True
    deps = dependencies or {}
    stored_scene = str(deps.get("scene_tf_hash") or "")
    stored_urdf = str(deps.get("urdf_hash") or "")
    current_scene = str(scene_tf_hash or "")
    current_urdf = str(urdf_hash or "")
    scene_ok = True
    urdf_ok = True
    messages = []
    if mode == "strict":
        if not current_scene:
            scene_ok = False
            messages.append("scene_tf hash missing")
        if not current_urdf:
            urdf_ok = False
            messages.append("urdf hash missing")
        if not stored_scene:
            scene_ok = False
            messages.append("atlas scene_tf hash missing")
        if not stored_urdf:
            urdf_ok = False
            messages.append("atlas urdf hash missing")
    if current_scene and stored_scene and stored_scene != current_scene:
        scene_ok = False
        messages.append(
            "scene_tf hash mismatch (atlas=%s, current=%s)" % (
                stored_scene[:8], current_scene[:8]))
    if current_urdf and stored_urdf and stored_urdf != current_urdf:
        urdf_ok = False
        messages.append(
            "urdf hash mismatch (atlas=%s, current=%s)" % (
                stored_urdf[:8], current_urdf[:8]))
    ok = scene_ok and urdf_ok
    return ok, ("; ".join(messages) if messages else "ok"), scene_ok, urdf_ok


def certification_block_reason(cfg, diagnostics):
    """Placement/selector reason when a strict atlas must not rank or reject.

    Empty means the caller may use the loaded atlas, or the policy is not
    the fail-closed certification policy. ``dump_only`` records without
    deciding, so a missing atlas does not fail the request. A semantics
    refusal (v1 artifact under strict, ST06) reports
    ``ATLAS_SEMANTICS_MISMATCH`` instead of the identity reason.
    """
    cfg = cfg or {}
    if not cfg.get("enabled"):
        return ""
    if str(cfg.get("hash_policy") or "") != "strict":
        return ""
    placement = cfg.get("placement") or {}
    if placement.get("dump_only"):
        return ""
    semantics = str((diagnostics or {}).get("semantics") or "")
    if semantics:
        return semantics
    if (diagnostics or {}).get("loaded"):
        return ""
    return ATLAS_IDENTITY_UNVERIFIED


def _payload_suffix(box_size):
    return "_payload_%.2fx%.2fx%.2f" % tuple(float(v) for v in box_size)


def _available_payload_atlases(atlas_dir, basename):
    prefix = basename + "_payload_"
    found = []
    try:
        names = os.listdir(atlas_dir)
    except OSError:
        return found
    for name in sorted(names):
        if not name.startswith(prefix) or not name.endswith(".npz"):
            continue
        parts = name[len(prefix):-len(".npz")].split("x")
        if len(parts) != 3:
            continue
        try:
            size = [float(part) for part in parts]
        except ValueError:
            continue
        found.append((size, os.path.join(atlas_dir, name)))
    return found


def _select_enveloping(available, box_size):
    """Smallest atlas whose payload envelops the box, else the largest."""
    def volume(entry):
        return entry[0][0] * entry[0][1] * entry[0][2]

    enveloping = [
        entry for entry in available
        if all(entry[0][i] >= float(box_size[i]) - 1e-9 for i in range(3))]
    if enveloping:
        return min(enveloping, key=volume)
    return max(available, key=volume)


def resolve_atlas_files(cfg, box_size=None, atlas_dir=None):
    """Resolve (npz_path, meta_path) under the configured selection policy.

    Order: atlas_file override > exact payload match > enveloping payload >
    largest payload > empty-load atlas. Returns None when nothing exists.
    """
    if atlas_dir is None:
        atlas_dir = resolve_atlas_dir(cfg.get("atlas_dir"))
    else:
        atlas_dir = resolve_atlas_dir(atlas_dir)
    basename = str(cfg.get("basename") or DEFAULT_ATLAS_RUNTIME["basename"])

    override = str(cfg.get("atlas_file") or "")
    if override:
        meta = os.path.splitext(override)[0] + ".yaml"
        if os.path.isfile(override) and os.path.isfile(meta):
            return override, meta
        sys.stderr.write(
            "luggage_planning: atlas_file override %r missing npz/meta; "
            "falling back to payload selection\n" % (override,))

    candidates = []
    payload_cfg = cfg.get("payload") or {}
    size = [float(v) for v in (box_size or [])]
    policy = str(payload_cfg.get("policy") or "auto")
    if policy == "explicit" or not size:
        explicit = [float(v) for v in payload_cfg.get("explicit_size") or []]
        if policy == "explicit" and min(explicit or [0.0]) > 0.0:
            size = explicit
        else:
            size = []
    if size:
        exact = os.path.join(
            atlas_dir, basename + _payload_suffix(size) + ".npz")
        if os.path.isfile(exact):
            candidates.append(exact)
        available = _available_payload_atlases(atlas_dir, basename)
        if available:
            candidates.append(_select_enveloping(available, size)[1])
    candidates.append(os.path.join(atlas_dir, basename + ".npz"))
    for npz_path in candidates:
        meta_path = os.path.splitext(npz_path)[0] + ".yaml"
        if os.path.isfile(npz_path) and os.path.isfile(meta_path):
            return npz_path, meta_path
    return None


def stored_scene_tf_hash(cfg):
    """Scene hash recorded on the empty-load atlas for this runtime cell."""
    files = resolve_atlas_files(cfg, None)
    if not files:
        return ""
    _npz, meta_path = files
    try:
        import yaml
        meta = yaml.safe_load(open(meta_path, "r", encoding="utf-8")) or {}
    except Exception:  # noqa: BLE001 - unreadable meta
        return ""
    if not isinstance(meta, dict):
        return ""
    return str((meta.get("dependencies") or {}).get("scene_tf_hash") or "")


def load_atlas_runtime_for_scene(scene_path, explicit_path=None):
    """Load the atlas cell sampled for this scene file.

    An explicit runtime yaml wins. Otherwise each installed cell
    (simulation ``atlas_runtime.yaml``, deploy ``atlas_runtime_deploy.yaml``)
    is matched by the ``scene_tf_hash`` stored on its empty-load atlas.
    ``scene_tf.yaml.example`` therefore selects the simulation reachable
    points, and ``scene_tf_deploy.yaml`` selects the deploy points. No
    match, or two matches, keeps the simulation cell so strict identity
    can refuse a foreign scene.
    """
    explicit = str(explicit_path or "").strip()
    if explicit:
        return load_atlas_runtime(explicit)
    scene_hash = compute_scene_tf_hash(scene_path)
    matched = []
    for name in (CONFIG_NAME, "atlas_runtime_deploy.yaml"):
        path = atlas_runtime_config_path(name=name)
        if not path:
            continue
        cfg = load_atlas_runtime(path)
        if scene_hash and stored_scene_tf_hash(cfg) == scene_hash:
            matched.append(cfg)
    if len(matched) == 1:
        return matched[0]
    if len(matched) > 1:
        sys.stderr.write(
            "luggage_planning: scene hash %s matches more than one atlas "
            "cell; using the simulation cell\n" % scene_hash[:8])
    return load_atlas_runtime()


def load_runtime_atlas(cfg, box_size=None, scene_tf_hash=None,
                       urdf_hash=None):
    """Load the atlas under hash_policy.

    Returns (atlas_or_None, diagnostics). warn keeps the atlas loaded and
    notes a supplied-hash mismatch. strict refuses to load when either
    hash is missing or does not match the atlas metadata, when the artifact
    is pi-folded v1 (ST06: strict never runs old semantics), or when the v2
    contract block does not validate against the identity reproduced from
    the artifact's own geometry/payload sections (a relabelled v1 cannot).
    """
    diagnostics = {
        "npz": None, "meta": None, "hash_policy": cfg.get("hash_policy"),
        "scene_tf_ok": True, "urdf_ok": True, "hash_message": "ok",
        "loaded": False, "reason": "", "semantics": "",
    }
    files = resolve_atlas_files(cfg, box_size)
    if files is None:
        diagnostics["reason"] = "atlas_files_not_found"
        sys.stderr.write(
            "luggage_planning: no atlas files resolved (dir=%r base=%r)\n"
            % (cfg.get("atlas_dir") or default_atlas_dir(),
               cfg.get("basename")))
        return None, diagnostics
    npz_path, meta_path = files
    diagnostics["npz"] = npz_path
    diagnostics["meta"] = meta_path
    atlas = ReachabilityAtlas.load(npz_path, meta_path)
    policy = str(cfg.get("hash_policy") or "warn")
    if policy != "off":
        contract = atlas.contract_record
        if policy == "strict" and contract is None:
            # ST06: strict certification requires directed 2pi concrete-tool
            # semantics. A v1 artifact is never relabelled to pass; it is
            # refused before any query or seed use.
            diagnostics["reason"] = "atlas_semantics_v1_strict"
            diagnostics["semantics"] = ATLAS_SEMANTICS_MISMATCH
            return None, diagnostics
        ok, message, scene_ok, urdf_ok = identity_verdict(
            atlas.meta.get("dependencies") or {},
            scene_tf_hash, urdf_hash, policy=policy)
        diagnostics["scene_tf_ok"] = scene_ok
        diagnostics["urdf_ok"] = urdf_ok
        diagnostics["hash_message"] = message
        if not ok:
            if policy == "strict":
                diagnostics["reason"] = (
                    "hash_mismatch_strict" if "mismatch" in message
                    else "hash_unverified_strict")
                return None, diagnostics
            sys.stderr.write(
                "luggage_planning: atlas %s (%s); continuing (warn)\n"
                % (message, os.path.basename(npz_path)))
        if contract is not None:
            verdict, contract_message = verify_atlas_contract(
                atlas, scene_tf_hash, urdf_hash, policy=policy)
            diagnostics["contract_ok"] = verdict
            diagnostics["contract_message"] = contract_message
            if not verdict:
                if policy == "strict":
                    diagnostics["reason"] = "atlas_contract_strict"
                    diagnostics["semantics"] = ATLAS_IDENTITY_UNVERIFIED
                    return None, diagnostics
                sys.stderr.write(
                    "luggage_planning: atlas contract %s (%s); continuing "
                    "(warn)\n" % (contract_message, os.path.basename(npz_path)))
    diagnostics["loaded"] = True
    diagnostics["reason"] = "ok"
    return atlas, diagnostics


def verify_atlas_contract(atlas, scene_tf_hash=None, urdf_hash=None,
                          policy="warn"):
    """Validate the artifact contract against its own identity blocks.

    The expected identity is reproduced from the artifact's geometry and
    payload sections (plus the live scene/URDF hashes when supplied), so a
    contract block whose hashes disagree with the arrays it certifies is a
    relabel, not an identity. Returns ``(ok, message)``.
    """
    from luggage_description.frame_contracts import (
        ContractError, validate_atlas)
    meta = atlas.meta or {}
    deps = meta.get("dependencies") or {}
    container = meta.get("container") or {}
    payload = meta.get("payload") or {}
    expected = {
        "scene_revision": str(deps.get("scene_tf_hash") or ""),
        "geometry_hash": _contract_geometry_hash(
            container.get("inner_dimensions") or (0.0, 0.0, 0.0),
            container.get("floor_z", 0.0),
            container.get("ceiling_z", 0.0),
            container.get("opening_side", "")),
        "urdf_hash": str(deps.get("urdf_hash") or ""),
        "payload_hash": _contract_payload_hash(payload),
    }
    try:
        validate_atlas(atlas.contract_record, expected)
    except ContractError as exc:
        return False, str(getattr(exc, "code", exc)) + ": " + str(
            getattr(exc, "field", ""))
    if scene_tf_hash and expected["scene_revision"] != str(scene_tf_hash):
        return False, "scene identity differs from the live scene"
    if urdf_hash and expected["urdf_hash"] != str(urdf_hash):
        return False, "urdf identity differs from the live model"
    return True, "ok"


def _seed_rows_ok(seed):
    values = [float(v) for v in seed or []]
    if len(values) != 6:
        return False
    if any(math.isnan(v) for v in values):
        return False
    # _pad_seeds zero-fills unused branch slots; a real joint vector never
    # reads all-zero (joint2 at exactly 0.0 rad is possible but not all six).
    return any(abs(v) > 1e-9 for v in values)


class QuerySemantics(object):
    """Correct-by-construction query layer over a loaded atlas.

    The atlas is a discrete cell-centre verdict, but the runtime asks about
    arbitrary points. Two discretisation artifacts follow, both fixed here
    on the query side (no re-sampling needed):

      floor_clamp -- a contact cell whose sampled CENTRE would drive the
        payload through the container floor says nothing about the real
        floor-resting pose. The grid is anchored at floor-0.03 with 0.15 m
        cells, so a carryon-class contact (z = floor+0.25 = 0.78) lands in
        iz1 whose centre 0.725 puts the box bottom 55 mm into the slab and
        the whole cell reads UNREACHABLE. The query is re-anchored to the
        lowest cell whose centre keeps ``box_height`` above ``floor_z``.

      neighbor_promote / strict_interior_reject -- a non-reachable cell
        with a reachable 26-neighbour (any yaw bin) is a discretisation
        boundary, not knowledge. Promote reports it as UNKNOWN (neutral
        prior, never a hard reject); strict-interior additionally blocks
        the hard reject even when promotion is off. Together they give the
        design guarantee: a point between two reachable points can never
        be hard-rejected, because rejection requires an unreachable-region
        interior.

    Pass-through attributes (``yaw_bins``/``resolution``/``meta``) keep the
    wrapper usable wherever a raw atlas is (hook duck-typing, seed query).
    """

    def __init__(self, atlas, floor_z=None, box_height=None, opts=None):
        self._inner = atlas
        self._floor_z = None if floor_z is None else float(floor_z)
        self._box_height = (
            None if box_height is None else float(box_height))
        opts = opts or {}
        self._floor_clamp = bool(opts.get("floor_clamp", True))
        self._neighbor_promote = bool(opts.get("neighbor_promote", True))
        self._strict_interior = bool(opts.get("strict_interior_reject", True))

    @property
    def yaw_bins(self):
        return self._inner.yaw_bins

    @property
    def resolution(self):
        return self._inner.resolution

    @property
    def meta(self):
        return self._inner.meta

    @property
    def contract_record(self):
        return getattr(self._inner, "contract_record", None)

    def query_pi_equivalents(self, x, y, z, yaw, yaw_tolerance=None):
        """Both-directed-grasp query through the semantics layer.

        See ``ReachabilityAtlas.query_pi_equivalents``; each branch query is
        re-anchored/promoted like a direct ``query`` here.
        """
        first = self.query(x, y, z, yaw, yaw_tolerance=yaw_tolerance)
        contract = self.contract_record
        if not isinstance(contract, dict):
            return first, bool(first.hard_reject_safe)
        second = self.query(
            x, y, z, float(yaw) + math.pi, yaw_tolerance=yaw_tolerance)

        def rank(result):
            return (
                1 if int(result.status) in (2, 3) else 0,
                1 if (result.contact_seeds or result.transit_seeds) else 0,
                int(result.status),
            )

        best = first if rank(first) >= rank(second) else second
        return best, bool(first.hard_reject_safe and second.hard_reject_safe)

    def _floor_feasible_z(self, z):
        """Re-anchor z onto the lowest cell whose centre clears the floor.

        Returns z unchanged when the containment cell's centre already
        keeps the payload above the floor (or when the rule is disabled /
        lacks the box height). Only the false-negative cells move.
        """
        if (not self._floor_clamp or self._box_height is None
                or self._floor_z is None):
            return float(z)
        grid = (self._inner.meta or {}).get("grid") or {}
        origin = grid.get("origin")
        size = grid.get("size") or (0, 0, 0)
        res = float(self._inner.resolution)
        if not origin or len(size) < 3 or res <= 0.0:
            return float(z)
        z0 = float(origin[2])
        nz = int(size[2])
        start = int(math.floor((float(z) - z0) / res))
        iz = start
        while (0 <= iz < nz
               and z0 + (iz + 0.5) * res - self._box_height
               < self._floor_z - 1e-9):
            iz += 1
        if iz == start or not (0 <= iz < nz):
            return float(z)
        return z0 + (iz + 0.5) * res

    def _neighborhood_reachable(self, indices):
        """Any {MARGINAL, REACHABLE} cell in the 26-neighbourhood, any yaw."""
        ix, iy, iz, _iyaw = indices
        grid = self._inner.status
        nx, ny, nz = grid.shape[0], grid.shape[1], grid.shape[2]
        return bool(
            (grid[max(0, ix - 1):min(nx, ix + 2),
                  max(0, iy - 1):min(ny, iy + 2),
                  max(0, iz - 1):min(nz, iz + 2), :] >= 2).any())

    def query(self, x, y, z, yaw=0.0, yaw_tolerance=None):
        result = self._inner.query(
            x, y, self._floor_feasible_z(z), yaw, yaw_tolerance)
        status = int(result.status)
        if status not in (0, 1) or result.indices is None:
            return result
        has_reachable_neighbor = self._neighborhood_reachable(result.indices)
        if has_reachable_neighbor and self._neighbor_promote:
            return result._replace(
                status=0, hard_reject_safe=False, score=0.0,
                reason="boundary_reachable_neighbor")
        if (has_reachable_neighbor and self._strict_interior
                and result.hard_reject_safe):
            return result._replace(hard_reject_safe=False)
        return result


def apply_query_semantics(atlas, cfg, floor_z=None, box_height=None):
    """Wrap ``atlas`` in QuerySemantics per the runtime config, or pass through.

    Returns the atlas unchanged when it is None or the semantics block is
    disabled, so call sites need no branching of their own.
    """
    if atlas is None:
        return None
    opts = (cfg or {}).get("query_semantics") or {}
    if not opts.get("enabled", True):
        return atlas
    return QuerySemantics(atlas, floor_z=floor_z, box_height=box_height,
                          opts=opts)


def snap_yaw_to_bin(atlas, yaw):
    """Fold a pi-periodic yaw onto the nearest atlas yaw bin.

    ``query`` measures plain angular distance, so a yaw of ~pi or -pi/2 would
    fail yaw_mismatch against bins [0, pi/2] even though the footprint is
    identical. Returns ``(bin_yaw, true_error_rad)``.
    """
    reduced = float(yaw) % math.pi
    best_bin, best_err = 0.0, float("inf")
    for bin_yaw in (atlas.yaw_bins or [0.0, math.pi / 2.0]):
        delta = abs(reduced - float(bin_yaw)) % math.pi
        err = min(delta, math.pi - delta)
        if err < best_err:
            best_err, best_bin = err, float(bin_yaw)
    return best_bin, best_err


def _extract_seeds(result, seed_set):
    rows = []
    if seed_set in ("transit", "both"):
        rows.extend(result.transit_seeds or [])
    if seed_set in ("contact", "both"):
        rows.extend(result.contact_seeds or [])
    seeds = [row for row in rows if _seed_rows_ok(row)]
    unique = []
    for row in seeds:
        key = tuple(round(float(v), 6) for v in row)
        if key not in [u[0] for u in unique]:
            unique.append((key, row))
    return [row for _, row in unique]


# 26-neighbourhood in ascending distance order (face, edge, corner), in cell
# units. A seed one cell (0.15 m) away is still an excellent IK seed.
_NEIGHBOR_OFFSETS = [
    (dx, dy, dz)
    for ring in (1,)  # extend here if a radius-2 fallback is ever needed
    for dx in range(-ring, ring + 1)
    for dy in range(-ring, ring + 1)
    for dz in range(-ring, ring + 1)
    if (dx, dy, dz) != (0, 0, 0)
]
_NEIGHBOR_OFFSETS.sort(key=lambda o: (abs(o[0]) + abs(o[1]) + abs(o[2]),
                                      o[0], o[1], o[2]))


def atlas_seeds_for_query(atlas, x, y, z, yaw, seed_set="transit",
                          max_seeds=3, query_offset=None):
    """Query the atlas for IK seeds at one container-frame contact point.

    The exact containment cell is queried first; when it holds no usable
    seeds (UNKNOWN cells are common near the opening — the 2026-09-22_1540
    slot point itself lands in an empty cell while its neighbour carries the
    validated seed), face/edge/corner neighbour cells are tried in ascending
    distance. Seeds only seed IK: they are re-solved collision-aware and
    never submitted blind, so borrowing a neighbour's branch is safe.

    Returns ``{"status", "status_name", "reason", "cell", "seed_cell",
    "seed_cell_offset", "yaw_error", "n_seeds", "seeds"}``.
    """
    empty = {
        "status": None, "status_name": "no_atlas", "reason": "no_atlas",
        "cell": None, "seed_cell": None, "seed_cell_offset": None,
        "yaw_error": None, "n_seeds": 0, "seeds": [],
    }
    if atlas is None:
        return empty
    offset = [float(v) for v in (query_offset or (0.0, 0.0, 0.0))]
    base = [float(x) + offset[0], float(y) + offset[1], float(z) + offset[2]]
    bin_yaw, yaw_err = snap_yaw_for_atlas(atlas, yaw)

    def _query_at(point):
        result = atlas.query(point[0], point[1], point[2], bin_yaw)
        seeds = _extract_seeds(result, seed_set)[:max(0, int(max_seeds))]
        return result, seeds

    result, seeds = _query_at(base)
    record = {
        "status": int(result.status),
        "status_name": STATUS_NAMES.get(int(result.status), "unknown"),
        "reason": str(result.reason),
        "cell": list(result.indices) if result.indices else None,
        "seed_cell": (list(result.indices) if result.indices else None),
        "seed_cell_offset": [0, 0, 0] if result.indices else None,
        "yaw_error": round(float(yaw_err), 4),
        "n_seeds": 0,
        "seeds": [],
    }
    if seeds:
        record["n_seeds"] = len(seeds)
        record["seeds"] = [[float(v) for v in row] for row in seeds]
        return record
    if result.indices is None:
        return record  # out of bounds / yaw mismatch: no cell to neighbour off

    resolution = float(atlas.resolution)
    grid_meta = ((atlas.meta or {}).get("grid") or {})
    grid_origin = [float(v) for v in grid_meta.get("origin", (0, 0, 0))]

    def _center_distance(cell):
        center = [
            grid_origin[i] + (cell[i] + 0.5) * resolution for i in range(3)]
        return sum((base[i] - center[i]) ** 2 for i in range(3))

    # Collect every seeded neighbour, then take seeds from the cells whose
    # centres are closest to the query point (deterministic; sorting by the
    # offset magnitude alone would tie-break on cell index, not proximity).
    found = []
    for dx, dy, dz in _NEIGHBOR_OFFSETS:
        shifted = [
            base[0] + dx * resolution,
            base[1] + dy * resolution,
            base[2] + dz * resolution,
        ]
        nb_result, nb_seeds = _query_at(shifted)
        if nb_seeds and nb_result.indices is not None:
            found.append((
                _center_distance(list(nb_result.indices)),
                list(nb_result.indices), [dx, dy, dz], nb_seeds))
    found.sort(key=lambda item: (item[0], item[1]))
    seeds = []
    seed_cell = None
    seed_offset = None
    for _, cell, cell_offset, cell_seeds in found:
        for row in cell_seeds:
            if len(seeds) >= max(0, int(max_seeds)):
                break
            if row not in seeds:
                seeds.append(row)
        if seeds and seed_cell is None:
            seed_cell = cell
            seed_offset = cell_offset
        if len(seeds) >= max(0, int(max_seeds)):
            break
    if seeds:
        record["seed_cell"] = seed_cell
        record["seed_cell_offset"] = seed_offset
        record["n_seeds"] = len(seeds)
        record["seeds"] = [[float(v) for v in row] for row in seeds]
    return record


def apply_ros_overrides(node, cfg):
    """Fold declared ``atlas_*`` node parameters back into ``cfg``.

    Nodes declare each parameter with its yaml value as the default, so this
    only changes ``cfg`` where the parameter was set explicitly.
    """
    for ros_name, path, cast in PARAM_SPEC:
        try:
            value = node.get_parameter(ros_name).value
        except Exception:  # noqa: BLE001 - parameter not declared
            continue
        if value is None:
            continue
        try:
            value = cast(value)
        except Exception:  # noqa: BLE001 - keep yaml value on bad type
            continue
        target = cfg
        for key in path[:-1]:
            target = target.setdefault(key, {})
        target[path[-1]] = value
    return cfg
