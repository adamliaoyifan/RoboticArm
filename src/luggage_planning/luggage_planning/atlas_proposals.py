"""Identity-qualified paired atlas proposals, never motion authority.

An explicit qualification binds exact NPZ/YAML bytes to a separately verified
model/scene/tool/payload contract. Matching old scene hashes alone is insufficient.
Branches retain storage indices: independently filtered seed lists must not be
zipped together. No stored complete edge exists in current artifacts; consumers
refine exact targets and check/plan in their current predicted carrying scene.
"""
from dataclasses import dataclass
import hashlib
import io
import math
from pathlib import Path

import numpy as np
import yaml

from luggage_description.frame_contracts import validate_atlas
from luggage_planning.atlas_frame_contracts import payload_hash, tool_target_in_base
from luggage_planning.placement_prior_contracts import Transform, finite_tuple


def artifact_digest(npz_bytes, yaml_bytes):
    digest = hashlib.sha256()
    for value in (npz_bytes, yaml_bytes):
        digest.update(len(value).to_bytes(8, "big"))
        digest.update(value)
    return digest.hexdigest()


@dataclass(frozen=True)
class AtlasQualification:
    artifact_sha256: str
    scene_tf_hash: str
    urdf_hash: str
    geometry_hash: str
    payload_hash: str
    validation_id: str

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError("qualification requires " + name)
        if (len(self.artifact_sha256) != 64
                or any(c not in "0123456789abcdef" for c in self.artifact_sha256)):
            raise ValueError("invalid qualified artifact SHA256")


@dataclass(frozen=True)
class AtlasBranchProposal:
    branch_id: str
    storage_index: int
    contact_joints: tuple
    transit_joints: tuple = ()
    transit_xyz_container: tuple = ()
    transit_lift_m: float | None = None
    reason: str = "CONTACT_ONLY"


@dataclass(frozen=True)
class AtlasPayloadRecord:
    enabled: bool
    shape: str | None
    size: tuple
    attach_link: str | None
    offset: tuple
    touch_links: tuple


@dataclass(frozen=True)
class AtlasProposalSet:
    source: AtlasQualification | None
    query_xyz_container: tuple
    query_yaw_rad: float
    status: int = 0
    reason: str = "ATLAS_UNAVAILABLE"
    cell: tuple = ()
    sample_xyz_container: tuple = ()
    sample_yaw_rad: float | None = None
    opening_connected: bool = False
    joint_margin_rad: float | None = None
    manipulability: float | None = None
    neighbor_confidence: float | None = None
    lift_clearance_m: float | None = None
    cell_branch_kind: int | None = None  # legacy scalar describes first branch only
    payload: AtlasPayloadRecord | None = None
    branches: tuple = ()
    edge_evidence: str = "MISSING_REQUIRES_ONLINE_CONNECTION"


def _optional_finite(value):
    return float(value) if value is not None and math.isfinite(float(value)) else None


class AtlasProposalReader:
    """Read immutable file bytes once; qualification comes from offline review.

    This class does not manufacture a qualification or update the old runtime's
    rejection semantics. Missing/mismatched qualification yields no proposals.
    """
    def __init__(self, npz_path, yaml_path, qualification=None):
        self.source = qualification
        self.atlas = None
        self.reason = "ATLAS_UNQUALIFIED"
        if not isinstance(qualification, AtlasQualification):
            return
        try:
            npz_raw, meta_raw = Path(npz_path).read_bytes(), Path(yaml_path).read_bytes()
            if artifact_digest(npz_raw, meta_raw) != qualification.artifact_sha256:
                self.reason = "ATLAS_ARTIFACT_MISMATCH"
                return
            meta = yaml.safe_load(meta_raw)
            if not isinstance(meta, dict) or any(not isinstance(meta.get(k), dict)
                    for k in ("dependencies", "grid", "contract", "payload")):
                self.reason = "ATLAS_INVALID_METADATA"
                return
            dependencies = meta.get("dependencies", {})
            if (dependencies.get("scene_tf_hash") != qualification.scene_tf_hash
                    or dependencies.get("urdf_hash") != qualification.urdf_hash
                    or dependencies.get("ik_link") != "suction_contact_frame"
                    or dependencies.get("ik_group") != "elfin_arm"
                    or meta["grid"].get("frame") != "container_link"):
                self.reason = "ATLAS_DEPENDENCY_MISMATCH"
                return
            validate_atlas(meta.get("contract"), {
                "scene_revision": qualification.scene_tf_hash,
                "urdf_hash": qualification.urdf_hash,
                "geometry_hash": qualification.geometry_hash,
                "payload_hash": qualification.payload_hash,
            })
            grid = meta["grid"]
            finite_tuple(grid.get("origin", ()), 3, "atlas origin")
            sizes = grid.get("size", ())
            resolution = float(grid.get("resolution_xyz", 0.))
            if (len(sizes) != 3 or any(type(n) is not int or n <= 0 for n in sizes)
                    or not math.isfinite(resolution) or resolution <= 0
                    or grid.get("yaw_bins") != meta["contract"]["yaw_bins_rad"]):
                self.reason = "ATLAS_GRID_CONTRACT_MISMATCH"
                return
            if payload_hash(meta.get("payload", {})) != qualification.payload_hash:
                self.reason = "ATLAS_PAYLOAD_MISMATCH"
                return
            payload = meta["payload"]
            if type(payload.get("enabled")) is not bool:
                raise ValueError("explicit payload enabled flag required")
            if payload["enabled"]:
                size = finite_tuple(payload.get("size", ()), 3, "payload size")
                finite_tuple(payload.get("offset", ()), 3, "payload offset")
                if (min(size) <= 0 or payload.get("shape") != "box"
                        or payload.get("attach_link") != "suction_contact_frame"
                        or not isinstance(payload.get("touch_links"), list)
                        or not all(isinstance(s, str) and s for s in payload["touch_links"])):
                    raise ValueError("invalid sampled payload geometry")
            from luggage_planning.reachability_atlas import ReachabilityAtlas
            with np.load(io.BytesIO(npz_raw), allow_pickle=False) as arrays:
                atlas = ReachabilityAtlas(dict(arrays), meta)
            if atlas._nseed > 64:
                self.reason = "ATLAS_BRANCH_CAP"
                return
            self.atlas, self.reason = atlas, "QUALIFIED"
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.reason = "ATLAS_INVALID: " + type(error).__name__

    def query(self, x, y, z, yaw=0.):
        point = finite_tuple((x, y, z), 3, "atlas query")
        angle = float(yaw)
        if not math.isfinite(angle):
            raise ValueError("finite atlas yaw required")
        empty = AtlasProposalSet(self.source, point, angle, reason=self.reason)
        if self.atlas is None:
            return empty
        return self.atlas.branch_proposals(point, angle, self.source)


def proposals_from_cell(atlas, point, yaw, source):
    """Called by the atlas's narrow public paired-array accessor."""
    result = atlas.query(*point, yaw)
    if result.indices is None:
        return AtlasProposalSet(source, point, yaw, reason=result.reason)
    cell = tuple(result.indices)
    sample = tuple(atlas._origin[i] + (cell[i]+.5)*atlas._resolution for i in range(3))
    branches = []
    count = int(atlas._solution_count[cell])
    reason = result.reason
    if count > atlas._contact_seed_capacity:
        reason = "INVALID_BRANCH_COUNT"
    elif result.status in (2, 3) and atlas._contact_ik[cell]:
        for index in range(count):
            contact = tuple(float(v) for v in atlas._contact_seeds[cell][index])
            if not all(math.isfinite(v) for v in contact):
                continue  # retain the original index of subsequent branches
            transit, target, lift = (), (), None
            if (atlas._transit_ik[cell] and index < atlas._transit_seed_capacity
                    and atlas._transit_target is not None):
                q = tuple(float(v) for v in atlas._transit_seeds[cell][index])
                xyz = tuple(float(v) for v in atlas._transit_target[cell][index])
                if all(math.isfinite(v) for v in q+xyz):
                    transit, target = q, xyz
                    if atlas._transit_lift is not None:
                        lift = _optional_finite(atlas._transit_lift[cell][index])
            branch_id = source.artifact_sha256 + ":" + ":".join(map(str, cell+(index,)))
            branches.append(AtlasBranchProposal(branch_id, index, contact, transit,
                target, lift, "PAIRED_SEEDS_ONLY" if transit else "CONTACT_ONLY"))
    payload = atlas.meta.get("payload", {})
    return AtlasProposalSet(source=source, query_xyz_container=point, query_yaw_rad=yaw,
        status=int(result.status), reason=reason, cell=cell, sample_xyz_container=sample,
        sample_yaw_rad=float(atlas._yaw_bins[cell[3]]), opening_connected=bool(result.opening_connected),
        joint_margin_rad=_optional_finite(result.joint_margin),
        manipulability=_optional_finite(result.manipulability),
        neighbor_confidence=_optional_finite(result.neighbor_confidence),
        lift_clearance_m=(_optional_finite(atlas._lift_clearance[cell])
                          if atlas._lift_clearance is not None else None),
        cell_branch_kind=int(atlas._branch_kind[cell]) if atlas._branch_kind is not None else None,
        payload=AtlasPayloadRecord(bool(payload.get("enabled")), payload.get("shape"),
            tuple(payload.get("size") or ()), payload.get("attach_link"),
            tuple(payload.get("offset") or ()), tuple(payload.get("touch_links") or ())),
        branches=tuple(branches))


@dataclass(frozen=True)
class RefinedAtlasPair:
    branch_id: str
    contact_joints: tuple
    transit_joints: tuple
    transit_world: Transform
    connection_positions: tuple


@dataclass(frozen=True)
class AtlasEndpointResult:
    pairs: tuple
    contact_options: tuple
    trace: tuple
    fallback_required: bool
    reason: str


def refine_atlas_pairs(proposals, *, expected_source, exact_contact_world,
                       world_from_container, refine, check_carrying, connect,
                       deadline, cancelled, clock):
    """Use both stored branches in the CURRENT carrying scene.

    Ports own the predicted attachment/snapshot and share the request deadline.
    ``refine(Transform,q,...)`` returns exact-target IK or None; carrying checks
    include the real measured box envelope even for an unloaded base atlas.
    ``check_carrying`` must return a literal bool; transport/verdict objects
    require an explicit adapter so a rejected but truthy object cannot pass.
    ``connect`` returns a checked, untimed geometric connection. All surviving
    pairs remain alternatives for the whole-chain planner. No path is executable.
    """
    pairs, contacts, trace = [], [], []
    if not isinstance(expected_source, AtlasQualification) or proposals.source != expected_source:
        return AtlasEndpointResult((), (), (), True, "ATLAS_SOURCE_MISMATCH")
    if not isinstance(exact_contact_world, Transform) or not isinstance(world_from_container, Transform):
        raise ValueError("explicit world-frame transforms required")
    for branch in proposals.branches:
        def stopped():
            return cancelled() or clock() >= deadline
        if stopped():
            return AtlasEndpointResult((), (), tuple(trace), True, "CANCELLED_OR_DEADLINE")
        kwargs = dict(deadline=deadline, cancelled=cancelled)
        q = refine(exact_contact_world, branch.contact_joints, **kwargs)
        if stopped():
            return AtlasEndpointResult((), (), tuple(trace), True, "CANCELLED_OR_DEADLINE")
        if q is None:
            trace.append((branch.branch_id, "CONTACT_REFINE_MISS"))
            continue
        q = finite_tuple(q, 6, "refined contact")
        contact_valid = check_carrying(q, **kwargs)
        if stopped():
            return AtlasEndpointResult((), (), tuple(trace), True, "CANCELLED_OR_DEADLINE")
        if contact_valid is not True:
            trace.append((branch.branch_id, "CONTACT_PAYLOAD_REJECT"))
            continue
        contacts.append((branch.branch_id, q))
        if not branch.transit_joints or not branch.transit_xyz_container:
            trace.append((branch.branch_id, "MISSING_TRANSIT_TARGET"))
            continue
        # This may be a predecessor's target, not contact + a global Z lift.
        _, quat = tool_target_in_base(0., 0., 0., proposals.sample_yaw_rad,
                                      (0., 0., 0.), (0., 0., 0.))
        transit_world = world_from_container.compose(Transform(branch.transit_xyz_container, quat))
        transit = refine(transit_world, branch.transit_joints, **kwargs)
        if stopped():
            return AtlasEndpointResult((), (), tuple(trace), True, "CANCELLED_OR_DEADLINE")
        if transit is None:
            trace.append((branch.branch_id, "TRANSIT_REFINE_MISS"))
            continue
        transit = finite_tuple(transit, 6, "refined transit")
        transit_valid = check_carrying(transit, **kwargs)
        if stopped():
            return AtlasEndpointResult((), (), tuple(trace), True, "CANCELLED_OR_DEADLINE")
        if transit_valid is not True:
            trace.append((branch.branch_id, "TRANSIT_PAYLOAD_REJECT"))
            continue
        if stopped():
            return AtlasEndpointResult((), (), tuple(trace), True, "CANCELLED_OR_DEADLINE")
        connection = connect(transit, q, **kwargs)
        if stopped():
            return AtlasEndpointResult((), (), tuple(trace), True, "CANCELLED_OR_DEADLINE")
        positions = tuple(tuple(p) for p in connection.positions)
        if (not connection.success or not connection.complete or not 2 <= len(positions) <= 50000
                or positions[0] != transit or positions[-1] != q
                or any(len(p) != 6 or not all(math.isfinite(v) for v in p) for p in positions)
                or any(max(abs(a-b) for a, b in zip(left, right)) > .01+1e-10
                       for left, right in zip(positions, positions[1:]))):
            trace.append((branch.branch_id, "ONLINE_CONNECTION_MISS"))
            continue
        pairs.append(RefinedAtlasPair(branch.branch_id, q, transit, transit_world, positions))
        trace.append((branch.branch_id, "ONLINE_PAIR_CHECKED"))
    return AtlasEndpointResult(tuple(pairs), tuple(contacts), tuple(trace), not bool(pairs),
                               "PAIRS_READY" if pairs else "ONLINE_FALLBACK_REQUIRED")
