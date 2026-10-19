"""Immutable inputs for speculative placement; no scene ownership or execution.

A Transform is T_parent_child. Grasp is T_box_tcp, matching the existing
source=suction_contact_frame,target=box transport (not its inverse).
All geometry is metres, all quaternions are xyzw. Directed box axes must be
preserved; callers may not canonicalize yaw modulo pi after selecting a grasp.
"""
from dataclasses import dataclass
import hashlib
import math

from luggage_description.frame_transforms import quaternion_product, rotate_vector


def finite_tuple(values, size, name):
    result = tuple(float(v) for v in values)
    if len(result) != size or not all(math.isfinite(v) for v in result):
        raise ValueError("invalid " + name)
    return result


@dataclass(frozen=True)
class Transform:
    translation: tuple
    quaternion_xyzw: tuple

    def __post_init__(self):
        object.__setattr__(self, 'translation', finite_tuple(self.translation, 3, 'translation'))
        q = finite_tuple(self.quaternion_xyzw, 4, 'quaternion')
        if abs(sum(v*v for v in q) - 1.0) > 1e-6:
            raise ValueError('quaternion must be unit length')
        object.__setattr__(self, 'quaternion_xyzw', q)

    def compose(self, child):
        rotated = rotate_vector(self.quaternion_xyzw, child.translation)
        return Transform(tuple(a+b for a, b in zip(self.translation, rotated)),
                         quaternion_product(self.quaternion_xyzw, child.quaternion_xyzw))

    def inverse(self):
        x, y, z, w = self.quaternion_xyzw
        q = (-x, -y, -z, w)
        return Transform(rotate_vector(q, tuple(-v for v in self.translation)), q)


@dataclass(frozen=True)
class Snapshot:
    """An atomic owner-published OCC snapshot and its private collision scene.

    The adapter must verify these describe the same revision. Passing arbitrary
    unrelated scene bytes with an OCC revision is not an admitted snapshot.
    Bytes are copied on construction; the worker cannot mutate source messages.
    """
    scene_revision: str
    map_revision: int
    geometry_hash: str
    model_hash: str
    world_from_container: Transform
    surface_cdr: bytes
    scene_cdr: bytes
    surface_content_hash: str = ""
    scene_content_hash: str = ""
    proposal_scene_hash: str = ""

    def __post_init__(self):
        if not all(isinstance(s, str) and s.strip() for s in
                   (self.scene_revision, self.geometry_hash, self.model_hash)):
            raise ValueError('snapshot identity required')
        if type(self.map_revision) is not int or self.map_revision < 0:
            raise ValueError('invalid map revision')
        for name in ('surface_cdr', 'scene_cdr'):
            value = bytes(getattr(self, name))
            if not value:
                raise ValueError('empty ' + name)
            object.__setattr__(self, name, value)

    @property
    def identity(self):
        # Include bytes and static transform, not only mutable publisher labels.
        return (self.scene_revision, self.map_revision, self.geometry_hash,
                self.model_hash, self.world_from_container,
                self.surface_content_hash or hashlib.sha256(self.surface_cdr).hexdigest(),
                self.scene_content_hash or hashlib.sha256(self.scene_cdr).hexdigest())

    @property
    def proposal_identity(self):
        """Ordering hints only; never a result-cache or execution identity.

        The scene adapter may omit scalar joint samples from this hash. Every
        current placement and measured collision query must still be rebuilt.
        Adapters without that explicit hash retain the complete identity.
        """
        return (*self.identity[:-1], self.proposal_scene_hash or self.identity[-1])


@dataclass(frozen=True)
class GraspCandidate:
    candidate_id: str
    box_from_tcp: Transform
    observation_id: str
    contact_model_hash: str
    rank: int
    pick_references: tuple = ()

    def __post_init__(self):
        if not all(isinstance(s, str) and s.strip() for s in
                   (self.candidate_id, self.observation_id, self.contact_model_hash)):
            raise ValueError('candidate provenance required')
        if type(self.rank) is not int or self.rank < 0:
            raise ValueError('invalid candidate rank')


@dataclass(frozen=True)
class PlacementCandidate:
    slot_id: str
    container_from_box: Transform
    # Box poses at transit/entry/insert stages. Final box pose is checked too.
    critical_box_poses: tuple = ()

    def __post_init__(self):
        if not isinstance(self.slot_id, str) or not self.slot_id.strip():
            raise ValueError('slot identity required')
        poses = tuple(self.critical_box_poses)
        if not all(isinstance(p, Transform) for p in poses):
            raise ValueError('invalid critical pose')
        object.__setattr__(self, 'critical_box_poses', poses)


@dataclass(frozen=True)
class PriorRequest:
    request_id: str
    snapshot: Snapshot
    box_wdh: tuple
    placements: tuple
    grasps: tuple
    # Hypothetical template can be screened, but never adopted as measured.
    measured: bool
    candidate_limit: int = 32
    budget_s: float = 10.0
    world_from_pick_box: Transform = None

    def __post_init__(self):
        size = finite_tuple(self.box_wdh, 3, 'box_wdh')
        if min(size) <= 0:
            raise ValueError('box dimensions must be positive')
        object.__setattr__(self, 'box_wdh', size)
        object.__setattr__(self, 'placements', tuple(self.placements))
        object.__setattr__(self, 'grasps', tuple(self.grasps))
        if not self.request_id or not self.placements or not self.grasps:
            raise ValueError('request, placements and grasps required')
        if type(self.measured) is not bool:
            raise ValueError('explicit measured provenance required')
        if type(self.candidate_limit) is not int or not 1 <= self.candidate_limit <= 32:
            raise ValueError('candidate_limit must be in [1,32]')
        if not math.isfinite(self.budget_s) or not 0 < self.budget_s <= 10:
            raise ValueError('budget_s must be in (0,10]')
        if len({g.candidate_id for g in self.grasps}) != len(self.grasps):
            raise ValueError('duplicate grasp IDs')
        if len({p.slot_id for p in self.placements}) != len(self.placements):
            raise ValueError('duplicate slot IDs')
        if len({(g.observation_id, g.contact_model_hash) for g in self.grasps}) != 1:
            raise ValueError('mixed grasp provenance')

    @property
    def reuse_key(self):
        # Deliberately omits request_id; adoption binds to the current request.
        return (self.snapshot.identity, self.box_wdh, self.placements,
                self.grasps, self.measured, self.candidate_limit, self.budget_s, self.world_from_pick_box)
