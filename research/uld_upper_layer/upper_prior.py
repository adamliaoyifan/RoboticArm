"""Read-only upper-placement prior: geometry windows, IK seeds, route hints.

Deterministic source-model geometry plus archived atlas/catalogue lookup. The
prior never writes the OCC, never commands motion or vacuum, and grants no
execution certificate: every seed or ordering hint is re-checked end to end by
the normal planner. Approximate matches carry seeds only.
"""
from dataclasses import dataclass, asdict
import hashlib
import json
import math
import time
from pathlib import Path
import types
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from xml.etree import ElementTree as ET

from candidates import matrix
from planning_types import digest
from support_geometry import measure_support

# Archived atlas orientation contract (orientation_reachability_g12_01):
# r0 = R_y(beta + 180deg), spin in {0, 180}; TCP rotations in elfin_base_link.
ATLAS_MODE_BETA = (('down', 0.), ('left', 90.), ('up', 180.), ('right', 270.))
ATLAS_SPINS = (0, 180)
_ASSEMBLY_CACHE = {}
_ATLAS_CACHE = {}


def atlas_rotation(mode):
    beta = dict(ATLAS_MODE_BETA)[mode]
    return Rotation.from_euler('y', beta + 180., degrees=True).as_matrix()


def link_transforms(urdf, q):
    """Same forward walk as FullArmScene.link_transforms, without a scene."""
    values = dict(zip([f'elfin_joint{i}' for i in range(1, 7)], q))
    from geometry import pose
    transforms = {urdf.root_link: np.eye(4)}
    pending = [urdf.root_link]
    while pending:
        parent = pending.pop()
        for name in urdf.children.get(parent, ()):
            joint = urdf.joints[name]
            rotation = joint['rot']
            if joint['type'] == 'revolute':
                rotation = rotation @ Rotation.from_rotvec(joint['axis'] * values.get(name, 0.)).as_matrix()
            transforms[joint['child']] = transforms[parent] @ pose(joint['xyz'], rotation)
            pending.append(joint['child'])
    return transforms


def fixed_assembly_vertices_tcp(assets):
    """All link6-fixed collision vertices in suction_contact_frame at q=0.

    Cached per asset revision; the projection the archived geometry.json was
    derived from, recomputed from the live source model in every process.
    """
    urdf = assets['urdf']
    key = digest(assets['hashes'])
    cached = _ASSEMBLY_CACHE.get(key)
    if cached is not None:
        return cached
    from assets import origin
    from full_arm import mesh_from_geometry
    transforms = link_transforms(urdf, [0.] * 6)
    tcp_inverse = np.linalg.inv(transforms['suction_contact_frame'])
    fixed = {'elfin_link6'}
    while True:
        expanded = fixed | {j['child'] for j in urdf.joints.values()
                            if j['type'] == 'fixed' and j['parent'] in fixed}
        if expanded == fixed:
            break
        fixed = expanded
    vertices = []
    for link in urdf.root.findall('link'):
        name = link.get('name')
        if name not in fixed:
            continue
        relative = tcp_inverse @ transforms[name]
        for collision in link.findall('collision'):
            transform = relative @ origin(collision)
            mesh = mesh_from_geometry(collision.find('geometry'), assets['repo'])
            points = np.asarray(mesh.vertices) @ transform[:3, :3].T + transform[:3, 3]
            vertices.append(points)
    if not vertices:
        raise ValueError('EMPTY_FIXED_ASSEMBLY')
    assembly = np.concatenate(vertices)
    _ASSEMBLY_CACHE[key] = assembly
    return assembly


def base_T_container(assets):
    """Container pose in elfin_base_link; recomputed, never archive-trusted."""
    urdf = assets['urdf']
    world_base = urdf.tcp([0.] * 6, urdf.path_to('elfin_base_link'))
    return np.linalg.inv(world_base) @ assets['world_container']


def joint_signature(element):
    return dict(type=element.get('type'),
                children={e.tag: dict(e.attrib) for e in element})


def rotation_distance(a, b):
    """Geodesic angle between two rotation matrices, radians."""
    return float((Rotation.from_matrix(np.asarray(a)).inv()
                  * Rotation.from_matrix(np.asarray(b))).magnitude())


@dataclass(frozen=True)
class PlacementGoal:
    goal_id: str
    final_box_pose: tuple  # 16-value row-major container SE3 of box centre


@dataclass(frozen=True)
class PriorQueryBudget:
    query_budget_s: float
    radius_m: float = .100
    radius_deg: float = 15.
    branches_per_stage: int = 8
    execution_budget_s: float | None = None
    support_area_ratio_min: float = .60
    support_com_margin_m: float = .020
    contact_tolerance_m: float = .001
    collision_margin_m: float = .005

    def __post_init__(self):
        for name in ('query_budget_s', 'radius_m', 'radius_deg'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError('INVALID_PRIOR_BUDGET:' + name)
        if (isinstance(self.branches_per_stage, bool) or not isinstance(self.branches_per_stage, int)
                or self.branches_per_stage < 1):
            raise ValueError('INVALID_PRIOR_BUDGET:branches_per_stage')
        if self.execution_budget_s is not None and self.execution_budget_s <= 0:
            raise ValueError('INVALID_PRIOR_BUDGET:execution_budget_s')

    @classmethod
    def from_config(cls, config, execution_policy=None):
        return cls(query_budget_s=config.prior_query_budget_s,
                   radius_m=config.prior_query_radius_m,
                   radius_deg=config.prior_query_radius_deg,
                   branches_per_stage=config.prior_branches_per_stage,
                   execution_budget_s=(getattr(execution_policy, 'sim_budget_s', None)
                                       if execution_policy is not None else None),
                   support_area_ratio_min=config.support_area_ratio_min,
                   support_com_margin_m=config.support_com_margin_m,
                   contact_tolerance_m=config.contact_tolerance_m,
                   collision_margin_m=config.collision_margin_m)

    @property
    def revision(self):
        return digest(asdict(self))


@dataclass(frozen=True)
class ModeHint:
    goal_id: str
    orientation_family: str  # down/side_a/up/side_b/tilted
    box_vertical_extent_m: float
    assembly_top_above_support_m: float
    required_gap_m: float
    available_gap_m: float | None
    within_ceiling_footprint: bool
    attachment_authority: str  # confirmed | archived_representative | none
    result: str  # GEOMETRY_WINDOW_OK | REJECT_NECESSARY_CEILING_COLLISION |
    #             EXACT_MESH_CHECK_REQUIRED | *_ARCHIVED_ATTACHMENT |
    #             NOT_EVALUATED_NO_ATTACHMENT
    witness: tuple  # (assembly_top_z, box_top_z, ceiling_z, support_z)


@dataclass(frozen=True)
class SeedSet:
    goal_id: str
    stage: str  # final | handoff | push_end | route
    target_container: tuple | None  # exact stage target, refined by consumers
    joints: tuple  # tuple of 6-tuples
    source: str
    distance_m: float | None
    distance_deg: float | None
    requires_refinement: bool = True  # approximate matches never certify


@dataclass(frozen=True)
class RouteTemplate:
    source_id: str
    stage_order: tuple
    waypoints: tuple  # ((stage, first_q, last_q), ...)
    final_box_pose: tuple
    handoff_box_pose: tuple
    handoff_tool_pose: tuple
    push_end_tool_pose: tuple
    duration_sim_s: float
    identity: tuple  # ((field, matches), ...)
    authority: str  # always seed_only
    admission: tuple  # ((check, admitted, reason, value), ...)


@dataclass(frozen=True)
class CandidateRank:
    goal_id: str
    rank: int
    basis: tuple


@dataclass(frozen=True)
class PriorHints:
    identity: tuple  # ((field, value), ...)
    effective: tuple  # ((param, value), ...)
    goal_poses: tuple  # ((goal_id, final_box_pose), ...)
    mode_hints: tuple
    seed_sets: tuple
    route_templates: tuple
    candidate_order: tuple
    diagnostics: tuple  # ((key, value), ...)

    def as_record(self):
        def plain(value):
            if isinstance(value, np.generic):
                return value.item()
            if isinstance(value, (tuple, list)):
                return [plain(v) for v in value]
            return value
        return dict(identity=dict(self.identity), effective=dict(self.effective),
                    goal_poses=[[g, p] for g, p in self.goal_poses],
                    mode_hints=[plain(asdict(m)) for m in self.mode_hints],
                    seed_sets=[plain(asdict(s)) for s in self.seed_sets],
                    route_templates=[plain(asdict(t)) for t in self.route_templates],
                    candidate_order=[plain(asdict(c)) for c in self.candidate_order],
                    diagnostics=dict(self.diagnostics))

    def goal_id_for(self, final_box_pose):
        pose = tuple(final_box_pose)
        for goal_id, candidate in self.goal_poses:
            if candidate == pose:
                return goal_id
        return None

    def diagnostics_dict(self):
        return dict(self.diagnostics)

    def joint_seeds_for(self, final_box_pose):
        """Flattened stage seeds for one action target, nearest families first."""
        goal_id = self.goal_id_for(final_box_pose)
        if goal_id is None:
            return ()
        order = {'final': 0, 'route': 1, 'handoff': 2, 'push_end': 3}
        rows = sorted((s for s in self.seed_sets if s.goal_id == goal_id),
                      key=lambda s: order.get(s.stage, 9))
        seeds = []
        for row in rows:
            for q in row.joints:
                if any(np.max(np.abs(np.array(q) - np.array(old))) < 1e-3 for old in seeds):
                    continue
                seeds.append(tuple(float(v) for v in q))
        return tuple(seeds)


def apply_prior_to_actions(actions, hints):
    """Order and pre-screen generated actions from prior mode hints.

    Only necessary collisions fully under the ceiling footprint with a
    confirmed attachment are dropped; every other action keeps its family and
    no candidate is invented, widened or resampled.
    """
    reject = {m.goal_id for m in hints.mode_hints
              if m.result == 'REJECT_NECESSARY_CEILING_COLLISION'
              and m.attachment_authority == 'confirmed'}
    rank = {c.goal_id: c.rank for c in hints.candidate_order}
    original = {a.action_id: index for index, a in enumerate(actions)}
    kept = []
    dropped = []
    for action in actions:
        goal_id = hints.goal_id_for(action.placement.final_box_pose)
        if goal_id in reject:
            dropped.append(dict(action_id=action.action_id, goal_id=goal_id,
                                result='REJECT_NECESSARY_CEILING_COLLISION'))
        else:
            kept.append(action)
    kept.sort(key=lambda a: (rank.get(hints.goal_id_for(a.placement.final_box_pose), len(rank)),
                             original[a.action_id]))
    return kept, dropped


def compose_seed_list(start_q, config, hints, action, random_seeds):
    """start_q first, prior seeds next, random fill; unchanged when disabled."""
    seeds = [list(start_q)]
    if hints is not None:
        for q in hints.joint_seeds_for(action.placement.final_box_pose):
            candidate = [float(v) for v in q]
            if any(np.max(np.abs(np.array(candidate) - np.array(old))) < 1e-3 for old in seeds):
                continue
            seeds.append(candidate)
            if len(seeds) >= config.ik_seed_count:
                return seeds
    seeds += random_seeds[:max(0, config.ik_seed_count - len(seeds))]
    return seeds


def _load_atlas_mode(path):
    cached = _ATLAS_CACHE.get(str(path))
    if cached is None:
        data = np.load(path, allow_pickle=False)
        cached = dict(points=np.asarray(data['points'], dtype=float),
                      pose_id=np.asarray(data['pose_id']),
                      spin=np.asarray(data['spin']),
                      joints=np.asarray(data['joints'], dtype=float),
                      jac_condition=np.asarray(data['jac_condition'], dtype=float))
        cached['tree'] = cKDTree(cached['points'])
        _ATLAS_CACHE[str(path)] = cached
    return cached


class UpperPlacementPrior:
    """Read-only prior bound to one asset revision; reusable within a process."""

    def __init__(self, assets, index_path=None):
        self._assets = assets
        self._urdf = assets['urdf']
        self._vertices = fixed_assembly_vertices_tcp(assets)
        self._base_container = base_T_container(assets)
        self._hull = assets['hull']
        self._limits = np.array(self._urdf.joint_limits())
        self._model_identity = digest(dict(urdf=digest(assets['xml']),
                                           assembly=hashlib.sha256(self._vertices.tobytes()).hexdigest(),
                                           hull=self._hull.geometry_hash,
                                           base_T_container=self._base_container.ravel().tolist()))
        self._index_path = Path(index_path).resolve() if index_path else None
        self._index = None

    # -- geometry, recomputed from the live source model every construction --

    def orientation_window(self, dimensions, attachment, clearance):
        """Necessary vertical fit per axis-aligned mode; archive-compatible."""
        dimensions = np.asarray(dimensions, dtype=float)
        attachment = matrix(attachment)
        windows = []
        for angle, label in ((0, 'down'), (90, 'side_a'), (180, 'up'), (270, 'side_b')):
            rotation = Rotation.from_euler('x', angle, degrees=True).as_matrix()
            half_height = np.abs(rotation[2]) @ dimensions / 2
            box_pose = np.eye(4)
            box_pose[:3, :3] = rotation
            box_pose[2, 3] = half_height
            tcp = box_pose @ attachment
            points = self._vertices @ tcp[:3, :3].T + tcp[:3, 3]
            required_gap = max(2 * half_height, points[:, 2].max()) + clearance
            windows.append(dict(orientation_family=label,
                                box_vertical_extent_m=float(2 * half_height),
                                assembly_top_above_support_m=float(points[:, 2].max()),
                                required_gap_m=float(required_gap),
                                maximum_support_z_container_m=float(self._hull.ceiling_z - required_gap)))
        return tuple(windows)

    def evaluate_goal(self, goal_box_pose, dimensions, attachment, clearance):
        """Pose-level window: exact ceiling necessity only inside the footprint."""
        goal = matrix(goal_box_pose)
        dimensions = np.asarray(dimensions, dtype=float)
        attachment = matrix(attachment)
        extent = float(np.abs(goal[2, :3]) @ dimensions)
        support_z = float(goal[2, 3] - extent / 2)
        box_top = float(goal[2, 3] + extent / 2)
        tcp = goal @ attachment
        points = self._vertices @ tcp[:3, :3].T + tcp[:3, 3]
        assembly_top = float(points[:, 2].max())
        half = np.array([self._hull.half_x, self._hull.half_y])
        corners = (np.array([[x, y] for x in (-1, 1) for y in (-1, 1)])
                   * dimensions[:2] / 2) @ goal[:2, :2].T + goal[:2, 3]
        within = bool(np.all(np.abs(points[:, :2]) <= half + 1e-12)
                      and np.all(np.abs(corners) <= half + 1e-12))
        required_gap = max(extent, assembly_top - support_z) + clearance
        available_gap = float(self._hull.ceiling_z - support_z)
        if not within:
            result = 'EXACT_MESH_CHECK_REQUIRED'
        elif required_gap > available_gap:
            result = 'REJECT_NECESSARY_CEILING_COLLISION'
        else:
            result = 'GEOMETRY_WINDOW_OK'
        return dict(orientation_family=self._family(goal, attachment),
                    within_ceiling_footprint=within, required_gap_m=float(required_gap),
                    available_gap_m=available_gap, result=result,
                    margin_m=float(available_gap - required_gap),
                    witness=(assembly_top, box_top, float(self._hull.ceiling_z), support_z))

    @staticmethod
    def _family(goal, attachment):
        normal = (goal @ attachment)[:3, :3] @ np.array([0., 0., 1.])
        for label, axis in (('up', (0., 0., 1.)), ('down', (0., 0., -1.)),
                            ('left', (-1., 0., 0.)), ('right', (1., 0., 0.))):
            if abs(float(normal @ np.array(axis)) - 1.) < 1e-9:
                return label
        return 'tilted'

    # -- archived index (catalogue + atlas), private read-only ---------------

    def _load_index(self):
        if self._index is not None:
            return self._index
        index = dict(status='MISSING', reason='no index path', records=(), atlases=(),
                     signature_ok=False, signature_detail=(), attachments=(),
                     archive_box_dimensions=None)
        path = self._index_path
        if path is not None:
            try:
                catalogue = json.loads((path / 'catalogue.json').read_text())
                atlas_index = json.loads((path / 'atlas_index.json').read_text())
                signature_ok, signature_detail = self._check_atlas_signature(atlas_index, path)
                orientation = path.parent / 'orientation_reachability_g12_01'
                atlases = []
                for row in atlas_index['atlases']:
                    artifact = Path(row['artifact'])
                    if not artifact.is_file():
                        artifact = orientation / (row['mode'] + '_atlas.npz')
                    atlases.append(dict(mode=row['mode'], path=artifact))
                try:
                    archive_dimensions = tuple(json.loads((path / 'geometry.json').read_text())
                                               ['box_dimensions'])
                except (OSError, ValueError, KeyError):
                    archive_dimensions = None
                records = tuple(catalogue['records'])
                attachments = tuple(sorted({tuple(r['box_T_tool']) for r in records
                                            if r.get('nominal_valid')}))
                index = dict(status='OK', reason='', records=records, atlases=tuple(atlases),
                             signature_ok=bool(signature_ok), signature_detail=signature_detail,
                             attachments=attachments, archive_box_dimensions=archive_dimensions)
            except (OSError, ValueError, KeyError, json.JSONDecodeError) as failure:
                index = dict(status='UNREADABLE', reason=str(failure), records=(), atlases=(),
                             signature_ok=False, signature_detail=(), attachments=(),
                             archive_box_dimensions=None)
        self._index = index
        return index

    def _check_atlas_signature(self, atlas_index, path):
        current = {j.get('name'): joint_signature(j)
                   for j in self._urdf.root.findall('joint')}
        detail = {}
        robot = path.parent / 'orientation_reachability_g12_01' / 'robot.urdf'
        if not robot.is_file():
            robot = Path(atlas_index['atlases'][0]['artifact']).parent / 'robot.urdf'
        if not robot.is_file():
            return None, detail
        old = {j.get('name'): joint_signature(j) for j in ET.parse(robot).getroot().findall('joint')}
        for name, signature in current.items():
            if name != 'world_base' and name in old:
                detail[name] = bool(signature == old[name])
        return all(detail.values()), detail

    def _atlas_seeds(self, tcp_base, budget, deadline):
        """Nearest archived IK branches by position + rotation distance."""
        index = self._load_index()
        if index['status'] != 'OK' or not index['signature_ok']:
            return ()
        target_position = np.asarray(tcp_base[:3, 3], dtype=float)
        target_rotation = np.asarray(tcp_base[:3, :3])
        radius = math.radians(budget.radius_deg)
        found = []
        for atlas in index['atlases']:
            if len(found) >= budget.branches_per_stage or time.monotonic() >= deadline:
                break
            try:
                data = _load_atlas_mode(atlas['path'])
            except (OSError, ValueError, KeyError):
                continue
            for spin in ATLAS_SPINS:
                if len(found) >= budget.branches_per_stage or time.monotonic() >= deadline:
                    break
                anchor = atlas_rotation(atlas['mode']) @ Rotation.from_euler(
                    'z', spin, degrees=True).as_matrix()
                angle = rotation_distance(anchor, target_rotation)
                if angle > radius:
                    continue
                ids = data['tree'].query_ball_point(target_position, budget.radius_m)
                if not ids:
                    continue
                ids.sort(key=lambda i: float(np.linalg.norm(data['points'][i] - target_position)))
                for point_id in ids:
                    rows = np.where((data['pose_id'] == point_id)
                                    & (data['spin'] == spin))[0]
                    rows = rows[np.argsort(data['jac_condition'][rows])]
                    for row in rows:
                        q = data['joints'][row]
                        if not (np.all(q >= self._limits[:, 0])
                                and np.all(q <= self._limits[:, 1])):
                            continue
                        found.append(dict(joints=tuple(float(v) for v in q),
                                          source='atlas:%s:%d' % (atlas['mode'], spin),
                                          distance_m=float(np.linalg.norm(
                                              data['points'][point_id] - target_position)),
                                          distance_deg=float(math.degrees(angle))))
                    if len(found) >= budget.branches_per_stage:
                        break
        found.sort(key=lambda r: (r['distance_m'], r['distance_deg']))
        return tuple(found[:budget.branches_per_stage])

    # -- public query ---------------------------------------------------------

    def query(self, request, goals, budget):
        """One request, one immutable hint set; per-goal fair budget slices."""
        started = time.monotonic()
        deadline = started + budget.query_budget_s
        index = self._load_index()
        goals = tuple(goals)
        per_goal = []
        mode_hints = []
        seed_sets = []
        templates = []
        slice_s = budget.query_budget_s / max(1, len(goals))
        carry = 0.
        for goal in goals:
            goal_started = time.monotonic()
            allowed = min(deadline, goal_started + slice_s + carry)
            status = 'queried'
            if goal_started >= deadline:
                status = 'budget_exhausted'
            hint = self._goal_mode_hint(request, goal, budget)
            mode_hints.append(hint)
            matches = self._record_matches(request, goal, budget)
            for record, distance_m, distance_deg in matches:
                templates.append(self._route_template(request, record,
                                                      distance_m, distance_deg, budget))
                route_joints = [tuple(stage['first_q']) for stage in record['stages']]
                route_joints.append(tuple(record['stages'][-1]['last_q']))
                seed_sets.append(SeedSet(goal.goal_id, 'route', None, tuple(route_joints),
                                         'catalogue:' + record['id'], distance_m, distance_deg))
            if goal_started < deadline:
                authority = 'confirmed' if request.confirmed_box_T_tool is not None \
                    else 'archived_attachment'
                for stage, pose in self._stage_targets(request, goal, matches):
                    if time.monotonic() >= allowed:
                        status = 'budget_exhausted'
                        break
                    tcp_base = self._base_container @ matrix(pose)
                    for row in self._atlas_seeds(tcp_base, budget, allowed):
                        seed_sets.append(SeedSet(goal.goal_id, stage, tuple(pose.ravel().tolist()),
                                                 (row['joints'],), row['source'],
                                                 row['distance_m'], row['distance_deg']))
            spent = time.monotonic() - goal_started
            carry = max(0., slice_s - spent)
            per_goal.append((goal.goal_id, status, float(time.monotonic() - goal_started)))
        seed_sets = self._merge_seed_sets(seed_sets)
        candidate_order = self._rank_goals(goals, mode_hints, seed_sets, templates)
        actionable = (any(m.result in ('REJECT_NECESSARY_CEILING_COLLISION', 'GEOMETRY_WINDOW_OK',
                                       'GEOMETRY_WINDOW_OK_ARCHIVED_ATTACHMENT')
                          for m in mode_hints) or bool(seed_sets) or bool(templates))
        identity = (('request_revision', request.revision),
                    ('snapshot_id', request.snapshot.snapshot_id),
                    ('geometry_hash', request.snapshot.geometry_hash),
                    ('frame_id', request.frame_id),
                    ('box_dimensions', tuple(request.box.dimensions)),
                    ('box_mass_kg', request.box.mass_kg),
                    ('box_com_local', tuple(request.box.com_local)),
                    ('start_q', tuple(request.start_q)),
                    ('confirmed_box_T_tool', (tuple(request.confirmed_box_T_tool)
                                              if request.confirmed_box_T_tool is not None else None)),
                    ('start_attached', request.start_attached),
                    ('model_identity', self._model_identity),
                    ('budget_revision', budget.revision))
        effective = (('query_budget_s', budget.query_budget_s),
                     ('radius_m', budget.radius_m), ('radius_deg', budget.radius_deg),
                     ('branches_per_stage', budget.branches_per_stage),
                     ('execution_budget_s', budget.execution_budget_s),
                     ('index_path', str(self._index_path) if self._index_path else None),
                     ('index_status', index['status']),
                     ('atlas_signature_ok', index['signature_ok']))
        diagnostics = (('index_status', index['status']), ('index_reason', index['reason']),
                       ('atlas_signature_ok', index['signature_ok']),
                       ('per_goal', per_goal),
                       ('goal_count', len(goals)),
                       ('seed_rows', len(seed_sets)), ('template_rows', len(templates)),
                       ('budget_exhausted', any(row[1] == 'budget_exhausted' for row in per_goal)),
                       ('fallback_required', not actionable))
        return PriorHints(identity=identity, effective=effective,
                          goal_poses=tuple((g.goal_id, tuple(g.final_box_pose)) for g in goals),
                          mode_hints=tuple(mode_hints), seed_sets=tuple(seed_sets),
                          route_templates=tuple(templates),
                          candidate_order=tuple(candidate_order), diagnostics=diagnostics)

    def _attachments_for(self, request):
        if request.confirmed_box_T_tool is not None:
            return (matrix(request.confirmed_box_T_tool),), 'confirmed'
        index = self._load_index()
        if index['attachments']:
            return tuple(matrix(a) for a in index['attachments']), 'archived_representative'
        return (), 'none'

    def _goal_mode_hint(self, request, goal, budget):
        attachments, authority = self._attachments_for(request)
        if not attachments:
            return ModeHint(goal.goal_id, 'tilted', 0., 0., 0., None, False, 'none',
                            'NOT_EVALUATED_NO_ATTACHMENT', (0., 0., float(self._hull.ceiling_z), 0.))
        evaluation = None
        for attachment in attachments:
            current = self.evaluate_goal(goal.final_box_pose, request.box.dimensions,
                                         attachment, budget.collision_margin_m)
            if evaluation is None or current['required_gap_m'] > evaluation['required_gap_m']:
                evaluation = current
        result = evaluation['result']
        if authority == 'archived_representative':
            result = {'GEOMETRY_WINDOW_OK': 'GEOMETRY_WINDOW_OK_ARCHIVED_ATTACHMENT',
                      'REJECT_NECESSARY_CEILING_COLLISION': 'APPROXIMATE_ARCHIVED_ATTACHMENT'
                      }.get(result, result)
        witness = evaluation['witness']
        extent = float(np.abs(matrix(goal.final_box_pose)[2, :3]) @ np.array(request.box.dimensions))
        return ModeHint(goal.goal_id, evaluation['orientation_family'], extent,
                        float(witness[0] - witness[3]), evaluation['required_gap_m'],
                        evaluation['available_gap_m'], evaluation['within_ceiling_footprint'],
                        authority, result, witness)

    def _record_matches(self, request, goal, budget):
        index = self._load_index()
        if index['status'] != 'OK':
            return ()
        target = matrix(goal.final_box_pose)
        matches = []
        for record in index['records']:
            if not record.get('nominal_valid'):
                continue
            final = matrix(record['poses_container_absolute']['final_box'])
            distance_m = float(np.linalg.norm(final[:3, 3] - target[:3, 3]))
            distance_deg = float(math.degrees(rotation_distance(final[:3, :3], target[:3, :3])))
            if distance_m <= budget.radius_m and distance_deg <= budget.radius_deg:
                matches.append((record, distance_m, distance_deg))
        matches.sort(key=lambda row: (row[1], row[2]))
        return tuple(matches)

    def _stage_targets(self, request, goal, matches):
        """Final target from the request itself; handoff/push from matched routes."""
        attachments, _ = self._attachments_for(request)
        if not attachments:
            return ()
        targets = [('final', matrix(goal.final_box_pose) @ attachments[0])]
        for record, _, _ in matches[:1]:
            poses = record['poses_container_absolute']
            targets.append(('handoff', matrix(poses['handoff_tool_pose'])))
            targets.append(('push_end', matrix(poses['push_end_tool_pose'])))
        return tuple(targets)

    def _route_template(self, request, record, distance_m, distance_deg, budget):
        poses = record['poses_container_absolute']
        index = self._load_index()
        identity = (('snapshot_id', record.get('snapshot_id') == request.snapshot.snapshot_id),
                    ('box_dimensions', (index['archive_box_dimensions'] is not None
                                        and tuple(index['archive_box_dimensions'])
                                        == tuple(request.box.dimensions))),
                    ('attachment', request.confirmed_box_T_tool is not None
                     and tuple(record['box_T_tool']) == tuple(request.confirmed_box_T_tool)),
                    ('config', False))
        admission = (self._support_admission(request, record, budget),
                     self._duration_admission(record, budget))
        waypoints = tuple((stage['name'], tuple(stage['first_q']), tuple(stage['last_q']))
                          for stage in record['stages'])
        return RouteTemplate(source_id=record['id'],
                             stage_order=tuple(stage['name'] for stage in record['stages']),
                             waypoints=waypoints,
                             final_box_pose=tuple(poses['final_box']),
                             handoff_box_pose=tuple(poses['handoff_box_pose']),
                             handoff_tool_pose=tuple(poses['handoff_tool_pose']),
                             push_end_tool_pose=tuple(poses['push_end_tool_pose']),
                             duration_sim_s=float(record['duration_sim_s']),
                             identity=tuple(identity), authority='seed_only',
                             admission=admission)

    def _support_admission(self, request, record, budget):
        """Recompute the archived handoff support against this request."""
        handoff = matrix(record['poses_container_absolute']['handoff_box_pose'])
        dimensions = np.array(request.box.dimensions)
        bottom = float(handoff[2, 3] - dimensions[2] / 2)
        patches = tuple(patch for patch in request.snapshot.surfaces
                        if patch.source_id in request.support_source_ids
                        and abs(patch.surface_z_container - bottom) <= budget.contact_tolerance_m)
        probe = types.SimpleNamespace(support_area_ratio_min=budget.support_area_ratio_min,
                                      support_com_margin_m=budget.support_com_margin_m,
                                      contact_tolerance_m=budget.contact_tolerance_m)
        support = measure_support(handoff, dimensions, patches, probe, request.box.com_local)
        return ('handoff_support', bool(support.accepted), support.reason,
                float(support.com_margin_m))

    @staticmethod
    def _duration_admission(record, budget):
        duration = float(record['duration_sim_s'])
        if budget.execution_budget_s is None:
            return ('execution_duration', None, 'NOT_EVALUATED_NO_BUDGET', duration)
        admitted = bool(duration <= budget.execution_budget_s)
        return ('execution_duration', admitted,
                'EXECUTION_DURATION_WITHIN_BUDGET' if admitted
                else 'EXECUTION_DURATION_EXCEEDS_BUDGET', duration)

    def _merge_seed_sets(self, seed_sets):
        merged = {}
        for row in seed_sets:
            key = (row.goal_id, row.stage, row.source, row.target_container)
            merged.setdefault(key, []).append(row)
        result = []
        for (goal_id, stage, source, target), rows in merged.items():
            joints = []
            for row in rows:
                for q in row.joints:
                    if any(np.max(np.abs(np.array(q) - np.array(old))) < 1e-3 for old in joints):
                        continue
                    joints.append(tuple(q))
            result.append(SeedSet(goal_id, stage, target, tuple(joints), source,
                                  rows[0].distance_m, rows[0].distance_deg))
        return tuple(result)

    def _rank_goals(self, goals, mode_hints, seed_sets, templates):
        hint_by_goal = {m.goal_id: m for m in mode_hints}
        rows = []
        for order, goal in enumerate(goals):
            hint = hint_by_goal.get(goal.goal_id)
            if hint is None:
                rows.append((2, 0., 1e9, 1, order, goal.goal_id))
                continue
            reject = hint.result == 'REJECT_NECESSARY_CEILING_COLLISION'
            margin = (hint.available_gap_m - hint.required_gap_m
                      if hint.available_gap_m is not None else -1e9)
            target = matrix(goal.final_box_pose)
            template_distance = min((float(np.linalg.norm(matrix(t.final_box_pose)[:3, 3]
                                                          - target[:3, 3]))
                                     for t in templates), default=1e9)
            has_seeds = any(s.goal_id == goal.goal_id for s in seed_sets)
            rows.append((1 if reject else 0, -margin, template_distance,
                         0 if has_seeds else 1, order, goal.goal_id))
        rows.sort()
        return tuple(CandidateRank(goal_id, rank,
                                   ('mode_hint', 'margin_m', 'template_distance_m', 'seeds'))
                     for rank, (*_, goal_id) in enumerate(rows))
