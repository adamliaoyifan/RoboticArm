"""Continuous noncontact clearance of the free-box pose envelope.

Declared support/panel contact pairs are reported separately as NOT EVALUATED
for uncertainty. A pass here cannot admit execution or certify the whole tube.
"""
import time
import numpy as np
import trimesh
from payload_envelope import separation_lower_bound
from plan_clearance import FullArmDistanceSampler, _push_motion, plan_digest, model_identity


def bound_envelope_path(path, evaluate, motion_bound, margin_m, deadline):
    """Prove a hard margin using interval motion bounds, without a ranking score.

    evaluate returns (distance rows, failure). An insufficient pointwise bound
    is inconclusive, never a claimed geometric collision. Deadline/failure
    cannot yield a partial pass.
    """
    path = np.asarray(path, dtype=float)
    if (path.ndim != 2 or len(path) < 2 or not np.isfinite(path).all() or
            not np.isfinite(margin_m) or margin_m <= 0 or not np.isfinite(deadline)):
        raise ValueError('INVALID_ENVELOPE_PATH_OR_POLICY')
    checks = leaves = 0
    minimum = float('inf'); witness = None
    def reject(reason, **details):
        return dict(noncontact_clearance_certified=False, reason=reason,
                    checks=checks, certified_intervals=leaves, **details)
    for segment, (qa, qb) in enumerate(zip(path, path[1:])):
        pending = [(0., 1.)]
        while pending:
            if time.monotonic() >= deadline:
                return reject('ENVELOPE_DEADLINE')
            low, high = pending.pop(); mid = (low + high) / 2
            rows, failure = evaluate(segment, mid, (1 - mid) * qa + mid * qb)
            checks += 1
            if time.monotonic() >= deadline:
                return reject('ENVELOPE_DEADLINE')
            if failure:
                return reject('ENVELOPE_NOMINAL_STATE_REJECTED', diagnostic=failure)
            if not rows:
                return reject('ENVELOPE_MISSING_DISTANCE_EVIDENCE')
            delta = np.abs(qb - qa) * (high - low) / 2
            interval_minimum = float('inf'); interval_pair = None
            for distance, pair in rows:
                bound = float(motion_bound(pair, delta, segment, low, high))
                if not np.isfinite(distance) or not np.isfinite(bound) or bound < 0:
                    return reject('INVALID_ENVELOPE_DISTANCE_BOUND', pair=pair)
                if distance < margin_m:
                    return reject('ENVELOPE_SAMPLE_UNPROVED', segment=segment, fraction=mid,
                                  pair=pair, lower_m=float(distance), required_m=margin_m)
                if distance - bound < interval_minimum:
                    interval_minimum = distance - bound; interval_pair = pair
            if interval_minimum >= margin_m:
                leaves += 1
                if interval_minimum < minimum:
                    minimum = float(interval_minimum)
                    witness = dict(segment=segment, fraction_bounds=[low, high], pair=interval_pair)
            elif mid == low or mid == high:
                return reject('ENVELOPE_INTERVAL_UNPROVED', segment=segment)
            else:
                pending.extend(((mid, high), (low, mid)))
    return dict(noncontact_clearance_certified=True, lower_m=minimum, witness=witness,
                checks=checks, certified_intervals=leaves)


class PayloadEnvelopeSampler:
    def __init__(self, scene, numeric_error_m, translation_m, angle_rad, box_motion=None,
                 transverse_to=None):
        if (not np.isfinite(translation_m) or translation_m < 0 or
                not np.isfinite(angle_rad) or not 0. <= angle_rad <= np.pi):
            raise ValueError('INVALID_PAYLOAD_ENVELOPE_ERROR_BOUNDS')
        if transverse_to is not None:
            direction = np.asarray(transverse_to, dtype=float)
            if (direction.shape != (3,) or not np.isfinite(direction).all() or
                    not np.isclose(np.linalg.norm(direction), 1., atol=1e-12, rtol=0)):
                raise ValueError('INVALID_TRANSVERSE_DIRECTION')
            transverse_to = direction.copy()
            transverse_to.flags.writeable = False
        self.nominal = FullArmDistanceSampler(scene, numeric_error_m, box_motion)
        self.scene = scene; self.numeric_error_m = numeric_error_m
        self.translation_m = translation_m; self.angle_rad = angle_rad
        self.transverse_to = transverse_to
        self.excluded_pairs = set()

    def evaluate(self, segment, fraction, q, contact_mode):
        sample = self.nominal.evaluate(segment, fraction, q, contact_mode)
        if sample.failure:
            return (), sample.failure
        try:
            return self.enclose_current_distances(sample.distances, contact_mode), None
        except ValueError as error:
            return (), dict(reason='ENVELOPE_PAIR_INVENTORY_INVALID', diagnostic=str(error))

    def enclose_current_distances(self, nominal_distances, contact_mode=False):
        """Bound an already checked source state, without another FK/FCL query.

        All supplied nominal distances must already include numeric enclosure.
        Used both by validation and fixed-free-box motion search.
        """
        scene = self.scene; pose = scene.payload_transform(); size = scene.payload['size']
        radius = float(np.linalg.norm(size) / 2)
        displacement = self.translation_m + 2 * radius * np.sin(self.angle_rad / 2)
        distances = {}
        for distance, pair, _ in nominal_distances:
            if 'payload' in pair:
                key = tuple(pair)
                distances[key] = max(distances.get(key, -float('inf')), distance)
        checked_names = {next(name for name in pair if name != 'payload') for pair in distances}
        # Explicit inventory prevents an unreported object-level omission from
        # being mistaken for a whole-tube certificate.
        for name in set(scene.robot_parts) | set(scene.world_parts):
            if name not in checked_names:
                support = (scene.payload['allow_support'] and
                           (name.startswith('lower_') or name.startswith('pickup_platform:')))
                panel = (scene.contact_mode(contact_mode) is not None and
                         name == scene.push_geometry.collision_key)
                if not support and not panel:
                    raise ValueError('UNDECLARED_NONCONTACT_PAIR_MISSING:' + name)
                self.excluded_pairs.add(('payload', name))
        rows = []
        for pair, nominal_lower in distances.items():
            lower = nominal_lower - displacement
            if lower < scene.config.collision_margin_m + 2 * scene.config.cartesian_step_m:
                other = next(name for name in pair if name != 'payload')
                if other in scene.robot_parts:
                    link, _, local = scene.robot_parts[other]
                    transform = scene.transforms[link] @ local
                    corners = scene.robot_bounds[other]
                else:
                    mesh, transform = scene.world_parts[other]
                    corners = trimesh.bounds.corners(mesh.bounds)
                vertices = corners @ transform[:3, :3].T + transform[:3, 3]
                axes = np.vstack((np.eye(3), pose[:3, :3].T, transform[:3, :3].T))
                projection = separation_lower_bound(pose, size, vertices, axes,
                    self.translation_m, self.angle_rad, self.transverse_to)
                lower = max(lower, projection['lower_m'] - self.numeric_error_m)
            rows.append((float(lower), pair))
        return tuple(rows)


def certify_payload_noncontact_tube(scene, state, plan, config, selection_policy,
                                   execution_policy, deadline, stages=None, emit=None):
    """Certify every requested released-box stage's noncontact envelope.

    Push translation error is perpendicular to its reference direction. Other
    stages use a full 3D translation ball. SO(3) error includes roll/pitch/yaw.
    The deliberately stronger 5mm ball outside push may fail conservatively;
    failure does not license reducing the requested domain.
    """
    free_stages = ('release_down', 'withdraw', 'push_pre', 'push_contact', 'push', 'retreat')
    requested = tuple(free_stages if stages is None else stages)
    if not requested or len(set(requested)) != len(requested) or set(requested) - set(free_stages):
        raise ValueError('INVALID_FREE_BOX_TUBE_STAGES')
    found = {s['name']: s for s in plan['stages']}
    if len(found) != len(plan['stages']):
        raise ValueError('DUPLICATE_FREE_BOX_TUBE_STAGE')
    if set(requested) - set(found):
        raise ValueError('MISSING_FREE_BOX_TUBE_STAGE')
    action = plan['action']; results = []
    if (action['placement']['snapshot_id'] != scene.snapshot.snapshot_id or
            action['placement']['config_revision'] != config.revision or config != scene.config):
        raise ValueError('FREE_BOX_TUBE_INPUT_IDENTITY_MISMATCH')
    for name in requested:
        stage = found[name]; trajectory = stage['trajectory']
        if (stage['payload_mode'] != 'world' or
                trajectory.get('interpolation') != 'cubic_zero_velocity_knots' or
                not np.allclose(trajectory['velocities'], 0., atol=0, rtol=0)):
            raise ValueError('UNSUPPORTED_FREE_BOX_TUBE_TRAJECTORY')
        box = action['placement']['final_box_pose'] if name == 'retreat' else action['handoff_box_pose']
        scene.set_payload(state.dimensions, 'world',
            scene.world_container @ np.array(box).reshape(4, 4),
            np.array(action['box_T_tool']).reshape(4, 4), allow_support=True)
        direction = scene.world_container[:3, :3] @ np.asarray(action['push_direction']) if name == 'push' else None
        sampler = PayloadEnvelopeSampler(scene, selection_policy.distance_numeric_error_m,
            execution_policy.push_lateral_error_m, np.deg2rad(execution_policy.push_rotation_error_deg),
            _push_motion(scene, stage, action) if name == 'push' else None, direction)
        if emit:
            emit(dict(event='payload_tube_stage_begin', stage=name))
        result = bound_envelope_path(trajectory['positions'],
            lambda i, u, q: sampler.evaluate(i, u, q, stage['contact_tool']),
            sampler.nominal.motion_bound, config.collision_margin_m, deadline)
        results.append(dict(stage=name, **result,
            translation_model='perpendicular_ball' if name == 'push' else 'isotropic_ball',
            transverse_direction_world=direction.tolist() if direction is not None else None,
            intended_contact_uncertainty='not_evaluated',
            excluded_pairs=[list(p) for p in sorted(sampler.excluded_pairs)]))
        if emit:
            emit(dict(event='payload_tube_stage_end', **results[-1]))
        if not result['noncontact_clearance_certified']:
            break
    return dict(noncontact_clearance_certified=len(results) == len(requested) and
                all(r['noncontact_clearance_certified'] for r in results), stages=results,
                requested_stages=list(requested), plan_digest=plan_digest(plan),
                snapshot_id=scene.snapshot.snapshot_id, config_revision=config.revision,
                execution_policy_revision=execution_policy.revision,
                clearance_policy_revision=selection_policy.clearance_revision,
                distance_numeric_error_m=selection_policy.distance_numeric_error_m,
                model_identity=model_identity(scene), translation_error_m=execution_policy.push_lateral_error_m,
                rotation_error_deg=execution_policy.push_rotation_error_deg,
                error_frame='world', rotation_model='SO3_geodesic_ball_about_geometric_box_centre',
                intended_contact_uncertainty='not_evaluated', execution_admitted=False,
                scope='continuous_noncontact_envelope_only_not_whole_tube_or_execution_certificate')
