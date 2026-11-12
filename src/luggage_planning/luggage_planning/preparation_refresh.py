"""Latest measured O certification of one retained whole-operation trajectory.

Read-only callbacks own leases, observation admission and full baseline reads.
This finalizer never searches, changes geometry, retimes, or writes a map/scene.
"""
from dataclasses import dataclass, replace
import hashlib
import math
import time

from luggage_mapping.source import OperationMapSource
from luggage_packing.placement_recheck import recheck_placement
from luggage_planning.joint_operation_contracts import seal_operation_plan
from luggage_planning.joint_operation_planner import certificate_matches
from luggage_planning.joint_operation_scenes import build_operation_scenes
from luggage_planning.placement_prior_snapshot import proposal_scene_hash
from luggage_planning.preparation_profile import StageTimings


@dataclass(frozen=True)
class RefreshCapture:
    surface: object
    volume: object
    source: OperationMapSource
    baseline: object
    snapshot: object


@dataclass(frozen=True)
class FinalizedPreparation:
    plan: object
    map_source: object
    reason: str
    trace: tuple = ()


def require_source_progress(original, current):
    """Same owner epoch/config/calibration and nondecreasing measured revision."""
    if type(original) is not OperationMapSource or type(current) is not OperationMapSource:
        raise ValueError('REFRESH_BOUND_OBSERVER_SOURCE_REQUIRED')
    fixed = ('scene_revision', 'geometry_hash', 'epoch', 'config_digest',
             'clock_domain', 'validity_generation')
    if any(getattr(original, key) != getattr(current, key) for key in fixed):
        raise ValueError('REFRESH_SOURCE_RESET_OR_CONFIGURATION_CHANGED')
    if any(getattr(current, key) < getattr(original, key) for key in
           ('map_revision', 'observed_revision', 'last_integrated_stamp_ns')):
        raise ValueError('REFRESH_SOURCE_BACKWARDS')
    # O increments observed_revision on each integration, but map_revision only
    # on changed keys. A new observation can therefore retain map_revision.
    if ((current.map_revision, current.observed_revision, current.last_integrated_stamp_ns) ==
            (original.map_revision, original.observed_revision, original.last_integrated_stamp_ns)
            and current.content_sha256 != original.content_sha256):
        raise ValueError('REFRESH_SOURCE_REVISION_CONTENT_MISMATCH')


class LatestMapFinalizer:
    """Bounded final certification within the original absolute deadline.

    Attempts is an operator-configured retry budget (default two), not a
    geometric or freshness tolerance. A moving source may yield no plan.
    """
    def __init__(self, *, original_surface, original_source, original_snapshot,
                 baseline_hash, capture, read_baseline, source_current,
                 observation_current, certification, attempts=2, clock=time.monotonic):
        if type(attempts) is not int or not 1 <= attempts <= 8:
            raise ValueError('BOUNDED_REFRESH_ATTEMPTS_REQUIRED')
        self.surface, self.source, self.snapshot = original_surface, original_source, original_snapshot
        self.baseline_hash = baseline_hash
        self.capture, self.read_baseline = capture, read_baseline
        self.source_current, self.observation_current = source_current, observation_current
        self.certification, self.attempts, self.clock = certification, attempts, clock

    def finalize(self, plan, candidate, inputs, deadline, *, cancelled=lambda:False):
        trace = []
        timing = StageTimings('latest_map', trace, clock=self.clock, unit=plan.candidate_id)
        def stopped():
            return cancelled() or not math.isfinite(deadline) or self.clock() >= deadline
        def finish(reason, final=None, source=None):
            return FinalizedPreparation(final, source, reason, tuple(trace))
        def observation():
            return timing.call('frozen_observation_and_start', self.observation_current, deadline)
        try:
            if stopped():return finish('REFRESH_CANCELLED_OR_DEADLINE')
            expected_snapshot = hashlib.sha256(repr(self.snapshot.identity).encode()).hexdigest()
            if (inputs.snapshot.identity != self.snapshot.identity
                    or plan.candidate_id != candidate.candidate_id or candidate.scenes.base_identity != self.snapshot.identity
                    or plan.identity.snapshot_id != expected_snapshot
                    or (plan.identity.scene_revision, plan.identity.geometry_hash, plan.identity.map_revision,
                        plan.identity.model_sha256) != (self.source.scene_revision, self.source.geometry_hash,
                                                       self.source.map_revision, self.snapshot.model_hash)):
                return finish('REFRESH_ORIGINAL_IDENTITY_MISMATCH')
            if not observation():return finish('REFRESH_OBSERVATION_OR_START_CHANGED')
            scene = timing.call('baseline_read', self.read_baseline, deadline)
            if not observation():return finish('REFRESH_OBSERVATION_OR_START_CHANGED')
            if (timing.call('source_current', self.source_current, self.source, deadline)
                    and proposal_scene_hash(scene) == self.baseline_hash):
                if stopped():return finish('REFRESH_CANCELLED_OR_DEADLINE')
                trace.append(('map_finalization', 'ORIGINAL_SOURCE_CURRENT', self.source.map_revision))
                return finish('PLAN_READY', plan, self.source)
            previous = self.source
            for attempt in range(1, self.attempts + 1):
                if stopped():return finish('REFRESH_CANCELLED_OR_DEADLINE')
                fresh = timing.call('complete_capture', self.capture, deadline)
                require_source_progress(previous, fresh.source)
                snapshot, record = fresh.snapshot, fresh.surface.record()
                if (snapshot.world_from_container != self.snapshot.world_from_container
                        or snapshot.model_hash != self.snapshot.model_hash
                        or (record['scene_revision'], record['geometry_hash'], record['map_revision']) !=
                           (fresh.source.scene_revision, fresh.source.geometry_hash, fresh.source.map_revision)
                        or (snapshot.scene_revision, snapshot.geometry_hash, snapshot.map_revision) !=
                           (fresh.source.scene_revision, fresh.source.geometry_hash, fresh.source.map_revision)
                        or fresh.volume is None or fresh.volume.source != fresh.source
                        or snapshot.surface_content_hash != hashlib.sha256(fresh.surface.record_json).hexdigest()):
                    return finish('REFRESH_CAPTURE_IDENTITY_MISMATCH')
                placed = snapshot.world_from_container.inverse().compose(plan.world_from_settled_box)
                qx, qy, qz, qw = placed.quaternion_xyzw
                if abs(qx) > 1e-6 or abs(qy) > 1e-6:
                    return finish('REFRESH_PLACEMENT_GEOMETRY_INVALID')
                reason = timing.call('placement_proof', recheck_placement, self.surface.record(), record,
                    fresh.volume, center_container=placed.translation,
                    box_size=plan.predicted_attachment.measured_dimensions_wdh,
                    yaw_rad=2 * math.atan2(qz, qw))
                if reason:return finish(reason)
                if stopped():return finish('REFRESH_CANCELLED_OR_DEADLINE')
                scenes = timing.call('phase_scenes', build_operation_scenes, snapshot,
                    pickup_object_id=candidate.scenes.pickup_object_id, geometry=plan.predicted_attachment,
                    world_from_pick_box=plan.world_from_pick_box,
                    world_from_settled_box=plan.world_from_settled_box)
                refreshed = replace(candidate, scenes=scenes)
                identity = replace(plan.identity, map_revision=snapshot.map_revision,
                    snapshot_id=hashlib.sha256(repr(snapshot.identity).encode()).hexdigest())
                draft = seal_operation_plan(replace(plan, identity=identity,
                    certificate_id='pending-native-certification', plan_digest=''))
                trace.append(('map_recertification', attempt, previous.map_revision,
                              fresh.source.map_revision, scenes.digests, draft.plan_digest))
                receipt = timing.call('whole_chain_certification', self.certification.certify,
                    draft, refreshed, deadline=deadline, cancelled=cancelled)
                if not certificate_matches(receipt, draft, refreshed):
                    return finish('REFRESH_NATIVE_CERTIFICATE_REFUSED')
                certified = seal_operation_plan(replace(draft, certificate_id=receipt.certificate_id, plan_digest=''))
                # Recheck after asynchronous certification and observation calls.
                if not observation():return finish('REFRESH_OBSERVATION_OR_START_CHANGED')
                scene = timing.call('final_baseline_read', self.read_baseline, deadline)
                if not observation():return finish('REFRESH_OBSERVATION_OR_START_CHANGED')
                current = timing.call('final_source_current', self.source_current, fresh.source, deadline)
                if stopped():return finish('REFRESH_CANCELLED_OR_DEADLINE')
                if current and proposal_scene_hash(scene) == proposal_scene_hash(fresh.baseline):
                    return finish('PLAN_READY', certified, fresh.source)
                previous = fresh.source
            return finish('REFRESH_SOURCE_UNSTABLE')
        except (TypeError, ValueError, RuntimeError, TimeoutError) as error:
            trace.append(('map_finalization_error', type(error).__name__, str(error)))
            return finish('REFRESH_REFUSED:' + str(error))
