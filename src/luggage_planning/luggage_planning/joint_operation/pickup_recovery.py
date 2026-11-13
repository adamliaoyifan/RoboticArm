"""Node-layer pre-pick workflow shared by observed cycle adapters.

Only owner-issued local observation needs can cause extra empty-view motion.
Ports own actual transports, raw acquisition and complete-plan preparation;
this workflow never creates observed support or executes a loading trajectory.
"""
from dataclasses import asdict, dataclass
import json
import math
import time
import uuid

from luggage_perception.stationary_pickup_association import StationaryPickupPolicy
from luggage_planning.joint_operation.view_proposals import PickupViewSampling


@dataclass(frozen=True)
class PickupPreparationAttempt:
    request: object
    response: object
    detection: object
    domain: object

    def result(self):
        if not self.response.success:
            raise RuntimeError('PREPARE:'+self.response.reason)
        return self.response.plan, self.detection, self.domain

    def observation_probes(self):
        if self.response.success or self.response.reason != 'SEARCH_INCOMPLETE':
            return ()
        return tuple(probe for probe in self.response.unresolved_contacts
            if (probe.domain_sha256, probe.contact_model_hash, probe.placement.identity.request_id)
            == (self.domain.domain_sha256, self.domain.contact_model_hash, self.request.request_id)
            and probe.query_id and probe.unproven_cell_count > 0)


@dataclass(frozen=True)
class PickupRecoverySettings:
    max_extra_view_motions: int
    total_recovery_budget_s: float
    sampling: object
    association_policy: object
    information_sample_shape: tuple
    max_ik_seeds: int

    def __post_init__(self):
        if (type(self.max_extra_view_motions) is not int
                or not 0 <= self.max_extra_view_motions < 2**32
                or not math.isfinite(self.total_recovery_budget_s)
                or self.total_recovery_budget_s < 0):
            raise ValueError('EXPLICIT_RECOVERY_LIMITS_REQUIRED')
        if not self.max_extra_view_motions:
            return
        shape = self.information_sample_shape
        if (self.total_recovery_budget_s <= 0 or type(self.sampling) is not PickupViewSampling
                or type(self.association_policy) is not StationaryPickupPolicy
                or len(shape) != 2 or any(type(v) is not int or v < 1 for v in shape)
                or math.prod(shape) > 4096
                or type(self.max_ik_seeds) is not int or not 1 <= self.max_ik_seeds <= 64):
            raise ValueError('EXPLICIT_RECOVERY_SAMPLING_AND_ASSOCIATION_REQUIRED')

    @classmethod
    def from_record(cls, record):
        if record.get('schema') != 'pickup_view_recovery_v1':
            raise ValueError('PICKUP_RECOVERY_CONFIG_SCHEMA_REQUIRED')
        count = record['max_extra_view_motions']
        if count == 0:
            return cls(count, record.get('total_recovery_budget_s', 0.), None, None, (), 0)
        return cls(count, record['total_recovery_budget_s'], PickupViewSampling(**record['sampling']),
            StationaryPickupPolicy(**record['association_policy']),
            tuple(record['information_sample_shape']), record['max_ik_seeds'])


@dataclass(frozen=True)
class PickupRecoveryPorts:
    # Each port must use this same absolute monotonic deadline. A late result
    # cannot extend the task or authorize a subsequent effect.
    execute_view: object
    observe_after: object
    continue_after: object
    prepare_current: object


class PickupPreparationRecovery:
    def __init__(self, settings, ports, *, record, clock=time.monotonic, request_id=None):
        self.settings, self.ports = settings, ports
        self.record, self.clock = record, clock
        self.request_id = request_id or (lambda: 'pickup-recovery-'+uuid.uuid4().hex)

    def run(self, initial, *, pickup_task_id, deadline=None):
        from action_msgs.msg import GoalStatus
        from luggage_msgs.action import ExecutePickupViewV2
        from luggage_msgs.msg import PreparePickupViewRequestV2, StationaryPickupPolicyV2
        from luggage_msgs.srv import ReprepareAfterPickupViewV2
        if initial.response.success:
            return initial.result()
        settings = self.settings
        probes = initial.observation_probes()
        if settings is None or not settings.max_extra_view_motions or not probes:
            return initial.result()
        if not pickup_task_id.strip():
            raise ValueError('STABLE_PICKUP_TASK_REQUIRED')
        self.record('pickup_recovery_settings.json', json.dumps(asdict(settings), sort_keys=True).encode())
        local_limit = self.clock()+settings.total_recovery_budget_s
        deadline = local_limit if deadline is None else min(local_limit, deadline)
        current = initial
        tried, count, fallback_used = set(), 0, set()
        # All queue entries refer to exactly one current verified acquisition.
        # A new associated domain replaces this queue; old geometry is not reused.
        queue = list(probes)
        while self.clock() < deadline:
            available = [p for p in queue if (current.domain.domain_sha256, p.query_id) not in tried]
            if count >= settings.max_extra_view_motions or not available:
                domain_id = current.domain.domain_sha256
                if current is initial or domain_id in fallback_used:
                    break
                fallback_used.add(domain_id)
                current = self.ports.prepare_current(current.detection, current.domain, deadline)
                if self.clock() >= deadline:
                    break
                if current.response.success:
                    return current.result()
                # Count exhaustion permits existing-data planning, never another
                # view. New ordinary probes otherwise retain its normal order.
                if count >= settings.max_extra_view_motions:
                    break
                queue = list(current.observation_probes())
                continue
            probe = available[0]
            tried.add((current.domain.domain_sha256, probe.query_id))
            sampling = asdict(settings.sampling)
            search = PreparePickupViewRequestV2(request_id=self.request_id(),
                task_request_id=current.request.request_id, domain_sha256=current.domain.domain_sha256,
                query_id=probe.query_id, max_ik_seeds=settings.max_ik_seeds, **sampling)
            goal = ExecutePickupViewV2.Goal(pickup_task_id=pickup_task_id, preparation=search,
                max_extra_view_motions=settings.max_extra_view_motions,
                total_recovery_budget_s=settings.total_recovery_budget_s,
                budget_s=max(0., deadline-self.clock()))
            wrapped = self.ports.execute_view(goal, deadline)
            result = wrapped.result
            self.record('pickup_recovery_progress.json', json.dumps(dict(task_id=pickup_task_id,
                domain_sha256=current.domain.domain_sha256, query_id=probe.query_id,
                status=wrapped.status, success=result.success, dispatched=result.dispatched_motions,
                remaining_s=max(0., deadline-self.clock())), sort_keys=True).encode())
            if (not result.motion_terminal or result.may_have_payload
                    or result.dispatched_motions < count):
                raise RuntimeError('PICKUP_RECOVERY_REQUIRES_TERMINAL_EMPTY_MOTION')
            count = result.dispatched_motions
            if self.clock() >= deadline:
                break
            if not result.success:
                continue
            if (wrapped.status != GoalStatus.STATUS_SUCCEEDED or not result.submitted
                    or not result.settled_stamp_valid):
                raise RuntimeError('PICKUP_RECOVERY_SETTLED_ACTION_REQUIRED')
            if (result.plan.request_id, result.plan.task_request_id,
                    result.plan.domain_sha256, result.plan.query_id) != (
                    search.request_id, search.task_request_id, search.domain_sha256, search.query_id):
                raise RuntimeError('PICKUP_RECOVERY_VIEW_CORRELATION_MISMATCH')
            detection, domain = self.ports.observe_after(int(result.settled_stamp_ns), deadline)
            if self.clock() >= deadline:
                break
            request = ReprepareAfterPickupViewV2.Request(request_id=self.request_id(),
                previous_view=result.plan, settled_stamp_valid=True,
                settled_stamp_ns=result.settled_stamp_ns,
                # User-confirmed premise: the ungrasped pickup box is stationary.
                pickup_stationary=True, detection=detection, generation=domain.generation,
                association_policy=StationaryPickupPolicyV2(**asdict(settings.association_policy)),
                information_sample_shape=list(settings.information_sample_shape),
                budget_s=max(0., deadline-self.clock()))
            response = self.ports.continue_after(request, deadline)
            if self.clock() >= deadline:
                break
            if response.associated:
                # Runtime verified and persisted the alias; only this exact new
                # measured acquisition may feed the next original-target attempt.
                current = PickupPreparationAttempt(request, response, response.detection, response.top_domain)
                if response.success:
                    return current.result()
                queue = list(current.observation_probes())
            elif response.success or response.plan.plan_digest:
                raise RuntimeError('ASSOCIATED_RECOVERY_PLAN_REQUIRED')
        return current.result()
