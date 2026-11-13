"""Whole-operation preparation callback core; no execution or map-write effects."""
from dataclasses import dataclass
import hashlib
import json
import math
import time

from luggage_perception.ros_suction_domain import require_matching_detection
from luggage_planning.joint_operation.adapters import operation_plan_to_message
from luggage_planning.joint_operation.candidates import (
    candidate_for_contact, group_placement_positions, propose_contacts,
)
from luggage_planning.joint_operation.contracts import OperationIdentity, MAX_PLACEMENT_CANDIDATES, DIRECTED_CONTACT_REVERSALS
from luggage_perception.observed_suction_domain import ObservedSuctionDomain
from luggage_planning.placement_prior.contracts import PlacementCandidate, Snapshot, Transform
from luggage_planning.joint_operation.preparation_profile import PreparationBudget, StageTimings
from luggage_planning.joint_operation.preparation_refresh import FinalizedPreparation
from luggage_planning.joint_operation.search_evidence import PreparationSearchEvidence


@dataclass(frozen=True)
class PreparationInputs:
    # All inputs are admitted/pinned by the node's read-only snapshot adapter.
    snapshot: Snapshot
    domain: ObservedSuctionDomain
    placements: tuple
    world_from_reference_tool: Transform
    pickup_object_id: str
    retained_observation: object = None

    def __post_init__(self):
        placements = tuple(self.placements)
        if (type(self.snapshot) is not Snapshot or type(self.domain) is not ObservedSuctionDomain
                or type(self.world_from_reference_tool) is not Transform
                or not isinstance(self.pickup_object_id,str) or not self.pickup_object_id.strip()
                or any(type(slot) is not PlacementCandidate for slot in placements)):
            raise ValueError('typed pinned operation inputs required')
        object.__setattr__(self,'placements',placements)


class JointOperationPreparation:
    def __init__(self, contacts, planner, contact_model, envelope_limits, *,
                 max_center_checks, max_contact_proposals_per_direction=2,
                 atlas=None, atlas_unavailable_reason='ATLAS_UNAVAILABLE_ONLINE_IK',
                 budget=None, clock=time.monotonic):
        if type(max_center_checks) is not int or not 1 <= max_center_checks <= 4096:
            raise ValueError('bounded geometric centre checks required')
        if (type(max_contact_proposals_per_direction) is not int
                or not 1 <= max_contact_proposals_per_direction <= 4):
            raise ValueError('one to four directed contact proposals required')
        self.contacts,self.planner = contacts,planner
        self.model,self.limits = contact_model,envelope_limits
        self.max_center_checks,self.clock = max_center_checks,clock
        self.max_contact_proposals_per_direction = max_contact_proposals_per_direction
        self.atlas = atlas
        self.atlas_unavailable_reason = atlas_unavailable_reason
        self.budget = budget if budget is not None else PreparationBudget()
        if type(self.budget) is not PreparationBudget:
            raise ValueError('TYPED_PREPARATION_BUDGET_REQUIRED')

    def prepare(self, request, response, inputs, *, deadline, revalidate, finalize=None, cancelled=lambda:False):
        from luggage_msgs.msg import JointOperationPlanV2
        began = self.clock()
        response.success,response.reason = False,'SEARCH_INCOMPLETE'
        response.plan = JointOperationPlanV2()
        trace = [('atlas','QUALIFIED_PROPOSALS' if self.atlas else self.atlas_unavailable_reason)]
        search_evidence = PreparationSearchEvidence(inputs.domain)
        timing = StageTimings('candidate_preparation', trace, clock=self.clock)
        def stopped(): return cancelled() or self.clock() >= deadline
        def finish(reason):
            response.reason = reason
            trace.append(('preparation_search_evidence',search_evidence.record(reason)))
            response.trace_json = json.dumps(trace,separators=(',',':'),allow_nan=False)
            response.elapsed_s = max(0.,self.clock()-began)
            return response
        try:
            snapshot,domain = inputs.snapshot,inputs.domain
            if (not request.request_id or not request.observation_id or not request.robot_state_stamp_valid
                    or not 1 <= request.candidate_limit <= MAX_PLACEMENT_CANDIDATES or not math.isfinite(request.budget_s)
                    or not self.budget.admits(request.budget_s) or not math.isfinite(deadline)
                    or (request.geometry_hash,request.expected_map_revision,request.expected_model_sha256) !=
                       (snapshot.geometry_hash,snapshot.map_revision,snapshot.model_hash)):
                return finish('PREPARATION_INPUT_IDENTITY_MISMATCH')
            require_matching_detection(domain,request.detection,generation=int(request.generation),
                                       contact_model_hash=self.model.identity_hash)
            identity = OperationIdentity(request.request_id,snapshot.scene_revision,
                hashlib.sha256(repr(snapshot.identity).encode()).hexdigest(),snapshot.geometry_hash,
                snapshot.model_hash,snapshot.map_revision,request.observation_id,domain.stamp_ns,
                int(request.robot_state_stamp_ns),domain.box_id,domain.generation,domain.contact_model_hash)
            planning_deadline = deadline-self.planner.reserve_s
            selected = None
            def candidates():
                nonlocal selected
                groups = 0
                for placement in group_placement_positions(inputs.placements):
                    timing.unit = placement.slot_id
                    if stopped() or self.clock() >= planning_deadline or groups >= request.candidate_limit:
                        return
                    groups += 1
                    target_box = snapshot.world_from_container.compose(placement.container_from_box)
                    streams = []
                    for contact_index,opposite in enumerate(DIRECTED_CONTACT_REVERSALS):
                        hint = inputs.world_from_reference_tool
                        if opposite:
                            hint = hint.compose(Transform((0.,0.,0.),(0.,0.,1.,0.)))
                        streams.append(propose_contacts(domain,self.model,
                            query_id=placement.slot_id+':heading-'+str(contact_index),
                            world_from_place_box=target_box,desired_world_from_tool=hint,
                            max_center_checks=self.max_center_checks,
                            max_proposals=self.max_contact_proposals_per_direction))
                    for contact_round in range(self.max_contact_proposals_per_direction):
                        if stopped() or self.clock() >= planning_deadline:return
                        proposed = []
                        for stream in streams:
                            proposal = timing.call('contact_proposal',next,stream,None)
                            if proposal is None:continue
                            search_evidence.proposal(proposal)
                            trace.append((proposal.query_id,proposal.reason,proposal.checked_centres))
                            if proposal.reason == 'OBSERVED_FOOTPRINT_PROPOSAL':proposed.append(proposal)
                        if not proposed:break
                        trace.append(('contact_round',placement.slot_id,contact_round,
                            placement.container_from_box.translation))
                        evidence = timing.call('contact_quality',self.contacts.evaluate,domain,request.detection,
                            [p.to_message() for p in proposed],self.model,deadline=planning_deadline,cancelled=cancelled,
                            retained_observation=inputs.retained_observation)
                        search_evidence.quality(evidence)
                        trace.append((placement.slot_id,evidence.reason,evidence.verdicts))
                        if not evidence.complete:continue
                        for rank,contact in enumerate(evidence.contacts):
                            if stopped() or self.clock() >= planning_deadline:return
                            candidate = timing.call('bound_candidate',candidate_for_contact,
                                snapshot,domain,contact,placement,self.limits,
                                pickup_object_id=inputs.pickup_object_id,observation_id=request.observation_id,rank=rank)
                            if self.atlas is not None:
                                candidate,atlas_trace = timing.call('atlas_augmentation',self.atlas.augment,
                                    candidate,request_id=request.request_id,
                                    deadline=planning_deadline,cancelled=cancelled,clock=self.clock)
                                trace.extend(atlas_trace)
                            selected = candidate
                            search_evidence.candidate_yielded()
                            yield candidate
            if stopped(): return finish('CANCELLED_OR_DEADLINE')
            result = self.planner.prepare(identity,tuple(float(v) for v in request.start_joints),candidates(),
                budget_s=max(1e-9,deadline-self.clock()),absolute_deadline=deadline,cancelled=cancelled)
            trace.extend(result.trace)
            if stopped(): return finish('CANCELLED_OR_DEADLINE')
            if result.status != 'PLAN_READY' or result.plan is None:
                return finish(result.status)
            final_plan = result.plan
            if finalize is not None:
                if selected is None or selected.candidate_id != final_plan.candidate_id:
                    return finish('PREPARATION_SELECTED_CANDIDATE_MISMATCH')
                final = timing.call('latest_map_finalization',finalize,final_plan,selected,inputs,deadline)
                if type(final) is not FinalizedPreparation:
                    return finish('PREPARATION_FINALIZATION_RESULT_INVALID')
                trace.extend(final.trace)
                if final.plan is None:return finish(final.reason)
                final_plan = final.plan
            elif not timing.call('snapshot_revalidation',revalidate,inputs,deadline):
                return finish('PREPARATION_SNAPSHOT_CHANGED')
            if stopped(): return finish('CANCELLED_OR_DEADLINE')
            output_identity = final_plan.identity if finalize is not None else identity
            message = operation_plan_to_message(final_plan,output_identity,final_plan.plan_digest)
            if stopped(): return finish('CANCELLED_OR_DEADLINE')
            response.plan,response.success = message,True
            return finish('PLAN_READY')
        except (TypeError,ValueError,RuntimeError) as error:
            trace.append(('input',type(error).__name__,str(error)))
            return finish('INVALID_PREPARATION_INPUT:'+str(error))
