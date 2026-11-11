"""Read-only bounded ROS service around a pinned perception-owned context."""
import json
import math
import time

from luggage_perception.ros_suction_domain import require_matching_detection


# Batch/iteration resource caps, not geometric inference thresholds. Eight
# candidate groups is the approved first-stage coupled-planner work bound.
MAX_QUERIES = 8
MAX_REFINEMENTS = 8


class DirectedSuctionService:
    def __init__(self, clock=time.monotonic):
        self._context = None
        self._clock = clock

    def replace_context(self, context):
        """One reference swap; an in-flight handler retains its own snapshot."""
        self._context = context

    def context_snapshot(self):
        """Current immutable acquisition for explicit owner retention."""
        return self._context

    def handle(self, request, response):
        from luggage_msgs.msg import DirectedSuctionContactV2
        started = self._clock()
        context = self._context
        # Explicitly clear reused response objects too; no stale/partial witness.
        response.success = False
        response.contacts = []
        response.header.frame_id = ''
        response.header.stamp.sec = response.header.stamp.nanosec = 0
        response.domain_sha256 = response.quality_policy_sha256 = response.samples_sha256 = ''

        def refuse(reason):
            response.success = False
            response.reason = reason
            response.contacts = []
            response.elapsed_s = max(0., self._clock()-started)
            return response

        try:
            if (not math.isfinite(request.budget_s) or request.budget_s <= 0
                    or not 1 <= len(request.queries) <= MAX_QUERIES
                    or not 1 <= request.max_refinements <= MAX_REFINEMENTS):
                return refuse('INVALID_QUALITY_BUDGET_OR_BATCH')
            identifiers = [query.query_id for query in request.queries]
            if any(not value.strip() for value in identifiers) or len(set(identifiers)) != len(identifiers):
                return refuse('INVALID_QUALITY_QUERY_ID')
            if any(not all(math.isfinite(v) for v in (*query.center_uv_top_m, query.yaw_rad))
                   for query in request.queries):
                return refuse('INVALID_QUALITY_QUERY_GEOMETRY')
            if context is None:
                return refuse('OBSERVED_QUALITY_CONTEXT_UNAVAILABLE')
            if (request.domain_sha256 != context.domain_sha256
                    or (request.expected_quality_policy_sha256
                        and request.expected_quality_policy_sha256 != context.policy_sha256)):
                return refuse('OBSERVED_QUALITY_IDENTITY_MISMATCH')
            require_matching_detection(context.domain, request.detection,
                generation=int(request.generation), contact_model_hash=context.domain.contact_model_hash)
            deadline = started+request.budget_s
            contacts = []
            for query in request.queries:
                result = context.evaluate(query.center_uv_top_m, query.yaw_rad,
                    deadline=deadline, max_refinements=int(request.max_refinements), clock=self._clock)
                if result.reason == 'QUALITY_DEADLINE':
                    return refuse('QUALITY_DEADLINE')
                contact = DirectedSuctionContactV2()
                contact.query_id = query.query_id
                contact.center_uv_top_m, contact.yaw_rad = list(result.query_uv), result.yaw_rad
                contact.accepted, contact.reason = result.accepted, result.reason
                if result.accepted:
                    contact.world_from_contact_frame = list(result.world_from_contact_frame)
                    contact.quality_metrics_json = json.dumps(dict(result.metrics), sort_keys=True,
                        separators=(',', ':'), allow_nan=False)
                contact.refinement_count, contact.evidence_sha256 = result.refinement_count, result.evidence_sha256
                contacts.append(contact)
            if self._clock() >= deadline:
                return refuse('QUALITY_DEADLINE')
            response.header.frame_id = 'world'
            response.header.stamp.sec, response.header.stamp.nanosec = divmod(context.domain.stamp_ns, 10**9)
            response.domain_sha256 = context.domain_sha256
            response.quality_policy_sha256 = context.policy_sha256
            response.samples_sha256 = context.samples_sha256
            response.contacts = contacts
            response.success, response.reason = True, 'QUALITY_BATCH_COMPLETE'
            finished = self._clock()
            if finished >= deadline:
                response.success = False
                return refuse('QUALITY_DEADLINE')
            response.elapsed_s = max(0., finished-started)
            return response
        except (TypeError, ValueError) as error:
            return refuse('INVALID_QUALITY_INPUT:'+str(error))
