"""Experimental, bounded traversal of a planned cuboid support seam.

This is a distinct contact mode, disabled by default. It never changes the
strict bottom/top support predicate. Edge contacts do not count as load-bearing
support; separate strict native support must remain. No force safety claim is
made. The caller retains all arm/tool collision and measured stability checks.
"""
import numpy as np
from edge_contact_diagnostic import diagnose_support_edge
from push_seam_events import trace_push_seams


class PushEdgeTransition:
    def __init__(self, action, dimensions, patches, policy):
        self._action = action
        self._size = dimensions
        self._policy = policy
        report = trace_push_seams(action.handoff_box_pose,
            action.placement.final_box_pose, dimensions, patches)
        allowed = set(action.handoff_support.source_ids) | set(action.placement.support.source_ids)
        self._events = {e['source_id']: e for e in report['events']
                        if e['kind'] == 'enter' and e['source_id'] in allowed and
                        abs(e['nominal_bottom_minus_top_m']) <= policy.push_edge_point_tolerance_m}
        self._first = {}
        self._last_observed_stamp = None
        self._count = 0
        self._maximum_depth = 0.
        self._maximum_duration = 0.

    def classify(self, box_pose, support_id, support_pose, bounds, point, normal,
                 depth, *, normal_toward_box):
        event = self._events.get(support_id)
        if event is None:
            return False
        start = np.asarray(self._action.handoff_box_pose).reshape(4, 4)
        box = np.asarray(box_pose).reshape(4, 4)
        direction = np.asarray(self._action.push_direction)
        progress = float((box[:3, 3] - start[:3, 3]) @ direction)
        # Event band includes the configured transverse/rotation tracking tube;
        # the actual two material edges must still coincide within 0.1 mm.
        if abs(progress-event['progress_m']) > self._policy.push_edge_event_band_m:
            return False
        result = diagnose_support_edge(box, self._size, support_pose, bounds,
            point, normal, direction,
            point_tolerance_m=self._policy.push_edge_point_tolerance_m,
            penetration_m=self._policy.push_edge_penetration_m,
            normal_alignment_deg=self._policy.contact_normal_alignment_deg,
            depth_m=depth, normal_toward_box=normal_toward_box)
        return result['edge_candidate'] and not result['current_point_predicate_passed']

    def observe(self, stamp, edge_contacts):
        """Persistent/recurrent contact at a seam cannot reset its time budget."""
        if stamp == self._last_observed_stamp:return None
        self._last_observed_stamp = stamp
        for support_id, depth in edge_contacts:
            first = self._first.setdefault(support_id, stamp)
            duration = stamp-first
            self._maximum_duration = max(self._maximum_duration, duration)
            if duration > self._policy.push_edge_max_duration_s:
                return dict(reason='PUSH_EDGE_TRANSITION_TIMEOUT', body=support_id,
                            duration_s=duration)
            self._count += 1
            self._maximum_depth = max(self._maximum_depth, abs(depth))
        return None

    def metrics(self):
        return dict(mode='experimental_planned_support_edge_transition',
            admitted_points=self._count, max_depth_m=self._maximum_depth,
            max_duration_s=self._maximum_duration,
            planned_source_events={k: v['progress_m'] for k,v in self._events.items()},
            scope='nominal_geometry_and_observed_traversal_no_force_or_robust_certificate')
