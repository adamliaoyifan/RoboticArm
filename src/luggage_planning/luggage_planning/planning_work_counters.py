"""Request-local boundary counters; never stand in for native solver internals."""
from threading import Lock


class PlanningWorkCounters:
    def __init__(self):
        self._lock = Lock()
        self._refinements = 0
        self._refinement_errors = 0

    def refine(self, operation, *args, **kwargs):
        with self._lock:
            self._refinements += 1
        try:
            return operation(*args, **kwargs)
        except Exception:
            with self._lock:
                self._refinement_errors += 1
            raise

    def snapshot(self, edge_builder):
        with self._lock:
            attempts, errors = self._refinements, self._refinement_errors
        return dict(
            scope='Python route-producer refinement and MoveIt edge boundary attempts',
            offline_refinement_calls=attempts, offline_refinement_exceptions=errors,
            edge_boundary_calls=edge_builder.moveit_call_counts(),
            edge_boundary_durations_s=edge_builder.call_durations(),
            native_ik_calls=None, native_collision_checks=None,
            total_service_calls=None, complete_for_benchmark=False,
            limitations=['One refinement call may perform multiple numerical iterations',
                        'MoveIt internally invokes IK and collision checks not observed here',
                        'Edge scene calls exclude other scene/validator/OCC service callers'])
