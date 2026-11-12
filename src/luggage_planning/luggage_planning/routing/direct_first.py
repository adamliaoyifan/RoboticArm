"""Candidate-local topology order with one deadline and lazy staging work."""
from luggage_planning.routing.grid_search import RoutePlanningError

# Reserve time for the optional alternative without extending candidate budget.
DIRECT_BUDGET_FRACTION = 0.60


class DirectFirstRoutePlanner:
    def __init__(self, factory, clock_ns, *, has_staging, attempts):
        self.factory = factory
        self.clock_ns = clock_ns
        self.has_staging = has_staging
        self.attempts = attempts
        self.current = None
        self.trace = None
        self.deadline_ns = None

    def __getattr__(self, name):
        # Certifier/retry diagnostics belong to the planner actually used.
        return getattr(self.current, name)

    def plan_route(self, *args, budget_ns):
        proposed_deadline = self.clock_ns() + max(0, int(budget_ns))
        self.deadline_ns = (proposed_deadline if self.deadline_ns is None else
                            min(self.deadline_ns, proposed_deadline))
        deadline = self.deadline_ns
        last_error = None
        for topology in ('direct', 'staging') if self.has_staging else ('direct',):
            remaining = deadline - self.clock_ns()
            if remaining <= 0:
                break
            allowance = (int(remaining * DIRECT_BUDGET_FRACTION)
                         if topology == 'direct' and self.has_staging else remaining)
            attempt_deadline = self.clock_ns() + allowance
            record = dict(topology=topology, budget_ns=allowance)
            self.attempts.append(record)
            started = self.clock_ns()
            try:
                # Staging IK and validation are performed only by this lazy call.
                self.current = self.factory(topology == 'staging', attempt_deadline)
                self.current.trace = self.trace
                left = min(deadline, attempt_deadline) - self.clock_ns()
                if left <= 0:
                    raise RoutePlanningError('PLANNING_BUDGET_EXHAUSTED',
                                             {'stage': 'topology_preparation'})
                route = self.current.plan_route(*args, budget_ns=left)
                record['outcome'] = 'planned'
                return route
            except RoutePlanningError as error:
                last_error = error
                record.update(outcome='failed', reason=error.code, detail=error.detail)
            finally:
                record['elapsed_ns'] = self.clock_ns() - started
        if last_error is not None:
            raise last_error
        raise RoutePlanningError('PLANNING_BUDGET_EXHAUSTED', {'stage': 'topology_budget'})
