"""Preparation watchdog policy and additive diagnostic stage timing.

The default is a configurable diagnostic watchdog, not a production latency
target. The request still sets one absolute deadline. Timings are nested scopes;
their durations must not be summed to infer total request time.
"""
from dataclasses import dataclass
import math
import time


DEFAULT_MAX_PREPARATION_BUDGET_S = 300.


@dataclass(frozen=True)
class PreparationBudget:
    max_budget_s: float = DEFAULT_MAX_PREPARATION_BUDGET_S

    def __post_init__(self):
        if (isinstance(self.max_budget_s, bool) or not math.isfinite(self.max_budget_s)
                or self.max_budget_s <= 0):
            raise ValueError('FINITE_POSITIVE_PREPARATION_WATCHDOG_REQUIRED')

    def admits(self, requested_s):
        return (not isinstance(requested_s, bool) and math.isfinite(requested_s)
                and 0 < requested_s <= self.max_budget_s)


def execution_observation_age_limit_ns(max_preparation_budget_s):
    """Initial physical-observation window plus the admitted planning budget.

    This bounds a NEW action; its owner must still retain the exact original
    observation. Replay tombstones use this same original-acquisition bound.
    """
    from luggage_planning.planning_contract.attachment import ATTACHMENT_MAX_AGE_NS
    budget = PreparationBudget(max_preparation_budget_s)
    return ATTACHMENT_MAX_AGE_NS + math.ceil(budget.max_budget_s * 1e9)


class StageTimings:
    """Append one bounded diagnostic per attempted boundary, including errors."""

    def __init__(self, scope, trace, *, clock=time.monotonic, unit=None):
        self.scope, self.trace, self.clock, self.unit = scope, trace, clock, unit

    def call(self, stage, function, *args, **kwargs):
        began = self.clock()
        outcome = 'completed'
        try:
            return function(*args, **kwargs)
        except BaseException:
            outcome = 'error'
            raise
        finally:
            row = ('timing', self.scope, stage, max(0., self.clock()-began), outcome)
            self.trace.append(row if self.unit is None else row+(self.unit,))
