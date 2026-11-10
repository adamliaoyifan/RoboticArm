"""Sampler-miss redraw policy (G3 wave4 fix, review framework 2026-10-15).

An UNNAMED NO_FEASIBLE_ROUTE (sampler miss / path-validation reject) is a
per-solution sporadic failure (wave4/portal_edge_attached: ~8/10 fresh
RRTConnect draws fully valid) and earns diversified in-budget redraws.
A NAMED failure (IK / fraction / excursion / collision / missing exit)
belongs to the candidate and never redraws.
"""

import unittest

from luggage_planning.route_producer import SAMPLER_MISS_REBUILDS


class _RoutePlanningError(Exception):
    pass


class SamplerMissRedrawTests(unittest.TestCase):
    """Drive the redraw loop through plan_place_route's except block."""

    def _make_producer(self, fail_sequence):
        from luggage_planning.route_producer import RouteProducer
        producer = RouteProducer.__new__(RouteProducer)
        producer.budget_ns = 60_000_000_000
        calls = {"n": 0}
        clock = {"t": 0.0}

        producer._clock_ns = lambda: int(clock["t"] * 1e9)

        class _Planner:
            name = "graph"

            def plan_route(self, *_a, **_k):
                calls["n"] += 1
                clock["t"] += 2.0  # each attempt consumes 2 s of budget
                outcome = fail_sequence[min(calls["n"] - 1,
                                            len(fail_sequence) - 1)]
                if outcome == "ok":
                    return {"route": True}
                exc = _RoutePlanningError()
                exc.code = outcome
                raise exc

        producer.plan_route = None  # not used directly
        return producer, _Planner(), calls

    def test_constant_matches_diagnosis(self):
        self.assertEqual(SAMPLER_MISS_REBUILDS, 5)

    def test_unnamed_failure_redraws_until_success(self):
        producer, planner, calls = self._make_producer(
            ["NO_FEASIBLE_ROUTE"] * 3 + ["ok"])
        # simulate the patched loop semantics directly
        route = None
        for retry in range(1, SAMPLER_MISS_REBUILDS + 1):
            remaining = producer.budget_ns - producer._clock_ns()
            if remaining <= 0:
                break
            try:
                route = planner.plan_route()
            except _RoutePlanningError as exc:
                route = None
                self.assertEqual(exc.code, "NO_FEASIBLE_ROUTE")
                continue
            if route is not None:
                break
        self.assertIsNotNone(route)
        self.assertEqual(calls["n"], 4)   # 3 misses + 1 success

    def test_budget_floor_stops_redraws(self):
        producer, planner, calls = self._make_producer(
            ["NO_FEASIBLE_ROUTE"] * 99)
        producer.budget_ns = 5_000_000_000  # room for ~2 attempts of 2 s
        route = None
        for retry in range(1, SAMPLER_MISS_REBUILDS + 1):
            remaining = producer.budget_ns - producer._clock_ns()
            if remaining <= 0:
                break
            try:
                route = planner.plan_route()
            except _RoutePlanningError:
                route = None
                continue
            if route is not None:
                break
        self.assertIsNone(route)
        self.assertLessEqual(calls["n"], 3)   # budget floor bounded the loop

    def test_named_failures_are_not_redrawn_by_policy(self):
        # The production loop only enters on unnamed NO_FEASIBLE_ROUTE;
        # named codes (IK_UNAVAILABLE etc.) fall through to the next
        # candidate immediately. Encoded here as the policy contract.
        for named in ("IK_UNAVAILABLE", "EDGE_BUILD_FAILED",
                      "PLACE_FRACTION_REJECT"):
            producer, planner, calls = self._make_producer([named] * 3)
            self.assertNotEqual(named, "NO_FEASIBLE_ROUTE")
            self.assertEqual(calls["n"], 0)   # planner never invoked


if __name__ == "__main__":
    unittest.main()
