"""ST09-B acceptance (f)/(h): warm 10/10 within budget, cold reported.

One cold plan_route (fresh stack, real MoveIt edge building) is timed and
reported separately - it may exceed the 10 s dev budget. Ten warm
requests on the same graph must all find the route within 10 s each
(served from verified cached edges). Durations, p50/max, MoveIt call
counts, budget snapshots, and digests land in the evidence JSON under
``ST09_B_EVIDENCE_ROOT`` (default: pytest tmp - runtime output only;
the closing evidence run points it at the task evidence directory).

Dev-time fixture capture: run once with ``ST09_B_SAVE_FIXTURE=1`` to
write ``fixtures/healthy_graph.json`` (the committed healthy store whose
digest the offline battery pins).
"""

from __future__ import division

import json
import os
import time
from pathlib import Path

import pytest

from st09_contracts.fixtures import make_snapshot

ROUTE_TOTAL_WALL_S = 10.0

REPO_ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope="module")
def chain(mock_graph):
    from route_harness import build_route_chain
    return build_route_chain(mock_graph.node)


def _percentile(values, fraction):
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round(fraction * (len(ordered) - 1)))))
    return ordered[index]


class TestBenchmark:

    def test_warm_ten_of_ten_within_budget_cold_reported(self, chain,
                                                         tmp_path):
        planner = chain["planner"]
        attachment, candidate, target = (chain["attachment"],
                                         chain["candidate"],
                                         chain["target"])

        cold_start = time.monotonic()
        cold_route = planner.plan_route(
            make_snapshot(1), attachment, candidate, (target,),
            budget_ns=int(ROUTE_TOTAL_WALL_S * 1e9))
        cold_s = time.monotonic() - cold_start

        warm = []
        for _ in range(10):
            began = time.monotonic()
            route = planner.plan_route(
                make_snapshot(1), attachment, candidate, (target,),
                budget_ns=int(ROUTE_TOTAL_WALL_S * 1e9))
            warm.append((time.monotonic() - began, route.route_id))
        assert len({route_id for _t, route_id in warm}) == 1
        assert all(duration <= ROUTE_TOTAL_WALL_S for duration, _ in warm)

        report = {
            "schema": "st09.b.benchmark/1",
            "cold_s": round(cold_s, 6),
            "cold_route_id": cold_route.route_id,
            "cold_within_budget": cold_s <= ROUTE_TOTAL_WALL_S,
            "warm_s": [round(duration, 6) for duration, _ in warm],
            "warm_p50_s": round(_percentile([d for d, _ in warm], 0.5), 6),
            "warm_max_s": round(max(d for d, _ in warm), 6),
            "warm_all_within_budget": True,
            "budget_snapshot": planner.last_budget_snapshot,
            "moveit_call_counts": chain["adapter"].moveit_call_counts(),
            "moveit_call_durations": chain["adapter"].call_durations(),
            "route_digest": cold_route.digest(),
            "edge_digests": list(cold_route.edge_digests),
            "graph_digest": chain["graph"].digest(),
            "note": "dev budgets; cold includes all new MoveIt edges",
        }
        evidence_root = os.environ.get("ST09_B_EVIDENCE_ROOT")
        out_path = (Path(evidence_root) if evidence_root else tmp_path)
        out_path.mkdir(parents=True, exist_ok=True)
        with open(out_path / "benchmark.json", "w") as handle:
            json.dump(report, handle, indent=1, sort_keys=True)

        if os.environ.get("ST09_B_SAVE_FIXTURE") == "1":
            from luggage_planning.atlas.graph import GraphStore
            fixtures = REPO_ROOT / "tests" / "st09_parallel" / "B" / "fixtures"
            store = GraphStore(fixtures)
            store.save(chain["graph"], "healthy_graph")
        # The report itself is asserted, not just written.
        assert report["warm_p50_s"] <= ROUTE_TOTAL_WALL_S
        assert report["moveit_call_counts"]["cartesian"] >= 3


class TestHealthyFixture:
    """The committed healthy store loads with candidate edges and a
    stable digest (offline; skips until the fixture has been captured)."""

    def test_committed_fixture_loads(self):
        fixture = (REPO_ROOT / "tests" / "st09_parallel" / "B" / "fixtures"
                   / "healthy_graph.json")
        if not fixture.exists():
            pytest.skip("healthy_graph.json not captured yet "
                        "(run once with ST09_B_SAVE_FIXTURE=1)")
        from luggage_planning.atlas.graph import GraphStore
        graph = GraphStore(fixture.parent).load("healthy_graph")
        assert graph.candidate_edges, "stored trajectories must load as candidates"
        assert graph.prior_gaps == ()
        # Round-trip stability: reloading reproduces the stored digest.
        again = GraphStore(fixture.parent).load("healthy_graph")
        assert again.digest() == graph.digest()
