# 2026-09-08 -- RL for packing density vs arm control

- role: eng
- agent: cursor-grok-b
- model: grok-4.6
- cli: cursor
- status: done

## Summary

End-to-end RL is a poor fit for Elfin joint control and Cartesian insert trajectories. Occupancy ratio lives in discrete placement ranking among hull/corridor/IK-safe slots. That layer already has EMS, heuristic scores, and a lookahead value estimator; a later learned ranker (LRF-PL1) may rerank those candidates but must not emit trajectories or convert an infeasible slot into a feasible one.

## Pointers

- `docs/plans/corridor_constraints.md`
- `docs/plans/learning_research_feasibility.md` (LRF-PL1)
- `src/luggage_packing/luggage_packing/value_estimator.py`
- `src/luggage_packing/luggage_packing/placement_solver.py`
- `docs/agents/eng/2026-09-05_1740_waypoint-cartesian-ik.md`
