# 2026-09-08 -- Global packing value vs myopic slot score

- role: eng
- agent: cursor-grok-b
- model: grok-4.6
- cli: cursor
- status: done

## Summary

The global objective is expected remaining reachable fill, not single-box score. Online packing cannot wait for a sequence-optimal layout because future sizes are unknown. Existing `value_hat` and CEM already target that Bellman backup, but the inner rollout still ignores IK and corridor, so it maximizes leftover geometry rather than leftover plannable volume. Fold atlas/corridor into the rollout inner policy and calibrate on full-horizon `reachable_fill_rate`; do not train a joint-space RL policy to discover that value.

## Pointers

- `src/luggage_packing/luggage_packing/value_estimator.py`
- `src/luggage_packing/luggage_packing/packing_replay.py`
- `src/luggage_packing/luggage_packing/placement_scoring.py`
- `src/luggage_packing/luggage_packing/insertion_corridor.py`
- `docs/agents/eng/2026-09-08_1131_trajectory-feasibility-vs-slot.md`
