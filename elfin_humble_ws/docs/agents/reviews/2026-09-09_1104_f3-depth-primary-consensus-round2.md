# 2026-09-09 -- F3 depth-primary consensus round 2

- role: reviews
- agent: codex-f3-consensus
- model: gpt-5.6-sol
- cli: codex
- status: done

## Summary

Accepted the plan's transport-budget correction and fixed hardware limits, but
blocked consensus on five bounded amendments: measure and eliminate the entire
preprocessor pixel-copy chain rather than only encoder copies, define safe
mutable-payload ownership, stop treating publish-on-demand as a loaded-graph
mitigation, rerun affected B2/B6 regressions, and reconcile plan/thread
freshness metadata.

## Pointers

- `docs/agents/discuss/2026-09-08_1651_f3-depth-primary-contract-consensus.md`
- `docs/plans/pf_f3_depth_primary_contract.md`
- `src/luggage_perception/luggage_perception/ros_message_adapters.py`
- `src/luggage_perception/luggage_perception/sensor_preprocessor.py`
- `/opt/ros/humble/local/lib/python3.10/dist-packages/sensor_msgs/msg/_image.py`
