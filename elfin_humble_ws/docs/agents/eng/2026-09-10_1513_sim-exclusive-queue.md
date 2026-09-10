# 2026-09-10 -- Sim contract: one Gazebo stack at a time

- role: eng
- agent: cursor
- model: opus-5
- cli: cursor
- status: done

## Summary

Added an exclusivity clause to the sim contract: the Gazebo stack is a single
machine-wide resource, so an agent MUST check
`pgrep -af 'ros2 launch luggage_gazebo'` and `/tmp/elfin_humble_sim.pid` before
launching and queue behind a running sim instead of starting a second world. A
distinct `ROS_DOMAIN_ID` isolates the ROS graph but not GPU/CPU contention, and
taking the slot by stopping another agent's sim is forbidden. Mirrored the rule
in `AGENTS.md` and the `docs/agents/README.md` "Do not" list;
`scripts/check_agent_contract.sh` passes.

## Pointers

- `.cursor/rules/sim-lifecycle.mdc`
- `AGENTS.md`
- `docs/agents/README.md`
