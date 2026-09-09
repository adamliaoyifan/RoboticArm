# 2026-09-09 -- PF-R9 generation 2 payload-backed depth-primary implementation

- status: open
- to_role: eng
- to_agent: claude
- to_model: glm-5.3
- kind: subtask
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R9
- depends_on: PF-R6,PF-R8
- revision: 5fe74ed72135e569efd1e01f47047f664234fb6b
- generation: 2
- plan_revision: 5fe74ed72135e569efd1e01f47047f664234fb6b
- dispatch_ready: yes

## Post -- reviews/codex-reviews-main -- 2026-09-09 11:47 -- codex/gpt-5.6-sol

User-directed execution handoff; no further consensus is required. Before claiming generation 2, acknowledge the stopped generation-1 thread with scripts/agent_mailbox.py stop-ack. Then claim this thread with scripts/agent_start.sh and own implementation, tests, repair loops, commits, simulation teardown, evidence, and D555 validation end to end. Use docs/plans/pf_r9_g2_payload_depth_primary_execution.md at the bound plan revision. Fixed cache contract: 15 entries and 1.0 second on camera and affected exact-join buffers. D1 complete-path measurement is first; D2 eliminates every avoidable preprocessor Python pixel copy while preserving mutation isolation; D3-D8 bars may not be lowered.

## Pointers

- `docs/plans/pf_r9_g2_payload_depth_primary_execution.md`
- `docs/plans/pf_f3_depth_primary_contract.md`
- `docs/status/evidence/platform_free_height/2026-09-08_pfr9_throughput/`
- `docs/status/evidence/d555_bringup/`
- `docs/agents/discuss/2026-09-07_2039_pf-r9-preprocessor-throughput.md`

## Open

- Acknowledge the PF-R9 generation-1 supersede, then implement and close PF-R9 generation 2 end to end against the exact payload/cache plan and D1-D8 acceptance.

