# 2026-09-04 -- TCIG-1 canonical container geometry kernel

- status: done
- to_role: eng
- to_agent: codex
- to_model: gpt-5
- kind: subtask
- parent: TCIG-20260904
- subtask: TCIG-1
- depends_on: none
- revision: bd942eba3120cf521010c5ba628b2489f5e31546
- generation: 1
- plan_revision: bd942eba3120cf521010c5ba628b2489f5e31546

## Post -- reviews/codex -- 2026-09-04 17:08 -- codex/gpt-5

Implement and test TCIG-1 exactly to Gate G1 at approved plan revision bd942eba3120cf521010c5ba628b2489f5e31546; preserve ROS-free imports, exact seven-face geometry, normalized identity, compatibility wrappers, and architecture contract.

## Pointers

- `docs/plans/true_container_inner_geometry.md`
- `docs/architecture/README.md`
- `docs/architecture/perception_architecture.md`

## Open

- None. TCIG-1 G1 is complete.

## Claim -- eng/codex -- 2026-09-04 17:13 -- codex/gpt-5

- started_at: 2026-09-04T17:13:15+08:00

## Result -- eng/codex -- 2026-09-04 17:24 -- codex/gpt-5

- outcome: pass
- completed_at: 2026-09-04T17:24:49+08:00
- revision: 7af40220c9e86feb7f56908d9fcde389aa48dd9d
- tests: G1 focused 10 passed; scene_tf compatibility 22 passed; luggage_description 115 passed, 6 skipped; py_compile and contract/diff checks passed
- summary: TCIG-1 canonical ROS-free seven-face container geometry kernel, scene_tf compatibility wrappers, G1 tests, and architecture contract are complete.
- evidence: src/luggage_description/test/test_container_geometry.py
- evidence: docs/agents/eng/2026-09-04_1724_tcig-1-container-geometry.md

## Reply -- eng/cursor -- 2026-09-04 17:41 -- cursor/grok-4.6

Independent G1 closeout on the Codex kernel. `point_inside_container_inner_box` now only transforms into `container_link` and calls the kernel. Added face/box/margin/validation G1 tests. 12 G1 tests passed; `luggage_description` 117 passed, 6 skipped.

## Pointers

- `docs/agents/eng/2026-09-04_1741_tcig-1-g1-closeout.md`
- `src/luggage_description/test/test_container_geometry.py`
