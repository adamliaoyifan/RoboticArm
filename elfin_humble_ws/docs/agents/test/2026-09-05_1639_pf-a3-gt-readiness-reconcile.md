# 2026-09-05 -- PF-A3 gt_readiness reconciliation (check 8)

- role: test
- agent: cursor
- model: grok-4.6
- cli: cursor
- status: done
- parent: PFH-REMEDIATION-20260904
- subtask: PF-A3
- revision: a3dba5e7e0c2fb219062cfb2a091267f6f2d606d

## Summary

Independent re-check of PF-A3 check 8 at `a3dba5e` (ancestor of workspace
`HEAD` `023ef22`). The historical audit at `c5921d5` stays unchanged:
`audit_outcome=pass`, `gt_readiness=blocked` because the spawner then
substituted catalog size into GetCurrentBox. At `a3dba5e`, that fallback is
gone. `gt_readiness` is pass. This is not a PF-R5 or PF-R7 close.

## Commands

- `git merge-base --is-ancestor a3dba5e HEAD`
- `PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:src/luggage_description:src/luggage_gazebo python3 -m pytest -q src/luggage_description/test/test_pf_r5a_gt_fail_closed.py`

## Evidence

- `docs/status/evidence/platform_free_height/2026-09-05_1620_pfr5b-closeout/RESULT.md`
- `src/luggage_description/test/test_pf_r5a_gt_fail_closed.py`
- `src/luggage_gazebo/scripts/pickup_box_spawner_node.py`

## Result

- pass
- audit_outcome: pass (unchanged from `c5921d5`)
- gt_readiness: pass at `a3dba5e`
- pytest: 10 passed in 0.04s
- `_observable_reference` / `_gt_size` raise `MeshReferenceError`; catalog dimensions are not substituted
- `handle_spawn_next` resolves GT before `handle_clear`; on failure restores RNG and returns `MESH_REFERENCE_UNAVAILABLE` with no spawn or publish

## Pointers

- `docs/agents/test/2026-09-04_2029_pf-a3-mesh-gt-stability.md`
- `docs/agents/discuss/2026-09-04_2022_pf-a3-mesh-gt-stability-audit.md`
- `docs/status/evidence/platform_free_height/2026-09-04_2024_pf-a3/`
