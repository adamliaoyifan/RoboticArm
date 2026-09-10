# 2026-09-10 -- PF-R10 g3 refine and accepted-mask experiment

- role: eng
- agent: cursor
- model: grok-4.6
- cli: cursor
- status: open
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R10
- base_revision: 0001c413147e4a2f00a01801d6a8ac16ca92ba93
- started_at: 2026-09-10T18:31:00+08:00
- completed_at: n/a

## Summary

Continued PF-R10 generation 3 toward C1. Vectorized `_refine_rectangle`
(same geometry) and stopped painting unaccepted YOLO AABBs into the cargo
mask. A no-dump scored `gate4_short6` passed C1 numbers (`t_first_full3d`
0.26–0.62 s). The next consecutive run failed two trials with whole-window
`DETECT_TOP_UNOBSERVABLE`. After the mask change, one scored run passed
again (`failed=0`, recovery 0.78–1.05 s). Two other launches died before
observe pose because `controller_manager` missed the 60 s spawner window.
Dirty tree; not a C1 identity streak.

## Requirement

Unchanged C1–C3 on one clean commit. This note is the experiment toward
that, not a closeout.

## Changed

- `src/luggage_perception/luggage_perception/luggage_box_estimator.py`
- `src/luggage_perception/test/test_luggage_box_estimator.py`
- `src/luggage_perception/luggage_perception/semantic_segmenter.py`
- `src/luggage_perception/test/test_semantic_segmenter.py`

## Verification

- `python3 -m pytest src/luggage_perception/test/test_luggage_box_estimator.py -q`: 25 passed.
- `python3 -m pytest src/luggage_perception/test/test_semantic_segmenter.py src/luggage_perception/test/test_pf_r8_acceptance.py src/luggage_perception/test/test_luggage_box_estimator.py -q`: 70 passed.
- Scored evidence: `docs/status/evidence/platform_free_height/2026-09-10_pfr10_g3/RESULT.md`. Residual 0 on completed runs.

## Result

- open: C1 numbers seen twice on a dirty tree; consecutive identity runs and C2/C3 are not recorded.

## Pointers

- `docs/agents/discuss/2026-09-10_1445_pf-r10-g3-closed-loop-place.md`
- `docs/agents/eng/2026-09-10_1520_pf-r10-g3-closed-loop-place.md`
- `docs/status/evidence/platform_free_height/2026-09-10_pfr10_g3/RESULT.md`
