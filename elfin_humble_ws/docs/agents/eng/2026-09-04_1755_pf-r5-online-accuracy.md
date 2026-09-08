# 2026-09-04 17:55 -- PF-R5 online accuracy

- role: eng
- agent: claude
- model: glm-5.3
- cli: claude-code
- status: done

## Summary

PF-R5 initially exposed mesh-reference and semantic-detection blockers. The
rework closed at `c5921d5` with all original Gate 4 limits passing in the clean
run8 evidence; the remaining simulation and performance items are follow-up
work outside this result.

## Rework result (2026-09-04 21:1x -- official PASS at c5921d5)

Per the reviews rework decision (2026-09-04_1943) and the user's gate
revision (docs/plans/platform_free_height_gate4_revision.md):

- Root cause chain established with data: (1) suitcase below the 0.04
  YOLO threshold at diagonal poses — fixed by a descriptive prompt
  (offline 16-pose sweep: failing frames 0.000 -> 0.17-0.23, threshold
  unchanged); (2) the recurring launch-context detector wedge was a
  stalled sim clock making tf2 sim-time timeouts infinite — fixed with
  wall-clock-bounded zero-timeout TF retries; (3) the base_link false
  positive is documented (dropped from cargo by the workspace crop;
  self-body masking of the pedestal remains a follow-up).
- Vintage-pose regression: test_vintage_pose_regression.py (3 tests,
  GPU-gated) with 4 saved fixtures from the failing sweep.
- Official run8: isolated clean worktree /tmp/pfr5_clean @ c5921d5
  (dirty=0), 30 trials, **every gate passes including the original
  top_surface_rate 0.9606 >= 0.95** — no gate was relaxed or split-open.
  Evidence: docs/status/evidence/platform_free_height/2026-09-04_2110_pfr5-g4s-run8-official/
  (RESULT.md + summary.json + frames.jsonl).
- Textured-sim finding: Fortress ogre2 permanently wedges the camera
  sensor render thread on runtime import of textured meshes (DAE/OBJ);
  verified live (spawn ok -> /camera/* stops -> delete does not
  recover). Textured simulation must preload textured models in the
  world SDF (init-time load) and place them via set_pose.

Follow-ups outside PF-R5: SIM 3-box closed-loop gate (needs the
GT-fallback-free closed-loop driver), representative detection gate
(needs textured sim or Gate 5 bags), pedestal self-body masking,
PF-R6 performance (active Hz 3.91 close to the 4 Hz bar).

## Pointers

- `docs/agents/discuss/2026-09-04_1453_pfh-remediation-r5.md`
- `docs/status/evidence/platform_free_height/2026-09-04_2110_pfr5-g4s-run8-official/`
- `docs/plans/platform_free_height_gate4_revision.md`
