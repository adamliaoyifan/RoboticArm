# 2026-09-10 -- PF-R10 generation 3 closed-loop placement then C1-C3

- status: open
- to_role: eng
- to_agent: cursor
- to_model: grok-4.6
- kind: integration
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R10
- depends_on: PF-R6,PF-R8,PF-R9
- revision: 0001c413147e4a2f00a01801d6a8ac16ca92ba93
- generation: 3
- plan_revision: 7b2a5f22e6957b6040e4849c64c847efa77cba00
- dispatch_ready: yes

## Post -- reviews/cursor -- 2026-09-10 14:45 -- cursor/grok-4.6

User-directed PF-R10 owner replacement; no new consensus is required. Generation 2 remains claimed by claude/glm-5.3 and is superseded by this thread. Acceptance C1-C3 is unchanged at plan revision 7b2a5f22e6957b6040e4849c64c847efa77cba00. Implement the 2026-09-10 reviews checkpoint, then close end to end.

Simulation-only next gate: after settle, verify entity identity, XY, roll/pitch, and pose persistence against /world/airport_loading/pose/info; retry a bounded number of times; fail the trial explicitly if the requested pose cannot be established. Do not filter failed placements out of scoring or change detector geometry thresholds. Keep --warmup-frames 30, the untrimmed recovery series, recovery no longer than 1.4 s, and at least 30 settled scored frames per trial. Repair the three stale-fixture failures and the eng-note status metadata. Pass only on one clean exact commit with three consecutive stored six-trial Gate-4 runs, PF-G6S, C3 teardown with zero residual processes, and complete raw plus summary artifacts. Hardware calibration and deployed TF-tree edits are not inputs.

## Pointers

- `docs/plans/pf_r8_r9_perception_acceptance.md`
- `docs/agents/discuss/2026-09-07_2039_pf-r10-gate4-integration.md`
- `docs/agents/reviews/2026-09-10_1425_claude-pfr9-pfr10-sim-review.md`
- `docs/status/evidence/platform_free_height/2026-09-09_pfr10_gate4_integration/RESULT.md`

## Open

- User-directed PF-R10 generation 3: implement bounded closed-loop Gazebo placement verification, repair stale fixtures, then pass C1-C3 on one clean commit with three consecutive gate4_short6 runs plus PF-G6S.

## Claim -- eng/cursor -- 2026-09-10 14:46 -- cursor/grok-4.6

- started_at: 2026-09-10T14:46:17+08:00
- claimed_generation: 3
- claimed_plan_revision: 7b2a5f22e6957b6040e4849c64c847efa77cba00
- claimed_dependencies: PF-R6=3,PF-R8=1,PF-R9=2

## Reply -- eng/cursor -- 2026-09-10 21:17 -- cursor/grok-4.6

Status for reviews. Generation 3 remains open. Placement closed-loop is on
master (`0d24f18`, `60ad37e`, `a5c5e29`). Perception fix
`df9c7a27f8ad81102b6f5462b08b724910b042cd` paints only accepted YOLO cargo
boxes and vectorizes `_refine_rectangle` without changing RANSAC thresholds.

Dirty-tree `exp_mask_accepted2` passed Gate-4 numbers (Hz 25.18, top 1.0,
FULL_3D 0.983, `failed=0`, recovery 0.78-1.05 s, width p95 0.041 m) and is
not C1. Clean identity run1 at `df9c7a2` (`dirty=0`) failed trial 2 large
vintage with whole-window `DETECT_LOW_CONFIDENCE` and 2946 cargo points.
Streak stopped. Teardown residual 0.

C1 still requires three consecutive `gate4_short6` plus PF-G6S on one clean
exact commit. Table: `docs/status/evidence/platform_free_height/2026-09-10_pfr10_g3/RESULT.md`.
The generation-3 mailbox row is restored as `Q-20260910-1`. Status for
reviews is `Q-20260910-3`
(`docs/agents/discuss/2026-09-10_2119_pfr10-g3-status-for-reviews.md`).
