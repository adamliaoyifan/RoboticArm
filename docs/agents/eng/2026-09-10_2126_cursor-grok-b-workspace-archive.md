# 2026-09-10 -- cursor-grok-b archive before ros2_humble rebaseline

- role: eng
- agent: cursor-grok-b
- model: grok-4.6
- cli: cursor
- status: done

## Summary

Archived this identity's work before `origin/ros2_humble` worktrees. Three
isolated branches still hold code that is not on `master` or
`origin/ros2_humble`: TCIG-4, TCIG-5, and LRF-P1. The LRF-P1 MuJoCo dirty tail
was checkpointed at `097cd8e1a65aa0a97801cfc2bb753baf5be072c3`. The missing
TCIG-5 mailbox thread was restored and checkpointed at
`7a8bff1a451d3c05e5c7e91b1cbc1a64263d0b96`. No worktree was deleted.

## Keep

Cherry-pick onto a later `ros2_humble` worktree in this order. Histories are
unrelated to `origin/ros2_humble`, so these are the unique task commits, not
the full branch first-parent lists.

TCIG-4, worktree `/home/adamliao/work/elfin_humble_ws_eng_tcig4`, branch
`agent/eng/tcig-4`, HEAD `10a93e898d618bc30144189d5e344992706ddf04`:

1. `bbbcf7a949ea68554ca269fc1dd724ac5c730a79`
2. `37c157f5f1f66a8f103d9d54d49387ea003b319e`
3. `d567ad52571abb2d351c06f44c709f713a0de97f`
4. `d7240056147aac356ff9b402222e1a9fa7fc9631`
5. `10a93e898d618bc30144189d5e344992706ddf04`

TCIG-5, worktree `/home/adamliao/work/elfin_humble_ws_eng_tcig5`, branch
`agent/eng/tcig-5`, HEAD `7a8bff1a451d3c05e5c7e91b1cbc1a64263d0b96`:

1. `fcdc3e711bd730b76f0a20e73840f52320d6b892`
2. `a5edabe8386adf9e461611caa0198a63a9d69d4a`
3. `7a8bff1a451d3c05e5c7e91b1cbc1a64263d0b96`

LRF-P1, worktree `/home/adamliao/work/elfin_humble_ws_eng_lrfp1`, branch
`agent/eng/lrf-p1`, HEAD `097cd8e1a65aa0a97801cfc2bb753baf5be072c3`:

1. `42996a7bd9f9ec1d1f93b6e5766ce366045f09a1`
2. `4a0397055cde2020d2d93f7268218a03ac731426`
3. `c09ec7009c5d58fd724373fceb43ecfc6c0bcc35`
4. `5c08312192f65a1bd75a301c927b2a557faa0ab4`
5. `097cd8e1a65aa0a97801cfc2bb753baf5be072c3`

Also keep the restored notes
`docs/agents/eng/2026-09-10_1619_closed-loop-vs-learning-proposal.md` and
this archive note. Design notes already on `master` stay as documentation.

## Drop

- Do not port the LRF-P1 residual box completer into production.
- Do not treat MuJoCo pinhole depth as hardware D455 evidence.
- Do not claim HE-1, PF-R10, DSIM, or PF-R7; those belong to `cursor/grok-4.6`
  or other owners.

## Unfinished

- LRF-A1 and LRF-PL1 remain planned and undispatched.
- Online container localization (P0-P2 geometric closed loop) is a proposal
  only; it was not implemented.
- Shared `master` is dirty with other agents' mailbox files; this identity
  did not checkpoint those.

## Pointers

- `docs/agents/discuss/2026-09-10_2121_workspace-archive-before-ros2-humble-rebaseline.md`
- `docs/agents/discuss/2026-09-10_1524_tcig-5-exact-metrics-g1.md`
- `docs/status/evidence/tcig5/2026-09-10_tcig-5-g1/`
- `docs/status/evidence/learning_research/LRF-P1/`
- `docs/plans/learning_research_feasibility.md`
