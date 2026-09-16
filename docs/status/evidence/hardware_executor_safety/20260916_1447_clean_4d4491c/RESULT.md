# HW-1 clean verification -- 2026-09-16

## Result

Pass for the authorized software scope at
`4d4491ca3ed365a1bb7f6399ca2619ee06aac58f`, dirty count zero. Real-cell
qualification was not authorized and remains `not_evaluated`.

## Commands and observations

- `PYTHONPATH=deployment_ws/src/elfin_trajectory_executor:${PYTHONPATH} python3 -m pytest -q deployment_ws/src/elfin_trajectory_executor/test`: 61 passed in 2.51 s.
- `colcon build --packages-select elfin_trajectory_executor --symlink-install`: pass.
- `colcon test --packages-select elfin_trajectory_executor`: 61 passed in 2.33 s.
- `colcon test-result --verbose`: command passed; this setuptools test runner does not emit parsed xUnit counts, while the retained colcon console result reports all 61 tests `OK`.
- `git status --porcelain`: empty after build/test because generated artifacts are ignored.
- `git diff --check`: pass.

The ROS action integration exercised ten accepted owner goals, ten overlapping
rejections, cancellation, an injected backend exception, ownership recovery,
and a succeeding post-error goal. Pure concurrency coverage ran 100 rounds of
eight simultaneous admission contenders and admitted exactly one each round.

## Contract-check exceptions

The primary workspace checker reports nine defects that predate HW-1:

- `docs/agents/eng/2026-09-14_1035_sim-r1-2-exploration-policy.md`: missing `started_at`, `completed_at`, `## Requirement`, and `## Result`.
- `docs/agents/eng/2026-09-14_2134_place-only-perfect-geometry.md`: missing `## Pointers`, `## Requirement`, and `## Result`.
- `docs/agents/test/2026-09-15_1041_pfr7-g4-offline.md`: missing `revision`.
- `docs/agents/test/2026-09-16_1018_pfr7-g9-stopack-g10-not-ready.md`: missing `revision`.

The clean checkout additionally lacks the ignored/untracked shared agent
documents, so its checker cannot certify that coordination tree. Neither set
of failures is modified or masked by HW-1.

## Hardware status

No CPS connection, robot enable, vacuum command, Gazebo process, or physical
motion occurred. Follow the committed hardware checklist only after a separate
operator authorization.
