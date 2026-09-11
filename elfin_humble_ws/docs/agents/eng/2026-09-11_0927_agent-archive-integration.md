# 2026-09-11 -- Agent archive integration

- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Integrated the useful archived agent work into a clean monorepo worktree based on `origin/ros2_humble@e25c7c9`. Existing snapshot content was retained, duplicate TCIG-2/TCIG-5/TCIG-7/SIM-R1 commits were not replayed, and the mixed 969-file WIP recovery branch was not merged.

## Changed

- Preserved shared-workspace PF-R10 perception and Gate-4 dump changes, HE-1 calibration tooling, pendant MCAP replay parsing, sampled replay evidence, lifecycle notes, and the one-simulator rule.
- Added the production TCIG-4 hull-eroded insertion corridor and its focused evidence.
- Added LRF-P1 as isolated research with its D455/MuJoCo checkpoint; no production dependency was introduced.
- Restored the required `.cursor/rules/` contract files omitted by the earlier snapshot while keeping excluded suction-panel archives and large candidate ZIP files absent.

## Verification

- Pendant replay, PF-R10 perception, and HE-1 focused pytest: `201 passed, 1 skipped`.
- TCIG-4 focused and luggage-packing regression: `144 passed`.
- LRF-P1 unittest with `MUJOCO_GL=egl`: `19 passed`.
- `scripts/check_agent_contract.sh`: pass.
- `git diff --check`: pass.
- No Gazebo stack was started.

## Pointers

- `docs/agents/discuss/2026-09-10_2121_workspace-archive-before-ros2-humble-rebaseline.md`
- `docs/agents/discuss/2026-09-10_2122_pendant-bag-replay-review-request.md`
- `docs/status/evidence/true_container_inner_geometry/d567ad52571abb2d351c06f44c709f713a0de97f/g4/`
- `docs/status/evidence/pendant_replay/RESULT.md`
- `docs/status/evidence/learning_research/LRF-P1/`
