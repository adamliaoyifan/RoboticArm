# 2026-09-09 -- HE-1 generation 2 -- D555 mechanical registration

- status: open
- to_role: eng
- to_agent: cursor
- to_model: grok-4.6
- kind: subtask
- parent: D555-HANDEYE-20260909
- subtask: HE-1
- depends_on: none
- revision: b45c4e875b76f28ffb323ef2faddc7ef8e9400c7
- generation: 2
- plan_revision: b45c4e875b76f28ffb323ef2faddc7ef8e9400c7
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-09-09 12:18 -- codex/gpt-5

User-directed HE-1 replacement; no consensus action is required. Read the complete HE-1 mechanical facts, frame contract, steps A-F, pass condition, and out-of-scope section in docs/plans/d555_handeye_calibration.md at exact plan revision b45c4e875b76f28ffb323ef2faddc7ef8e9400c7. Claim this thread with scripts/agent_start.sh before editing and own acquisition, implementation, tests, fixes, evidence, commit, role note, and closure end to end. The operator confirms: D555 fastens on the inclined elongated rectangular bar, which has four holes total; D555 bottom is parallel to that mounting region; optical windows face outward; the separate square opening is for Mid360; and another distinct two-hole interface connects the mount to the EEF flange. Determine from official CAD which subset of the bar's four holes actually mounts D555; do not confuse the Mid360 feature or EEF flange holes with camera datums. Report the transform specifically as ^eef_mount_adapter T_d555_link and fail closed if the official mechanical-to-ROS d555_link datum cannot be established. Do not edit URDF, xacro, or YAML.

## Pointers

- `docs/plans/d555_handeye_calibration.md`
- `src/luggage_gazebo/models/arm_realsense/arm_realsense_v1.3.stl`
- `src/luggage_description/urdf/eef_sensor_mount.urdf.xacro`
- `src/luggage_description/config/camera_mount_origin.xacro`

## Open

- Execute HE-1 generation 2 end to end against plan revision b45c4e8: derive and validate eef_mount_adapter to d555_link from official D555 CAD, the mount STL, and the operator-confirmed assembly constraints; emit calibration assets and evidence; do not apply the transform.
## Claim -- eng/cursor -- 2026-09-09 12:27 -- cursor/grok-4.6

- started_at: 2026-09-09T12:27:22+08:00
- claimed_generation: 2
- claimed_plan_revision: b45c4e875b76f28ffb323ef2faddc7ef8e9400c7
- claimed_dependencies: none

## Result -- eng/cursor -- 2026-09-09 13:01 -- cursor/grok-4.6

- outcome: blocked
- completed_at: 2026-09-09T13:01:01+08:00
- summary: Unique STL_BAR_H0/H3 mechanical seed derived; official D555 CAD/datasheet give no housing-to-left-IR datum, so eef_mount_adapter T_d555_link is not reported. Board and capture/solve scripts emitted. No URDF edits.
- evidence: docs/status/evidence/d555_handeye/20260909_he1_g2/
