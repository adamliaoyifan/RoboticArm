# 2026-09-09 -- D555 hand-eye dispatch and F3 evidence fold-in

- role: reviews
- agent: cursor
- model: opus5
- cli: cursor
- status: done

## Summary

Two actions on the back of HB-1/HB-2/HB-3 passing at
`40ab61ca0f6d5237f26091d946c05227bb6fc468`.

First, folded the hardware evidence into the open F3 consensus thread
`Q-20260908-3`. Three of that audit's questions now have measured answers.
The split-optics question resolves as "the driver publishes both": aligned
depth in `d555_color_optical_frame` with colour K, native depth and
`/depth/color/points` in `d555_depth_optical_frame` with depth K, each
internally consistent, so the contract must choose rather than discover.
Full-cloud transport measures about 513 Mbps of roughly 701 Mbps total on the
1 GbE PoE link, and the recorded 896x504 at 30 Hz failure computes to about
2.17 Gbps, so the DDS device drop was link saturation. The hardware cloud is
unorganized with `width * point_step != row_step`, which means PF-R9's stride-2
path degrades to flat indexing on hardware and the current cloud path does not
transfer at all.

Also supplied the full-cloud consumer inventory the audit needs for its
removal-or-migration acceptance item. There are exactly two production
consumers. `semantic_point_filter` is fully replaceable and simplified: under
colour-aligned depth its depth-to-colour projection becomes the identity, so
the extrinsic, the projection, the rounding, and the project-outside-image loss
all disappear. `luggage_detector._raw_cloud_cb` is the real gap, because the
support-plane RANSAC needs an annulus outside the cargo mask at
`support_inner_margin` 0.03 to `support_outer_margin` 0.18 m. Two candidate
replacements were offered, with decimated full deprojection recommended over
dilated mask because `min_support_points` is only 80 and it needs no
range-dependent parameter.

Second, wrote `docs/plans/d555_handeye_calibration.md` and dispatched HE-1 and
HE-2 to `eng/cursor/grok-4.6` at plan revision
`271267cc86be5da04a499ea4f64cadc54d72bc05`.

The board geometry was derived from HB-1's measured intrinsics rather than a
datasheet, and that derivation changed the plan. At the measured 640x360
`fx 323.18`, a 37.5 mm ArUco marker is about 12 px at 1.0 m and will not
decode. The plan therefore specifies capture at 1280 colour where `fx` is about
646, which is legitimate because `T_flange_camera` is a rigid-body transform
and resolution independent, and which is affordable because disabling the
point cloud frees about 513 Mbps.

## Acceptance

- HE-1: CAD seed with a stated derivation method and uncertainty from
  `arm_realsense_v1.3.stl`, its delta against the GUI-tuned value, a true-scale
  self-describing printable board, and offline-runnable capture and solve
  scripts.
- HE-2: multi-method `cv2.calibrateHandEye` spread, rotation-deg and
  translation-mm residuals, 5-pose holdout error, CAD agreement within 10 mm,
  and the HB-3 plane residual re-measured with the new transform showing the
  ~2 deg angle collapse.

## Consensus

- Codex agent: `codex-f3-consensus`
- Thread: `docs/agents/discuss/2026-09-08_1651_f3-depth-primary-contract-consensus.md`
- Result: not required for this dispatch

HE-1 and HE-2 are measurement and asset generation. Neither writes the
calibrated transform into URDF, xacro, or yaml, and neither touches a message
contract or an acceptance bar. Applying the result is a separate change that
must also resolve the `realsense_d435.yaml` contradiction.

## Subtasks

| ID | Owner agent/model | Depends on | Scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| HE-1 | `cursor/grok-4.6` | none | CAD seed plus ChArUco assets, offline | Seed with derivation, true-scale board, runnable scripts | Offline geometry checks |
| HE-2 | `cursor/grok-4.6` | HE-1 | Capture, solve, validate | Residuals, holdout, CAD agreement, HB-3 collapse | Multi-solver plus independent re-measurement |

## Risks

- Pose diversity is an observability requirement, not a preference. Rotation in
  `AX = XB` is unobservable when poses differ only by translation or share one
  rotation axis. HB-3's three floor-viewing poses were that degenerate case, so
  the plan mandates at least 3 distinct rotation axes with at least 30 deg
  between extremes.
- `/joint_states` from the Sep-2 executor published all zeros, per HB-3. HE-2
  must read CPS `HRIF_ReadActACS`. Using the topic would silently corrupt every
  captured pose.
- Board flatness and print scale are the usual silent error sources. A 1 mm bow
  over 400 mm is absorbed as extrinsic error, and a 1 percent print scale error
  maps one-to-one onto translation, about 5 mm at a 0.5 m lever. Both are
  written into the plan as hard requirements.
- `arm_realsense_v1.3.stl` is a mesh, not parametric CAD, so the seed carries
  an uncertainty that HE-1 must state rather than assume away.
- A good solver residual on the calibration set does not demonstrate
  correctness. The HB-3 re-measurement is the only acceptance criterion
  independent of the calibration data.

## Pointers

- `docs/plans/d555_handeye_calibration.md`
- `docs/agents/discuss/2026-09-09_1026_d555-handeye-he-1.md`
- `docs/agents/discuss/2026-09-09_1026_d555-handeye-he-2.md`
- `docs/agents/discuss/2026-09-08_1651_f3-depth-primary-contract-consensus.md`
- `docs/status/evidence/d555_bringup/README.md`
- plan revision `271267cc86be5da04a499ea4f64cadc54d72bc05`
