# 2026-09-08 -- Dispatch D555 hardware bring-up verification

- role: reviews
- agent: cursor
- model: opus5
- cli: cursor
- status: done

## Summary

A hardware visualisation showed the D555 RGBD cloud roughly coplanar with
Mid-360 floor points. That rules out a gross mount error, because the two
sensors reach the world through different mount transforms, so a badly wrong
camera extrinsic would separate the clouds. It cannot answer whether RGB and
depth are aligned, for three reasons: the RGBD surface rendered monochrome
grey, which is equally consistent with a grey floor and with an RViz
`Color Transformer` set to FlatColor or Intensity; the error class in question
is 6-9 cm on a 95 mm baseline and is not resolvable at that zoom; and
alignment is internal to the camera, affecting which colour lands on which
point and which frame the cloud occupies rather than gross placement.

Wrote `docs/plans/d555_hardware_bringup_verification.md` and committed it at
`0f392ef6b06ba1bde6328609edf3c99a11160a42` to obtain a real `plan_revision`,
then dispatched three read-only subtasks to `eng/cursor/grok-4.6`.

The same commit demotes `docs/plans/d455_replace_d435/` to reference, corrects
the two false same-pocket claims, and records that the mount transform's
provenance is `eof_mount_stack_tune_gui.py` visual tuning against a D435 body
origin rather than metrology.

## Acceptance

- HB-1: all nine device-fact items recorded, with the frame-versus-K
  consistency stated as an explicit consistent-or-inconsistent verdict and the
  ROS distro question answered plainly.
- HB-2: signed u and v pixel offsets at approximately 0.6 m, 1.0 m, and 2.0 m,
  their metric conversion, and an evidence-backed colour-aligned or
  depth-native verdict. A quantified null result passes; "looks right" does not.
- HB-3: nine numbers (three poses times pose id, plane offset in mm, plane
  angle in deg), the translation-versus-rotation conclusion, and a verdict on
  whether the observed Mid-360 edge doubling is RViz `Decay Time` or
  absent-deskew motion distortion.

## Consensus

- Codex agent: `codex-reviews-main` and `codex-f3-consensus`
- Thread: `docs/agents/discuss/2026-09-08_1643_d455-sim-parity-and-min-z-conflict.md`
- Thread: `docs/agents/discuss/2026-09-08_1651_f3-depth-primary-contract-consensus.md`
- Result: not required for this dispatch

These three subtasks are read-only fact-finding. They change no code, no URDF,
no message contract, and no acceptance bar, and their output is the evidence
those two threads are blocked on. Dispatching after they close would deadlock:
`Q-20260908-2` needs the measured mount residual and the distro answer, and
`Q-20260908-3` needs the colour-aligned versus depth-native verdict. Any
resulting change to geometry, contracts, or bars remains owned by those
threads and is explicitly out of scope in the plan.

## Subtasks

| ID | Owner agent/model | Depends on | Scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| HB-1 | `cursor/grok-4.6` | none | D555 device fact dump, arm stationary | Nine items plus explicit frame-versus-K verdict | Read-only topic, TF, parameter inspection |
| HB-2 | `cursor/grok-4.6` | HB-1 | RGB-to-depth alignment by target overlay | Signed pixel offsets at three depths plus verdict | Target overlay at known distances |
| HB-3 | `cursor/grok-4.6` | HB-1 | Mount residual against Mid-360 at three poses | Nine numbers plus error-type and smearing verdicts | Plane fitting on both clouds |

## Risks

- HB-1 needs no arm motion; HB-2 and HB-3 do. The plan restricts them to static
  poses commanded one at a time with the operator present, read-only, low
  speed, small amplitude, no packing cycle, and no gripper command.
- The plan forbids writing a corrected mount transform. HB-3 measures the
  residual only. Applying a fix before CAD seeding and a hand-eye run would
  bake a GUI-tuned error into TF.
- `realsense_d435.yaml` is internally inconsistent: `mount.tune_joints` is
  `rx -1.3634512, ry 0.0376991, rz 1.5707963` while `mount.fixed.rpy` is
  `0.0376991, 1.3634512, 1.5707963`, with roll and pitch swapped and a sign
  change. Owners must report it, not edit it. Using either value as a
  calibration seed before resolving it risks baking in a transpose error.
- The camera mass in that yaml is `0.072` against a D555 datasheet 337 g. The
  extra 265 g at the wrist affects arm payload margin and suction-panel
  dynamics. Not in scope here; recorded so it is not lost.
- If HB-1 finds the frame and K inconsistent, cargo geometry measured to date
  inherits a 6-9 cm class error and prior PF-R hardware conclusions would need
  review.

## Pointers

- `docs/plans/d555_hardware_bringup_verification.md`
- `docs/agents/discuss/2026-09-08_1714_d555-bringup-hb-1.md`
- `docs/agents/discuss/2026-09-08_1714_d555-bringup-hb-2.md`
- `docs/agents/discuss/2026-09-08_1714_d555-bringup-hb-3.md`
- `src/luggage_description/config/camera_mount_origin.xacro`
- `src/luggage_description/config/realsense_d435.yaml`
- plan revision `0f392ef6b06ba1bde6328609edf3c99a11160a42`
