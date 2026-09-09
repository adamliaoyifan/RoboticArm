# 2026-09-09 -- D555 HE-1 generation 2 dispatch

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Recorded the user-confirmed D555, Mid360, and EEF-flange mount interfaces in
the hand-eye plan and dispatched HE-1 generation 2 to `cursor/grok-4.6` at
exact plan revision `b45c4e875b76f28ffb323ef2faddc7ef8e9400c7`.

The revised method registers official D555 CAD to the inclined rectangular
bar through fitted hole axes and the seating plane, enumerates the four-hole
correspondences, rejects wrong orientations using the outward-facing optical
windows, Mid360 square opening, EEF two-hole interface, and collision checks,
then composes the mechanical result to the explicitly verified ROS
`d555_link`. It fails closed rather than treating an arbitrary CAD origin as
the camera root frame.

HE-1 generation 1 and HE-2 generation 1 were superseded because their plan
revision is stale. HE-2 generation 2 is recorded with `dispatch_ready: no` and
cannot run until the new HE-1 passes and the physical board is ready.

## Acceptance

- Official inputs and derived features are traceable with source hashes,
  units, feature labels, annotated renders, and numeric fit evidence.
- The selected D555 hole subset and all rejected correspondences are recorded;
  Mid360 and EEF-flange features cannot be used as camera datums.
- `^eef_mount_adapter T_d555_link` has an authoritative mechanical-to-ROS
  datum, numeric uncertainty, forward/inverse checks, and all plan geometry
  thresholds pass.
- Printable board, capture tooling, solve tooling, offline tests, evidence,
  implementation commit, eng note, and task closure are owned end to end by
  `cursor/grok-4.6`.
- No URDF, xacro, or YAML transform is applied by HE-1.

## Risks

- The official archive may define only an enclosure CAD origin and omit the
  left-IR/depth mechanical datum. That produces a blocked HE-1 result after
  preserving the valid mount-to-housing transform; visual inference is not an
  acceptable substitute.
- The four holes on the rectangular bar may contain more than one compatible
  subset. The operator constraints and complete assembly collision checks must
  resolve the symmetry, otherwise the result remains blocked.
- Cursor is file-only in the current runtime registry, so dispatch is durable
  in the mailbox but not a live RPC delivery.

## Pointers

- `docs/plans/d555_handeye_calibration.md`
- `docs/agents/discuss/2026-09-09_1218_d555-handeye-he-1-generation2.md`
- `docs/agents/discuss/2026-09-09_1219_d555-handeye-he-2-generation2.md`
- plan revision `b45c4e875b76f28ffb323ef2faddc7ef8e9400c7`
