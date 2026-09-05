# Platform-free Gate 4 Revision — detection availability split

> **Status: PROPOSAL — pending explicit user confirmation.**
> Recorded from an in-session user direction but not yet confirmed on
> the record; PF-R5 closure does NOT depend on this split (run8 passed
> the original unreduced gate).

Date: 2026-09-04 (proposal recorded by eng; supersedes nothing until
confirmed)

## Unchanged hard gates (no relaxation)

- All geometry error limits (top/support/height/XY/width/depth).
- FULL_3D rate among valid-top frames >= 0.95.
- false measured height = 0.
- Raw-only input fails closed.
- No online GT / spawner geometry reads in online nodes.
- Exact-stamp cargo/raw join, stamped status validation, stamped TF
  (no latest-TF fallback).
- GT reference fail-closed semantics.
- Evidence from one isolated clean worktree at an exact committed
  revision.
- PF-R6 >= 4 Hz active-window output.

## Revised structure

### PF-R5-GEO — platform-free geometry acceptance (blocking)

Scored on settled frames that have a valid semantic observation
(the observation itself must be production-path: semantic chain, no GT).
All geometry limits above apply on that population. The 30-trial matrix,
coverage gate, raw-only negative control, and revision evidence remain
mandatory.

### PF-R5-SIM-DETECTION — sim detection diagnostics (non-blocking)

Frame/trial recall of the flat-color STL suitcase visuals is reported as
a diagnostic: recall value, failing-trial distribution, failure frame
images (retained, never deleted), and the documented root cause
(YOLO-World open-vocabulary recall on untextured geometry; robot
base_link false-positive fill-in). This metric MUST NOT be used to claim
real-robot detection reliability.

### SIM 3-box closed-loop gate (blocking)

Fixed, detectable sim asset and pose; three consecutive boxes must
complete detection -> pick -> place with no GT fallback anywhere in the
loop (planning consumes the detected luggage; no GetCurrentBox geometry
in online paths). Validates the production state machine and closed-loop
usability.

### Representative detection hard gate (blocking for hardware release)

On textured simulation or a real rosbag, per-trial statistics:
- at least 30 trials;
- valid top / full geometry within a bounded time per trial;
- suggested pass bar: at least 29/30;
- scene/robot false positives producing valid geometry: 0.
Real-robot acceptance additionally requires Gate 5 (rosbag replay).

## Rationale

The single top_surface_rate gate conflated YOLO-World recall on
untextured geometry with the platform-free geometry contract. Splitting
recall out as a diagnostic preserves every geometry/contract gate while
making detection availability measurable on representative (textured or
real) data. Lowering 0.95 to the measured value was explicitly rejected.

## Engineering constraints discovered (2026-09-04)

- Gazebo Fortress ogre2 dies (camera sensor render thread wedges
  permanently) on RUNTIME import of textured meshes (DAE/OBJ): verified
  live — spawn succeeds, /camera/* stops publishing, deletion does not
  recover. The asset pipeline's STL + flat SDF material conversion
  exists precisely because of this.
- Textured simulation therefore requires world-preloaded textured
  models (loaded at world init, not runtime import), with the spawner
  placing them via set_pose from a parking pose.
