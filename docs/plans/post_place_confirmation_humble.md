# Humble measured post-place confirmation

- parent: `POST-PLACE-CONFIRM-20260929`
- approved_by: user prompt of 2026-09-29
- base_revision: `763361d339d2a8df11363d1d1f76118c1b6183b9`

## Scope and invariants

Implement the closed loop `planned -> executed -> released -> observed ->
measured -> stable -> safety revalidated -> committed`. Integrate ROS 2
bringup commit `98db923` first. Product code must not import, install, copy, or
modify ROS 1/reference implementations. Both sim and hardware authorize commit
from stamped `/luggage/semantic/cargo_points_untracked`; Gazebo truth is
offline scoring only. A safe measured deviation is committed at its measured
pose. The fixed `post_place_verify` view may be followed by at most two
unvisited active views. Stability uses a 0.8--3.0 s adaptive window.

Interfaces comprise `PlacedBoxVerification.msg`, extended
`VerifyPlacedBox.srv`, `ConfirmPlacement.action`, and
`CommitVerifiedBox.srv`. `AddPlacedBox`/`RemovePlacedBox` gain stable box IDs;
fixture-only planned commits require explicit `SOURCE_FIXTURE`. Production
planning consumes the revision/hash-consistent verified ledger and rejects a
non-empty compatibility `ComputePlacement.request.placed` list.

The ROS-free estimator searches planned XY +0.15 m and Z +0.20 m, downsamples
at 5 mm, uses 12 mm top-plane tolerance and at least 100 top points, subtracts
the pre-place surface, rejects ambiguous candidates and unordered pickup/post
footprint differences over 60 mm, and computes Z from measured top minus half
pickup height. Conservative W/D are max(pickup, measured) plus 20 mm per side;
height adds 10 mm and its centre moves up 5 mm.

Early stability requires three estimates near 0/0.4/0.8 s, centre drift <=10
mm, yaw drift <=1 degree, and non-increasing change. Otherwise extend through
3.0 s and require <=20 mm / <=2 degrees. Planned tolerance is XY 40 mm, Z 30
mm, yaw 5 degrees and is diagnostic only. Safety additionally requires
roll/pitch <=2 degrees, seven-face-hull containment with >=10 mm lateral
margin, overlap <=1e-5 m3, support error <= one voxel, LBCP support with 15%
short-side CoG margin, unchanged map identity, and scene/map/verification ID
and geometry agreement.

Commit is a CAS transaction: add conservative actual scene object by box ID,
commit the same verified geometry against the original revision/hash, read
back equality, integrate the accepted view, and compensate any partial
failure. CAS failure removes the scene object and requires full
re-verification. Only `committed && safe_to_continue` advances or finalizes.

## Acceptance and evidence

Focused tests cover nominal, offsets/yaw, swapped axes, neighbour clutter,
occlusion, stale cloud, missing TF, wrong identity, insufficient/ambiguous
points, drift/settling, tilt/hull/overlap/support failures, one/two recovery
views, transactional failures and generated ROS 2 interface round trips. The
full build/test commands are the user-specified seven-package `colcon build`
and five-package `colcon test`, followed by `colcon test-result --verbose`.
Product manifests/install sources must have zero `rospy|roslib|actionlib|catkin`
matches.

Estimator gates: XY p95 <=20 mm, Z p95 <=15 mm, yaw p95 <=3 degrees; wrong-box
association, planned production commit, unsafe continuation, and scene/map
split-brain are all zero. Latency p95 gates are CPU 100 ms, normal adaptive
confirmation 3.5 s, extended confirmation 5.5 s, fixed-view release-to-commit
15 s, one/two active views 27/40 s, and map+scene commit 200 ms. No samples
means `not_evaluated`, never substitution by a timeout.

Sim scoring freezes revision/config/dirty hash, then runs
`scripts/post_place_confirmation_run.sh --profile sim --campaign scored_30`:
30 units (10 each size; 12 nominal, 9 safe deviations, 9 recovery-view), at
least 28/30, and all four zero-count safety gates. This is a binomial AND-style
campaign intended to distinguish p=.95 from p=.80 (broken pass probability
about 4.4%, healthy about 81.2%). Twelve named deterministic fault cases all
must pass. Production nodes run with Gazebo truth disconnected. Each failed
case must preserve the T2 boundary bundle required by
`.cursor/rules/debug-evidence.mdc` before teardown/rerun.

Hardware scoring freezes the same identities, then runs the `d555_site`
`hardware_8` campaign: sizes 3/3/2, >=7/8, zero unsafe/wrong commits, plus
three measured fixtures for deviation, recovery view, and safe-out-of-plan.
This distinguishes p<=.5 from p>=.9 but cannot establish a stronger claim.
Failures preserve two cloud windows, RGB/depth, TF/joints, pre/post map,
ledger, verification JSON, scene object, action trace and offline score.

Failure bundles live under `docs/status/evidence/post_place_confirmation/`.
Always-on traces contain request/box/verification IDs, stages, stamps, map
identity, point counts and timings. Perception failures trigger raw cloud,
RGB/depth, TF, pre-map, candidates and estimate for the bounded 0.5 s before
release through 3.5 s after the last accepted view. Commit failures additionally
trigger scene before/after, ledger before/after, service/action messages and
rollback trace. JSON/NPZ/PCD/rosbag2 are bounded to the failed case; manifest
records sizes, missing items, capture completeness and replay command.

## Subtasks

| ID | Owner | Depends | Base | Scope | Acceptance / tests | Evidence |
|---|---|---|---|---|---|---|
| ST-1 | `codex/gpt-5` | none | `763361d` | preserve dirty manifest; integrate `98db923` in isolated worktree | baseline build/test and zero ROS1 product matches | passing commit + baseline logs |
| ST-2 | `codex/gpt-5` | ST-1 | ST-1 result | interfaces, ROS-free estimator/stability/safety policy | focused algorithm and generated-interface tests above | passing commit + focused logs |
| ST-3 | `codex/gpt-5` | ST-2 | ST-2 result | verified ledger CAS, scene compensation and orchestrator action | transaction/orchestrator fault matrix | passing commit + focused logs |
| ST-4 | `codex/gpt-5` | ST-3 | ST-3 result | sim adapters, fixtures, campaign/evidence runner | non-scored launch gates then scored_30 criteria | frozen revision + evidence bundle |
| ST-5 | `codex/gpt-5` | ST-3 | ST-3 result | D555 pose/profile and hardware evidence plumbing | hardware_8 criteria; physical execution may be `not_evaluated` without cell | frozen revision + evidence bundle |
| INTEGRATION | `codex/gpt-5` | ST-4,ST-5 | both results | full build/regression and docs; close only demonstrated RS scope | all offline/available live gates and explicit not_evaluated claims | integration commit + summary |

RS-2, RS-3 and RS-6 close only after both scored sim and hardware gates pass.
RS-7 closes only for the evidenced post-place portion; initial exploration
requires separate Humble evidence.
