# G1 result: upward carry and support transfer has geometry candidates

Experiment code: `5bc9f85`; plan: `45a50252`; source base: `c7039905`.
Branch: `research/uld-upper-layer-g1`. The experiment used a clean isolated
worktree and completed one deterministic 864-case matrix in about 31 s.
This is an offline geometry/pose study, not execution success-rate evidence.

## Observed results

| Box | Center contact candidates | Rear-offset candidates |
|---|---:|---:|
| 0.55 x 0.40 x 0.25 m | 0 / 144 | 36 / 144 |
| 0.70 x 0.45 x 0.28 m | 0 / 144 | 64 / 144 |
| 0.80 x 0.50 x 0.32 m | 0 / 144 | 68 / 144 |

168 fixed-tool geometry candidates, 612 empty conservative handoff windows,
84 aperture-clearance rejections. All cases were classified and captured.
These are parameter combinations, not repeated random trials.

For central COM, long-edge insertion, rear contact, tool yaw 90 degrees,
3 cm ceiling gap and support starting at the opening, the allowed insertion
intervals are approximately:

| Length | Minimum for static support | Maximum before tool clearance loss | Window |
|---|---:|---:|---:|
| 55 cm | 29.5 cm | 35.0 cm | 5.5 cm |
| 70 cm | 37.0 cm | 50.0 cm | 13.0 cm |
| 80 cm | 42.0 cm | 60.0 cm | 18.0 cm |

These include the declared 2 cm COM margin and 5 mm tool/obstacle clearance.
The whole fixed tool, including sensors and adapter, was checked using
conservative per-mesh boxes. Full-arm geometry was not checked.

For COM fractions 0.4/0.5/0.6 from the leading edge, rear-grasp candidate
counts were carryon 24/12/0, standard 28/24/12, large 28/28/12, each out of
48 combinations. The small case is particularly sensitive to rear-heavy
contents. No center-grasp case passed this conservative G1 model.

The frozen largest-window selection chose six cases (one per size/entry-axis
pair), all with COM fraction 0.4, 3 cm ceiling gap, rear grasp and yaw 90.
All five tested poses in each case had joint-limited IK solutions: upward
approach, handoff, withdrawal, rear push start and end. This is 30 pose
solutions for selected favorable cases; it does not establish central-COM
IK coverage, collision-free arm poses, or continuous paths.

## Interpretation and next boundary

The user-proposed action is not ruled out by the existing fixed tool's size.
Rear-offset contact supplies a handoff window that centered contact lacked
under this model. Existing downward-only controls were never called.

The remaining gate is the complete moving arm and transition path, followed
by an independently configured contact-physics experiment. In particular,
withdrawal-to-rear-push orientation change is explicitly unchecked. Outside
pickup and inversion, load/peeling limits, arbitrary real cargo COM, friction,
deformable/movable support and visual occlusion also remain untested. No
production fallback routing has been installed.

## Validation and evidence

- Ten behavioral unit tests pass. One earlier fixture identity error was
  preserved and corrected before the experiment; collision rules stayed strict.
- 864 case records; six IK trace files; expanded source URDF and input hashes;
  geometry snapshot, summary, manifest and four-stage PNG/SVG were saved.
- Experiment evidence is about 1.7 MB; capture_complete=true; no simulator ran.
- `git diff --check` passes in the isolated worktree.
- Shared agent-contract checker has 146 existing diagnostics, none naming
  this task. The full file-by-file diagnostics are retained; no global
  contract-compliance claim is made.

Primary-workspace evidence:
`docs/status/evidence/uld_upper_layer_feasibility/g1_20261110_01/`.
Development diagnostics:
`docs/status/evidence/uld_upper_layer_feasibility/g1_development/`.
