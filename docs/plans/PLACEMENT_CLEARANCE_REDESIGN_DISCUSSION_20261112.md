# Placement clearance redesign discussion -- 2026-11-12

- status: draft
- owner: reviews/codex/gpt-6
- base_revision: c3f495a3
- dispatch_ready: no

## Confirmed user decisions

Final settled neighbouring cargo may touch; no solid penetration or pushing
cargo into place. This permits nominal contact proposals, not intentional
side contact during carried motion or blanket allowed-collision pairs.
The user now explicitly reopens the earlier5cm release gap decision: height
and method are to be validated and discussed, not fixed by this draft.
No behavior changes or physical execution are authorized by this document.

## Scope and inventory

| Item | Current source behavior | Proposed decision boundary |
|---|---|---|
| collision_margin_m | default0.05m; native O volume boxes grow on every face | Retire unexplained blanket distance; explicit uncertainty accounting and residual clearance, not arbitrary replacement |
| same_layer_clearance_m | solver default0.05m | No mandatory process gap required by user; nominal contact may be proposed, execution feasibility still decides |
| structural gap_min | default0.05m proposal spacing | Must consume the same process-gap policy as exhaustive candidates |
| occupancy inflate_m | legacy path default0.05m; XY footprint/obstacle growth | Trace effective call paths and possible repeated coverage; do not assume cumulative inflation without evidence |
| carry_margin_m | default0.05m above tops; required carry gap=max(carry_margin,collision_pad) | Separate route-height proposal from final placement and measurement uncertainty |
| support_tol | default0.05m peak-height/parent-layer tolerance | Separate support classification policy; no evidence that surfaces5cm apart are mechanically supporting |
| RELEASE_DROP_M | default0.05m box-bottom gap, vacuum-off fall | Reopened; examine error, landing displacement and ability to certify empty retreat |
| other0.05 uses | grid/search thresholds, model size consistency, time periods or tilt ratios | Inventory by units/consumer; not blanket replaced as5cm clearance |

Source defaults are not claims of active deployed overrides. Current O default
resolution is1cm;2cm is an experiment variant, not a user-selected live change.

## Proposed independent policies

1. Layout process gap: requested nominal geometric separation. User requires
   no universal positive minimum. Nominal exact contact remains a candidate;
   a chosen pose must separately satisfy uncertainty-aware trajectory/settling
   checks. No promise that every real box can be placed at exactly zero gap.
2. Geometry uncertainty: each contribution has owner, evidence, frame, axes,
   object identity, acquisition time and validity. Keep voxel quantization,
   observed surfaces, box dimensions/pose, grasp transform and robot tracking
   distinct. Full O cells already contain their covered cell volume; do not
   add a second full-cell allowance for the same quantization. Different
   independent errors may accumulate. Correlation must not be guessed; no
   unjustified root-sum-square error reduction. Existing10%/5deg/5mm attachment
   policy is also an uncalibrated bound and must enter this ledger, not silently
   stack with a new generic margin. No zero sensor error inferred from missing
   calibration or a currently configured zero term.
3. Motion clearance: certify actual robot/tool and owned attachment envelope
   against occupied and unknown space, distinguishing collision pairs and
   phase. Optional preferred clearance ranks proposals; hard geometric and
   uncertainty requirements determine feasibility. A near-obstacle path is
   not rejected solely for missing an arbitrary comfort distance. No contact
   pushing, new unknown exemption, planned-AABB carving or blanket ACM change.
4. Support/settling/release: explicit known support identity and local shape;
   separate observed height variation from plane-estimation uncertainty.
   Bottom support contact after release is distinct from side collision during
   carrying. No force/current/contact signal is assumed. Lower noncontact
   release may be explored using observed height, actual geometry and execution
   bounds. Zero-gap contact descent or force-guided insertion is not implied.
   Positive free fall can change final pose: nominal final contact plus no
   pushing cannot prove a measured zero-gap landing without settling evidence.

All policies are proposed identities/semantics, not final numeric values.
O-to-native conversion, cheap ranking, placement generation, atlas consuming
paths, full certification and runtime revalidation must reference compatible
versioned policies. Do not leave old5cm defaults in a fallback or duplicate
uncertainty on both sides without an explicit owner and reason.
An atlas remains a static seed/quality provider, never current-O certification.

## Prior boundary

Keep optional speculative placement sidecar separate from measured-O slot
selection. Disabling speculative prefetch is possible but not required to
remove blanket5cm. Shared pure geometry/transform/snapshot helpers remain until
consumers are migrated. No wholesale deletion of placement_prior_* modules.

## Ordered review checkpoints

1. Close inventory: owner, units, caller, phase, effective source/install/launch
   value and evidence for every clearance/support/release term. Export the
   effective configuration at preparation and runtime; do not infer live state
   from defaults. User has decided contact objective and release review scope.
2. Review uncertainty ledger and proposed minimal geometric policy with user.
   Existing pickup bags can support repeatability only, not absolute accuracy
   or release/multibox misassociation. Mark unavailable bounds unknown; never
   manufacture calibrated values or convert unknown error to zero. Define
   which cases are offline geometry-only and what further measured data would
   qualify actual deployment before assigning replacement distances.
3. Decide release method/height and support rules separately with user. Keep
   capability requirements concrete; do not rely on unconfirmed force/current,
   precise contacts or independent sensors. Record unsettled/drop cases and
   inability to certify retreat as explicit negative outcomes.
4. After semantics and numeric acceptance are agreed, supersede affected
   claimed tasks with a higher generation; implement one shared policy and
   remove hidden5cm fallback behaviors. This draft is not dispatch-ready and
   does not rewrite generation2 integration acceptance.
5. Reproduce on frozen actual deploy_update full robot/tool and current O.
   Change one policy component per comparison. Preserve same raw acquisitions,
   box identity, attachment and original timed route for attribution; subsequent
   replanning is a separate paired comparison with equal input/budget. Unknown
   coverage must be reported separately when grid resolution changes.

## Required behavioral cases for the later executable plan

Exact nominal contact vs positive penetration; demanded positive process gap
vs zero process gap; same geometry through exhaustive/structural candidate and
full preparation/runtime paths; nominal box vs conservative envelope; support
contact vs neighbour side collision; observed occupied vs unknown rejection;
rotated boxes and off-centre fixed grasps; release clears cup but retreat still
blocked; unobstructed complete pick/place/release/retreat witness on full model.
Each must record actual offending bodies, occupied/unknown contributors,
minimum distance or penetration, named budget terms, pose/attachment and policy
identities. Geometry pass never substitutes for executed settling evidence.
Exact input matrix, metrics/thresholds, repetition/aggregation, commands,
artifacts and revision rule must be fixed before runnable dispatch. Calibration
and release thresholds are deliberately undecided here; no reliability or
latency improvement claim is authorized by this draft.

## Pointers

- docs/agents/reviews/2026-11-12_1236_occupancy-margin-provenance.md
- docs/agents/reviews/2026-11-12_1238_prior-purpose-and-removal-boundary.md
- docs/agents/eng/2026-11-12_1206_release-occupancy-resolution.md
- docs/architecture/placement.md
- src/luggage_planning/luggage_planning/occupancy_place_paths.py
- src/luggage_planning/luggage_planning/placement_prior_snapshot.py
- src/luggage_planning/luggage_planning/placement_prior_requests.py
- src/luggage_planning/config/attachment_geometry.yaml
- src/luggage_packing/luggage_packing/placement_solver.py
- src/luggage_packing/luggage_packing/placement_structural_candidates.py
