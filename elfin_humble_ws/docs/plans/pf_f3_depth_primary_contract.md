# PF-F3 — depth-primary sensor contract

Date: 2026-09-09 (reviews revision after consensus round 1, `cursor/opus5`)
Parent: `PFH-REMEDIATION-20260904`
Base revision: `271267cc86be5da04a499ea4f64cadc54d72bc05`
Consensus thread: `docs/agents/discuss/2026-09-08_1651_f3-depth-primary-contract-consensus.md`

Status: **not dispatchable**. This revision exists to be audited in consensus
round 2. `dispatch_ready` stays `no` on every row until
`consensus: reached` is recorded on the thread above.

## 1. Why this plan exists

PF-R9 generation 1 passed B1, B2, B4, B5, and B6 and failed B3 by measurement
(0.462 emitted/RGB against 0.8; `raw_img->pre_rgb` p50 256 ms against 60 ms).
That fired the contingency written into
`docs/plans/pf_r8_r9_perception_acceptance.md` section 3, which named
pixel-space masking as out of scope and requiring an amendment. This plan is
that amendment.

Consensus round 1
(`docs/agents/reviews/2026-09-09_1033_f3-depth-primary-consensus-round1.md`)
accepted the direction and blocked dispatch pending seven concrete plan
constraints. Sections 4 through 10 bind those seven. Section 3 records a
transport-budget problem the round-1 audit did not cover; it is the reason
this plan does not simply re-issue the B3 bars unchanged.

### What is already settled

- Depth-primary is the production contract. The full camera cloud is not
  transported or decoded.
- The canonical grid is colour-aligned depth.
- Support geometry uses candidate B-ii (bounded local decimated full-grid
  deprojection), with B-i (range-dependent mask dilation) as a measured
  fallback only.

No requirement-level objection remains to any of these.

## 2. Hardware facts this plan is built on

Measured in HB-1/HB-2/HB-3 (`docs/status/evidence/d555_bringup/`, revision
`40ab61ca0f6d5237f26091d946c05227bb6fc468`), not from datasheets.

| Fact | Value | Consequence |
|---|---|---|
| Stable profile | 640x360 @ 15 Hz | Any bar in absolute Hz or ms needs a hardware variant |
| Colour image | rgb8, step 1920 | 691,200 B/frame |
| Aligned depth | **16UC1**, step 1280, `d555_color_optical_frame`, colour K | 460,800 B/frame; same encoding as the sim canonical depth |
| Native depth / `depth/color/points` | `d555_depth_optical_frame`, depth K | Not legal canonical inputs |
| Cloud share of link | ~513 of ~701 Mbps on 1 GbE PoE | Deleting it removes 73 % of camera network load |
| Cloud header | `height 1`, `width 205662`, `row_step 4,608,000` | Self-inconsistent and unorganized; the PF-R9 stride-2 path does not transfer to hardware at all |
| 896x504 @ 30 Hz | ~2.17 Gbps, DDS device drop | Payload does not degrade gracefully, it de-enumerates the camera |
| Mount extrinsic | ~2 deg rotation error, all three poses | World-frame geometry accuracy is not a valid F3 acceptance input until HE-2 closes |

The 16UC1 match between hardware aligned depth and the sim canonical depth is
what makes one canonical grid definition serve both backends.

## 3. The transport budget — the constraint round 1 did not cover

Round-1 item 4 says to keep the B3 bars (emitted/unique-RGB >= 0.8,
`raw_img->pre_rgb` p50 <= 60 ms). Binding them unchanged, with no other
requirement, would very likely reproduce the generation-1 failure. The
arithmetic has to be in the plan rather than discovered after dispatch.

PF-R9 generation 1 measured a publisher-thread period of ~90 ms for ~1.8 MB
per emission, i.e. ~20 MB/s of effective publish throughput. That 1.8 MB was
colour 0.92 MB plus stride-2 cloud 0.9 MB. The 0.61 MB depth image was
**skipped**, because nothing subscribed to it.

Under a depth-primary contract the depth image stops being optional: it is the
canonical product. So the sim per-emission payload does not fall from 1.8 MB
to 0.9 MB. It falls to:

| Product | 640x480 sim | 640x360 hardware |
|---|---|---|
| colour rgb8 | 921,600 B | 691,200 B |
| depth 16UC1 | 614,400 B | 460,800 B |
| **paired total** | **1.536 MB** | **1.152 MB** |

At the generation-1 observed ~20 MB/s that is ~76 ms per emission, about
13 Hz, or **0.55x** — still under the 0.8 bar. Payload reduction alone does
not clear D3.

What makes the bar reachable is that the ~20 MB/s is not a DDS wire limit. It
is dominated by Python-side per-publish work that depth-primary either removes
or can remove:

1. The cloud contributed decode, `isfinite`, transform, and re-encode over
   77k points per emission. Depth-primary deletes this entirely.
2. `ros_message_adapters.image_msg_from_frame` and `depth_msg_from_frame` each
   perform `np.ascontiguousarray(...).tobytes()`, a full pixel copy per
   publish, before rclpy copies again into the message field and once more
   into the CDR buffer. For colour the republished bytes are **identical to
   the received bytes**; the preprocessor adds a stamp and a pairing decision,
   not pixels.

Therefore this plan makes zero-copy republish a first-class deliverable (D2)
rather than an optimisation the owner may skip, and it requires the throughput
budget to be measured **before** the refactor is committed to (D1), in the
same measurement-first style that made B1 useful.

### Pre-authorised mitigation ladder

If D1 projects a ceiling below the D3 bar, the owner applies these in order
without a new consensus round, and records which step cleared it:

1. **Reference republish.** Carry the received `data` buffer on `RgbFrame` and
   `DepthFrame` and publish it by reference. No `tobytes()`, no
   `ascontiguousarray`, no numpy round-trip on the republish path. Mutation
   isolation (F3-3) still applies to the decoded arrays.
2. **Publish on demand, per product.** Serialise a product only when it has a
   subscriber. Generalise the existing depth-image special case.
3. If both are implemented and D3 is still missed, return `blocked` with the
   measured ceiling and the per-stage attribution. Two named escalations exist
   — a C++ composable preprocessor, and a contract in which the preprocessor
   publishes an authoritative accepted-stamp index instead of republishing
   pixels — and **both require a new consensus item.**

Not available to this plan: lowering D3, and changing the sim colour profile
to 640x360 @ 15 Hz. The second is a genuine parity improvement and it would
cut the payload, but the sim camera profile belongs to `Q-20260908-2`; taking
it here would rewrite that question's scope from inside another plan.

## 4. F3-1 — the canonical grid

**Canonical is colour-aligned depth.**

A valid paired output consists of exactly four products plus status:

| Product | Requirement |
|---|---|
| colour image | canonical grid |
| colour-aligned depth image | canonical grid, 16UC1 millimetres |
| colour camera info | describes the canonical grid |
| aligned-depth camera info | `K`/`P` describe the **canonical grid**, not the native depth grid |

All four MUST share identical `width`, identical `height`, identical optical
`frame_id`, and the identical primary stamp. A set that violates any of these
is not a paired output and MUST NOT be published (F3-3).

### Backend binding

Namespace, stream profile, and aligned-product selection live in launch and
config. They MUST NOT appear as constants in algorithm code.

| Backend | Colour | Canonical depth | Aligned-depth camera info |
|---|---|---|---|
| Gazebo | `/camera/color/image_raw` | `/camera/depth/image_raw` | `/camera/depth/camera_info` |
| D555 | `/camera/d555/color/image_raw` | `/camera/d555/aligned_depth_to_color/image_raw` | `/camera/d555/aligned_depth_to_color/camera_info` |

Gazebo is legal on its native depth image **only because** its colour and
depth grids are already identical and share one optical frame. That is a
property of the gz `rgbd_camera`, not a general licence; a backend whose
colour and depth grids differ MUST supply an aligned product.

**Explicitly illegal canonical inputs.** Both carry depth-native geometry:

- `/camera/d555/depth/image_rect_raw` — native depth grid, `d555_depth_optical_frame`, depth K.
- `/camera/d555/depth/color/points` — named "color", but is **depth-native**.
  HB-1 confirms `d555_depth_optical_frame` and depth K (fx 321.51 against
  colour 323.18). Deprojecting or unprojecting it with colour K reintroduces
  the 58.77 mm lever that HB-2 measured as 12-54 px of error. This topic is
  named here because its name is the trap.

## 5. F3-2 — geometry responsibilities

Every responsibility below is assigned to exactly one node.

### Preprocessor

Pairs the canonical set and republishes it. It **no longer subscribes to,
waits for, transforms, or publishes camera points.** `input.camera_points`,
the cloud ring buffer, `_decode_cloud`, `_decimate_cloud`, the cloud stage
timing, and `/luggage/preprocessed/camera/depth/points` are removed, not
disabled behind a flag.

`cloud_ok` leaves the flag set. `depth_ok` becomes the flag that gates a
paired output.

### `semantic_point_filter`

Exact-joins aligned depth, aligned-depth camera info, the semantic mask, and
the optional instance mask. Then, in this order:

1. Select the **union** of configured cargo and obstacle pixels.
2. Deproject only selected pixels with valid depth:
   `x = (u - cx) * z / fx`, `y = (v - cy) * z / fy`, `z` from the depth pixel.
3. Emit both existing output clouds, unchanged in topic, type, and frame
   semantics.

The current `_project_to_color` path — transform depth-frame points by
`depth_to_color`, project with `u = (fx*x + cx*z)/z`, `np.rint`, then look up
`mask[v,u]` — is **deleted**. Under a colour-aligned contract that projection
is the identity by construction. The extrinsic, the rounding, and the
project-outside-image loss all disappear. Point count falls from ~205k to
order 1.5k-15k.

### `luggage_detector`

Separately exact-joins the same aligned depth and aligned-depth camera info
for support geometry. It does not receive a cloud and does not read the
filter's outputs for this purpose.

**Baseline is candidate B-ii:** bounded, configurable, local full-grid
decimated deprojection, used solely for the support fit. The default stride is
set by the owner and MUST be justified by D6's retention margin, not chosen by
preference. B-i (range-dependent mask dilation) is a fallback that may be
adopted **only** if B-ii is measured to be stride-sensitive, and adopting it
requires recording that measurement.

`_raw_cloud_cb`, `_pop_raw_world_with_retry`, and the raw-cloud buffer are
removed.

### Module placement

Deprojection and decimation maths go in a new ROS-free module under
`src/luggage_perception/luggage_perception/` (suggested
`depth_deprojection.py`), unit-testable without rclpy, consistent with how
`semantic_point_filter.py` relates to `scripts/semantic_point_filter_node.py`.
They MUST NOT be added inline to the node files, which are already large.

## 6. F3-3 — fail-closed and timing

Restated in full because the change moves geometry across node boundaries.

- Any mismatch in dimension, optical frame, camera info, encoding, or exact
  stamp produces **no geometry** and increments a **named** reason counter.
- Invalid, zero, and non-finite depth pixels are discarded before
  deprojection.
- No consumer may add approximate pairing. Tolerance pairing stays in the
  preprocessor alone.
- No latest-TF fallback. No `now()` stamps. No raw-depth fallback when the
  aligned product is absent. No empty array standing in for a valid
  observation.
- Cargo, obstacle, and support geometry all inherit the paired acquisition
  stamp and a truthful optical `frame_id`.
- `false_measured_height` stays 0. Availability bought by admitting a wrong
  plane is a failure, not a trade.

## 7. F3-4 — acceptance, replacing the cloud-specific bars

Common window for D3 through D6: one probe run of **>= 120 s** with the arm at
`pickup_observe`, discarding the first 15 s as warmup. State the exact command.
All ratios are computed on that window from counter deltas, never lifetime
totals.

Mapping to the superseded criteria: D3 is old B3 unchanged; D4 replaces old B4
in full; D1, D2, D5, D6, D7 are new; old B1, B2, B5, B6 stay passed and are
not re-run except where the refactor touches them.

- **D1 — throughput budget, measured first.** Before committing to the
  refactor, record per-emission serialised bytes per product, publisher-thread
  period, and the per-stage attribution of that period. Project the achievable
  emission rate for the depth-primary product set and compare it to the D3
  bar. State which mitigation-ladder steps (section 3) are required. This is
  the deliverable that either justifies or retires the ladder; it is not
  optional and it precedes the code.
- **D2 — zero-copy republish.** Colour and canonical depth are republished
  without a per-publish pixel copy. No `tobytes()` or `ascontiguousarray` on
  the republish path. A test asserts the published `data` is the received
  buffer by identity or by a copy-count probe, and that F3-3 mutation
  isolation on the decoded arrays still holds.
- **D3 — throughput (unchanged from B3).** Emitted-observation rate
  >= **0.8 x** the colour arrival rate over the window, denominator being
  unique colour header stamps received, not callbacks; and `raw_img->pre_rgb`
  p50 <= **60 ms**.
- **D4 — depth-primary join ratios.** All four on the common window:
  - preprocessor `depth_ok` >= **0.95** of emitted observations;
  - filter exact join `joined / depth` >= **0.95**;
  - detector same-stamp support-depth hit per joined cargo observation
    >= **0.95**;
  - stale drops `(stale_depth_dropped + stale_mask_dropped) / (depth + mask)`
    < **0.05**.
- **D5 — fixtures.** Committed fixtures at **both 640x480 and 640x360**,
  covering: deprojection correctness against a known intrinsic and a known
  plane; invalid, zero, and non-finite depth; `K` mismatch; optical-frame
  mismatch; width/height mismatch; mutation isolation; exact-stamp rejection.
  Each failure fixture asserts the named reason counter, not merely absence of
  output.
- **D6 — support retention.** The B-ii sampler retains
  >= `min_support_points` (80) with a **stated margin**, reported at the
  configured default stride and at one stride above it. Report the retained
  annulus point count at `support_inner_margin` 0.03 to `support_outer_margin`
  0.18 m.
- **D7 — consumer removal is proven, not asserted.** A repository search
  showing **no production publisher or subscriber** of
  `/luggage/preprocessed/camera/depth/points` is an acceptance artefact,
  recorded as the command and its output. ROS 1 reference files
  (`task_cloud_filter_node.cpp`, reached only from `active_loading.launch`)
  are excluded from the ROS 2 chain and MUST stay unmodified. Probes and eval
  harnesses (`pf_r9_b34_probe.py`, `stage_perf_probe.py`,
  `pick_retreat_eval_driver.py`) are migrated, not deleted.
- **D8 — hardware stream and pixel-grid validation.** Section 8.

## 8. F3-5 — the hardware observable

D8 is a separate observable from D3-D7 and runs on the live cell.

Setup: D555 at 640x360 @ 15 Hz, **>= 120 s**, with point-cloud publication
*and* subscription disabled (`pointcloud.enable:=false` on the driver, and no
subscriber in the graph).

Required:

- the device **remains enumerated** for the whole run, checked at start and
  end; a DDS device drop is an outright failure, not a degraded result;
- emitted / unique-colour >= **0.8**;
- every D4 ratio holds at its D4 threshold;
- every F3-3 fail-closed reason counter is present and, where not exercised,
  zero;
- colour and aligned depth agree on width, height, optical frame, and stamp on
  every emitted pair;
- report `raw -> preprocessed` colour latency p50, p95, and max.

**The latency limit is set here, before dispatch, and may not be chosen after
the run:** p50 <= **60 ms** and p95 <= **150 ms**. The p50 is retained from
D3 rather than re-derived; at the measured 14-15 Hz colour rate a 67-71 ms
frame period makes 60 ms roughly one frame of pipeline depth. The p95 of
150 ms is about two frame periods plus margin and stays well inside
`camera_horizon_sec` 0.35. If the measurement misses either, report the value
and raise an amendment; do not rescore.

**What D8 explicitly does not test.** World-frame geometry accuracy. HB-3
measured a ~2 deg mount rotation error that is not F3's defect and is not
F3's to fix. Any hardware check of cargo height, support plane, or base-frame
position is sequenced **after HE-2**
(`docs/plans/d555_handeye_calibration.md`) and MUST NOT gate PF-R9
generation 2. Pixel-grid and stream-contract validation carry no dependency on
the mount transform and run immediately.

## 9. F3-6 — contracts and consumers

### Architecture amendment, applied before code dispatch

`docs/architecture/sensor_data_pipeline.md` is normative, so it changes first.
The exact edits, drafted here so round 2 audits the wording rather than an
intention:

1. **Sensor registry** — mark `/camera/depth/points` as a raw backend stream
   that the preprocessor does not consume, and add the D555 rows from
   section 4, including the depth-native warning on
   `/camera/d555/depth/color/points`.
2. **Published topic set** — remove
   `/luggage/preprocessed/camera/depth/points`. Add the statement that the
   published set is the canonical paired grid of section 4 and that the four
   products share width, height, optical frame, and stamp.
3. **Per-stream structures** — `RgbFrame` and `DepthFrame` gain a raw payload
   reference so D2's zero-copy republish is expressible. `CameraCloud` is
   removed from the preprocessor's internal structures.
4. **`SyncedObservation`** — drop `camera_points`; `depth_ok` becomes the
   gating flag for a paired output.
5. **Trap 1** (gz points carry the optical `frame_id` while holding
   `camera_link` data) is retained as a **historical** note explaining why the
   cloud path is gone, not as live guidance.
6. Add the F3-3 fail-closed list as a rule block.

Applied only after `consensus: reached`. Until then the amendment lives here.

### Gate 5

`docs/plans/platform_free_height_gate5_bag_contract.md`, its checker, and its
fixtures:

- remove `/luggage/preprocessed/camera/depth/points` from required online
  topics;
- require colour, aligned depth, and **both** camera infos;
- record `width`, `height`, `rate`, alignment status, and optical frame as
  manifest metadata;
- validate the **manifest-declared** optical frame instead of hard-coding
  `camera_depth_optical_frame`, so a `d555_color_optical_frame` recording is
  legal; update `BAG_MISSING_FRAME` accordingly;
- accommodate a 16:9, 15 Hz hardware recording.

### Migration inventory

Production launch and config, `luggage_detector`, `semantic_point_filter`,
probes, eval subscribers, and RViz references. D7 is the proof that this
inventory is complete.

## 10. F3-7 — lifecycle

Applied literally, per `docs/agents/README.md`.

| Task | Generation 1 state | Action |
|---|---|---|
| PF-R9 | **claimed**, `outcome: blocked` | Supersede with generation 2. Request and record a stop acknowledgement from `claude/glm-5.3` **before** the supersede transition. Generation 2 is based on the accepted generation-1 code, which is kept: B1, B2, B5, B6 passed and are not redone. |
| PF-R10 | unclaimed, `dispatch_ready: yes` | Replace with generation 2. Plan revision, upstream generation, counters, and acceptance inputs all change. **PF-R10 may not run on the current stride-2 state.** Generation 1's row is withdrawn now so it cannot be claimed against a superseded dependency. |
| PF-R7 | generation 2, unclaimed | **No change.** Owner `cursor/grok-4.6`, dependency ids, scope, and acceptance are identical; its Claim snapshots the highest passing PF-R9/PF-R10 generations. If its plan or acceptance text later changes, supersede rather than edit claimed state. |
| PF-R8 | `status: done`, passed | Untouched. |

Only the replacement rows dispatch, at the exact plan revision of this
document once round 2 records `consensus: reached`.

## 11. Subtasks

Owners are provisional until round 2 closes. `dispatch_ready: no` on both.

| ID | Owner agent/model | Depends on | Bounded scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| PF-R9 g2 | `claude/glm-5.3` | PF-R6, PF-R8 | Depth-primary contract: preprocessor cloud removal and zero-copy republish, filter mask-first deprojection, detector B-ii support sampler, new ROS-free deprojection module, Gate 5 and consumer migration | D1-D8 | Focused unit tests, D5 fixtures at both resolutions, >=120 s sim probe, >=120 s hardware D8 run |
| PF-R10 g2 | `claude/glm-5.3` | PF-R6, PF-R8, PF-R9 g2 | Integration: whole-chain gate4 re-baseline on one committed revision | C1-C3 of `pf_r8_r9_perception_acceptance.md`, unchanged | `gate4_short6` x3 + PF-G6S lifecycle |

PF-R10's C1-C3 are carried over verbatim: three consecutive passing runs on one
committed revision with `dirty=0`, `active_output_hz >= 4.0`,
`top_surface_rate >= 0.95`, FULL_3D rate >= 0.95, `false_measured_height == 0`,
`failed == 0`, plus the PF-G6S buffer, executor-lag, RSS-slope, and residual-
process rules. Round 1 required the geometry limits and FULL_3D to survive the
replacement; carrying C1-C3 unchanged is how that is enforced.

Note that generation 1's whole-chain sanity already measured
`active_output_hz` 12.30 and `top_surface_rate` 1.000 at stride 2, with
`full3d_rate` 0.935 marginally under the 0.95 bar on a 6-trial short run. C1's
three-run rule is what settles whether that is short-run variance.

## 12. Sequencing against the calibration board

The ChArUco board is a physical artefact that does not exist yet. It gates
**one** subtask.

| Work | Board needed? | State |
|---|---|---|
| HE-1 — CAD seed, printable board asset, capture and solve scripts | **No** | Dispatched, `Q-20260909-1` |
| F3 round 2 consensus and the section 9 architecture amendment | No | This document |
| PF-R9 g2 — full depth-primary refactor and D1-D7 sim acceptance | No | Blocked on consensus only |
| PF-R10 g2 — Gate 4 integration | No | Blocked on PF-R9 g2 |
| Gate 5 contract, checker, and fixture migration | No | Inside PF-R9 g2 |
| D8 — hardware stream and pixel-grid validation | **No** | Camera is present; needs no world-frame accuracy |
| Board print, flat mount, calliper pitch measurement | Produces it | **Critical path, start on HE-1 delivery** |
| HE-2 — capture, solve, validate | **Yes** | `Q-20260909-2`, depends on HE-1 |
| Hardware world-frame geometry accuracy re-check | Yes, via HE-2 | Sequenced after HE-2, does not gate PF-R9 g2 |

Two things follow.

First, **the board blocks only HE-2**, and by round-1 item 5 it does not block
F3 at all — including F3's hardware run, because D8 measures stream contract
and pixel-grid agreement, neither of which passes through the mount transform.
The entire F3 chain is runnable on the 5090 sim host plus the existing camera.

Second, **HE-1 is the critical path to the board**, not a parallel task. HE-1
deliverable 4 is the true-scale printable asset; the board cannot be printed
until it exists. The physical steps after that — print matte, mount on 3 mm
aluminium composite or 5 mm acrylic to 0.5 mm flatness, measure the printed
pitch across 5 squares with callipers — have real lead time and are on the
user, not on an agent.

## 13. Risks

- **D3 may still be unreachable.** Section 3 is an argument that the
  generation-1 ceiling was Python copy cost rather than DDS wire cost, but it
  is an argument, not a measurement. D1 exists to settle it before the
  refactor is sunk. If D1 projects failure with the full ladder applied, the
  honest outcome is to escalate composition or the accepted-stamp-index
  contract as a new consensus item, and F3 stalls at that point.
- **D2 is where a mutation-isolation defect would be introduced.**
  Republishing a received buffer by reference is exactly the change that makes
  two nodes share memory by accident. The D5 mutation fixture is not optional.
- **B-ii's stride is a geometry risk, not a throughput risk.** Generation 1
  showed stride 3 broke the downstream join; here the stride is local to the
  detector and cannot affect transport. The failure mode is a thin support
  annulus at long range, which D6 measures directly.
- **Gate 5's optical-frame check is a silent-breakage risk.** Hard-coding
  `camera_depth_optical_frame` today means a D555 recording in
  `d555_color_optical_frame` would fail validation for the wrong reason. The
  manifest-declared check must land with the topic change, not after it.
- **`/camera/d555/depth/color/points` will be reintroduced by someone reading
  its name.** It is named in section 4, in the architecture amendment, and in
  the RViz migration for that reason.

## 14. Pointers

- `docs/agents/discuss/2026-09-08_1651_f3-depth-primary-contract-consensus.md`
- `docs/agents/reviews/2026-09-09_1033_f3-depth-primary-consensus-round1.md`
- `docs/plans/pf_r8_r9_perception_acceptance.md`
- `docs/status/evidence/platform_free_height/2026-09-08_pfr9_throughput/RESULT.md`
- `docs/status/evidence/d555_bringup/README.md`
- `docs/architecture/sensor_data_pipeline.md`
- `docs/architecture/perception_architecture.md`
- `docs/plans/platform_free_height_gate5_bag_contract.md`
- `docs/plans/d555_handeye_calibration.md`
- `src/luggage_perception/luggage_perception/ros_message_adapters.py`
- `src/luggage_perception/luggage_perception/semantic_point_filter.py`
- `src/luggage_perception/scripts/luggage_detector_node.py`
