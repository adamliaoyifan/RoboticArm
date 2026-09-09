# PF-R9 generation 2 -- payload-backed depth-primary execution

Date: 2026-09-09
Parent: `PFH-REMEDIATION-20260904`
Subtask: `PF-R9`
Owner: `eng/claude/glm-5.3`
Implementation base: `bebaa7c9c9a8f75be4f0779a2dcc271bca408258`

Status: **approved by direct user decision**. This plan does not require a
further consensus round. Dispatch is gated only by committing this plan and
completing the stop acknowledgement for claimed PF-R9 generation 1.

This is the executable PF-R9 generation-2 plan. It incorporates the accepted
depth-primary scope from `pf_f3_depth_primary_contract.md` and replaces that
document's D1/D2 transport wording, camera-buffer sizing, mitigation ladder,
and consensus dispatch gate where they differ.

## 1. Objective

Replace full camera-cloud transport with a colour-aligned RGBD contract while
meeting the original PF-R9 throughput and latency bars. Preserve the original
ROS message payload through preprocessor receive, pairing, buffering,
copy-out, queueing, and republish without an avoidable Python-owned full-frame
copy.

The implementation must also migrate the semantic filter and detector from
transported camera clouds to local depth deprojection, update Gate 5, and
prove the complete depth-primary graph in simulation and on the current D555.

Changing the simulation camera profile is not in scope. The implementation
must pass with the existing 640x480 simulation profile. The D555 hardware
profile may be configured as colour plus colour-aligned depth at 640x360,
15 Hz.

## 2. Fixed product contract

The canonical acquisition contains:

1. colour image;
2. colour-aligned depth image in `16UC1` millimetres;
3. colour camera info;
4. aligned-depth camera info;
5. preprocessor status.

The four image/calibration products must have the same primary stamp, width,
height, and truthful colour optical frame. Aligned-depth `K` and `P` describe
the colour pixel grid. A mismatch is rejected as one acquisition and produces
no geometry.

Backend bindings remain configurable:

| Backend | Colour | Canonical depth | Optical frame |
|---|---|---|---|
| Gazebo | `/camera/color/image_raw` | `/camera/depth/image_raw` | configured common optical frame |
| D555 | `/camera/d555/color/image_raw` | `/camera/d555/aligned_depth_to_color/image_raw` | `d555_color_optical_frame` |

Native D555 depth and `/camera/d555/depth/color/points` are not legal
canonical inputs. The preprocessor no longer subscribes to, waits for,
transforms, or publishes a camera point cloud.

## 3. Payload ownership contract

### 3.1 Ownership

The ROS callback receives the source `sensor_msgs/Image` and hands an opaque
payload reference to the ROS-free frame structure. The payload reference owns
a strong reference to the source `array.array('B')` or source message for as
long as any buffered frame, observation, or in-flight publish refers to it.

The following rules are mandatory:

- The preprocessor treats the source payload as immutable after receipt.
- `RgbFrame.copy()`, `DepthFrame.copy()`, and `SyncedObservation.copy()` share
  the opaque payload reference; they do not copy pixel bytes.
- Algorithm code receives a read-only NumPy view produced from a read-only
  `memoryview` with `np.frombuffer`. It cannot unwrap or replace the mutable
  ROS data array, and `view.setflags(write=True)` must fail.
- Only the ROS adapter/node layer may unwrap the payload for republish.
- A newly built output `Image` may rewrite the paired header and metadata but
  assigns the original `array.array('B')` to `out.data`.
- D2 means no avoidable Python payload copy. DDS/CDR serialization may still
  copy and must not be described as loaned-message or end-to-end zero-copy.
- Payload lifetime extends through `Publisher.publish()` return. No source
  payload may be mutated or recycled while referenced.

The preferred representation is an opaque immutable wrapper in the ROS
adapter boundary, referenced by `RgbFrame` and `DepthFrame`. Do not put a raw
public `array.array`, writable NumPy array, or ROS message into the algorithm
API.

### 3.2 Read-only decoded views

RGB and depth view construction must respect `height`, `width`, `step`,
encoding, channel count, and endianness without an eager `.copy()` or
`.astype()`.

- Contiguous and row-padded images are both valid when their declared buffer
  length is sufficient.
- Truncated buffers fail closed with a named counter.
- Big-endian `16UC1` may remain a big-endian read-only view. It must not be
  silently converted by copying merely to obtain native byte order.
- Any downstream computation that truly needs a writable array may make a
  local, measured working copy after exact join. That copy is not allowed on
  the receive-pair-republish path.

### 3.3 Copy boundary

The no-copy path covers all of:

```text
ROS receive
  -> view construction
  -> ring-buffer insertion
  -> RGB/depth pairing
  -> SyncedObservation construction and copy-out
  -> emit queue
  -> output Image field assignment
  -> Publisher.publish call
```

There must be no full-frame `copy`, `astype`, `ascontiguousarray`, `tobytes`,
`bytes`, or equivalent materialisation on that path for a valid canonical
input. This requirement applies to both colour and aligned depth.

Generated masks, instance maps, sparse point results, and algorithm working
sets are outside this identity requirement because they are new products.

## 4. Buffer and lookup contract

### 4.1 Capacity

All camera acquisition and downstream exact-join buffers introduced or
touched by this change use:

```yaml
camera_maxlen: 15
camera_horizon_sec: 1.0
```

This means each raw camera stream stores at most 15 payloads and no payload
older than one second of the canonical camera stream. The accepted-pair or
exact-join index also stores at most 15 acquisitions. Index entries and copied
frame wrappers share the original payload; they do not duplicate its bytes.

At the two accepted profiles, 15 complete RGBD acquisitions account for:

| Profile | Bytes per RGBD acquisition | 15-acquisition payload ceiling |
|---|---:|---:|
| 640x480 RGB8 + 16UC1 | 1,536,000 | 23,040,000 B |
| 640x360 RGB8 + 16UC1 | 1,152,000 | 17,280,000 B |

The existing emit queue remains bounded at four observations. Those entries
share payloads. Diagnostics must report cache occupancy, payload bytes, emit
queue depth, and eviction counters separately. Tests must not claim the
entire process has only 15 live acquisitions: an evicted payload may remain
alive briefly while an in-flight callback or one of the four queue entries
holds a reference. This tail is bounded by the callback and queue limits and
must not grow over time.

### 4.2 Update and eviction

- Insert by exact integer `(sec, nanosec)` stamp; do not use float equality as
  the payload identity.
- An identical stream/stamp replaces its index entry without increasing
  occupancy.
- Slight out-of-order input within `rollback_sec` is inserted in stamp order.
- A backward jump greater than `rollback_sec` clears all camera, accepted,
  downstream exact-join, and pending-output indexes, increments the local
  `camera_epoch`, and increments a rollback counter.
- Capacity eviction removes the oldest stamp first and increments
  `evicted_capacity`.
- Horizon eviction removes entries older than `newest_camera_stamp - 1.0`
  and increments `evicted_horizon`.
- Camera eviction uses the canonical camera stream clock. Joint, IMU, lidar,
  wall-clock time, and a paused bag must not age camera payloads.

### 4.3 Mask/depth association

Tolerance pairing occurs only in the preprocessor. Once it accepts an RGBD
pair, downstream mask and geometry association is exact.

For this generation, the exact integer primary stamp is the wire-visible
accepted acquisition key. `camera_epoch` is a local reset/diagnostic domain;
it is not claimed to cross DDS because the existing image messages do not
carry it. On rollback every participating node must clear all join buffers and
pending output before accepting the new timeline. A future custom token
message is not required by this generation. A consumer receiving a mask must
retrieve the depth selected for that accepted acquisition; it must not perform
a second nearest-neighbour search.

The current ROS nodes remain separate processes. Payload references therefore
remove preprocessor-side Python copies but are not pointers that can cross
DDS. Downstream nodes retain their received aligned-depth messages in their
own bounded 15-entry/1-second exact-join buffers and decode a read-only view
only after a matching mask or cargo observation arrives.

## 5. Implementation scope

### 5.1 Architecture first

Update `docs/architecture/sensor_data_pipeline.md` before product code:

- define the canonical colour-aligned RGBD set;
- remove the preprocessed camera-cloud product and `camera_points` ownership;
- define opaque immutable payload-reference ownership and application-layer
  copy semantics;
- define 15-entry/1-second, primary-camera-clock buffer behavior;
- define exact accepted-acquisition association and rollback epoch invalidation;
- retain fail-closed, truthful frame, acquisition-stamp, and algorithm-core
  isolation rules.

Update other architecture documents only where they directly contradict this
contract. Do not perform unrelated documentation cleanup.

### 5.2 Baseline measurement before functional refactor

Instrument and record the complete current path, not only publisher-thread
period:

- receive-to-view construction;
- frame validation and ring-buffer insertion;
- observation build and each copy-out boundary;
- emit-queue wait;
- output message construction and `data` assignment;
- `Publisher.publish()` duration;
- bytes per product and full accepted subscriber graph rate.

Record p50, p95, max, call count, and payload bytes for each stage. Store raw
results and a short interpretation under
`docs/status/evidence/platform_free_height/<run>/`. This measurement is D1
and precedes removal of the measured copy sites.

### 5.3 Preprocessor and adapters

- Add the opaque payload reference and read-only stride-aware view helpers.
- Remove eager RGB/depth copies throughout receive, buffer, observation, and
  emit paths.
- Republish source application payloads by identity.
- Change defaults and YAML to `camera_maxlen=15`,
  `camera_horizon_sec=1.0`.
- Separate camera-clock pruning from unrelated sensor clocks.
- Make depth the mandatory pair gate and remove camera-cloud input, decode,
  transform, wait, status, and output paths.
- Keep per-product publish-on-demand only as idle/debug resource hygiene. It
  receives no D3 performance credit because colour and depth both have
  subscribers in the accepted graph.
- Add diagnostics for payload copies, payload bytes, occupancy, both eviction
  causes, exact lookup misses, epoch, rollback resets, and emit queue depth.

### 5.4 Semantic filter

- Exact-join aligned depth, aligned-depth camera info, semantic mask, and
  optional instance mask by accepted acquisition.
- Buffer each side at 15 entries/1 second using primary camera stamps.
- Select the configured cargo/obstacle pixel union first, then deproject only
  valid selected pixels.
- Preserve both existing output clouds, stamps, frame semantics, and named
  failure diagnostics.
- Delete the depth-point-to-colour reprojection path; aligned pixels already
  share the colour grid.

### 5.5 Detector support geometry

- Replace the transported raw-cloud subscription and buffer with exact-joined
  aligned depth and camera info, each bounded at 15 entries/1 second.
- Locally deproject a deterministic stride-decimated full depth grid for the
  support annulus. Default stride is chosen from D6 measurement.
- Preserve the current support margins, minimum support count, fail-closed
  gates, acquisition stamp, and `false_measured_height == 0` contract.

Place shared deprojection and sampling maths in a ROS-free module under
`src/luggage_perception/luggage_perception/`, with ROS-node code limited to
message handling, lookup, diagnostics, and publication.

### 5.6 Launch, probes, and Gate 5

- Remove every production publisher/subscriber and RViz dependency on
  `/luggage/preprocessed/camera/depth/points`.
- Migrate PF-R9 probes, stage probes, and Gate 4 drivers to aligned depth.
- Preserve ROS 1 reference files unchanged.
- Gate 5 requires colour, aligned depth, and both camera infos; removes the
  preprocessed cloud; records width, height, rate, alignment, and optical
  frame in the manifest; and validates the manifest-declared frame instead of
  hard-coding `camera_depth_optical_frame`.
- Backend topic names, aligned-product selection, and profile stay in launch
  or config, never algorithm constants.

## 6. Acceptance criteria

### D1 -- complete-path baseline

Before the functional refactor, evidence contains the stage attribution from
section 5.2 under the full accepted subscriber graph. It identifies every
avoidable full-frame materialisation and projects the post-change rate for the
1.536 MB simulation pair and 1.152 MB hardware pair.

Pass requires complete measurements; a guessed DDS throughput or
publisher-only period is insufficient.

### D2 -- payload identity and isolation

All conditions must pass:

- Valid RGB and aligned depth traverse the complete section 3.3 path with
  zero preprocessor-owned full-frame materialisations.
- Before `Publisher.publish`, output `Image.data` is the same
  `array.array('B')` object as the received source payload, or an equivalent
  copy-count probe proves zero materialisations if ROS generated bindings make
  identity observation unavailable.
- Frame/observation copy operations preserve payload-reference identity.
- Decoded views are non-writeable; attempted normal NumPy mutation raises.
- Mutating a copied frame's metadata or replacing a local view cannot corrupt
  buffered data or another observation.
- Evicting an index does not invalidate an in-flight queued reference.
- Tests state explicitly that CDR still serializes the message.

### D3 -- throughput and latency

Run at least 120 seconds after 15 seconds warmup in simulation with the full
accepted subscriber graph. Compute counter deltas over the scoring window.

- emitted observations / unique colour header stamps >= **0.80**;
- raw colour receipt to preprocessed colour p50 <= **60 ms**;
- report p95, max, input rate, output rate, emitted payload bytes, queue drops,
  and payload copy count;
- payload copy count on the canonical republish path equals zero.

The bars may not be lowered. Publish-on-demand is not counted as mitigation in
this graph.

### D4 -- depth-primary joins

Over the same window:

- preprocessor valid paired depth / emitted observation >= **0.95**;
- semantic-filter exact joins / received accepted depth >= **0.95**;
- detector support-depth hits / joined cargo observations >= **0.95**;
- `(stale_depth_dropped + stale_mask_dropped) / (depth + mask) < 0.05`;
- all lookup misses and evictions are reported by named counters.

### D5 -- unit and mutation fixtures

Committed fixtures cover both 640x480 and 640x360:

- RGB8 and 16UC1 zero-copy view/republish identity;
- padded `step`, truncated buffer, and big-endian depth;
- known-intrinsic and known-plane deprojection;
- invalid, zero, and non-finite depth;
- K, frame, dimension, encoding, and exact-stamp mismatch;
- same-stamp replacement without occupancy growth;
- 16th-frame capacity eviction with newest 15 retained;
- one-second horizon eviction;
- out-of-order insertion within rollback tolerance;
- rollback flush, local epoch increment, pending-output clear, and absence of
  any pre-rollback entry from subsequent lookup;
- camera cache unaffected by newer joint/IMU/lidar timestamps;
- decoded-view, frame-copy, observation-copy, queue, and eviction mutation
  isolation.

Every failure fixture asserts its named reason counter, not only missing
output.

### D6 -- support retention

At the configured detector stride and one stride above it, report valid
support-annulus point count for margins 0.03 to 0.18 m. The configured stride
must retain at least `min_support_points=80` with a stated numerical margin on
both accepted resolutions. The subsequent PF-R10 integration remains
responsible for three consecutive Gate 4 runs, but PF-R9 fixtures must not
regress the established geometry limits: top-Z p95/max <=15/25 mm, support-Z
p95/max <=15/25 mm, measured-height p95/max <=25/40 mm, XY-centre p95 <=30 mm,
width/depth p95 <=50 mm per axis, and `false_measured_height == 0`.

### D7 -- migration and boundedness

- Repository search finds no ROS 2 production publisher/subscriber of
  `/luggage/preprocessed/camera/depth/points`.
- All affected preprocessor, adapter, semantic-filter, detector, exact-join,
  Gate 5 checker, and fixture suites pass.
- Adapted B2/B6 regressions cover late valid depth, out-of-tolerance depth,
  deadline clock isolation, missing-depth fail-closed behavior, time rollback,
  and payload/view mutation isolation.
- A sustained test shows camera buffers <=15, downstream join buffers <=15,
  emit queue <=4, bounded RSS, and no monotonic live-payload growth.
- `colcon build --packages-select luggage_perception` passes.

### D8 -- D555 stream validation

Run D555 colour plus aligned depth at 640x360, 15 Hz for at least 120 seconds,
with point-cloud publication and subscription disabled.

- device remains enumerated at start, throughout sampling, and at end;
- a DDS device drop is an immediate failure;
- emitted / unique colour >= **0.80**;
- raw-to-preprocessed colour p50 <= **60 ms**, p95 <= **150 ms**; report max;
- D4 ratios pass;
- every emitted colour/depth pair agrees on exact stamp, 640x360 dimensions,
  `d555_color_optical_frame`, and colour-grid intrinsics;
- camera buffers remain <=15 and cover no more than one second;
- payload-copy count remains zero on the preprocessor canonical path;
- point-cloud publishers/subscribers remain absent.

D8 does not test world-frame geometry. The known mount error remains HE-2
scope and does not block PF-R9 generation 2.

## 7. Required test commands and evidence

The owner determines exact focused pytest paths after inspecting affected
tests, but completion evidence must include the literal commands for:

1. all preprocessor and ROS adapter tests;
2. semantic filter, detector join, and deprojection tests;
3. Gate 5 checker and manifest fixtures;
4. full `luggage_perception` pytest suite;
5. `colcon build --packages-select luggage_perception`;
6. D1 baseline probe;
7. D3/D4 >=120-second simulation probe;
8. D8 >=120-second D555 probe;
9. `rg` production cloud-consumer audit;
10. `scripts/stop_sim.sh` followed by a residual-process check.

Raw JSON/JSONL/log output belongs under `docs/status/evidence/`, not under
`docs/agents/`. Every evidence summary records the exact tested commit and
dirty-file count. Simulation must be torn down on success, failure, timeout,
or interruption.

## 8. Execution order and blocked outcome

1. Claim PF-R9 generation 2 against this exact plan revision.
2. Record D1 before functional refactoring.
3. Amend architecture.
4. Implement payload ownership, buffer changes, depth-primary preprocessor,
   consumers, Gate 5, and diagnostics.
5. Run and repair focused and full tests plus build.
6. Commit an implementation candidate.
7. Run D3-D7 simulation acceptance on the exact candidate.
8. Run D8 hardware acceptance on that candidate.
9. Repair failures and repeat affected acceptance until pass.
10. Commit evidence/notes, close the task with a passing exact revision, and
    leave no simulation processes.

Ordinary test or performance failure is not a blocked outcome; the owner
continues fixing and retesting. After camera-cloud removal and all avoidable
preprocessor Python payload copies are eliminated, if D3 or D8 still cannot
meet its fixed bar, close as `blocked` with the measured per-stage ceiling and
bring these alternatives directly to the user:

- C++ composable preprocessor with an intra-process/shared-memory data plane;
- accepted-acquisition index with colocated consumers instead of pixel
  republish.

Lowering the bars, changing the simulation profile, reintroducing the camera
cloud, or claiming publish-on-demand credit is not authorized.

## 9. Pointers

- `docs/plans/pf_f3_depth_primary_contract.md`
- `docs/plans/pf_r8_r9_perception_acceptance.md`
- `docs/architecture/sensor_data_pipeline.md`
- `docs/architecture/perception_architecture.md`
- `docs/plans/platform_free_height_gate5_bag_contract.md`
- `docs/status/evidence/platform_free_height/2026-09-08_pfr9_throughput/`
- `docs/status/evidence/d555_bringup/`
- `src/luggage_perception/luggage_perception/sensor_types.py`
- `src/luggage_perception/luggage_perception/stamp_ring_buffer.py`
- `src/luggage_perception/luggage_perception/ros_message_adapters.py`
- `src/luggage_perception/luggage_perception/sensor_preprocessor.py`
- `src/luggage_perception/scripts/sensor_preprocessor_node.py`
- `src/luggage_perception/scripts/semantic_point_filter_node.py`
- `src/luggage_perception/scripts/luggage_detector_node.py`
