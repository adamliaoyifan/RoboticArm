# S5 typed boundary checkpoint (formal campaign pending)

S4 product bb4cb175, adopted baseline b33dda91, plan b9f4b2dc. No overall G2 PASS.

Four typed read-only services and nine typed DTO messages are registered. They wrap immutable query handles in the mapper's existing reentrant callback group; primary Store writer, heartbeat parameters, lease, executor and callback-group topology remain unchanged. Python schema-1 meanings are retained. The registry supports strict same-token/full-identity resume without renewing expiry; invalid requests preserve an existing token unless explicit release was requested. Protocol details are in src/luggage_perception/QUERY_API.md.

## Preflight verification

- Independent /tmp/occ-query-g2-build and install initially built luggage_msgs/description/packing/perception/planning, including generated type-support libraries, in 80s. No shared overlay rebuilt. The missing YOLO checkpoint warning is unrelated to these read-only services; no semantic detector launched.
- Six real generated-message serialization tests pass, including summary, frozen old-generation paging, explicit release/expiry/invalid-cursor retry, enums/errors, nullable full surface and patches, and distance policies. No new contract test skipped.
- Initial typed summary test exposed adapter KeyError(placed_count). Fixed mapping to existing committed_box_count; old summary DTO unchanged. Raw response/logger/summary and replay pointer retained in preflight/failures/typed_summary/.
- Additional existing mapper/grid/confidence/occ-input/view/snapshot and adapter regression: 114 passed, four existing skips. This includes existing no-motion live mapper tests. Their environment used the default ROS domain (unset); this is rehearsal, not isolated formal timing evidence. Subsequent standalone runtime campaigns use dedicated domains.
- Existing skips, all the parked prior-node semantics in test_cargo_map_integrate_live.py: test_full_depth_endpoint_isolated_from_empty_cargo_at_same_stamp; test_depth_observer_real_ros_client_reaches_owner; test_volume_service_exact_revision_read_only_and_stale_refusal; test_volume_read_waits_for_inflight_map_mutation. They do not cover the replaced prior depth observer/volume-wait behavior; not silently counted as covered.
- A source-side read-only ROS wiring fixture is added for installed-module provenance, unique domain, per-request serialized CDR artifacts and bounded shutdown. Its actual installed rehearsal is still pending at this checkpoint. The pure frozen st09_contracts package must also be independently built before that rehearsal.

## Outstanding mandatory work

Full CLI oracle/benchmark/concurrency/planning/replay implementation, full 20x500 shape/resolution/actual-box matrix, clean-artifact paired 3+30 groups, write P95 regression, 120sx3 heartbeat and real placement hit/fallback service checks are not_evaluated. Existing offline unit/concurrency tests and a fast development number do not pass these gates. No simulation/motion or production integration performed.

## Installed rehearsal and oracle runner progress

- Clean e11e2883 independent installed rehearsal completed at deployment 1.49x1.97x1.48m/.01m in ROS domain 183: five read services x3 =15 successful calls. All product modules came from /tmp/occ-query-g2-install, including independently built st09_contracts. CDR requests/responses, loaded paths and timing/RSS are preserved in preflight/installed_smoke_01/. These three observations per endpoint are NOT formal P95/performance/heartbeat gates.
- Expanded unscored oracle fixture preflight: 504 rows and then 576 rows, each exactly equal to b33 across .01/.02/.05 x cuboid/chamfer x0/5/20 initialized actual geometry records. Initial 6-cell fixture could not physically distinguish five tiny neighbors under unchanged .05m commit dedupe tolerance. Failure source/manifest retained; fixture expanded to .30m with >=.10m center separation, asserting actual box count rather than truncating workload. This is a test-data repair, no product tolerance change.
- New process-isolated oracle/replay runner and fixture capture frozen source archives, input journals, primary fact/legacy export hashes, full index checks, per-map independent distance queries and failed-case before/after arrays. Ten argument-boundary/help tests pass. The staged CLI currently implements oracle/replay only; benchmark/concurrency/planning remain to implement. Frozen-run/replay behavior must be exercised next; do not count the preflight as the formal20x500 campaign.

## Frozen runner checks and bounded-distance finding

Frozen a646cd71 rehearsal_01: baseline/candidate source hashes unchanged; 576 rows and normalized input journals identical. This exercised actual archives/worker subprocesses rather than parser text. Synthetic replay fixture reconstructs matching before/after arrays from both sources and demonstrates five fact fields changed across its selected transaction; it is a capture pipeline test, not a historical product failure.

Frozen rehearsal_02 uses 1,000 distance points per initial/final map. On the first .01m/cuboid/zero-initial-box final map, a 250-point batch exhausted the unchanged two-million-cell cap after 1,999,200 examined cells, 544 blocks, 25.855595ms. Named incomplete result was correctly returned; this is a new query workload failure, not an OCC fact difference. Full map/points/response, journal and immutable source archives are retained under s5/frozen_rehearsal_02/. Source hashes stayed unchanged. It is not charged as a formal 3+30 product campaign.

A focused query-only fix proves zero distance by probing the closed volumes containing the point, checking exact grid-plane neighbors in smallest-flat order. This avoids full-block scans for points already inside the target state. +/-1 index brackets handle division rounding, final predicates use identical AABB arithmetic. All budgets and old primary facts remain unchanged. Distance suite:12 passed, including plane +/-ULP ties and <=8 cell probes per unknown point. New frozen workload rerun pending; no gate success-rate or overall PASS claimed.

Frozen 78a38934 rehearsal_03 completed: 576 rows and normalized input journals match b33; all initial/final maps passed 1,000-point independent distance checks across the 18 workload groups. The prior work-cap failure remains retained. This is still a one-seed/32-transaction rehearsal, not the formal matrix.

Typed distance request now explicitly defaults radius to .30m in its IDL, matching the existing Python/planned default. Previously the generated request silently defaulted to zero. Interface/perception independently rebuilt; six typed serialization tests pass, including an assertion on the generated default. No old wire field/layout, primary map behavior or budget limit changed.

Formal oracle_matrix_01 at clean3f605 failed after baseline180000 rows / candidate1500 rows, seed2/.01/cuboid/initial0 final distance batch.16.276ms,1999707 examined cells,981 block visits, first incomplete point217/250. Complete input/map/response/source capture retained; see FAILURE_DIAGNOSIS.json. Repeated immutable membership decoding was charged for each point. Request-local AABB/membership reuse now charges one membership scan plus every actual target-volume test, preserving fixed caps and writer.13 distance tests pass, including sparse250-point oracle and reduced-cap refusal. Full acceptance remains not_evaluated until replacement frozen campaign.
