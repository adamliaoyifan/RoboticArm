# G2 Z5mm placement diagnostic — 2026-10-22

Requested changes implemented; closed-loop attempt executed and reassessed.
Functional closed-loop PASS was NOT achieved: one attempted box, zero packed,
no selected/certified carry route. No new-route inter-knot success is claimed.

## Configuration and code

Plan3235d8e1; implementation01900d92; failure-cause capture repair752e1835;
offline staging attributionf23fab94. Live03 ran frozen752e1835.
Tool XYZ position half-ranges [100,100,5]mm; per-face size50mm and yaw+/-5deg
unchanged. Nominal measured attachment retained. Editable configuration:
src/luggage_planning/config/payload_envelope.yaml.

Explicit diagnostic_skip_slot_envelope_check=true bypasses the new slot hull
prefilter. It is wired through launch/node/factory, recorded per candidate and
in manifest, and defaultsfalse outside this diagnostic. Full-arm trajectory
collision checks, link2 prohibition and post-place verification remain. The
run did NOT bypass those checks or certify the unchanged floor-overlap goal.
Runner freezes a config copy and its SHA256; installed files match source.

## Offline and behavioural verification

102 focused tests passed, including default slot refusal, explicit bypass
reaching planning and trajectory collision refusal remaining active. A further
11 driver tests passed for preserving exact pre-pick ContractError instead of
rewriting all causes to COMPUTE_TIMEOUT. Three packages built successfully.

Original captured case, same337 knots: G2 envelope0.849686410 x0.684422518
x0.327540865m covers all40 vertices of5 convex physical collision cuboids.
Minimum slack52.472mm. Envelope rejects153/337 knots, including all15/15 known
physical-collision knots. These are offline model checks, not motion trials.
Original saved DESCEND endpoint envelope remains5.013mm below floor; the
placement prefilter is explicitly bypassed for this experiment.

## Live attempts

| Run | Revision | Observation | Product result |
|---|---|---|---|
| live_01 |01900d92| Robot-description parameter response timed out; controller service unavailable; gain readiness never reached |0 attempted, startup failure; owned teardown complete|
| live_02 |01900d92| Plugin gain0.8 and new config/switch confirmed; pre-pick placement returned COMPUTE_TIMEOUT; driver overwrote underlying ContractError |0 attempted, root cause unknown; capture defect repaired before retry|
| live_03 |752e1835| Gain0.8; single /clock publisher; pick/vacuum attached; slot bypass confirmed in response rows |1 attempted,0 packed, PLANNING_BUDGET_EXHAUSTED|

Live03:180s configured budget; total select177582ms, prep44ms,
search177538ms, certify0ms, revalidate0ms. Tried332/791 slots;459 untried.
331 rows EDGE_BUILD_FAILED with detail staging_endpoint/no_valid_staging;
1 NO_FEASIBLE_ROUTE. Zero carry segments submitted. Suite wall206.7s.
Driver executor returned normally with failed=false before nodes_destroyed.
Shutdown-only node tracebacks are not classified as in-run executor failures.
All owned Gazebo/bridge processes stopped after each run.

## Collision attribution after run

Replay uses captured live03 expanded URDF and full-precision endpoint joints,
not rounded console joints. Native FCL0.7 tests payload against arm link1–4
collision meshes; links5/6 remain existing touch exemptions. No arm-world or
whole-configuration-space completeness is asserted by this offline probe.

Across21 distinct saved states (start +20 staging endpoints):

- Point-estimate payload intersects arm at16/20 staging states: link1 in11,
  link2 in16 (sets overlap).
- G2 envelope intersects arm at20/20 staging states: link1 in20, link2 in20.
- Start has no payload collision against the four tested arm links.
- The rejected stage is before placement/goal certification. Merely bypassing
  the slot prefilter does not repair the current staging candidate family.

This demonstrates rejection by the larger model and loss of all saved staging
candidates, not collision-free executed placement, a success-rate improvement,
or proof that no route exists outside this candidate set. To seek closure while
retaining current bounds, expand/rework staging search; changing XY/size bounds
instead requires an explicit new bound choice and coverage revalidation.

## Reproduction and artifacts

Source ROS/install; PYTHONPATH for offline probes only includes repo root,
src/luggage_planning and src/luggage_description. Live runtime uses install only.

```
PHASE2_SKIP_SLOT_ENVELOPE_CHECK=1 PHASE2_RUN_OUT=<fresh absolute output> bash scripts/g3/n3_functional_run.sh
python3 scripts/g3/replay_envelope_anchor_collisions.py docs/status/evidence/occ_owner_b1/payload_envelope_g2_20261022/live_03
```

Full selection request (including slots/surface), responseCDR/JSON, URDF,
scene/dumps, lifecycle, joint/plugin/follow streams and configs are durable in
both worktree and primary evidence directory. Large binary streams stay local;
manifest inventory/hashes identify them. Current trial bytes~140MB, within1GiB.
Do not blindly repeat this product failure without a new measured hypothesis.
