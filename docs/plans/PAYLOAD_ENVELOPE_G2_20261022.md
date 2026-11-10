# Payload envelope G2: Z5mm and diagnostic placement gate bypass

Supersedes G1 (1ce56843); user explicitly requested Z position half-range5mm,
place without legality check, then closed-loop run and effectiveness validation.
Base e7743bf0 in /tmp/elfin-master-pack-baseline-g3; codex/gpt-6 owns execution.

## Implementation and acceptance

- Set tool-axis translation bounds [0.100,0.100,0.005]m in editable YAML;
  keep yaw+/-5deg, per-face size margins50mm, nominal measured transform.
- Add explicit diagnostic_skip_slot_envelope_check (defaultfalse) to selector,
  factory and route producer, expose via launch. For this diagnostic run settrue.
  Record the switch in manifest and per-candidate rows. This bypasses the new
  closed-hull slot-envelope filter, not full-arm/trajectory collision checks,
  link2 permissions, suction ownership, settle or verification.
- Behaviour tests: default rejects floor-penetrating envelope before planning;
  explicit bypass reaches edge planning and records diagnostic mode; collision
  in trajectory still rejects; unchanged bound/identity tests pass.
- Offline exact original case: all40 vertices/five physical cuboids covered,
  all15 known collision nodes rejected. Report new original-goal floor deficit;
  no statistical probability or new-route success inferred from old replay.
- Freeze code/config revision, build planning+gazebo+contracts and verify installed
  inputs. Run scripts/g3/n3_functional_run.sh with PHASE2_SKIP_SLOT_ENVELOPE_CHECK=1,
  PHASE2_RUN_OUT under docs/status/evidence/occ_owner_b1/payload_envelope_g2_20261022/.
  Domain7/headless/seed0/inch20/gain0.8/180s search+client; startup is non-scored.
  One non-scored n=3 functional attempt, stop on first failure. No changes to
  certified artifact during a run. Startup failures may repeat within20min
  after complete owned teardown. Preserve input/output before another case run.
- Report actual verified/committed/map-confirmed boxes, first failure stage,
  executor health, collision contacts and physical reference replay where motion
  exists. Functional pass requires3/3, MAX_BOXES, exit0. AND3 is a functional
  closure gate, not a per-unit probability estimate. A refusal/failure is notpass.

## Evidence and lifecycle

Use existing runner capture: manifest/config/hash and stage logs (T0/T1),
selection request/response, failure joint/vacuum ring and5sim-sec retention,
plugin/follow trace, bounded contacts8MiB on failed handoff (T2). Preserve scene,
URDF and selected trajectory for offline replay, references separate from online
inputs. Max1GiB/run,3GiBtotal; report truncation/missing capture explicitly and
repair capture before repeating that failure. Copy durable outputs to primary.
Single sim only; scripts/stop_sim.sh owned-PGID teardown on every exit. No
hardware motion. New scope permits trying the loop before new-route success.
