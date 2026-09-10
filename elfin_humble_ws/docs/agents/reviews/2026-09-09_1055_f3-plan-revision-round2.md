# 2026-09-09 -- F3 plan revision for consensus round 2

- role: reviews
- agent: cursor
- model: opus5
- cli: cursor
- status: done

## Summary

Bound all seven consensus round-1 constraints into a new plan document,
`docs/plans/pf_f3_depth_primary_contract.md` at plan revision `312b6fc`, and
requested the PF-R9 generation 1 stop acknowledgement that round-1 item 7
requires before the supersede transition.

Round-1 items map one-to-one onto plan sections: canonical grid 4, geometry
responsibilities 5, fail-closed 6, replacement acceptance 7, hardware
observable 8, contracts and consumers 9, lifecycle 10. Subtasks are section 11
with `dispatch_ready: no`. Nothing dispatches until `Q-20260908-3` records
`consensus: reached`.

### One addition round 1 did not cover

Round-1 item 4 keeps the B3 bars unchanged. Bound with no other requirement
that would very likely reproduce the generation-1 failure, because deleting
the cloud is necessary but not sufficient.

PF-R9 generation 1 measured ~90 ms of publisher-thread period for ~1.8 MB per
emission, about 20 MB/s. That 1.8 MB was colour 0.92 MB plus stride-2 cloud
0.9 MB; the 0.61 MB depth image was **skipped** for want of a subscriber.
Depth-primary makes that image the canonical product, so the sim payload lands
at 1.536 MB, not 0.9 MB — about 0.55x against the 0.8 bar at the measured
ceiling.

The bar is still reachable because ~20 MB/s is not a DDS wire limit. It is
dominated by Python per-publish work, and the image half is avoidable:
`image_msg_from_frame` and `depth_msg_from_frame` each do
`np.ascontiguousarray(...).tobytes()` before rclpy copies into the field and
again into CDR, while the republished colour bytes are identical to the
received bytes.

Plan response: D1 makes the throughput budget a measured deliverable preceding
the refactor; D2 makes zero-copy republish a first-class acceptance item with
a mutation-isolation fixture; section 3 states a pre-authorised mitigation
ladder and names the two escalations that would need a new consensus item.
Lowering D3 is not on the ladder. Aligning the sim colour profile to 640x360
at 15 Hz would also cut payload and is a real parity gain, but it is excluded
because that profile belongs to `Q-20260908-2`.

This forces one architecture edit item 6 did not list: `RgbFrame` and
`DepthFrame` must carry a raw payload reference for D2 to be expressible.

### Decisions recorded

- Hardware latency limit fixed **before** dispatch per item 5: p50 <= 60 ms,
  p95 <= 150 ms, with the derivation stated in section 8.
- A DDS device drop during D8 is an outright failure, not a degraded result.
- D8 excludes world-frame geometry accuracy; that check is sequenced after
  HE-2 and does not gate PF-R9 generation 2.
- Gate 5's `BAG_MISSING_FRAME` must move to a manifest-declared optical frame
  with the topic change, otherwise a legal `d555_color_optical_frame`
  recording fails validation for the wrong reason.

### Calibration-board sequencing

Section 12 answers which work is board-independent. The board gates **only**
HE-2. The whole F3 chain, including the D8 hardware run, is runnable now on
the sim host plus the existing camera, because D8 measures stream contract and
pixel-grid agreement and neither passes through the mount transform. HE-1 is
the critical path to the board rather than a parallel task, since its
deliverable 4 is the printable asset.

## Pointers

- `docs/plans/pf_f3_depth_primary_contract.md`
- `docs/agents/discuss/2026-09-08_1651_f3-depth-primary-contract-consensus.md`
- `docs/agents/reviews/2026-09-09_1033_f3-depth-primary-consensus-round1.md`
- `docs/agents/discuss/2026-09-09_1053_pf-r9-gen1-stop-ack.md`
- `docs/status/evidence/platform_free_height/2026-09-08_pfr9_throughput/RESULT.md`
- `docs/status/evidence/d555_bringup/README.md`
