# Closed-loop pick / retreat eval

- Date: 2026-09-01
- Commit: ``
- `ROS_DOMAIN_ID`: 7
- N requested / completed: 4 / 4
- Config: `use_semantic=True` visual_kind=mesh sequence_ids=large observe=pickup_observe backend=unknown
- Detection tols: xy=0.030 z=0.020 size=0.050 yaw=0.150 iou=0.60
- Spawn-visual tols: xy=0.080 z=0.060
- Retreat check: expected ΔZ=0.350 m, tol=0.080 m

## Rates (independent denominators)

| Metric | Rate | n |
|---|---|---|
| Detect pass (vs GetCurrentBox catalog AABB, diagnostic) | 0.0% | 0 / 0 |
| Detect usable (perception estimate) | 0.0% | 0 / 4 |
| Plan pass (4 segments, among detect-usable) | 0.0% | 0 / 0 |
| Retreat height (among plan-pass) | 0.0% | 0 / 0 |
| End-to-end pick (retreat-ok / N) | 0.0% | 0 / 4 |
| YOLO ready (post-mask cargo box) | 100.0% | 4 / 4 |
| Spawn visual = GT (depth blob) | 0.0% | 0 / 4 |

## Geometric pick (attach, no vacuum)

Attach XY error vs measured box centre; Z vs measured box top.

| | n | mean | std | p50 | p95 |
|---|---|---|---|---|---|
| attach XY vs GT (m) | 0 | — | — | — | — |
| attach |Z| vs GT (m) | 0 | — | — | — | — |
| attach XY vs measured (m) | 0 | — | — | — | — |
| attach |Z| vs measured (m) | 0 | — | — | — | — |
| retreat ΔZ (m) | 0 | — | — | — | — |

## Spawn → detect latency (sim time, s)

Clock starts when `SpawnNextBox` returns. YOLO ready is the first post-mask-filter cargo box (`raw_cargo`) for that generation. Spawn visual is when the platform depth blob matches this spawn's catalog AABB (GetCurrentBox). DetectLuggage is scored against that same AABB. A leftover previous mesh fails at the visual gate, not DETECT_GATE. A size/iou miss does not abort the pick unless `--strict-gt`. Detect is `DetectLuggage` cloud stamp. `YOLO_NOT_READY`, `TRACKER_STALE`, `CARGO_NOT_READY`, and `SPAWN_VISUAL_TF` stay blocking. `SPAWN_VISUAL_MISMATCH` and `DETECT_GATE:*` are recorded but not blocking by default.

| | n | mean | std | p50 | p95 |
|---|---|---|---|---|---|
| spawn → YOLO box | 4 | 0.1322 | 0.0973 | 0.1445 | 0.2156 |
| spawn → visual=GT | 0 | — | — | — | — |
| spawn → DetectLuggage | 0 | — | — | — | — |

## Fail codes

```json
{
  "SPAWN_VISUAL_TF": 4
}
```

## Per catalog

```json
{
  "carryon": {
    "n": 4,
    "detect_ok": 0,
    "detect_usable": 0,
    "plan_ok": 0,
    "retreat_ok": 0
  }
}
```

## Per visual

```json
{
  "suitcase_loafbrr": {
    "n": 3,
    "detect_ok": 0,
    "detect_usable": 0,
    "plan_ok": 0,
    "retreat_ok": 0
  },
  "suitcase_vintage": {
    "n": 1,
    "detect_ok": 0,
    "detect_usable": 0,
    "plan_ok": 0,
    "retreat_ok": 0
  }
}
```

## Trials

| i | catalog | visual | detect | usable | plan | retreat | attach XY | ΔZ | fail |
|---|---|---|---|---|---|---|---|---|---|
| 00 | carryon | suitcase_loafbrr | — | — | 0/0 | — | — | — | SPAWN_VISUAL_TF |
| 01 | carryon | suitcase_loafbrr | — | — | 0/0 | — | — | — | SPAWN_VISUAL_TF |
| 02 | carryon | suitcase_loafbrr | — | — | 0/0 | — | — | — | SPAWN_VISUAL_TF |
| 03 | carryon | suitcase_vintage | — | — | 0/0 | — | — | — | SPAWN_VISUAL_TF |

## Known deviations

- `--use-vacuum` (default off) adds PlanningScene `pickup_box` before BuildMotionSequence, VacuumCommand after attach, and release after retreat. Geometric pick remains the n50_v2 baseline.
- MoveIt planning scene contains the suitcase only when `--use-vacuum`.
- `keep_camera_down` / `lock_wrist` are not implemented.
- Detect overlay GT is GetCurrentBox catalog AABB at the spawn origin, not the mesh lid-band or the depth-blob silhouette.
- Visual gate compares the platform depth blob to that AABB. `SPAWN_VISUAL_MISMATCH` is kept if later DETECT_GATE also fires.
- `pickup_observe` camera at about (-1.0, 0, 1.9), optical +Z down, centred on the pickup platform so large (0.80 m) fits in the D435 FOV. Older (-0.8, 0, 1.7) clipped the image left.
- `SPAWN_VISUAL_MISMATCH` and `DETECT_GATE:*` are diagnostic unless `--strict-gt`. They do not abort the pick.
- `SPAWN_VISUAL_TF` is a camera TF lookup failure during the visual gate, not a leftover mesh.
- `TRACKER_STALE` means YOLO already had a post-mask cargo box for this spawn, but the cargo tracker was still on clear / the previous generation. `CARGO_NOT_READY` is reserved for a matching epoch with zero cargo points.

