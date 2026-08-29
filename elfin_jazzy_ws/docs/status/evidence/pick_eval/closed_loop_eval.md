# Closed-loop pick / retreat eval

- Date: 2026-08-28
- Commit: ``
- `ROS_DOMAIN_ID`: 0
- N requested / completed: 20 / 20
- Config: `use_semantic=True` visual_kind=mesh sequence_ids=carryon,standard,large observe=pickup_observe backend=bbox_fill:yolov8s-world.pt
- Detection tols: xy=0.030 z=0.020 size=0.050 yaw=0.150 iou=0.60
- Retreat check: expected ΔZ=0.350 m, tol=0.080 m

## Rates (independent denominators)

| Metric | Rate | n |
|---|---|---|
| Detect pass | 57.9% | 11 / 19 |
| Plan pass (4 segments, among detect-pass) | 100.0% | 11 / 11 |
| Retreat height (among plan-pass) | 100.0% | 11 / 11 |
| End-to-end pick (retreat-ok / N) | 55.0% | 11 / 20 |

## Geometric pick (attach, no vacuum)

Attach XY error vs measured box centre; Z vs measured box top.

| | n | mean | std | p50 | p95 |
|---|---|---|---|---|---|
| attach XY vs GT (m) | 11 | 0.0094 | 0.0096 | 0.0052 | 0.0287 |
| attach |Z| vs GT (m) | 11 | 0.0076 | 0.0046 | 0.0053 | 0.0138 |
| attach XY vs measured (m) | 11 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| attach |Z| vs measured (m) | 11 | 0.0001 | 0.0000 | 0.0001 | 0.0001 |
| retreat ΔZ (m) | 11 | 0.3499 | 0.0000 | 0.3499 | 0.3499 |

## Fail codes

```json
{
  "DETECT_GATE:xy": 6,
  "DETECT_GATE:size": 1,
  "DETECT_GATE:iou": 1,
  "DETECT_ESTIMATION_FAILED": 1
}
```

## Per catalog

```json
{
  "large": {
    "n": 7,
    "detect_ok": 2,
    "plan_ok": 2,
    "retreat_ok": 2
  },
  "carryon": {
    "n": 7,
    "detect_ok": 5,
    "plan_ok": 5,
    "retreat_ok": 5
  },
  "standard": {
    "n": 6,
    "detect_ok": 4,
    "plan_ok": 4,
    "retreat_ok": 4
  }
}
```

## Per visual

```json
{
  "suitcase_vintage": {
    "n": 10,
    "detect_ok": 4,
    "plan_ok": 4,
    "retreat_ok": 4
  },
  "suitcase_loafbrr": {
    "n": 10,
    "detect_ok": 7,
    "plan_ok": 7,
    "retreat_ok": 7
  }
}
```

## Trials

| i | catalog | visual | detect | plan | retreat | attach XY | ΔZ | fail |
|---|---|---|---|---|---|---|---|---|
| 00 | large | suitcase_vintage | perception estimate (conf=1.00) | 0/0 | — | — | — | DETECT_GATE:xy |
| 01 | carryon | suitcase_loafbrr | perception estimate (conf=1.00) | 0/0 | — | — | — | DETECT_GATE:xy |
| 02 | standard | suitcase_loafbrr | perception estimate (conf=1.00) | 0/0 | — | — | — | DETECT_GATE:size |
| 03 | large | suitcase_loafbrr | ok | 4/4 | ok | 0.029 | 0.350 |  |
| 04 | carryon | suitcase_loafbrr | ok | 4/4 | ok | 0.006 | 0.350 |  |
| 05 | standard | suitcase_loafbrr | ok | 4/4 | ok | 0.008 | 0.350 |  |
| 06 | large | suitcase_vintage | perception estimate (conf=1.00) | 0/0 | — | — | — | DETECT_GATE:xy |
| 07 | carryon | suitcase_vintage | ok | 4/4 | ok | 0.005 | 0.350 |  |
| 08 | standard | suitcase_vintage | ok | 4/4 | ok | 0.003 | 0.350 |  |
| 09 | large | suitcase_loafbrr | ok | 4/4 | ok | 0.028 | 0.350 |  |
| 10 | carryon | suitcase_loafbrr | ok | 4/4 | ok | 0.006 | 0.350 |  |
| 11 | standard | suitcase_vintage | ok | 4/4 | ok | 0.004 | 0.350 |  |
| 12 | large | suitcase_vintage | perception estimate (conf=1.00) | 0/0 | — | — | — | DETECT_GATE:xy |
| 13 | carryon | suitcase_vintage | perception estimate (conf=1.00) | 0/0 | — | — | — | DETECT_GATE:iou |
| 14 | standard | suitcase_loafbrr | DETECT_ESTIMATION_FAILED | 0/0 | — | — | — | DETECT_ESTIMATION_FAILED |
| 15 | large | suitcase_vintage | perception estimate (conf=1.00) | 0/0 | — | — | — | DETECT_GATE:xy |
| 16 | carryon | suitcase_loafbrr | ok | 4/4 | ok | 0.005 | 0.350 |  |
| 17 | standard | suitcase_vintage | ok | 4/4 | ok | 0.004 | 0.350 |  |
| 18 | large | suitcase_vintage | perception estimate (conf=1.00) | 0/0 | — | — | — | DETECT_GATE:xy |
| 19 | carryon | suitcase_loafbrr | ok | 4/4 | ok | 0.005 | 0.350 |  |

## Known deviations

- No vacuum / ACM attach: a successful pick is geometric (PlanMotion success + retreat ΔZ), not grasp retention.
- MoveIt planning scene does not contain the suitcase collision object; attach may physically contact the box in Gazebo.
- `keep_camera_down` / `lock_wrist` are not implemented.
- Detection tols were not retuned for this run.

