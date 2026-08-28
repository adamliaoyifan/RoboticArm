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
| Detect pass | 100.0% | 6 / 6 |
| Plan pass (4 segments, among detect-pass) | 0.0% | 0 / 6 |
| Retreat height (among plan-pass) | 0.0% | 0 / 0 |
| End-to-end pick (retreat-ok / N) | 0.0% | 0 / 20 |

## Geometric pick (attach, no vacuum)

Attach XY error vs measured box centre; Z vs measured box top.

| | n | mean | std | p50 | p95 |
|---|---|---|---|---|---|
| attach XY (m) | 0 | — | — | — | — |
| attach |Z| (m) | 0 | — | — | — | — |
| retreat ΔZ (m) | 0 | — | — | — | — |

## Fail codes

```json
{
  "PLAN_pre_grasp": 6,
  "GOTO_FAILED": 14
}
```

## Per catalog

```json
{
  "carryon": {
    "n": 7,
    "detect_ok": 2,
    "plan_ok": 0,
    "retreat_ok": 0
  },
  "standard": {
    "n": 7,
    "detect_ok": 3,
    "plan_ok": 0,
    "retreat_ok": 0
  },
  "large": {
    "n": 6,
    "detect_ok": 1,
    "plan_ok": 0,
    "retreat_ok": 0
  }
}
```

## Per visual

```json
{
  "suitcase_vintage": {
    "n": 8,
    "detect_ok": 3,
    "plan_ok": 0,
    "retreat_ok": 0
  },
  "suitcase_loafbrr": {
    "n": 12,
    "detect_ok": 3,
    "plan_ok": 0,
    "retreat_ok": 0
  }
}
```

## Trials

| i | catalog | visual | detect | plan | retreat | attach XY | ΔZ | fail |
|---|---|---|---|---|---|---|---|---|
| 00 | carryon | suitcase_vintage | ok | 0/4 | — | — | — | PLAN_pre_grasp |
| 01 | standard | suitcase_loafbrr | — | 0/0 | — | — | — | GOTO_FAILED |
| 02 | large | suitcase_loafbrr | ok | 0/4 | — | — | — | PLAN_pre_grasp |
| 03 | carryon | suitcase_loafbrr | — | 0/0 | — | — | — | GOTO_FAILED |
| 04 | standard | suitcase_vintage | ok | 0/4 | — | — | — | PLAN_pre_grasp |
| 05 | large | suitcase_vintage | — | 0/0 | — | — | — | GOTO_FAILED |
| 06 | carryon | suitcase_loafbrr | — | 0/0 | — | — | — | GOTO_FAILED |
| 07 | standard | suitcase_vintage | ok | 0/4 | — | — | — | PLAN_pre_grasp |
| 08 | large | suitcase_vintage | — | 0/0 | — | — | — | GOTO_FAILED |
| 09 | carryon | suitcase_loafbrr | — | 0/0 | — | — | — | GOTO_FAILED |
| 10 | standard | suitcase_loafbrr | ok | 0/4 | — | — | — | PLAN_pre_grasp |
| 11 | large | suitcase_vintage | — | 0/0 | — | — | — | GOTO_FAILED |
| 12 | carryon | suitcase_loafbrr | ok | 0/4 | — | — | — | PLAN_pre_grasp |
| 13 | standard | suitcase_vintage | — | 0/0 | — | — | — | GOTO_FAILED |
| 14 | large | suitcase_loafbrr | — | 0/0 | — | — | — | GOTO_FAILED |
| 15 | carryon | suitcase_loafbrr | — | 0/0 | — | — | — | GOTO_FAILED |
| 16 | standard | suitcase_loafbrr | — | 0/0 | — | — | — | GOTO_FAILED |
| 17 | large | suitcase_loafbrr | — | 0/0 | — | — | — | GOTO_FAILED |
| 18 | carryon | suitcase_loafbrr | — | 0/0 | — | — | — | GOTO_FAILED |
| 19 | standard | suitcase_vintage | — | 0/0 | — | — | — | GOTO_FAILED |

## Known deviations

- No vacuum / ACM attach: a successful pick is geometric (PlanMotion success + retreat ΔZ), not grasp retention.
- MoveIt planning scene does not contain the suitcase collision object; attach may physically contact the box in Gazebo.
- `keep_camera_down` / `lock_wrist` are not implemented.
- Detection tols were not retuned for this run.

