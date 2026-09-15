# 现场进度（2026-09-15 19:57 +08）

权威叙述在 `deployment_ws/SITE_EXECUTION_README.md`（与 `SITE_EXECUTION_PLAN.txt` 同步）的 **「当前测试进度」** 节。本文件是证据目录索引。

**执行树：** `/home/adamliao/work/RoboticArm-master`  
**Domain：** `ROS_DOMAIN_ID=7`  
**下一闸：** Gate 5 密封 pick 5/5（未跑）

## 硬约束：真空接触面不能有高低差

当前真空只在杯口压**一张无高低差的平面**时才能密封。

- 有高低差（凸起 / 凹陷 / 台阶 / 折痕 / 拉杆）→ 泵开 8 s，DI0 仍为 0 → **FAIL**
  - `20260915_1952_g4_contact/`
  - `20260915_1954_g4_contact2/`
- 无高低差平面 → `sealed in 1.87 s` → **PASS**
  - `20260915_1956_g4_flat/`
- 空中短测只验针脚，不验密封
  - `20260915_1948_g4/`

Gate 5 必须用无高低差平面顶。漏气时加长超时无效。

## 闸门

| 闸 | 结果 | 目录 |
|---|---|---|
| 0 / 0.5 | PASS | `20260915_1815_g0_g05/` |
| 1c Livox overlay | PASS | `20260915_1825_g1c_g1d_pp/` |
| 1d detect B 3/3（YOLO bbox 裁剪） | PASS | `20260915_1905_g1d_yolo_crop/` |
| 2  2° FJT | PASS | `20260915_1929_g2/` |
| 3 A/B detect+plan-only（`--skip-observe`） | PASS | `20260915_1936_g3/` |
| 4 针脚 | PASS | `20260915_1948_g4/` |
| 4 接触密封（平面） | PASS | `20260915_1956_g4_flat/` |
| 5 sealed pick 5/5 | 未跑 | — |
