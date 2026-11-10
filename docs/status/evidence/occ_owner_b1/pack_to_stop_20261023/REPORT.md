# 持续装箱到终止 — 2026-10-23

## 结果

**完整闭环成功5箱；第6箱抓取附着后，放置点重算返回PLACE_CANDIDATE_EXHAUSTED，流程停止。不是BIN_FULL，不能据此认定容量为5箱。**

单次运行，无跳箱、人工放置或失败重跑。开始14:28:43，驱动结束14:57:32，所属仿真14:57:33清理；suite计时1726.6s（约28分47秒）。退出码1表示装箱流程失败终止，观察任务已完成。

冻结版本47e8560cce366616b260b56aa9c3189f555d561c；相对上轮验证产品ddc35211的src/差异为空。只新增计划/证据和验证脚本上限参数。场景A-prime/603f9b6523e104f675cbeea77805e1ff，domain7，seed0，inch20重复序列，180s选择，prior关闭，包络sha256=3fa5139862ff950eb84ed5e139752e4d9776ded7d75007ba8ee9209cecae9956；XY<=5mm/Z仅观察。max_boxes999、timeout43200s均未触及。

## 逐箱核对

| 箱序号 | 结果 | 释放XY偏差 | 提交/回程冻结地图版本 |
|---|---|---|---|
| 1 | 释放/验证/回观察/记账全部通过 | 0m | 2/2 |
| 2 | 全部通过 | 0m | 3/3 |
| 3 | 全部通过 | 0m | 4/4 |
| 4 | 全部通过 | 0m | 5/5 |
| 5 | 全部通过 | 0m | 6/6 |
| 6 | 附着成功；放置重算无候选，未建携带路线、未释放、未提交 | 未评估 | 当前地图6 |

前5箱均有指令后的释放确认、verify=ok、return_observe=true，提交地图与回程admission/frozen版本和digest一致。audit核对ledger_committed=5、complete_closed_boxes=5、suite_boxes_packed=5。账本共6条含失败箱；suite.boxes_attempted=5是终止分支没有递增seq的既有统计行为，不能据此说仅尝试5箱。本轮未改产品修复计数。

## 终止证据和解释边界

直接观察：第6箱vac_attach=attached measured pickup_box_0006_inch20，payload_sync generation12成功；预抓取名义预览feasible10/returned5。附着后实际规划返回：PLACE_CANDIDATE_EXHAUSTED no_candidate: outside_hull=141 unknown_above_floor=59。trial.place_state=INIT、segments_planned=0、segments_succeeded=0，后续验证未发生。

源码机制：place_smoke_driver在确认附着后使用Attachment提供的conservative_geometry重新计算实际放置候选。outside_hull表示越出可用容器边界；unknown_above_floor表示高于地面的候选支撑证据不足。不能由汇总推断物理舱已满、轨迹碰撞或裕量过大是唯一根因。没有通过改变裕量/门槛继续装箱。

suite体积占比4.20%、footprint/floor比37.09%，仅作其输出口径描述，不证明容量。catalog_counts=unknown也是既有元数据不足；inch20输入由manifest/sequence/trial独立确认。

证据限制：失败箱compute_placement.json、reject_histogram.json为空；拒绝汇总实际在trial.extras.compute_placement字符串。通用replay manifest误标motion且缺last_boundary；新增failure_manifest纠正阶段，明确缺少实际附着后完整请求/响应、conservative_geometry和逐候选记录，capture_complete=false/replay_possible=false。检测、预览请求、地图、场景、关节窗口及日志均已保存，但不足以精确回放该次放置计算。未盲目重跑；若继续诊断，应先补齐该边界采集。现有release/joint捕获上限也不支持整场逐时物理无碰撞声明。

## 复核及产物

python3 <root>/audit_live.py <root>/live_01：6账本项、5完整通过、与suite成功数一致。bash -n scripts/g3/n3_functional_run.sh通过；本轮未改产品，不重复上轮188测试+70子测试和构建。

- live_01/driver_out/suite.json、ledger.jsonl、dumps/box_05_PLACE_CANDIDATE_EXHAUSTED/trial.json。
- live_01/failure_manifest.json：缺失项与阶段说明。
- live_01/driver_out/final_layout/layout.html、occupancy_gt.png：离线GT参考，未作在线规划输入。
- final_state.json：产品源码无变化、仅预存未追踪pydeps-frozen、无残留仿真。
- 原始产物全量复制主workspace；相对路径/大小/sha256见artifact_manifest.json。
