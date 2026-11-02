# OCC-FREESPACE-IMPLEMENT gen2 — 复核五组缺陷一次补齐

- owner: eng / claude / glm-5.3（Q-20261102-3，2026-11-02 14:19 领取；复核 fd84b8b5）
- worktree: /tmp/elfin-occ-freespace-impl-g2，分支 agent/eng/occ-freespace-impl-g2，自 65766502
- freeze: FREEZE.json head ca28c885（clean；6 二进制 + 12 工具/静态库/typesupport/
  消费者测试 + 6 输入帧 + 2 fixture 按实际文件字节 + COMMANDS 本身；判官字段
  修正后重新冻结，SHA 全一致）
- 正式评分未启动：score 默认关闭（实现完整、有界同源预演已验证）；S3 仍不开放
- 证据根: docs/status/evidence/occ_freespace_final_20261102/implementation_g2/

## 复核五组缺陷的处置（全部为原合同未落实项，无新增指标）

1. **P/H/R/L 判官完整性** — 状态/身份/释放结果/数值健全在短样本与 formal 两路
   都必查（240 个 INTERNAL_ERROR、600 空行、零秒/NaN 峰全部转红并有健康对照）；
   formal 分支补齐逐 ID 恰一成功终态、输入 350/500ms、查询 250/500ms、调度
   20/50ms（去掉十倍放宽与缺失 max）、快照年龄 ≤1s、health 间隔、窗口 0.5s 闭合、
   过载真实压力 + 排空截止 0 在途 + 同 epoch 恢复（epoch_before/after 相等）、
   L 120min 时长 + 110min 评分 + RSS 十分位中位增长 ≤5% + 每桶 ≥600 样本
   （末尾不完整桶豁免，中间稀疏必拒）。
2. **完整查询与租约** — full_query 于 finally 释放（超时路径已测释放）；全候选
   物化 + 与 O9 CLI 同一 canonical 摘要；页身份=租约身份、数组长度/verify 3n、
   游标进展 min(cursor+limit, feasible)、物化数==feasible_total；释放状态入结果。
   1/100 短页、错身份、游标停滞、释放失败、超时全部点名并保租约闭环。
3. **调度接线与 TIMEOUT 定位** — R 过载输入与查询并发；恢复首输入与首
   WaitSnapshot 同一 t0 同时开始；排空按真实回执时刻判定（在途清单保留）；
   L 绝对时钟循环（查询耗时不再拉长发送周期）+ 完整查询记录 + 逐帧终态时间线
   + 1Hz 节点 RSS 序列；executor 显式 4 线程；输入 stamp 每周期推进（匹配实际
   频率，地图年龄随墙钟）；终态收集只认合法终态（QUEUED/PROCESSING 非终态）；
   R 保存发布/回调/回执/Wait/快照时间线。
4. **可运行评分、冻结与失败重放** — score 完整实现且默认关闭：formal 走同一
   campaign 源（P/H/R/L）+ 既有 stage runner（O8/O5/O6）；OCC_FS_SCORE_REHEARSAL=1
   有界同源路径已实跑验证（campaign wiring_ok + O8 bounded）。freeze 记录并按
   字节校验工具/静态库/typesupport/消费者测试/每帧输入/fixture/命令脚本；
   require_freeze 验文件 SHA 非 descriptor 自报。judge 从文件重判全部 F 案例与
   各阶段工件（不信 summary 标志）。replay 保留退出码。verify 入口对守卫本身
   做负例（未授权 score、篡改 SHA 的 o9 都必须拒）。
5. **完整链 O9 边界** — profile_cli 的 freespace_full 时钟移到 acquire 之前
   （全链=acquire→全页→释放，acquire 段单独分列 freespace_acquire_ms）；修复后
   重冻结（ca28c885）再测四操作 30 对：**verdict pass、performance_certified=
   true、零失败**（等锁 3.0μs）—— roi 1.0105 / integrate_30k 1.0097 /
   surface_first 0.9794 / **freespace_full 0.9981** 全 ≤1.10，30 对每对的
   freespace 候选摘要 ON/OFF 一致；epoch-reject 重放执行成功。

## 验证

- 离线：test_freespace_g2_offline 18 通过（audit 三反例永久红 + full_query 五
  故障注入 + 健康 formal 对照绿 + 超门/计数谎报/身份漂移拒绝 + score 未授权拒）；
  gen1 offline 对齐新判官后 24 通过；全套 82 passed。
- 行为二进制（freespace/repair_s1）通过。
- 短接线预演（新调度器）：P/R/L 全部仅 short_sample 标记，wiring_ok=true。
- 有界 score 预演：campaign wiring_ok=true + O8 bounded（非计分、不可认证）。
- F 矩阵（g2 重跑）：**360/360** 通过；judge 入口从磁盘逐案重判（含全部阶段
  工件与 O9 verdict），problems=[]。

## 诚实边界

- 正式 P/R/L/O8/O5/O6 未启动；有界路径不构成任何正式结论。
- 短样本 R 的首 Wait 在真实并发下仍可能 TIMEOUT——判官只验形状并保留时间线，
  ≤2s/年龄≤2s 属 formal 门；S3 正式 R 以时间线定位而非臆断吞吐。
- 输入 stamp 采用计划周期步进；与"真实采集时间戳"的严格一致由 S3 formal 输入
  manifest 绑定（本代帧来自固定 emit-window 生成器）。
