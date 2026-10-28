# G4-B Stage 1 定向审查

- revision: fd793593ba34d3c9af376b6aac52415f9d78989e
- reviewer: reviews/codex/gpt-5
- verdict: preparation_requires_fixes（Binding保留，评分未准入）

## 验证范围

相对0da4086b只有scripts/g3/occ_g4b_capacity_core.py和src/luggage_perception/test/test_g4b_capacity_core.py新增，无产品修改。独立运行 `python3 -m pytest -q src/luggage_perception/test/test_g4b_capacity_core.py`：11 passed in0.02s。另直接import真实核心执行下列反例，未跑ROS/性能/仿真。

## Findings

### P1：四分类没有coalesced_into分支（核心155-189行）

输入commit A@1.0，READY@1.5 target_identity=B、covered_members={A,B}，实际输出A exact_visible；代码只会返回exact_visible/timeout/export_failed。测试名all_four_classes也只断言三类。按合同必须显式核对READY目标身份：同identity才exact，不同identity但有合法覆盖账证明才coalesced。不能以members任意交集当整个commit覆盖；须使用提交唯一身份及对应export链。补精确/合批/错世界或错覆盖/超时/导出失败各自反例，同判官同测试。

### P1：C2闭合判官接受不完整或非法记录（138-152行）

`c2_closure([{'frame_id':'f','admitted':True,'class_':'queued','terminal':'garbage'}])`输出complete=true，无合法终态、无提交或传播证据仍过。`[{'frame_id':'f','admitted':False,'class_':'named_rejected','terminal':'rejected'}]`则closure_holds=false，实际没有第三类具名拒绝/失败分账。queued/sent还不是服务端accepted。

改为每个唯一计划frame的状态转移及互斥终态计数：未准入、发送后拒绝/失败、服务端已接纳分别核对；已接纳且产生commit必须有合法传播分类，接纳后写失败保留具名失败而不是捏造commit。未知终态、重复/缺frame、未完成queue、缺传播记录均不可complete。输入总量须与固定240个计划ID集合join，不能只对收到的记录自计数。

### P1：admit_c2不是动态准入状态机（109-135行）

每次调用从空状态开始，只累计send_inflight/queued，完全不接受完成/拒绝事件；整窗输入只能发首笔、排第二笔，后续全部未准入。逐arrival调用又会每次清空状态，无法执行单在途限制。当前只实现“永不完成的饱和特例”，不能作为正常C2共用核心。

增加显式state+arrival/completion/accept/reject/queue-promotion事件接口（纯函数返回新状态也可），运行器和测试共用。快写20ms/0.5s到达应全240笔完成；慢写应排队/拒绝/完成后晋升且无饥饿；同刻完成与到达顺序须确定；drain30s未终结不得complete。不得另写运行器私有逻辑让测试只验证静态替身。

### P2：CURRENT只有汇总检查，不能证明重试预算（94-106行及测试）

探针20条重复check_id=0、attempts=0、total_ms=-1、outcome=ok，实际complete=true。即使输入都为正常正数，现有字段也不能证明20个独立计划检查、50ms重试、非传播错误立即终止或跨代固定句柄。test_bounded_current_budget_never_resets只构造一条timeout，并未驱动重试状态机。

补check_id及scheduled/start/attempt/result/end时间线、具名reason和identity/token；用fake clock驱动真实核心验证首个绝对deadline不重置、仅传播等待可重试、成功立即终止、不接受期限后成功。20个不同检查各有独立2s期限，不共享跨检查预算。判官核对唯一ID集合和有限非负时间，重复或缺检查不得过；固定句柄仍由后续接线行为验证。

### P2：调度窗口结束后仍能新发请求（63-83行）

`QueryScheduler({'roi':5},window_s=1).advance(1.2)`输出['roi']、issued=1、missed=4。评分截止不得发送历史due。补now>=window边界：剩余due逐项具名missed，但不得新发；中途调度停顿需保留scheduled与actual send时间，不把延迟到达冒充按时。due_times测试10Hz/120s应精确1200而非>=119。

## 后续范围

仅修scripts和专用tests，增加上述真实判官反例。Binding与产品0da4086b不变，A/A7/A8/A9不重开；不需要用户再授权。Stage1尚非完整C1/C2交付，评分、资源代表组和全矩阵继续等待准备门完成。修复后回报脚本SHA及定向测试，不增加性能重跑。
