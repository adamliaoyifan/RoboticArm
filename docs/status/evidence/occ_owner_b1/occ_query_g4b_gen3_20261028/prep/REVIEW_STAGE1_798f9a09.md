# G4-B Stage1第二轮复核

- revision: 798f9a09
- verdict: preparation_requires_fixes
- scope: scripts/tests，产品0da4086b不变，不撤回Binding

## 验证

20项现有行为测试独立通过（0.02s）。diff相对0da4086b仍仅核心脚本和专用测试。窗口后不发、动态晋升、原垃圾终态/重复检查ID等反例有实际修复，但不接受“五项合同全部关闭”。

独立反例脚本：`python3 docs/status/evidence/occ_owner_b1/occ_query_g4b_gen3_20261028/prep/review_798f9a09_probe.py /tmp/elfin-occ-g4b-gen3`。真实核心结果如下，无ROS/仿真/性能运行。

## P1 CURRENT状态机仍允许预算重置和非法重试

核心142-145行start可反复调用并覆盖deadline；154-158行只看outcome标签、不验reason也不强制next_retry；187行仅排NaN和负数、不排正无穷；判官只信finished。

- 20个不同检查均先start(s)、wait(s+1.9)、再start(s+2)、ok(s+3)，实际20/20 complete=true。以首次start为基准已超2s。
- wait(reason=PERMISSION_DENIED)后仅1ms就ok(identity=None)，20/20 complete=true。非传播错误、未守50ms且无身份均被接受。
- start(inf)、ok(inf)，20/20 complete=true。

要求：start只允许一次；状态机保留首次绝对deadline和next_retry，验证单调有限时间、具名允许的传播reason、成功身份；判官从事件轨迹独立核对，不能仅信可变finished。scheduled_s需与固定check_id计划对应。每次调用参数不合法须具名拒绝/判失败，不能靠运行器自觉。可用fake clock完成，不新增实测。

## P1 合批覆盖仍保留任意members交集的后门

338-339行仍用 `identity in covered_members OR members交集`。commit A、members={SHARED}、ledger A coalesced into B、READY B covered_members={SHARED}（根本没有A），实际输出coalesced_into。

要求：exact匹配完整提交identity与READY target；coalesced必须由同世界export账本及covered_members明确证明A本身被B覆盖，不能共享某个旧成员就算A送达。否则显式coverage_invalid并使有效性门失败，不能误记可见或从分母删除。补上述反例；审查first READY选择与合批链累积覆盖是否一致，不能用一个无关早期交集抢占后续合法事件。

## P2 C2仅frame_id join，错误输入仍可闭合

271-275行只比ID集合，计划input_hash=correct、实际同ID input_hash=wrong，设置合法终态与传播后complete=true。这会让运行器送错/替换输入仍把完整矩阵签为通过。

要求：join固定计划frame_id、input_hash及计划到达时刻，实际发送/到达延迟单列；不要求实际时刻等于计划时刻。判官不将独立问题条数当frame数加入闭合公式（当前299-302行len(problems)还含全局drain错误，不是互斥分账）。缺失/非法传播应让complete=false，但账本完整不代表传播性能PASS，timeout/export_failed仍由独立性能/正确性门判失败。

## 执行边界

此次限以上三项的同核心修复和行为测试，不重开已闭合调度窗口/动态队列，不撤回产品Binding。Stage2不涉及判官的进程接线可继续准备，但正式评分与代表组仍等待全部准备门。不能把更多进程harness测试当上述确定性反例的替代。A门、产品、性能预算均不动。
