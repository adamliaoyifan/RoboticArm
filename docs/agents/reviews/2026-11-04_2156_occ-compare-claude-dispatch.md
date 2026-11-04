# 2026-11-04 -- 双后端比较完整合同与Claude派发

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

按用户要求补全并批准G3执行合同：实际产品a4ac889e/cbdd5364、工具9cc59da1单列、共同C及adapter边界、确定性输入选择与时钟、请求账本/终态/截止、完整查询与选位时钟、分母/分位/超时删失/比值、CPU进程计费和RSS/PSS采样、C0–C8验收、15类判官反例、阶段dump矩阵及预算/收口。使用方geometry/frame/unit拆为3项，共10状态×2后端。既有结构/reset收口不重开，不带入1cm大矩阵或新长稳。

Owner按用户指定改为eng/claude/glm-5.3，文件派发单一OCC-BACKEND-COMPARISON gen3任务，准备通过后自动冻结计分，真实失败/未评也能工程归档。明确计划及工具尚未执行，不代表owner已经Claim。保留主workspace在途源码与暂存区，计划文件独立提交；派发线程绑定该确切plan_revision。

## Verification

核对现有路由与notify接口，静态检查三份合同引用、样本算术（B1每侧1440更新/查询，B2每侧720链，B3每侧1800输入/1440查询）、10状态数量、截止/分位定义和scope一致性。未运行产品、ROS、仿真或比较工具。

## Pointers

- docs/plans/OCC_DUAL_BACKEND_COMPARISON_G3_20261104.md
- docs/plans/OCC_DUAL_BACKEND_CONSUMERS_G3_20261104.md
- docs/plans/OCC_DUAL_BACKEND_DATA_ACCEPTANCE_G3_20261104.md
- docs/agents/reviews/2026-11-04_2132_occ-next-plan-status.md
