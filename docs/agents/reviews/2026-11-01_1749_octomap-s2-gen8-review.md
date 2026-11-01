# 2026-11-01 -- S2 gen8：累计分配不是峰值，S3仍关闭

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

候选03334c0b工作树clean，FREEZE五个二进制/库SHA一致，O9独立重判30/30 trace完整；不否定已测O9。独立固定s2g8安装执行test_repair_s0及test_s2*offline，103 passed、10 deselected（live和持锁实机排除）；未重跑ROS/O9计时。gen8已实现全区域体素读取及多项旧反例拒绝，但S2全通过仍不成立，S3保持关闭。继续沿原合同补两类缺陷，不能用scope声明改变512MiB峰值门。

## Evidence and observations

本轮audit.py以保存的gen8实测O8 artifact为健康对照，健康PASS；分别离线注入以下错误仍PASS，不表示实机曾返回这些错误：

- duration_measured_s=NaN或Inf：hold时长比较未检有限性。
- 两个探针都删除log_odds_hash：None==None使“概率未变”成立。
- after探针改为1格及不同region，保留哈希：判官不核对前后区域/数量一致性，仍通过。
- 实际资源适配器输入read_view_bytes=1024、cache_bytes=1024，但read_view_bytes_peak=1GiB、read_view_alloc_bytes=2GiB；_peak_resources_from_documents→_resources仍pass。正式门只取采样当前值，已携带的峰值被忽略。
- 在健康资源行后加入read_view_bytes=NaN的行，适配器peak()跳过该值，判官仍pass。坏样本从统计中消失，违反既有缺字段/非有限值不得通过规则。

## Source-established mechanisms

AllocationTotals只有add，无释放扣减；total是sum(历次分配)，不是max_t(同时存活字节)。反复分配/释放100MiB，真实峰可始终100MiB而累计达到任意大。它能证明发生过分配，不能代替峰值测量；把累计直接接入512MiB判官也会错拒健康复用。原read_view_bytes_peak/cache_bytes_peak仍只在usage采样时更新，仍可漏瞬态。新字段又未进入资源判官，故当前资源资格尚未完成。

_hold_probe读取states/log_odds后只返回哈希，没有保存数组或可验证的内容文件引用；判官所谓“重算”是两个哈希字符串比较，不是从内容复算。当前API读取这两数组是正确进展，但独立复算证据缺失。ReadCells循环未核对page.identity，log_odds被round到6位且未检非有限值；Surface循环未核对identity/游标重复。完整身份、内容和失败释放规则仍需落实，不能仅自报pages_done。

## S2 gen9 acceptance supplement

继承813f78b5及原验收1–8、门限/矩阵/预算，不扩大产品范围。owner eng/claude/glm-5.3，基线03334c0b。只完成下面两项后报告，S3仍不领取：

1. 真实资源峰值及端到端判定。
   - 分配增加current、最后所有者释放减少current，同一所有权域维护peak=max(peak,current)；累计流量可另列，名称与门用途分开。共享对象去重，容量扩缩及控制块/目录/cache按原所有权清单计入；需要可审计的同一时刻read_view+cache总峰值，不能把不同时刻单类峰相加后冒充实测同时峰。
   - 计分适配器/判官必须消费真实峰值，缺峰值或覆盖不足NOT_EVALUATED；任何行缺失/bool/NaN/Inf/负值不得由max过滤。高峰1GiB但采样当前仅2KiB反例必须FAIL；512MiB边界健康对照按原<=门通过。
   - 独立行为测试：同对象尺寸顺序分配释放N次时累计增长、真实峰不随N增长；并存两对象峰增加；共享多句柄不倍增；采样间瞬态超门必须捕获；最后释放活值回基线。穿过产品→资源接口/CLI→adapter→artifact→judge，不只检查类自身total增加。若尚做不到，按NOT_EVALUATED/blocked收口，不再pass_with_scope。
2. 概率oracle证据与fail-closed验证。
   - 同租约完整读取固定旧区域state/log_odds（含unknown），无损保存压缩内容或可独立校验内容引用及SHA/身份/区域/格数；判官或独立复算入口必须校验实际内容、计算SHA再判概率不变。不得只比较两个自报字符串；数值精度规则不得未经合同用round掩盖变化。
   - 校验两探针同region/shape/cell count，完整分页身份/游标/数量/状态、有限log_odds及有效state；失败路径finally释放租约，记录释放结果。改变一格概率但几何不变必须FAIL；缺哈希/缺内容/假SHA/不同区域或1格/身份错/重复分页/NaN值均拒绝。
   - duration_measured_s以及所有年龄/health/RSS/资源输入统一拒bool、NaN、Inf、负值、缺失；本轮audit所有反例反转，保存实测artifact健康对照仍通过。保持原10s hold合同（不得新增9s通过豁免）；实测no_input/遮挡/重复ID概率oracle接线通过，按原O8资格规定复测。

先离线和小规模非计分行为，失败证据持久化后再重跑。修复后精确COMMANDS与候选/输入/二进制冻结，重测受影响O9。保留gen6/7/8历史，不启动正式O5–O8，不用已知正式延迟风险阻止完成资格。预演预算2h不含等锁、容量预算沿原计划，若不足如实not_evaluated。工程交付与产品验收继续分开。

## Pointers

- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen8_review/audit.py
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen8_review/RESULTS.json
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen8_review/pytest.log
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen8_review/o9_rejudge.json
- docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_gen8/FINAL_REPORT.md
- docs/agents/reviews/2026-11-01_1503_octomap-s2-gen7-review.md
