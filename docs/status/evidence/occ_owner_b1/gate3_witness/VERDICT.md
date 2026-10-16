# Gate-3 n=2 功能见证 · 判定书（2026-10-16）

## 执行与冻结

- 正式见证 = r10（`master_pack_recovery/2026-10-14_g3/gate3_witness_b1r10`），
  载体 scripts/g3/run.sh score（score_run01 同源），冻结 HEAD **473c028d**
  （B1→B3+B4Gate2 全部提交+见证过程揭出的四个修复），install 同步重建。
- 见证过程累计揭出并修复（全部先离线钉测试再重冻结）：v2 投影量化越界
  （7391f460）、st09_contracts 部署缺口（重建）、place_smoke 缺摘要助手
  （a19760fd）、placed 门计数语义（473c028d）。r0–r7 均为前置段中止
  （launch 门/就绪抖动/租约竞态），零箱；r8–r9 部分执行；**额度按一次
  n=2 见证计（r10）**。

## 判定：FAIL（超差 2.8 mm）

| 箱 | 结果 | 周期 | 说明 |
|---|---|---|---|
| 0 | ✅ COMMIT | 188.7 s | 全链：compute→select→B3 verify_selected（admission 生效）→pick→carry→descend→release→verify→v2 带锚提交 |
| 1 | ❌ PLACE_VERIFY_XY | 168.8 s | 全链执行至最终几何 verify：XY 误差 **0.0428 m > 0.04 m 门**（z 0.3 mm、settle 漂移 1 µm） |

预登记判据要求两箱均过 verify——箱 1 超差 2.8 mm，判定 **FAIL**。证据
（ledger、双箱 dump、launch.log、sdf、capture 目录）完整保留；不改规则、
不重跑（重跑须新的明示授权）。

## 归因初判（待离线深挖，不属本判定书）

- 链路/合同层零缺陷信号：B1 提交带锚、B2 单源门与身份栅栏、B3 admission
  与下一箱等待确认门全部按设计工作（r10 两箱日志可证）。
- 箱 1 的 4.28 cm 落位散布属物理/控制层（真空释放弹性、跟随控制），与
  G1 时间账、QP carry 战役的已知余量问题同族。4 cm 门对 inch20 箱是紧
  门——是否放宽属产品决策，不在本campaign权限内。

## 结论

B1→B4 开发完成：B1 事务+快照（含三审幂等合同）、B2 消费者单源、B3 执行
失效接入、B4 Gate-1/Gate-2 全绿、Gate-3 已执行（FAIL, 2.8 mm 物理边缘）。
Plan-B 一致性链在真实仿真中端到端成立（两箱全链、无合同类失败）；n=2
功能见证按规格未通过，稳定性与落位精度优化属后续派单。
