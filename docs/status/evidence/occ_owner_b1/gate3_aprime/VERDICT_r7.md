# Gate-3 A′ · r7 计分运行判定（2026-10-19，冻结 HEAD 3f5daf97）

## 判定表（按用户裁定格式，不合成为总 PASS）

| 门 | 判定 | 依据 |
|---|---|---|
| 离线门（位姿流五类/冻结次序/五体解析/凹区/一致性/拒绝路径） | **PASS** | 64 测全绿（test_release_pose_stream 13 + recess/footprint 15 + suitcase_visual 等） |
| 非计分预检 | **PASS** | 2 次引导 ~10min（≤30min 预算）：boot 清扫归属、服务就绪、位姿流 57ms 新鲜、gz 接受五碰撞体 SDF、接触传感器在线（body_collision 实报）、teardown 零残留 |
| **r7 产品门（两箱）** | **FAIL** | 箱 0 于 transit 段中止（CARRY_FAULT_PAYLOAD_LOST），boxes_packed=0；运动已开始 → 本次额度已消耗 |
| 释放位姿修复门（新鲜度/一致性/首步） | **NOT_EVALUATED** | 未到达释放边界（中止在 carry 段） |
| 释放后工具接触门 | **NOT_EVALUATED** | 无动态体生成；接触流 0 字节——按合同记 not_evaluated，不按零接触通过 |
| 凹区在线验证 | **部分** | 五体 SDF 在计分运行中真实服役（pick＋41s carry 全程跟随正常）；释放初态检查未到达 |

## r7 失败机制（证据链）

段记录真空字典（segments.jsonl，transit）：

```
attached=true  vacuum_on=true  fail_reason="set_pose timeout"
cancel_status=SUCCEEDED  frac=1.000（轨迹本体完成）
```

- 30Hz 运动学跟随中**单次** set_pose 服务调用超过其 50ms 预算（vacuum_controller
  `_follow_tick` → `follow_step` → gz set_pose timeout_sec=0.05），该次失败被
  **锁存**进 `_last_fail_reason` 并随状态发布；
- 携带门 `_carrying_vacuum_holds` 把任何 fail_reason 判 PAYLOAD_LOST → 取消
  （轨迹已完成为时已晚）→ ABORT。
- **载荷从未丢失**：attached 恒真、follow_skipped=0、箱体全程跟随。这是
  跟随环的既有脆弱性（单 tick 瞬态无宽限、失败锁存不清除），由 r7 边际
  增高的负载（五碰撞体＋50Hz 五条目传感器＋捕获进程）首次踩中——非本轮
  改动的逻辑错误（释放路径未执行；凹区几何物理表现正常）。
- 失败合同行为正确：未删除实体、未动地图（中止在放置规划段，无 pending 需清）。

## 证据索引（每束可关联）

| 束 | 路径 |
|---|---|
| r7 计分运行 | `master_pack_recovery/2026-10-14_g3/gate3_aprime_r7/`（manifest head=3f5daf97、trials、dumps/box_00_PLACE_PLAN_transit、segments.jsonl 真空字典、launch/driver logs、rtf.jsonl） |
| 离线门 | `luggage_gazebo/test/test_release_pose_stream.py`、`luggage_description/test/test_contact_recess_collision.py`（含全足迹投影与准入包络） |
| 预检 | `occ_owner_b1/gate3_aprime/preflight_r7_192617/`（verdict.json PASS） |
| 几何统一 | `occ_owner_b1/gate3_aprime/unified_box_geometry.json` + UNIFIED_GEOMETRY.md |
| 历史判定 | r5 PASS（有效 n=2）、r6（机理证据）、VERDICT.md |

## 遗留项（待授权，本轮不动）

1. **跟随环宽限**（r7 根因）：单次 set_pose 超时不应立即判载荷丢失——连续
   N 次失败（如 3 tick ≈100ms）才置 fail_reason，或跟随成功即清除上次
   瞬态原因；超时预算 50ms 是否放宽属同一决定。改动小、可离线测。
2. 释放位姿流/五体接触/凹区释放初态三门在 r7 未被触达——随下一次计分
   运行一并评估（需新额度）。
3. 产品门状态：**r5 仍是唯一有效的 n=2 PASS**（判定书 VERDICT.md）；r7
   FAIL 记录在案。

## 收口

本轮交付：版本 3f5daf97（代码/安装/资产冻结于 manifest）；判定表如上；
证据索引如上。不开展 r8、机制归因、叠箱、容量实验。仿真已清理
（teardown 零残留 ign、SHM 已清）。
