# Gate-3 n=2 功能见证 · 预登记（执行前落盘）

- **登记时间**: 2026-10-16（用户当轮明示授权"现在授权执行"）
- **代码冻结**: HEAD `4bee002351a52283ccb5483bac49a8859cf8338b`
  （分支 agent/eng/master-pack-baseline-g3；B1→B3+B4Gate2 全部提交之后，
  唯一追加为本登记所需的 --frame-runtime 运行开关）
- **安装**: 同工作树 colcon 全量重建（6 包 0 失败，/tmp/freeze_build.log）
- **运行命令**: scripts/pack_to_full_run.sh --out <本目录>/run01
  --max-boxes 2 --sequence-ids carryon,carryon
- **链路**: v2 typed（--frame-runtime）；B3 旗标生产默认开
  （--verify-selected-route、--await-map-confirmed）
- **通过判据（全部满足才算链路通过；只证一次流程，不签稳定性）**:
  1. 两箱 ledger committed=true，fail_code 空；
  2. 每箱含放置、退出（retreat/unloaded 段执行）与 verify 通过；
  3. 箱 1 提交后地图确认（occupancy_confirmed）到达后箱 2 才开始规划
     （B3 await-map-confirmed 门，get_stats/日志可证）；
  4. 箱 2 交接（spawn→detect→compute→select→commit）完整；
  5. v2 commit 带 verify 锚（ledger verify_anchor 有 request_id/delta），
     admission 生效（validate_selected_route 调用记录）。
- **见证额度**: 1 次。失败保留完整证据束离线归因，不重跑、不临时改规则。
