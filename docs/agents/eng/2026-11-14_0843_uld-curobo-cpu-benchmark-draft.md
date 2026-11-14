# 2026-11-14 -- 重建并发布 ULD cuRobo CPU benchmark 草案

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

用户确认旧计划不可得，授权重建并发布新草案。新增独立计划，所有新增矩阵、30秒每请求
解释、P95/P99门槛和统计规则明确为本次拟定；benchmark dispatch_ready=no。
本次仅文档发布与研究单测，不运行性能/GPU/sim，不声明速度或物理成功。

## Changed

- 从开发 d4c88854384d1c849635636e2fc47d2e15e1b92d 新建
  research/uld-curobo-cpu-benchmark-g1，独立 worktree
  /tmp/elfin_uld_curobo_cpu_benchmark_g1。
- fetch 后 origin/master4f7f7b20 是本地 master63dba44c 的祖先；rebase 后
  b77091ad45323edd1d18824813155d76f8aebdc1 保留8个开发补丁，range-diff全部等价。
- 原开发 worktree 的 RESULTS.md 未提交删除保持原状。
- docs/plans/ULD_CUROBO_CPU_BENCHMARK_G1.md 新建草案；本次只精确添加该文件和本记录。
  跨 agent 生命周期在主工作区，分支仅包含此角色记录快照，不复制 mailbox。

## Verification

- source /opt/ros/humble/setup.bash 后运行
  ELFIN_E2E_ROOT=/home/adamliao/work/elfin_e2e /tmp/uld-upper-python/bin/python
  -m unittest discover -s research/uld_upper_layer/tests：296 tests，0 failure/error/skip，25.076s。
  测试 revision 为干净 b77091ad45323edd1d18824813155d76f8aebdc1；随后仅文档变化。
- git range-diff d6e2a276..d4c88854 master..b77091ad：8个等价补丁；新分支含本地和远端master。
- git diff --check、精确 staged 文件清单和最终远端等值核对见生命周期 Result 与证据 REPORT.md。
- scripts/check_agent_contract.sh 在主工作区发现193项已有问题；完整逐文件/缺字段清单为
  agent_contract_primary.log。未修改无关角色记录，不声明全仓 contract 合规。
- satellite 本身没有复制主工作区被忽略的协调树，contract 检查报告8个既有缺文件；
  完整文件名见 agent_contract_branch.log，不为通过检查复制第二套 mailbox。
- 性能、GPU runtime/资产资格、物理执行均 not_evaluated。草案的未来 harness 尚需独立实现。

## Pointers

- docs/plans/ULD_CUROBO_CPU_BENCHMARK_G1.md
- docs/agents/discuss/2026-11-14_0843_uld-curobo-cpu-benchmark-publication-g2.md
- docs/status/evidence/uld_upper_layer_v1/curobo_cpu_benchmark_publication_g1/manifest.json
- docs/status/evidence/uld_upper_layer_v1/curobo_cpu_benchmark_publication_g1/research_unit_tests.log
- docs/status/evidence/uld_upper_layer_v1/curobo_cpu_benchmark_publication_g1/agent_contract_primary.log
- docs/status/evidence/uld_upper_layer_v1/curobo_cpu_benchmark_publication_g1/REPORT.md
