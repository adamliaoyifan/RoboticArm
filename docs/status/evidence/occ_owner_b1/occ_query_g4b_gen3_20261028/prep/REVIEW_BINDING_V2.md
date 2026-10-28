# G4-B manifest v2独立复核与Binding

- date: 2026-10-28
- reviewer: reviews/codex/gpt-5
- verdict: mechanical_admission_pass
- scope: 产品/install/IDL绑定及准备阶段开放，不是B验收PASS

## 独立复算

未调用owner的install_manifest.py判官。用pathlib遍历全目录、拒绝symlink、按相对POSIX路径bytes排序，逐文件重读内容计算hash和size，与files_candidate.jsonl/files_baseline.jsonl逐记录相等，再以 `<sha256>  ./<path>\n` 重算汇总。

- candidate 2748文件，a1180acb4fc2b1f56ec014524fca839da6aa1f13bdd74a72b40dcca299fcdcbe。
- baseline 2431文件，a6f2dc37908a48dfd40971a8f37bd6d52ca92817e33f6f0c4f79def443b97114。
- IDL实际文件重读：182/e9fd1867ba377cda92cbc44dfde0a8e3ddaf9fd472c593ebcfe2feb5c446916a，对161/bb73e2205a8f5286b8dde3f633fd9ef9cd4c6355e1d2ffc0e9e140c6a417791a。集合差新增21、删除0，共享161的内容hash全等。
- 两树HEAD分别0da4086bce23be49ec8eeb934a15fb6829803abe、b33dda919521ef043fa9e30b07dcc79c87f76fce；git status --porcelain均为空。

v1摘要作废结论保留；v2关闭安装清单缺口，不再追查旧口径。

## 重放核验与证据限制

读取真实replay_fault_check/replay_out/manifest.json：worker exit1、reproduced=true、exact_map_and_points=true；healthy同类manifest：exit0、reproduced=false、source_hashes_unchanged=true。为受控oracle断言故障，不宣称复现了产品bug。

tamper目录replay_out/manifest.json实际是复制的fault旧结果（argv指向fault目录），不作为本次拒绝证据。reviews另以新输出目录独立执行：

```bash
cd /tmp/elfin-occ-boundary-repair-g4a-g3
python3 scripts/g3/occ_query_eval.py replay --bundle /home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_owner_b1/occ_query_g4b_gen3_20261028/prep/replay_tamper_check/stage0/candidate/distance_failure --out /home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_owner_b1/occ_query_g4b_gen3_20261028/prep/review_v2_tamper_guard
```

退出1，调用栈 replay→replay_distance_case，occ_query_eval.py:240 `RuntimeError: frozen source changed`，在启动worker前拒绝。由此关闭篡改负对照，不要求重跑其他案例；旧复制manifest保留但不能引用为新结果。

## 准入结论

绑定产品/install/IDL并允许B领取准备阶段；脚本SHA、输入矩阵/命令、C1/C2行为与资源仪表验证仍须评分前冻结。不得将准备开放解释为已具备计分条件，不重开A。计划见OCC_G4B_ACCEPTANCE_GEN3_20261028.md §8，线程Binding为权威交接。
