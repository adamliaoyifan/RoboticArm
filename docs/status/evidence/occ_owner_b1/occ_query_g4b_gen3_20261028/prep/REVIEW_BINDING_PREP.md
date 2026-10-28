# G4-B Binding 准备复核

- date: 2026-10-28
- reviewer: reviews/codex/gpt-5
- conclusion: pending_manifest_reconciliation（非产品失败，不重跑性能）

## 已核实

candidate HEAD=0da4086bce23be49ec8eeb934a15fb6829803abe，baseline HEAD=b33dda919521ef043fa9e30b07dcc79c87f76fce，两工作树git status --porcelain均为空。candidate安装目录未发现symlink。IDL定义数量182/161与申报一致。

## 安装指纹不一致

在各自工作树根目录执行申报的原命令 `find install/g4b -type f | sort | xargs sha256sum | sha256sum`；计数用 `find install/g4b -type f | wc -l`：

| side | 申报数量/hash前缀 | 复核数量/完整hash |
|---|---|---|
| candidate | 2747 / 4c6f64692da6 | 2748 / f40f9b9aff3b61de5ea2a8933d471e8669b9d8e9fd23005e23b99d2f3dc35397 |
| baseline | 2430 / b777da4630b2 | 2431 / aa33efb111ab1e496b8272bd6c38e587268ab24052192c6ac272ccbc2eb807c1 |

同规则加NUL分隔的复算与上述hash一致。单独排除COLCON_IGNORE或.colcon_install_layout均未复现申报hash，不能据此断言某个文件就是原因。未修改安装内容，未重建、未跑ROS/性能。

IDL实际查询：`find install/g4b/luggage_msgs/share/luggage_msgs -type f \( -name '*.msg' -o -name '*.srv' \) | sort | xargs sha256sum | sha256sum`。本命令candidate=8f5cab1a0ffd2ff25a1ac2d2594a6ad581b9ff9a3963c7dfa5b23ae826a9db59，baseline=323945900492706119d81f3176b19ed1e7bb46e2cd9877c6fee204a26c2eac9e。IDL申报仅描述摘要规则，未定义path前缀/拼接字节格式；此差异本身不证明IDL变化，但需要实际可执行命令或脚本。

## 最小补交

不重建、不重复A门。保存逐文件相对路径/size/SHA256清单（证据放install目录外），明确根路径、是否含隐藏文件/pyc、symlink策略、locale排序、汇总字节规则和实际命令；保留原manifest，追加v2。若持有原逐文件清单，输出增删改差分；若没有，原摘要记不可追溯，直接冻结可独立复算的新清单，不猜测历史变化原因。IDL同样保存名字与内容hash；列出新增21项及共享项有无变化。

## 其他准备边界

typed-V2仅candidate验收可接受，但不能用baseline facade对candidate typed端到端时延求比率；共享语义oracle与candidate typed绝对门分开。输入manifest尚未展开、C1/C2及资源fixture尚未实现，均仍是评分前准备项。脚手架可独立提交scripts及其专用tests，产品src/st09_contracts/配置不得变；原“仅scripts”不得解释为禁止行为测试。

synthetic wiring exit0/reproduced=false只证明入口可达，不能签故障重放判别力。准备阶段还需受控失败fixture：未修件reproduced=true、健康对照false、源/输入损坏具名拒绝；禁止为此开产品性能或仿真运行。

协议批准及资源归属不重开。仅安装manifest机械项待核平，未追加Binding、dispatch_ready仍no。核平后允许领取准备阶段，脚本SHA/输入矩阵/行为与资源仪表验证冻结后才进入评分。
