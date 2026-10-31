# OCC distance JIT — deployment pin (g8, Q-20261031-7)

reviews 裁定：numba **不 vendor**，先钉版本与部署环境；有/无 JIT 分别验证
（功能回退 ≠ 性能达标）。

## 依赖钉版（2026-10-31 验证环境）

```
pip install numba==0.68.0 llvmlite==0.50.0     # user-level，不入库
```

- Python 3.10.12 / Linux 6.8.0 x86_64 / glibc 2.35 / numpy 2.2.6
  （完整记录：`docs/status/evidence/occ_owner_b1/occ_query_opt_20261031/VERSIONS.json`）
- 机会主义加载：`luggage_perception.distance_kernels_jit` 导入失败即回退原
  Python 路径（模块惰性，不报错）；回退**只保证功能等价**，性能门按 JIT=1 判定。

## 环境开关

| 变量 | 语义 |
|---|---|
| `DISTANCE_JIT=0` | 导入时禁用 JIT 路径，走原 Python 遍历（功能/对照用） |
| `DISTANCE_JIT`（默认/1） | 有 numba 即用 JIT 内核 |

## 验证口径（冻结协议沿用）

- **功能门**（oracle golden、查询契约、回归抽样）：`DISTANCE_JIT=0` 与 `=1`
  双跑，均须全绿（golden 文件与路径无关，即双路径逐位相等的证明）。
- **性能门**（查询矩阵 12 组、C2 负载排演）：以 `=1` 判定；`=0` 抽查仅作披露
  （当前参考值：b0/b5 中位 18.8/20.1ms，vs JIT 8.5/9.1ms）。
