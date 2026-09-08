# PF-R5 官方结果 — run8（2026-09-04 21:10）

- 运行环境：隔离干净 git worktree `/tmp/pfr5_clean` @ **c5921d5**（dirty=0）
- 口径：`docs/plans/platform_free_height_gate4_revision.md`（用户裁定）
- 启动参数：见 summary.json `launch_params`
- 判定：**全部门通过（gate4_pass=true, failures=[]）**

## 结果（30-trial，settled 254 帧）

| 门 | 限值 | 实测 | 判定 |
|---|---|---|---|
| top_surface_rate | ≥0.95 | **0.9606** | ✓（原门，未动用拆分豁免） |
| full3d_rate | ≥0.95 | 0.959 | ✓ |
| top Z p95/max | 15/25 mm | 10.7/10.8 | ✓ |
| support Z p95/max | 15/25 mm | **0.0/0.0** | ✓ |
| height p95/max | 25/40 mm | 10.7/10.8 | ✓ |
| XY p95 | 30 mm | 10.4 | ✓ |
| width/depth p95 | 50 mm | 33.6/42.1 | ✓ |
| false measured height | 0 | 0 | ✓ |
| coverage | 3 尺寸×10 | 3×10 / 30XY / 22yaw | ✓ |
| stale instance / spawn fail | 0 | 0/0 | ✓ |
| active Hz（诊断，G6 另行） | — | 3.91 | — |

revision 证据：`summary.json` 内 `git_commit=c5921d5…, git_dirty_files=0`。

## 新口径下的定位

- **PF-R5-GEO（几何验收，阻塞门）**：通过（上表全部几何/契约门）。
- **PF-R5-SIM-DETECTION（检测诊断，非阻塞）**：flat STL recall 0.9606；
  失败分布：1 trial（10 帧）整 trial 漏检，根因与 run5/6/7 相同
  （YOLO-World 对无纹理几何的召回上限 + base_link 误检填位，图像证据在
  `2026-09-04_2035_fail_hunt/`）。不得以此宣称实机检测可靠性。
- **SIM 三箱闭环门 / 代表性检测硬门**：另行验收（前者需去 GT fallback
  的闭环驱动；后者需 textured simulation（world 预置纹理模型，Fortress
  禁止运行时导入纹理网格，实测相机渲染线程永久死亡）或实机 rosbag）。

## 与历史 run 的关系

run1-7 为诊断运行（revision 证据不合格或含未提交修复）；run8 是首个
干净 revision 上的正式通过结果。run7 的 0.886 含主工作区脏状态因素。
