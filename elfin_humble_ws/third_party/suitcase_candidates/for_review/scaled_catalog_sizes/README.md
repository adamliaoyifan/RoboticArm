# 按 catalog 三档缩放（CloudCompare 预览）

目录尺寸（米，W × D × H，Z 朝上）：

| 档位 | 对应 catalog | W | D | H |
|------|----------------|-----|-----|-----|
| small_carryon | carryon | 0.55 | 0.40 | 0.25 |
| mid_standard | standard | 0.70 | 0.45 | 0.28 |
| large | large | 0.80 | 0.50 | 0.32 |

两份网格都先躺平（最薄的轴当高度），再 **按轴分别缩放** 去贴合这三档。
不是等比缩放：轮子、拉杆插孔、把手会跟着被拉长/压扁。碰撞以后仍用 `<box>`。

Gazebo 里也可以不烤三种 mesh，只在 SDF 里写 `<scale>sx sy sz</scale>`。
这些文件只是给你在 CloudCompare 里看三档长得像不像。
