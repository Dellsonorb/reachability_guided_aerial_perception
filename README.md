# Reachability-guided aerial perception and Ground retrieval

P450 使用正常 RGB-D / MID360 感知发现目标并观察环境；AGENT 用 manipulation
relevance 引导后续观测，选择 BUNKER 的 exact validated 站位；SIM 通过导航、
D435 精定位、AUBO i5 / AG95 的碰撞感知规划与物理夹持完成提升和保持。

## 当前启动入口

```bash
# 查看实际共用配置，不启动机器人
python3 scripts/run_retrieval.py run --dry-run --output-dir outputs/tasks/natural-001

# 一次自然任务（默认 Ours；--method generic 使用共用执行层）
python3 scripts/run_retrieval.py run --output-dir outputs/tasks/natural-001

# 另一个终端查看当前状态／原始失败
python3 scripts/run_retrieval.py status outputs/tasks/natural-001
```

[完整启动说明、依赖、正常任务输入和状态解释](docs/CURRENT_SIM_TASK.md)。
当前入口使用既有本机 SIM / PX4 / RM4D 环境；不是“一条命令安装所有依赖”。
仅适用于仿真，不能直接发送到实机。

## 已做到哪里

[最新固定版本六场景配对评估](docs/FIXED_VERSION_PAIRED_RESULTS.md)：12 次计划任务
全部完成，无额外启动或运行中改参；Ours 6/6、Generic 5/6。唯一失败是
Hard02/Generic 三窗后仍缺真实 ground support；其余11次均由程序自主选站，完成
导航、精定位、物理抓取、提升与短时保持。一个 discordant pair 和一组少用一窗
只是值得独立验证的开发信号，不是统计优势。成功任务的距离缺测保留；共用执行层
未再表现为系统性阻断。建议转入正式实验方案设计，不继续主动增加功能或恢复旧矩阵。

[此前六场景 Generic/Ours 配对开发结果](docs/DEV_MULTISCENE_PAIRED_RESULTS.md)：
12 次自然任务中，8 次 confirmed，8 次完成实际 Ground 抓取/提升/保持，7 次原独立
retrieval 检查通过；4 次 Hard 缺真实地面支持，1 次检查器缺早期观测。历史原结果保留。

当前开发在此基础上修复检查器观测保留，提供统一任务入口，并增加共用
`screened_candidate` 停止规则：真实观测确认 + 完整机器人操作预检通过后交接，
无需仅因尚有未知区域就耗完三窗。到位后仍用实际姿态和新近场感知重新规划；
预检不等于实际 `D_exec`。[此前四次启动](docs/TASK_HANDOFF_DEVELOPMENT_RESULTS.md)
的两次成功、两次运行时失败仍保留。

[最新有限扫描开发批次](docs/FINITE_SCAN_DEVELOPMENT_RESULTS.md)已完成六次启动：
基线自然任务成功；新版 Hard Ours、Moderate Generic/Ours 均完成真实 retrieval；
Hard Generic 的越界采集失败保留。共用到位/HOVER 就绪缺陷修复后，最新版
自然完整任务再次成功，程序自主选站，物体实际升高149.0 mm并短时保持。
Moderate 配对分别用2窗（Ours）/3窗（Generic），但不据此宣称统计优势。

[任务交接设计](docs/superpowers/specs/2026-09-10-task-handoff-design.md)；
[Hard 理想观测机会与真实支持诊断](outputs/development/task-handoff-analysis/REPORT.md)。
[进一步扫描/回波诊断](outputs/development/task-handoff-analysis/SCAN-PATTERN-REPORT.md)
确认 nominal FOV 内的 cell 不一定在有限扫描窗口中获得真实 ground endpoint。
现已接入[有限扫描机会模型](docs/superpowers/specs/2026-09-10-finite-scan-design.md)：
读取真实传感器方向程序，逐射线地面投影与局部遮挡，共用1 m视点格点。
六个新版实际后续观测窗口的支持区域漏预测69→43 cell-window，误报4→4；
漂移和保守遮挡仍产生误差，预测不代替任何真实回波或支持票。
尚未证明 Ours 的统计优势，未启动新的正式矩阵。

## 代码边界

- `src/reachability_guided_aerial_perception`：A1 joint-margin manipulation field。
- `src/environment_belief`：A2 原始 Free / Occupied / Unknown 与 observation deficit。
- `src/operational_gating`、`src/task_relevant_uncertainty`：真实 endpoint/物体几何、
  exact footprint 支持和 task uncertainty；不把 UNKNOWN 当 FREE。
- `src/reachability_guided_nbv`：共用候选、visibility、cost 与两种 gain。
- `src/sim_active_perception`、`scripts/run_a5_sim.py`：数值/ROS 边界和任务交接。
- `scripts/run_retrieval.py`：当前单任务入口；复用既有 runner，不自动批量/重试。
- SIM 独立仓库：公共 TF、飞行、导航、相机、MoveIt、夹爪和物理执行。
- 原 RM4D baseline 不改；Ground 使用[独立负z任务域 asset](docs/A5_TASK_DOMAIN_ASSET.md)。

Generic/Ours 公平共享感知、候选、可见性、成本、预算、操作筛选和物理成功条件。
Gazebo GT 只用于仿真场景初始化和外部结果测量，从不作为算法感知输入。

## 范围与历史

当前是已知砖块类型、初始相机搜索区域、静态水平地面和有界作业区内的开发程序，
不是任意场景/物体搜索。LiDAR 模型是名义悬停下的有限扫描相位机会，不是校准
回波概率；旧配置保留理想模型。12mm 是开发净空余量，不是校准安全保证。未满足真实支持、规划或抓取
条件时报告失败，不补票、不瞬移、不伪造抓取。

历史[原A5](docs/A5_SIM_ACTIVE_PERCEPTION.md)、[v1.1](docs/OBJECT_AWARE_GATING_V11.md)、
[v1.2](docs/EXACT_POSE_SUPPORT_V12.md)、[Pilot-1](docs/A6_PILOT_RESULTS.md)和后续
开发记录保留。A1 原始定义/离线用法可在 `06c695d:README.md` 查看
（`git show 06c695d:README.md`）；那是阶段存档，不代表当前尚未实现 A2–A5。
原配置和 legacy 停止默认仍可复现旧版本；不恢复旧560次 formal matrix。
