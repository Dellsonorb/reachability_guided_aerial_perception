# A6 — Paper 1 formal experiment design

状态：**2026-09-08 用户已批准 pilot；formal matrix 未授权。**
本文件保留原设计提案。执行前的精确定义以
[A6 pilot protocol](../../A6_PILOT_PROTOCOL.md) 为准：3-window、停止、候选选择、
invalid/失败规则、simulation-time 效率和唯一主比较已明确；本次禁止额外开发补跑。
共同初始视点未通过 [RGB-D 几何前置检查](../../A6_PREFLIGHT_GEOMETRY.md)，
当前 0/14 attempts，尚未进入 ROS pilot 执行。

## 1. 冻结基线与研究问题

A5 已通过 [AGENT PR #5](https://github.com/Dellsonorb/reachability_guided_aerial_perception/pull/5)
合并 main。A1–A5 的共同冻结点为
`060a4e28c045614ad0814f4e41e126f491b55937`，包含已 review 的 A5 `d63b9f0`。
A6 从该 main 创建 `feature/a6-formal-experiments`，本轮只新增本文档。
“冻结”是研究开发边界，不新增 lock、hash 审计或执行框架。

全部方法共用：

- SIM main `2e7feaa7585425a13208741b10b4a4bd8e14fefa`，含已验证的 public map/localization/flight 修复。
- 冻结 RM4D `rm4d-aubo-baseline-v1 @ e9d431299053f38a4a4319aed3dfeccc261b9fac`。
- 已提交的独立 `assets/rm4d_ground_task_v1`，workspace z `[-0.25,1.3] m`、task floor `-0.472 m`；使用已验证 frame bridge。
- 原始 10M map、M-R1～M-R4、机器人模型、安装、IK/FK/collision gates 和 A1–A5 代码不变。

RM4D-only 也使用同一 task-domain asset；不能给它原 map 的错误任务覆盖而给 Ours 新 map。
这不是重新评测冻结 literature baseline 本身；论文中标为 “RM4D-only (shared task-domain asset)”。

预先指定的研究问题：

1. **RQ1**：Ours 是否提高 execution-validated Ground candidate discovery 和 E2E retrieval success？
2. **RQ2**：Ours 是否以更少实际 UAV 观测、距离、时间完成任务？首次获得 Ground evidence 的资源需求是否下降？
3. **RQ3**：Generic/Ours 的差异是否由 gain 的 manipulation weighting 引起，而不是不同候选、visibility、cost、预算或 updater？

Primary endpoint：**E2E retrieval success**。Primary contrast：**Ours − Generic NBV**。
Ours 与 RM4D-only / Fixed-view 的比较是系统级对照，不单独承担 weighting 的因果归因。

## 2. 采用的实验路线

采用 **配对自然 Gazebo E2E + 同状态双评分诊断**：前者提供真实任务结果，后者直接检查 weighting 是否为唯一评分差异。
每个独立 scene seed 对四个方法分别从相同初始配置重启运行；方法顺序随机且平衡。
不在一次已经改变的 belief/robot 状态上依次切换方法。

两种未采用的替代方案：

- 只做离线 shared-cloud replay：控制严格且便宜，但没有反事实视点的真实回波和实际 retrieval outcome，不能回答 primary endpoint。
- 加入 discovery-triggered early stop、更多候选或长 mission：可能提高效率，但改变冻结 A5 的行为，本阶段不采用。

同状态诊断直接使用 A4 已有的 task/generic 双 ranking；它不执行第二条反事实轨迹，不能当成额外独立 trial。
配对/分块用于控制可控场景差异，顺序随机化处理运行时干扰；原则参考
[NIST randomized block designs](https://www.itl.nist.gov/div898/handbook/pri/section3/pri332.htm)。

## 3. 四个固定 baseline 的操作定义

全部方法有相同的自然 aerial RGB-D bootstrap、map-frame grasp 生成、RM4D query/validation、frame bridge、UAV return/landing 和 Ground manipulation backend。
不向任何方法提供 Gazebo target truth，不重写旧观测时间戳，不向四次 live run 注入同一份陈旧传感器消息。

| 方法 | MID360/A2 观测策略 | Ground handoff | 比较含义 |
|---|---|---|---|
| RM4D-only | 不用 MID360/A2 决策，无额外 scan dwell | 原 RM4D 返回顺序的 exact top-1；不做 A2 gate | 无 aerial environment assessment 的系统参考 |
| Fixed-view + RM4D | 固定初始 pose，连续获得 3 个独立 5 s 窗口；不做 NBV | 同 A5 的 catalog、representative/exact footprint gate、最高 relevance/first-tie 选择 | 等观测预算但不换视点 |
| Generic NBV + RM4D | A4 generic gain 选视点，其余与 Ours 相同 | 同 A5 | weighting 的主要对照 |
| Ours | 原冻结 A5：A1→A2→A3→A4 task gain | 原冻结 A5 | 待检验方法 |

**Fixed-view 不是 single-window strawman。** A2 默认 `free_observations=2`，一次窗口最多一票；
给 Fixed-view 仅一帧会使其在没有任何障碍时也无法形成 FREE。因此固定视点也给满 3 个窗口，
窗口具有不同真实时间戳，不能把一份 cloud 重放三次伪造 evidence。
其位置/yaw 不更新到 NBV，实际漂移和 dwell 仍按同一 acquisition 规则记录。

RM4D-only 的 aerial RGB-D view 仍计入 UAV 任务视点与距离/时间，不能因 MID360 windows=0 就宣称零 aerial 成本。
其 top-1 ranking 保留 frozen RM4D 的原语义；可能含 travel/final_score，但这些绝不进入 A1 relevance 或 Generic/Ours 的 observation gain。
RM4D-only 没有 A2 ground-support discovery 指标，记 **N/A**，不是零；所有方法都能比较后述 execution-validated discovery 和 E2E。

Generic / Fixed / Ours 的 exact catalog 是 A5 `candidate_catalog` 恢复的 **每个 A1 cell 的原 candidate winner**，
不是 cell center，也不是全部 evaluated valid candidates。相同输入下 catalog、候选顺序、margin gate、tie-breaking 完全相同。
每个 trial 仅选择并尝试 **一个** Ground candidate；不在某方法失败后追加 top-K fallback、重新规划 RM4D 或再次起飞。
六次 approach-branch 尝试等已有 backend 内部限制对所有方法相同，不能误计为六个 Ground candidates。

## 4. Generic / Ours 的严格公共设置

令当前 belief 的 `unknown_score` 为 \(u_N(x)=e^{-N(x)/\tau}\)，\(\tau=2\)，
\(\alpha=1-e^{-1/\tau}\)。同一个 grid、snapshot、current UAV pose、ordered candidate list 上：

\[
G_O(v)=\alpha\sum_{x\in D_{\rm support}} \mathcal V(v,x)u_N(x)M_{\rm operational}(x),\qquad
G_G(v)=\alpha\sum_x \mathcal V(v,x)u_N(x).
\]

\(D_{\rm support}\) 是 A3 有 nominal validated support 的环境域，含 operational weight 为零的 BLOCKED_ONLY。
Ours 的求和沿用 A4 的 no-support NaN mask；Generic 使用整个相同 A2 grid，
不额外裁剪 task ROI、不偷偷排除普通 unknown。两者都是 predicted observation gain / uncertainty-reduction surrogate，不是 calibrated EIG。

\[
C(v)=\|p_v-p_{\rm current}\|_2+0.25|\operatorname{wrap}(\psi_v-\psi_{\rm current})|,
\quad S_m(v)=G_m(v)-0.25C(v).
\]

| 公共项 | v1 提案：沿用冻结接口/配置 |
|---|---|
| Grid | map、0.10 m；围绕 aerial grasp 的 4×4 m grid；A1/A2/A3 origin/shape 一致 |
| Candidate generator | 当前实测 pose 的 XY offsets `{-2,0,2} m`；固定当前高度；8 yaw；current/stay first |
| 候选过滤 | 相同 operating bounds `x[-4,4], y[-3,3], z[0.5,3] m`；相同近距离 yaw-only 过滤与稳定顺序 |
| MID360 model | 原完整 mount transform；水平 360°、垂直 [-7°,52°]；相同 range/FOV |
| Occlusion | 已知 OCCUPIED 的 1.0 m assumed prisms；UNKNOWN 不遮挡；相同 3D intersection |
| A2 | endpoint evidence、occupied priority、ground z=0/tolerance=.02 m、obstacle min=.05 m、2 次 free votes |
| Acquisition | 5 s/window，一窗口一票；相同 stamped TF、低速/位置/yaw、20 s capture guard |
| Flight | 同一 public action；FLY_TO tolerance=.05 m、TAKEOFF 原 .15 m；相同返回/降落逻辑 |
| Budget | 3 个总 MID360 observation windows，含 initial 和 stay/rescan，不是 3 次额外飞行 |
| Ground/backend | max travel=3 m、navigation timeout=120 s；原 MoveIt/AG95/refine/lift 配置 |

所有姿态是 public map 下 BUNKER/UAV base_link 的原语义，不移动地面平面或安装高度。
以上是设计提案中的统一配置，不为各方法/难度分别调 lambda、height、range、window 或 robot threshold。
gain 都是 cell sum，不乘 cell area；不做 smoothing、按方法归一化、独立调参或统一量纲的额外补偿。

“共用候选”指 **同输入时完全相同的 generator、过滤与 ordered candidates**。
闭环分叉后 current pose/belief 不同，后续物理候选集合可以自然不同；不能为了强行相同而引入新 global lattice。
每个已访问 snapshot 都保存公共候选与两种得分，检查 \(S_O-S_G=G_O-G_G\)，
并记录两个 argmax 是否不同、所选视点的 predicted/realized task-support observation。
这不是要求不同轨迹得到相同真实回波。

独立 live runs 的初始 RGB-D 数值及 RM4D catalog 仍可能因传感器/物理时序而不同。
使用相同 scene/可用 seed/initial pose，记录 grasp 差异、evaluated 数量和 catalog；不事后删掉不匹配的 trial。
因此 live E2E 的结论是配对随机化下的系统效果，严格 gain-only 机制结论来自同状态双评分，不能宣称逐消息反事实重演。

### 停止条件不能偷偷改变

Ours 直接保留 A5：无有效候选、无正 predicted task gain、best score≤0、或达到 observation budget 时停止。
不因首次发现 confirmed candidate 提前停止。只在停止后按原规则 handoff；无 confirmed candidate 则失败。

Generic 使用 **同一停止逻辑作用于自己的 generic gain/score**，其他 handoff 条件不变。
A4 的全局 `NO_PREDICTED_TASK_GAIN` 是 task-specific：不能错误地把它当成 Generic 的零增益，
应检查有效候选的 generic gain 和 `generic_order`。实现仅在未来 A6 薄 policy adapter 中选用已有结果，不修改 A4/A5。
Fixed-view 完成 3 个窗口后 handoff；RM4D-only 在 query 后 handoff；共同 fatal interface failures 立即终止。

stay/rescan 的 cost=0 且 FREE 的 unknown_score 可为正，Ours/Generic 都可能始终用满 3 个窗口。
这时 **不能声称减少实际 view count**；可以如实报告首次 evidence discovery 更早或实际飞行距离更短。
不把 first-discovery counterfactual 当成已经发生的 early-stop 任务。
也不预设 Ours 在 Easy 上比无需主动观察的 RM4D-only 更省资源；重点比较 Generic/Ours 的成功率与实际资源共同表现。

## 5. 场景生成原则与 Easy / Moderate / Hard

v1 只覆盖静态水平地面、单个相同 brick、相同传感器和 robot dynamics。
真实障碍先固定为地面支撑的 1.0 m 实体，避免同时改变 assumed-height mismatch。
没有悬空物、动态人群、新材质 dropout、强光变化、陡坡或新的 manipulation task。
这些不是为了让 Ours 必赢，而是先隔离已提出的 task-weighted observation 机制。

场景由少量可解释的 box/wall 布局加小扰动组成。所有方法运行同一个 scene specification。
障碍放在任务地面 footprint 与 initial sensor 之间，可遮挡观测，但至少保留一个合理的 exact base region 和 Ground 通路；
不得把所有可达 pose 都物理占满。occluder 避开最终 arm sweep、UAV 起降区和飞行高度带，
不通过引入一个所有方法都未建模的新 3D arm obstacle 扩大 A6 问题。
物理碰撞/控制失败仍照实计入，不把简易几何筛查称为完整可行性证明。

定义用于**场景描述/分档、不给 controller**的量：

- `high-support patch`：\(M_{\rm nominal}\ge .7\) 的 cells 与所有 nominal-clear、relevance≥.7 的 catalog exact footprints 的并集相交；
  使用这个完整集合，不手选有利小块。首先要求至少一个完整 nominal-clear exact footprint，不以其实际是否被某策略发现来选场景。
- `o_task`：上述 nominal-clear footprints 的 ground cells 中，满足初始 range/FOV 但被真实实体遮挡的比例；
  3D 场景几何仅用于离线标注，不能用 A4 自己的 optimistic V 给自己当 truth。
- `a_irrel`：第一次真实窗口后满足 `N=0 AND (nominal support≤.1 OR 位于当前 validated footprint union 外)` 的面积。
  后一种只叫 **candidate-relative unsupported area**，绝不把 A1 UNASSESSED 宣称为全局不可达。
- `a_task_unknown`：high-support patch 中 `N=0` 的面积。FREE/UNKNOWN state 受两票门槛影响，不能直接用初帧 UNKNOWN cell 数替代遮挡量。

| 难度 | 几何布局与建议描述范围 | 应检验的现象 |
|---|---|---|
| Easy | 无任务遮挡或 `o_task≤.10`；initial view 可覆盖至少一个完整 nominal-clear footprint；无需绕行才能看见 | RM4D-only / Fixed-view 有合理成功机会，Ours 不应仅靠 baseline 初始化缺陷获胜 |
| Moderate | 单侧短墙/box，约 `.25≤o_task≤.50`；一次侧向观察有机会补齐 footprint；普通 unknown 与 task unknown 并存 | active observation 是否改善 Ground support confirmation |
| Hard | 两段交错遮挡或带开口屏障，约 `.60≤o_task≤.85`；存在可绕看的 task support；同时 `a_irrel≥3 m²` 且 `a_irrel/max(a_task_unknown,.1 m²)≥2` | 有限预算下是否被大面积低 relevance/unsupported unknown 分散观察资源 |

上述数值是 **pilot 几何可实现性目标**，不是已生成的场景结果或方法通过门槛。
所有难度保持相同 grid 面积；不通过单独放大 Generic 的地图制造 Hard。
Hard 的 distractor 区需要落在候选的 range/FOV 内，且朝它观察与朝 task patch 观察在几何方向/遮挡上有差别；
如果一个视点轻易同时覆盖两者，它不是有判别力的 Hard 布局，但不能仅因 Generic 与 Ours 最后选择相同就剔除 trial。
场景资格由预先写好的几何规则判定，不按 Ours 成功、Generic 失败或某个得分差筛 seed。
难度标签由 pilot 校准后的模板与任务运行前几何规则锁定；first-window 面积等实测量作为操控有效性描述。
formal trial 的感知噪声导致实测量偏出提案范围时，照实报告，不能根据某方法的结果事后调档或剔除。

初始 aerial pose 提案为固定 map `(-2.5,0,1.5), yaw=0`，target nominal `(2,0)`。
这是各方法/难度共同的 setup 候选，不是运行时由 truth 或 relevance 自适应生成的 view。
相比 A5 示例 `(-.5,0,1.5)`，更远的观测距离旨在让倾斜 MID360 同时看到可用的 ground footprint，
避免 Fixed-view 因垂直 FOV 而先天失败；**尚未验证该距离的 RGB-D bootstrap**，须先在 pilot Easy 检查。
若无法同时满足 RGB-D bootstrap 与基本 ground observation，不启动 matrix；统一调整 scene setup 后重新送审，
不能修改 A2 两票规则或把 Fixed-view 静默替换成另一算法。

### 随机化变量与不变量

| 变量 | pilot 后冻结的建议采样域/控制方式 |
|---|---|
| target XY | `(2,0)+Uniform([-.15,.15]²) m`；不改尺寸、地面支撑高度和质量 |
| target yaw | `Uniform(-30°,30°)`；保持同一 side-up grasp task |
| 遮挡实体 | 难度模板内位置扰动≤.15 m、朝向扰动≤10°；厚度 .10–.20 m、长度 .6–1.2 m；height固定1 m |
| 左右方向 | 成对镜像/方向平衡，避免总是“向左是 task、向右是 distractor” |
| UAV initial target pose | 所有方法共享；pilot 通过后固定，不按方法或成功结果调整 |
| BUNKER initial parking | `(3,-2.5)` 小幅≤.10 m 扰动，yaw `π±5°`；保持不侵入候选 footprint union |
| acquisition timing / planner stochasticity | 能显式设置的 SIM/solver seed 分别记录；不能设置的 scan phase/OS scheduling 记录为残余随机性，不承诺逐 bit 确定性 |
| 运行顺序 | 每 scene block 四方法随机排列；跨 blocks 平衡位置，单机不并发跑 Gazebo |

模板/扰动先通过 workspace coverage、起降区、粗几何通路、传感器 bootstrap 的方法无关检查。
不要为新 seeds 重建 RM4D asset、增加 validation budget 或静默剔除 query 无解的任务。
固定模板间的差异是难度因子，seed 扰动是重复单位；不得把 cell、packet、多个 yaw 或同一 seed 的重复运行当成独立场景样本。
pilot seeds 与 formal seeds 分离；根据 pilot 修改的规则须在 formal 前定稿。

## 6. 指标、时间和分母

### 6.1 Primary：E2E retrieval success

每个正常启动的 task attempt 产生 \(S\in\{0,1\}\)。成功必须经历真实观测与规划执行，并满足：

- 实际 Ground candidate handoff 与 BUNKER 到达/停止；新鲜 D435 refine；
- 原 collision-aware MoveIt approach/descend 和真实 arm/gripper controller 完成；
- `/ground/gripper/grasp_confirmed` 为真，按原 `retention_hold=1 s` 保持；
- 实际 brick 相对 settled 初始高度提升≥.10 m，TCP lift≥.10 m；
- 在共同任务 deadline 内完成，无中途 teleport、ground-truth grasp 替换或人为放宽 gate。

复用现有 physical checker 的物理条件。Gazebo model state 只供外部 outcome 测量，绝不供策略决策。
recorded `LIFT` 不能独立替代实际目标升高；A5 的 `physical_summary.status=PASS` 是参考已验证结果，不是未来 trial 的默认值。
建议共同总任务 wall deadline=1200 s；保留原各阶段 timeout，超时不追加方法特有预算。

\(\widehat p_m=\sum_i S_{im}/n_m\)。分层给出 success numerator/denominator，另给 Easy/Moderate/Hard 等权总体。
同时记录 task physical outcome 与 teardown/checker transport 状态，不能把测量失败谎报为 grasp failure。

### 6.2 必须分开的三层 candidate 指标

1. **RM4D validated support**：inverse/evaluated/valid 数、A1 coverage/status、catalog size；不是全局 capability map。
2. **Evidence-supported discovery**：\(\mathcal C_{env}^{(r)}\) 是 A5 原 `confirmed=True` 集合：
   original exact pose footprint 非空、不越界、所有 covered cells 为 FREE，且其 A3 representative 未 blocked。
   记录 first round、first timestamp、数量及 exact IDs；不称为 navigation/manipulation 可行性证明。
3. **Execution-validated discovery（跨四方法共同的 discovery endpoint）**：所选 candidate 已由真实 BUNKER 到达，
   D435 成功 refine，refined exact grasp 通过原 collision-aware IK/reverse/current-state/forward continuation checks，
   并实际到达验证过的 pregrasp。记 `D_exec=1`，否则=0；闭爪、持物与 lift 仍由 primary endpoint 单独决定。

第三层只验证本 trial 实际尝试的一个 candidate；不声称已知整个 catalog 有多少“真可执行” pose。
无需对每个 hypothetical candidate 重启 SIM 或新增 oracle validator。
“执行后确认可行”与“观察时首次获得证据”的时间不同，分别记录 `t_exec_ready` 与 `t_first_env`。
RM4D-only 的第二层为 N/A，第三层照常计量；这避免把“不使用 A2”人为算成 discovery failure。

### 6.3 资源效率

| 指标 | 定义 |
|---|---|
| `N_obs` | 成功完成并输入 A2 的独立 MID360 窗口数；initial/rescan 都算；另记 capture attempts/failed windows |
| `N_view_visits` | aerial bootstrap visit + 后续每次 sensing action（stay/rescan 也算）；首个 lidar window 与 bootstrap 同 visit 不重复计 |
| `N_nbv_moves` | 实际非原地 NBV command 数；不混入初始出航、return/landing |
| `N_first_env` | 首次非空 \(\mathcal C_{env}\) 的窗口号；未达到记 missing-with-failure，不填0 |
| `D_uav_active` | 从初始 sensing pose 到 active loop 结束的公共 map pose 轨迹累积3D弧长，包括 drift；不是 waypoint 直线距离之和 |
| `D_uav_total` | task start 到 landed 的全部 UAV 路径，含出航、active、return/landing；yaw 单独累计 |
| `T_active` | 第一 sensing window 开始到 active decision 停止；同时给 simulation time 和 monotonic wall time |
| `T_first_env` | 同一起点到首次 evidence-supported discovery；只是诊断，不假装系统在此结束 |
| `T_task` | 公共 runtime ready 后 task start 到 physical lift/terminal failure；包含 bootstrap、计算、飞行、Ground/refine/grasp |
| Ground / arm | Ground实际XY路径、导航时间/到达误差；refine耗时/age/结果；approach、descend、close、retention、lift 各阶段结果/耗时 |

轨迹建议按公共 TF/odometry 的 10 Hz 同一采样规则记录，不做方法特有 smoothing；记录缺测率，不把缺测段当零距离。
评分的 \(C(v)\) 只是 proxy，必须与实测轨迹距离及 wall/sim time 分开。
setup/build/SIM启动时间不计入 task time；计算等待、失败的 acquisition、动作超时计入，不因方法失败而清零。
RM4D-only 的 `T_active/D_uav_active/N_obs` 为零，因为没有 MID360 active loop；其实际 bootstrap/query 等待期间的飞行、漂移与时间仍计入 total 指标。

为避免“快速失败看起来高效”：

- 对所有有效 task attempts 报告 success-vs-resource budget 曲线 \(F_m(b)=n^{-1}\sum_i 1[S_{im}=1\land resource_{im}\le b]\)。
- 同时给各方法失败率、全部消耗分布；成功样本的 median/IQR 明确标为 conditional。
- Generic/Ours 可额外在 **共同成功的 paired blocks** 比较实际 distance/time，必须给 paired subset 大小，不替代全体 success 比较。
- 未 discovery 的 first-discovery 指标是“未达成”，不能当成耗时0；不把方法终止失败当独立删失来推断它以后会成功。

## 7. 失败分类与运行有效性

每 trial 保存最早终止的 `failure_stage`、具体 `reason`、最后完成阶段和已发生的资源消耗；不把未进入的下游阶段记为失败。

| 分类 | 例子与计数规则 |
|---|---|
| INITIAL_PERCEPTION | 无/过期 RGB-D observation、target ambiguity、grasp generation failure |
| RM4D_QUERY | 接口错误、timeout、NO_INVERSE_REACHABLE、无 validated candidate；保留原 reason，不把 UNASSESSED 当 INFEASIBLE |
| AERIAL_OBSERVATION | flight/settle/capture/TF age 错误、窗口超时；即使只发生在某方法多一次飞行时也计该方法失败 |
| NO_GROUND_HANDOFF | 无 confirmed candidate；细分 budget、nonpositive/no predicted gain、no valid viewpoint，以及 occupied/unknown/clipped footprint |
| GROUND_NAVIGATION | action abort/timeout、travel guard、到达/停止验证失败 |
| INITIAL_APPROACH | 前往 aerial-derived observation/pregrasp 的 IK/planning/execution 失败 |
| D435_REFINE | 无新鲜近场 target、target geometry 不合法或不可接受 |
| REFINED_APPROACH / DESCEND | refined IK、collision、Cartesian fraction、joint plan 或执行失败 |
| GRASP / RETENTION / LIFT | 未确认抓取、脱落、brick/TCP高度不足；分别记录 |
| MEASUREMENT_INVALID / PLATFORM_INVALID | 启动前服务缺失、非方法相关主机/仿真崩溃、checker确实丢失结局证据；不能据此推断任务成功 |

任务内的传感器等待、复杂视点导致的执行失败、CPU计算timeout都属于结果，不以“platform noise”名义任意排除。
仅有独立于方法选择的启动/测量故障可标 invalid；原记录保留，给出原因和数量。
若无法确认故障与方法选择无关，按 task failure 计，不能仅凭“Gazebo/runtime error”标签排除。
pilot 期间排查即可；formal 若出现 invalid，按预先一致规则重跑整个 paired block，最多一次，
不只补跑失败方法直到成功。未能补齐的 block 单列，并同时给把 invalid 按失败计的保守成功率，避免选择性删除。
不新增自动 retry/lifecycle 系统。

## 8. 三个 ablation，不增加其它轴

| Ablation | 唯一变化 | 明确保留 |
|---|---|---|
| w/o task weighting | \(G_O\to G_G\) | 等同已定义 Generic NBV；复用结果，不当成新的独立 baseline 再运行/计样本 |
| w/o occlusion reasoning | task gain 使用 A4 已给的 `range_fov`，不减去 segment/prism occlusion | 同 candidate validity、range/FOV、occupied-target排除、A3 occupied-blocked support、A2 updater、cost、budget；不把 occupied 改 FREE，不把height设0 |
| w/o flight cost | 通过现有 NBVConfig 设置 `flight_weight=0` | 原 weighting、candidate set、visibility、tie-break、observation与停止语义；飞行实际消耗仍完整记录 |

ablation 是 A6 中显式命名的外部选择/配置变体，不覆写冻结方法文件。
不混合多个 ablation，不 sweep 多个 H/tau/lambda，不添加 footprint、uncertainty、RL 等消融。
准确名称为 w/o flight-cost **penalty**：lambda=0 去掉评分罚项，原 equal-score 时的 cost tie-break 仍保留，不宣称完全 cost-blind。
Generic/Ours 的 gain 总量尺度也因 weighting 而变，公共 lambda 不等于“已单独识别纯空间选择性而排除了尺度效应”。
v1 主张限定为 **使用/不使用完整 task weighting 的效果**；三个消融不能被夸大为证明每一种可能机制均已隔离。

## 9. Formal 分析提案（不是执行授权）

建议预算基点：每档 **30 个独立 scene seeds**，四方法各一次，共90 paired blocks / 360 runs；
两个非重复 ablation 先只放 Hard 同30 seeds，再加60 runs。合计420只是设计预算，不是当前要运行的任务。
该数量不是由14-run pilot 推出的统计功效保证；正式样本量、资源预算和最小有意义效应须在 pilot 后的 formal review 一并批准，不能看显著性边跑边加。
建议事前关注的实际效应为 E2E **15 percentage points**，但不把达不到它的结果隐藏。

- 按档报告 success numerator/denominator 和 Wilson 95% interval；依据
  [NIST proportion confidence intervals](https://www.itl.nist.gov/div898/handbook/prc/section2/prc241.htm)，不用小样本下简单对称 Wald 区间。
- 主要报告 paired risk difference：每 block \(S_O-S_G\) 的均值；总体按三档等权。
- 推断以独立 scene block 为单位；重复运行同seed只衡量runtime variability，不能当新增独立scene。
- 主要 binary comparison 给 discordant counts `Ours-only success / Generic-only success` 和预先指定的双侧 exact McNemar 检验；
  即对不一致配对做 p=.5 的 exact binomial test。计算原理可用
  [R stats exact binomial documentation](https://stat.ethz.ch/R-manual/R-devel/library/stats/html/binom.test.html)核对，不引入R依赖。
  没有 discordant pair 时不宣称证明等价；报告数量和区间。
- 分档与两个其它 baseline/ablation 对照首先报告 effect/interval，v1 不根据大量次级p值选择性宣布显著。
  如后续要求多个 confirmatory hypotheses，先修订分析计划再开 formal，不能事后改主比较。
- 成功率/资源曲线优先于“只在成功样本上省时”的叙述。报告差、零差或反向差都接受；pilot不用于论文显著性结论。

不把预测 gain、unknown-score 下降、FREE cell 数或同状态 argmax 差异当成 E2E 提升的替代证据。

## 10. 最小 pilot：审查后才执行

目的仅是验证实验能公平执行、场景有几何区分度、指标可观测，不估计论文优势。

1. **离线 protocol check**：用已有保存的一个/少量 snapshot 检查同状态候选顺序、V、cost、updater一致；
   Generic 与 w/o weighting 完全等价；no-occlusion 不修改 belief/A3 blocking；no-cost 只去掉代价项。
   这是未来实现时的局部测试，不创建通用实验/审计框架。
2. **Easy 先行**：验证所提初始 view 的真实 RGB-D bootstrap 和固定视点多窗口 ground evidence。
   若基础 sensing 几何无法成立，停止剩余 pilot，不用一个结构性必败 Fixed-view 跑比较。
3. **三个场景各一 seed × 四方法 = 12 个自然 E2E attempts**，每次独立初始化，沿用3窗口上限。
4. **Hard 同一 seed** 加 w/o occlusion、w/o flight cost 各一次；w/o weighting 复用 Generic。上限合计 **14 次 E2E attempts**。
   不因 Ours 输而换seed重试；环境/接口修复后的重跑单独记为开发重跑，不混入14次pilot summary。

pilot 通过条件：

- 公共 geometry/frame、gate 和 baseline definitions 与本文一致，无 frozen method diff。
- Easy 确认存在正常的固定视点获取 ground evidence 的机会；不要求所有trial成功。
- Hard 确实同时含 task occlusion 与 candidate-relative irrelevant unknown，且不把全部 Ground pose/flight routes 封死。
- 能区分 `confirmed`、`D_exec`、physical success，资源和终止原因无缺失/混淆。
- 同状态 Generic/Ours 的数值差只来自gain；不存在不同预算、origin、候选过滤、scene hints、占据处理或fallback。
- 可记录选择相同/不同以及实际回波与预测机会的差异；**不要求 Ours 必须赢才算 protocol 合格**。

pilot 后提交每次结果、全部失败、配置差异、可实现性和正式样本量/时间建议，再等待 formal review。
若观测无法改善 footprint evidence、Hard 实際失去判别力、或当前结果反驳研究假设，明确报告并请求研究决策，
不自动更改 A1–A5、选择更有利 seeds 或扩大budget掩盖问题。

## 11. 最小后续实现边界与本轮交付

设计获准后，仅需要：一个少量静态场景描述文件、薄 baseline/ablation policy adapter、复用SIM启动与A5接口、
简单逐trial CSV/JSON与必要图表。记录 `scene_id, seed, method, config, stage, reason, outcomes, resources` 即可；
按需保留 clouds/TF 用于算法调试，不建设 artifact lock/provenance/lifecycle/verifier 或大型benchmark框架。
本轮不创建这些执行文件，不启动 Gazebo、不跑pilot、不生成formal matrix。

审查重点：四个baseline的操作定义；Fixed-view的等窗口预算；execution-validated discovery口径；
冻结停止规则下RQ2的可检验范围；Easy/Hard几何目标；14次pilot与后续formal预算。
**用户审查本文后才进入局部实现/pilot；formal执行仍需单独审查授权。**

### 本地定义依据

- [A4 定义与几何边界](../../A4_REACHABILITY_GUIDED_NBV.md)
- [A5 实际E2E与配置](../../A5_SIM_ACTIVE_PERCEPTION.md)
- [Task-domain asset 与 frame calibration](../../A5_TASK_DOMAIN_ASSET.md)
- [A5 catalog/decision](../../../src/sim_active_perception/core.py)
- [A4 task/generic scoring](../../../src/reachability_guided_nbv/core.py)
- [A4 visibility components](../../../src/reachability_guided_nbv/geometry.py)
- [A2 两票 FREE 与 unknown_score](../../../src/environment_belief/core.py)
