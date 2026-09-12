# Candidate-confirmation v1：在线开发验证

## 版本与范围

开发分支 `feature/dev-candidate-confirmation-v1`；全部在线任务使用
AGENT `a3c62c4e611d702b3ce3edb86be313e9e6282979`、SIM `a0ae8e3`。
运行后仅整理分析、文档，不改变这一运行版本。原 Ours、Generic、正式 212 次
结果和论文统计保留；新结果不并入原正式统计。

固定批次为 [confirmation_dev_v1.json](../configs/confirmation_dev_v1.json)，
公共执行配置为 [current_sim_task.json](../configs/current_sim_task.json)。
四个旧场景专用于开发：Easy003 / Moderate009 / Hard015 / Hard023；各运行
一次 Ours 和一次 confirmation，顺序在启动前确定。没有根据本批结果换场景。
共同组件、3 窗口预算、真实地面支持、12 mm 开发余量、完整机器人/负载碰撞
检查及物理抓取保持条件一致。每次从正常 P450 感知开始，站位由程序选择。

## 实现的目标是什么

对当前 operational-unblocked、未越界的原始 exact winner 集合 Q，保存每个
候选自己的 footprint cell 集合 S_q。真实支持计数为 H(x)，缺票数为
`d(x)=max(0,2-H(x))`。最多搜索剩余两次窗口：

```text
存在 q ∈ Q，使所有 x ∈ S_q 满足：
H(x) + hit(v1,x) + hit(v2,x) ≥ 2。
```

这保留了候选内部的 AND 条件和候选之间的 OR 条件，不把不同候选的覆盖拼接成
一个不存在的可确认站位。已阻断候选不会靠预测获得解除。

有限扫描模型提供两个名义 envelope：`w==1` 为全部采样相位覆盖，`w>0` 为至少
一个采样相位覆盖。后者逐 cell 的相位可能互不兼容，并不是可实现的联合相位或
校准概率。前者也只对固定姿态/几何/离散相位模型成立，不保证真实回波。

预先固定的排序为：全采样相位方案优先于乐观方案，然后窗口少优先、共同飞行
代价小优先、稳定 ID 破同分。只执行第一步，下一次用真实观测重规划。搜索是当前
视点目录内的局部两步子图，不是完整的未来视点树。

- `SUPPORT_BUDGET_IMPOSSIBLE`：当前可用 exact winners 都存在真实缺票数大于
  剩余窗口的 cell。仅是这些候选的支持预算结论，不是所有未来站位物理不可行。
- `NO_NOMINAL_PLAN`：真实票数允许，但受限名义搜索没找到方案；按预先约定交还
  原 Ours 决策，不冒充物理不可能。
- 预测不修改 evidence；confirmation 不等于 D_exec，更不等于 retrieval。
- 共享执行筛选仍在 handoff 前运行。worker 的建议视点可能被成功 handoff 取消；
  汇总分别保存 worker proposal 和实际 `A5_DECISION`。

代码入口：[confirmation.py](../src/a6_pilot/confirmation.py)。
独立方法名 `confirmation`，原 `ours` / `generic` 路由保留。

## 离线范围与局限

新代码只读重放旧 212 任务中的 541 个可用状态，覆盖全部 80 个历史 Ours
成功场景、14 个有观测状态的 Ours 失败场景（包括 5 个确认失败）；另外两个
Ours 任务在产生该类状态前失败，未补造快照。

80 个 Ours 成功场景首轮中，75 个找到全采样相位名义方案、4 个乐观方案、
1 个未找到方案；66 个首动作与同状态 Ours 不同，27 个与单步缺票对照不同。
这些数字只说明策略差异和代码覆盖，不能转化为新方法成功率。

数据：[replay_results/summary.json](../analysis/confirmation_development/replay_results/summary.json)。
Generic 保留为旧结果和新状态上的共用 ranking 参照，没有额外飞行 Generic。
单步缺票对照也仅记录同状态动作，不宣称完成了在线消融。

## 在线结果与解释

最终逐任务值由
[summarize_online.py](../analysis/confirmation_development/summarize_online.py)
从 entry、attempt、真实观测、decision、events、metrics 和 physical summary
提取，见 [tasks.csv](../analysis/confirmation_development/online_results/tasks.csv)
与 [summary.json](../analysis/confirmation_development/online_results/summary.json)。
不重新分类或替换原任务。

8/8 计划任务完成，8 次启动，全部 `VALID_TRIAL`，无基础设施替代、无人工控制
介入。四组配对的 runtime commits 与公共 task profile 均一致；采集期间没有
方法、参数、SIM 或执行接口修改。

| 场景 | 方法 | confirmed 数 | D_exec | retrieval | 窗口 / 换位 | 首个终止原因 |
|---|---|---:|---|---|---|---|
| Easy003 | Ours | 3 | 是 | 成功 | 2 / 1 | 提升保持完成 |
| Easy003 | confirmation | 2 | 是 | 成功 | 3 / 1 | 提升保持完成 |
| Moderate009 | Ours | 3 | 是 | 成功 | 2 / 1 | 提升保持完成 |
| Moderate009 | confirmation | 2 | 是 | 成功 | 2 / 1 | 提升保持完成 |
| Hard015 | Ours | 5 | 是 | 失败 | 3 / 1 | 下降实测终点碰撞检查拒绝 |
| Hard015 | confirmation | 3 | 否 | 失败 | 3 / 1 | 近场观察初态碰撞检查拒绝 |
| Hard023 | Ours | 0 | 否 | 失败 | 3 / 2 | 窗口用尽，缺真实地面支持 |
| Hard023 | confirmation | 0 | 否 | 失败 | 3 / 2 | 窗口用尽，缺真实地面支持 |

两者均 3/4 场景确认、2/4 retrieval；D_exec 为 Ours 3/4、新策略 2/4。
retrieval 配对结果为 both-success=2、both-failure=2、两类 discordance 均为 0。
这是四个有意选取的旧开发场景各一次运行，不作显著性检验，不把没有观察到差异
写成方法等效。没有观察到本批 Ours retrieval 成功而新策略失败，但 Easy
观测窗口退化，并且未改善 Hard 的实际结果。

| 场景 / 方法 | active sim s | Ground sim s | task sim s | policy wall s | observe worker wall s |
|---|---:|---:|---:|---:|---:|
| Easy003 / Ours | 63.336 | 109.866 | 231.083 | 29.013 | 35.578 |
| Easy003 / confirmation | 107.777 | 107.219 | 274.765 | 71.814 | 82.888 |
| Moderate009 / Ours | 69.227 | 98.306 | 222.902 | 42.229 | 49.685 |
| Moderate009 / confirmation | 77.989 | 91.969 | 246.268 | 49.603 | 57.555 |
| Hard015 / Ours | 94.125 | 108.795 | 258.637 | 42.362 | 52.144 |
| Hard015 / confirmation | 101.105 | 56.687 | 224.285 | 53.647 | 64.632 |
| Hard023 / Ours | 82.805 | — | 123.539 | 51.710 | 62.220 |
| Hard023 / confirmation | 81.134 | — | 118.902 | 53.655 | 64.664 |

两步 solver 在四次新策略任务中累计分别为 0.0364、0.0316、0.0283、0.0276
wall s；共用有限扫描预测和 worker 工作占主要计算时间。不同真实状态及系统
负载影响 worker 耗时，不能把表中整项差值归因于两步搜索。前 6 条 UAV active
距离不完整，保持 NULL；Hard023 的 Ours / 新策略完整记录为 5.732 / 5.785 m，
但两者均任务失败，不提出成功任务距离节省结论。

### Easy003：成功保留，但窗口变多

两者都完成导航、D435 精定位、D_exec、夹持、约 0.149 m 物理提升和保持。
Ours 2 窗口，新策略 3 窗口；主动阶段 63.336 → 107.777 sim s。
新策略首轮选择原地观察再换位的两步方案 `[0,81]`，下一轮重规划为 `[57]`，
不是逐项执行初始整条名义序列。真实最少缺支持 cell 为
Ours `105→0`、新策略 `106→3→0`。

新策略把全采样相位两窗方案排在乐观一窗方案前，是本版的保守选择，不是额外
窗口被真实票数必然要求。该例是观测效率的负面证据，不能因为最终成功而隐藏。
同一初始状态离线检查为 0 个全采样相位一窗方案、5 个乐观一窗方案；后者包括
原 Ours 的视点 41。它解释优先级为何选择多一窗，但不证明反事实飞行一定成功。

### Moderate009：两者成功，未显示两步预测收益

两者均 2 窗口完成物理 retrieval。新策略实际首轮选择的是一窗方案 `[81]`，
与其单步缺票对照一致，不支持把该次成功归功于两步 lookahead。
Ours / 新策略主动阶段分别为 69.227 / 77.989 sim s；真实最少缺支持 cell
均为 `105→0`。两次完整执行均通过。

### Hard015：两者确认，执行阶段分别失败

Ours 3 窗口得到 5 个 confirmed candidates，新策略 3 窗口得到 3 个。
两者实际选择的 sensing 动作均为一次换位后原地再观察（目录 ID `[82,0]`；
绝对姿态受每次真实反馈影响，并非完全相同轨迹）。两者选中 source584，
共同名义执行筛选通过。

- Ours 到达 D_exec，下降后实测关节终点在感知碰撞模型中被拒绝：
  `ground/left_finger_pad/perceived_pick_target`。未到夹持/提升。
- 新策略导航完成，进入 D435 观察前的整机初态检查拒绝
  `perceived_pick_target/ground/base_link`；未到 D_exec。
  实际停车相对目标误差约 58.9 mm、4.11°；感知目标在底盘系的 XY 从名义
  `[0.4283,-0.5038]` 变为到位后的 `[0.4753,-0.4141]`。

上述证据是实测状态与感知几何上的 collision-guard rejection，不是以 GT
证明了物理接触。名义筛选明确记录需到位复验，且不保证相机观察与 transit。
没有证据足以把这两次改判为基础设施无效；原样保留为有效执行失败，不重跑。
也不能将新策略更早失败导致的较短 Ground 时间写成效率优势。

历史 Ours 在该场景成功，而本批 Ours 自身也失败，说明单次历史成功不是稳定的
逐次成功保证。本批不能把两条不同的执行失败因果归结为 NBV 目标函数。

### Hard023：新策略名义可完成，真实预算仍不足

新策略首轮预测 source500 可由 `[81,81]` 在两窗内确认。真实第二轮后，
source500 缺 81 个支持 cell、但都已有一票；剩一窗在票数上仍可完成。
然而新姿态/目录的最佳乐观视点也只预测覆盖其中 78 个，故正确报告
`NO_NOMINAL_PLAN`，而非真实票数不可能。按原 Ours fallback 再换位。

第三轮后 source500 仅缺 cell696 的第二票，真实计数仍为 1，且未被阻断。
末轮实际所选视点对 cell696 的名义 opportunity 为 0；保持未确认是正确的。
不能补票，也不能把首轮名义方案当成实际完成。该例暴露了固定名义两步模型
对后续实际观测姿态/扫描覆盖变化的脆弱性，不是已满足条件却被表示阻断。
本批 Ours 同样在第三窗后剩 1 个真实缺支持 cell，未确认；最少缺支持数轨迹为
`101→81→1`，新策略为 `101→76→1`。中间少了五个缺支持 cell 没有转化为最终
确认或 retrieval，因此不算任务收益。

## 时间和测量口径

- 窗口包括初始窗口；换位由每个接受窗口的真实 packet pose 推导，使用已有
  0.2 m / 0.2 rad 描述性阈值。原地重扫不是换位。
- active 和任务时间用 simulation time；active 沿用公共指标，包含决策计算及
  handoff 前执行筛选，不是纯传感器曝光时间。
- Ground 时间从导航阶段开始到 LIFT/FAILED，不包括此前 UAV landing。
- policy / worker 用 wall time，分别计共用预测及完整 observe worker；不含
  进程启动。两步 solver 单独记录，不把它等同端到端决策开销。
- 不完整距离保持 NULL；不以观测距离下界主张节省。失败任务的更短时间不是
  成功条件相同的效率比较。

## 阶段判断

本版实现了可解释的 candidate-confirmation 目标、两步预算推理和在线重规划，
但不能仅凭这些机制称为更好的任务完成策略。Easy 的额外窗口、Moderate 无两步
增益、Hard015 的后续执行失败和 Hard023 的名义预测失配都必须保留。

下一步若继续研究，优先检验名义相位保守性与窗口代价的取舍、预测对实际姿态
变化的敏感性；先与简单单步候选缺票策略做有控制的比较。执行到位鲁棒性是共用
下游限制，应独立归因。不能据本批四个旧场景进行统计优势声明或替换论文正式
方法。本批后停止，不追加场景或启动矩阵。

**结论：保留为独立实验分支，不替换现有 Ours。** 目前证据不支持晋升为更好的
任务完成策略，也没有证明两步比单步缺票更有价值。

## 检查与资源

实现前后相关 123 项回归通过，覆盖新 predictor、原方法路由、有限扫描 worker、
A5 handoff、operational gating 和 exact support。独立代码审查另做了小随机
实例的穷举排序对照；结果审查核对阶段、时间和缺测，并修正了汇总中 worker
建议与实际 handoff 的标签混淆。此修正仅在离线分析，不影响在线程序或结果。

从首次启动到最后任务收尾为 3015.12 wall s（约 50.25 min），未触及 6 h 上限。
批次原始目录占用约 26.76 GiB，结束时数据盘空闲约 313.12 GiB；未用两次
基础设施预留启动，未删除历史数据。包括另存的离线 worker 数据后仍远低于
80 GiB 新增上限。原始 bag/观测留在本机，不把大体积原始数据误称为已推送 Git。

## 复现入口

```bash
PYTHONPATH=src OPENBLAS_NUM_THREADS=1 python3 analysis/confirmation_development/replay.py
python3 analysis/confirmation_development/summarize_online.py
# 在线命令示例，仅供后续复现；本批已执行，不自动重跑：
python3 scripts/run_retrieval.py run --scene-file configs/paper1_eval.json \
  --scene-id eval-easy-003 --method confirmation --output-dir <new-output-directory>
```

在线原始观测、日志、bag、物理检查结果保存在
`outputs/development/confirmation-v1/slot-*`；未删除或覆盖历史数据。
