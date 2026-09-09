# 当前任务入口、提前交接与观测覆盖差距

2026-09-10。开发记录，不是正式效果实验。历史版本、十二次配对结果及所有失败保留。

## 距离项目目标还有什么

现有程序已经不止单场景演示：上一批六场景的十二次自然任务中，八次完成机器人抓取、提升和保持，七次通过原独立检查；四次 Hard 在真实支持不足时停止。本轮解决了入口分散、已具备执行条件仍继续观测、早期观测被检查器丢弃和一个可复现的 ROS 连接问题。但不能宣布多场景能力和运行可靠性已经完成：Hard 的观测预测仍过于理想化，采样时序补丁也还缺最新完整在线回归。

选择的路线是先复用已有执行能力、补齐实际任务交接和使用方式，再依据真实扫描诊断 Hard；没有凭四个失败重新设计评分或调整 collision gate。

## 四次在线启动，全部保留

上限四次，包括失败。预定 Moderate01 Generic/Ours、Hard02 Ours/Generic；第 2 次出现可复现公共 TF 连接阻塞后，在剩余启动前明确改为修复版本的 Moderate01 Generic/Ours。没有第五次、自动重试或替换历史结果。Hard 仅重放既有传感器记录。

| 启动 | 方法 | AGENT 运行版本 | 完成窗口 / NBV 换位 | confirmed | 实际机器人终点 | retrieval |
|---|---|---|---:|---:|---|---|
| 01 | Generic | `5a0a346` | 2 / 1 | 0→4 | 到位→精定位→D_exec→抓取→lift/保持 | 成功 |
| 02 | Ours | `5a0a346` | 0 / 0 | 未评估 | 起飞后、目标观测前 TF 不连通；正常降落 | 失败 |
| 03 | Generic | `846debc` | 0 / 0 | 未评估 | 目标感知/RM4D 完成，首窗开始前 current-TF 检查失败；降落 | 失败 |
| 04 | Ours | `846debc` | 3 / 2 | 0→0→6 | 到位→精定位→D_exec→抓取→lift/保持 | 成功 |

四次原 `attempt.json` 均为 `VALID_TRIAL`，未追改为 INVALID，也未删除失败。02/03 的工程归因是补充诊断，不改变原方法结果。每一对的运行版本及除方法外的 profile 均匹配，但各有一次前 NBV 运行时失败，**不能用其 discordance 推断 task weighting 的效果**。两次成功不能跨版本拼成新的效果比较。

原始紧凑记录：[四次任务](../outputs/development/task-handoff/)。
[逐阶段、资源与 same-state 数据](../outputs/development/task-handoff-analysis/task-results.json)；
[结果重建脚本](../outputs/development/task-handoff-analysis/summarize_task_runs.py)。

### 真正发生的机器人行为

01/04 都从正常 P450 RGB-D 感知开始，由本次 MID360 更新、程序自行选站；无人工站位、历史 confirmed pose 或控制介入。均自主选择 rank 1 / source582，但精确位置分别是：

- 01：`[2.403905041, -0.626560953, 2.967059728]`（map x/y/yaw）。
- 04：`[2.404017583, -0.619996610, 2.967059728]`。

两次均执行现有完整机器人预检、到位后真实姿态/新 D435 重新规划、实际夹持后负载建模和检查。目标实际升高分别 **148.787 / 148.674 mm**，TCP 升高 **149.438 / 149.462 mm**；原有短时保持通过，不是长时间负载可靠性保证。12 mm 开发净空余量、显式 SIM 速度反馈及原生诊断、真实接触和成功条件不变。

候选/IK 回退本轮未触发，不能宣称其在线可靠性得到验证。相机回退自然触发：01 当前视图、view0、view1 无有效精定位，view2 成功；04 当前视图失败后 view0 成功。没有将这些中间拒绝伪装成直接观察成功。

### 仿真时间与距离

| 成功任务 | 首次 confirmation（active 起点后 s） | active s | 起飞至降落 s | 导航 s | D435/refine s | 整任务 s |
|---|---:|---:|---:|---:|---:|---:|
| 01 Generic | 44.333 | 56.496 | 120.748 | 36.281 | 83.358 | 296.387 |
| 04 Ours | 50.847 | 63.587 | 125.726 | 32.479 | 14.118 | 215.455 |

active 包含其中的操作预检，不遗漏计算期间悬停时间。成功任务 UAV 完整路径因少量 TF 样本无效保持 `null`；active 可观测距离下界为 4.353 / 4.713 m，总 UAV 距离下界为 13.468 / 13.132 m。不能把下界当完整距离。02/03 的快速失败不是高效率成功，未参与这张成功任务时间表。

五份实际取得的 same-state ranking 中，Generic/Ours 的 top viewpoint 均相同；它们共用预测、候选和成本。本轮没有观察到由不同 gain weighting 引起的选择分歧，更不证明 Ours 优势。

## 实际修订与证据

1. **统一单任务入口**：`scripts/run_retrieval.py run/status`，共用 `configs/current_sim_task.json`，直接沿用既有 SIM runner、public flight、MoveIt 和感知。记录当前状态、首次失败及实际 commit；无自动矩阵或自动重试。明确依赖已安装工作站环境、已知砖块类型/初始搜索区域，不声称任意物体搜索或自动部署实机。
2. **共用 screened handoff**：真实支持已确认的 exact winner 通过现有完整操作预检后停止观测；预检失败且仍有合法后续观测则继续。预检不等于真实 D_exec，也不保证导航和相机可见性。01 在两窗后确实执行成功；不是只通过单元测试。旧 legacy 停止模式保留，Generic/Ours 共用新规则。
3. **SIM 检查器观测保留**（`a0ae8e3`）：只替换两个按消息数量截断的 deque；原 frame/stamp/age、顺序、接触和物理提升检查全部保留。01 的实际 bag 含 6,976 条 aerial pose，6,439 条已晚于早期 handoff 匹配范围。真实 callback 重放中，旧 5,000 容量丢失 handoff、新存储保留匹配。见 [实际重放](../outputs/development/task-handoff-analysis/checker-replay.json)。这不追改上一批失败，也不补造观察。
4. **单机 ROS 连接修复**（`846debc`）：02 的 localization 随机复用端口33701；遗留的无关 Gazebo PID1201814 仍向该端口的主机网络地址重连，发送 `ff00000000000000017f`。独立监听复现相同10字节及实际发送进程；ROS 将头部解释为255字节长度，串行握手因等待余下数据连续阻塞约30秒。当前单机 runner 显式配置 ROS IPv4 回环地址，定位节点实际绑定回环，冲突端口局部对照拒绝旧 LAN 路由、正常本机连接成功；03/04 均无该阻塞。未停止无关 Gazebo，未改 map 几何或系统 ROS。部分 C++/Gazebo 监听仍为 wildcard，不宣称通用网络隔离。
5. **实际 TF 就绪**：`846debc` 起飞前同时检查本任务 buffer 的新鲜 UAV/Ground TF 与 native flight health，不依赖 setup 节点自己的 ready；使用既有30s preflight allowance。03 之后又在 `460a810` 增加 current-pose 的既有 tf_timeout 内 wall-time 等待；负 age 和超过0.5s均不接受，timeout0及明确 timestamp 查询不变。03 周围 TF 持续健康、同刻轨迹出现小负 age；一毫秒 future-TF/callback skew 能重现旧即时失败，但原失败未记录 age，不能断言其精确原因已唯一证明。新错误包含实际 age。**460a810 仅在四次启动后做离线回归，未声称已在线 E2E 验证。**

## Hard：下一步为什么值得做

[完整扫描诊断](../outputs/development/task-handoff-analysis/SCAN-PATTERN-REPORT.md)
只使用原始 runtime endpoints、逐 packet 公共 TF、已有 MID360 扫描模式和感知 belief。
主 agent 重新计算并匹配 407 packets、698,036 个 retained returns、八份累计支持数组和七组/十五条残余射线追踪；无新投票、GT 几何或观测补造。

- Hard01 Ours window2 中，150 个预测可见 footprint-union cells 有18个没有 ground return；全部在记录扫描的 z=0 平面上没有射线落入该 cell。11个连允许高度带机会也没有；另外7个的15条 band-crossing rays 实际命中相邻 cell。closest source623 缺少的 cell419 正是此类，并非 mapper 丢失真实票。
- 另外三次 Hard window2 的缺失 cell 均有真实前景截获射线，当前预测也排除了它们。不能把所有缺口都归因于同一个采样问题。
- 三份 first-state 候选清单连理想两窗完成机会也没有；唯一有机会的 Hard01 Ours 已选中组成该理想组合的首视点，却未获得完整真实支持。更换 mass score 本身不是已证实的解法。

最值得继续的是**共用观测预测/候选生成与有限扫描内真实 footprint 支持的一致性**：区分视野、扫描方向、实际 endpoint 和重复 ground support。先解决有明确证据的预测偏差，再评估是否需要 completion-aware 决策。Generic/Ours 公平共享共用修订。当前证据不要求推翻 manipulation-relevance 引导感知的核心方向，也不支持进入正式效果实验、增加预算或修改成功条件。

## 版本、复现与剩余限制

AGENT 当前分支 `feature/dev-task-handoff`：`5a0a346` 实现入口/交接，`846debc` 单机连接/起飞 TF 修复，`460a810` 离线 current-TF 等待补丁。
SIM `feature/fix-checker-observation-retention @ a0ae8e3`，实际机器人执行仍为此前安装的 `fbb191b` 系列；checker直接使用该分支脚本。RM4D baseline `e9d431299...`、独立 task asset 和旧实验保留，不改原模型/map。

相关验证：173项 AGENT ROS/adapter/入口/指标测试、58项数值/core/支持回归、136项 SIM 完整机器人/执行/观察/检查器回归均通过；不是以这些数量代替在线结果。独立实现审查发现并修正了指标时段、入口中断清理和 preflight 的时序空隙；最终代码与 Hard 重放经过复核。

使用方法见 [CURRENT_SIM_TASK.md](CURRENT_SIM_TASK.md)。大 bag、原生动力学 CSV 和 ROS 日志仍保留本地，沿用现有忽略规则；紧凑结果与脚本进入 Git。只停止本轮拥有的运行进程，旧四个 A5 输出目录和无关 simulator 保留。开发分支保持 Draft，不合并 main，不启动正式矩阵。

尚缺：最后 TF 等待补丁的在线验证；更符合扫描能力的共用 acquisition model；更广泛且独立的场景验证。相机选择的时耗仍有明显波动，候选回退未在线触发，净空余量未校准为通用保证。当前是有多场景实际证据的开发系统，**尚不是完成全部目标或具备 formal-readiness 的结论**。
