# A4 v1 — Reachability-Guided Next-Best-View

A4 是独立的、ROS-independent、单步离线研究原型：

`A3 task field + 同一时刻的 A2 belief + 当前 UAV pose → candidate ranking`

不修改 A1/A2/A3，不执行观测或飞行，不连接 SIM。这里的 gain 是 **predicted observation gain / uncertainty-reduction surrogate**，不是校准概率意义的 expected information gain，也不是 Shannon entropy。

## 1. 数学定义

保留 A2 的 observation-deficit heuristic：

\[
u_N(x)=\exp[-N(x)/\tau],\qquad
\Delta u(x)=u_N(x)-u_{N+1}(x)
           =(1-e^{-1/\tau})u_N(x).
\]

`N = A2.observation_count`，`tau = A2.config.unknown_scale`（默认 2）。一次预测机会只计一次新的 informative endpoint；不改变真实 N。即使 A2 已为 FREE，仍可有正的 `unknown_score` 和边际下降量。

固定当前 A3 的 operational support：

\[
U_{\rm task}(x)=u_N(x)M_{\rm operational}(x),
\]
\[
\boxed{G_{\rm task}(v)=(1-e^{-1/\tau})
       \sum_{x\in D}\mathcal V(v,x)U_{\rm task}(x)}.
\]

`D` 为具有 nominal validated support 的环境 cell 域。`BLOCKED_ONLY` 在其中保留零 operational gain；`NO_VALIDATED_SUPPORT` 保留 NaN/mask，不参与求和，不被解释为全局 manipulation 无价值。A4 不因一次假想发现新障碍而重新计算整片 A3 footprint support。

因此，无飞行代价时，A4 v1 本质上选择能够观测最大 **task-relevant uncertainty mass** 的候选。`tau` 对全部 cell 相同，因此只是公共正系数；不是重新设计一套不确定性或 belief 模型。

对照方法只去掉 manipulation 权重：

\[
G_{\rm generic}(v)=\sum_{x\in\mathrm{A2\ grid}}\mathcal V(v,x)\Delta u(x).
\]

两者的候选、FOV、遮挡、range、飞行代价完全相同。Generic 也是 ground-endpoint observation-deficit surrogate，不是体积探索或熵 NBV。

所有求和均为固定 0.10 m grid 的 cell sum，不乘 cell area；gain 的单位是 heuristic-cell units。不是跨 resolution 的归一化比较。

## 2. 实际安装变换与 yaw 搜索

2026-09-07 读取并交叉核对了以下 SIM **现有配置**，没有启动 SIM 或读取 live TF：

- `/media/lu/P450_PAPER/SIM/p450_sim_v1/src/platform/sim_platform_bringup/config/p450_mid360_tf_contract.yaml`
- 同一包的 `launch/p450_runtime.launch`，其中 `Lidar/offset_*` 显式覆盖初始通用 YAML。

实际启用配置是 `uav1/base_link → uav1/lidar_link`：

\[
T_{\rm uav,lidar}=\begin{bmatrix}R_y(0.35)&t\\0&1\end{bmatrix},\quad
t=(0.14714489037277257,\ 0,\ 0.27696863564236896)\ {m m}.
\]

它由 mount `(0.13,0,0.23), pitch=0.35` 与 ray sensor 局部 `(0,0,0.05)` 复合得到；不能直接相加 z，也不能使用旧 `sensor_tf_offset_mid360.yaml` 的 zero-pitch 近似。

UAV 按 level hover 建模，即 roll=pitch=0，yaw 可变：

\[
T_{\rm map,lidar}(v)=T_{\rm map,uav}(p_v,\psi_v)T_{\rm uav,lidar}.
\]

例如 UAV 在 `(0,0,1.5)`、地面点 `(4,0,0)`，yaw=0 时传感器仰角约 −4.706°，yaw=π 时约 −43.248°。虽然水平 FOV 是 360°，倾斜的传感器垂直轴使 yaw 改变地面覆盖，所以当前 profile **保留 8 个 yaw**。

仅当传感器竖直轴与 UAV 竖直轴平行、且没有 XY 安装偏移时，360° 水平 FOV 的 sensing yaw 才等价：这种自定义 profile 自动只保留当前 yaw。XY lever arm 也会随 yaw 转动传感器原点，不能无条件合并。

## 3. 几何可观测性 V(v,x)

目标点是环境 cell 的中心 `(x_center,y_center,A2.ground_z_m)`。转换到传感器坐标 `s=(sx,sy,sz)` 后，仰角为 `atan2(sz,hypot(sx,sy))`。

MID360 官方标称水平 FOV 为 360°，垂直 FOV 为 −7°～52°。[Livox 官方规格](https://www.livoxtech.com/mid-360/specs)

`V=1` 当且仅当：

1. 目标 cell 不是 A2 OCCUPIED；
2. 传感器坐标中的距离满足 A2 的 `min_range_m < r < max_range_m`（默认 0.20–40 m，严格边界）；
3. 仰角在闭区间 `[−7°,52°]`；
4. 传感器原点到地面中心的 **3D segment** 不与任何已知 occupied prism 相交。

每个 A2 OCCUPIED cell 的闭 XY 矩形被挤出为：

\[
B_x=\mathrm{cell}_{xy}\times[z_g,z_g+H_{\rm assumed}],\quad H_{\rm assumed}=1.0\ {\rm m}.
\]

`H_assumed` 只是固定的 **occlusion surrogate**，不是 A2 测得的障碍高度。采用 segment/AABB slab intersection，接触边界也算遮挡。高度超过 1 m 的实际障碍可能漏遮挡；较矮或 overhead 的物体也可能被此投影近似过度遮挡。这是已批准的模型边界，没有新增 voxel/height-map/clearance 模型。

UNKNOWN 不遮挡；grid 外没有已知障碍模型。可见性因此是 **optimistic predicted visibility**，不证明真实无遮挡。对空白 cell 的射线也不构成 A2 free evidence。

`V=1` 只表示 **idealized one-informative-endpoint opportunity**。没有扫描 pattern、dwell-time、反射率、点密度、首次回波命中概率或通信/时序模型，不能声称单帧 MID360 一定观测全部这些 cell。重复调用 scorer 不会增减 A2 的 N；新的 N 只能来自外部真正的新观测。

传感器原点位于假设 prism 内、或不高于 ground 的候选不参与排序。这只是几何接口约束，不是 UAV 机体碰撞、路径或导航可行性判断。

## 4. 候选与 flight cost

默认候选是以当前位置为中心的 XY lattice：每轴 `[-4,-2,0,2,4] m`，高度固定为当前 UAV z，yaw 为相对当前朝向的 8 个 45° 样本。当前 pose 排第一，精确重复候选移除。当前倾斜 profile 共 **200 个候选**；centered upright profile 共 25 个。

候选不读取 A3 relevance。API 也允许传入显式有限候选集用于隔离因素的合成实验，此时应自行包含 stay/rescan。三个演示采用 8 headings 或明确的两个观察位置，而不是把这些小型对照宣称为大规模候选搜索性能验证。

\[
C(v)=\|p_v-p_0\|_2+\rho_\psi|\mathrm{wrap}(\psi_v-\psi_0)|,
\quad\mathrm{Score}(v)=G(v)-\lambda C(v).
\]

默认 `rho_yaw=.25 m/rad`、`lambda=.25 heuristic-cell-units/m`，均显式可配置，仅为演示设置，没有拟合或自动调参。它是直线位移加转向 proxy，不是路径长度、真实能耗或 collision-free flight cost。

按 score 降序，其次 cost 升序，再按原候选 ID 稳定排序。存在正 task gain 时可选择原地 rescan；所有有效候选都没有正 task gain 时返回 `NO_PREDICTED_TASK_GAIN` 和 `best_task=None`，不回退到 Generic exploration。无有效候选返回 `NO_VALID_CANDIDATE`。所有排名都是一步建议，不是执行指令。

## 5. API / 数据结构 / 状态

```python
from reachability_guided_nbv import Viewpoint, NBVConfig, rank_viewpoints

# a3_field 与 a2_belief 必须是同一环境 snapshot，使用已有内存对象。
current = Viewpoint(position_xyz=(0, 0, 1.5), yaw_rad=0, frame_id='map')
ranking = rank_viewpoints(a3_field, a2_belief, current,
                          config=NBVConfig(flight_weight=.25))
selection = ranking.best_task  # CandidateEvaluation 或 None；不自动飞行。
if selection is not None:
    contribution = ranking.task_contribution(selection.candidate_id)
```

输入检查只覆盖本算法直接需要的 frame/grid、有限数值、rigid TF、A3/A2 snapshot 配对以及 gain 恒等式；不重采样。未知程度或占据状态已有变化时，调用方先用现有 A3 builder 重建对应 field，再调用 A4。

| 结构 | 主要字段 / 含义 |
|---|---|
| `Viewpoint` | `position_xyz`, `yaw_rad`, `frame_id='map'`；level-hover UAV base pose |
| `SensorModel` | 完整 `T_uav_lidar`、垂直 FOV、`yaw_equivalent` |
| `NBVConfig` | lattice offsets / yaw count、assumed height、lambda、yaw-cost coefficient |
| `VisibilityPrediction` | `visible[H,W]`, `range_fov[H,W]`, `occluded[H,W]`, sensor origin、几何状态 |
| `CandidateEvaluation` | ID、pose、状态、task/generic gain、flight cost、两种 score、可见 cell 数 |
| `NBVResult` | `task_order`, `generic_order`, `best_task`, `best_generic`, `visibility[K,H,W]`、`delta_unknown`、cell contributions、A3 source/mask |

`range_fov` 已排除 occupied targets；`occluded` 只标记原本满足 range/FOV 的地面目标。不存在从其布尔补集推断 FREE/UNKNOWN 的规则。

语义继承：A1 `UNASSESSED/INFEASIBLE` 保留在 source arrays；A2 `UNKNOWN` 与它们独立。A3 `M_nominal`、`M_operational`、`BLOCKED_ONLY/NO_VALIDATED_SUPPORT`、clipped footprint 状态和来源均保存。`blocked` 仍仅指 A2-occupied-blocked，不是 navigation infeasible；A1 cell center + best_yaw 仍是离散代表 pose，不声称其本身通过原 candidate IK。

输出 `ranking.json` 保存候选、排名、配置、A3 coverage 与 pose 来源；`fields.npz` 保存 A2/A3 输入数组、visibility 和贡献，可直接重算各候选 gain。NaN 保留 no-support 语义。没有额外锁、provenance 或执行生命周期体系。

## 6. 三组离线验证结果

全部来自合成 A1 candidate fixtures 和合成 A2 endpoint histories，经原有公开 A1/A2/A3 builder 构造输入；不是真实 RM4D/IK 或 MID360 实测。非目标背景有 8 次地面 endpoint 观测，unknown regions 为 N=0。

| 场景 | 数值结果 | 说明 |
|---|---|---|
| task vs generic，8 个原地 headings | Ours 选东侧 #0，task gain **28.684**；西侧 #4 task gain **17.942** | 西侧有更大 unknown 和四个低 R=.12 footprints；东侧只有一个高 R=.9 footprint |
| 同一场景的 Generic | 选西侧 #4，generic gain **302.668**；东侧为 **61.594** | 相同候选、几何、代价，不同 weighting 产生不同选择 |
| occlusion，两个观察位置 | 无墙选 #0：gain **35.058**；有墙后 #0 降为 **0**，转选 #1：gain **35.058**、score **32.612** | 墙在 footprint 外；A3 operational support 和 U_task 数组逐项不变 |
| flight cost，两个观察位置 | lambda=0：远处 **29.003 > 28.684**；lambda=.25：远处 score **26.281 < 28.684**，留在 #0 | 远处多 **0.319** gain，却需 10.1 m 位移及 π 转向，C=**10.885** |

另有直接几何断言：同一条射线在 x≈1 m 从 1 m 棱柱上方经过，仍可见；棱柱移到 x≈3 m 时射线穿入其中，被遮挡。它能区分“XY 路径相交”和“真实 3D 相交”。

- [Task vs Generic 图](../outputs/a4/task_vs_generic/heading_choice/nbv.png)
- [无墙](../outputs/a4/occlusion/clear/nbv.png) / [有墙](../outputs/a4/occlusion/known_barrier/nbv.png)
- [越顶与穿越 side view](../outputs/a4/occlusion/through_vs_above.png)
- [无飞行代价](../outputs/a4/flight_cost/zero_cost/nbv.png) / [有飞行代价](../outputs/a4/flight_cost/with_cost/nbv.png)
- [数值汇总](../outputs/a4/summary.json)

图中使用原始 cell，不做 smoothing。黄色 predicted visibility 不是已观测 free；灰色 task 域是 NO_VALIDATED_SUPPORT，不是 zero relevance。

## 7. 复现与阶段边界

在仓库根目录运行：

```bash
PYTHONPATH=src MPLCONFIGDIR=/tmp/a4-mpl XDG_CACHE_HOME=/tmp/a4-cache \
  /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
  scripts/run_a4_offline_validation.py --output outputs/a4

PYTHONPATH=src MPLCONFIGDIR=/tmp/a4-mpl XDG_CACHE_HOME=/tmp/a4-cache \
  /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
  -m unittest discover -s tests -v
```

这个 Python 路径只是复用现有依赖环境；A4 算法不调用 frozen RM4D。输出只覆盖指定目录下 A4 命名结果。

最终验证：**147 项测试通过**（120 项冻结模块测试 + 27 项 A4 测试）。测试驱动覆盖 TF、3D 穿越/越顶、gain 恒等式、正的 FREE 边际收益、同源输入和完整 200-candidate 排序。独立代码审查无 critical/important 问题；审查后的两处小修正为可配置高度图注与严格 range 等值边界测试。

本阶段停在离线单步候选排序。SIM 的 Livox custom message、真实扫描时序/运动、flight execution、A5/mission、RL/World Model、benchmark 均未实现；A1/A2/A3 冻结模块及其已保存结果没有改动。
