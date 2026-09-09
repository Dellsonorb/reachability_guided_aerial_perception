# 最新版多场景配对开发验证结果

2026-09-09—10。**12/12 计划任务完成，六组配对可比较；额外启动0/4，INVALID_TRIAL=0，未重跑。**
七次原始独立 retrieval 检查通过，四次 Hard 在三窗内没有 confirmed candidate，
一次机器人完成 lift/保持但独立检查器失败。全部原始结果保留，没有为成功改代码、
参数、场景或判据，也没有恢复 formal matrix。这是开发评估，不是统计优势证据。

## 版本、共同条件与文档勘误

运行时 AGENT `c8b5b1bb47a332f9020487fe4dcf866c2403df34`；它仅在已通过自然
E2E 的 `6dfef58e54401f81188ff8d62b126e8dae43d961` 上加入本批场景/计划。
SIM 始终为 `e4e4f692f1524ded98e78abb82e39fd2dc7a86d2`，安装的运行代码为
`fbb191b`。十二次任务之间没有运行代码或配置变化，六对生成的 runtime.launch
相同，全部初始算法配置、sensor/A2/NBV 公共配置一致。

保留12mm开发净空余量、显式SIM integrated-pose速度反馈及原生诊断量、完整机器人/
夹爪/感知物体/夹后负载检查、真实接触闭合与原lift/保持条件。共同初始位姿
(-1.4,0,1.2,0)、4m observer gate、三窗、flight cost、Ground选择不变。
Generic/Ours 只改变 NBV gain weighting。所有任务从本轮正常P450感知开始，
没有人工指定站位、历史 confirmed pose 回放或运行中人工控制干预。

**勘误而非调参：**运行前计划将target allowance简写成历史例子0.04395m。
实际基线一直使用sensor/pixel几何推导规则，早期Pilot-2协议也明确此点；本批
记录为0.0462497737–0.0472089237m。代码拒绝外部提供调优allowance。两种方法
共用同一推导函数，数值随各自当前感知几何变化。本次修正文字，不修改原记录。

## 六组实际机器人行为

“完整”表示导航停车、D435 refine、实际collision-aware pregrasp(`D_exec`)、
下降、物理夹持、lift/保持均走完。retrieval列仍取原独立检查器/attempt判定。
确认列为第1/2/3窗 confirmed candidate 数。

| 场景 / seed | Generic：确认 → 最深阶段 / retrieval | Ours：确认 → 最深阶段 / retrieval |
| --- | --- | --- |
| Easy01 / 325746326 | 0/0/6 → 完整；通过 | 0/0/5 → 完整；通过 |
| Moderate01 / 1460872549 | 0/1/6 → 完整；通过 | 0/0/6 → 完整；**检查器失配，原结果false** |
| Hard01 / 1973399495 | 0/0/0 → 三窗停止，Ground未启动；失败 | 0/0/0 → 三窗停止，Ground未启动；失败 |
| Easy02 / 2001349247 | 0/4/6 → 完整；通过 | 0/0/4 → 完整；通过 |
| Moderate02 / 1293116599 | 0/3/5 → 完整；通过 | 0/3/5 → 完整；通过 |
| Hard02 / 1301813125 | 0/0/0 → 三窗停止，Ground未启动；失败 | 0/0/0 → 三窗停止，Ground未启动；失败 |

因此`confirmed → D_exec → adapter lift/保持 → 原始retrieval通过`为
**8 → 8 → 8 → 7**。没有导航、refine、规划、下降、夹持或lift的终止失败。
原始配对为三组双成功、两组双失败、一组Generic通过/Ours检查器失败；最后一组
discordance不是task weighting优势的证据，不作方法胜负或显著性推断。

八次Ground任务均由程序选择rank1 exact winner：Easy01/Moderate01各自为本轮
source582，Easy02为本轮source624，Moderate02为本轮source583。**编号不代表
跨场景相同姿态。**预检与实际到位后重规划均由首个measured-seed分支通过；
候选级/IK分支回退本批未触发，不能宣称已在线验证其恢复能力。

八次均先发生current-view D435超时，随后使用已有相机观察姿态获得有效精定位；
Moderate01/Ours额外经历view0、view1超时，最终view2成功。全部是程序自然行为，
无本批新加fallback。Ground refine耗时13.53–71.72sim s，说明相机可见性仍有
明显姿态依赖，而不是“目标IK可达就一定能看清”。

## 窗口、距离和时间

每次均完成三个当前MID360窗口，无丢弃窗口；换位数不含初始出航和返航。
同位重扫仍消耗窗口。距离为完整记录的主动感知段积分，时间均为simulation time。
`T_terminal`到首次方法终止，不包括失败后的全部清理/降落，不能视为成功所需时间。

| Slot / 场景-方法 | 窗/换位/同位重扫 | 首次确认窗 | 主动段 m / s | 导航 / refine s | T_terminal s |
| --- | --- | --- | --- | --- | --- |
| 1 Easy01-G | 3/1/1 | 3 | 3.405 / 51.958 | 31.753 / 14.227 | 212.641 |
| 2 Easy01-O | 3/2/0 | 3 | 4.257 / 52.616 | 41.865 / 14.619 | 223.834 |
| 3 Moderate01-O | 3/1/1 | 3 | 4.246 / 54.481 | 39.090 / 71.715 | 286.148 |
| 4 Moderate01-G | 3/2/0 | 2 | 4.356 / 50.994 | 39.350 / 15.895 | 242.895 |
| 5 Hard01-G | 3/2/0 | — | 4.346 / 50.377 | 未进入 / 未进入 | 92.803 |
| 6 Hard01-O | 3/2/0 | — | 3.525 / 52.166 | 未进入 / 未进入 | 92.922 |
| 7 Easy02-O | 3/2/0 | 3 | 4.295 / 51.922 | 56.813 / 14.095 | 240.135 |
| 8 Easy02-G | 3/2/0 | 2 | 3.379 / 53.068 | 30.430 / 13.534 | 227.635 |
| 9 Moderate02-G | 3/2/0 | 2 | 4.202 / 53.191 | 25.219 / 17.903 | 225.445 |
| 10 Moderate02-O | 3/2/0 | 2 | 4.216 / 52.792 | 51.015 / 13.765 | 233.689 |
| 11 Hard02-O | 3/2/0 | — | 7.027 / 71.053 | 未进入 / 未进入 | 111.901 |
| 12 Hard02-G | 3/2/0 | — | 4.245 / 51.654 | 未进入 / 未进入 | 92.724 |

八次进入Ground的UAV总程都有采样缺口；其全程距离保持null，只报告原始lower bound。
四次Hard截至方法终止的完整`uav_total`分别为8.357、7.469、10.938、8.243m，
**不含随后清理/降落**；另录`uav_cleanup`为1.177、3.232、1.253、1.132m。
不要将前一段距离与“起飞阶段至落地时间”配成完整飞行的效率指标。Ground总程除slot11
外均不完整；不能以近零lower bound代替“确证未移动”的完整测量，也不把已有
Ground位移gate误称为路径长度gate。全程缺测标记、下界、起飞阶段至落地时间、
全部执行阶段耗时见[逐任务CSV](../outputs/development/multiscene-paired-analysis/tasks.csv)
和[完整JSON](../outputs/development/multiscene-paired-analysis/results.json)。
Hard的短终止时间是任务未进入执行，并非高效率成功；本批没有窗口节省证据。

## Hard：实际支持不足，不是已允许继续却被内部表示永久阻断

四次最终仍有5–6个几何未阻断exact winners。将原始runtime endpoint用保存的公共
TF重新计数，全部12个Hard窗口的累计ground_presence与记录完全一致；预测visibility
重算也一致。下面只列与后续实际窗口一一对应的预测，未推定第四窗效果。

| Hard任务 | 几何可用winners | 各winner最终缺支持cell数 | 后续窗相关区域预测cell → 实际ground命中cell |
| --- | ---: | --- | --- |
| Hard01-G | 5 | 12,16,24,14,17 | 114→114；111→111 |
| Hard01-O | 6 | 4,1,6,26,21,17 | 150→132；152→148 |
| Hard02-O | 6 | 26,25,24,29,42,49 | 144→144；142→105 |
| Hard02-G | 5 | 4,3,1,4,5 | 144→144；139→139 |

“相关区域”是该次最终几何可用winner footprints的并集，任务间并集并非相同集合。
一次命中不等于满足原两窗支持条件。Hard01/Ours的source623、Hard02/Generic的
source541各只差一个cell，但该cell确实仅有一窗支持，仍正确拒绝confirmation。
其他任务既有零命中cell也有仅一次命中的cell，不应视为FREE。

差距包含两层：部分已预测可见cell没有实际地面端点；即使预测区域全部命中，
当前窗口序列也未使整个footprint获得两次支持。Hard02/Ours最后一窗全图预测556
cell、实际在预测内命中493；63个缺口中13个cell中心在实际所有chunk姿态下都不
满足FOV，50个在现有模型下至少一次可见但仍无地面命中。没有保存到遮挡前景射线
见证，不能进一步武断称其全部由遮挡或扫描pattern引起。仅能确认ideal endpoint
opportunity不是实际采样保证，且task uncertainty mass下降不等于整footprint完成。
见[逐cell覆盖诊断](../outputs/development/multiscene-paired-analysis/hard/diagnostic.json)。

十二个final states的exact anchors、A2原始数组、operational votes和全部assessments
重建一致。非winner仍只做诊断：四次Hard分别存在1/1/1/2个“winner blocked但有
几何未阻断非winner”的cell，共3/2/1/3个alternatives；它们也全部缺真实ground支持
（分别缺15/15/15、1/3、51、2/1/3个cell）。因此本批没有发现“仅换成已充分观测的
nonwinner就能解除Hard固定死锁”的证据，没有实现multi-support或重选。
完整T/E/A、raw-grid/operational blocking与target alias保留数见
[mechanism](../outputs/development/multiscene-paired-analysis/mechanism.json)及
[exact/nonwinner诊断](../outputs/development/multiscene-paired-analysis/exact-final.json)。

## 唯一检查器失败：与机器人执行失败分开

Slot3原始`VALID_TRIAL/retrieval=false`保持不变。adapter完成LIFT，TCP提升149.419mm，
真实接触与原保持阶段结束。独立原生运动诊断显示lift阶段物体实际上升147.586mm，
保持阶段0.661sim s内物体在腕子坐标系的最大位移约0.01186mm；这不是长时保持保证。
这些原生物理量仅用于离线诊断，绝未作为运行算法输入。

检查器报`no map observation matched status stamp 33.811000`，发生于LIFT后检查
早期AIR_HANDOFF的topic匹配时；其后其他检查没有完成。原physical-summary中的
物体提升指标仍缺失，不用上述离线诊断回填或将原结果改为PASS。

已复现一个真实checker bookkeeping缺陷：air/ground观测均只保留5000条，而早期
匹配在整任务末尾才验证。原样送入真实类的回调，匹配样本起初通过，6000条后续
回调后被淘汰并产生同样错误；仅在离线内存中改成lossless retention后保留匹配，
错误frame/stamp和超龄输入仍被拒绝。**本次现场究竟是淘汰还是原始订阅漏收仍未
完全证实**：旧记录没有checker内部buffer，diagnostic bag也没有air pose/UAV state
两个原检查输入。不能声称已经重建全部独立成功条件。

本批为保持统一版本未修改SIM checker，也未用额外启动追逐PASS。最小下一步是
修复有界任务内的观测保留，并保存这两个已有topic用于局部复验；不改任何物理或
感知新鲜度条件。见[可重复的小型对照](../outputs/development/multiscene-paired-analysis/checker_retention_reproduction.py)
和[原生运动诊断](../outputs/development/multiscene-paired-analysis/physical-launch03/REPORT.md)。

## 结论与下一步

- 共同Ground执行层在本批已不再系统性阻断Generic/Ours：八次有确认的任务均完成
  停车、有效refine、实际pregrasp、夹持和带物提升。仅限这八次，不是任意站位保证。
- 剩余主要能力边界是Hard的真实扫描支持/预算；还有一个已复现但现场触发原因尚不
  唯一确定的检查器记录缺陷。未发现新的已观测/几何允许却永久阻断的内部矛盾。
- **先做明确的小型checker修复和Hard覆盖边界复核，暂不进入效果/formal实验。**
  当前数据既没有证明Ours优势，也不足以推翻manipulation relevance引导感知的
  核心方向；不要从这个小批次决定正式样本量。
- 六个seed按原规则一次性产生，恰好全部target yaw为负；没有重抽补齐“好看”机制。
  这批覆盖有限，且传感器/OMPL时序未被scene seed完全控制，最终测试集仍需独立。

## 复核与交付

36个保存快照的两种gain/公共成本/argmax数值检查通过，22个同状态快照会给出不同
viewpoint选择；这是公共组件公平性诊断，不声称分叉后的闭环观测相同。
相关AGENT74项、SIM执行/相机/完整模型127项回归通过。独立只读审查复核了实际
启动顺序、十二次原始结果、共同配置、回退和缺测；修正上述allowance文字勘误。
原始attempt/events/metrics/点云/field保存于
[批次目录](../outputs/development/multiscene-paired/)，已有bag/native CSV保留本地，
继续按原.gitignore不入Git。没有删除旧输出、修改SIM或RM4D、增加框架或正式运行。

重建当前描述性表：运行
`python3 outputs/development/multiscene-paired-analysis/assemble_report.py`。
其他离线命令及输入范围见[分析说明](../outputs/development/multiscene-paired-analysis/README.md)。
本批在开发结果review checkpoint停止，四次额外启动额度全部未使用。
