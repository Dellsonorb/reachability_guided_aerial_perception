# Pilot-1: structural-blocker diagnosis and proposed gating revision

Historical pre-implementation review retained below. The user subsequently
approved the bounded revision; implementation and regression outcomes are in
[OBJECT_AWARE_GATING_V11.md](OBJECT_AWARE_GATING_V11.md). The six execution-failure
classifications and original Pilot-1 results are not changed by that revision.

2026-09-08. **Review proposal, not an implemented or approved method change.**
Pilot-1 remains a protocol/debugging pilot. No Pilot-2, formal run, formal sample
size, new runtime activation, threshold change, or result reclassification was
performed for this investigation. A1–A5 and all original observations are intact.

## 1. Hard: continuous geometry is not the same as cell overlap

This analysis uses each run's recorded `AIR_HANDOFF.target_map` (RGB-D object
center and undirected long-axis yaw), **not** grasp TCP as object center. The
known `.240 × .053 × .115 m` object-size prior is already used by the public
retrieval interface (`SIM .../air_ground_pick_demo/config/demo.yaml:68`). It is
not an extent measurement from Gazebo. No `scene_spec`, model-state target pose,
or nominal scene yaw enters these calculations.

The perceived object rectangle is the size-prior XY rectangle transformed by
that live pose. BUNKER uses the unchanged **padded** continuous footprint,
half-length `.52 m`, half-width `.39 m`, at each original exact candidate pose.
Closed polygon intersection is tested by separating axes; disjoint distances
are minimum vertex-to-segment distances in both directions. Contact is blocked.
This diagnoses collision with the configured footprint and perceived model,
**not** actual chassis contact, navigation clearance, or certainty that a noisy
perceived object pose equals the physical object pose.

| Hard run | Exact catalog | Continuous intersection with perceived object | Disjoint but grid-blocked | Source-585 exact/object gap |
|---|---:|---:|---:|---:|
| Generic, slot 10 | 35 | 27 | 8 | .116752 m |
| Fixed, slot 11 | 35 | 27 | 8 | .117386 m |
| Ours, slot 12 attempt 02 | 33 | 26 | 7 | .117820 m |
| No occlusion, slot 13 | 35 | 27 | 8 | .117425 m |
| No cost, slot 14 | 34 | 28 | 6 | .122595 m |

The disjoint candidate suffixes (all other exact catalog entries intersect):

- Generic: `000001, 000008, 000010, 000012, 000017, 000021, 000034, 000068`.
- Fixed: `000001, 000008, 000010, 000012, 000017, 000021, 000034, 000067`.
- Ours: `000008, 000010, 000012, 000017, 000021, 000032, 000055`.
- No occlusion: `000001, 000008, 000010, 000012, 000017, 000021, 000034, 000067`.
- No cost: `000007, 000008, 000012, 000016, 000045, 000057`.

The three `000001` disjoint gaps are only 3.437, .778 and .609 mm respectively;
they are not robust clearance claims. No uncertainty padding was chosen to
alter this table. These **perceived-pose** results supersede the old report's
nominal-scene comparison for this question.

### Reproducible boundary example, without ground truth

[Hard Ours decision](../outputs/a6/pilot-20260908/slot-12-hard-ours-attempt-02/data/rounds/round-03/decision.json):
candidate `000008`, source 585, exact `(2.625410927, -.444112455, 2.094395102)`.
The live perceived object is `(2.121143447, .069170489, .059759734)`,
yaw `.241755535`. Their continuous XY gap is **.117820 m**.

All exact candidates nevertheless intersect occupied cell 781. For this run its
bounds are `[2.221143447, 2.321143447] × [-.030829511, .069170489]` m.
All seven saved occupied endpoints in this cell are **outside** candidate 000008's
continuous footprint, at distances `.111785–.116358 m`. Its A3 representative
center `(2.671143447, -.480829511)` misses the cell; hence representative support
is unblocked while the exact pose is blocked. This is geometric rasterization
aliasing, not a floating-point/contact-tolerance bug or failed candidate IK.

Endpoint replay applies saved `valid_return`, finite/range checks, full
`T_map_sensor`, A2 height thresholds and its searchsorted cell convention.
Across the five runs, the occupied cells intersecting any exact footprint are
`{778,779,780,781,819,820,821}`. Their 118 / 279 / 303 / 365 / 323 occupied
endpoints are near the live perceived target. Respectively 105 / 250 / 207 / 320 /
127 lie inside its XY size-prior rectangle. All have map z `.0501–.1146 m`,
consistent with the perceived brick's vertical extent. A2 retains these as
generic OCCUPIED evidence; there is no object identity channel.

**Association limitation matters:** the remaining endpoints extend up to
10.585 / 10.531 / 8.359 / 9.123 / 8.278 mm outside the respective perceived XY
boxes. In particular, *all seven* Ours cell-781 endpoints are 7.918–8.359 mm
outside that box. Saved clouds contain no semantic labels, and the corresponding
RGB-D masks/point subsets were not retained. Their spatial consistency supports
target-associated blocking, but does not prove every point's object identity.
Do not enlarge an association threshold to make this example pass.

Thus cases **1 and 2 are both present**, conditional on the explicit perceived
object model. Case **3 is a confirmed representation limitation**: A2/A3/A5 have
no way to distinguish occupied manipuland evidence from environmental evidence.
Attribution of every boundary endpoint remains unverified, not silently assumed.

### Occupancy removal alone would still be wrong

Ours cell 781 has A2 `occupied_evidence=2`, `free_evidence=1`. The raw three
windows contain ground/occupied endpoint counts `(23,0), (31,4), (22,3)`.
Occupied priority correctly suppresses the two mixed-window free votes. Merely
dropping its occupied count would leave only one free vote, not the frozen two
required for FREE. Conversely, setting the cell FREE would invent evidence.

For source-585 exact footprints, the numbers of cells with fewer than two raw
ground-containing windows are Generic 13, Fixed 67, Ours 0, no-occlusion 76,
no-cost 0. This is a **ground-evidence diagnostic only**, before semantic
association, environmental obstruction and continuous-object checks; it is not
a counterfactual success/discovery result or permission to relabel Pilot-1.

## 2. Minimal object-aware operational gate — proposal only

Recommended over either a blanket target-cell exemption or a new fine-grid/voxel
map: retain A2 verbatim and introduce a small **derived operational-evidence
view**, shared by representative A3 poses and exact A5 candidates.

Inputs already available at the integration boundary are the live perceived
target pose/known size and stamped RGB-D/MID360 geometry. A minimal perception
extension would expose the existing target segmentation/backprojected points,
rather than just its final pose. Target association must use this runtime
geometry and registration consistency, not a cell's proximity to the expected
target location. Geometrically ambiguous points remain environmental/ambiguous
occupied evidence. A pose box alone cannot confidently associate the cell-781
returns above. No fitted tolerance is approved by this report.

Keep a perceived object box/polygon `O_hat` and three small companion quantities:

- Target-associated occupied evidence, for explicit object geometry checks.
- `E_env[c]`: non-target **or ambiguously associated** occupied evidence. A mixed
  target/environment cell remains blocked. Preserve the same height, per-window
  voting, persistence and environmental occupied priority as today.
- `F_env[c]`: actual ground-return votes from the original windows, counted at
  most once per cell/window and suppressed by environmental/ambiguous occupied
  evidence. Target returns never create a free vote. Recover this from raw
  observations, not subtraction from A2's already-collapsed counters.

For either pose p and its unchanged continuous footprint P(p), define:

```
B(p) = [some covered cell has E_env > 0]
       OR [P(p) intersects the perceived object XY geometry]
M_operational(x) = max R(q) over unblocked representative poses covering x
U_task(x) = A2.unknown_score(x) * M_operational(x)
```

Object contact/intersection blocks just as environmental contact does. A
perceived model unsupported by the available registration/segmentation must not
produce an object-clear exception. This remains a conservative ground-plane
operational check, not a full 3D chassis/arm or navigation-clearance claim.

Exact handoff retains the exact original catalog, R/first-tie ordering, no
clipping, and the source representative's unblocked requirement. Replace its
literal `all(A2.state == FREE)` condition with `not B(exact_pose)` and at least
the **unchanged two** `F_env` votes in every covered cell. No ground observation
means unconfirmed, even if target collision has been disproved. Do not remove
the representative requirement as an unrelated selection change.

API sketch: `PerceivedManipuland(frame_id, stamp, pose, size, associated_points)`;
`OperationalEvidenceView(grid, environment_occupied_votes, ground_votes,
target_occupied_votes, manipuland)`; common `assess_pose` returns environmental
blocking, target collision, association ambiguity, clipping and ground support
separately. Missing/ambiguous associations retain ordinary occupied blocking.
This is one companion view, not a second mapper, voxel model or new framework.

**Frozen-semantic impact requiring approval:** A3 `blocked` is no longer literal
`A2 occupied > 0`; it becomes environment-or-object-geometry blocked. A5
confirmation no longer requires literal A2 FREE everywhere. Preserve the old
frozen implementations/results and identify the revision explicitly. Do not
disguise it as threshold tuning or an unchanged A5 integration patch.

A1 R/coverage, `M_nominal`, A2 state/votes/unknown score and A4 scoring,
visibility/occlusion/cost remain unchanged. UNKNOWN still does not block A3;
it does not satisfy A5 ground support. FREE still need not have zero unknown
score. A1 UNASSESSED stays separate from environmental UNKNOWN. Occupancy never
writes back A1. Generic/Ours must receive the same object-aware gate and evidence
view; only their observation-gain weighting differs. Raw A2 remains the A4
occlusion input in this minimal revision, including its existing surrogate
limitations; changing that would be a separate proposal.

Minimum checks after approval, before considering any pilot: continuous target
intersection/contact must remain blocked; target-only boundary overlap may pass
only with attributed target geometry and genuine ground votes; mixed/ambiguous
occupancy must block; target-only observations must not create FREE; exact and
representative checks must use one rule without substituting the representative
for the exact pose. Replay the retained clouds without changing Pilot-1 outcomes.

## 3. Six execution failures: observed failures versus causal defects

All six are valid, entered-task failures. None is reclassified INVALID merely
because a platform/perception limitation is suspected. Reproducing an error
message from a formula is not reproducing the original robot failure.

| Slot / method | Verified failure mechanism | Defect attribution |
|---|---|---|
| 3 / Easy RM4D-only | 120 simulation-second navigation timeout; repeated near-goal orbiting, never simultaneous XY/yaw convergence | Genuine observed navigation failure; a common configuration defect is confirmed below, but its causal attribution is not established |
| 9 / Hard RM4D-only | Same timeout; initially reaches near goal, then nearly stationary about .0612 m away with about 2.33 rad yaw error | Genuine observed navigation failure; not the same motion pattern as slot 3, and no causal runtime reproduction yet |
| 5 / Moderate Ours | Ground RGB-D callback runs, but unstable/no-usable-red-component observations do not yield a fresh accepted pose | Genuine near-field observation failure; not demonstrated stale-topic, dead node or TF-delivery defect |
| 6 / Moderate RM4D-only | Repeated no-usable-red-component rejections despite executed observation pregrasp | Genuine near-field observation failure; sensor/root-cause reproducibility not established |
| 7 / Moderate Generic | A fresh pose arrives; z=-.0429 m fails unchanged expected .0575 ± .0300 m height gate | Genuine rejected perception estimate; a low-surface estimator failure mode is reproducible, actual image/TF cause is not |
| 4 / Easy Generic | Arm controller aborts descend with GOAL_TOLERANCE_VIOLATED at wrist_1_joint | Genuine controller execution/settling failure; no repeatable dynamics/controller defect established from this one trajectory |

### Navigation

The public TF is fresh during both failed stages (maximum recorded ages .011 s
and .003 s). Slot 3 accumulates about 15.829 rad absolute yaw and ends .1583 m /
1.988 rad from goal. Slot 9 becomes nearly stationary during its last minute;
it is inaccurate to describe it as continuously spinning. Both receive 240 new
global plans in 120 seconds. Successful Easy Fixed also turns substantially
(about 9.916 rad), so turning alone does not diagnose a frame-sign defect.

**Reproducible configuration defect, not yet a reproduced causal failure:**
`src/ground/bunker_navigation/config/local_planner.yaml:17` and both captured
launch dumps set `/ground/move_base/DWAPlannerROS/latch_xy_goal_tolerance=true`.
The installed latch controller is constructed with an empty name and reads
`/ground/move_base/latch_xy_goal_tolerance`, default false. The latter key is
absent. Installed headers/library inspection and read-only `rosgraph` namespace
resolution corroborate the mismatch; see the
[ROS constructor](https://github.com/ros-planning/navigation/blob/noetic-devel/base_local_planner/src/latched_stop_rotate_controller.cpp#L22).
The minimal configuration correction would put the same intended boolean at the
consumed key, not loosen .06/.08 tolerances. It has **not** been applied.

Additionally, [DWA 1.17.3 setPlan](https://github.com/ros-planning/navigation/blob/1.17.3/dwa_local_planner/src/dwa_planner_ros.cpp#L134)
resets the XY latch with every new plan. The recorded 2 Hz replanning therefore
prevents promising that correcting the namespace alone fixes either timeout.
Neither commands, odometry twists nor costmap/controller branch histories were
retained. A focused unchanged-goal replay is needed to attribute the actual stall
or orbiting before proposing further changes. No controller setting or
navigation timeout was changed.

### D435

Read the **ground observer's own logs**, not identically worded aerial warnings.
Slot 5 includes an unstable window at sim 174.266 followed by repeated no-usable
red-component rejections; no-component rejection precedes backprojection/TF.
Slot 7 passed delivery/freshness and failed height validation after observation
pregrasp execution succeeded. The whole 29.607 / 29.341 s refine stages include
arm movement; the subsequent observation wait is the existing **20 wall-second**
runtime guard, not a demonstrated simulation-clock fault. Efficiency metrics
remain simulation time.

`perception.py:195–214` estimates top height from the upper percentile of whatever
masked points remain, then subtracts half the known object height. Offline replay
of a stable masked plane at z=.0146 returns center z=-.0429, reproducing the
estimator's response and correct height rejection—not the actual failed image.
Stable wrong XY/yaw can still pass: the near-field gate checks height/freshness,
not aerial/refined XY-yaw consistency. Slot 4's accepted refinement differs from
its aerial estimate by .210785 m XY and 84.739 degrees modulo-pi yaw. This is a
runtime cross-view discrepancy, not proof of which estimate is physically right.

The active D435 model has consistent co-located color/depth optical conventions.
An ideal projection using its existing mount, intrinsics and nominal pregrasp
clips the target top (approximately v=140–670 on a 480-pixel-high image). A
synthetic top-surface replay gives a 43.25 mm center bias, but preserves height
and yaw. This demonstrates a partial-view estimator limitation, not reproduction
of the much larger slot-4 discrepancy or the three Moderate failures.

There is also a conditional TF-model limitation: the active planar BUNKER plugin
publishes only relative x/y/yaw with z=0; static map-to-odom supplies spawn height.
Physical heave/roll/pitch would be omitted. The records do not establish such
motion or its contribution to any failed estimate; freshness alone does not
prove geometric accuracy. Actual raw RGB-D, masks, full camera/ground pose
and arm joint histories during refine were not saved, so object leaving FOV,
partial/incorrect segmentation, registration and view-dependent geometry cannot
be separated reliably from these logs alone. Do not add a Z offset, loosen the
height gate, or call this a reproduced global map calibration defect.

### Descend: correction to the Pilot-1 summary wording

The old description "DESCEND joint-speed check" is misleading. In slot 4,
`runtime.log:1071` reports `GOAL_TOLERANCE_VIOLATED: wrist_1_joint goal error
0.048390` at sim 261.817. SIM `run_air_ground_pick_demo.py:1046–1050` samples
`max_joint_speed` **after** `group.execute()` fails. `wrist_3_joint=1.1306` is one
latest-message diagnostic, neither the abort threshold nor a trajectory maximum.

The loaded position tolerance is .05 rad, stopped-velocity tolerance .10 rad/s,
and goal grace 2 s. The installed controller prints position error even when
velocity is the failed condition; .048390 < .05 therefore points to failure to
settle in velocity, conditional on the unrecorded per-goal overrides. Both
Fixed/Generic descents planned 31 points with 100% Cartesian fraction; Generic
execution lasted 6.534 versus Fixed 4.511 simulation seconds, consistent with
using the extra settling allowance. No clock reset/header rejection is logged.

Neither arm trajectory points nor controller desired/actual/error or joint-state
histories were retained. The two runs also used different refined geometry, so
they do not establish repeatability. Keep this as a genuine execution failure,
not an INVALID trial or permission to relax tolerance/retiming.

## 4. Review boundary

The Hard blocker is sufficient reason **not to scale the current protocol**:
more observation cannot clear persistent A2 occupied evidence, while real target
collisions must still be blocked. It does not establish Ours superiority or
justify selecting replacement scenes. The minimal proposed gate changes frozen
A3/A5 semantics and therefore awaits explicit review before implementation.

For unresolved downstream causes, the next useful work would be focused
diagnostic replays, not Pilot-2: navigation goal/cmd/odom/TF and costmap at the
goal boundary; synchronized D435 RGB-D/mask/camera TF and joints at the unchanged
observation pose; and executed joint trajectory/controller feedback through the
descend settling interval. Record only those necessary existing topics. No new
framework, success threshold or experiment matrix is proposed.

Verification for this review: the existing footprint/A5 core checks pass
**21/21** (2.205 s). The working-tree comparison against HEAD shows no change to
`src`, `scripts`, `tests`, `configs` or `assets`; only this report was added.
No implementation, commit/push, merge, or trial-result edit accompanies it.
