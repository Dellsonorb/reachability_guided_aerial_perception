# Ground execution reliability — bounded development results

2026-09-09. **8/8 Gazebo startup invocations completed; no further starts.**
All eight reached the real Ground task boundary; no startup-invalid run or
automatic retry occurred. This batch is not a new Generic/Ours comparison.
It reuses archived runtime-confirmed exact candidates and aerial target poses,
with fresh Ground RGB-D and real robot execution. Historical paired results,
Pilot/formal records, RM4D assets, and operational gates remain untouched.

**Outcome:** Easy now reaches actual-tolerance arrival, fresh D435 refinement,
collision-aware pregrasp (`D_exec`), descent and grasp confirmation, but lift
still fails. Moderate obtains valid refinement; collision-aware grasp IK still
fails, and a non-executing diagnostic exposes arm/gripper↔BUNKER collisions.
Ground execution has improved, but retrieval reliability is not solved and
method development is not ready to close.

## What ran

[Original bounded plan](GROUND_EXECUTION_BATCH.md).
[All new records](../outputs/development/ground-execution-batch/).
Times below are simulation seconds from `GROUND_REPLAY_START`, not wall time.
`arrival-conditioned` means the base was spawned at the archived candidate to
isolate camera/manipulation; it is **not** evidence of successful navigation.
Every row is conditioned on historical confirmation, not new discovery.

| Start | Archived source / diagnostic | Change tested | Arrival / refinement | D_exec time | Retrieval / retained failure |
|---|---|---|---|---:|---|
| 01 | Easy Ours / navigation only | Install XY latch namespace correction | 120 s navigation timeout | Not assessed | Not assessed |
| 02 | Same / navigation only | Measure physics velocity before new command | 120 s navigation timeout | Not assessed | Not assessed |
| 03 | Easy Generic / arrival-conditioned | Camera-centered views + measured-top validation | Fresh refine at13.055 s |22.878| Fail: lift wrist velocity |
| 04 | Moderate Generic / arrival-conditioned | Same shared camera/refine | Fresh refine at12.700 s | No | Fail: six grasp IK codes−31 |
| 05 | Easy Ours / full Ground segment | Preserve declared base friction in SDFormat | 120 s navigation timeout | No | Fail: navigation |
| 06 | Same / full Ground segment | Eliminate periodic same-goal latch resets | MoveBase success; **98.66 mm actual error**; refine41.328 s |43.705| Fail: lift wrist velocity; arrival not a valid accuracy pass |
| 07 | Same / full Ground segment | CoG↔base-origin twist transport + fresh arrival check | Stop32.816 s; **59.414 mm /0.068291 rad**; refine45.572 s |48.039| Fail: lift wrist velocity |
| 08 | Moderate Generic / arrival-conditioned | Final shared stack + post-failure IK/collision diagnosis | Third bounded camera view; refine61.050 s | No | Fail: six grasp IK codes−31; diagnostic solution collides with base |

No row is a full aerial E2E trial. Ground-segment retrieval is false in all
six attempts that assess it; navigation-only retrieval remains unassessed.
Five attempts reach fresh refinement; three reach D_exec, descend and close.
The most informative improvement is launch07: original-position navigation
through D_exec, with the unchanged actual arrival and stop requirements.
Its arrival error is close to60mm; one pass is not a reliability estimate.
Launch06 is retained exactly as run, not retrospectively promoted or rewritten.
Derived `data/metrics.json` files distinguish recorded stops, actual arrival
events, D_exec and physical outcomes. Ground paths use the saved public TF;
missing samples remain unavailable, not interpolated. For example07's observed
path lower bound is2.2068m, but one missing sample prevents claiming a complete
path length. There are no UAV windows/distance metrics in a Ground-only replay.

No complete aerial E2E starts were spent: local lift still fails, so the
remaining budget went to discriminating the independent Moderate blocker.

## Navigation: the namespace fix was necessary but not sufficient

[Detailed diagnosis](../outputs/development/ground-execution-batch/hard-diagnostic/nav-diagnosis/REPORT.md).
The sequence isolated four additional causes rather than extending the timeout:

- Public odometry sampled velocity after imposing the next command, echoing a
  request that physics had not achieved. Read completed-physics feedback first.
- An unprefixed base collision name caused real SDFormat9 conversion to lose
  its already declared zero-friction surface. Correct the name; preserve base
  geometry, inertia, joints, collision presence, and the declared surface value.
  Launch05 then tracks low-speed commands, but still fails navigation.
- The2Hz global planner repeatedly clears DWA's XY latch on the same goal.
  Set periodic planning to0, retaining replanning on new goal/control failure,
  live local costmaps/collision checking and15Hz control. This is an explicit
  common SIM navigation configuration revision, not an algorithm score change.
- ROS Twist is for the base-link origin; Gazebo ODE applies linear velocity at
  the CoG. Use `v_CoG = v_base + omega × r` with the live model's CoG offset.
  Then independently verify actual stopped pose using the original60mm/0.08rad
  limits. Launch07 passes this check; no automatic corrective retry was added.

Original velocity/acceleration bounds, navigation timeout, collision checks,
and stop thresholds are unchanged. The base remains stationary during the
failed lift; base drift does not explain the wrist failure. In launch05, all
recorded target contacts were with the floor: the earlier swept-footprint
contact hypothesis is not asserted as that attempt's cause.

## D435: observation and grasp are separate feasibility questions

[Perception replay and mounting](../outputs/development/ground-execution-batch/camera-diagnostic/REPORT.md).
The public TCP→optical translation is(−0.095,0,0.075)m with rotated optical
axes. A grasp-centered arm pose does not center the target in the camera.
The old Easy partial red face is nearly vertical and image-clipped, yet the
legacy top estimator produced center z≈0.015m. That is a surface-model error,
not a reason to widen the existing target-height gate.

`camera_centered_v1` uses the full runtime mounting transform and CameraInfo,
checks all eight cuboid corners, and collision-aware-plans six bounded top
views (depth0.45/0.55/0.35m × two rolls). It does not require a grasp-IK branch
before allowing the camera to observe. A fresh red-surface cue can aim XY only;
it cannot supply grasp height or confirm the object. Aerial defaults are unchanged.

Ground refinement now requires an unclipped measured top, near-horizontal
normal (15°), and spans90–110%of the known0.240×0.053m top. It computes height
from the measured surface and retains the existing30mm center-height gate.
It does not replace observations with nominal dimensions/ground height.
These checks are deliberately conservative for partial/occluded surfaces.

All five camera attempts eventually refine. In08, views0/1 have successful
collision-aware arm motions but settle with image-clipped target observations;
view2 at0.55m yields fresh valid refinement. This is a genuine visibility/aiming
boundary handled by bounded observation, not by accepting a partial top.
An in-frame predicted cue is not a calibrated claim of visible full geometry;
occlusion is accepted only through subsequent real observations.

## Remaining manipulation failures

**Easy lift (03/06/07):** D_exec, complete collision-aware descent and close
succeed, followed by unchanged controller/stop rejection at wrist3 speeds
0.1561/0.1638/0.1679rad/s. Physical retrieval remains false. The repeated
near-constant position but high reported velocity is a controller/physics
diagnostic lead, not permission to substitute position differences for velocity
or relax the0.10rad/s condition. No gains or settling limits were tuned.
The [reproducible wrist audit](../outputs/development/ground-execution-batch/wrist-diagnostic/REPORT.md)
finds only74–147µrad sampled position span while every settling-window velocity
sample violates the limit. Native controller and joint-state streams share
simulated hardware feedback; even their1ms phase separation does not recover
all physics steps. The evidence does **not** establish that reported velocity
is false or that a particular backend defect is proven.

**Moderate grasp (04/08):** after valid refinement, all six collision-aware IK
calls return−31. In08 a single **post-failure, non-executing** kinematic query
returns a solution. State validity rejects that solution with contacts:

- `ground/base_link` ↔ `ground/forearm_link`;
- `ground/base_link` ↔ `ground/wrist_1_link`;
- `ground/base_link` ↔ `ground/left_finger`;
- `ground/base_link` ↔ `ground/left_outer_knuckle`.

No collision-disabled trajectory is planned or executed. This establishes
that an available kinematic branch is not full-robot collision free; it does
not prove that every possible branch is impossible. Current confirmed means
environment/ground support, not a guarantee of refined full-robot D_exec.
Respecting a real robot collision is a correct execution failure, not another
coarse-grid permanent blocker. Broader full-robot grasp feasibility remains
an unresolved handoff limitation; this batch does not reselect candidates.

## Hard: prediction versus actual support, no gate revision

[Complete per-cell/window diagnosis](../outputs/development/ground-execution-batch/hard-diagnostic/REPORT.md).
Both historical Hard runs reproduce all saved presence arrays and selected
visibility masks exactly. No lost evidence, fabricated votes or frame mismatch
is found. Of Generic's37 unique final missing cells,36 have measured foreground
ray interceptions and are already predicted occluded. For Ours,33 unique final
missing cells have no scheduled ground-cell ray in window2;18 nevertheless fall
inside the idealized predicted visibility mask. The actual MID360 pattern has
azimuth-dependent lower elevation limits rather than a uniform−7° rectangle.

Ours' best winners finish with103/108 and102/107 genuinely twice-supported
cells. The missing cells acquire first real ground support only in window3.
This is sparse sensing/model mismatch within the three-window limit, not
evidence that already sufficient true support is internally blocked. No
automatic votes, UNKNOWN→FREE, extra window, or operational-gate change.

## Verification, review and disposition

Production changes are common to Generic/Ours. `src/`, `configs/`, RM4D assets,
and historical results have no change from the incoming AGENT checkpoint.
Only Ground integration, diagnostics and tests change in AGENT; general robot
execution/perception changes live on the independent SIM feature branch.
SIM checkpoint: `feature/fix-ground-execution-reliability @ 7d2abf6`, stacked
on the preserved latch-namespace fix `ece36ec`. See its
[execution revision note](https://github.com/Dellsonorb/Simulation-Platform/blob/7d2abf6/docs/GROUND_EXECUTION_RELIABILITY_DEV.md).
[SIM PR#3](https://github.com/Dellsonorb/Simulation-Platform/pull/3) remains Draft;
AGENT PR#8 is the existing stacked development Draft. No main merge.

Fresh final verification: **637 core tests (22 environment skips)**;
**172 native-Noetic focused tests**, no skips; **103 targeted SIM tests**
(89 observer/demo/platform/conversion/twist +7 approach +7 navigation).
All pass; the suites overlap and are not summed. Changed SIM packages build
and install with the existing p450-clean profile. Both source diffs pass
`git diff --check`. A broad initial native-Python3.8 test selection incorrectly
included Python3.10 core modules; that command failed on type annotations.
The corrected core/native split above passes; no core compatibility changes
were made merely to accommodate the wrong test interpreter.

```bash
MPLCONFIGDIR=/tmp/a6-mpl XDG_CACHE_HOME=/tmp/a6-cache PYTHONPATH=src \
  /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python -m unittest discover -s tests
source /opt/ros/noetic/setup.bash
PYTHONPATH="tests:scripts:$PYTHONPATH" /usr/bin/python3 -m unittest \
  test_a5_manipulation test_a5_planning_geometry test_a5_ros_support \
  test_a5_target_support test_a5_tf_wait test_a6_adapter test_a6_attempt \
  test_a6_setup test_ground_segment
```

Independent review found two Ground-diagnostic reporting issues (pre-task
startup classification and missing conditioned metrics); these are corrected
with tests and offline derived reporting, without another Gazebo invocation.
Attempt records and historical results remain authoritative and unchanged.
Raw bags remain local and ignored by Git; compact events, public-pose traces,
attempts, physical summaries and diagnostic reports accompany the checkpoint.
All owned11951/11952 runtime processes have exited. The unrelated pre-existing
Gazebo instance and four unrelated untracked old A5 output directories were
left untouched.

Stop at this bounded development checkpoint; keep PRs Draft. No formal runs,
test-set selection, method-win filtering, budget extension or new framework.
The next priority is wrist/controller–physics consistency, then shared
full-robot grasp feasibility after Ground refinement. Hard scan-pattern
prediction remains secondary. **Do not declare Ground reliability or formal
readiness from this batch.**
