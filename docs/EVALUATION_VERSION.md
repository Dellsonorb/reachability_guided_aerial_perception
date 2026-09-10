# Paper 1 evaluation version: finite-scan v1

2026-09-10. **Working evaluation version, not a completed formal experiment.**
This delivery names the existing complete program; it changes no robot runtime,
method, sensor, threshold, configuration or success predicate. The separate
[formal proposal](PAPER1_FORMAL_EXPERIMENT_PLAN.md) is not an instruction to launch
the old matrix. Machine-readable references: [evaluation_version.json](../configs/evaluation_version.json).

## One version, two repositories

Use tag `paper1-eval-finite-scan-v1` in **both** repositories, not either `main`.
The AGENT tag includes this documentation over the runtime below. The SIM tag
points directly to the already installed platform correction stack.

| Component | Evaluated runtime reference |
|---|---|
| AGENT | `1a6e85b42e9d9fd642454de499672004503c3772` |
| Latest twelve tasks | AGENT `0de63c5511b4732bb24090d7a75656eb2b752f1b`; same executable runtime |
| SIM | `a0ae8e32889e92a86f24d2794c7cb143999915fd` |
| Original RM4D baseline | `e9d431299053f38a4a4319aed3dfeccc261b9fac` |
| Independent task asset | This AGENT revision's `assets/rm4d_ground_task_v1/{rmap.npy,metadata.json}` |
| PX4 | `713814f4eea5990e49dd776a38a36ad53e171f60` |
| PX4 SITL Gazebo submodule | `9566172e7c0760f66681304b963d675c5b120daf` |

Executable code/profile/asset have no changes from the twelve-task version to
the reference above. The runtime includes the `66c20aa` arrival/HOVER readiness
fix. Do not substitute the old positive-z literature map: the independent asset
covers flange workspace z `[-0.25,1.3] m`; its collision floor is `-0.472 m` in
the calibrated internal robot convention. Coverage and collision floor are
different quantities. The map↔RM4D reference bridge and perceived map target
remain unchanged. Original baseline/tag/map and all historical results remain.

### Stacked PR map

These are cumulative ancestry, **not eight patches to replay on top of the tip**.
At this checkpoint all listed development PRs remain Draft; neither main is
silently promoted to the evaluation version. Tags provide a single named
checkout without squashing, rebasing or deleting historical branches/results.

| Repository | Dependency order, oldest → newest |
|---|---|
| AGENT | [#6](https://github.com/Dellsonorb/reachability_guided_aerial_perception/pull/6) A6/history → [#7](https://github.com/Dellsonorb/reachability_guided_aerial_perception/pull/7) sub-cell geometry → [#8](https://github.com/Dellsonorb/reachability_guided_aerial_perception/pull/8) measured presence → [#9](https://github.com/Dellsonorb/reachability_guided_aerial_perception/pull/9) clearance → [#10](https://github.com/Dellsonorb/reachability_guided_aerial_perception/pull/10) paired development → [#11](https://github.com/Dellsonorb/reachability_guided_aerial_perception/pull/11) task entry/screened handoff → [#12](https://github.com/Dellsonorb/reachability_guided_aerial_perception/pull/12) finite scan/readiness → [#13](https://github.com/Dellsonorb/reachability_guided_aerial_perception/pull/13) fixed evaluation and this index |
| SIM | [#2](https://github.com/Dellsonorb/Simulation-Platform/pull/2) XY latch → [#3](https://github.com/Dellsonorb/Simulation-Platform/pull/3) Ground frames/camera → [#4](https://github.com/Dellsonorb/Simulation-Platform/pull/4) velocity/full robot → [#5](https://github.com/Dellsonorb/Simulation-Platform/pull/5) clearance → [#6](https://github.com/Dellsonorb/Simulation-Platform/pull/6) checker observation retention |

AGENT main `060a4e2` is historical A1–A5. SIM localization PR #1 is already
merged into main `2e7feaa`; the later fixes descend from its source `5e25039`
(same tree at that localization checkpoint), not from the merge commit itself.
No localization fix is missing. Later merging these stacks is repository
housekeeping, not an additional method contribution. Every platform fix is
shared by Generic/Ours and all physically executing controls.

## Installed environment and launch contract

The current installation is at
`/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation`, with
`install/p450-clean` and `install/p450-runtime-overlays`. Noetic scripts use
`/usr/bin/python3` (3.8.10); the numerical worker uses
`/media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python` (3.10.21).
Ubuntu20.04.6, Gazebo11.15.1, MoveIt1.1.16, MAVROS1.20.1, move_base1.17.3,
NumPy2.2.6, SciPy1.15.2, Matplotlib3.10.9 and PyBullet3.2.5 were observed on this
workstation. Exact installed package strings are in the small version JSON;
`pyproject.toml` dependency ranges are not a frozen installed environment.

The runner loads some SIM source files directly (demo/checker/geometry), while
ROS resolves installed packages, launch files, URDF, sensor assets and plugins.
The 2026-09-10 read-only inspection found no unexpected source/install text
differences in the installed packages or executable-section differences in the
checked Ground/Livox plugins. The finite-scan predictor reads the installed
MID360 CSV/SDF used by the sensor. This checks today's installation, not every
historical binary. A fresh rebuild on another machine is not automatically a
new E2E-validated installation; prepare it using SIM's existing build guide and
run a separate development smoke test before independent evaluation.

From this AGENT checkout, the existing public single-task entry is:

```bash
# Read-only command preview; creates no task and starts no robot
python3 scripts/run_retrieval.py run --dry-run \
  --method generic --output-dir outputs/tasks/version-preview

# One normal sensor-driven task, when execution is actually requested
python3 scripts/run_retrieval.py run --method ours \
  --scene-file configs/fixed_version_paired.json --scene-id fixed-easy-01 \
  --output-dir outputs/tasks/new-development-run

python3 scripts/run_retrieval.py status outputs/tasks/new-development-run
```

The example scene is **development data**, never a final test scene. Use a new
output directory; no automatic replacement/retry. The entry enables
`--integrated-joint-velocity --full-robot-manipulation --execution-clearance
--ground-dynamics` together, including native feedback diagnostics.
It supports Generic/Ours, not a matrix. Source/install paths can be supplied
through `--sim-root`, `--rm4d-root`, and the existing `P450_PX4_ROOT` environment
variable. The default RM4D checkout is currently under `/tmp`; keep a persistent
checkout on another host. `CORE` in `run_a6_attempt.py` is still an absolute
workstation path, with no public `--core-python` override. Do not describe this
as a portable installer. See [CURRENT_SIM_TASK.md](CURRENT_SIM_TASK.md) for
display/environment setup, single-host ROS ports11951/11952 and failure exits.

## Common executable configuration

Authoritative profile: [current_sim_task.json](../configs/current_sim_task.json).

| Setting | Value / actual semantics |
|---|---|
| Frames and ground grid | map;0.10m;4×4m target-local grid; horizontal ground z=0 |
| Initial observation | map `[-1.4,0,1.2,0]` (XYZ/yaw); launch `[-0.5,0,0.15,0]` |
| Sensing budget | At most3 completed, accepted five-simulation-second windows, including initial; discarded acquisition attempts recorded separately |
| Target observer | Existing RGB-D4.0m depth gate; known brick class/dimensions |
| Support | Exact validated per-cell winner; v1.4 operational geometry and real ground-presence votes; at least2 observations for every required footprint cell |
| Acquisition prediction | finite_scan_v1: installed80-phase MID360 sequence; nominal51 packets/window; no fabricated returns |
| Viewpoints / cost | Existing1m XY lattice over offsets[-2,-1,0,1,2]; installed tilted sensor extrinsic/yaw search; shared flight cost weight0.25 |
| Flight bounds | x[-4,4],y[-3,3],z[0.5,3]m |
| Stop/handoff | `screened_candidate`: actual confirmation + bounded shared whole-manipulation preview (up to4 candidates); otherwise existing score/candidate/budget stop |
| Ground | max travel3m; navigation timeout120s; fresh D435 observation alternatives; actual-arrival replanning |
| Execution clearance | SIM `CHASSIS_MARGIN_M=0.012m`; development allowance, not calibrated safety assurance |
| Feedback / contact | Explicit SIM integrated-position-interval velocity backend; native diagnostics retained; full robot/AG95/perceived target/loaded-motion checks and real physical grasp |
| Hover acceptance | Existing position0.1m,yaw0.1rad,speed0.1m/s,dwell0.5s; settle timeout45s; no relaxation |
| Interface guards | Capture20 wall s; task1200 wall s; core900 wall s; TF max age0.5s/timeout1s |

The same-state only NBV difference is generic unknown weighting versus
manipulation-support weighting. Successful previews are not `D_exec`: that
requires actual BUNKER arrival, new D435 refinement and executed collision-aware
pregrasp. Non-winner alternatives stay diagnostic; no environment-driven winner
reselection. Candidate/IK fallbacks exist in the common bounded execution layer
but were not exercised by the latest twelve tasks.

RM4D's256-candidate validation budget still yields a partially assessed field,
not a complete robot capability map. A1 `UNASSESSED` is not `INFEASIBLE` and is
independent of environmental UNKNOWN. Relevance remains valid-gated joint-margin
quality; the operational geometry never rewrites raw A2 evidence into FREE.

## Supported domain, evidence and limits

This version retrieves a known red brick on static horizontal ground, initially
within the configured aerial camera search region, in a bounded UAV/Ground work
area. It is not arbitrary object search, dynamic clutter, unknown terrain or a
real-robot deployment. Nominal ray/ground opportunity is not calibrated return
probability. UNKNOWN does not occlude prediction or become FREE. Assumed1m
occluder height is not measured obstacle height. Exact winners, finite scanning,
conservative geometry and three windows can legitimately leave support missing.
Perceived camera alternatives, whole-robot kinematics and sampled collision
checks also have finite capabilities. Lift success includes the existing ≥0.10m
rise/physical checks and **short** retention, not indefinite load holding.

[Latest twelve development tasks](FIXED_VERSION_PAIRED_RESULTS.md): Ours6/6,
Generic5/6; all11 handoffs complete navigation/refine/pregrasp/grasp/lift/short
hold. One Hard/Generic task exhausts real support observations. No shared Ground
execution failure occurs in that batch, but six nearby scenes establish neither
general reliability nor an Ours advantage. Prior camera/flight/control failures
and all interrupted v1.1 slots remain development records, not final statistics.

**No distance-saving claim in this release.** Successful-task paths have missing
samples. Retained partial path sums are lower bounds, not full measurements.
Observation commands are not actual relocations; the plan defines separate
measured observation-location changes. Simulation active time, Ground time,
windows, task success and missingness remain useful without changing a recorder
or interpreting a rapid failure as efficient retrieval.

The old `configs/a6_formal.json` and `analyze_a6_formal.py` command-line workflow
target the historical560-slot protocol. They are **not** this release's formal
launcher/analyzer. Current execution flags require the runner's historical
`DEVELOPMENT_BATCH` enum. Formal activation will need a small explicit cohort/
launcher-label update and matching offline accounting, not a silent enum switch
or old matrix restart. That future bookkeeping must preserve this runtime and
be verified before the first final scene; it is not implemented or run here.

## This delivery's verification

No Gazebo/ROS task or setup simulation was started. Single-task `--dry-run`
created no output. Runtime/profile/asset diffs against `0de63c5` are empty;
56 existing entry/attempt/metrics tests and33 scan/statistical-helper tests pass.
JSON/reference checks and the sample-size calculations pass. Independent review
checked dependency composition, method fairness, IID tier counts, power/bounds,
controls, measurement semantics and compute/storage arithmetic. These checks
verify the documentation delivery, not any unrun final scene or fresh rebuild.
