# A6 formal simulation — execution record

## Prospective freeze, before any formal outcome

The [formal protocol](A6_FORMAL_PROTOCOL.md) defines 120 independent scenes,
40 per existing tier, and 560 valid method slots. Exact seeds/specifications
and balanced order are in `configs/a6_formal.json`. Pilot-1/2 remain separate.
At creation of this record: **0/560 method slots; 0/120 setup scenes**. No
formal outcome has informed the design or sample size. The prospective freeze
was committed and pushed as `b8848ae` before the first setup activation.

Independent reviews cleared the seed/order/common-settings implementation,
paired analysis and stage-scoped native D435 logging. The review corrected the
analysis's input-identity checks and first-activation sensitivity wording before
execution. No A1–A5 method/asset or SIM source change is part of this freeze.

Verification before activation:

- Core: 496 tests, no failures, 21 environment-related skips.
- System Python/Noetic with its original PYTHONPATH retained: 74 tests pass,
  including TF-bag/OpenCV, launcher and paired-analysis integration cases.
- Frozen-baseline/task-map checks with the actual RM4D_ROOT: nine tests pass,
  including original positive-workspace and separate integration-asset geometry.
- Independent logging review: 44 core tests and 27 system-Python launcher
  tests pass. Default Pilot-2 command and adapter parameters are unchanged.
- Source diff against `56ece4a` is empty for research `src`, A5 runtime/target
  support, assets and Pilot-1 configuration/protocol. SIM worktree is clean.
- A first broad Noetic test command incorrectly included a Python-3.10-only
  scene-description test and overwrote the ROS PYTHONPATH. It was a test-command
  environment error, not a robot run or source defect. The corrected interpreter
  split above passes; no frozen source was changed to accommodate it.

Original online resource metrics remain primary, causal bag reconstructions
secondary. Full-trial control/TF plus handoff-to-end native D435 images project
about 680 GB total including setup/other outputs; current capacity is about
884 GB. This is an explicitly conditional storage estimate, not a worst-case
guarantee. The unadopted BZ2 probe and every pilot original are preserved.

## Running and reporting the fixed study

Use the existing P450 environment wrapper and owned ports 11951/11952. Set
`P450_PX4_ROOT` before the wrapper; set ROS/GAZEBO master environment variables
after it because the wrapper sanitizes inherited variables. Its runtime
dependencies are the unchanged local SIM and RM4D baseline/task-domain asset.
Never run two Gazebo instances concurrently.

For each frozen scene block, first run the existing launcher with
`--config configs/a6_formal.json --setup-scene SCENE_ID --output-dir NEW_DIR`.
Inspect `data/setup_summary.json`; do not use RM4D or method results to qualify
or replace a scene. Then run each prelisted slot with the same configuration,
`--slot NUMBER --output-dir NEW_DIR`. Keep all activations and classify terminal
measurements before considering an identical-slot INVALID rerun.

Generate progress summaries with `scripts/summarize_a6_pilot.py --config
configs/a6_formal.json --results-dir outputs/a6/formal --output NEW_SUMMARY`.
Despite its retained filename, this collector selects only FORMAL_ATTEMPT
records for the formal config; it never pools pilots or setup activations.
Use the existing scoring/operational/scene-description reporters uniformly.
The secondary bag resource reporter reads `diagnostics.bag`; it still contains
the complete original control/TF topic set.

The existing core interpreter has SciPy 1.15.2 for offline analysis. After all
planned valid slots complete, run `scripts/analyze_a6_formal.py --config
configs/a6_formal.json --summary FINAL_SUMMARY --output FINAL_ANALYSIS` there.
Before full completion it reports INCOMPLETE and no inferential p-value. The
primary exact-null interpretation, approximate versus conservative intervals,
secondary Holm family and missingness rules are fixed in the protocol.

No interim efficacy analysis, sample-size revision, scene replacement or method
tuning is permitted. Append actual progress, invalid history, stage/failure
tables and final figures as execution proceeds. Scientific stop conditions
remain those explicitly specified by the user.

## Execution progress

`easy-001` (seed565465572) passed its first method-independent setup activation:
846 distinct ground cells, 5.025 s window, mean ground-return map-z
`.000014826 m`, maximum absolute z `.010173 m`, accepted original RGB-D gate.
The native control bag finalized normally; no image recorder started because
setup never selects a Ground handoff. This activation is outside the method
sample. Its full output is `outputs/a6/formal-setup/easy-001-01`.

Formal slot1 (Easy-001/RM4D-only) completed VALID/SUCCESS: D_exec true at
122.649 simulation seconds, physical brick lift `.148212 m`, TCP lift
`.149478 m`, grasp confirmed and checker CHECKS_PASS. MID360 windows0 and
environment confirmation N/A, as prescribed. Task duration123.148 sim s, UAV
total distance5.218002 m. Ground online distance is unavailable because of one
sampling gap; its observed lower bound2.551287 m is not substituted for a
complete path. Both native recorders finalized with exit0.

Slot1's native image recording begins at51.908, before navigation64.617 and
GROUND_OBSERVE110.032. Both streams contain the exact refine stamp120.398 and
continue beyond LIFT. Control TF retains four `/tf` and eleven `/tf_static`
publisher connections. Both recorder logs are free of warning/overflow errors.
The separate secondary report reproduces original metrics and preserves the
genuine Ground sampling gap. Combined native bags are1.617 GB; this handoff
run is not representative of the planning mean that includes no-handoff runs.

Slot2 (Easy-001/Fixed) completed VALID/FAILURE after three completed windows,
zero NBV moves,35.273 active simulation seconds, without a confirmed candidate.
There are still five exact-unblocked candidates and three combined-clear
candidates (sources541/501/580). Final missing ground-support cells are11/17/37,
respectively, out of103/108/103. This is finite-budget coverage failure, not
an all-candidate operational deadlock. All three saved shared-scoring checks
pass; two have different task/generic argmaxes. One partial capture was discarded
by the unchanged hover rule; the three actual voted windows completed normally.
No handoff means no image recorder; control recording finalized without warnings.
Original UAV path metrics remain missing for one TF sample; secondary estimates
are separately5.353145 m total/1.449205 m active, not primary replacements.

Slot3 (Easy-001/Generic) completed VALID/FAILURE: three windows, two NBV moves,
one confirmed candidate first obtained in window3 at53.533 active sim s. BUNKER
navigation succeeded in76.351 sim s. D435 refinement then rejected measured
target z about`.0060 m` against the unchanged`.0575 +/- .0300 m` gate; D_exec
and retrieval are false. This actual perception/execution failure is retained,
not reclassified or rerun. Active duration53.534 sim s; UAV active/total
distance3.418733/10.957603 m. Original Ground distance is missing for nine TF
samples; partial lower bound2.871577 m is not a complete path. All six saved
shared-scoring snapshots through slot3 pass.

Slot4 (Easy-001/Ours) completed VALID/SUCCESS: three windows, two NBV moves,
two confirmed candidates in window3, actual D_exec and physical retrieval.
Brick/TCP lift are`.148917/.149760 m`; checker CHECKS_PASS. Navigation took
78.742 sim s, refine11.422 sim s. Active duration50.518 sim s, task215.794 sim s;
UAV active/total distance4.222348/12.661519 m. Original Ground path is missing
for four TF samples and remains missing in the primary resource table.

| Easy-001 method | Confirmed | D_exec | Retrieval | Terminal result |
|---|---:|---|---|---|
| RM4D-only | N/A | Yes | Success | Physical checks pass |
| Fixed | 0 | No | Failure | Three-window ground-support budget |
| Generic | 1 | No | Failure | Fresh but height-inconsistent D435 refine |
| Ours | 2 | Yes | Success | Physical checks pass |

This first primary pair is Ours-only. It is a descriptive record, not an interim
test or efficacy claim. Ours used more UAV distance here despite success;
shorter active duration is not automatically overall retrieval efficiency.
Nine saved same-state scoring checks pass; five task/generic argmaxes differ.
All three descriptive initial snapshots have true-box occlusion0 and33 nominal-
clear high-relevance footprints. Fixed/Ours have zero fully initially visible
high footprints versus one for Generic, missing the old Easy visibility proposal
target. This measured FOV/pose variation is reported, not a reason to discard,
relabel or replace the scene.

At the end of this first block: **4/560**, zero INVALID activations. Independent
saved-state review checked all nine score/NPZ identities with zero numerical
discrepancy and 315 exact-candidate assessments against evidence and geometry.
Actual Generic/Ours actions and common stopping rules agree. No fairness or
scientific-stop concern was found. The Fixed evidence does not establish that
additional fixed rescans would necessarily succeed; it establishes only the
narrower finite-budget-no-confirmation result.

### Moderate-001, replicate 1

Seed1863681335 passed the unchanged setup: 591 distinct ground cells, 5.026 s
window, mean ground-return z `.000017639 m`, maximum absolute z `.014352 m`,
accepted original RGB-D gate. Setup output is
`outputs/a6/formal-setup/moderate-001-01`. This is outside the method sample.

Slot5 (Fixed) completed VALID/FAILURE. Three completed windows yielded counts
0/0/2; first confirmation35.838 active sim s, active35.839, task230.190 sim s.
One unsettled partial capture was discarded by the existing rule; no extra
completed window was granted. Selected source584 had96/96 real ground-supported
cells, no occupied or operational block, relevance1. Navigation succeeded in
89.608 sim s. The subsequent ground_refine stage failed after54.906 sim s with
the recorded message `MoveIt refined pregrasp planning failed`.

Focused classification distinguishes that message from fresh D435 refinement:
there is no GROUND_REFINED, D_exec-ready or arm trajectory goal/result. The
shared A5 helper also uses this error text for the **initial observation-position
pregrasp based on the aerial target**. Timing fits six10-wall-second grasp-IK
attempts before Cartesian/OMPL planning; exact service response codes are not
recorded. Causal TF before the inferred first request gives actual base
`(2.403554,-.524504,3.063060)` versus selected
`(2.391735,-.537797,-pi)`, a1.779 cm XY/4.500-degree yaw error, TF age.059 s.
This shifts the target's base-relative XY by6.93 cm; it plausibly affects IK
but does not prove causation. Auxiliary-frame TF warnings also occur in a
successful run. No integration defect is demonstrated; preserve this valid
planning failure, without adjusting parameters or retrying.

Slot5 UAV active/total paths are1.422095/6.486255 m. Original Ground distance
is unavailable (two TF misses). The uniform secondary report reproduces raw
metrics and non-path values, and Ground remains unavailable for a genuine
sampling gap. Native image indexes cover the full failed refine interval,
836frames/stream. Both recorders finalized with exit0 and no overflow warnings.

Slot6 (RM4D-only) completed VALID/FAILURE, original120-s navigation timeout,
zero MID360 windows, confirmation N/A, no D_exec or retrieval. Control replay
shows the goal ACTIVE until timeout/cancellation, then PREEMPTED. All1,800
navigation/guard commands agree;6,000 odometry samples have maximum gap.030 s.
Final error is.134160 m XY/.341021 rad yaw; no recorded pose met both original
tolerances. The last30 s contain.210 m travel and2.927 rad turning, supporting
navigation-convergence failure, not command starvation or a demonstrated
platform invalidity. Both recorders finalized normally. Original/secondary UAV
total5.963524 m and Ground2.656754 m match with no missing samples or gaps.

Slot7 (Ours) completed VALID/FAILURE: three windows, two moves, two confirmed
candidates first in window3 at52.427 active sim s. Active/task times are
52.428/201.129 sim s; UAV active/total paths3.403576/11.493130 m. Navigation
succeeded in33.952 sim s; ground_refine failed after54.374 sim s. As in slot5,
there is no fresh accepted GROUND_REFINED, D_exec or arm trajectory. The failure
is the aerial-target observation-position pregrasp, with timing consistent
with six10-wall-second IK attempts; exact rejection codes remain unknown.
Actual causal base pose `(2.412979,-.502685,3.062860)` differs from the exact
selection by4.280 cm/.07873 rad, changing the same target's base-relative XY
by9.35 cm. This is a plausible feasibility influence, not a proved cause.
Original Ground distance is unavailable for one sampling gap, which persists
in the separately preserved secondary reconstruction. Both native recorders
finalized normally; images span the entire failing phase.

Slot8 (Generic) completed VALID/FAILURE: three windows, two moves, two final
confirmed candidates. First confirmation occurred in window2 at43.641 active
sim s; the method correctly continued to the common stopping rule, not an
early confirmation stop. Active/task times are53.053/267.293 sim s; UAV
active/total paths4.216646/12.518591 m. Navigation succeeded in101.146 sim s;
ground_refine then failed after53.544 sim s with the same recorded pregrasp
planning message. D_exec/retrieval are false. Original Ground path is missing
for two TF samples, not replaced with a partial path.

| Moderate-001 method | Final confirmed | D_exec | Retrieval | Terminal stage |
|---|---:|---|---|---|
| RM4D-only | N/A | No | Failure | Navigation timeout |
| Fixed | 2 | No | Failure | Observation-position pregrasp planning |
| Generic | 2 | No | Failure | Observation-position pregrasp planning |
| Ours | 2 | No | Failure | Observation-position pregrasp planning |

This primary pair is neither-success. All three environment-gated methods
passed confirmation and Ground navigation; the shared execution bottleneck is
reported rather than tuning the methods or erasing their failures. Across the
two closed blocks, all18 shared-score snapshots pass and12 argmaxes differ.
The first-window descriptions have five proposal mismatches among six saved
snapshots; no scene is replaced or relabeled. Detailed descriptive fields are
in `outputs/a6/formal/scene-description-progress.json`.

Current completed method sample: **8/560**, two successes/six failures, zero
INVALID activations. The two primary paired records are Easy-001 Ours-only
and Moderate-001 neither-success, with no interim test or efficacy conclusion.
Slot8's focused review likewise places its failure before fresh refinement:
no GROUND_REFINED/D_exec or arm trajectory. Both recorders finalized normally;
there is no demonstrated platform defect. These failures are not INVALIDs.

Hard-001 (seed719495214) has now passed setup:399 ground cells,5.028 s window,
mean ground-return z.000044814 m, maximum absolute z.017458 m, original RGB-D
gate accepted. All three first-replicate setups have passed without pose/seed
changes. Hard core/ablation slots9–14 follow in the prelisted order. Final
inferential analysis remains unavailable until every planned valid slot
completes. Progress is in Draft PR#6; main is not merged.

### Hard-001, replicate 1

Slot9 (Fixed) completed VALID/FAILURE after three windows, one discarded partial
capture, no confirmed candidate and no handoff. Active/task times are
36.569/76.506 sim s. Of36 exact catalog entries,33 are raw-grid blocked and33
object-aware exact blocked. Two combined-clear sources666/664 remain, with
65/107 and53/106 real ground-supported cells (42/53 missing). TARGET,
ENVIRONMENT and AMBIGUOUS class votes/cells are15/5,84/28 and18/6; these classes
can overlap. No exact TARGET-alias rescue occurred. This is a retained finite-
budget coverage failure, not an all-candidate blocking claim. Uniform secondary
resources reproduce original values with no missing/gap samples; no image bag
exists because no handoff occurred.

Slot10 (RM4D-only) completed VALID/FAILURE: navigation succeeded, then the
ground_refine stage151.673–163.001 rejected a fresh map target height
`-.003387794 m` against the unchanged `.0575 +/- .0300 m` gate. The public pose
stamp162.750 was received162.967: image age251 ms at rejection. Both RGB/depth
images and both CameraInfo messages have the same162.750 stamp, matching
intrinsics/frame metadata and the declared identity depth-to-color transform.
Offline frozen geometry reproduces exactly7,376 mask/depth points and single-
frame estimated center z`-.00339535 m`. Visible mapped z95 is`.054116 m`, from
which the frozen upright-brick estimator subtracts half-height`.0575 m`.
The red component touches the image bottom. Its
[unmodified RGB frame](../outputs/a6/formal/slot-010-hard-001-rm4d-only-01/target-disturbance-rgb.png)
is a diagnostic extract, not an efficacy figure or a modified observation.

Saved `/pick_target/contacts` establishes physical target disturbance during
navigation: BUNKER `ground/base_link` contact starts141.407 and ends143.991.
The floor-support rectangle measures`.240 x .053 m` at141.357 before contact,
then`.240 x .115 m` at144.991 and151.611, before GROUND_OBSERVE151.673. These
match the known brick dimensions and strongly support tipping onto its narrow
side. The ground-contact centroid moves`.186507 m`, from
`(1.993060,.076431)` to`(1.896128,-.082908)`. This accounts for a low visible
surface without demonstrating any registration/calibration defect. Contact
measurements are retrospective diagnostics, never algorithm inputs. Preserve
the original VALID ground_refine height-rejection outcome; do not relabel or
retry it. No complete four-frame fusion replay is claimed.

Slot11 (Ours) completed VALID/SUCCESS: third-window confirmation of source666,
105/105 real ground-supported footprint cells, no continuous target or other
operational block. There are34 catalog entries,30 raw/operational exact blocked;
the other combined-clear source664 has106/108 ground-supported cells. Class
votes/cells are TARGET16/6, ENVIRONMENT84/28, AMBIGUOUS17/6. No exact TARGET-
alias rescue occurred. First confirmation51.332 active sim s; active51.333,
task170.711; two NBV actions and three completed windows. Navigation/refinement/
executed refined pregrasp take33.610/11.085/1.097 sim s. D_exec is true at
157.475 sim s since task start. Brick/TCP lift are`.148579/.149572 m`, grasp
and retention pass, physical checker CHECKS_PASS. UAV active/total paths are
3.433868/11.075556 m. Original Ground path is missing; the causal secondary
also retains a sampling gap. Both native recorders finalized normally.

Slot12 (Generic) completed VALID/FAILURE, three windows/two NBV actions, zero
confirmation and no handoff. Active/task times51.513/92.385 sim s. Catalog34,
raw/operational exact blocked30; combined-clear sources666/664 have88/106 and
88/108 real support (18/20 missing). Class votes/cells TARGET5/5,
ENVIRONMENT81/28, AMBIGUOUS6/6; no exact alias rescue. Original and secondary
resources agree without missing samples/gaps; UAV active/total4.26225/8.27451 m.
Shorter total runtime than Ours includes this early valid failure, not faster
successful retrieval. The Hard primary pair is Ours-only; no interim inference.

Slot13 (no occlusion) completed VALID/FAILURE after three windows/two NBV
actions, no confirmation or handoff. Active/task53.157/93.680 sim s; UAV active/
total1.314361/5.302689 m. Catalog36, raw/operational exact blocked32; two
combined-clear sources666/664 have52/106 and43/107 supported cells,54/64 missing.
Class votes/cells TARGET16/6, ENVIRONMENT84/28, AMBIGUOUS18/6; no alias rescue.
Original Ground path remains missing; the secondary-only reconstruction is
complete at.001955 m. No outcome/resource substitution is made.

A bounded contact check on the earlier Easy/Generic slot3 also shows real
BUNKER–target contact, from181.239 during navigation through215.962 just before
the215.977 height rejection. Floor-contact centroids move about.19916 m. Unlike
slot10, a settled`.240 x .115 m` side-support rectangle is not established;
sampled four-point supports remain`.240 x .053 m`, with only two floor points
at the end. Physical disturbance is observed, but its exact contribution to
the rejected estimate remains uncertain. Keep that original ground_refine
failure, not a new class or an INVALID. A clear selected footprint does not
establish collision-free navigation or swept turning geometry.

Slot14 (no cost) completed VALID/FAILURE after confirming two candidates in
window3 at52.940 active sim s. Sources666/664 have106/106 and107/107 ground
support; catalog35, raw/operational exact blocked31, no alias rescue. Class
votes/cells TARGET14/5, ENVIRONMENT82/28, AMBIGUOUS17/6. Navigation/refinement/
refined pregrasp/descend succeeded; D_exec true at167.593 sim s since task
start. Close failed after.488 sim s with AG95 action state4 (ABORTED); task
duration173.162 sim s. All three recorded arm trajectories returned success,
the last at184.551 before close184.559–185.047. During184.400–185.050 the
knuckle remained`[-.02350,.00796] rad`, below the original.20-rad closed
threshold; all14 grasp samples were false, with left-finger/floor target
contacts only. Confirmed-stall acceptance conditions were not met. The actual
gripper action's detailed error/tolerance code is not in the first-replicate
bag topic set; do not invent it. This is unsuccessful physical closure, not a
demonstrated platform invalidity. UAV active/total3.498481/10.964507 m; original
Ground distance remains missing, including a gap in its secondary estimate.

Independent Hard review checked15 shared-ranking/action snapshots and525
exact/representative assessments. Actual ablations also pass:195 no-occlusion
rows use range/FOV-only visibility with original costs/gates;147 no-cost rows
retain visibility/gains and remove only the score penalty, with original
tie-breaking. All methods retain combined-clear candidates. Support fractions
above are each trial's own perceived geometry, not identical cross-run cells.
No fairness, structural-handoff or scientific-stop concern was found.

First formal replicate complete: **14/560 valid slots**, three successes,
eleven failures, zero INVALID activations; three unchanged setup checks pass.
Primary records are Easy Ours-only, Moderate neither-success and Hard Ours-only.
There is no inferential efficacy look. The two Hard ablations fail at different
stages: no-occlusion at confirmation budget, no-cost at close despite D_exec.
Across the replicate,33 shared-scoring snapshots pass and27 argmaxes differ;
1,161 candidate assessments have been independently reviewed. All14 native
control/TF diagnostic reports reproduce original metrics/non-path values;
secondary paths remain separately labeled. Next is prelisted Hard-002 setup,
then slots15–20, followed by Easy-002 and Moderate-002 in the frozen order.

### Storage-only correction after replicate 1

Actual first-replicate native bags total22,889,315,916 bytes, including
21,903,572,765 image bytes and985,743,151 control bytes. Three setup bags total
44,267,748 bytes. Extrapolating this one observed mix to the remaining39
replicates gives894,409,762,896 bytes, versus860,572,139,520 bytes available at
the check: a33.84 GB shortfall before other files. This is a conditional storage
projection, not a reason to change sample size, outcomes or acquisition.

Use the already installed Zstandard CLI1.5.6 to archive **the complete native
image-bag byte stream**, only after capture, native health checks and readers
have finished. This is not `rosbag` rewriting/recompression and does not merge
publisher connections or remove images. Control bags remain directly readable;
all observation/metric/physical-result files remain unchanged. The first
slot5 probe, at default level3/one worker, compressed2,872,802,712 bytes to
2,364,217,216 bytes (ratio.822965) in8.68 wall seconds. Decompression was
compared byte-for-byte with the source, exit0, in2.62 s. The source native
format and all its connection metadata are therefore retained exactly.

Closed image bags are now stored as`diagnostics-images.bag.zst`; their expanded
`.bag` copies are removed after successful compression. Restore one when needed
with the installed standard CLI, keeping the archive:

```bash
zstd -d --keep TRIAL/diagnostics-images.bag.zst -o TRIAL/diagnostics-images.bag
```

Do not overwrite an existing restored bag. Capture still writes the original
native bag first, with unchanged topics/rates/trigger and no compression work
during the trial. The original attempt's finalized/path fields describe that
capture, not a rewritten historical record. Archives remain local and ignored
by Git, like their expanded originals. The source-byte-equivalent storage
change does not affect method or robot-efficiency measurements. No new archive
framework, evidence chain, dependency installation or sample change is added.

All10 first-replicate image archives passed the standard decoder integrity
test (exit0,32.23 s); total archived image bytes17,673,211,074, saving
4,230,361,691 bytes. Control native bytes remain985,743,151. At this observed
mix, full-study image archives plus native control project746.36 GB, before
setup/other files. This remains a sample-dependent planning estimate; keep
monitoring capacity between blocks. No Pilot-1/2 bag was changed.

### Passive gripper logging correction, from replicate 2

Slot14's missing detailed gripper-controller response motivates five additional
**formal-only passive** native control topics:

```
/ground/gripper_controller/follow_joint_trajectory/goal
/ground/gripper_controller/follow_joint_trajectory/status
/ground/gripper_controller/follow_joint_trajectory/result
/ground/gripper_controller/follow_joint_trajectory/cancel
/ground/gripper_controller/state
```

Names match unchanged SIM demo/controller wiring. Only the existing recorder's
topic list changes, uniformly for all subsequent formal methods and setups;
there is no new robot-side client, command, delay or policy input. Original
Pilot/default recording and both image scopes remain unchanged. The original
14 measurements, success criteria and failure stages are not rewritten or
retried; slot14's detailed gripper error remains unknown. Formal configuration,
seeds/order, deadlines, thresholds, costs, gates and A1–A5 are unchanged.

Tests first fail for the five absent topics, then pass after the eight-line
recorder extension. Fresh full core suite:517 tests,21 unchanged environment
skips; Noetic/OpenCV/launcher/analysis split:75 tests pass without skips. The
source diff remains empty for research src, A5 scripts/assets and Pilot-1
configuration/protocol; SIM worktree remains clean.

The actual frozen/task-domain RM4D asset split also passes all9 tests, covering
the remaining external-map skips. Independent review returns GO: the five
topics are passive and formal-only; both image scopes, disabled diagnostics
and unchanged Pilot recording are tested. The storage change preserves native
bytes and historical results. No engineering or scientific-stop blocker is
identified before replicate2.

## Replicate2: scientific pause after slots15–16 (2026-09-09)

Hard-002 (prelisted seed958985919) passes the unchanged method-independent
setup: fixed initial pose`(-1.4,0,1.2,0)`,364 distinct ground cells,
5.028 sim-second window, maximum absolute ground-return z.0192213 m. New
passive gripper topics are present in the native setup bag (3,833 state and
192 status messages); no gripper action is expected during setup.

Slot15 Generic completes three windows/two NBV moves and fails with zero
confirmed candidates at the original budget. Active/task52.169/92.235 sim s;
UAV active/total4.175445/8.118505 m, all original paths complete. Slot16 Fixed,
already launched when the structural issue was noticed in slot15's closed
mechanism report, completes three windows/two stationary rescans with one
discarded partial capture and fails at the same budget. Active/task37.051/
76.722 sim s, UAV active1.382368 m; its original Ground path remains missing
(one missing sample), with a separately labeled complete secondary estimate.
Both retain **VALID_TRIAL failures**, no D_exec or retrieval, no handoff/image
capture. Both control recorders exit0/finalize cleanly; all secondary reports
reproduce raw/non-path metrics. These are not INVALIDs and will not be retried.

### Newly demonstrated scene-local v1.1 structural handoff lock

This is not a new efficacy look or a reason to select different scenes. The
user's explicit scientific-stop condition is now met by runtime/perception
geometry in slot15:

- All34 A1 catalog representatives are blocked in the **first** window and
  remain blocked through window3. M_nominal covers321 cells, but M_operational
  has no positive cells and U_task mass is0 in all three saved snapshots.
- Of34 exact catalog poses,33 are operational-blocked. Exact source624 is
  unblocked, has no ENVIRONMENT/AMBIGUOUS footprint cells and no continuous
  target collision, and reaches87/88 real ground-supported cells by window3.
- The exact padded footprint and already-allowance-expanded perceived target
  have a19.4184 mm separating gap along map y. Replacing the exact source624
  pose with its A1 cell center shifts y by+40.5502 mm and creates21.1318 mm
  projection overlap; the full continuous-rectangle SAT confirms collision.
  A3 explicitly uses a discrete representative, not an IK-validated cell
  center. A5 additionally requires this representative to be unblocked, so
  the clear exact pose cannot be confirmed even if its last ground cell is
  observed. This is a representative-quantization veto, not evidence that
  the exact pose is physically impossible.
- All34 representatives also intersect accumulated AMBIGUOUS evidence (none
  intersects ENVIRONMENT evidence);31 have continuous target collision.
  Shared cell781 already receives three ambiguous endpoints in window1.
  Their initial RGB-D reference lacks positive interior-mask/valid-depth
  association. Later observations cannot erase the retained ambiguous vote;
  target/reference geometry is fixed during this air phase. Extra ground
  votes therefore cannot remove either fixed blocker. No TARGET exception,
  threshold relaxation or counter subtraction is used in this diagnosis.

Independent reconstruction matches all102 saved exact assessments, A3 arrays,
three policy/action checks, every raw A2 snapshot and all four derived vote
arrays from the original three point-cloud windows. The original sensor/pixel
formula reproduces allowance.04645728611061248 m; no constant is adjusted.
All inputs to this diagnosis are recorded public runtime/perception geometry,
not Gazebo GT. Details and a reproducible scene-local diagnostic are saved with
slot15. This establishes a sensing-irreparable lock for this initial catalog,
not proved full navigation/manipulation feasibility of source624, and not a
claim that all scenes or all methods necessarily lock. Slot16's separate
perception retains one combined-clear candidate, with15/107 ground cells
supported at its final window; its own outcome is not reclassified.

### Research decision required; no further formal activation

No slot17 or later is started. SIM is shut down. Current retained sample is
**16/560 valid slots:3 retrieval successes,13 method failures,0 INVALIDs**.
Only the original three primary scene pairs are complete; Hard-002 is
incomplete. All39 saved same-state score identities pass. No interim p-value,
final inference, sample adaptation, seed replacement or efficacy claim is made.
The original formal configuration/protocol and all historical results remain
unchanged; this pause is documented here, not by rewriting the freeze.

The narrow proposed research revision is to anchor A3 footprint support to
the **same original exact validated per-cell winner** already retained by A5,
and use that pose consistently for operational projection/confirmation. Keep
the0.10 m grid, R values, original candidate catalog/order, A2, positive target
association, all allowances/padding, AMBIGUOUS/ENVIRONMENT blocking, genuine
continuous target collision and real ground-support votes unchanged. This
would remove the additional snapped-representative veto, but would also change
A3 nominal/operational support masks, U_task and consequently viewpoint gains
and possibly rankings, even for some poses where both versions are clear.
Unchanged formulas/constants do not mean unchanged method behavior. It is a
change to frozen A3/A5 pose-support semantics and is **not implemented or
authorized as an ordinary engineering fix**. The shared anchor must apply to
all five environment methods (Fixed, Generic, Ours and both Hard ablations),
not only the primary pair; RM4D-only retains its existing path. No improvement
or general elimination of ambiguous-evidence deadlocks is claimed in advance.
Any approved revision requires its own regression/readiness and prospective
experiment-version decision; these16 v1.1 outcomes must not be overwritten,
reclassified or silently pooled with a revised method.

Checkpoint verification: root reran the saved-state reconstruction successfully
(102 exact/102 representative assessments,36 A3 arrays,15 A2 arrays,12
operational arrays and3 scoring/action snapshots);47 report/table/formal-design/
analysis tests pass. A second independent review agrees with the scientific
pause and the above qualifications; no remaining material findings. Research
source/A5/assets/Pilot-1 files still match approved v1.1, SIM worktree is clean,
and owned ports11951/11952 have no listener. No active simulator or bag reader
is left running. The diagnostic figure and reproduction command are in
`outputs/a6/formal/slot-015-hard-002-generic-01/structural-gating-diagnosis.md`.
