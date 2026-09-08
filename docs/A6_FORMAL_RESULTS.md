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

Current completed method sample: **4/560**, zero INVALID activations. The next
fixed block is Moderate-001, beginning with its method-independent setup. The
prescribed final inferential analysis remains unavailable until all planned
valid slots complete. Progress is tracked in Draft PR#6; main is not merged.
