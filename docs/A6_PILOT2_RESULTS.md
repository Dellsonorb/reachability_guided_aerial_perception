# Pilot-2 results — 14 slots complete, readiness review

No formal runs have started. Pilot-2 methods: **14/14 valid completed slots**,
with one additional INVALID method activation retained and successfully repeated.
Two retrieval successes and twelve valid failures are retained. Formal readiness
review is separate from this completed pilot. Protocol/seeds were committed before any method outcome in
`18fb57d`, with the serialized configuration and shared v1.1 switch in `ed87bd6`.
Pilot-1 and all frozen algorithm files remain unchanged.

## Preparation

- Nominal depth setup passes the necessary gate for all three seeds:
  Easy 3.319–3.585 m, Moderate 3.303–3.565 m, Hard 3.443–3.711 m.
  This does not establish actual FOV or observed support.
- Core regression: 433 tests, 427 passed and 6 OpenCV-only skipped; system
  Python `test_a5_target_support` passes all 11 including those 6.
- Independent review of `56ece4a..ed87bd6`: no important correctness/fairness
  findings; 39 focused tests and all six method CLI parses pass.
- System volume initially had only about 120 MB free. Moved the two prior
  project-owned temporary SIM review snapshots (not Git worktrees) from `/tmp`
  to `/media/lu/P450_PAPER/AGENT/runtime-scratch/`, preserving both; this freed
  about 600 MB. Bags go directly to the experiment volume and are Git-ignored.

## Setup activation history

`outputs/a6/pilot2/setup/easy-01`: no method started. The invocation omitted
the existing wrapper's display variables. Aerial RGB-D produced no frame and
`spawn_pick_target` reported its camera timeout at sim 35.808 s. Consequently
the common setup could not find `pick_target`. This is a launch-environment
error, not scene rejection or a method failure. Keep the directory and repeat
the identical Easy seed with the correct public wrapper environment; no SIM
source/threshold/timeout change is needed.

Every subsequent invocation must pass the established rendering inputs before
the existing environment-cleaning wrapper, for example:

```bash
P450_PX4_ROOT=/media/lu/P450_PAPER/P450-PAPER/workspaces/dependencies/px4 \
P450_GAZEBO_DISPLAY=:0 \
P450_GAZEBO_XAUTHORITY=/run/user/1000/gdm/Xauthority \
/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation/scripts/with_p450_env.bash \
  /usr/bin/env ROS_MASTER_URI=http://127.0.0.1:11951 \
  GAZEBO_MASTER_URI=http://127.0.0.1:11952 \
  /usr/bin/python3 -u -B scripts/run_a6_attempt.py \
  --config configs/a6_pilot2.json --setup-scene easy \
  --output-dir outputs/a6/pilot2/setup/easy-02
```

Use the serialized slot instead of `--setup-scene` only after all three setup
checks pass. A new output directory is required for each activation.

All three live setup checks now pass, without changing any pose, seed or gate:

| Setup | Window (sim s) | Ground cells | Mean ground z (m) |
|---|---:|---:|---:|
| Easy `easy-02` | 5.030 | 856 | 0.0000385 |
| Moderate `moderate-01` | 5.088 | 559 | 0.0000261 |
| Hard `hard-01` | 5.032 | 396 | 0.0000560 |

Each obtained an accepted aerial target and available v1.1 runtime perception
reference. No setup executed RM4D or policy scoring. Common initial pose remains
map (-1.4,0,1.2), yaw=0 for all methods. Setup bags close normally and can be
read: Easy 59,662 / Moderate 57,545 / Hard 61,131 messages, including RGB/depth,
controller state and TF. No overflow/drop warning was found. Unused action goal
and result streams correctly have no messages during sensor-only setup.

The mechanism reporter's independent review found two ordinary reporting bugs:
truncated compressed NPZ aborted aggregation, and sensor-only setup records
were counted as method attempts. Both are fixed with reproducing tests: missing
NPZ stays unavailable without losing other outcomes; setup is excluded while
INVALID method activations remain. Focused rereview: 25 reporter/scoring tests
pass and no remaining actionable findings. Natural v1.1 fixture still reports
two confirmed candidates and zero alias-retained exact candidates; this is not
presented as a rescue result.

Final pre-activation regression: 451 tests, 445 passed with 6 OpenCV-only skips;
the system Python's 11 target-support tests pass, covering all 6. The diff from
`56ece4a` leaves `src/`, A5 runtime/target-support code, and Pilot-1 configuration
unchanged. Setup, reporter and code review preparation gates are satisfied.

## Completed method activations and engineering diagnosis

| Slot / activation | Confirmation | D_exec | Primary retrieval |
|---|---|---|---|
| 1 Easy RM4D-only / 01 | N/A | yes | valid failure: retention confirmation |
| 2 Easy Fixed / 01 | 2 candidates at window 3 | yes | INVALID: checker startup loss |
| 2 Easy Fixed / 02 | 1 candidate at window 3 | yes | success: independent physical checks pass |
| 3 Easy Generic / 01 | 3 candidates at window 3 | yes | success: independent physical checks pass |
| 4 Easy Ours / 01 | 1 candidate at window 2 | no | valid failure: Ground navigation timeout |
| 5 Moderate Fixed / 01 | none in 3 windows | no | valid failure: budget without confirmed candidate |
| 6 Moderate Generic / 01 | none in 3 windows | no | valid failure: budget without confirmed candidate |
| 7 Moderate Ours / 01 | 1 candidate at window 2 | no | valid failure: Ground navigation timeout |
| 8 Moderate RM4D-only / 01 | N/A | yes | valid failure: lift Cartesian execution check |
| 9 Hard Generic / 01 | none in 3 windows | no | valid failure: budget without confirmed candidate |
| 10 Hard Ours / 01 | none in 3 windows | no | valid failure: budget without confirmed candidate |
| 11 Hard RM4D-only / 01 | N/A | no | valid failure: Ground navigation timeout |
| 12 Hard Fixed / 01 | none in 3 windows | no | valid failure: budget without confirmed candidate |
| 13 Hard no-occlusion / 01 | none in 3 windows | no | valid failure: budget without confirmed candidate |
| 14 Hard no-cost / 01 | none in 3 windows | no | valid failure: budget without confirmed candidate |

Slot1's four arm trajectories all report SUCCEEDED/error0. The accepted close
stall was followed by successful lift motion, then `AG95 lost grasp confirmation
during lift` at sim161.298. The recorded gripper remained above the closure
threshold. The missing grasp-confirmation Bool means false versus stale receipt
cannot be distinguished; neither physical slipping nor a platform defect is
proven. Keep the valid failure. Subsequent diagnostic commands additionally
record the existing `/ground/gripper/grasp_confirmed` and `/pick_target/contacts`
sensor streams; they are never new algorithm inputs and no gripper setting changes.

Slot2's adapter completed LIFT, but the checker saw `TAKEOFF` as its first state
and exited before physical measurement. Saved adapter events have the correct
initial sequence. Both ROS logs show duplicate connections to the same peer
during that burst. Isolated real-ROS reproduction with a diagnostic-only 60 ms
connection-replacement delay produced exactly `[TAKEOFF]` despite a live queue
size of10. This is not an actual method failure or a positive physical result.
The initial automatic classification is retained in `attempt.json`'s
`classification_review`; raw physical summary, metrics and observations remain.

Minimal A6-only fix: the adapter announces final construction on its existing
stdout log, and orchestration then starts the unchanged checker. A5/SIM code,
task status sequence, total wall guard and subscriber wait are unchanged. The
checker remains alive during adapter cleanup. Deferred subscription preserved
all four initial states in all6 focused replacement-delay cases and all8 prior
first-write-delay cases. Independent review and45 focused tests pass. Repeat
only slot2's INVALID activation with the same seed/method/settings.

The identical slot2 repeat passes all physical checks: brick lift .14773 m and
TCP lift .14948 m, with confirmation retained. It uses 3 completed windows,
2 stationary rescans, 36.606 s active time and 140.994 s task time (simulation).
Actual UAV hover travel is not zeroed: 1.50322 m active, 6.62501 m total.
The focused reproduction is retained outside the repo at
`../runtime-scratch/publisher_startup_probe.py` and
`../runtime-scratch/publisher_startup_probe_results.json`.

Slot3 Generic also passes the independent physical checker. It uses 3 windows
and 2 NBV moves; first confirmation occurs in window3, after 50.695 active
simulation seconds. Active time is 50.696 s, task time 164.352 s; UAV travel is
3.44632 m active and 10.83132 m total. Ground total travel remains unavailable
because its online trace contains missing samples. These are descriptive
records, not a population method-effect conclusion.

Slot4 Ours confirms one pose in window2 (41.649 active simulation seconds),
completes the frozen third window without first-confirmation early stop, then
hands off exact candidate000006. Ground approaches its XY but fails to converge
to the full navigation pose within the original 120 s guard. Keep the valid
navigation failure: D_exec=false and all refinement/manipulation stages remain
NOT_REACHED. Active time is 52.575 s, task time 229.549 s; UAV distance is
3.45344 m active / 10.88072 m total. No settings or outcome-based rerun change.
The first complete primary pair is **Generic-only success** (b=0,c=1,n=1).
Earlier confirmation is not equated with execution or retrieval success.

Slot4's passive bag establishes accepted/ACTIVE navigation, 1,800 matching
navigation/guarded commands and 6,000 continuous odometry samples (max gap29 ms).
Final XY/yaw errors are .06100 m / 1.12350 rad. The base briefly enters the frozen
.06 m XY tolerance, then stalls around its boundary without yaw convergence;
late commands continue rather than being dropped. Final local costmap is free,
and recorded contacts show the target on the ground plane, not a BUNKER contact.
This supports a controller/base convergence failure, not a demonstrated startup,
transport or independent integration defect. Keep the valid failure and settings.

Saved-only audit of the valid Fixed/Generic final windows finds respectively
29/33 and 31/35 exact operationally blocked candidates, zero target-alias-retained
exact candidates, and TARGET/AMBIGUOUS/ENVIRONMENT vote sums 15/17/0 and 11/16/0.
Cause categories overlap. Their selected exact poses each have 108 supported
cells, no operational blocker and no continuous target collision.
Both reject `candidate-000014` (source745) solely through the preserved A3
representative guard despite complete, unblocked exact geometry. Record this
disagreement; alternative poses completed retrieval, so it is not yet evidence
of a method-wide structural deadlock or a reason to change the frozen guard.
Reassessment identifies both continuous expanded-target overlap and two
AMBIGUOUS cells (780,821) at this representative, versus 40.55–41.04 mm exact
continuous clearance and 108/108 exact ground support in the final Easy windows.
Its exact raw occupied overlap is zero, so this is not a TARGET alias-retention
case. The representative has 105/107 ground-supported cells. Exact/representative
replay matches saved gates; only this candidate is lost among otherwise fully
supported exact poses (Fixed2→1, Generic4→3, Ours2→1).

Raw public TF remained available for slot1's missing resource samples; local
buffer initialization/backlog caused the holes. Slot2 also has small local
TF-versus-captured-clock races. Original path metrics stay unchanged/null where
incomplete. The separately labelled uniform causal bag-derived secondary report
is implemented and independently reviewed. Its 24 focused Noetic tests pass.
A review-found clock-reset edge case is fixed: any reset in original samples
or events makes the entire secondary trace unavailable before reading bag
messages, avoiding false epoch attribution. It cannot reclassify retrieval,
insert samples, or repair real scheduling gaps. Original metrics remain primary.

Easy-block checkpoint verification: core discovery runs 468 tests (452 pass,
16 environment-specific skips); Noetic's 24 reporter/metric tests and 11
target-support tests pass and cover those skips. All 12 saved sensing snapshots
pass same-state score/mask/cost identities. Four first-window scene descriptions
(including the INVALID measurement activation) match the Easy proposal targets;
these are post-freeze descriptions, never scene admission or outcome filters.
No `src/`, frozen A5 method, Pilot-1 protocol/configuration or SIM change is made.

## Moderate block

Fixed's three rounds have 35 exact candidates, 33 exact operational blockers,
34 representative/combined blockers and zero confirmation. Candidate000002
(source666) remains exact/representative-unblocked and unclipped, with no target
collision or occupied-class overlap. Its final missing support is **one cell781
with one ground vote**, while the other 105 footprint cells have three votes.
This is insufficient measured support within the budget, not all-candidate
operational blocking. Candidate000021/source745 is exact-clear but its
representative overlaps an AMBIGUOUS cell; it also lacks six exact support cells.
Do not turn either into a confirmed pose or change the two-vote threshold.

Generic completes all three windows and likewise terminates without handoff.
Its two combined-clear candidates (source666/745) each support105/106 cells,
missing only cell822's second vote. Two other exact-clear poses are excluded
by continuous representative-target collision, with no class overlap.

Ours confirms source666 in windows2/3 (106/106 ground, no exact or representative
blocker), then times out during 120 s Ground navigation. D_exec and retrieval
remain false. The Moderate primary pair is neither-success, despite different
failure stages. RM4D-only completes navigation/refinement/pregrasp/descend/close,
but fails during lift: `max_joint_speed=0.1659 joint=wrist_3_joint`. D_exec=true
is retained separately from retrieval=false. All four outcomes remain valid.
Native controller logs identify `GOAL_TOLERANCE_VIOLATED` at wrist3, followed by
MoveIt CONTROL_FAILED; the speed is an appended diagnostic, not a new AGENT
threshold. Lift planning itself produced32 points/100% completion. The controller
checks velocity as well as position while printing only position error, so its
printed .000032 rad does not establish a false rejection. Concurrent .1659 rad/s
exceeds the original .10 stopped-velocity tolerance. Exact state timing remains
to be cross-checked in the retained bag; no demonstrated integration defect.

The Fixed missing-support cell781 is raw UNKNOWN with one ground vote and no
occupied-class evidence. All65 generated viewpoints pass range, but elevation
excludes51/49/50 per round and the remaining14/16/15 rays all hit neighboring
cell780's mixed TARGET+AMBIGUOUS **1 m surrogate prism**. This explains saved
zero predicted visibility, not real physical impossibility. Generic's nearby781
and Ours'781/822 receive actual ground votes1→2→3. Independently perceived grid
origins differ by millimeters; these are nearby cells, not identical inputs.
Ours therefore demonstrates achievable handoff within the frozen budget.

Final Fixed/Generic/Ours class vote totals are TARGET15/12/15,
ENVIRONMENT108/101/106, AMBIGUOUS18/13/18. Exact blockers33/32/34 and
combined-clear1/2/1 are separate. No alias rescue is required or manufactured.
All21 saved Easy/Moderate snapshots pass shared scoring identities;19 select
different task/generic argmaxes. No post-outcome tuning or scene replacement.

Moderate/Ours navigation differs from Easy: it turns substantially on approach,
then oscillates near the XY tolerance, ending at .0602255 m / .186489 rad errors
without simultaneous .06 m / .08 rad convergence. Last33.64 s travel is12.2 mm,
with95.4 mrad cumulative yaw, rather than Easy's nearly stationary large-yaw
stall. The goal stays ACTIVE; the 120 s timeout cancels it to PREEMPTED.
All1,800 navigation/guarded commands match, 6,001 odometry samples remain
continuous, and the last pre-failure local costmap is free. Retain the observed
convergence failure; no independent integration defect has been demonstrated.

The post-freeze scene description measures Moderate task-support occlusion at
.173–.178, below the proposal's .25–.50 descriptive target. Report this mismatch;
do not reject/relabel the seed, move walls or select a replacement. The scene
comes from the original method-blind distribution; this finding preceded Hard.

## Hard block

Generic's35 candidates include three exact/representative-clear poses throughout
all windows: candidate000000/source582, 000010/source501 and000015/source500.
Their final ground-support deficits are respectively4/99,2/107 and4/103 cells.
All three are unclipped with zero occupied-class overlap or continuous target
collision. Thus the actual budget failure does not establish all-candidate
operational deadlock. Ours also completes the budget with no confirmation, but
retains two combined-clear poses, source582 (97/99 ground) and501 (103/106).
Its missing cells417/457 and576/616/656 have actual ground-vote histories0→0→1;
Generic's missing737/738/739/740/783 remain1→1→1. All are raw UNKNOWN with zero
occupied-class votes. This does not prove another window would succeed, but
does refute absolute sensing impossibility for Ours' missing cells.

Hard Generic blockers comprise27 expanded-target-collision+AMBIGUOUS,
3 collision-only and2 AMBIGUOUS-only; Ours has30 collision+AMBIGUOUS and
3 AMBIGUOUS-only. Neither has ENVIRONMENT-overlap or representative-only
blockers. Raw and operational blocked counts remain32 and33; alias retention
is zero. Collision refers to perceived expanded geometry, not GT-confirmed
physical collision. Final TARGET/ENVIRONMENT/AMBIGUOUS vote sums are6/82/5
for Generic and13/84/18 for Ours; five cells carry mixed target/ambiguous evidence.
The third primary pair is neither-success: overall b=0,c=1,neither=2,n=3.
These descriptive outcomes do not support an E2E superiority claim.

Hard Fixed and both predeclared ablations also terminate at three windows with
no confirmed candidate. RM4D-only reaches its original top-one navigation stage,
but times out without D_exec. No natural alias rescue is required or claimed.
The Hard scene description meets the original targets: task-support occlusion
.651–.688 across five saved sensing activations, with sufficient task-irrelevant
unknown by the original descriptive checks. This does not override failures.

## Simulation-time resources (original online metrics)

Windows count completed observations, including the initial MID360 window.
Moves exclude the initial flight and return; Fixed's two rescans are not NBV
moves. First confirmation is elapsed from active-phase start. UAV time is the
recorded UAV-total interval, ending at landing or terminal failure; post-failure
cleanup is separate, retained in JSON. Task time includes Ground/manipulation
when reached. Distances are actual trajectories, including hover drift.

| Slot | Method / tier | Windows / moves | First confirmed (s) | Active (s) | UAV active (m) | UAV total (m) | UAV time (s) | Task (s) |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| 1 | Easy RM4D-only | 0 / 0 | — | 0 | 0 | unavailable | 81.994 | 149.406 |
| 2 | Easy Fixed | 3 / 0 | 36.605 | 36.606 | 1.503 | 6.625 | 86.834 | 140.994 |
| 3 | Easy Generic | 3 / 2 | 50.695 | 50.696 | 3.446 | 10.831 | 111.969 | 164.352 |
| 4 | Easy Ours | 3 / 2 | 41.649 | 52.575 | 3.453 | 10.881 | 109.543 | 229.549 |
| 5 | Moderate Fixed | 3 / 0 | — | 35.417 | 1.420 | 5.439 | 77.689 | 77.689 |
| 6 | Moderate Generic | 3 / 2 | — | 51.706 | 4.271 | 7.983 | 92.771 | 92.771 |
| 7 | Moderate Ours | 3 / 2 | 39.486 | 50.276 | unavailable | unavailable | 111.845 | 231.853 |
| 8 | Moderate RM4D-only | 0 / 0 | — | 0 | 0 | 6.107 | 83.694 | 196.286 |
| 9 | Hard Generic | 3 / 2 | — | 52.674 | 4.151 | 8.084 | 91.950 | 91.950 |
| 10 | Hard Ours | 3 / 2 | — | 46.398 | 3.307 | 7.206 | 92.542 | 92.542 |
| 11 | Hard RM4D-only | 0 / 0 | — | 0 | 0 | 5.252 | 52.619 | 172.629 |
| 12 | Hard Fixed | 3 / 0 | — | 37.180 | 1.472 | 5.462 | 76.846 | 76.846 |
| 13 | Hard no-occlusion | 3 / 2 | — | 51.288 | 3.358 | 7.298 | 93.080 | 93.080 |
| 14 | Hard no-cost | 3 / 2 | — | 45.894 | 3.403 | 7.404 | 93.248 | 93.248 |

Unavailable online distances remain unavailable, not zero or successful-case
imputation. Uniform bag-derived resources are separate secondary reports; they
do not overwrite the table or any primary outcome. All environment methods use
three completed windows, so this pilot shows no observation-window advantage.

## Final classification and verification

Valid failure taxonomy: seven active-phase budget/no-confirmation failures,
three Ground navigation failures, one lift-controller failure and one retention
confirmation failure. Zero terminal refine/descend failures does not mean every
run reached those stages. D_exec occurs in slots1/2/3/8; retrieval only in2/3.

The sole method INVALID is slot2's missed checker initialization sequence,
followed by its identical successful repeat. The separate Easy setup launch
failure and its repeat remain outside the method denominator. No valid failure
is rerun or reclassified. First-activation-invalid-as-failure sensitivity leaves
the primary Ours/Generic paired outcomes unchanged (the INVALID was Fixed).

Slot8's full bag verifies ABORTED/error-5 GOAL_TOLERANCE_VIOLATED at208.231 s.
All200 controller samples in the final2 s exceed the original .10 rad/s stopped
velocity limit at wrist3 (actual -.2434 to -.1604 rad/s); position error stays
below .000100 rad. All147 lift-phase grasp confirmations are true. Joint state
at208.228 reproduces -.165902459 rad/s. This establishes terminal controller
velocity failure, not grasp loss; it does not independently diagnose its physical
dynamics root cause or justify modifying the controller.

Slot11's goal remains ACTIVE until its120 s cancel→PREEMPTED. Navigation and
guarded commands match across1,800 samples;6,000 odometry samples remain fresh.
It turns/drifts repeatedly (about17.26 rad heading travel), ending .094492 m /
.414176 rad from goal; no sample satisfies both frozen tolerances. Unlike the
other two navigation failures, its final local costmap retains obstacle cells.
A13.469 s local-plan publication gap does not imply lost navigation commands
or automatically make the observed timeout INVALID.

Final core regression:468 tests,452 pass with16 environment-specific skips;
Noetic's35 reporter/metric/target-support tests pass and cover those skips.
All36 saved scoring snapshots (including retained INVALID diagnostics and both
ablations) pass shared gain/mask/cost identities;33 task/generic argmaxes differ.
This is same-state scoring, not a claim that closed-loop methods visit identical
states. All12 scene-description snapshots remain descriptive, with only the
three Moderate occlusion-target mismatches reported above.

Machine-readable results: `outputs/a6/pilot2/summary-final.json`,
`mechanism-final.json`, `scoring-final.json`, `scene-description-final.json`.
Per-window PNGs and original JSON/NPZ observations are retained under every
sensing activation; native diagnostic bags remain local and Git-ignored.
The inherited A4 figure subtitle describes its predicted-view panels; these A6
upper-panel fields use actual saved SIM observations, not a synthetic cloud.

## Formal-readiness decision

Independent final review supports **GO to a separately predeclared formal
study**, not a claim that Ours is superior. It verifies all1,248 recorded exact
assessments against blocker/confirmation rules, and separately checks actual
Fixed/no-occlusion/no-cost decisions across nine windows and489 viewpoint
evaluations (including ablation masks, ordering and stopping, not only shared
ranking files). All match the frozen implementation. The last three slots retain
three combined-clear candidates each, with unmet actual ground-support votes.
No user-listed mandatory scientific stop condition is established.

Keep the **actual common SIM navigation configuration** for the next study.
The known latch namespace mismatch and replanning/latch interaction are a
documented limitation, not a reason to describe the platform as flawless,
erase navigation failures or tune successful-goal behavior. No causal repair
has been established here, and no mixed platform version may enter a primary
pair. Source/algorithm settings and success definitions remain unchanged.

All15 secondary bag reports reproduce original metrics and leave non-path
metrics unchanged. Among14 valid slots, online/secondary complete counts are
UAV-total12/14 versus14/14, UAV-active13/14 versus14/14, Ground-total7/14 versus
10/14. Actual scheduling gaps remain in slots1/2-repeat/3/8; no poses or samples
are invented to close them. Keep original online metrics primary for this pilot.

Next: prospectively define formal sample size, paired binary analysis,
independent seeds/order and resource reporting, then execute under the user's
conditional authorization. Neither Pilot-1 nor Pilot-2 will be pooled into formal
inference. No formal sample size has been selected or formal activation run at
this pilot checkpoint. Low pilot success is reported; it does not authorize
changing the method, sampling distribution, budget, primary endpoint or success.
