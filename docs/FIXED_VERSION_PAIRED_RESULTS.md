# Fixed-version six-scene paired development results

2026-09-10. Development evaluation only; no final test or formal matrix.

## Outcome and fixed version

All **12 planned tasks** ran once in the predeclared order: **11 retrieval
successes and one valid sensing-budget failure**. No infrastructure retries,
extra starts, human control/pose selection, runtime repairs or parameter changes
occurred. The two-start reserve was not used. All twelve output directories have
their entry and finished attempt records; there are no omitted entry-only starts.
Earlier development and formal-interruption results remain untouched.

Runtime commits for all twelve: AGENT `0de63c5511b4732bb24090d7a75656eb2b752f1b`,
SIM `a0ae8e32889e92a86f24d2794c7cb143999915fd`, original RM4D
`e9d431299053f38a4a4319aed3dfeccc261b9fac` plus the existing independent Ground
task-domain asset. AGENT runtime is unchanged from `54aa4c7`, including the
arrival/HOVER fix at `66c20aa`; `0de63c5` adds only pre-run evaluation material.

The [pre-run plan](FIXED_VERSION_PAIRED_EVALUATION.md) and
[six frozen seeds / twelve ordered slots](../configs/fixed_version_paired.json)
precede every method run. Generic/Ours share the current finite-scan profile,
candidate/visibility/cost/budget/updater, exact-winner operational gate, Ground
selection and full execution. Keep three five-second windows, 12 mm development
clearance, explicitly configured integrated SIM velocity feedback with native
diagnostics, and full robot/gripper/perceived target/loaded-motion checks.
No scene or outcome was replaced and no GT supplied algorithm perception.
The no-human-control statement comes from this batch's serial launch history,
not the offline summarizer's constant `human_control_intervention` field alone.

## Six paired outcomes

Numbers are Generic / Ours. C is the final confirmed-candidate count. All
successful tasks complete navigation, fresh D435 refine, actual-arrival
collision-aware pregrasp (`D_exec`), physical grasp, lift and short retention.

| Scene | C | Windows | Sensing flight commands | D_exec | Retrieval |
|---|---:|---:|---:|---|---|
| Easy01 | 3 / 4 | 3 / 2 | 2 / 1 | yes / yes | success / success |
| Moderate01 | 1 / 1 | 2 / 2 | 1 / 1 | yes / yes | success / success |
| Hard01 | 1 / 3 | 3 / 3 | 2 / 2 | yes / yes | success / success |
| Easy02 | 6 / 6 | 3 / 3 | 2 / 2 | yes / yes | success / success |
| Moderate02 | 5 / 1 | 3 / 3 | 2 / 2 | yes / yes | success / success |
| Hard02 | 0 / 5 | 3 / 3 | 2 / 2 | no / yes | budget failure / success |

Primary paired binary counts: five both-success pairs, one Ours-only pair,
zero Generic-only pairs, zero both-failure pairs. Generic is 5/6 and Ours 6/6.
This one discordant development pair is worth independent validation; it does
not establish a statistical success advantage or its effect size.

The 11 reached Ground chains all complete: `confirmed → navigation → refine →
D_exec → grasp → lift/hold`. The failed Hard02/Generic does not reach any Ground
stage; those fields remain NOT_REACHED, not Ground execution failures. Successful
object rise is 148.757–149.227 mm. Retention stages last 0.698–0.836 simulation
seconds: this is the existing short hold check, not a long-duration load test.
Physical target measurements are external checks, never algorithm input.

Autonomous selected source IDs (Generic/Ours) are Easy01 706/706, Moderate01
624/624, Hard01 542/624, Easy02 583/583, Moderate02 665/705, Hard02 none/625.
Each is selected from that task's new perception and exact evaluated candidates,
not supplied as a manual or historical successful station.

## Efficiency without treating failure as fast retrieval

All times use simulation time. Active span includes sensing, movement,
computation and the common execution screen; it is not pure flight time.

| Scene | Active s, G / O | Whole-task s, G / O | Interpretation |
|---|---:|---:|---|
| Easy01 | 97.062 / 83.716 | 266.056 / 249.919 | Ours uses one fewer window |
| Moderate01 | 65.172 / 69.617 | 308.382 / 235.948 | Ground camera attempts dominate task difference |
| Hard01 | 85.410 / 90.301 | 251.857 / 257.470 | both succeed in three windows |
| Easy02 | 91.196 / 91.696 | 257.870 / 266.216 | both succeed in three windows |
| Moderate02 | 109.038 / 95.707 | 268.448 / 272.045 | shorter active span does not imply shorter task |
| Hard02 | 72.666 / 90.938 | 115.209 / 253.817 | Generic ends before handoff; not an efficiency win |

Across the five jointly successful pairs, mean active time is 89.576 s Generic
versus 86.207 s Ours; mean task time is 270.523 versus 256.320 s. These are
descriptive reductions only. Ours is faster in active time in two of those five
pairs, slower in three; it uses one fewer window only in Easy01. The whole-task
mean is strongly influenced by Moderate01: Generic spends 74.395 s in refine
versus Ours 14.513 s, accounting for 59.882 s of its 72.434 s task-time gap.
Do not attribute that complete gap to aerial task weighting.

All six pairs use 17 Generic versus 16 Ours windows (11 versus 10 sensing flight
commands, the raw `nbv_moves` field). These are not 21 spatial relocations:
slots 05, 06, 07, 08, 10, 11 and 12 choose a zero XYZ/yaw offset from the measured
UAV pose after window2, although the existing adapter records `rescan=false`.
Thus there are 8 Generic and 6 Ours nonzero-offset sensing choices, plus 3 and 4
zero-offset further-observation choices. Actual hover drift and flown displacement
are separate, incompletely measured quantities. This reporting clarification
does not alter the runtime actions, original counters or paired results.
The five jointly successful pairs use 14 versus 13 windows. First confirmation
times, every stage duration, lower-bound paths and exact selected poses are in
[the compact task records](../outputs/development/fixed-version-analysis/task-results.json).

UAV active/total distance is missing in the eleven successful tasks because
their trajectory records contain missing samples; observed lower bounds are
retained, not promoted to complete distances. Only the failed Hard02/Generic
has complete recorded active distance (5.532 m) and total distance to failure
(9.511 m). Its cleanup adds 1.435 m separately. There is no complete successful
distance pair and therefore **no measured distance-saving claim**. Cleanup
landing and any takeoff-to-cleanup-landing field are not successful task time.

## Failure and remaining operational limitations

The sole terminal failure is `VIEW_BUDGET_REACHED` without a confirmed exact
candidate, Hard02/Generic, seed 1339075492. It is `VALID_TRIAL`, retrieval false;
the platform launcher exiting zero only means it completed recording/cleanup.
The public task entry correctly returns failure. It is neither INVALID_TRIAL
nor a navigation/manipulation failure, and was not rerun.

Its final 35 exact winners contain 30 operationally blocked and five geometrically
clear winners. The best remaining real-ground deficit decreases 106 → 81 → 18
cells across the three observations, but never reaches zero. Raw grid blocking
remains 33/35 while operational blocking is 30/35. Thus even this failed run is
not stuck solely because every winner is represented as geometrically blocked;
real supporting observations are still missing. Source623's 18 deficient cells
all have one real support vote, not two; 15 have zero saved occluded opportunity
at both later selected poses. Generic's third choice is a zero-offset further
window. The paired Ours task confirms five and executes source625; no fourth
window is executed or invented.

[The full Hard diagnostic](../outputs/development/fixed-version-analysis/hard-report.md)
reproduces 12 actual endpoint-to-presence increments, eight saved selected-view
forecasts and 12 exact decisions across the four Hard tasks. In Hard02/Generic,
the viable 169-cell union gets 126 and 134 real ground hits in later windows,
versus 169 and 169 for its Ours pair (different actual poses and sensor states).
Generic retains five clear winners missing 18–29 cells each; its one diagnostic
clear non-winner still misses 21. No already-supported alternative was uncovered
by this diagnostic, which examines non-winners of geometry-blocked winners, not
all non-winners of clear-but-unsupported winners. Ours has a supported non-winner,
but also five confirmed winners already; no alternative is reselected.

The prediction is still approximate. Hard02/Generic retains 50 actual packets
per later window with one phase gap, versus 51 nominal packets. Replaying those
same phases with measured per-packet transforms removes its 7 and 2 nominal
positive/no-ground mismatches; it does not explain every no-ground-return cell.
Actual pose drift, conservative occlusion and exact-plane versus accepted ground
band boundaries remain explicit limitations. Across these eight Hard transitions,
old center-model FP/FN is 12/70 cell-windows versus saved finite-model 13/36:
fewer missed actual-hit cells, not uniformly better prediction or a calibrated
guarantee. No lost observation is imputed. See the
[failed task's third-window geometry](../outputs/development/fixed-version-analysis/hard_slot-12-hard02-generic_window3.png).

There are no terminal navigation, refine, collision/planning, grasp, lift,
startup/TF or recording failures in this batch. Nevertheless, near-field D435
observation at the initial arm pose fails in the successful runs and requires
the existing active camera-pose alternatives. Moderate01/Generic needs views
0, 1 and 2; this finite observation search remains a meaningful capability and
time limitation. Success of a grasp precheck does not guarantee camera visibility.

Every reached candidate screen accepts rank 1, and no inter-candidate fallback
is demonstrated here. Keep unexecuted alternatives diagnostic; do not describe
them as successful recoveries. The 12 mm allowance and sampled collision checks
are development safeguards, not a calibrated clearance or continuous safety
guarantee. Finite scanning, actual hover drift and conservative surrogate
occluders still do not guarantee whole-footprint repeated support in three views.

## Review, reproducibility and next step

All 33 saved shared-state rankings pass the gain/cost/order identities using the
same candidates and opportunity tensor. All 33 have differing Generic/Ours
argmax; this is not 33 independent trials, and includes terminal-state rankings
that do not cause another flight. Divergent closed-loop trials naturally have
different subsequent sensed states. Fairness means shared algorithms/settings,
not identical later observations. Candidate coordinates/counts can differ across
the two independently sensed trials; each same-state check uses exactly one
ordered candidate set for both scores.

Relevant unchanged regressions passed both before and after the batch: 174 ROS/
adapter/entry/metrics tests and 89 numerical/scan/geometry/policy tests. Independent
review checks seeds/profile/entry before execution and checks raw physical,
stage, trajectory and paired results afterward. No additional SIM code is changed.

Records live under `outputs/development/fixed-version/slot-*`; compact per-window
evidence, ranking, candidate decisions, sensor NPZs and events are retained.
Large native dynamics CSVs and diagnostic bags stay local under existing ignore
rules. `fixed-version-analysis/summarize_runs.py` and the existing
`scripts/a6_scoring_diagnostics.py` reproduce compact summaries. No new experiment
engine, safety/evidence framework or formal matrix is introduced.

**Recommendation: stop proactive feature development and define a formal
experiment protocol around this fixed implementation.** Within the tested known
brick class/search region, static horizontal ground, bounded UAV workspace and
the existing three obstacle templates, the common full Ground chain no longer
shows systematic execution failure. The remaining observed terminal limit is
finite-budget sensing, not a demonstrated software contradiction. This is enough
to plan independent evaluation, not enough to claim general reliability or Ours
superiority.

Keep primary paired E2E success and explicitly separated windows, active time,
execution time and missing-distance policy. The single discordant pair and small,
nonuniform timing effects justify further validation, not outcome-driven tuning.
Six nearby development scenes cannot characterize an arbitrary workspace or a
reliable effect size. Define task-domain coverage and freeze an independent test
set before seeing its method outcomes; do not choose scenes to manufacture more
discordance. If distance is a formal efficiency claim, its recording completeness
needs resolution or it must remain an unavailable secondary measure. This batch
does not select a formal sample size or start final experiments.
