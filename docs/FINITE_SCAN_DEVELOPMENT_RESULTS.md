# Finite-scan acquisition: bounded development results

Development only, 2026-09-10. This batch is not a formal test or a basis for a
statistical weighting-advantage claim. Historical outcomes remain unchanged.

## Changes and evidence

The new common predictor uses the selected SIM's installed MID360 ray schedule,
publication guards and actual five-second window. Its nominal horizon is 51
packets and its opportunity is averaged over the 80 cyclic packet starts. It
intersects actual rays with the 0.10 m half-open ground grid rather than testing
only cell centers against a rectangular angular envelope. The shared XY lattice
now has 1 m spacing within the previous ±2 m extent; no observation budget or
flight-cost change is involved.

The predictor also uses existing local operational evidence consistently:
perceived target OBB, complete AMBIGUOUS 33 mm disks as one-metre cylinders, and
full-cell prisms for ENVIRONMENT or missing history. This changes prediction,
not raw A2 evidence, collision/confirmation gates, or the requirement for two real
ground-support votes. Unknown remains transparent. Opportunity is not a return
probability and never becomes a sensor observation.

Four historical Hard second-window replays reduce support-union FP/FN from
0/27, 18/10, 0/13, 0/18 to 0/21, 0/5, 0/9, 0/10 respectively. These are same
recorded observations under different predictors, not new retrieval results.
See [independent geometry/replay review](../outputs/development/finite-scan-analysis/OPERATIONAL-OCCLUSION-REVIEW.md).

## Online record

Start cap is six, including failures. The predeclared order is in
[the run plan](FINITE_SCAN_DEVELOPMENT.md). Starts 2–5 share AGENT `6cb5f0c`
and SIM `a0ae8e3`; no production changes are made within those pairs.
Start 1 is a pre-model execution baseline at `73902d9` and is not pooled with them.

| Start | Scene / method | Windows | Confirmed | D_exec | Retrieval | First failure |
|---|---|---:|---:|---|---|---|
| 1 | Natural / Ours, pre-model | 2 | 4 | yes | yes | — |
| 2 | Hard01 / Generic | 2 (third capture not begun) | 0 | no | no | measured capture anchor outside flight bounds |
| 3 | Hard01 / Ours | 3 | 3 | yes | yes | — |
| 4 | Moderate01 / Ours | 2 | 2 | yes | yes | — |
| 5 | Moderate01 / Generic | 3 | 6 | yes | yes | — |
| 6 | Natural / Ours, `66c20aa` | 2 | 3 | yes | yes | — |

All six authorized starts are complete; there are no infrastructure replacement
trials or extra launches. Every attempt remains `VALID_TRIAL` as originally
recorded. Start6 is a separate post-repair regression, not a replacement or a
reclassification of start2. No human control/pose selection was used in any task.

Start 3 autonomously selected source623, navigated, obtained fresh near-field
D435 geometry, replanned on actual arrival, grasped, lifted and held. Independent
physical checks measured 148.661 mm object rise and 149.409 mm TCP rise. This is
the first complete physical task with the new finite-scan model; no station was
provided manually.

The Hard pair does not isolate a weighting advantage: Generic ended on a common
runtime boundary defect, while Ours completed. Do not treat its shorter failed
run as higher efficiency. Missing sampled-path distances remain missing.

All three paired new-model tasks that reached Ground handoff completed navigation,
fresh refine, D_exec, physical closure, loaded lift and short retention. The
Moderate tasks both selected source582 from their own new perception, not a stored
successful pose; object rises were 149.233 mm (Ours) and 148.684 mm (Generic).
Every task that reached execution screening accepted its first candidate, so inter-candidate
fallback is **not newly validated** here. Camera fallback did occur naturally:
Moderate Ours used view1 after current/view0 observation timeouts. A successful
execution preview still does not guarantee direct camera visibility.

Simulation-time measurements (active span includes computation and execution
screening, not just airborne translation):

| Task | Windows / actual NBV flight actions | First confirmation, active s | Active s | Task s | Active distance m |
|---|---:|---:|---:|---:|---:|
| Hard01 Generic | 2 / 2 | not reached | 56.142 | 102.956 | 5.618 |
| Hard01 Ours | 3 / 2 | 77.533 | 94.505 | 253.606 | missing; observed lower bound 6.336 |
| Moderate01 Ours | 2 / 1 | 53.608 | 69.526 | 268.776 | missing; observed lower bound 3.899 |
| Moderate01 Generic | 3 / 2 | 72.621 | 86.531 | 271.362 | missing; observed lower bound 5.638 |

The Moderate pair has a descriptive two-versus-three-window difference, but is
one reused development scene. No causal/statistical advantage or precise distance
advantage follows. Different closed-loop trials naturally have different sensed
states; both gains were additionally checked on each **same saved state**.

## Prediction versus actual support

Across the six completed new NBV-to-observation transitions, old center-model
support-union FP/FN total 4/69 cell-windows versus 4/43 for the new shared model.
These repeated cell-windows are not independent statistical samples. Every actual
presence increment reproduces from original retained endpoints and public TF;
all saved selected-view opportunities replay exactly. In Hard Ours window3 the
166-cell viable union all receives real ground hits, old FN5 becomes new FN0.
Moderate Ours window2 similarly has 159/159 real union hits and FN3 becomes FN0.

The remaining four nominal-positive misses occur in Moderate Generic. Replaying
actual packet transforms removes precisely the missing cells (506 in window2;
542/548/701 in window3). This supports realized pose/scan geometry as their cause,
not lost counter updates. Future pose/phase is not known to the algorithm. The
observed windows contain 51–52 consecutive packets; nominal prediction stays51.
Other remaining misses primarily reflect conservative known-occluder geometry.
Whole-grid prediction is not perfect or uniformly improved: Moderate Generic
window2 FP increases38→41 while FN decreases159→112. The new model is neither
calibrated nor a guarantee of repeated whole-footprint support within three views.

See [same-state diagnosis and plots](../outputs/development/finite-scan-analysis/new_task_findings_01.md).

## Remaining support semantics

Final blocked winner counts are30/36 (Hard Generic),31/36 (Hard Ours),30/35
(Moderate Ours),28/34 (Moderate Generic). Every such winner intersects the perceived
expanded target rectangle; many additionally intersect retained ambiguity disks.
No final winner is blocked solely by coarse environment or missing-history cells.
Clear but unsupported candidates remain unconfirmed; no vote is manufactured.

The existing diagnostic still finds blocked-winner/clear-non-winner alternatives:
Hard source543 has two unsupported alternatives for Generic and three supported
ones for Ours; Moderate Ours source539 has one alternative missing15 ground cells.
None was reselected or executed. Both Ours tasks already had other confirmed
winners. Hard Generic still had six clear but unsupported winners before its
capture-bounds failure. These records do not show another structural deadlock or
justify multi-support changes.

## Boundary defect diagnosis

Start 2 requested `(-3.346690, -2.959591, 1.153080)` m. Arrival accepted
`(-3.321880, -3.018693, 1.176819)` m: position error 68.353 mm is within the
existing 100 mm tolerance, but measured y is 18.693 mm outside the flight bound.
Arrival omitted the in-bounds conjunct that acquisition already enforces.
HOVER latches the current position, so merely waiting after HOVER is insufficient.
Capture then aborted before its existing bounded readiness dwell.

The minimal repair is to require measured bounds during the existing commanded-
position arrival dwell before HOVER, and let capture's existing readiness dwell
handle a transient out-of-bounds first sample. Neither bounds nor position/yaw/
speed/freshness/duration thresholds should be relaxed. No new candidate inset,
retry flight, ground vote, or extended deadline is justified by this diagnosis.
The original result is retained. Post-failure landing data cannot prove that the
original FLY_TO controller would have recovered if allowed to keep tracking.

The two-site adapter repair is committed as `66c20aa`. Seven focused regression
failures against the old code become passing tests: recorded bounds conflict,
before-HOVER ordering, interrupted dwell, unchanged deadline, transient recovery
and persistent rejection without new NPZ/votes/stamp changes. An independent
review reproduced old failures and passed the new 72-test adapter suite. The fix
does not change the controller's HOVER semantics or add an action fallback.

Start6 then ran the complete natural task at this exact code version. The program
selected source624 at `(2.474273,-0.470448,-pi)` from this run's perception.
Independent checks passed fresh aerial/near-field observations, navigation,
physical grip and loaded lift/short retention: object rise149.039 mm, TCP
rise149.786 mm. It used two windows/one NBV flight action, active67.481 sim s,
task222.895 sim s; active path remains missing with observed lower bound2.956 m.
This validates latest full-task integration, **not** a new online Hard-boundary
recovery comparison and not a guarantee that the original Hard task would succeed.

## Verification, delivery and next step

Final production regression:177 AGENT ROS/adapter/entry/metrics tests,130 modern
numerical/policy/geometry tests, and74 unchanged SIM execution/observation tests
passed. Both the finite model and the small boundary repair received independent
specification/code review. Tests support—but do not replace—the robot results.
All10 new paired-state score snapshots reproduce gain/cost/order identities
exactly; Generic/Ours argmax differs in8/10 states. This is same-state arithmetic,
not a claim that divergent trajectories share later observations or prove an
effect. The two natural runs' four snapshots are kept separately.

Runtime versions: paired tasks AGENT `6cb5f0c`, latest natural AGENT `66c20aa`;
SIM `a0ae8e3` (previous installed execution stack plus direct-source checker);
RM4D baseline `e9d431299053f38a4a4319aed3dfeccc261b9fac` and the unchanged independent
Ground task asset. Current config remains `sensor-retrieval-finite-scan-v1`,
v1.4 operational evidence/exact winners, five-second windows with cap3, original
flight cost and physical constraints,12 mm development execution allowance,
explicit integrated SIM feedback with native diagnostics retained. SIM has no
working-tree change in this batch. No main merge or formal matrix is performed.

Compact per-run fields, association totals, raw/operational blocking, candidate
choices, stage outcomes, paths and same-state scores are in
[task-results.json](../outputs/development/finite-scan-analysis/task-results.json).
Original observations/events/decisions are under
`outputs/development/finite-scan/`; large bags/native CSVs remain local under the
existing ignore rules. The four older untracked A5 directories are untouched.

**Recommendation:** stop adding acquisition/gating features now and move to
fixed-version small paired evaluation, without resuming the old formal matrix.
The shared Ground chain is not systematically blocking the tested tasks, and
the known arrival/capture contradiction is corrected. The next evaluation should
assess this implementation rather than tune it to ensure wins, including remaining
sensor-limited and near-boundary behavior. It must not pool these reused development
scenes with an independent final set.

Remaining limits are explicit: nominal hover versus actual drift; conservative
assumed occluder height/extent; finite repeated footprint support; a known object
class/initial search region; finite camera/IK alternatives; and occasional missing
trajectory samples. Keep distance unavailable/lower-bounded until actually measured;
this batch cannot establish calibrated acquisition, general reliability, precise
distance savings, or statistical superiority. No newly observed structural
support/handoff deadlock justifies another method revision at this checkpoint.
