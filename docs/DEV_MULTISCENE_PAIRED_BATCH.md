# Latest-version paired development batch

## Scope and fixed plan (before any method run)

Evaluate the just-passed natural aerial-to-Ground implementation, not a new
method, formal test set, effect-size study or restart of the old560 matrix.
AGENT baseline `6dfef58e54401f81188ff8d62b126e8dae43d961`, SIM
`e4e4f692f1524ded98e78abb82e39fd2dc7a86d2` (installed runtime code `fbb191b`).
Work in the user-specified AGENT checkout on `feature/dev-multiscene-paired`;
preserve existing untracked A5 logs and all historical branches/results.

**12 planned natural tasks; at most4 additional online starts**, including
failed startup invocations. Each task starts with normal P450 perception and
current real MID360 data. No Ground replay, conditioned arrival, archived
confirmation, candidate override or manual station. Serial runs on the existing
11951/11952 masters avoid competition between new trial worlds. Do not stop
after normal collision, planning, sensing-budget or grasp failures. No retry
until success. No formal runs.

## All scenes and order frozen together

Serialized configuration: `configs/dev_multiscene_paired.json`. Seeds are the
first six eligible draws of `Random(2026090921).randrange(1,2**31)`, excluding
every seed in baseline `configs/*.json`; no collisions/redraws occurred. Assign
replicate1 then2, each Easy/Moderate/Hard. Per seed copy the existing six draws:
target x=2+U(-.15,.15), y=U(-.15,.15), yaw=U(-pi/6,pi/6); BUNKER
x=3+U(-.1,.1), y=-2.5+U(-.1,.1), yaw=pi+U(-pi/36,pi/36). Tier obstacles,
target/BUNKER z and model properties are copied unchanged from the current
development config. No score, candidate, outcome or desired-mechanism filter.

| Slots | Scene | Seed | Order |
| --- | --- | ---: | --- |
| 1–2 | Easy01 | 325746326 | Generic, Ours |
| 3–4 | Moderate01 | 1460872549 | Ours, Generic |
| 5–6 | Hard01 | 1973399495 | Generic, Ours |
| 7–8 | Easy02 | 2001349247 | Ours, Generic |
| 9–10 | Moderate02 | 1293116599 | Generic, Ours |
| 11–12 | Hard02 | 1301813125 | Ours, Generic |

Each tier uses both first-method orders. Offline necessary RGB-D admission
only; no additional online setup activations are scheduled. Runtime setup
failures remain recorded. Seed freezing does not control unseeded Gazebo,
scan or OMPL timing; these are six development pairs, not inferential evidence.

## Common runtime and physical contract

Keep baseline initial map view(-1.4,0,1.2,0), launch(-.5,0,.15,0), 4m RGB-D
gate, three5s windows, original stop rule,20s capture/1200s task wall guards,
flight weight.25 and all shared config values. Both methods share v1.4 real
ground-presence confirmation, exact-winner support, v1.1 target allowance
.04395m, endpoint-local AMBIGUOUS blockers, A2 raw counters/thresholds, visibility,
cost, candidate ordering and selection. Only NBV gain weighting differs.

Run all tasks with `--integrated-joint-velocity --full-robot-manipulation
--execution-clearance --ground-dynamics`. Keep native diagnostic quantities;
no filtering of real motion. Preserve12mm development chassis allowance,
whole-robot/hand/perceived target/attached-load checks, actual contact/closure,
lift/retention thresholds, and bounded shared execution preview of at most4
already-confirmed exact winners. D435 and actual-arrival revalidation remain
mandatory. Record naturally triggered candidate/IK fallback; absent fallback
is not online validation of it.12mm is not a calibrated safety guarantee.

## Execution, reserves and version consistency

1. Check clean tracked baseline, existing runner/selection/metrics tests and
   method-independent optical-depth bounds. Commit this plan and all scenes
   before activating slot1.
2. Execute slots1–12 in order, each in its own directory under
   `outputs/development/multiscene-paired/`. Read outcome/first failure before
   the next slot, without pausing for routine approval or tuning a method.
3. Reserve starts require a written specific infrastructure invalidity or
   reproducible interface/runtime diagnostic; record the allocation before
   invocation. Preserve every earlier record, never overwrite/reclassify a
   valid failure to improve results. Ordinary failures consume their planned
   task and need no rerun.
4. No code change is planned. If an actual engineering bug requires a minimal
   fix, document affected versions/runs. Compare only same-code/config pairs.
   If a pair is split by a fix, use justified reserve activations for a complete
   same-version pair if budget permits; otherwise mark it non-comparable. Do
   not pool versions or silently replace old outcomes. Metadata/report-only
   commits are distinguished from executable changes.
5. Summarize actual observations → confirmation → parking → D435 refine →
   executed collision-aware pregrasp(D_exec) → physical grasp → lift/retention.
   Keep missing values missing, failed/not-reached separate, and record human
   intervention, windows/moves, first discovery, simulation time/distance and
   incomplete-path lower bounds. Fast failure is not efficiency evidence.
6. For Hard, compare existing predicted visibility with actual MID360 ground
   support over windows, without extra budget/votes. Review same-state
   Generic/Ours diagnostics and shared gate/screen behavior. Classify defects
   versus sensing/budget/geometric limits without assuming either method wins.
7. Run relevant regressions, independently review outputs/claims, record every
   launch and unused reserve, commit/push a Draft development checkpoint, stop.

Algorithm inputs never use Gazebo GT. Existing native/physical checker logs
may use simulation state only for offline motion/success diagnosis. No new
benchmark, audit, evidence, approval or mission framework.

## Launch ledger

At prospective freeze:0/12 tasks,0/4 reserve starts. All future invocations and
their original outcomes will be listed in `DEV_MULTISCENE_PAIRED_RESULTS.md`.
