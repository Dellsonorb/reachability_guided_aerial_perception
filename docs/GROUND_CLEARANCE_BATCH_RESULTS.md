# Ground clearance development results

Status: **complete, stopped at 5/6 Gazebo starts**. No sixth start or formal
experiments. Earlier Ground manipulation results remain preserved without
reclassification. Latest natural E2E passes with program-selected station;
this is development validation, not a reliability estimate or policy comparison.

| Start | Scope and version | Robot result | D_exec → retrieval |
|---|---|---|---|
|01|Natural Easy/Ours, SIM625b84c|Confirmed and parked; stale startup TF stops before camera|no → no|
|02|Arrival-conditioned source624, SIM1f47fe4|Camera/refine pass; six bounded IK branches reject before pregrasp|no → no|
|03|Arrival-conditioned source664, SIM1f47fe4|Pregrasp, descend and real grip pass; checking timeout before lift command|yes → no|
|04|Same source664, SIMfbb191b transport correction|Physical grip, payload-aware lift and bounded retention pass|yes → yes|
|05|Final natural Easy/Ours, SIMfbb191b|Normal aerial loop and program-selected source585 through physical lift|yes → yes|

Allfive are `VALID_TRIAL`; zero startup-invalid/reclassified attempts. Start04
is the one predeclared engineering contrast, not a repeat-until-success loop.
AGENT execution code is `7da6fde` for starts02–05. Source624/664 in local runs
are archived station conditions; only01 and05 are natural complete tasks.

## Natural latest-code reference

`launch-01-natural-latest`: AGENT `7320b9e`, SIM `625b84c`, Easy/Ours,
integrated-pose SIM feedback and full-robot manipulation explicitly enabled.
No replay, candidate override, or manual station. Original result:
**VALID_TRIAL / retrieval failed**.

- Normal P450 perception and three MID360 windows produced confirmed counts
  **0 → 1 → 5**. Two NBV moves occurred. Active flight distance3.25856m and
  active interval43.507sim seconds; total UAV distance11.0870m.
- Program selected exact winner source585 / candidate-000006 at
  `[2.430140806, -0.621187969, 2.268928028]`.
- BUNKER navigation and parking completed in35.077sim seconds. Actual arrival
  `[2.417332605, -0.677964202, 2.199209587]` passed the unchanged arrival checks.
- Refine failed before camera movement: `target planning-frame transform is
  stale`. Therefore **confirmed=yes → D_exec=no → retrieval=no**; the latest
  contact-height and payload checks were configured but not reached. This run
  cannot demonstrate latest integrated grasp/lift completion.

The recorded public TF chain remained fresh: age17ms at failure, dynamic gaps
at most21ms. Native replay plus installed startup/TF behavior support delayed
subscriber processing during MoveIt construction, rather than missing platform
localization. SIM `157f296` adds bounded freshness waiting, preserving the500ms
freshness threshold and original500ms wait budget. The exact stale cached stamp
inside the failed process was not logged. Review added a delayed-wake deadline
test; fresh data arriving after the budget is still rejected. Local starts02–04
and final natural05 exercise the correction without recurrence.

## Independent clearance basis

Prior launches05–08 contain26,683 paired desired/actual arm samples. Maximum
corresponding collision-box point displacement due to arm tracking is11.490625mm;
the common planning-only chassis allowance is **12mm**, rounded up at1mm.
Maximum TCP error11.266851mm. These are development observations, not a
calibrated bound for every branch/load or sensing error.

Native MoveIt reproduces source624's old planned0.404698mm finger/chassis gap
and its measured model collision. A copy of the authoritative chassis box
expanded12mm rejects that old plan before motion. Checked home and observation
endpoints remain valid. Only root-rigid structural links are exempt from this
planning copy; arm-mounted camera/pads and prospective payload are not.
Physical robot geometry, true collision checks and actual grasp constraints
remain unchanged. Other collision pairs do not acquire a claimed positive margin.

AG95 arm common-mode motion cancels internally; a global24mm self-clearance
requirement is not justified. Real loaded master articulation reached0.521260rad,
so the new prospective closure/load check extends to0.522rad without changing
the0.70rad force goal or replacing real grasp confirmation. Ground perception
repeatability is not a calibrated pose covariance and is not used to invent a
general target inflation.

Diagnostics and reproducible scripts are under
`outputs/development/ground-clearance-batch/offline-{errors,moveit,tf}`. Native
bags and physics CSV remain local diagnostics only, never algorithm input.
The completed local and final natural results follow.

## Local source624: rejection before dangerous approach

Start2, AGENT `7da6fde` / SIM `1f47fe4`, conditions on archived confirmation
and exact arrival; it is not a natural aerial result. Initial scene TF succeeds,
the actual checked camera trajectory succeeds, and fresh D435 refines the brick
to `[2.058624841, 0.079334934, 0.058551287, 0.055369519]` in map. The finite
two-yaw/three-seed search rejects all six at collision-aware grasp IK (-31).
No pregrasp, descent, physical close or lift command is sent. The original
`VALID_TRIAL / retrieval_success=false` is preserved; D_exec remains false.
This demonstrates pre-motion rejection at the problematic station, not global
IK impossibility or proof that every rejection is caused solely by the 12mm
guard. Independent native replay establishes that the prior submillimetre
plan violates the new guard. Source664 is a separate predeclared local check.

## Source664: actual grasp, runtime checking failure and focused correction

Start3 (`1f47fe4`) accepts the first whole-chain branch and reaches actual
pregrasp/D_exec, descend and physical grip. Its loaded lift check exceeds60s
before any lift action, so retrieval remains failed. This is a new checking
throughput limitation, not recurrence of the previous wrist settling failure.

Installed rospy reconnects and queries the master on every nonpersistent
service call. The new path used it thousands of times per trajectory. Native
transport-only comparison reproduces300 connections vs1 and mean0.503ms vs
0.119ms with identical responses; this diagnostic explicitly bypasses master
lookup and does not measure full MoveIt computation. SIM `fbb191b` reuses only
the serialized read-only validity connection. Errors still propagate; no
geometry/sample/timeguard or physical acceptance rule changes.

Start4, the one declared engineering contrast, completes the whole local chain:
target/TCP rise **149.139/149.731mm**, retained fresh grasp, actual endpoint
validity and independent physical checks pass. Actual loaded-lift command
checks **18,152states** before execution. Camera planning-to-checked interval
falls from23.106s/7,846states to3.408s/8,941states; loaded checking plus planning
takes11.28s before command, below the unchanged60s check guard. Different fresh
perception and stochastic MoveIt trajectories mean this is not an identical
trajectory timing benchmark. Source3 and4 pregrasp paths themselves differ;
do not attribute all total-time change to connection reuse.

Both local runs condition on archived source664 and arrival. They cannot
demonstrate autonomous aerial station choice. Final natural start5 supplies
that separate test; it does not repeat this known station.

## Final natural E2E: program-selected source585 through physical lift

`launch-05-natural-clearance`, AGENT `7da6fde` / SIM `fbb191b`, uses normal
P450 aerial perception, actual MID360 observations and A1–A4 processing. No
archived handoff, candidate override or conditioned arrival. Three completed
windows (two NBV moves) yield **0 → 0 → 5** confirmed candidates. The first
ranked candidate passes the common execution preview; no online candidate,
yaw or seed fallback is demonstrated. The bounded fallback is unit-tested.

Program-selected exact source585/candidate-000006 is
`[2.425755006, -0.623608986, 2.268928028]` in map. Actual parking is
`[2.483541970, -0.630380201, 2.335430674]`: XY error58.182mm and yaw error
0.066503rad pass the unchanged60mm/0.08rad checks. The accepted D435 target is
`[1.911143038, -0.123858335, 0.058586495, 0.220424220]`. Actual station/target
revalidation accepts the first measured-seed, zero-symmetry-offset branch.

The same task actually exercises checked camera motion, contact-configured
grasp height (+13.290562mm from established linkage geometry), full-robot
pregrasp, descend, seven sampled closure states, fresh physical grasp, measured
TCP payload attachment, and **20,688 loaded lift states** before dispatch.
Measured endpoints remain valid. Physical checker reports `CHECKS_PASS`:
**target rise148.951mm, TCP rise149.480mm**. Independent native trace reports
maximum target-in-hand displacement2.572mm during the whole lift stage and
0.00965mm over the original0.829sim-second retention interval. This is bounded
retention, not prolonged holding or a calibrated robustness guarantee.

The physical close is not ordinary gripper action success: in05 the recorded
close action reports `ABORTED/-4`, path error0.080112rad, while the existing
fresh true-contact plus minimum-closure gate accepts the real contact stall
at0.4706rad. No arm action abort is accepted; allfour arm motions report success.
This distinction and the original gripper result are retained, not relabeled.

SIM integrated-feedback mode is explicitly confirmed. Both six-arm public
feedback streams match same-stamp preceding1ms native position increments
exactly. The wrist's independent relative-quaternion increment agrees; raw
native rate discrepancy remains recorded. Public wrist speed reaches0.3343rad/s
during the full lift window: genuine motion is not filtered/clipped away.
Gripper lacks an independent native1kHz column, as previously documented.
Physical model/link/target data are offline diagnostics only, never action input.

Efficiency uses simulation time: active52.418s/4.31638m, execution preview12.287s,
navigation37.420s, D435 refine14.422s, refined pregrasp20.211s, descend7.054s,
close1.316s, lift16.367s, retention0.829s; whole task224.333s. Total UAV/Ground
sampled path distances remain **unavailable** due to one/three missing samples;
13.1590m/2.34667m are lower bounds, not complete distances. The checker's1.96623m
Ground travel is endpoint displacement, not that path integral. Legacy metrics
`physical_success=null` is unchanged; physical success comes separately from
the original attempt and physical checker, not a rewritten metric.

## Remaining limits and review

- Source624 still has no accepted branch in the bounded tested search. Do not
  call every possible grasp impossible. No target cell, collision exclusion or
  physical-grip exception is added to rescue source624; the documented intended
  finger-contact pairs and root-rigid guard exemptions remain explicit.
- The12mm chassis allowance derives from finite previous tracking observations,
  not calibrated total sensing/navigation uncertainty or universal self/world
  clearance. This uses the existing authoritative collision model, not perfect
  physical geometry. Sampling at1ms is not continuous collision certification.
- Candidate preview is nominal and bounded to four confirmed exact winners;
  actual arrival/refinement still needs revalidation. This successful natural
  run does not establish alternate-candidate fallback reliability or population
  retrieval success, nor Ours superiority over Generic. Candidate selection,
  visibility, flight cost, updater and execution remain common; the intended
  generic-unknown versus task-weighted gain distinction is unchanged.
- Hard sensing/coverage and missing ground support are untouched and remain a
  separate unresolved line. No formal matrix, new scene selection, extra
  observation budget or benchmark infrastructure was introduced.

Fresh verification: core656 tests run (634pass,22environment skips); native AGENT187
tests and SIM127 tests pass (overlapping suites), plus independent native
geometry/controller interpolation, runtime-code and final-results review.
`p450-clean` builds/install pass. No pending batch runtime; ports11951/11952
clear, unrelated prior Gazebo and four old A5 output directories preserved.
Large bags/native CSV remain local and ignored. Compact observations/results,
diagnostic scripts and reports are versioned. Stop at the development checkpoint;
Git delivery is recorded below; no main merge.

## Git delivery

- AGENT results checkpoint `cf35d19`, integration code `7da6fde`, branch
  `feature/dev-ground-clearance`: [Draft PR9](https://github.com/Dellsonorb/reachability_guided_aerial_perception/pull/9),
  stacked on preserved `feature/dev-operational-consistency` / PR8.
- SIM checkpoint `e4e4f69`, runtime code `fbb191b` / `1f47fe4`, branch
  `feature/fix-ground-execution-clearance`: [Draft PR5](https://github.com/Dellsonorb/Simulation-Platform/pull/5),
  stacked on preserved `feature/fix-ground-manipulation-completion` / PR4.
- Both branches pushed; Draft status, historical branches/results and current
  workspaces retained. This small follow-up only records delivery links.
