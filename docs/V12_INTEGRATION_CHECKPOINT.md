# v1.2 natural integration regression

Status: **v1.2 integration complete; stop for review**. This is an original
natural A5 regression, not a fresh-seed pilot or a formal statistical sample.
The historical16 v1.1 formal outcomes remain unchanged development diagnostics;
no slot17 is started. Exact-winner support and non-winner diagnostic-only
semantics remain frozen at9f3a527.

## Runtime defect and bounded repair

The first new activation is retained in
[`natural-attempt-01`](../outputs/a6/v12-integration/natural-attempt-01/physical_summary.json).
It passes the unchanged hover condition, completes three real MID360 windows
and two NBV flights, confirms exact source543 and navigates BUNKER. It fails
preparing the near-field D435 approach, before `GROUND_REFINED`, with:

```
Requested time222.406; latest TF222.405; map → ground/base_link
```

This is an activated failed regression, not INVALID_TRIAL or a revised pilot
classification. The prior9f3a527 activated hover failure is also preserved.
The new run's hover success does not establish a fix or root cause for that
prior failure; hover/controller settings are untouched.

The public TF bag shows `map → ground/odom` static and `/gazebo` publishing
`ground/odom → ground/base_link` at50Hz throughout the relevant interval,
including222.405,222.425 and beyond223.705. Exact222.406 is interpolable;
neither geometry nor TF authority is missing. MoveIt construction finishes at
C++ simulation time223.659 while Python's callback-fed clock/cache is behind.
The failure is in inherited `_observe_ground_target_rm4d`, not a refined-grasp
lookup: MoveIt initialization is followed by an approach pose stamped with
Python `rospy.Time.now()` and its exact-time transform.

A separate **no-motion** live probe records a1.2190923s Python heartbeat gap
across native MoveGroup construction. Python ROS time remains66.971 before and
after it while C++ reaches68.102; callbacks catch up after a wall yield. This
probe's stamp happened to be bracketed and its lookup succeeded; it demonstrates
callback starvation, not a second failed E2E. See
[measured probe summary](../outputs/a6/v12-integration/tf-init-diagnostic/summary.json).

Installed Noetic tf2 starts its timeout with `rospy.Time.now()`. A deterministic
native Buffer reproduction delivers the queued clock jump before the next TF:
the original0.5s ROS-time wait expires after a single polling sleep and produces
the same1ms extrapolation. Delivering222.425 makes the unchanged222.406 query
succeed. Thus a small timestamp gap is a callback/readiness race, not permission
to change timestamps or geometry.

The scoped integration repair uses a0.5s **monotonic wall readiness bound** for
pose-transform lookup, yielding between zero-timeout exact queries. Exact
requested frames/stamps and transformation math stay unchanged. Explicit latest
lookup for the inherited floor remains explicit; it is never a fallback for
an exact query. Missing TF still fails on deadline. No motion retry, replacement
candidate, threshold change, controller change or SIM source change is included.

Repair commit: `7745dc0`. Twelve focused tests cover delayed TF after clock
catch-up, deadline expiry under a frozen ROS clock, late arrival, transient and
non-transient errors, shutdown, exact/zero stamps, same-frame copy and explicit
latest-floor behavior. The native test first reproduces the old tf2 failure,
then obtains exact-stamp interpolation with the repaired wait. Independent
specification and code-quality reviews pass without material findings.

## Retained failed-run numerical replay

All three saved decision responses reproduce exactly from their original
MID360 arrays and runtime RGB-D reference. All43 final A2, operational, A3,
A4/ranking and decision checks pass. A4 uses the unchanged task-gain formula;
maximum numerical gain residual across windows is7.11e-15.

| Window | Confirmed | Blocked winners /33 | Positive operational cells | Task uncertainty mass | Non-winner cells / alternatives |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 0 | 28 | 175 | 175.000000 | 2 /3 |
| 2 | 0 | 30 | 162 | 99.044906 | 3 /6 |
| 3 | 1 | 32 | 106 | 38.995221 | 2 /4 |

The selected original winner is `candidate-000008`, source543, exact map
`(2.396419380206644,-0.6049564643790278,2.094395102393195)` with relevance1.0.
It has106/106 real ground-support cells and no operational blockers. Alternatives
are only geometric diagnostics: none is promoted, and `selection_changed=false`
throughout. The subsequent TF failure does not negate candidate confirmation
and does not establish execution-validated discovery or retrieval success.

Full per-window IDs and checks are in the
[derived replay JSON](../outputs/a6/v12-integration/natural-attempt-01-replay.json).
The accompanying read-only replay emits JSON to stdout and never alters original
observations/results:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src:tests:scripts \
MPLCONFIGDIR=/tmp/a6-mpl XDG_CACHE_HOME=/tmp/a6-cache \
/media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
outputs/a6/v12-integration/replay_natural.py \
outputs/a6/v12-integration/natural-attempt-01
```

## Verification before repaired natural activation

- [Core suite](../outputs/a6/v12-integration/core-tests.log):550 tests,
  22 environment-dependent skips, no failures (80.206s).
- [Native Noetic split](../outputs/a6/v12-integration/noetic-tests.log):87 pass
  (3.420s), including all12 native TF tests.
- [Actual RM4D/task-map tests](../outputs/a6/v12-integration/task-map-tests.log):
  nine pass (1.679s). These suites overlap; counts are not added.
- Source-boundary diff against9f3a527 is empty for all research modules, assets,
  configs and original Pilot-1/Pilot-2/formal/v1.1/v1.2 regression outputs.
  SIM remains clean. The repair is only in `scripts/run_a5_sim.py`, with focused
  tests and one existing test's fault-injection target adjusted to the override.

## Repaired natural E2E — PASS

[`natural-attempt-02`](../outputs/a6/v12-integration/natural-attempt-02/physical_summary.json)
runs repair `7745dc0` with the original natural configuration: UAV initial
view `(-.5,0,1.5,0)`, BUNKER `(3,-2.5,pi)`, three 5 s windows, .05 m flight/facade
tolerance,120 s navigation guard and3 m Ground travel guard. Hover remains
.10 m/.10 rad/.10 m/s with.5 s dwell and45 s wall guard. The separate frozen
experimental common view `(-1.4,0,1.2,0)` is not changed or used for this natural
regression. The checker starts after adapter readiness; no early status
subscriber substitutes for it. Both adapter and checker exit0, followed by
normal roslaunch exit0 and released ports11951/11952. Existing checker
finalization changes CHECKS_PASS to PASS only after that teardown.

| Stage | Simulation time (s) | Result |
| --- | ---: | --- |
| PREFLIGHT | 421.463 | Original public runtime ready |
| Initial accepted aerial handoff | 438.681 | Runtime RGB-D reference available |
| MID360 windows complete | 464.320 /493.743 /516.113 | Real A2 updates; 2 NBV flights |
| Ground selection | 518.715 | 2 confirmed; original winner candidate-000008 selected |
| Ground arrival | 563.429 | Navigation succeeds; travel1.961913 m |
| D435 refined target | 574.357 | Fresh map observation, stamp573.835; age.522 s |
| GRASP begins | 575.390 | Refined collision-aware pregrasp executed; D_exec reached |
| LIFT | 588.450 | Grasp confirmed and physically checked retrieval |

The selected exact pose is source544, evaluation8, relevance1.0,
`(2.3934511620720507,-0.6039048728552128,2.094395102393195)` in map. It has
**107/107** real ground-support cells, no clipping and no operational blocker.
The other confirmed original winner is source543/candidate-000009. No
non-winner alternative is selected. Perception/grid placement differs slightly
between fresh physical runs, so source IDs/catalog counts are not treated as
cross-run physical identities.

Independent checker: **brick lift .148691897 m**, **TCP lift .149553405 m**,
three successful arm-controller goals, grasp confirmation, one takeoff and
one landing. Task elapsed PREFLIGHT→LIFT is166.987 simulation seconds; Ground
approach→stop is23.078 simulation seconds. These are descriptive regression
measurements, not paired efficiency estimates. Gazebo model state is used only
by the existing physical checker, never as an algorithm input.

Window durations are5.091/5.095/5.086 simulation seconds, with52 chunks each.
During the second capture, the existing unchanged hover gate discarded34
partial chunks and an empty restart before obtaining a valid complete window
within its original guard. Those partial chunks do not update A2; no extra
observation vote/window, corrective flight or new fallback was introduced.

### Successful-run replay and non-winner diagnostic

All three decisions reproduce exactly; all43 final artifact checks and all A4
formula checks pass. Maximum task-gain numerical residual is1.42e-14.
Full IDs, blockers, evidence votes and checks are in
[natural-attempt-02-replay.json](../outputs/a6/v12-integration/natural-attempt-02-replay.json).

| Window | Confirmed | Blocked winners /37 | Positive operational cells | Task uncertainty mass | Non-winner cells / alternatives |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 0 | 31 | 176 | 176.000000 | 2 /3 |
| 2 | 0 | 35 | 114 | 69.144495 | 2 /4 |
| 3 | 2 | 35 | 114 | 41.938256 | 2 /4 |

Later-window diagnostics contain source624 alternatives000004/000007 and
source581 alternatives000061/000073; both cells' original winners remain
blocked. These alternatives remain diagnostic only. Other exact winners
confirm and complete retrieval, so this natural run does not demonstrate a
new structural handoff deadlock or justify multi-support semantics.

Same-state best task/generic viewpoint IDs are9/24,1/16,0/0. They share the
candidate/visibility/cost model, but only Ours is physically executed here.
These dual rankings are not a Generic execution outcome or evidence of method
superiority. The [saved visualization](../outputs/a6/v12-integration/natural-attempt-02/nbv.png)
uses actual A2/A3 inputs with hypothetical A4 visibility. Its inherited
"no real scan or flight" caption refers to the prediction model, not to the
completed physical observations documented here; the frozen renderer is unchanged.

Post-natural [focused regressions](../outputs/a6/v12-integration/post-natural-tests.log):
153 tests, one native-only skip, no failures (9.909s), covering exact anchors,
Hard-002 replay, v1.1 operational gate and A5/A6 integration/manipulation.
Hard-002 still removes source624's false center collision while87/88 genuine
ground-support cells prevent confirmation. No old outcome is reclassified.
The additional fresh [six-snapshot Hard-002 replay](../outputs/a6/v12-integration/hard002-replay.json)
also passes every legacy-array/ranking and A4-input check; it retains compact
source624 and non-winner diagnostics without duplicating the original full report.

## Review boundary and follow-up

The current gap is closed for the requested **single natural regression**,
not a claim of universal hover reliability or statistical v1.2 superiority.
The old hover failure lacks sufficient retained diagnostics to establish its
root cause and remains explicitly unresolved historically. No hover/controller
repair or relaxation was used in either new activation.

Keep the feature checkpoint and PR Draft; do not merge or resume the historical
matrix. The [fresh-seed proposal](V12_FRESH_SEED_VALIDATION_PROPOSAL.md) recommends
six paired validation attempts (three new seeds × Generic/Ours), **not executed**.
No final formal sample size is selected. Stop here for review.

Independent final delivery review found no material blockers: physical results,
stage times, saved-state replays, diagnostic-only alternatives, proposal scope
and frozen source boundaries match the retained data. New observations,
target references, decisions/rankings, checker summaries, adapter logs and test
logs are included in the feature checkpoint. Raw public TF/control bags and ROS
runtime logs remain locally alongside each attempt, excluded from Git by the
existing ignore rules. Four preexisting unrelated A5 diagnostic directories are
untouched. No A6/SIM process remains from these runs.
