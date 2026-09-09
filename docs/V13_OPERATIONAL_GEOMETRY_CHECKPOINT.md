# v1.3 operational geometry — development checkpoint

2026-09-09. Development branch `feature/v13-operational-geometry`, based on
`a89c84d`. The user's updated autonomous-development authority permits local
method iteration while preserving historical results, runtime perception and
shared Generic/Ours components. This is **not final-test evidence**.
No remaining fresh-validation slot or new formal slot was run.
PR #6 remains Draft; main and the old A6 branch remain unchanged.

## Selected small revision

The local extension is sufficient for the observed AMBIGUOUS whole-cell veto;
no voxel map, height-map, alternate candidate search or new execution framework
is introduced. A2/classification/ground votes, continuous TARGET collision,
v1.2 exact winners and A4 formulas remain unchanged in this revision.

For each retained AMBIGUOUS occupied endpoint `p_e` in public map XY, define
the closed disk `E_e = {x : ||x-p_e|| <= rho}`, `rho = .033 m`. For exact padded
BUNKER footprint `F_q`, an endpoint blocks iff `E_e` intersects `F_q`. This
includes points in neighboring cells; the query is not limited to the original
coarse-cell overlap. Tangency counts as collision.

The name `public-map-tf-conditional-disk33mm-v1` records an **engineering
approximation conditional on authoritative public-map TF**, not a calibrated
probability or total physical error bound. The declared LiDAR budget is
`3*.010 + .002 = .032 m`, with a separate `.001 m` numerical rounding reserve.
The current custom simulator does not actually apply that declared Gaussian
noise/quantization, and the old bag cannot bound position/attitude acquisition
age. These limitations remain explicit. The value was declared before v1.3
development execution and was not selected from winner gaps or trial outcomes.
It is independent of the RGB-D association allowance/formula. Full source
reasoning and all five geometries are in the
[design record](superpowers/specs/2026-09-09-v13-ambiguous-subcell-design.md).

`AmbiguousEndpointEvidence` is an optional sidecar to `OperationalEvidenceView`:
map XY, original observation/row index, cell ID, complete per-cell window-vote
counts, named profile and radius. Complete counts are **window groups, not
point counts**. Missing endpoints, incomplete historical groups or unavailable
profile retain the original whole-cell blocker. No old endpoint is invented,
removed by later ground votes or reclassified as TARGET/FREE.

Both A3 and A5 call the same `assess_footprint`; a separate diagnostic lists
endpoint intersections, fallback cells and spatially separated coarse aliases.
`blocked` still means operational occupied blocking, not full navigation
infeasibility. Actual ground support for every footprint cell is still required
for confirmation. Legacy v1/v1.1 defaults and snapshot fields remain compatible;
runtime opt-in is `--operational-gating v1.3 --support-anchor exact_winner`.

## Recorded-state development replay

These are counterfactual calculations on original runtime observations, not
new trials or replacement outcomes. Seven recorded rounds pass exact-winner,
nominal support, four operational vote-array, shared A3/A5 gate, Generic/Ours
candidate/visibility/assessment and A4 gain-formula consistency checks.

| Original input | Rounds | Blocked winners, v1.2 → v1.3 | Confirmed, v1.2 → v1.3 |
| --- | ---: | --- | --- |
| Fresh Moderate Ours slot 003 | 1 | 34 → 29 | 0 → 0 |
| Historical Hard-002 Generic slot 015 | 3 | 33/33/33 → 30/30/30 | 0/0/0 → 0/0/0 |
| Original natural A5 attempt 02 | 3 | 31/35/35 → 31/31/31 | 0/0/2 → 0/0/2 |

For Moderate, all 29 true expanded-target intersections still block. The five
other original exact winners have minimum distances to all 109 AMBIGUOUS
endpoints of 118.215, 98.215, 91.850, 106.091 and 71.850 mm (source IDs
585/705/584/623/622). Their disk-to-footprint clearances are therefore
85.215/65.215/58.850/73.091/38.850 mm. All five have zero cells meeting the
two-window support requirement at this first observation; none is confirmed.

The same Moderate state changes from zero operational uncertainty mass and
`NO_PREDICTED_TASK_GAIN` to mass109.192660, best task gain39.622847 and
score39.122847. A4 proposes further observation, not a Ground handoff.
This establishes removal of this recorded geometric veto; it does not prove
future sensing or retrieval will succeed. Hard-002 source624 still has only
87/88 real supporting cells at its third window and remains unconfirmed.
Non-winner alternatives are counted in every replay but never selected.

ENVIRONMENT diagnosis found no environment-blocked exact winner in nine
reviewed fresh/natural snapshots; the Moderate nearest environment endpoint
to any winner is0.686950 m away. There is no present evidence to extend this
revision to ENVIRONMENT geometry. Repeated AMBIGUOUS evidence may still
suppress a required ground vote in a given window. That residual interaction
is tracked, not erased or declared solved from a single stored observation.

Derived reports and plots:

- [Moderate](../outputs/diagnostics/v13-development/moderate/replay.json),
  [A4 plot](../outputs/diagnostics/v13-development/moderate/round-01/nbv.png).
- [Hard-002](../outputs/diagnostics/v13-development/hard002/replay.json).
- [Natural recorded states](../outputs/diagnostics/v13-development/natural-recorded/replay.json).
- [Original endpoint/association geometry](../outputs/diagnostics/v13-design/moderate-endpoints.json).

Reproduction uses the existing core environment; choose a new output folder:

```bash
PYTHONPATH=src:scripts MPLCONFIGDIR=/tmp/a6-mpl XDG_CACHE_HOME=/tmp/a6-cache \
  /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
  scripts/replay_v13_operational.py \
  --data-dir outputs/a6/v12-fresh-validation/slot-003-moderate-ours-01/data \
  --output-dir /tmp/v13-moderate-new-replay
```

The replay rebases a declared original RGB-D reference to the attempt's bundled
file, allowing relocated records without modifying `initial.json`. It never
reconstructs missing source evidence. Output may not overwrite the original
attempt. The raw inputs and old results remain unchanged.

## Natural runtime regression and engineering record

The existing natural configuration is retained: initial aerial view
`(-.5,0,1.5,0)`, BUNKER `(3,-2.5,pi)`, three five-second windows, unchanged
hover/observer gates, .05 m flight/facade tolerance,120 s navigation guard and
3 m Ground travel guard. The separate experimental initial pose is unchanged.
This is one development scene, not an additional validation seed or method pair.

The dedicated11951/11952 runtime is recorded under
`outputs/a6/v13-development/natural-attempt-01`. Two preflight launches are
retained as development runner errors, not method failures:

1. Buffered Python stdout delayed the readiness log until after the unchanged
   ten-second subscriber deadline. Checker registration occurred after adapter
   exit; no PREFLIGHT or observation window occurred.
2. With unbuffered stdout, the temporary runner incorrectly reused A6's exact
   `A6_ADAPTER_READY` marker for the A5 adapter. It again failed before PREFLIGHT.

The third launch uses `python -u` and matches A5's actual readiness message;
the physical checker then connects before the original deadline. No repository
method/runtime thresholds or SIM controller settings were changed for either
runner correction. The earlier launches never flew or moved Ground.

The third launch reached runtime RGB-D perception and A1, then failed before
its first MID360 window: post-core settling fixed an **uncommanded transient**
capture anchor at sim719.884, map`(-.379530,-.048424,1.217717,-.008364)`.
The vehicle subsequently recovered elsewhere; waiting for a return to that
old measured point was not waiting for the commanded flight goal.

Offline composition of the retained public TF with2025 state samples across
sim719.890–760.370 found zero samples passing both old position/speed gates.
Of those,1849 samples satisfy speed<=.1 m/s; their closest distance to the old
anchor is.152575 m, beyond the unchanged.1 m tolerance. This diagnoses the
fixed transient-anchor mismatch; it does not identify or claim to fix the
underlying estimator/controller drift. The failed activation remains retained,
not retroactively INVALID_TRIAL or successful.

The scoped repair establishes the post-core XYZ anchor **during a stable
dwell**: any failed check resets dwell and updates only the uncommanded XYZ
reference. Position must stay within.1 m over the full.5 s dwell; yaw, speed,
freshness, flight bounds and45 s wall deadline remain enforced. Commanded
flight-arrival checks remain strict and use the original goal. The subsequent
five-second acquisition/stamped-pose gates are unchanged. No motion retry,
replacement viewpoint, extra window or controller change is introduced.
The initial capture yaw remains the reference across both phases, preventing
two successive.1 rad tolerances from compounding. `A5_SETTLED` retains the
actual measured pose; capture metadata uses stable XYZ and the original yaw.

Five focused tests cover recovery away from an uncommanded transient point,
strict commanded arrival, speed/yaw/bounds rejection, rapid position changes
resetting dwell, and cross-phase yaw drift. The yaw issue was found in
independent review, reproduced by a failing full-capture test, then corrected.
The reviewer independently passed all24 capture tests with no remaining finding.
A fourth launch ran this repair in a fresh dedicated runtime with the same
natural scene and parameters. **Natural A5 E2E PASS**:
[physical checker](../outputs/a6/v13-development/natural-attempt-04/physical_summary.json),
[exact decision/field replay](../outputs/a6/v13-development/natural-attempt-04-replay.json),
[same-state gating comparison](../outputs/a6/v13-development/natural-attempt-04-gating-comparison.json),
[field/NBV visualization](../outputs/a6/v13-development/natural-attempt-04/nbv.png).

| Stage | Simulation time (s) | Result |
| --- | ---: | --- |
| PREFLIGHT | 42.703 | Public runtime ready |
| Runtime aerial handoff | 65.423 | Map target and RGB-D reference available |
| MID360 windows complete | 91.583 /109.732 /133.692 | 3 real windows; 2 NBV flights |
| Exact Ground selection | 136.841 | 1 confirmed original winner,107/107 support cells |
| Ground arrival | 182.275 | Navigation complete; reported Ground travel1.959931 m |
| D435 refined observation | 193.294 | Fresh map observation, age.257 s |
| GRASP | 194.360 | Collision-aware refined pregrasp succeeded; D_exec reached |
| LIFT | 207.227 | Physical grasp and lift confirmed |

Chosen original source543/candidate000008, relevance1.0, exact map pose
`(2.394126013,-.603736279,2.094395102)`. Physical brick lift is
**.148775374 m**, TCP lift.149557892 m, with three successful arm-controller
goals and one takeoff/landing. PREFLIGHT→LIFT takes164.524 simulation seconds.
Windows last5.025/5.021/5.097 simulation seconds,51/51/52 packets. The two
active commanded displacements are2 m each; this4 m sum is **not a measured
UAV path metric** or a paired efficiency estimate.

| Window | Raw grid-blocked | Old object-aware blocked | v1.3 blocked | Confirmed | TARGET / ENVIRONMENT / AMBIGUOUS window-cell votes |
| --- | ---: | ---: | ---: | ---: | --- |
| 1 | 0 | 28 | 28 | 0 | 0 /0 /0 |
| 2 | 32 | 32 | 28 | 0 | 5 /0 /7 |
| 3 | 32 | 32 | 28 | 1 | 8 /0 /12 |

The28 expanded-target collisions remain blocked, including before occupied
endpoints are observed. The final sidecar contains53 real AMBIGUOUS endpoints
representing12 complete window-cell votes. Four winners previously blocked by
coarse-cell aliasing are retained in windows2/3, but this does not create extra confirmation. Ground
vote totals are341/1626/2974. No raw A2 or association vote changes in the
same-input comparison. All three saved runtime decisions reproduce exactly;
all43 final field/ranking/decision checks pass. Maximum A4 gain-formula error
is7.11e-15. Same-state best Ours/Generic IDs are9/24,1/2,0/0; only Ours was
physically executed. This is not a Generic outcome or a superiority claim.
Non-winner diagnostics stay2 cells/3 alternatives per round, with no reselection.

Adapter/checker exit0. The dedicated runtime was then interrupted normally;
the interactive wrapper reports130 for Ctrl-C. No owned11951/11952 listener
or run process remains. Only after this teardown was CHECKS_PASS finalized to
PASS. The unrelated pre-existing11345 Gazebo process was left untouched.

## Review and verification

Independent specification and quality reviews passed for the shared core and
runtime integration. Review fixes were limited to unavailable-profile NPZ
serialization, source-output guards, explicit diagnostic mismatch errors and
the relocated recorded-reference path. Relevant new tests were run failing
before implementation, then passing. Final verification:

- [Full core suite](../outputs/a6/v13-development/final-core-tests.log):
  592 tests,22 environment-dependent skips, no failures,60.277 s.
- [Native Noetic split](../outputs/a6/v13-development/noetic-tests.log):
  125 tests, no failures,3.624 s.
- [Actual frozen RM4D/task-map suite](../outputs/a6/v13-development/task-map-tests.log):
  9 tests, no failures,1.148 s. Suites overlap; counts are not added.
- Seven historical development rounds plus three new natural rounds pass
  shared-method/unchanged-vote checks. Natural saved decisions and43 final
  artifacts reproduce exactly. Geometry tests include contact, rotation,
  neighboring cells, incomplete history and missing profile fallback.
- The source/result diff remains empty for A2, A4, RM4D assets and all original
  Pilot-1/Pilot-2/formal/v1.2 development and fresh-validation outputs. SIM
  remains clean at5e25039; the frozen RM4D checkout remains unchanged.

This checkpoint removes the observed AMBIGUOUS geometric veto and passes the
existing natural integration regression. It does **not** establish fresh-seed
efficacy, formal readiness, universally reliable hover, or a bounded real-world
localization envelope. The repeated-ambiguity/ground-support interaction remains
a development question if new observations demonstrate it. No final-test
protocol, seed, primary metric, comparison component or success definition has
been changed. No remaining validation/formal slot was run.
