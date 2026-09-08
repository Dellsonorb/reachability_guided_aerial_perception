# A6 formal simulation protocol — prospective v1.1 study

2026-09-08. Written after the independently seeded Pilot-2 readiness review and
before any formal method outcome. This is a separate study, not a continuation
of the pilot's statistical sample. The current user authorization permits
formal execution after the documented readiness conditions; the earlier design
proposal's stop-before-formal restriction is historical.

## 1. Question, population and frozen system

The **sole primary comparison** is Ours versus Generic NBV on binary physical
E2E retrieval success. The question is whether manipulation-weighted predicted
observation gain improves the complete system, not merely its own score.
Confirmation, execution-validated discovery, resources and the other methods
are secondary. No success-only efficiency result substitutes for E2E success.

The target population is an **equal mixture of the three existing scene
templates**, with exactly the original target/BUNKER perturbation distribution.
It is not all warehouses, all obstacle layouts, all retrieval objects or real
robots. Fixed templates, narrow perturbations and uncontrolled residual
scan/physics/planner timing limit generalization. No new wall randomization or
mirror distribution is introduced from the original unimplemented proposal.

Use the research implementation at `56ece4a` with its approved A3/A5 object-aware
v1.1 gate and unchanged A1/A2/A4. SIM's actual common configuration remains the
one used in Pilot-2 (public correction included in main `2e7feaa`); frozen RM4D
and the independent task-domain asset are unchanged. A6 launcher/reporter fixes
are integration only and are separately recorded in Git.

Generic and Ours share, for identical input, the ordered candidate generator,
range/FOV/assumed-prism visibility, cost, observation budget, A2 updater,
representative/exact v1.1 gate, Ground selection and execution pipeline. Only
generic unknown versus manipulation-weighted gain differs. Closed-loop runs
may subsequently have different beliefs, poses, clouds and catalogs. Same-state
dual rankings check the gain-only identity; they are not independent trials.

The v1.1 association allowance is the unchanged sensor/pixel geometry formula
and constants, not a fitted per-scene value. The prior `.0439527451 m` value is
one accepted-depth/intrinsics result. Record each computed value; never tune it
to a rescue count, discovery, score or retrieval result. A2 raw evidence/state
and all thresholds remain unchanged. ENVIRONMENT/AMBIGUOUS block; TARGET uses
the continuous expanded perceived object versus padded footprint intersection;
real ground votes remain necessary. No runtime Gazebo GT is used.

## 2. Exact method and endpoint semantics inherited unchanged

The operational definitions in [Pilot-1 protocol](A6_PILOT_PROTOCOL.md), as
explicitly revised by [Pilot-2 v1.1 protocol](A6_PILOT2_PROTOCOL.md), apply:

- Common map initial sensing pose `(-1.4,0,1.2)`, yaw `0`; UAV launch
  `(-.5,0,.15)`, yaw `0`. The RGB-D depth gate remains `4.0 m`.
- Three **total completed** independent MID360 windows, each `5 s` simulation
  time; initial and stationary rescans count. Capture wall guard `20 s`, common
  task wall guard `1200 s`, Ground navigation timeout `120 s` remain unchanged.
- Generic/Ours apply the same stop tests to their **own** gain and score:
  no valid candidate, no positive gain, best score at most zero, or the third
  completed window. No early stop at first confirmed Ground candidate.
- Fixed obtains three real windows at the fixed sensing pose. RM4D-only uses
  no MID360 decision windows and keeps original exact top-one ranking; its
  aerial bootstrap still costs real flight/time. A2 confirmation is N/A for it.
- For all environment-gated methods, the catalog contains original validated
  A1-cell winner candidates, not cell centers. At stop, select maximum relevance
  among the same combined representative/exact confirmed catalog, retaining
  original first-tie order. Attempt exactly one Ground candidate; no fallback.
- Confirmation requires real ground-support votes and the shared v1.1 gate.
  A1 UNASSESSED, raw A2 UNKNOWN, operational blocking and execution failure
  remain different concepts. Confirmation is not navigation feasibility.
- `D_exec` requires actual BUNKER arrival, fresh D435 refinement and successful
  collision-aware refined pregrasp execution. It is not a hypothetical IK count.
- E2E success requires the unchanged executed pipeline and independent physical
  checker: grasp/retention, brick lift at least `.10 m`, TCP lift at least `.10 m`.
  A status string LIFT alone cannot establish success. Checker model state is
  outcome measurement only, never an algorithm input.

The exact shared numerical settings are serialized in `configs/a6_formal.json`
and tested equal to Pilot-2. No A1–A5 code, asset, threshold, gain scale, flight
cost, window budget, Ground selection or success-definition tuning is permitted.

## 3. Sample, seeds, order and admissibility

Fix **120 independent scene seeds: 40 Easy, 40 Moderate, 40 Hard**. Every scene
runs RM4D-only, Fixed-view, Generic and Ours: 480 valid slots. Each Hard scene
also runs w/o occlusion and w/o flight cost: 80 additional slots. Total:
**560 valid method slots**, plus separately identified setup/INVALID activations.
Generic already is w/o task weighting; do not duplicate it as another ablation.
One scene is the paired replication unit, not a cell, viewpoint, cloud or retry.

Seed RNG is `random.Random(202609083)`. Take sequential `randrange(1,2**31)` draws,
skipping only exact duplicates and the six previously used Pilot-1/Pilot-2
seeds. Assign eligible draws in replicate-major Easy/Moderate/Hard order.
Serialize all 120 scene specifications **before running any method**. Each
scene's own RNG makes exactly the original six uniform draws, in order:
target XY about `(2,0)` with ±`.15 m`; target yaw ±30 degrees; BUNKER XY about
`(3,-2.5)` with ±`.10 m`; BUNKER yaw about pi with ±5 degrees. Target z `.0575`,
BUNKER z `.36`, brick properties and all tier boxes are copied unchanged.

Order RNG is independently `random.Random(202609084)`. First, for each tier in
Easy/Moderate/Hard order and each consecutive group of four replicates, shuffle
the four core methods and shuffle the four left-rotation indices. Assign each
rotation exactly once to that group's four scene blocks. This gives each core
method exactly ten occurrences in each core position per tier. Then shuffle
the three tier blocks independently for each replicate and append them in
that order. After a Hard block's four core methods, append its two ablations;
their order is no-occlusion/no-cost for odd numbered replicates and reversed
for even numbered replicates. Their later position is a secondary-comparison
limitation; fresh simulator resets prevent task-state carryover.

The thin generator and its tests reproduce this configuration without reading
any experiment output. The actual slot list is the execution order. Run one
fresh SIM at a time. INVALID reruns stay next to their original slot; the next
valid method is not selected based on the preceding outcome.

Before each scene's method block, use the unchanged method-independent setup
check at the fixed pose: public TF/hover/readiness, accepted RGB-D depth/FOV,
and at least 100 distinct `.10 m` ground cells from the real five-second MID360
window. This separate setup activation does not query RM4D, select a candidate,
score a viewpoint, execute retrieval or contribute an endpoint. It may be
diagnosed/repeated for proven platform invalidity, but cannot select a seed.
All seeds are already frozen. A genuine setup-domain conflict is documented
and investigated; it is not silently replaced by a new seed or a different pose.

No scene is filtered or relabeled using score gap, confirmation, catalog size,
method win, actual retrieval or first-window difficulty descriptions. The old
occlusion/unknown-area targets remain post-freeze descriptive manipulation
checks. Pilot-2 Moderate already undershot its suggested occlusion interval;
retain and report this limitation instead of moving its wall. Natural seeds
need not produce TARGET-alias rescue; synthetic regressions test that mechanism.

## 4. Prospective sample-size rationale

The design target is an engineering-relevant **20 percentage-point** paired
success difference, not a Pilot-2 effect estimate. For planning, assume each
tier has discordance probability `d=.50`, Ours-only probability `.35`, Generic-
only probability `.15`. The other outcomes share the remaining `.50` arbitrarily.
Use a two-sided exact McNemar test at alpha `.05`, with no continuity correction
or mid-p substitution. The planning distribution is:

`D ~ Binomial(N,d)`, `B | D ~ Binomial(D,(d+delta)/(2*d))`.

Sum the exact rejection probability over both binomials. At `N=120`, delta `.20`,
power is approximately `.8622` for `d=.50`; sensitivity is `.9851` for `.30`,
`.9343` for `.40`, and `.7879` for `.60`. The 40-per-tier choice also balances
core method positions. These are assumptions, not claims about actual effect
or nuisance rates. The study is not promised 80% power for smaller effects,
all heterogeneous tiers, or an additional confidence-bound criterion. The
conservative bound below is not an added primary rejection gate: that would
have extremely low power at this sample size even under the planning effect.

There is no efficacy interim test, outcome-based sample-size increase, favorable
seed replacement or optional stopping. Pilot-1/2 data are never pooled into the
formal analysis. Operational monitoring may inspect failures/fairness/measurement
and stop for the user's substantive scientific conditions, not a promising p.

## 5. Primary paired inference and its precise null

For tier h, report all four paired binary outcomes, including Ours-only `b_h`
and Generic-only `c_h`; let `b=sum b_h`, `c=sum c_h`. The prespecified two-sided
exact McNemar p-value is `min(1, 2*BinomialCDF(min(b,c); b+c, .5))`, or `1` when
there are no discordant pairs. This is the standard exact-binomial construction
documented by [statsmodels McNemar](https://www.statsmodels.org/dev/generated/statsmodels.stats.contingency_tables.mcnemar.html).

Because tier counts are fixed, its exact null is **paired directional symmetry
within every tier**, `P_h(Ours-only)=P_h(Generic-only)`. Discordance rates may
differ by tier under that null. Do not mislabel it as an exact test of only the
weaker average risk-difference null when opposing tier effects can cancel.
Report every tier's discordance table and direction to expose such heterogeneity.

Estimate the equal-tier success difference by
`Delta_hat = (1/3)*sum_h (b_h-c_h)/n_h`; with the complete balanced sample this
equals `(b-c)/120`. For a boundary-safe, stratification-valid conservative 95%
interval, form six simultaneous Clopper–Pearson intervals: one for each tier's
Ours-only and Generic-only probability, each with confidence `1-.05/6`.
Then report `[mean(L_b-U_c), mean(U_b-L_c)]`. The union bound gives at least 95%
simultaneous coverage without assuming equal tier probabilities; within-pair
multinomial dependence does not invalidate it. Exact proportion intervals use
[SciPy's Clopper–Pearson option](https://docs.scipy.org/doc/scipy-1.15.0/reference/generated/scipy.stats._result_classes.BinomTestResult.proportion_ci.html).
All-zero discordance therefore produces an interval of positive width, not a
misleading bootstrap `[0,0]`.

Also report an explicitly **approximate** stratified paired-difference normal
interval for effect-size precision. With `d_i=S_O-S_G`, use
`s_h^2=(b_h+c_h-n_h*delta_hat_h^2)/(n_h-1)` and
`SE^2=(1/9)*sum_h s_h^2/n_h`; the interval is `Delta_hat +/- 1.96*SE`, intersected
with `[-1,1]`. This follows the variance of independent stratified means applied
to [within-pair differences](https://www.itl.nist.gov/div898/software/dataplot/refman1/auxillar/t_test.htm).
It does not assume equal tier effects or variances. It is not exact finite-sample
inference, and clipping does not repair poor coverage. Flag any tier with fewer
than five Ours-only or Generic-only pairs, and any zero-variance tier; these are
reporting warnings, not coverage guarantees or reasons to change sample size.
When total variance is zero, mark the approximate interval unavailable and keep
the conservative bound. Always show both bounds, not only the more favorable
one. Paired-proportion normal-interval limitations are documented by
[Newcombe's paired-data study](https://pubmed.ncbi.nlm.nih.gov/9839354/).

One primary p-value is tested at `.05`; no other baseline is promoted to primary.
Positive observed difference plus rejection is reported with the precise null
above. Average-benefit evidence supported only by the normal interval is labeled
approximate, never an exact 5% test of the weaker average null; sparse warnings
prevent treating that interval alone as decisive. A finite-sample positive
average-effect claim requires the conservative bound to exclude zero. These
effect-size interpretations are not additional primary rejection gates.
A nonsignificant result is not equivalence; a negative estimate or failure to confirm benefit is reported
without changing the claim's endpoint. Small per-tier samples are not advertised
as independently powered subgroup studies.

Final inference requires all 120 prescribed primary pairs. Incomplete pairs
remain explicitly missing and prevent a claim of a completed planned analysis;
they are not silently removed or imputed as task failures. Proven INVALID trials
are rerun under the unchanged slot. In a separately labeled sensitivity, treat
the first directory-ordered activation's completed INVALID as failure and retain
its paired counts. An unfinished first activation remains missing even if a
later activation completes. Never replace the primary valid-slot results with
this sensitivity.

## 6. Secondary outcomes, ablations and measurement

For every seed/method report confirmed candidate counts and first discovery,
`confirmed → D_exec → retrieval`, actual windows/moves, UAV active/total
distance and simulation time, Ground travel/navigation, D435 refine and each
arm/gripper/lift stage. Explicitly report numerator, applicable denominator,
unreached stages and missing measurements. RM4D-only confirmation is N/A.

All robot-efficiency metrics use simulation time. Frozen online trajectory
metrics remain primary resource measurements, with the same `.3 s` path-gap
rule and `.5 s` TF age. Missing distance is not zero; observed partial length is
only a lower bound. Run the existing causal TF-bag reconstruction uniformly
for all activations as a **separate secondary diagnostic**, preserving original
sampling timestamps and never overwriting online metrics or outcomes.

Report paired complete-resource differences with their availability denominators,
per-tier distributions and success/failure context. Early failure can consume
less flight/time, so it does not itself prove efficiency. First-confirmation
timing is conditional on that stage being reached, not an actually executed
early-stop policy. Joint-success resource summaries, if shown, are explicitly
conditional diagnostics. Do not invent zero/capped discovery times for failures.

RM4D-only and Fixed comparisons are descriptive system references. Hard
w/o-occlusion and w/o-cost compare to Ours on their same 40 seeds; report paired
counts, risk differences and two exact McNemar p-values as secondary, with Holm
correction across those two tests. They do not expand the sole primary family.
No extra formal claim is based on unplanned subgroup/metric p-value hunting.

Retain per-window v1.1 TARGET/ENVIRONMENT/AMBIGUOUS votes and cells (mixed classes
are not exclusive), raw versus operational blockers, continuous collisions,
TARGET-alias retained candidates, representative-only blockers, real ground
deficits and confirmation. Same-state Generic/Ours gain/penalty/order diagnostics
and the actual ablation policy checks remain offline only, never policy feedback.

## 7. Failures, platform limitations and storage

Apply the unchanged taxonomy before considering a rerun. An actually reported
method/execution failure retains priority over checker cleanup/transport errors.
Budget exhaustion, no confirmed candidate, genuine nav/refine/descend/grasp/lift
failures are valid failures. Only independently demonstrated platform/setup or
primary-measurement invalidity permits the same-slot rerun. Preserve every
activation and its reason; do not retry valid failures to obtain success.

Pilot-2 exposed valid navigation non-convergence, terminal arm velocity rejection
and retention loss. Their common platform behavior is retained for this study.
In particular, the known `latch_xy_goal_tolerance` namespace mismatch is not
silently repaired or used to erase nav failures; causation for those failures
was not established. This can mask an observation-policy benefit and limits
interpretation. The study estimates performance of the actual shared system,
not an oracle flawless navigation/manipulation backend.

The A6 checker startup correction at `3647c54` is retained: start the unchanged
physical checker after the final adapter publisher-ready log marker, within
the unchanged existing waits. Passive native topic recording is common to all
methods and never subscribes to demo status or provides GT to the controller.
Raw A2 windows, events, decisions, scores and measured outcomes remain saved.

Formal control bags record native LZ4 throughout the trial, with all previous
non-image topics unchanged. Set the common diagnostic-only option
`diagnostic_image_scope=ground_handoff_to_end`: start a separate native LZ4
D435 RGB/depth recorder when the external launcher observes `A5_SELECTED` or
`A6_RM4D_SELECTED` in the already flushed event file. Both are actual handoff
selection events before return/landing/navigation, not score/confirmation tests.
Keep every subsequent raw image through shutdown, irrespective of success or
failure. No handoff means no Ground refinement and no image recorder. Calibration,
observer outputs, control/TF and raw A2 observations remain continuously saved.
Do not subscribe to demo status, wait for the recorder in the robot, alter sensor
rates, thin images, or change the common deadline. Record start/exit/finalization
and check that images cover an actually reached refine stage.

This is prospective stage-scoped diagnostic collection, not retrospective
retention based on a method win. Pilot originals remain unchanged. A measured
native BZ2 conversion saved only 26.05% and merged publisher connection metadata;
it is **not adopted**. Its preserved probe copy is outside the experiment data.
Pilot-2 indices project selection-scoped bags at approximately 651.5 GB for 560
valid runs, plus 20.9 GB setup controls and 7.7 GB other outputs: approximately
680 GB against 884 GB available at planning. This assumes a Pilot-like stage
mix, not a worst-case guarantee; all-nav-timeout behavior could exceed capacity.
Check actual disk use between scene blocks and solve any storage issue as an
engineering matter without changing N, scenes, methods, outcomes or silently
discarding collected observations. No additional archive savings are assumed.

Before formal activation commit/push this protocol, serialized configuration
and reviewed tested integration changes. Keep source/output checkpoints on the
A6 feature branch, no main merge. A demonstrated runtime bug may receive a
minimal documented fix, with affected runs identified and fair paired reruns
when genuinely invalid; it cannot authorize result-driven method tuning.

Stop for a scientific decision only if the user-listed structural deadlock,
core-definition change, irreparable fairness, required result-driven protocol
change, runtime GT requirement, or systematic core-hypothesis refutation is
established. Ordinary diagnostics continue autonomously. Final reporting must
include null/negative evidence, limitations and incomplete measurements.
