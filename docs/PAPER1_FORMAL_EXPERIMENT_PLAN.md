# Paper 1 prospective experiment proposal — evaluation finite-scan v1

2026-09-10. **DESIGN ONLY. No final tasks, setup simulations or matrix launched.**
Use [the named evaluation version](EVALUATION_VERSION.md). This replaces neither
historical protocols nor their results. No final scene seeds have been executed;
the independent scene manifest must be generated in full before any final method
run. This document proposes the study, rather than declaring it completed or
quietly resuming the interrupted560-slot matrix.

## Questions, choices and scope

Primary: does Ours improve physical E2E retrieval probability under the **same
three-window budget** relative to Generic? Secondary: where success is comparable,
does it reduce observation use/active time, and does any change propagate to
Ground execution time? Negative or inconclusive evidence is a valid outcome.

Choose concentrated paired evaluation instead of running all controls everywhere.
Choose modest variations of the current supported scene geometry rather than
only more jittered copies of three identical wall layouts, or an arbitrary-world
generalization claim. Choose no distance claim instead of modifying runtime
logging before evaluation. These choices allocate resources to the research
contrast and expose the program's actual limits without new architecture.

The population is a known brick in the current search/work region, static level
ground and zero/one/two low walls. The study does not establish arbitrary-object,
dynamic-scene, rough-terrain, long-hold or real-world performance. The complete
controller/MoveIt/perception/sensor installation is common to every method.

## 1. Scene population and an independent manifest

Primary **N=96 independently sampled scenes**. Draw difficulty IID from
`(easy,moderate,hard)` with probability1/3 each, not a forced quota. With
Python3.8 `random.Random(2026091021)` and96 successive `choice` calls on that tuple,
the pre-method tier draw is **38 Easy,28 Moderate,30 Hard**. This draw was checked
once, without any method run. Do not redraw it to rebalance counts. Target
estimand is the unweighted uniform-mixture population, not an equal-weight
post-stratification of the realized counts. This preserves a simple marginal
paired-binary analysis while covering all three conditions.

Before activation, use a separate `random.Random(2026091022)` to draw unique
positive31-bit scene seeds, rejecting **only** duplicate or historically used/
reserved seed numbers. Build that exclusion list from existing scene configs and
recorded setup/development outputs, including Pilot-1/Pilot-2, interrupted formal
seeds (even unexecuted ones), Hard diagnosis, natural regression and all recent
batches. Natural fixed poses and previously manually constructed scenes are not
final cases. Save the96 IDs, seeds, all geometry, subset membership and full
ordered task list in one ordinary JSON before the first final method observation.
No outcome-dependent replacement, optional sample-size increase or seed search.

For each seed, keep the original first six uniform draws in this exact order:

1. Brick center x=`2+U(-.15,.15)`, y=`U(-.15,.15)`, yaw=`U(-π/6,π/6)`.
2. BUNKER center x=`3+U(-.1,.1)`, y=`-2.5+U(-.1,.1)`, yaw=`π+U(-π/36,π/36)`.
3. Brick z=.0575m; nominal BUNKER base_link z=.36m; unchanged physical models.

Easy has no wall. Moderate has the existing wall centered(.4,.65), size
(.15,1,1)m. Hard has existing walls centered(.3,.5) and(.5,-.4), each size
(.15,.6,1)m. **Prospective coverage extension, not a new method:** in each
Moderate/Hard scene, after the first six draws, independently perturb each wall
center x/y by U(-.15,.15)m and yaw by U(-π/12,π/12), in listed wall order.
Keep heights, lengths, colors, sensor models, brick dimensions and ground fixed.
Existing box-spawn interfaces suffice. This changes occlusion and free-space
alignment without selecting scenes by which method benefits. Hard means the
two-wall generative condition; it does not guarantee that every realization
actually blocks a valuable support region or produces an Ours rescue.

Admissibility is **setup-only**: finite rigid geometry, valid spawn poses, no
initial solid-body interpenetration, target within the public camera's nominal
depth/FOV envelope and known work region. Check geometry algebraically before
launch; these ranges do not intentionally generate overlaps. If a generated
entry violates these rules, stop pre-run and fix the generation specification
consistently, keeping the rejected manifest; do not hand-replace that scene.
Do not reject for candidate count, support, NBV scores, free route, reachable
grasp, camera-refine success, retrieval or method disparity. Runtime sensing
failures in an admissible setup count as outcomes. No per-seed test-drive or
retuning of the initial observation pose. Scene metadata initializes Gazebo;
only runtime sensor/TF data crosses into task decisions.

The extension is deliberately small but has **not** been online validated here.
If it reveals genuine planning/sensing limits, retain failures. If it reveals a
software defect, follow the version policy below, not hidden scene filtering.

## 2. Methods, pairing and number of launches

| Cohort | Scenes | Methods / additional tasks |
|---|---:|---:|
| Primary | All96 | Generic+Ours =192 |
| Context controls | First2 generated IDs in each tier =6 | RM4D-only+Fixed =12 |
| Mechanism ablations | First4 generated Hard IDs | Ours w/o occlusion + Ours w/o flight cost =8 |
| Total planned | 96 distinct scenes | **212 tasks** |
| Infrastructure reserve | Same original scene/method, not new scenes | At most12 additional starts; **224 total cap** |

Generic is already the **w/o task weighting** ablation: do not add a duplicate.
Reuse the originally scheduled Ours outcome on each auxiliary scene; do not run
a favorable new Ours counterpart. Controls/ablations are small descriptive
mechanism checks, not powered secondary superiority experiments.

Generic/Ours share initialization, scan opportunity/occlusion, viewpoint
generation, flight weight.25, budget, A2 updates, v1.4 evidence, exact winners,
candidate confirmation/preview/order/fallback, navigation/camera/planning/grasp
and physical checker. Only NBV gain weighting differs. Same-state diagnostics
calculate both scores on one ordered candidate set and the same opportunity
tensor. Closed-loop observations may then differ; forcing them identical would
remove the decision being tested. Diagnostic states are not independent trials.

RM4D-only still uses initial aerial RGB-D and the same calibrated task-domain
asset, chooses the original top1, and skips MID360 confirmation and pre-handoff
multi-candidate screening. Later full-robot physical checks remain enabled. It is
an intentionally different context control, **not** an isolation of task
weighting and not the literature map's original domain. Fixed reobserves the
current measured hover location (normal drift allowed), uses the same3-window
cap and screened-candidate stop, and does not select translated NBVs. No-
occlusion removes only occlusion from gain opportunity, not operational collision
blocking. No-cost sets policy flight weight to0; inspect saved `policy_*` scores
because common ranking diagnostics still contain the full-method cost. Neither
ablation relaxes real collision, sensor validity or physical success checks.

Use fresh simulator processes/reset for every method. With independent
`random.Random(2026091023)`, shuffle scene blocks; within each tier randomly assign
exactly half Generic-first and half Ours-first (counts above are even). Methods
within a pair run consecutively. Predeclare auxiliary method order using the
same schedule RNG and run them after the primary pair in that scene block. Keep
the scene's physics seed identical if the existing platform exposes it; record
whether it does. A scene seed is **not** a promise of identical asynchronous
scan phase or controller randomness. Do not add phase locking or replay GT.
No favorable-order or successful-trial repetition; no human waypoint selection.

## 3. Fixed budget, stopping and exact execution

Keep three **accepted real five-simulation-second windows**, initial included;
at most two subsequent sensing decisions. A further decision can be a true
translation, yaw-only action or another observation at nearly the same location.
Discarded captures are logged but do not invent accepted votes. Existing bounded
capture/hover/interface timeouts remain; no fourth window or repeated launch to
make support appear.

After each accepted update, check real whole-footprint support and operational
geometry on exact per-cell winners. In the existing stable relevance order,
screen up to4 confirmed candidates with the shared complete manipulation model.
A successful screen ends active sensing. Otherwise use the method's gain/cost
policy while budget remains; stop at the existing nonpositive-score/no-view/
budget condition. No screenable candidate is a valid task failure, not INVALID.
Keep Ground travel bound3m and all existing camera/IK/preview search limits.

Selected exact pose comes from this run's evaluated candidates, not field cell
center or a past confirmed pose. BUNKER must navigate and stop; D435 must provide
fresh usable geometry; actual-arrival planning and execution must pass. Preview
is not `D_exec`. Non-winner diagnostics cannot reselect support. Failed navigation,
refine, planning, contact, lift or retention remains a method result when the
platform executed its normal contract. Keep12mm development clearance, explicit
SIM velocity mode/native diagnostics, full robot/gripper/target/payload checks
and the existing physical retrieval checker unchanged.

## 4. Outcomes and efficiency clocks

Primary per task: **physical E2E retrieval success**, including independent
perception/arrival/grasp/lift/short-hold checks; not launcher exit0, heatmap,
confirmation, precheck or a controller's `LIFT` message alone. Record binary
result plus first terminal cause, actual source/pose and any human intervention.
Report `observation → confirmed → screened → arrived/stopped → D435 refined →
D_exec → grasp → lift/retained`, with stage attempts and NOT_REACHED distinct
from failure. First confirmation and final counts are separate; confirmation
can precede a rejected execution preview.

| Metric | Definition and missingness |
|---|---|
| Windows | Capture calls, discarded attempts, accepted windows and voted updates separately; primary budget is accepted/voted windows, not flight commands |
| Actual observation-location changes | Between consecutive accepted windows, compare measured UAV poses from each window's first accepted packet: saved `chunk_T_map_sensor` composed with the known sensor-to-UAV mount. Do not use goals/`rescan`/`nbv_moves`; the capture anchor's yaw is also a retained reference, not necessarily the packet's actual yaw. Report XYZ displacement and wrapped yaw change; translation >.20m and yaw >.20rad separately (twice existing.10 tolerances). This is resolved observation-location change beyond the reporting hover scale, **not travelled distance**. Missing transform/mount makes classification missing, not zero. Also list nonzero-offset commands and actual yaw-only changes. |
| Active time | First `A6_CAPTURE_START` to `A6_ACTIVE_STOP`, or first terminal failure; includes sensing, motion, computation, hover and shared preview. Initial takeoff/RGB-D/RM4D query excluded and reported separately. |
| Preview time | Sum actual `execution_screen` intervals; nested inside active time, never add twice |
| Ground execution time | First `A6_STAGE_START:ground_navigation` through terminal `LIFT`/`FAILED`; breakdown navigation/stop, D435, pregrasp, descend, close, lift/retention. NOT_REACHED/null for no handoff, not efficient zero. |
| End-to-end time | `A6_TASK_START` to terminal outcome, with independent success flag; normal return/landing before Ground is included; post-failure cleanup excluded |
| Clock | Simulation time for all robot efficiency; wall time and real-time factor only for compute budgeting/runtime diagnosis. Reset/missing boundary means missing time. |
| Distance | **Not an inferential metric or claimed saving in this release.** Preserve existing missing/partial records; never substitute lower bounds or endpoint displacement for full flight/Ground path. |

Counts/times/packet poses available in existing events/NPZ/TF are reduced offline; no new
sensor subscription or trajectory estimator is required here. Validate the
location-change reducer on saved records before final use; below-threshold
motion is not claimed absent. Sensor alignment drift and yaw can matter even
when the reporting category says no resolved location change.

Keep mechanism diagnostics: raw grid blocking → object-aware operational
blocking → confirmation → D_exec → retrieval; TARGET/ENVIRONMENT/AMBIGUOUS,
exact winners/non-winner alternatives, forecast opportunity versus actual
ground-presence increments and remaining deficits. These explain failures,
never add votes, ignore occupied geometry or modify the tested selection.

## 5. Primary analysis, precision and efficiency interpretation

The unit is one independent **paired scene**, not a view/candidate/repeated
state. Let b=Ours-only successes, c=Generic-only successes, a=both successes,
d=both failures. Report all four counts and marginal successes. Primary effect
is Δ=(b−c)/96, Ours minus Generic. Because tiers were sampled IID from the fixed
mixture, the primary target is its marginal effect; do not change to fixed-tier
weights after seeing results. Tier-specific outcomes/intervals expose heterogeneity
but are descriptive and not three extra primary tests.

Use one two-sided **exact McNemar test**, α=.05, conditional on b+c discordances;
p=1 if there are none. This is a paired-binomial test, not an independent two-
proportion test. Official [statsmodels documentation](https://www.statsmodels.org/stable/generated/statsmodels.stats.contingency_tables.mcnemar.html)
specifies the exact-binomial option; [SciPy binomtest](https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.binomtest.html)
provides the underlying exact test/interval functionality already available.

Also report a boundary-safe conservative95% Δ interval: obtain97.5% two-sided
Clopper–Pearson intervals `[L_b,U_b]` for b/96 and `[L_c,U_c]` for c/96, then
`[L_b−U_c,U_b−L_c]`. Bonferroni gives at least95% joint coverage without assuming
the two discordant categories independent. This bound can be considerably wider
than a normal interval; label any paired normal/scene-bootstrap interval as an
approximate sensitivity, not a replacement chosen for significance. Existing
`paired_statistics` on three fixed tiers is **not** this primary calculation.

### Why96 pairs, and what they cannot establish

Use exact planning power with D~Binomial(N,ρ), B|D~Binomial(D,(ρ+Δ)/(2ρ)), where
ρ is discordance probability. These are illustrative homogeneous mixture
planning scenarios, **not fitted from the one development discordance**.

| Δ | ρ | Exact test power atN=96 |
|---:|---:|---:|
| +.10 | .20 | .5249 |
| +.15 | .30 | .7262 |
| +.20 | .30 | .9538 |
| +.20 | .40 | .8621 |
| +.25 | .40 | .9781 |

Reproduce without starting a robot:

```bash
PYTHONPATH=scripts OPENBLAS_NUM_THREADS=1 \
  /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python -c \
  'from analyze_a6_formal import planning_power; print(planning_power(96,.20,.40))'
```

This is a resource-bounded study sensitive to moderate/large differences, not a
promise to detect a5–10 percentage-point gain. Test power is not the probability
that the conservative interval excludes0. Do not increase N after inspecting
discordances or stop early for apparent efficacy. If uncertainty remains wide,
the answer is inconclusive; a larger independent study is a separate decision.

For “similar success,” predeclare a **±5 percentage-point practical band** (one
task in20), not a success/physics threshold. Only a95% Δ interval wholly inside
that band supports the stated precision of similarity; p>.05 alone never does.
All96 pairs concordant gives about±4.46pp with the chosen bound. Exhaustive
enumeration of the4,753 feasible(b,c) tables confirms that **only b=c=0** can
meet the±5pp criterion atN=96 with this conservative procedure. Even one
discordance makes that precision claim unresolved; do not broaden the band or
switch interval procedures after results. This study is not generally powered
to establish success equivalence; conditional time estimates can still be
reported honestly without an equal-success claim.

Secondary efficiency is estimation, not a second uncorrected primary test.
Report paired windows/location changes/active/Ground/whole-task time on **jointly
successful scenes**, with the joint-success fraction and paired mean/median
differences; use10,000 whole-scene paired bootstrap resamples (fixed seed
2026091024) for descriptive intervals, flag sparse/degenerate samples. This is
explicitly conditional on both succeeding, not an unbiased all-task cost effect.
Do not count either member of a discordant failure as faster successful retrieval.

Also show empirical `P(retrieval AND windows≤k)` for k=1,2,3 and
`P(retrieval AND active_time≤t)` / `P(retrieval AND task_time≤t)` using **all96
scenes per method**. Failure contributes no retrieval at any t, not a fast event
or noninformative censoring. Timing-missing successes give lower/upper curve
bounds, not dropped denominators. No runtime is stopped at a plotting threshold.
Curves distinguish observed success-resource tradeoffs from survivor-only means;
they are descriptive, not multiple pointwise significance searches.

Claim wording follows evidence: superiority/worse/inconclusive on primary
success; optionally lower conditional observation cost with adequately bounded
similarity and a consistent all-task tradeoff. Decompose Ground camera/navigation
time before attributing a whole-task difference to aerial weighting. Do not call
shared platform repairs an Ours contribution. Auxiliary controls/ablations show
raw paired tables/diagnostics only; eight ablation runs cannot establish their
universal necessity. No distance, energy or unmeasured-efficiency claim.

## 6. Failures, software fixes and incomplete data

Preserve original attempts in unique directories. Record first terminal cause
plus downstream NOT_REACHED: aerial detection, capture/hover acceptance, no inverse
support, operational collision, insufficient real ground support/budget, preview
rejection, navigation/stop, D435 visibility/refine, IK/planning, descent/contact,
lift/retention. A valid scene can be physically unreachable. Finite scan error,
incomplete sensing, conservative model refusal and bounded solver/camera-search
failures are normal capability outcomes unless a specific software defect is
demonstrated. Same treatment for both methods.

`INVALID_TRIAL` requires a concrete contract/data defect: simulator startup
failure before usable task execution, wrong loaded configuration/frame, broken
process/communication, clock reset, or loss of evidence required to determine
primary success. A normal planning timeout or missing candidate is not INVALID.
Missing **secondary distance/timing** alone does not invalidate known physical
success/failure. Ambiguous root cause is not automatically a license to rerun.
Use existing logs/focused offline reproduction, with method label hidden where
practical during classification; report supporting reason and affected stage.

For a confirmed infrastructure-invalid slot, at most one same-scene/method
replacement, charged to the12-start reserve. Keep the original result and first-
activation sensitivity (invalid counted unsuccessful); never replace valid
failures. Exhausted reserve/remaining unclassifiable slots means incomplete study,
not success by dropping them. Show missing-pair counts and best/worst primary
effect bounds rather than silently computing a complete-pair selected estimate.

No proactive method changes during final collection. A real software/recording
bug gets a minimal versioned fix and affected-scope explanation. If runtime or
loaded configuration changes, both methods of an affected pair must use the
same replacement version; a broad shared fix ends the old homogeneous cohort.
Do not silently pool versions. A logging-only/offline reducer correction may
recompute preserved raw records if it does not change robot decisions/outcomes.
Old failures stay accessible even when excluded for a proven defect. If a fix
requires more than the declared reserve or creates a new study, stop and report
the resource/protocol impact instead of reusing final data for tuning.

## 7. Compute and data budget, without new infrastructure

One existing workstation, one Gazebo task at a time. Current twelve-task launches
average329.7 wall s (median335.0,max413.4). At that descriptive rate212 tasks take
19.4h; reserve224 takes20.5h. Plan **24–36 wall h**, with a120h total scheduling
cap including diagnostics; per-task1200s guard plus startup/cleanup remain.
These are not timing guarantees for unseen scenes. No extra machine/GPU purchase,
new framework or auto-expanding matrix.

Current data disk has about635GiB free; root has only3.5GiB. Write future data to
the existing data disk, not root. Budget **≤500GiB new retained data**, preserve
≥100GiB free, and pause collection before a new pair if these limits cannot hold.
Disk exhaustion is an incomplete resource-limited study, not scene exclusion.

Use identical current recording during every task. After a completed pair, while
no simulator is running, ordinary lossless gzip of the new native dynamics CSV
reduces storage without altering recordings/robot timing. A read-only stream
measurement of one existing1.3GiB CSV produced238,562,303 compressed bytes at
gzip-1; the old file was not changed. This is a budget example, not an assumed
compression guarantee. Keep all events, configs, endpoint NPZs, rankings/decisions,
physical checker output, native nonimage TF/joint/control bags and CSV data.

Full RGB-D bags are retained for the predeclared first2 scene IDs in every tier
and **all their methods**, and for every failure/invalid/unresolved task. For
other new successful tasks they are temporary diagnostics: retain through offline
physical/stage/missingness review and export the representative observation/
refine frames, then omit only that declared temporary bag from long-term storage.
Never apply this policy to historical development outputs. Both methods are
recorded identically at runtime, irrespective of later retention. Keep a simple
per-task note of retained files; no artifact/provenance service or lock system.
At current approximate sizes, compressed CSV+nonimage data for224 starts plus
about28 predefined RGB-D tasks is roughly340GiB, leaving room for failures and
temporary files inside the500GiB cap. Check actual growth; do not promise an
unbounded failure-bag allowance or delete unresolved evidence to finish.

## 8. Before activation and deliverables

This turn produces version/protocol documents, dependency/PR organization and
offline verification only. No final scene is test-flown. Before later formal
activation: review this proposal, generate the complete independent manifest
and task order, implement only the small explicit final-cohort launcher label/
offline metric reducer needed by the old development-only flag guards, and test
them without changing execution semantics. Existing `run_retrieval.py` is the
working single-task entry; other existing controls use `run_a6_attempt.py` with
explicit slots/flags. The old formal CLI/analyzer hardcodes a different cohort;
never feed this proposal to it unchanged or claim that a runnable new matrix
already exists. A separate existing development scene is sufficient if a runtime
smoke test becomes necessary; its outcome never enters the96 final scenes.

Final report: complete paired outcome table; primary counts/effect/interval/test;
tier coverage; stage/failure distribution; measured window/location changes,
active versus Ground timings and missingness; shared-state scores and forecast/
real-support deficits; control/ablation results; invalids/replacements/version
effects; and claim limits. All historical pilots, interrupted16 v1.1 slots and
recent development batches remain separate. **Do not start the matrix as part
of this documentation delivery.**
