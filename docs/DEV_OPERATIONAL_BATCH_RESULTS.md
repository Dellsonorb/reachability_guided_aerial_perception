# Bounded operational-consistency batch — completed development checkpoint

2026-09-09. **12/12 startup invocations used; 3/3 fresh development pairs
completed. No formal runs.** Two invocations were invalid before method start;
ten were valid method runs. Historical versions/results remain unchanged.

The v1.4 ground-presence repair removes the reproduced evidence/geometry
contradiction. No recurrence was found in this batch. This does **not** establish
E2E readiness: fresh Generic and Ours both retrieve 0/3. Do not end overall
method/integration development or infer superiority from these results.

## Revision and scope

Method implementation checkpoint: `5b09ed9` on
`feature/dev-operational-consistency`; accepted v1.3 `c7824ad` is preserved.
All six fresh paired runs used the same v1.4 method and installed SIM `5e25039`.
The RM4D baseline and task-domain asset were not changed. All seeds and their
order were serialized before outcomes; see [batch plan](DEV_OPERATIONAL_BATCH.md).

The known Moderate replay exposed actual ground returns suppressed solely by
an AMBIGUOUS endpoint in the same coarse cell, although its 33 mm blocker disk
was continuously separate from the footprint. For example source705 has
6/7/4 actual inside-footprint ground points in the relevant cell over the
three windows; nearest AMBIGUOUS points are 94.4/87.5/86.2 mm away. Source623
has 8/8/7 ground points and 85.6/71.2/67.9 mm separation. Legacy votes stayed zero.

v1.4 adds positive **measured** presence, separate from negative collision:

`P(C) = sum_k 1[window k contains an accepted ground-height endpoint in C]`.

Confirmation requires `P(C) >= 2` in every covered cell, no clipping, and no
operational blocker. No obstacle is removed by those ground votes. A2 raw
state/evidence/counts, legacy ground votes, T/E/A association, continuous
TARGET and endpoint-disk AMBIGUOUS collision, ENVIRONMENT blocking, exact
winners, A4 scoring and the three-window budget remain unchanged. Historical
inputs without this optional sidecar retain legacy behavior. See
[mathematics and tests](superpowers/specs/2026-09-09-v14-ground-presence.md).

The 33 mm disk remains a declared engineering envelope conditional on public
map/TF, not calibrated total error. Ground presence is still cell-level sampled
support, not certification of every sub-cell surface. An occupied endpoint can
coexist with genuine ground support; collision remains an independent gate.

## Every invocation (no valid failure replaced)

| Launch | Scenario/method | Version | Outcome |
|---|---|---|---|
| 01 | Wrapper preflight | v1.3 | INVALID: PX4 path missing; no SIM or method start |
| 02 | Known Moderate 405111274 / Ours | v1.3 | VALID: no confirmation in three windows |
| 03 | Known Hard-002 958985919 / Ours | v1.3 | VALID: no confirmation in three windows |
| 04 | Same known Moderate / Ours | v1.4 | VALID: 4 confirmed, D_exec=1, physical retrieval success |
| 05 | Same known Hard-002 / Ours | v1.4 | VALID: 3 confirmed, navigation succeeds, arm observation-position planning fails |
| 06 | Fresh Easy / Generic | v1.4 | VALID failure: D435 height inconsistency |
| 07 | Fresh Easy / Ours | v1.4 | VALID failure: navigation timeout |
| 08 | Fresh Moderate / Ours | v1.4 | VALID failure: D435 observation timeout |
| 09 | Fresh Moderate / Generic | v1.4 | VALID failure: arm observation-position planning |
| 10 | Fresh Hard / Generic | v1.4 | VALID failure: insufficient measured support, budget reached |
| 11 | Fresh Hard / Ours | v1.4 | INVALID: PX4 startup return 2, public state/TF readiness never reached; no method start |
| 12 | Same fresh Hard / Ours | v1.4 | VALID failure: insufficient measured support, budget reached |

Launch12 replaces only the invalid startup11. It uses the original seed and
unchanged configuration. It consumes the final reserve; the proposed focused
online navigation fix check was cancelled. No additional Gazebo was launched.
Each attempt has its own [record directory](../outputs/development/operational-batch).

Known regressions are not fresh evaluation: actual v1.3 confirmation trajectories
were 0/0/0 for both. Actual v1.4 online trajectories are Moderate 0/3/4 and
Hard-002 0/0/3. Hard-002 source624 becomes confirmed with **99/99** supported
cells. The successful Moderate physical checker reports CHECKS_PASS and target
lift **0.148671 m** with grasp/retention checks passing. Offline counterfactual
v1.4 replays of the original v1.3 observations give 0/3/3 and 0/0/1 respectively;
those are not new trials and do not overwrite the original failures.

## Fresh paired results

`D_env` means at least one confirmed exact candidate. `D_exec` additionally
requires actual navigation, fresh D435 refine and collision-aware continuation
to verified pregrasp. Neither alone is physical retrieval success.

| Scene seed | Method | Candidates | Raw blocked → operational blocked → confirmed (last window) | D_env / D_exec / retrieval | First confirmation window |
|---|---|---:|---|---|---:|
| Easy 1026751387 | Generic | 33 | 33 → 28 → 5 | 1 / 0 / 0 | 3 |
| Easy 1026751387 | Ours | 33 | 33 → 28 → 5 | 1 / 0 / 0 | 3 |
| Moderate 1026806174 | Generic | 34 | 32 → 29 → 5 | 1 / 0 / 0 | 3 |
| Moderate 1026806174 | Ours | 34 | 32 → 30 → 3 | 1 / 0 / 0 | 2 |
| Hard 1448047368 | Generic | 33 | 28 → 28 → 0 | 0 / 0 / 0 | — |
| Hard 1448047368 | Ours | 34 | 31 → 29 → 0 | 0 / 0 / 0 | — |

Both methods: D_env **2/3**, D_exec **0/3**, retrieval **0/3**. Paired binary
retrieval outcomes: three both-fail pairs, **zero discordant pairs**. No
effectiveness inference or sample-size recommendation from this development batch.

All six runs completed/voted three windows with no discarded windows. Easy
Generic used one NBV move and one same-pose rescan; the other five used two
NBV moves. Times below use simulation time, not wall-clock startup/compute time.
Active time includes capture, compute, settling and inter-view flight. First
confirmation time is measured from active start, not an early-stop event.

| Scene/method | Active UAV distance m | Active time s | First confirmation s | Total UAV distance m | Navigation time s |
|---|---:|---:|---:|---:|---:|
| Easy Generic | unavailable (observed lower bound 3.339) | 50.416 | 50.415 | unavailable | 24.621, succeeded |
| Easy Ours | 7.039 | 62.096 | 62.094 | 14.644 | 120.007, failed |
| Moderate Generic | 4.224 | 54.227 | 54.226 | 12.374 | 34.215, succeeded |
| Moderate Ours | 3.375 | 52.976 | 41.931 | 10.743 | 36.207, succeeded |
| Hard Generic | 4.379 | 48.868 | — | 8.452 | not reached |
| Hard Ours | 3.479 | 51.193 | — | 7.201 | not reached |

Easy Generic has one missing active TF sample; its lower bound is not a complete
path estimate. Some total Ground trajectories also have missing samples/gaps;
their original `metrics.json` keeps distance null and observed segments separate.
Efficiency is mixed; Ours does not uniformly improve it. Total-UAV measurement
endpoints follow the existing metrics policy and should not be conflated with
the active-only phase.

## Mechanism diagnostics and remaining structural questions

Final association **vote sums**, not exclusive point counts or disjoint cells:

| Run | TARGET / ENVIRONMENT / AMBIGUOUS | Legacy ground / new measured presence |
|---|---|---|
| Easy Generic | 13 / 0 / 19 | 4045 / 4062 |
| Easy Ours | 12 / 0 / 19 | 2891 / 2905 |
| Moderate Generic | 10 / 88 / 20 | 3089 / 3134 |
| Moderate Ours | 19 / 98 / 23 | 2493 / 2513 |
| Hard Generic | 5 / 82 / 6 | 2551 / 2585 |
| Hard Ours | 11 / 84 / 17 | 1915 / 1927 |

Full candidate-level blocker causes, TARGET alias retention and stage outcomes
are in [mechanism.json](../outputs/development/operational-batch/reports/mechanism.json).
Both known and fresh replays preserve real target collisions; no evidence was
deleted or converted to FREE. ENVIRONMENT blocking remains coarse and conservative;
this batch does not demonstrate an ENVIRONMENT-induced permanent handoff deadlock.

Independent raw-point binning reproduces both Hard presence grids exactly.
Generic's five viable winners miss **20/27/16/19/25** ground-support cells;
each has 6–15 cells with no ground endpoint in any window. Ours' five viable
winners miss **5/5/11/21/25**; the best two each have five cells observed in
only one window. Their final best task scores remain positive, 25.690 and
24.421 respectively. This is insufficient measured support under the budget,
not proof of absent physical ground or another lost-vote defect. Predicted
visibility/gain does not guarantee an actual informative ground endpoint.

Non-winner alternatives remain diagnostics, not reselection:

| Pair member | Blocked-winner cells with clear alternatives | Clear non-winners / ground-supported |
|---|---:|---:|
| Easy Generic / Ours | 3 / 3 | 6/6 and 6/6 |
| Moderate Generic / Ours | 2 / 3 | 5/5 and 8/5 |
| Hard Generic / Ours | 2 / 2 | 3/0 and 3/0 |

These alternatives are not proven executable. Hard alternatives still miss
18/19/24 (Generic) and 7/9/28 (Ours) support cells. Thus per-cell winner
compression loses options, but it alone does not explain these Hard failures.
No evidence here justifies silently switching to multi-support selection.

## Execution failure review (runtime/perception only)

- **Easy Generic:** D435 center height 0.013816 m fails the unchanged expected
  0.0575 ± 0.03 m gate. Registered RGB/depth contain a bottom-clipped red
  vertical surface. Treating its highest sampled points as a full brick top
  and subtracting half-height reproduces the estimate. Public-TF gray ground
  is around 2.45 mm, not a 44 mm global height bias. Partial visibility/top
  estimation is the immediate issue; exact occlusion cause remains unproven.
- **Easy Ours:** valid 120 s navigation timeout. Minimum XY error 0.0580 m,
  final 0.0614 m; yaw never simultaneously satisfies the 0.08 rad gate. A
  concrete unused latch parameter namespace is diagnosed separately below.
- **Moderate Ours:** arm observation positioning succeeds; the subsequent
  190 saved RGB frames have zero red-mask pixels and 190 observer rejections.
  Missing visibility, not a stale-pose or height-gate rejection. Occlusion
  versus outside-FOV geometry has not yet been disambiguated.
- **Moderate Generic and known Hard-002 v1.4:** the text “refined pregrasp
  planning failed” occurs while positioning for observation from the **aerial**
  estimate, before waiting for D435/GROUND_REFINED. Fresh D435 tracking poses
  exist. No arm trajectory, Cartesian-path or OMPL request is recorded during
  this step; failed grasp-IK attempts are consistent with the trace. Missing
  service response codes prevent separating collision from no-solution causes.
  Do not relabel these as sensor timeout or a proven platform defect.

All these method/execution failures remain VALID. No navigation/refine/grasp
threshold, collision condition or success criterion was relaxed.

### Separate SIM configuration correction — offline only

SIM `feature/fix-ground-latch-namespace @ ece36ec` ([Draft PR #2](https://github.com/Dellsonorb/Simulation-Platform/pull/2)) moves the existing intended `true` latch
parameter from the unused DWA child namespace to the node-private namespace
read by the installed ROS navigation 1.17.3 controller. Other navigation
settings are unchanged. See the upstream [constructor](https://github.com/ros-planning/navigation/blob/1.17.3/base_local_planner/src/latched_stop_rotate_controller.cpp#L22)
and [DWA construction/reset](https://github.com/ros-planning/navigation/blob/1.17.3/dwa_local_planner/src/dwa_planner_ros.cpp#L87).

The focused test fails before and passes after the YAML correction. **It is
not installed into this batch's runtime and has not passed an online navigation
regression.** Global replanning resets the latch; namespace correction alone
may not cure oscillation. The final reserve was used for invalid startup11.
No claim of repaired navigation or retroactive success is made.

## Review, reproducibility and disposition

Independent review confirmed:

- All three pairs have byte-identical serialized runtime launch files and the
  same scene/config/initialization. Runtime perception and later states differ
  naturally; no claim of identical clouds across independent closed-loop runs.
- All **30** recorded rounds reproduce actual-version exact assessments;
  all **24 v1.4** rounds reproduce field/evidence arrays (NaNs equal), A3
  poses/summaries and ranked candidates. A2 and legacy evidence are unchanged.
- Same-state Generic/Ours identity/argmax checks pass with maximum error **0**;
  different argmax in **14/18 fresh snapshots** (26/30 including known cases).
  Each comparison uses one common candidate tensor, visibility and cost, only
  changing gain weighting. Actual saved actions match their policy ranking.
  [Full scores](../outputs/development/operational-batch/reports/scoring.json).

Fresh verification: core **616 tests, 22 skips**, native-ROS focused suite
**146 tests, 1 skip**, SIM non-ROS navigation **60 tests**, SIM public platform
contracts **80 tests** passed. Suites overlap and must not be added together.
A broad unsourced SIM discovery first encountered the ROS-only lifecycle test's
missing `rospy`; the proper non-ROS suite was then selected. No ROS lifecycle
or online navigation regression is claimed for the source-only SIM patch.

Replay using the existing script, for example:

```bash
PYTHONPATH=src:scripts python scripts/replay_v13_operational.py --include-v14 \
  --data-dir outputs/development/operational-batch/launch-12-hard-ours/data \
  --output-dir /tmp/dev-hard-replay-new
```

The original observations, outputs, metrics and replays are delivered in the
repository; large raw diagnostic/image bags remain locally in attempt dirs
and are not Git payloads. Selected runtime/adapter logs accompany the failures.
Visuals: [successful known Moderate](../outputs/development/operational-batch/replay-04-v14/round-03/nbv.png),
[fresh Hard Ours](../outputs/development/operational-batch/replay-12-v14/round-03/nbv.png).
These are plots of saved observations and surrogate predictions, not extra runs.

**Disposition:** freeze the v1.4 consistency repair as a development checkpoint;
stop this batch. Remaining work is targeted execution robustness (navigation
parameter deployment/check, D435 visibility/partial-surface estimation, and
precise preposition IK failure diagnostics), plus continued honest assessment
of sparse endpoint acquisition. Overall development is not finished. No new
formal matrix, final-test reuse, method-win filtering, budget extension or
additional acceptance framework was introduced.
