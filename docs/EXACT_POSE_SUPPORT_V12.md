# v1.2 exact-pose support anchoring — review checkpoint

**Update:** the subsequent [v1.2 integration-complete checkpoint](V12_INTEGRATION_CHECKPOINT.md)
passes the original natural A5 E2E after a bounded TF-readiness repair. The
review below records the earlier9f3a527 checkpoint and its failures unchanged.
No formal restart or multi-support revision follows the new integration pass.

Scope: the user-authorized A3/A5 anchor revision, synthetic/saved-state tests,
and the original natural A5 regression. This is not a formal experiment restart.
**Core and recorded-state checks pass; natural E2E acceptance does not pass.**
The activated natural run stops on the original hover-stability check before
its first MID360 window. v1.2 is not declared fully accepted or formal-ready.
The 16 v1.1 formal outcomes remain interrupted-development evidence with their
original classifications; none enters final v1.2 statistics. Slot 17 is not run.

## Definition and unchanged boundaries

For each A1 feasible (HIGH/LOW) cell C, let I(C) contain the original evaluated
candidates in that cell that pass the unchanged A1 validity gate. Recover

```
i*(C) = first original evaluation index attaining max_{i in I(C)} v_i
q*(C) = (x_i*, y_i*, yaw_i*)
R(C) = v_i*
```

Ties use the original evaluation order, including saturated relevance ties.
No environment evidence, outcome, ID sorting, travel, residual, or larger raw
margin beyond saturation participates. Reconstruction checks feasible-cell
coverage and agreement with A1 relevance and best_yaw (absolute 1e-12 floating
roundoff, not a physical tolerance). No new IK is claimed: the anchor is the
original pose accepted by the frozen candidate validation, not the cell center.

With the unchanged padded BUNKER footprint F and v1.1 operational gate B:

```
M_nominal(x)     = max_{C: x intersects F(q*(C))} R(C)
M_operational(x) = max_{C: x intersects F(q*(C)), not B(q*(C))} R(C)
U_task(x)        = unknown_score(x) * M_operational(x)
```

The existing grid projection, max/tie aggregation, clipping, unsupported-cell
states, and no-smoothing rules remain. Only the footprint anchor changes.
ENVIRONMENT/AMBIGUOUS evidence and true continuous target collision still block;
UNKNOWN does not block. A5 confirmation additionally requires actual ground
votes for every covered cell and no clipping. A cell is never made FREE or
given votes by this revision. `blocked` is the operational evidence gate, not
a complete navigation/MoveIt feasibility judgment.

A1, A2, v1.1 association and allowance calculation, A4, RM4D assets, and SIM
source are unchanged. Generic/Ours share the same anchoring and operational
gate. Their only primary policy difference remains generic versus task gain.
A4 continues to compute `(1-exp(-1/tau)) * sum(V * U_task)` minus flight cost.
An anchor change may change its task input and ranking, not its scoring formula.

## API and reproduction

`reconstruct_winner_anchors(field, evaluated_candidates)` returns immutable
`WinnerAnchor(source_id, candidate_id, evaluation_index, x, y, yaw, relevance)`
records in original evaluation-index order. `source_id` remains the A1 flat cell
ID. Supply the complete original evaluated-candidate sequence.

`build_task_uncertainty(..., evaluated_candidates=...)` uses exact anchors;
omitting this argument retains historical center semantics. A5's
`A5Config(support_anchor='exact_winner')` and the runtime option
`--support-anchor exact_winner` select v1.2 explicitly. Historical configuration
without the option continues to use `cell_center`; old trials are not silently
reinterpreted. Existing `PoseSupport` fields remain compatible.

The shared `build_support_task` is used for decisions and saved A3/A4 outputs
by both A5 and A6 workers. An exact task must agree with the full exact candidate
catalog before A5 assessment. Saved summaries add
`anchor_semantics='exact-validated-winner-v1.2'`, winner identities/poses,
and the tie rule. No non-winner is promoted by diagnostics.

## Recorded Hard-002 regression

The [read-only replay report](../outputs/a6/exact-anchor-v12/recorded-replay/replay.json)
covers all three saved windows of both slot 15 Generic and slot 16 Fixed.
It reproduces legacy A2 arrays, derived operational arrays, A3 arrays/poses,
exact assessments, and A4 ranking before changing only the anchor.
New outputs are separate from the original trial directories.

| Saved state | Blocked support poses, v1.1 → v1.2 | Positive operational cells, v1.1 → v1.2 | Confirmed, v1.1 → v1.2 |
| --- | ---: | ---: | ---: |
| Hard-002 Generic, each window | 34 → 33 | 0 → 88 | 0 → 0 |
| Hard-002 Fixed, each window | 32 → 33 | 119 → 107 | 0 → 0 |

For Generic source 624, exact xy `(2.370081136, -0.547832936)` replaces center
`(2.369065100, -0.507282760)`, eliminating the approximately 40.6 mm displacement
and its false continuous collision. The original exact gate is already clear;
v1.2 removes the inconsistent center veto. After window 3 it still has **87/88
ground-supported cells**, with cell 739 unsupported, so confirmation stays false.
Its operational footprint returns to the uncertainty field; this permits
further informative sensing in principle, not a claim of successful handoff
within the recorded three-window budget.

Generic-state task uncertainty mass becomes 88.0000, 53.7682, and 33.0055 over
the three recorded windows. Fixed-state support instead shrinks: exact anchoring
is not a one-way relaxation. Fixed source 624 remains AMBIGUOUS-blocked despite
no continuous target intersection. No ambiguous evidence is relabeled.

[Final Generic support/uncertainty visualization](../outputs/a6/exact-anchor-v12/recorded-replay/slot-015-hard-002-generic-01/round-03/support-comparison.png)
compares the two anchor definitions on identical perception data.

Re-ranking preserves candidate viewpoints, visibility, delta-unknown, flight
cost, generic gain/order, and all exact operational assessments. The new task
gain satisfies the unchanged A4 formula. These are saved-state diagnostics,
not hypothetical v1.2 executed trajectories or corrected trial outcomes.

### Diagnostic only: viable non-winners

The same-cell winner-blocked/non-winner-unblocked test finds, in each snapshot:

| Recorded attempt | Cells | Unblocked non-winners | Source cells (alternative counts) |
| --- | ---: | ---: | --- |
| Generic | 3 | 7 | 543 (4), 581 (2), 621 (1) |
| Fixed | 2 | 4 | 624 (2), 581 (2) |

Repeated snapshots are not independent observations and are not summed.
These are geometric-gate-clear alternatives, not necessarily confirmed or
executable candidates. Their IDs, relevance and poses are retained in the
report but never fed back into selection. Multi-support semantics would be a
separate research revision; none is implemented or assumed necessary here.

## Natural regression and verification

The natural run uses the original natural A5 scene/settings documented in
[v1.1 reproduction instructions](OBJECT_AWARE_GATING_V11.md#regression-boundaries),
with only `--support-anchor exact_winner` added to the v1.1 invocation and a new
output directory. It is not a new pilot or a formal sample. The following
startup-only orchestration repair was necessary; sensing, scoring, budget,
navigation and manipulation settings were not changed.

| Launch | Task activation | Result |
| --- | --- | --- |
| natural-a5-attempt-01 | No | Invalid startup: 2 s diagnostic connection timeout |
| natural-a5-attempt-02 | No | Invalid startup: same issue persists with 10 s wait |
| natural-a5-attempt-03 | Yes | Genuine regression failure: UAV did not settle before first MID360 window |

The activated [adapter log](../outputs/a6/exact-anchor-v12/natural-a5-attempt-03/adapter.log)
and [raw physical checker result](../outputs/a6/exact-anchor-v12/natural-a5-attempt-03/physical_summary.json)
record the failed run. Initial view remains `(-.5,0,1.5,0)`, BUNKER `(3,-2.5,pi)`,
three 5 s windows allowed, .05 m flight/facade tolerance and 120 s navigation
guard. The saved initial config selects `exact_winner` and v1.1 evidence.

Observed sequence (simulation seconds, not wall-efficiency inference):

- PREFLIGHT 30.769; TAKEOFF 30.776; AIR_HANDOFF 53.380.
- RM4D returns 144 evaluations, 136 gate-valid candidates and 34 feasible cells.
- At 74.209, the first capture anchor is measured at
  `(-.475320344,-.049913570,1.111257785,.002283183)`, .392710 m from the requested
  view. This is recorded public pose, not an applied offset.
- The unchanged `_a5_capture` freezes that measured anchor, then calls the
  unchanged `_a5_wait_settled`; after its 45 s wall guard, FAILED is published
  at 112.984 sim s. Existing failure cleanup lands the UAV; owned runtime exits.

Completed MID360 windows: **0**. A3/A4 decision, candidate confirmation, Ground
handoff/navigation, Ground D435 refinement, D_exec and retrieval: **not reached**. Retrieval success
is false. This failure is retained, not relabeled INVALID_TRIAL, retried until
green, or attributed to exact support gating, which has not executed yet.
The log establishes the failing integration stage, but does not establish the
full root cause of the UAV's inability to settle: no per-sample pose/velocity
diagnostic was retained. In particular the nonzero anchor discrepancy alone
does not prove bad TF, a controller defect, or which settle predicate failed.
No change to the frozen capture/hover semantics or thresholds is made here.

### Engineering findings, separate from method outcomes

The first two natural launches failed **before adapter.run()**, without robot
task activation. Their adapter logs and raw checker summaries are retained in
`natural-a5-attempt-01` and `natural-a5-attempt-02`. The checker reports its generic
LIFT timeout on cancellation; neither launch is a genuine retrieval failure.
The first diagnostic connection wait expired after 2 s. Reusing A6's existing
10 s startup wait also failed on launch 2, so that attempt is not described as
a successful timeout repair.

Root cause: A5 released the inherited status publisher's last handle before
creating its replacement on the same topic/URI. Local rospy source and the
saved ROS master log show the intervening unregister/register, which can strand
the checker's connection attempt. The minimal fix acquires the new handle
before releasing the old one, preserving the shared topic implementation. The
optional startup connection wait remains bounded at A6's existing 10 s wall
time, before any task timing or actions.

Existing ROS connections retain their old queue size; acquiring a new handle
does not resize them. Therefore launch 3 starts the existing passive checker
after the adapter's `A5 adapter ready; waiting for status subscriber` log and
before the adapter leaves its startup wait. This supplies queue size 10 to the
new connection without changing payloads, robot behavior or SIM source.

The offline replay initially followed a historical absolute RGB-D reference
path. It now binds an in-memory initial-data copy to the selected attempt's
bundled reference; a relocated-checkout/stale-path test reproduces and fixes
that issue without rewriting original JSON or association inputs.

### Verification and review

Core implementation `8aea2de` follows the committed design `eb8824e` and passes
independent specification and code-quality review. The separate replay path
fix and pre-task publisher repair also received focused independent review.
TDD covered feature-absence failures, input-validation failures, relocated
reference failure, delayed connection and publisher acquisition/release order.

Fresh full core suite: **538 tests, 21 environment-dependent skips, no failures**
(56.222 s). [Test log](../outputs/a6/exact-anchor-v12/core-tests.log).
Separate native Noetic tests: **75 pass** (2.659 s), and actual RM4D task-map
tests: **9 pass** (1.263 s). These overlap the core suite; they are not added
as independent test counts. [Noetic log](../outputs/a6/exact-anchor-v12/noetic-tests.log),
[task-map log](../outputs/a6/exact-anchor-v12/task-map-tests.log).
The 6 Hard snapshots reproduce old arrays and scores before showing the exact
anchor differences; worker tests exercise actual A5/A6 saved fields and A4 inputs.
No Gazebo GT is used for algorithm or replay geometry. The existing checker is
evaluation-only.

Source-boundary comparison to `699b701` is empty for A1, A2, v1.1 operational
gating, A4, A6 policy formulas, configs/assets and every original Pilot-1,
Pilot-2, v1.1 regression and formal output. SIM and frozen RM4D worktrees are
clean. Only new regression output is added. The four preexisting unrelated
untracked A5 diagnostic directories are left alone.

Checkpoint decision: stop for review with the natural acceptance gap explicit.
Do not launch slot 17, another pilot, formal matrix, or multi-support revision.
The saved-state geometric correction is supported; successful live end-to-end
validation of the new anchor has not yet been established by this run.
