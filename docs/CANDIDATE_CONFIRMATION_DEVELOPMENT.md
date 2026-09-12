# Candidate-confirmation-oriented development v1

2026-09-12. Starts from `3c8b040`; historical evaluation and analysis remain
unchanged. New branch `feature/dev-candidate-confirmation-v1` in the requested
main AGENT checkout, retaining local sensor assets/results without duplicating
them into another worktree. SIM stays at `a0ae8e3` unless a demonstrated generic
platform defect requires a separately recorded fix. No formal matrix.

## Design and alternatives

1. **Chosen: bounded receding-horizon exact-candidate confirmation.** Reuse the
   saved/shared finite-scan opportunity and candidate-to-footprint incidence.
   Search one/two future windows in the current view catalogue, with the existing
   local generation and facade transition constraints. Execute only the first
   action; recompute on the next real observation. Fast, directly testable, but
   restricted graph and fixed nominal geometry remain limitations.
2. Full expanding lattice and all joint packet phases: potentially finds more
   plans, but adds prediction cost and model dependence before the bounded
   version has live evidence. Not implemented in this batch.
3. One-step missing-support reduction: kept as same-state diagnostic to separate
   candidate-oriented progress from two-step completion. It need not be flown
   separately in this first eight-task batch.

For eligible original exact winners q, ground counts H and cells S_q, require
`all(H[x] + sum(future_hits[x]) >= 2 for x in S_q)` for at least one q.
One future window contributes at most one vote/cell. Use all-phase `w==1` and
optimistic `w>0` envelopes, not independent probabilities. No belief mutation.
Current blockers and perceived geometry remain unchanged during prediction.

Order plans lexicographically by: (1) all-phase completion before optimistic
completion; (2) fewer observation windows; (3) existing displacement/yaw cost
sum; (4) stable viewpoint IDs. Existing flight weight is positive and common;
scaling the tie-break cost by it does not change this order. This is a new
completion objective, not the old A4 weighted-sum score or a calibrated expected
retrieval probability. Log which exact candidates each nominated plan supports.

No-plan distinctions:

- `SUPPORT_BUDGET_IMPOSSIBLE`: at least one current operationally viable winner
  exists but every such winner contains a real deficit exceeding remaining
  windows. Stop sensing; do not alter votes or declare all navigation infeasible.
- `NO_NOMINAL_PLAN`: counts permit completion, but restricted nominal search
  finds none. Delegate action/stopping to unchanged Ours, explicitly recorded as
  fallback, not a physical impossibility proof.
- No eligible candidates: record `NO_CURRENT_SUPPORT`; delegate to Ours.
- Already confirmed: preserve Ours and the shared collision-aware execution
  screen/handoff. A failed execution screen does not get a fabricated new grasp.
- Budget exhausted: no extra window. Ground handoff still runs normally first.

The fallback is defined before online results and used for every scene. It
preserves Ours when this predictor cannot justify an alternative; it is not a
scene-specific retry. Confirmed support is not D_exec or retrieval.

## Bounded online development batch

Eight planned full sensor-to-retrieval tasks, maximum ten starts including all
startup failures. Two reserve starts only for explicit runtime/recording defect
diagnosis; no normal failure retry. Wall ceiling six hours from first start,
new storage cap 80 GiB, existing disk must retain at least 100 GiB free. Check
resources before each start; do not delete historical/unresolved data.

Use unchanged explicit scene geometry from `configs/paper1_eval.json` with the
current shared development task profile. These are intentionally selected OLD
development cases, not independent final-test data. Selection covers successful
Ours, a prior nominal overprediction, a Generic/Ours discordance and Ours failure;
no task outcome is used by the online algorithm.

| Global slots | Existing scene | Order | Development purpose |
|---|---|---|---|
| 1–2 | eval-easy-003 | ours, confirmation | both-success regression, nominal overprediction |
| 3–4 | eval-moderate-009 | confirmation, ours | successful manipulation, observation-model discrepancy |
| 5–6 | eval-hard-015 | ours, confirmation | historical Generic failure / Ours success |
| 7–8 | eval-hard-023 | confirmation, ours | historical Ours confirmation failure / no nominal plan |

Same version and shared task profile within each pair; no manual station, GT,
increased budget, different 12 mm margin or physical success predicate. Generic
is retained as historical and same-state reference, not falsely called an online
same-version run. If code changes during the batch, report exact affected scope
and do not pool mixed-version pairs. No outcome-driven new scenes or starts.

## Validation and deliverable

Replay all available primary historical states, not only Generic failures.
Compare new actions to actual Ours commands and a one-step deficit diagnostic;
report no-plan/budget-impossible separately and model/real calibration limits.
Unit tests cover AND/OR incidence, real deficits, repeat windows, plan length,
phase envelopes, cost ties, facade reachability, no-plan delegation, confirmation
and exhaustion. Original Ours/Generic outputs must stay unchanged.

Online record confirmed → execution-screen → navigation/refine → D_exec →
grasp/lift/retention, first terminal cause, windows, real location changes,
active/ground simulation time, and policy/worker computation wall time. A quick
failure is not an efficiency improvement. Report negative cases and unchanged
capability boundaries. Preserve all original data. Stop after this batch and
review; no automatic further development or large matrix.
