# Candidate confirmation implementation plan

> Agentic execution: use executing-plans inline, under the user's autonomous
> engineering authorization; review code before the bounded online batch.

**Goal:** Implement a separate two-window confirmation strategy and compare
actual full-task outcomes against current Ours in four development scenes.

**Architecture:** New pure NumPy module `src/a6_pilot/confirmation.py` consumes
unchanged ranking/assessment/current counts, returning a choice plus diagnostics.
Route only explicit method `confirmation` through it in existing policy/CLI.
Original Ours and Generic calculations and shared execution stay intact.

**Tech stack:** Existing Python/NumPy/ROS bridge, current SIM, unittest; no new
dependencies. Design and bounded task order: `docs/CANDIDATE_CONFIRMATION_DEVELOPMENT.md`.

## 1. Pure predictor (red, green, replay)

- [x] Add `tests/test_confirmation_policy.py`: import existence assertion;
  `H=[1,1], S=[[0,1]], w=eye(2)` needs two complementary views;
  `H=[0], b=1` reports true budget impossibility; `H=[1],w=0` only nominal no-plan;
  existing complete candidates/empty eligible catalogue do not fabricate actions;
  compare live core bounds to the accepted offline reference over recorded states.
- [x] Run `PYTHONPATH=src:scripts python3 -m unittest discover -s tests -p test_confirmation_policy.py`;
  observe missing-feature failure before implementation.
- [x] Implement `plan_confirmation(h,supports,opportunity,poses,valid,first_cost,
  remaining,offsets,yaw_samples,yaw_cost,facade_tolerance)` in the new module.
  Return status, nominal tier, first/second IDs, length/cost, supported candidate
  indices and same-state one-step deficit ID. The pure core takes no outcomes.
- [x] Test complementary support, prioritization, repeat/same-yaw filtering,
  immutable inputs, exact tie-break and no-plan states. Run relevant existing tests.

## 2. Opt-in runtime routing and logging

- [x] Test development entry accepts `confirmation` and shares the original
  profile exactly; existing evaluation cohort does not gain any new slots.
- [x] Add explicit method to `src/a6_pilot/policy.py`, `scripts/run_a6_sim.py`,
  `scripts/run_retrieval.py`; route after original shared `decide` only for that
  method. Old `ours` return remains exact and historical package is not changed.
- [x] Add worker elapsed-wall timing for all methods around policy evaluation,
  recorded outside the method decision and forwarded in A6_ENV_RESULT. New
  module logs its extra solver time and fallback/plan semantics in decision JSON.
- [x] Reuse old 541 snapshots for read-only policy replay in a new development
  results directory; retain baseline analysis. Check all 80 Ours successes and
  all five Ours confirmation failures, plus the full primary population.

## 3. Bounded online comparison

- [x] Commit implementation/config/design after tests and independent review.
  Prepare eight explicit rows from the already specified scenes and balanced order.
- [x] Execute one existing `run_retrieval.py run` task per row, fresh perception,
  output under `outputs/development/confirmation-v1/slot-NN-*`. No retry on
  method failure; enforce ten-start/six-hour/80-GiB caps with existing checks.
- [x] Reuse existing event/metric readers, adding only field extraction for
  confirmation and computation time. Save per-run paired summary and first causes.
- [x] Review actual outcomes vs claims, run regressions, preserve old results,
  commit/push development checkpoint and report. Do not start another batch.

Result: 8/8 valid tasks, no retries, all runtime `a3c62c4`; both methods 2/4
retrieval and 3/4 confirmation. New method takes an extra Easy window and does
not rescue either Hard case. See `docs/CANDIDATE_CONFIRMATION_DEVELOPMENT_RESULTS.md`.
Keep experimental; no promotion to Ours, no additional batch or formal matrix.
