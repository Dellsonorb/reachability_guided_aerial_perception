# Completion-aware offline validation

**Goal:** Answer four research questions from the 212 selected fixed-evaluation
tasks without changing runtime code, original results, or launching a simulator.
**Architecture:** A standalone NumPy/CSV/JSON analysis reads retained snapshots;
its predictive functions accept only the current snapshot, not outcomes, seeds,
later observations, or simulator truth. Historical outcomes enter reporting only.
**Execution:** Inline, using writing-plans and test-driven-development principles;
the existing workspace/data are retained, with a separate analysis Git branch.

## Fixed analysis definitions (before aggregate counterfactual results)

- Population: all 212 selected tasks; main strata are all 29 Generic primary
  VIEW_BUDGET_REACHED/no-confirmation failures and all 80 Ours primary successes.
  Early missing snapshots and RM4D-only bypass remain accounted for.
- Preserve exact winners and each snapshot's operational blocking, footprint,
  H>=2 threshold, sensor model, candidate bounds and three-window budget.
- At round r, remaining windows b=3-r. A zero-ticket cell after round 2 proves
  that candidate cannot confirm in one remaining window (one vote/cell/window).
- For saved opportunity w, L=1[w==1] and U=1[w>0] are conservative all-phase
  and optimistic any-phase hit envelopes under the existing nominal model.
  Evaluate AND across a candidate's missing cells, OR across eligible candidates.
  U can combine incompatible phases and is not an existence proof; L can miss
  phase-dependent complementary coverage. Neither is a real-return probability.
- Two-step search uses ordered pairs of the CURRENT saved view catalogue only.
  The second view must also belong to the unchanged local generator at the
  first view. This is a restricted same-state graph, not the full future lattice.
  Repeat/stay is permitted. Known blockers are held fixed for prediction;
  future unknown obstacles and realized pose/scan deviations are not predicted.
- Reference offline selection: maximize any-candidate all-phase completion,
  then any-phase completion, then minimize total unchanged displacement/yaw cost,
  then stable view IDs. If no optimistic completion exists, abstain. Record
  strict completion-tier improvement separately from cost/tie-only changes.
  No new flight weight, probability threshold, outcome fitting or vote writes.
- Validate one-step predictions against the NEXT HISTORICAL observation only
  for actually commanded views, identified from A5_DECISION events. Later data
  is never supplied to the predictive function. Count missed and unexpected
  ground hits, predicted-vs-actual candidate confirmation and newly blocked poses.
- Track every exact candidate per round. Report both the current-best deficit
  envelope (can change anchor) and a fixed round-1 prospectively chosen anchor
  (fewest missing tickets, then relevance, then original evaluation index).
- Report whether the recorded first action belongs to an optimal completion
  tier, not merely whether an arbitrary deterministic tie-break changes it.
- This is development/post-hoc diagnosis. Unexecuted views have NO measured
  success labels; no off-policy retrieval effect or new significance test.

## Tasks

- [x] Add failing unit tests for deficit bounds, complementarity, blocked/clipped
  candidates, phase envelopes, generator-limited transitions, cost and tie logic.
  Run: `python3 -m unittest discover -s analysis/completion_aware -p 'test_*.py'`.
- [x] Implement standalone `offline.py`, passing only h, incidence, w and views
  into prediction; add immutable-read/temporal-separation checks in tests.
- [x] Analyze all 212 records; export per-slot, per-round, per-candidate,
  actual-transition, pairing and summary CSV/JSON into this analysis directory.
- [x] Investigate prediction-vs-observation discrepancies without changing the
  scoring definitions. Add limited exact scan-phase reconstruction if envelope
  ambiguity materially prevents answering the questions; declare its scope.
- [x] Write findings and limitations, inspect descriptive trajectory figures,
  run relevant tests and confirm no protected changes; deliver on analysis branch.

No criterion requires attractive outcomes or rescue of any particular scene.

## Implementation/review notes

- A read-only rerun corrected the offline second-leg graph to also match A5's
  facade position/yaw-only filter; tolerance comes from each saved initial config
  (0.05 m), not a new parameter. No online code or original result changed.
- Already-confirmed round-2 states are explicitly marked and excluded from
  unresolved-sensing counts. Candidate-level false positives are reported even
  where a different candidate confirms in the same transition.
- All four optimistic-only nominated failure plans received joint-phase checks.
  No expanded search, phase fitting or outcome-driven plan reselection was used.
