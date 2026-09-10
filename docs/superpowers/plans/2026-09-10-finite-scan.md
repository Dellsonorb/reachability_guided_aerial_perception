# Finite Scan Acquisition Implementation Plan

> **For agentic workers:** Use subagent-driven-development for the bounded numerical
> component, parent integration, and independent specification/code review. Steps
> below track this batch, not a new runtime lifecycle.

**Goal:** Match shared NBV acquisition to finite scheduled sensor rays and validate
the latest complete physical task without fabricating evidence.

**Architecture:** Optional numerical predictor between the unchanged belief and
existing gain formulas. A thin adapter passes the installed scan asset and actual
capture duration. Historical idealized mode remains the default for old configs.

**Tech Stack:** Existing Python 3.10/NumPy numerical worker and Python 3.8 ROS adapter.

**Implementation checkpoint:** Steps 1–2 and the offline/model portions of step 3
are complete. Start 1 physically retrieved and held the object. Numerical,
file-boundary, policy and runtime-routing regressions passed, followed by an
independent specification/code-quality review (no blocking finding). The detailed
task list below retains the original test-first sequence. Starts 2–5 remain the
required online verification; no claim of finite-model E2E success is made yet.

## 1. Baseline and exact model

- [x] Preserve prior branch/history; run relevant legacy numerical tests.
- [x] Start natural full task with prior code plus the latest TF-wait fix (start 1).
- [ ] Finish start 1 and retain its actual result before production edits.
- [ ] Add `tests/test_finite_scan.py`: use four one-ray packets hitting two cells;
  with two-packet horizon assert phase fractions `[.5, .75]` for appropriate
  cyclic hit sets, and horizon >=4 saturates; verify against brute cyclic windows.
- [ ] Observe missing-model test failure, then implement
  `src/reachability_guided_nbv/finite_scan.py`: immutable schedule with unit rays,
  `packet_rows`, `packet_period_s`; `FiniteScan.from_csv(path, window_s, sensor)`;
  cached downward slopes; `predict(belief, viewpoint, config)` returning packet
  hits, unoccluded opportunity, opportunity and status. Use `searchsorted(...,
  side='right')-1` for the same half-open grid as A2. Known-prism rejection uses
  `segments_intersect_box` on actual ground endpoints before packet binning.
- [ ] Run `PYTHONPATH=src:tests:scripts <core-python> -m unittest test_finite_scan`;
  check all geometry cases described in the design, then commit numerical code.

## 2. Shared ranking and runtime integration

- [ ] Add failing tests for `rank_viewpoints(..., scan=...)` gain identity,
  unchanged inputs and Generic/Ours matching ranking on the same state.
- [ ] Extend `NBVResult` with float opportunity and unoccluded opportunity arrays;
  use `alpha * sum(opportunity * uncertainty)` and existing cost. Binary visibility
  is `opportunity > 0`; counts mean possible cells, not certain observations.
- [ ] Update `src/reachability_guided_nbv/outputs.py` to save actual gain inputs and
  metadata; keep old field names, clarify plot labels. Update no-occlusion routing
  in `src/a6_pilot/policy.py` to use unoccluded opportunity rather than restore FOV.
- [ ] Add optional path/window fields to `src/sim_active_perception/core.py`;
  create one predictor per ranking. Add adapter/parser routing in
  `scripts/run_a5_sim.py` and `scripts/run_a6_attempt.py`. Preserve old configs;
  enable explicit finite mode in `configs/current_sim_task.json`.
- [ ] Test parser->worker duration/path, old default, nonexistent asset, both
  method decisions, and serialization. Run legacy tests, inspect diff and commit.

## 3. Replay and bounded online batch

- [ ] Additional concrete consistency test: an AMBIGUOUS disk on the left edge
  of a raw OCCUPIED cell must not block a vertical ray through the separated right
  side. The same disk, target box, ENVIRONMENT or missing history must still block
  true intersections. Add `tests/test_operational_occlusion.py` first; implement
  `src/reachability_guided_nbv/operational_occlusion.py` with actual closed cylinders
  (not their AABBs), target OBB and fallback cell prisms. Pass one shared geometry
  object through ranking to finite `predict`; never mutate raw context or votes.
- [ ] Test that finite scan does not first cull raw OCCUPIED cells when this
  explicit operational geometry is supplied, and that custom sensor mismatch
  is rejected. Extend real file-boundary/policy tests for both methods.
- [ ] Change only current profile offsets to `[-2,-1,0,1,2]` after the documented
  offline common-candidate coverage finding; retain old/default profiles.

- [ ] Independently review specification then code quality; resolve real issues.
- [ ] Replay all eight existing Hard windows: compare old center model and actual
  production finite model against recorded ground presence. Distinguish nominal
  pose from actual TF, phase horizon and known occlusion. Keep original outcomes.
- [ ] Decide common lattice retention/refinement from offline coverage, not wins.
- [ ] Freeze code/config before starts 2/3 (Hard01 Generic/Ours) and starts 4/5
  (Moderate01 Ours/Generic). No production changes during a run or mixed-version
  effect claims. Reserve sixth start only for a documented runtime diagnosis.
- [ ] Record `observation -> confirmed -> navigation -> refine -> D_exec ->
  grasp -> lift/hold`, windows/moves, sim time, missing-path data and first failure.
- [ ] Compare predicted acquisition with actual support, preserve all failures,
  report remaining capability limits and fixed-version evaluation readiness.
- [ ] Relevant regression, final diff/status, commit/push development checkpoint;
  keep Draft, no main merge or formal matrix.
