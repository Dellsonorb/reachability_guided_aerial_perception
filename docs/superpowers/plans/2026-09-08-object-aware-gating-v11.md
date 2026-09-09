# A3/A5 v1.1 Object-aware Operational Gating Implementation Plan

> **For agentic workers:** Use executing-plans for the coupled core/integration
> work; independent sensor inspection and spec/quality review use bounded agents.
> Work in the user-designated feature/a6-formal-experiments checkout.

**Goal:** Implement the approved object-aware gate, preserve v1/Pilot-1, validate
synthetic cases, recorded Hard states and the original natural A5 scene, then stop.

**Architecture:** A2 remains byte-for-byte unchanged. A derived view independently
counts genuine ground and TARGET/ENVIRONMENT/AMBIGUOUS occupied votes from the
same raw observations. A3 representatives and A5 exact candidates call a shared
continuous-footprint assessor. v1 is the default; v1.1 is an explicit invocation.

**Tech Stack:** Existing Python 3.10/numpy core, Python 3.8 ROS boundary, existing
SIM RGB-D geometry helpers and public TF/topics; no new dependency or framework.

## Approved design / implementation choices

The user approved the proposal in `docs/A6_PILOT1_DIAGNOSIS_REVIEW.md`. Retain
ENVIRONMENT/AMBIGUOUS blocking, true target collision and per-window real ground
votes. Missing association never grants a target exception. Any metric allowance
in association must derive from sensor geometry/resolution, must also bound the
object collision geometry, and must be recorded before natural-scene execution.
It is not selected by candidate confirmation or Ours/Generic score.

Keep the padded .52/.39 m footprint, .10 m grid, source representative guard,
catalog/tie ordering, A2 thresholds, A4 raw-A2 visibility/cost, all execution
parameters and every Pilot-1 file/result. No XY-latch fix is part of this plan.

## Task 1 — Derived evidence and continuous collision (root)

Files: create `src/operational_gating/{__init__,core}.py` and
`tests/test_operational_gating.py`.

- [x] RED: use an existing A5 ground-grid fixture and a target that lies outside
  the continuous footprint but inside an overlapping .10 m cell. Assert:

```python
view = derive_operational_evidence(grid, observations, target, labels=labels)
pose = assess_footprint(view, (0., 0.), 0.)
assert not pose.blocked
assert pose.ground_supported
assert raw_belief.state.flat[target_cell] == 100
```

- [x] Add independent tests: target contact/intersection, ENVIRONMENT and
  AMBIGUOUS in the same cell, no ground invented from target-only returns, one
  vote per window, two true ground windows despite target co-occurrence, clipping,
  missing association, malformed labels/frames, no mutation of raw inputs.
- [x] GREEN: `PerceivedTarget` holds map center/yaw, known dimensions and explicit
  geometry allowance; `OccupiedClass` has TARGET/ENVIRONMENT/AMBIGUOUS;
  `derive_operational_evidence` applies original return/height filtering and
  counts exclusive environmental/ambiguous occupied priority over ground.
  `assess_footprint` calls existing raster overlap plus rectangle SAT for object
  contact, returns separate blocking causes and ground support.
- [x] Run `PYTHONPATH=src:tests $CORE_PY -m unittest test_operational_gating -v`;
  observe RED then GREEN. Review model and counters before integration.

## Task 2 — Shared optional A3/A5/A6 semantics (root)

Files: modify `src/task_relevant_uncertainty/core.py`,
`src/sim_active_perception/core.py`, `src/a6_pilot/policy.py`; create
`tests/test_operational_integration.py`.

- [x] RED: exercise `build_task_uncertainty(..., operational=view)` and
  `assess_candidates(..., operational=view)` on representative/exact mismatch.
  Check true target intersection blocks both; aliasing alone does not; raw
  occupied counts remain diagnostic; R/M_nominal/A2 are unchanged.
- [x] GREEN: optional keyword defaults to the untouched v1 branch; v1.1 uses
  `assess_footprint` for both pose types. Keep unknown separate from blocking and
  explicit object-aware blocked state. Add version/cause diagnostics only to
  v1.1 decisions, not Pilot-1 artifacts.
- [x] RED/GREEN fairness: pass the same view into Generic/Ours and both optional
  ablation paths; shared candidate assessments, V, cost and A2 must agree.
- [x] Run new tests plus existing `test_task_geometry`, `test_a5_core` and policy
  tests. Obtain spec then code-quality review of these changes.

## Task 3 — Runtime perception association and worker boundary

Files: create `scripts/a5_target_support.py`,
`src/operational_gating/association.py`, `tests/test_target_association.py`;
modify A5 parser/adapter and A5/A6 worker observation handling only as needed.

- [x] Resolve physical association allowance from actual camera intrinsics,
  pixel support and SIM LiDAR model before coding its classifier. Document the
  selected rule in `docs/OBJECT_AWARE_GATING_V11.md` before runtime verification.
- [x] RED/GREEN pure projection tests: matching red segmented surface + object
  geometry yields TARGET; wrong depth, outside geometry, missing evidence and
  unsupported/occluded views yield AMBIGUOUS near target; far returns remain
  ENVIRONMENT. No GT access and no hard-coded target pose.
- [x] Cache a synchronized initial aerial RGB-D sample tied to AIR_HANDOFF and
  its image-time map TF. Reuse existing SIM segmentation/registration helpers.
  Persist only required numerical support, image mask/depth/intrinsics and
  stamp/frame for reproducible point association; do not change SIM observer.
- [x] Add explicit `--operational-gating v1.1` option. Worker input includes
  runtime target geometry and support file; v1 has no added subscribers or
  association behavior. Save derived arrays/causes separately from raw A2.
- [x] Check Python 3.8 imports and parser/worker tests; v1 calls stay compatible.

## Task 4 — Recorded Hard replay (separate output, no trial mutation)

Files: create `scripts/replay_operational_gating.py` and focused replay tests;
write derived output under `outputs/a6/operational-v11/`, never Pilot-1 paths.

- [x] RED/GREEN replay actual Hard observations, AIR_HANDOFF and exact catalogs.
  Compare legacy blocked/confirmed to v1.1 by candidate and cause. Missing RGB-D
  association in Pilot-1 remains AMBIGUOUS; do not invent labels to unlock it.
- [x] Include known .117820 m continuous-separation case in a deterministic
  geometry regression; test confirmed TARGET alias handling with synthetic
  evidence separately and clearly label it synthetic, never a corrected trial.
- [x] Verify all five raw A2 replay arrays agree with saved values and that
  Pilot-1 result files are unchanged. Document counts and association limits.

## Task 5 — Original A5 natural Gazebo E2E and final review

- [x] Recover the original successful natural A5 launch/adapter settings from
  its existing artifacts. Run that scene with v1.1 explicitly enabled, from a
  fresh aerial/MID360 first observation. Keep all navigation/refine/arm settings.
- [x] Record real observed target association, decisions, Ground, refine,
  descend/grasp/lift and existing physical checker outcome. Retain any failed
  diagnostic activation; do not substitute a synthetic success or tune inputs.
- [x] Run proportional core/ROS tests, full regression, independent spec and
  quality review. Check frozen A1/A2/A4/assets and all Pilot-1 files unchanged.
- [x] Update v1.1 documentation with actual before/after counts, limitations,
  exact commands and E2E result.

Delivery action after validation: commit/push this scoped checkpoint on the
current branch; no main merge, no Pilot-2, no formal runs or sample-size
selection. Branch history records delivery; stop at review.

## Verification checkpoint

427 core-discovery tests: 421 passed, 6 OpenCV tests exercised under system
Python (11/11 boundary tests passed). Recorded Hard: 5 slots / 15 rounds;
raw A2 and v1 assessments exact, 172 blocked / 0 confirmed under both versions
without archived positive association. Original natural A5: one activation PASS,
physical lift .1485801 m, normal runtime shutdown. Independent core and runtime
boundary spec/quality reviews passed. Local integration issues were corrected
through RED/GREEN tests; no execution parameter, Pilot-1 result or SIM source edit.
