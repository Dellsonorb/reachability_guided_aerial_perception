# v1.2 Natural Integration Recovery Plan

> **For agentic workers:** Use subagent-driven-development for a bounded
> independent platform trace and two-stage repair review. Root owns the single
> serial SIM and the integration sequence. User-authorized routine decisions
> do not wait for another approval. Keep the designated A6 feature checkout.

**Goal:** Pass the original natural A5 E2E with frozen v1.2 semantics, then stop.

**Architecture:** Observe the unchanged public runtime sequence first; repair
only a proven orchestration/platform defect at its source. Keep numerical
research modules untouched. Native rosbag plus existing checker are sufficient.

**Tech Stack:** Existing Noetic/Python3.8, core Python3.10, SIM/PX4, NumPy; no new
dependency or mission/benchmark/evidence framework.

## 1. Reproduce and localize (before any repair)

- [ ] Verify baseline tests and existing feature checkout; preserve unrelated
  untracked A5 directories and all original results.
- [ ] Bounded independent trace: follow public flight facade HOVER/FLY_TO through
  Prometheus native position/velocity semantics and localization transforms.
  Read only; do not change parameters or run another SIM.
- [ ] Root launches original natural configuration into a new
  `outputs/a6/v12-integration/natural-attempt-01`; starts native rosbag for
  `/clock`, `/tf`, `/tf_static`, `/rosout_agg`, `/uav1/prometheus/state`,
  `/uav1/prometheus/odom`, `/uav1/prometheus/control_state`,
  `/uav1/prometheus/command`, `/uav1/mavros/local_position/pose`,
  `/uav1/mavros/local_position/velocity_local`, `/uav1/mavros/setpoint_raw/local`,
  and runtime flight goal/result/feedback. Do not record demo status as a
  subscriber that could satisfy the startup checker wait.
- [ ] Run existing A5 adapter with `--operational-gating v1.1
  --support-anchor exact_winner --wait-for-status-subscriber`; connect the
  existing physical checker after the readiness log. Frozen natural settings
  and exact command templates are in `docs/OBJECT_AWARE_GATING_V11.md`.
- [ ] Read the saved trace with native rosbag and compare measured map pose,
  native pose/setpoint, state velocity, message age and action times around
  `_a5_wait_settled`. Report the actual failing predicate before choosing a fix.

## 2. TDD only the demonstrated integration repair

- [ ] Once the root cause is known, append the concrete patch/test steps here
  before implementation. This diagnostic plan intentionally does not guess a
  control change. Use existing `tests/test_a5_ros_support.py` boundaries where
  possible and a focused new test only if needed. The RED test must reproduce
  the observed ordering/data failure with unchanged acceptance constants.
- [ ] Run the RED test, apply the smallest source patch, rerun GREEN and nearby
  adapter regressions. No condition relaxation or alternative candidate/view.
- [ ] Independent spec review, then quality review; resolve actual findings.

## 3. Natural acceptance and checkpoint

- [ ] Run the original natural E2E after any proven repair, keep all results,
  and verify physical lift through the existing checker. No favorable-outcome
  selection, unclassified retry or formal slot17.
- [ ] Reconstruct all natural observation windows from retained NPZs, check
  exact support/A4 input consistency and retain per-cell non-winner diagnostics
  without using them for selection.
- [ ] Run `PYTHONPATH=src:tests CORE -m unittest discover -s tests -q`, native
  Noetic split and actual RM4D/task-map tests; replay Hard-002 unchanged.
- [ ] Write integration report and a small fresh-seed validation proposal.
  Proposal only: no seed screening by scores/outcomes and no new experiment.
- [ ] Final independent review; confirm source boundaries/working tree, close
  only owned SIM processes, commit/push feature checkpoint, update Draft PR and
  stop. Do not merge or resume the interrupted560-slot protocol.

## Concrete repair after natural-attempt-01 diagnosis

The unchanged run passed hover, all three windows, exact confirmation and
Ground navigation. It failed preparing the D435 observation pregrasp, before
`GROUND_REFINED`: Python requested TF at222.406 while its cache ended222.405.
The public bag contains bracketing TF222.405/222.425 and later transforms.
Native MoveIt construction blocks Python callbacks: a no-motion live probe
measured a1.219s Python heartbeat gap and unchanged Python ROS time66.971
while C++ advanced to68.102. Installed tf2 uses a ROS-time timeout, so queued
clock delivery can expire the0.5s wait before queued TF delivery. A deterministic
native tf2 replay reproduces the original1ms error with this callback order.

- [ ] RED: reproduce clock catch-up before TF delivery; require unchanged
  exact stamp to succeed once its bracketing transform arrives. Also test
  permanent missing TF, shutdown, non-TF exceptions and explicit latest usage.
- [ ] Add only an A5 adapter pose-transform override using nonblocking exact
  lookup and a0.5s monotonic wall readiness bound, yielding between attempts.
  Preserve inherited same-frame copy, requested frames/stamp and explicit
  `use_latest` behavior. No stamp offset, new geometry tolerance, motion retry,
  candidate replacement, method change or SIM source patch.
- [ ] Test adapter wiring and native tf2 clock-order reproduction, then review
  and run a fresh original natural regression. Retain the failed activation.
