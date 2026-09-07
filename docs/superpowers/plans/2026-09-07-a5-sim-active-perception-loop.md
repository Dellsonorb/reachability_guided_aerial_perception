# A5 SIM Active-Perception Loop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans. The user authorized continuous execution without ordinary approval pauses.

**Goal:** Verify frozen A1-A4 controlling one natural active-observation-to-lift SIM run.

**Architecture:** A Python 3.10 core reconstructs bounded observation history and returns A4/ground decisions. A system-Python ROS adapter reuses the existing SIM demo's robot-facing execution. Request/response JSON and observation NPZ files are the only cross-environment boundary.

**Tech Stack:** Existing NumPy/Matplotlib, unittest, ROS Noetic/Gazebo/MoveIt and frozen RM4D; no new dependencies.

## Execution status — 2026-09-07

Tasks 1 and 2 are implemented and committed, with red/green unit tests, real
subprocess/ROS import checks, independent spec review and code-quality review.
Task 3 launched one actual natural Gazebo attempt (`gazebo-WwmQis`). It failed at
the first A3/A4 decision: public TF did not place the ground in A2's frozen height
band, all 21 representatives were occupied-blocked, and no next task viewpoint
or exact ground candidate was selected. UAV cleanup landed and the runtime was
stopped. See [the integration report](../../A5_SIM_ACTIVE_PERCEPTION.md).

Subsequent authorized SIM platform repair is isolated on
`feature/fix-sim-uav-map-localization @ 02cd039`. Landed/hover real ground checks
and P450/Ground/D435 public frame regressions passed. A fresh clear-parking run
executed two actual A4-selected flights, but three single-message observations
left the best exact footprint with only 17/108 FREE cells. A5 correctly aborted.

E2E success, A5 PR merge and A5 completion remain **open**. The next bounded
integration step is a 5-second stable-hover acquisition window, with each chunk
transformed at its own timestamp and the whole window remaining ONE A2 vote per
cell. Frozen A1-A4, thresholds, occupied priority and exact all-FREE acceptance
remain unchanged. No Ground/grasp/lift acceptance has yet been met.

## Task 1 — Core integration and exact candidate catalog (root)

Create `src/sim_active_perception/{__init__,core,worker}.py`, `scripts/a5_core_worker.py`, `tests/test_a5_core.py`.

- [ ] Write failing tests for exact per-cell winner recovery, first equal-margin tie, exact-footprint occupied rejection and all-FREE acceptance with positive unknown_score, no clipping, stable timestamp history, max-round/score stops, and executable A4 candidate filtering.
- [ ] Run `PYTHONPATH=src /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python -m unittest tests.test_a5_core -v` and observe missing implementation.
- [ ] Implement direct composition of frozen builders. Central identity assertion: `selected['x'] == original['bunker_x']`, not `A1.cell_center_x`. Cache the complete RM4D result and exact grasp once; observe requests replay each history frame exactly once into a new A2 mapper.
- [ ] Test subprocess request/response and failure behavior using a synthetic cached initial result, without live RM4D or ROS.
- [ ] Run targeted tests and save local commit after verification.

## Task 2 — Thin ROS execution adapter (delegated independently)

Create only `scripts/run_a5_sim.py`, `scripts/a5_ros_support.py`, `tests/test_a5_ros_support.py`. Do not edit frozen modules, core-worker files or SIM source.

- [ ] Write failing source-independent helper tests for yaw quaternion/pose arrival, finite XYZ extraction, increasing fresh scan timestamps and worker response handling.
- [ ] Reuse the SIM `AirGroundPickDemo` class locally, overriding `_run_air_phase` and `_select_rm4d_candidate`. Load public demo parameters; leave manipulation/navigation stages unchanged. Start without the original demo node (`run_demo=false`).
- [ ] In the aerial phase takeoff/view/observe as before, send init to the core, collect stamped stable-hover PointCloud2, send observe, execute returned viewpoint, and repeat until stop. Preserve exact candidate response for the inherited ground phase. Return to the initial clear view pose and land after successful selection. No goal/measurement oracle substitution.
- [ ] Core invocation is a bounded subprocess with the exact interface in the A5 spec. Save every received cloud once as NPZ; response-file errors/timeouts abort the run through existing cleanup. Do not add a message bridge/service framework.
- [ ] Add `--help` and import checks, tests, self-review, then spec compliance review followed by code-quality review. Root handles SIM launch and E2E, so do not start Gazebo independently.

## Task 3 — Run configuration, debugging, documentation and Git (root)

Create `scripts/run_a5_gazebo.bash` and `docs/A5_SIM_ACTIVE_PERCEPTION.md`; preserve a small final `outputs/a5/` summary/visualization only after actual results exist.

- [ ] Resolve existing PX4 runtime, exact frozen RM4D checkout, map and Python paths without changing their methods; start a dedicated ROS/Gazebo port pair and process group using the existing SIM environment wrapper.
- [ ] Launch public air-ground demo runtime with `run_demo=false`, MID360 true, same natural brick and existing sensors/MoveIt; run only the A5 ROS adapter.
- [ ] Inspect failures with systematic-debugging. Correct only A5 integration/local run settings; if results negate a research assumption or require frozen changes, stop and report the precise boundary.
- [ ] Require real A4-selected displacement, real endpoint-induced belief/candidate change, navigation and D435 refinement, retained AG95 grasp and lift. Save concise scalar results and field plots; label unsuccessful attempts honestly.
- [ ] Perform final independent review, one complete frozen+A5 test run, syntax checks and frozen-path diff check. Commit the completed A5 work and perform the appropriate authorized PR workflow only if genuinely complete. Do not enter paper experiments or a new stage.

## Bounded follow-up — hover-window acquisition

- [ ] Delegated adapter implementation: test `merge_cloud_chunks` with distinct translated/pitched sensor poses, stamps and normalized frames; then collect a finite positive `--cloud-window-s` below the existing capture timeout. Re-express all points into the last sensor frame/time, save one observation and minimal chunk geometry for debugging.
- [ ] Root frozen-A2 integration test: multiple chunks covering the same ground cells yield observation_count=1, free_evidence=1, unknown_score=exp(-1/2), not the number of packets. A second whole window is required for FREE.
- [ ] Review, fresh natural Gazebo run, exact candidate/navigation/refinement/grasp/lift verification. Do not loosen semantics if the remaining outcome is negative.
