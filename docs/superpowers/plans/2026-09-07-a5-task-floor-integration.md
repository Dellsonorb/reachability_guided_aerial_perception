# A5 Calibrated Task-Domain Asset Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development. Steps use checkbox syntax for tracking.

**Goal:** Generate a separate negative-z runtime RM4D asset, validate its calibrated floor, then continue the natural A5 loop.

**Architecture:** Reuse frozen RM4D classes unchanged in an independently owned PyBullet world. Copy the original positive map slices; populate only five new negative slices using the same collision-free uniform-joint sampling method. A5 chooses this runtime context explicitly; A1–A4 remain unchanged.

**Tech Stack:** Existing Python 3.10, NumPy, PyBullet, SciPy; existing ROS Noetic adapter and SIM public APIs.

Progress: Tasks 1–4 implemented, reviewed and validated. Natural run
`natural-approach-j87FQA` reached physical grasp/lift and existing checker PASS
after clean teardown. AGENT Git closeout follows; SIM commit `5e25039` is local,
with its remote push separately blocked by automatic permission review.
Actual results are in `docs/A5_TASK_DOMAIN_ASSET.md`. The independent acquisition
timeout diagnosis and bounded fix are in
`docs/superpowers/specs/2026-09-07-a5-low-speed-acquisition-design.md`.

## Task 1: Independent asset, sampler and validator

Files: create `src/sim_active_perception/task_map.py`, `scripts/build_a5_task_map.py`, `tests/test_a5_task_map.py`.

- [x] Write failing tests for floor derivation, unchanged overlap, independent plane and retained collision gates. Use real frozen classes when the local frozen checkout is available; skip that integration subset elsewhere.
- [x] Run `PYTHONPATH=src /media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python -B -m unittest tests.test_a5_task_map -v` and confirm the missing functionality fails.
- [x] Implement `open_task_rm4d_api(rm4d_root, config_path, asset_dir, frame_calibration)` as a context manager. Derive `task_floor_z = map_ground_z - ground_reference_height_m - T_bunker_aubo[2,3] + robot_base_z`; require agreement with recorded calibration. Keep robot base at the frozen 0.01 m. Move only this simulator's plane. Construct unchanged validator/planner classes with the independent world and truthful task-asset metadata.
- [x] Builder takes frozen root/config/map, a saved calibration JSON and output directory; workspace lower bound -0.25, same resolution/XY/theta/upper bound. Store explicit floor inputs and result separately from workspace bounds. Use seed 42 and initially 100,000 collision-free samples; storage-filter FK outside new negative bins without changing proposal/rejection method. Copy positive occupancy exactly. Save one map and sampling summary, no checkpoint framework.
- [x] Run tests; review spec then quality. Commit only scoped files.

## Task 2: A5 opt-in integration boundary

Files: modify `src/sim_active_perception/worker.py`, `scripts/run_a5_sim.py`, `tests/test_a5_frame_bridge.py`, `tests/test_a5_ros_support.py`.

- [x] Add failing tests: optional `rm4d_task_asset` chooses the task API, current TF calibration is passed, exact map grasp remains unchanged, baseline route still works.
- [x] Implement selection:

```python
asset = request.get('rm4d_task_asset')
context = (open_task_rm4d_api(request['rm4d_root'], request['rm4d_config'], asset,
                             request['frame_calibration']) if asset else
           open_frozen_rm4d_api(request['rm4d_root'], request['rm4d_config'], request['rm4d_map']))
```

- [x] Add optional `--rm4d-task-asset` directory argument, forward it through ROS/file boundary. Label task-domain raw results separately from frozen baseline results; keep existing baseline output compatibility.
- [x] Run focused A5 tests and review the boundary changes.

## Task 3: Generate and validate the runtime asset

Files: new `assets/rm4d_ground_task_v1/`, selected diagnostics under `outputs/a5/`, update A5 documentation.

- [x] Generate negative slices with the Task 1 CLI. Record actual sample count; this is not a new 10M baseline or complete capability claim.
- [x] Verify original positive slices bit-for-bit, representative forward/inverse queries, ground 0 -> floor -0.472, nominal robot/floor and self-collision gates.
- [x] Run the actual saved grasp and small positive/negative targets through RM4D -> IK -> FK -> collision. Report failures as well as successes.
- [x] Start existing Gazebo wrapper with BUNKER initial parking (3,-2.5), public TF localization fix. Use existing `check_a5_frame_bridge.py` on floor-near valid candidates for actual SIM collision-aware IK/planning; do not execute diagnostic arm trajectories.

## Task 4: Return to natural A5 E2E

Files: existing A5 integration only if runtime diagnosis proves a change necessary; corresponding focused tests and documentation.

- [x] Restart from real aerial target and MID360 packets with the task asset, preserving exact map TCP and frame bridge.
- [x] Diagnose any sampling-window timeout separately using stamped TF and existing dwell semantics. Write a failing regression before any runtime fix; do not modify A2 thresholds, ground z, votes or frozen methods.
- [x] Continue belief -> observation/NBV -> exact Ground candidate -> BUNKER -> D435 -> grasp -> lift. Use the existing physical checker to verify the final manipulation outcome.
- [ ] Run one complete unit suite, focused code reviews and `git diff --check`; confirm frozen source/repository diffs remain empty. Document actual outcomes and commit/push A5 WIP or completed integration as justified; do not merge incomplete A5.

Only pause for unresolved task-floor/SIM physical disagreement or a required robot/algorithm change, as explicitly instructed by the user.

### Local runtime guard adjustment after natural retry

`natural-refined-0coa8b` completed three observations and exact candidate
confirmation, but its approximately 2 m Ground approach exhausted the inherited
60 s navigation timeout. Public TF after stopping was 0.06057 m from the goal,
with 0.33094 rad remaining yaw error. No collision/goal acceptance is relaxed.
Expose the already existing SIM `navigation_timeout` as an optional A5 CLI
override, retaining the default unchanged; use 120 s for the next natural run.
This is a bounded execution timeout, not a research parameter or a claim that
the failed run was feasible. Test default preservation, positive finite override
and unchanged goal tolerance before wiring it. Keep any subsequent failure.
