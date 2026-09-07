# A5 Reference Frame Bridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task.

**Goal:** Preserve AUBO-relative grasp geometry between frozen RM4D and public SIM map, then resume A5.

**Architecture:** A5-only pure geometry bridge plus existing ROS/file boundary. Read public Ground TF; do not modify SIM physical definitions or frozen algorithms.

**Tech Stack:** Existing NumPy, unittest, ROS Noetic tf2/MoveIt, frozen Python 3.10 worker.

### Task 1: Pure bridge and worker

- [ ] Add `tests/test_a5_frame_bridge.py`; first assert translated query z and invariant local grasp geometry (e.g. map TCP z=.08, reference TCP z=-.28 with h=.36).
- [ ] Run `PYTHONPATH=src python -m unittest tests.test_a5_frame_bridge -v` and observe missing implementation failure.
- [ ] Add `src/sim_active_perception/frame_bridge.py`: validated horizontal TF calibration, map-to-reference query, reference-to-map result conversion. Keep exact input and original result immutable by copying.
- [ ] Wire `src/sim_active_perception/worker.py` initialization to `frame_calibration` request; preserve baseline result, converted result, and original grasp in `initial.json`.
- [ ] Test original flags/margins/order unchanged, local TCP/flange invariance at rotated/translated poses, non-default height, invalid transform/mount rejection. Run focused A5 tests and obtain spec/quality review.

### Task 2: Public TF boundary and real planning validation

- [ ] Add a test showing init includes `T_map_ground_odom`, `T_ground_odom_bunker`, and `T_bunker_aubo` from public TF.
- [ ] Wire `scripts/run_a5_sim.py` to acquire those transforms using existing tf buffer and finite/age checks. No numerical 0.36 default in AGENT code.
- [ ] Add small integration diagnostic `scripts/check_a5_frame_bridge.py` to compare saved baseline/adapted candidate local TCP with SIM mount and call existing MoveIt IK/planning (no execution).
- [ ] Launch existing natural SIM composition; run a calibrated representative frozen query and the actual planning diagnostic. Record numerical transform errors and service results, not simulated success.

### Task 3: Resume A5

- [ ] Only after bridge checks, rerun fresh initial observation through A1–A4 and existing Ground/manipulation pipeline.
- [ ] If capture timeout recurs, inspect packet TF/physical motion separately; use a focused failing test before any runtime fix. Never alter A2 evidence rules.
- [ ] Run focused/full existing tests once at handoff, inspect frozen-directory diff, update A5 docs with actual outcome, commit/push WIP only while E2E incomplete. No merge to main.

## Execution checkpoint

- [x] Tasks 1–2 implemented with red/green tests and independent spec/quality review.
- [x] Public nominal height .36 and relative mount .122 confirmed from sources and live TF.
- [x] In-domain representative: 192 valid candidates, local matrix error <=5.56e-16; actual SIM collision-aware IK + 28-point RRTConnect plan passed for candidate-000014 (no execution).
- [x] Fresh natural A5 retry: exact TCP z=.082807036, calibrated query yields zero inverse-reachable before validation; A1 NO_INVERSE_REACHABLE preserved.
- [x] Existing full suite: 219 tests passed; frozen source diff empty.
- [ ] A5 E2E: blocked by frozen map coverage (internal flange z<0, stored domain [0,1.3]). Task/scene or map coverage decision needed; do not modify frozen baseline.
- [ ] Separate sampling-window timeout remains; not changed while map coverage prevents a valid Ground decision.
