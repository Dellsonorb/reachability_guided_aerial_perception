# Ground execution reliability — bounded development batch

Start: AGENT 37e2870, SIM source ece36ec (latch namespace fix, not yet installed).
Work in AGENT's current feature/dev-operational-consistency checkout; preserve
old records. SIM execution changes get feature/fix-ground-execution-reliability
based on ece36ec. No new worktree, method gate revision, formal matrix or final
test-set use. Generic/Ours use the same execution code.

## Bound and design decisions before online work

**At most eight Gazebo startup invocations, including startup failures.** Each
has a separate output directory under outputs/development/ground-execution-batch.
No unlogged restarts or repeated trial-until-success. One scene and selected
Ground candidate per diagnostic; any observation-pose search is explicitly
bounded and logged. Stop the batch at the cap even if failures remain.

1. Install the already reviewed XY-latch namespace fix; replay the recorded
   Easy/Ours navigation goal from previous launch07, no UAV flight. Keep the
   original 120 simulation-second navigation timeout and real stop conditions.
2. If it fails, diagnose actual public pose/velocity/action/plan traces before
   another navigation revision. A namespace correction is not itself success.
3. Ground segment checks reuse original Easy/Moderate candidate and aerial
   perception records: first identify exact IK/visibility failures, then test
   physically justified camera-aware observation/refine corrections. Prefer
   previous launch06/08/09. No candidate reselection or target GT input.
4. Reserve up to two starts for complete E2E only after relevant local fixes
   pass. Remaining starts address diagnosed local problems or invalid setup;
   local problems take precedence over spending a full-chain regression.

Ground-only runs are **conditioned on recorded confirmation**, not new A1/A2
discovery trials or new aerial observations. Fresh D435 perception is mandatory
before manipulation. No synthetic AIR_HANDOFF/TAKEOFF statuses. Ground-only
physical checks reuse existing navigation, fresh-observation, arm-controller,
grasp-retention and minimum physical lift criteria, omitting only nonexistent
flight stages. Simulation model pose is permitted solely in the separate
physical checker, never in robot action or target estimation.

## Alternatives and selected direction

Retrying the old grasp-seeded observation path cannot fix camera FOV or partial
surface assumptions. A single fixed arm posture is simple but not consistent
across base yaw and target location. Prefer bounded camera-aware poses derived
from runtime target pose, CameraInfo and known optical↔end-effector transform,
with collision-aware arm planning and fresh visual acceptance. Do not infer
camera visibility from grasp reachability. Choose exact view geometry after
offline projection of previous failed frames; do not change the operational gate.

For partial surfaces, compare explicit object-surface fitting with rejecting
unsupported top-surface estimates and acquiring an informative view. Never
replace measured height with the nominal target center merely to pass its gate.
Keep real collision/grasp/lift acceptance; classify unavailable view/IK as a
capability boundary when that is what the evidence actually establishes.

Hard is an independent **offline diagnostic**: compare each chosen view's
predicted mask to actual endpoint hits and real support changes. Distinguish
policy same-pose requests from corrective flights due to measured hover drift.
No added observation votes/windows, no UNKNOWN→FREE or gate edits.

## Implementation plan

Use test-first development and focused independent review. Ordinary design and
runtime decisions are autonomous under the user's authorization.

- [ ] `scripts/run_ground_sim.py`: thin subclass of existing A6/A5 adapter;
  recover one confirmed `A5_SELECTED` and finite original `AIR_HANDOFF.target_map`
  from a saved run, set the same cached exact candidate, call only the real
  Ground stages. Tests in `tests/test_ground_segment.py` cover missing/duplicate/
  nonfinite/unconfirmed inputs, no UAV calls, and proper terminal reporting.
- [ ] `scripts/run_a6_attempt.py`: opt-in Ground replay dispatch and independent
  diagnostic kind; assert recorded scene equals configured scene. Reuse startup,
  process cleanup and existing native-rate logs. Default method runs unchanged.
  Navigation-only is not labeled retrieval success or failure.
- [ ] SIM `scripts/check_air_ground_pick_demo.py`: opt-in Ground-only expected
  status sequence; reuse all applicable existing physical checks. Add tests
  demonstrating no invented flight events and unchanged physical failure gates.
- [ ] Install bunker_navigation from SIM's source-only fix using the existing
  catkin p450-clean profile; confirm installed YAML and run focused tests. Run
  launch01 once and diagnose the resulting navigation trajectory.
- [ ] For each diagnosed execution correction: capture a failing offline/unit
  case, make the smallest coherent change, test and perform a bounded Ground
  segment check. Record MoveIt error codes at existing service calls and camera
  projection/observation reasons, not a new logging framework.
- [ ] After local checks pass, run at most two original full E2E regressions
  within the same eight-start cap. Use existing v1.4 method/configuration.
- [ ] Review results, run focused/full regressions, report conditioned
  `confirmed → D_exec → retrieval` separately from E2E, commit/push checkpoints
  and keep PRs Draft unless independently ready. Stop; no formal runs.

Launches used at this plan commit: **0/8**.
