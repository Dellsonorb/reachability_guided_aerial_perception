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

- [x] `scripts/run_ground_sim.py`: thin reuse of existing A6/A5 adapter;
  recover one confirmed `A5_SELECTED` and finite original `AIR_HANDOFF.target_map`
  from a saved run, set the same cached exact candidate, call only the real
  Ground stages. Tests in `tests/test_ground_segment.py` cover missing/duplicate/
  nonfinite/unconfirmed inputs, no UAV calls, and proper terminal reporting.
- [x] `scripts/run_a6_attempt.py`: opt-in Ground replay dispatch and independent
  diagnostic kind; assert recorded scene equals configured scene. Reuse startup,
  process cleanup and existing native-rate logs. Default method runs unchanged.
  Navigation-only is not labeled retrieval success or failure.
- [x] SIM `scripts/check_air_ground_pick_demo.py`: opt-in Ground-only expected
  status sequence; reuse all applicable existing physical checks. Add tests
  demonstrating no invented flight events and unchanged physical failure gates.
- [x] Install bunker_navigation from SIM's source-only fix using the existing
  catkin p450-clean profile; confirm installed YAML and run focused tests. Run
  launch01 once and diagnose the resulting navigation trajectory.
- [x] For each diagnosed execution correction: capture a failing offline/unit
  case, make the smallest coherent change, test and perform a bounded Ground
  segment check. Record MoveIt error codes at existing service calls and camera
  projection/observation reasons, not a new logging framework.
- [ ] After local checks pass, run at most two original full E2E regressions
  within the same eight-start cap. **Not activated:** local lift still fails;
  the remaining starts were spent on cause-discriminating Ground diagnostics.
- [x] Review results, run focused/full regressions, report conditioned
  `confirmed → D_exec → retrieval` separately from E2E, commit/push checkpoints
  and keep PRs Draft unless independently ready. Stop; no formal runs.

Launches used at this plan commit: **0/8**.

## In-batch diagnostic decisions (not a new trial protocol)

- Launch01: namespace fix installed; still the original 120 simulation-second
  navigation timeout. Public odometry was sampled after imposing commanded
  velocity and therefore echoed an unachieved request.
- Launch02: sample completed-physics velocity before the next request; feedback
  agrees with pose changes, but physical low-speed response remains poor.
- Launch03/04: isolate Easy/Moderate camera/refine by spawning BUNKER at the
  archived exact candidate. Target/obstacles unchanged. These are explicitly
  **conditioned on arrival**, not navigation successes or full E2E trials.
  A6 methods are not run or compared in these Ground-only diagnostics.
- Camera-centered observation v1: current accepted view first, otherwise six
  views at measured-target-top depths .45/.55/.35 m × two optical rolls. Use
  full measured TCP↔camera extrinsic, check calibrated full-cuboid FOV, plan
  collision-aware observation without a grasp-IK prerequisite. Fresh measured
  surface cue can aim XY; only a fresh accepted top estimate refines the grasp.
  Ground-only top validity requires normal within15°, 90–110%known top spans,
  no image-border clipping. Old aerial defaults, .03 m center-height acceptance,
  real collision, controller stop and physical lift conditions remain unchanged.
- Launch03 reached D_exec and gripper confirmation, then failed lift controller
  goal settling. Launch04 refined successfully, then grasp IK returned -31 six
  times. Preserve both failures; no extra retry in either invocation.
- Offline real SDFormat9 conversion isolated a renderer bug: unprefixed
  `base_link_collision` on `ground/base_link` loses its declared zero-friction
  surface. Naming it `ground/base_link_collision` preserves that surface;
  geometry, masses, inertia, all17joints and other surfaces remain identical.
  Launch05 tests this correction with original Easy navigation plus Ground.

Invocations activated so far: **5/8**. No controller gains, DWA parameters,
navigation timeout, observation budget, operational gate or success criteria
have been adjusted.

Launch05 still timed out. After the surface correction, measured velocity now
tracks commands (e.g. 15–20 s mean commanded/measured yaw rates
−.1530/−.1541 rad/s), but the base alternates XY control and final rotation.
It enters the .06 m XY tolerance at27.311s; none of its in-tolerance samples
meet yaw acceptance. No target↔robot contacts were recorded in this attempt.
Upstream DWA1.17.3 unconditionally resets its XY latch on every `setPlan`;
the configured2Hz periodic global planner repeats this for the same goal.
For launch06, common SIM point-goal navigation uses planner_frequency=0
(new-goal/control-failure replanning). Live local costmaps and collision
checking,15Hz control,.06m/.08rad tolerances and120s timeout remain unchanged.
This is a documented navigation configuration revision, shared by both methods.
Activated **6/8**; launch05 remains a failure.

Launch06 returned MoveBase success, refined, reached D_exec, descended and
closed, but again failed lift stop-velocity acceptance. Its measured arrival
XY error was98.66mm despite entering the60mm latch earlier. This is NOT yet
an accurate-candidate-arrival pass. The base stayed stable during manipulation;
the displacement occurred in final rotation. Another frame bug is explicit in
the Gazebo API: SetLinearVel commands the ODE center of mass, while ROS Twist
commands base_link origin. Converted base CoG is(.024387,.003131,-.102779)m.
Apply v_CoG=v_base+omega×r_CoG (no dynamics/tuning change), and require the
fresh actual final pose to meet the original60mm/0.08rad limits before handoff.
Unit tests cover numeric twist transport and rejection of historical98.66mm
latch overshoot. Launch07 will verify the combined correction on the same Easy
Ground segment. No success criterion is loosened; no historical result edited.

Final online status: **8/8 activated, all eight completed, zero startup-invalid
runs, no retries beyond the cap.** Launch07 passed actual arrival at59.414mm /
0.068291rad, fresh refinement and D_exec, then failed the same wrist lift check.
Launch08 used the final startup for Moderate at-candidate observation and a
single post-failure non-executing IK/collision probe. Its third camera view
refined successfully; all six collision-aware grasp IK calls failed. The
diagnostic kinematic solution collides with BUNKER's base at forearm, wrist1,
left finger and left outer knuckle. The diagnostic solution was not executed.

The bounded online batch is stopped. Final results, review and regression
disposition are in [GROUND_EXECUTION_BATCH_RESULTS.md](GROUND_EXECUTION_BATCH_RESULTS.md).
