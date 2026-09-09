# Ground manipulation completion — bounded implementation plan

> For agentic workers: use subagent-driven-development for independent tasks,
> systematic-debugging for cause isolation, and test-driven-development for fixes.

**Goal:** establish and repair the Easy lift failure, then test complete-robot
manipulation feasibility and perform a small number of Ground regressions.

**Architecture:** reuse the Ground-only replay and real execution stack. First
add opt-in diagnostic sampling at the simulator physics boundary, with no
actuation change; compare loaded and unloaded behavior. Put proven execution
fixes on a separate SIM branch; share any planning revision across Generic/Ours.

**Tech stack:** existing ROS Noetic, Gazebo11/ODE, MoveIt, native Python3.8
adapter and Python3.10 offline code. No new dependency or experiment framework.

## Budget and preserved boundaries

At most **eight Gazebo startup invocations**, including failures. Reuse
archived runtime/perception inputs; never use simulator GT for action geometry.
Each run gets a new directory under `outputs/development/ground-manipulation-batch`.
No repeated full task until success. Each diagnostic invocation has a declared
finite list of phases; post-failure probes do not change its original failure.

Priorities: starts1–4 for targeted lift diagnosis/fix,5–6 for complete-robot
Moderate/Easy Ground validation,7 for one full aerial E2E **only after local
grasp/lift passes**,8 reserved for one diagnosed contrast/regression or invalid
startup. Allocation may move between local tasks, never beyond8; no full E2E
is obligatory if local execution still fails. Report used and unused starts.

Historical AGENT4d5116d and SIM7d2abf6 plus their configurations/results remain
preserved. Work in current AGENT feature/dev-operational-consistency; SIM uses
feature/fix-ground-manipulation-completion based on7d2abf6. No operational gate,
Hard scan model, ground votes, RM4D asset, formal matrix or final-test changes.
No relaxed stop/collision/grasp/lift acceptance. Controller or trajectory
changes require a specific diagnosed physical/control reason and comparison.

## Design decisions

Increasing settling time cannot distinguish a feedback discrepancy from actual
oscillation. Replacing velocity with finite differences risks hiding unresolved
physics. Start instead with simultaneous native hinge angle/rate and parent/
child pose/angular velocity at every physics step, before/after base actuation
and after physics. Initially, difference velocities remain diagnostic, never
acceptance. Any later feedback revision requires independent joint/link-motion
validation first, as recorded in the execution decisions below; a controller
PASS alone is not that validation.
Record physical target motion separately for loaded/unloaded and retention
checks, never pass it to perception, planning, support or ranking.

For load isolation, after a retained failed lift, record a short loaded hold,
open the real gripper, then record the unloaded hold; no target reset or
teleportation. The primary result remains failed. Additional independent empty
motion/base coupling contrasts will be chosen from the measured discrepancy,
not tuned against a PASS. Each such contrast is recorded before execution.

Moderate must use the whole BUNKER/AUBO/AG95 collision model plus runtime
perceived task object. Inspect world-object and attached-object semantics,
then a bounded deterministic set of IK branches and physically valid approach
poses. Check observation/pregrasp/descent/closure/lift, not one isolated IK.
One colliding solution is not a proof of infeasibility. Any revised selection
is versioned and shared; original exact winners/results are not overwritten.

## Tasks

- [ ] Baseline: run existing Ground adapter/manipulation tests; inspect current
  SIM controller, mimic, base-actuation and collision-scene paths. Preserve
  the four unrelated old untracked A5 directories and existing external SIM.
- [ ] SIM diagnostic-only `ground_dynamics_trace.hh` plus integration in
  `bunker_planar_move_plugin.cpp`: enable only via
  `P450_GROUND_DYNAMICS_CSV`; default off. Emit time/phase, wrist angle/rate,
  axis, parent/child pose/angular velocity, base pose/twist, joint effort and
  target pose/twist. Use read-only physics APIs. Test default-off behavior,
  before/after/end placement and a compiled scalar angular projection case;
  build existing package before first invocation.
- [ ] AGENT `run_a6_attempt.py`: opt-in Ground diagnostic argument sets the
  CSV destination before runtime spawn and records its meaning. Reject it
  outside Ground development replay. Include native model/link states only in
  the diagnostic bag, not in algorithm inputs. Unit-test opt-in and exclusions.
- [ ] AGENT `run_ground_sim.py`: optional bounded post-failure load contrast;
  original FAILED occurs first, then loaded hold/open/unloaded hold with
  explicit phase events. Use original gripper actions; do not publish LIFT,
  D_exec or a replacement success. Tests prove failure preservation and order.
- [ ] Launch01: original Easy camera/manipulation at archived exact candidate,
  original control settings, physics logging and loaded/unloaded contrast.
  Compare within-step vs between-step changes to distinguish source/readout,
  actual oscillation, gripping load and base transport. No feedback substitution.
- [ ] For the layer actually implicated: capture a failing numerical/behavior
  regression, implement the minimal correction, verify it, then use a declared
  local contrast with unchanged acceptance and independent motion/retention
  checks. Retain each earlier failure and configuration. Record decisions here.
- [ ] Offline Moderate full-robot scene/IK diagnosis; then implement only the
  necessary shared collision-aware branch/approach handling with tests. Check
  target as a world obstacle before grip and attached geometry after grip,
  while preserving legitimate contact only at intended finger surfaces.
- [ ] Within remaining startup cap, test complete Ground segments at distinct
  archived stations; if local grasp/lift passes, one original full aerial E2E.
  Report trajectories/object retention and capability limits, not merely PASS.
- [ ] Independent review, relevant regression, final report, commit/push Draft
  checkpoints, verify owned runtime cleanup and worktree state; stop. No formal.

Self-review: the eight-start bound includes all failures; finite post-failure
contrasts cannot replace task outcomes. Physics and target truth are diagnostics
only. Empty motion and released-object checks cannot count as retrieval. No
unresolved design question needs user approval under the current autonomy.

**Startup ledger at plan commit: 0/8.**

## Execution ledger

1. `launch-01-easy-loaded-unloaded`: original Easy Ours archived exact station,
   conditioned on arrival; original gains/controller/trajectory behavior. Opt-in
   physics trace and, only after recorded lift failure, two2s holds around one
   real gripper opening. Independent logger review and69 Ground/adapter tests
   passed before activation. No post-failure phase may revise the task outcome.
   Result: original controls passed; TCP149.76mm and target149.40mm actual lift,
   grasp retained. No release contrast triggered. This is a success control,
   not a fix claim; compare its exact joint trajectory with historical failures.
2. `launch-02-easy-generic-original`: archived Easy Generic station that failed
   in prior batch launch03, again conditioned on arrival with unchanged control
   and the same trace/post-failure contrast. Tests the previously failing local
   station rather than repeating until success.
   Result: native wrist speed0.1897rad/s caused retained lift failure. During
   the post-command2s interval, all1kHz speeds are negative; integral−0.336535rad
   versus joint/relative-link rotation−0.00001788rad. Base setters change neither
   wrist angle nor velocity immediately. Physical target rose149.30mm and stayed
   held; this does not override the failed controller acceptance. Closed hold
   completed, opening failed before open hold: stale desired0.700rad versus
   actual0.517rad violates the unchanged0.08rad opening-path constraint.

Source-based next hypothesis: Gazebo11 `quickstep_update_bodies.cpp` integrates
pose using `caccel_erp`, then removes that ERP contribution from stored body
velocities. This exactly permits the observed persistent native-rate/pose
disagreement. Verify real release first, then one ODE `world` solver contrast
with unchanged robot/contact/control/acceptance. Do not replace native velocity.

3. `launch-03-easy-measured-release`: same archived Easy Generic station and
   QuickStep; only production behavior change is fresh measured q/dq as the
   explicit opening-trajectory start (future t=0 point). Endpoint, duration,
   close behavior and all constraints unchanged. Independent review and5
   native release tests plus70 AGENT tests passed. Record gripper goal/results
   as well as states. Retain any primary lift failure before the release probe.
   Result: lift still fails0.2105rad/s. Revised release starts from actual
   0.51122rad, native0.26772rad/s and no longer aborts immediately; later it
   genuinely lags the requested opening and violates the same0.08rad path
   bound after1s. Do not claim complete release or an empty-hand hold.
4. `launch-04-easy-world-solver`: same archived Easy Generic station,
   measured-start opening retained, only ODE solver `quick`→`world` in an
   attempt-local copy of the current platform world. Timestep1ms, all contact,
   friction, robot, gains, motion generation and success constraints unchanged.
   Native rate vs actual joint/link motion and physical object holding are the
   endpoints; a PASS alone is insufficient. No default SIM world change yet.
   Result: rejected alternative. Direct solver produces LCP `s<=0` numerical
   errors and fails initial opening, before camera/grasp/lift. Keep original
   recorded task failure; diagnose it as the counterfactual solver's platform
   numerical failure, not evidence of Ground-task infeasibility. Do not adopt
   `world` or relax contacts to make it work.

Launch03's release attempt did change the constraint regime: after48.520s,
native wrist-rate median−0.000041rad/s versus−0.163121 while firmly closed.
Target remains elevated, with only3.317mm relative-hand displacement; this
isolates a gripping-constraint contribution, **not** an unloaded-weight contrast.

Next correction under independent review: a SIM-only hardware-feedback adapter
reporting explicitly named per-physics-step **integrated-pose joint velocity**,
not QuickStep's stored post-ERP velocity. Before activation, validate position
increments against independent link-relative quaternion motion over substantial
unloaded wrist rotation and loaded holding. Preserve native diagnostics and
all control/gain/collision/acceptance settings. Do not infer validity from PASS;
require actual motion-rate agreement and physical retention in the next run.

5. `launch-05-easy-integrated-feedback`: same Easy Generic station, original
   QuickStep/robot/gains/trajectory/acceptance, with the explicitly opt-in SIM
   `IntegratedVelocityRobotHWSim` interval-average feedback. Stock readSim runs
   first; velocity elements update in place, writeSim remains stock. Invalid
   native samples and first/reset samples are not replaced with invented zeros.
   Raw native trace is retained. Before activation:8 numeric/registration tests,
   actual offline Pluginlib construction,36 relevant SIM regressions and132
   AGENT tests passed. This run must validate nonzero pre-close motion, loaded
   motion/hold, public corrected feedback, and real object lift/retention.
   New target/payload/preshape planning remains disabled to isolate this change.
   Result: CHECKS_PASS. Same archived Generic station now executes the lift;
   checker TCP149.7548mm/target149.4129mm, with physical grasp retention. Across
   whole observation/pregrasp/lift/retention phases, all six public arm joints
   exactly match same-stamp preceding1ms native-position increments; positions
   also match exactly, with no missing/nonfinite samples. Independent wrist
   relative-quaternion rate errors max0.003887rad/s during observation,
   0.0006454 during pregrasp, <8e-8 during loaded lift/retention. Actual wrist
   travel is +2.4653/−3.1963rad before close, not suppressed motion. Native ODE
   bias remains−0.139rad/s median in retention, while real1kHz pose-rate max is
   0.08399rad/s. Target-hand movement max1.466mm during lift and0.1446mm during
   the existing0.797s retention stage. Only49.4ms elapses from nominal lift
   endpoint to stage end: no claim of an added2s settling check. This is a
   successful independently checked development contrast, not a reliability
   rate estimate or a fix to the underlying ODE solver. See
   `outputs/development/ground-manipulation-batch/dynamics-analysis/launch-05-public/`.

Before launch06, offline review caught two defects in the new MoveIt scene
helper: ignoring Noetic CollisionObject.pose when attaching a round-tripped
world box, and issuing redundant world REMOVE after attachment ADD. Both must
pass a real native PlanningScene round-trip test before activation. This new
planning mode remains opt-in and shared across methods; it was disabled in05.

6. `launch-06-moderate-full-robot`: archived Moderate Generic source624 at its
   exact station (arrival-conditioned), QuickStep plus integrated feedback and
   the shared full-robot perceived-target/payload/preshape path. No candidate,
   target pose, TCP insertion or IK budget change. Record physics and one
   existing read-only post-failure IK diagnosis; never execute a diagnostic
   collision-disabled solution. Native scene round-trip18 tests and runtime19
   tests independently passed; full SIM review74 tests passed. Both attachment
   defects fixed before activation. New path built successfully, defaults off.
   Result: camera/refine and measured preshape pass (q0.290640rad,
   conservative aperture65.448mm). First grasp IK succeeds, both approach
   fractions1.0, pregrasp executes and original D_exec is recorded at37.413s.
   At37.424s measured whole-robot validity rejects left_finger/base_link before
   descend; close/lift not reached. No diagnostic solution is executed. Thus
   candidate planning improved, but full manipulation is not yet feasible at
   the measured pregrasp; D_exec does not certify the entire grasp chain.

7. `launch-07-easy-full-e2e`: original development Easy scene, Ours, normal UAV
   initial observation/A1–A4 loop and Ground navigation, no replay/arrival
   conditioning. Shared integrated feedback plus full-robot manipulation.
   Same scene/config/initial pose/budget/gates; no GT-derived algorithm geometry.
   This is the sole full aerial E2E invocation in the batch, after05's locally
   validated real grasp/lift. A failure remains a failure, not a reason to
   repeat the complete aerial task until success.
