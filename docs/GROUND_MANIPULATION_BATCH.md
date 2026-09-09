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
and after physics. Difference velocities remain diagnostic, never acceptance.
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
