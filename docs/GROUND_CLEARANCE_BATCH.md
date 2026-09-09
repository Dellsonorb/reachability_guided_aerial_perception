# Ground execution clearance — bounded development batch

**Goal:** test the latest natural aerial-to-Ground retrieval, then improve the
shared execution layer's whole-motion geometric clearance. This is development,
not a formal experiment or a calibrated safety guarantee.

## Preserved versions and budget

Previous AGENT `c97298f` and SIM `625b84c`, all eight prior starts, and their
failures remain unchanged. Work branches: `feature/dev-ground-clearance` and
SIM `feature/fix-ground-execution-clearance`, in the existing linked workspace.

At most **six Gazebo startup invocations**, including startup failures. The
runner's existing development scenes and normal initial pose are retained.
Start 1 is a natural Easy/Ours aerial E2E with latest full-robot manipulation
and explicitly configured SIM integrated-pose velocity feedback. No archived
handoff, manual candidate/base pose or retry-to-success. Preserve native
physics diagnostics, used only for offline motion/retention checks.

Allocate subsequent starts to targeted Ground comparisons of whole-path
clearance after offline diagnosis, reserving at most one further natural E2E
after a substantive shared execution revision. Do not expend all remaining
starts solely to obtain a successful E2E. Record each purpose before launch.

## Design and implementation sequence

1. Run current focused regressions. Permit existing read-only native physics
   tracing for full **development** E2E as well as Ground replay; still reject
   formal/non-development tracing and keep post-failure load probes replay-only.
   Test these restrictions before changing the runner.
2. Run start 1 using ordinary P450 perception, active observations, exact
   candidate selection, navigation, real D435 refine, close and loaded lift.
   Distinguish task outcome from independent robot/object motion diagnostics.
3. In parallel offline work, extract desired-versus-measured full-path errors
   from existing runs and inspect installed MoveIt whole-robot self/world/
   attached-object distance APIs. A nominal collision-free IK is not sufficient.
   Do not derive a margin from the requirement to rescue any particular source.
4. Select and document a small shared execution revision using existing MoveIt:
   complete robot + runtime perceived object before grasp, narrowly allowed
   intended finger contacts during closure, attached perceived payload afterward.
   Evaluate bounded reasonable IK/approach/candidate alternatives before motion,
   with a documented geometry/error-based development clearance. Keep actual
   collision and grasp checks. Uniform link padding alone does not cover self
   collision and can incorrectly exclude intended contact; check installed APIs.
5. Implement with failing geometric/selection tests, then targeted Ground runs.
   Generic/Ours share all execution changes. No GT action geometry, altered raw
   ground votes, relaxed physical criteria, or concealed fallback after failure.
6. After a substantive locally validated revision, one natural E2E with
   program-selected station if the remaining budget allows. Review relevant
   regressions, preserve all outcomes, document capability limits, commit/push
   Draft branch checkpoints, stop. No formal matrix or Hard coverage changes.

## Run ledger

**5/6 completed; batch stopped, no sixth start.** Start 1: `launch-01-natural-latest`, development slot 4
(Easy/Ours), complete normal aerial pipeline, latest production implementation
before clearance changes; native physics trace enabled, no diagnostic motions.
AGENT `7320b9e`, SIM `625b84c`: three observation windows, program-selected
source585 at `[2.430140806, -0.621187969, 2.268928028]`; real navigation and stop
succeeded. The task then failed at initial perceived-target scene update:
`target planning-frame transform is stale`, before camera observation. Preserve
the original `VALID_TRIAL / retrieval_success=false`; do not relabel it as lift
or clearance failure. Native TF replay shows map-to-base age only17ms at failure
while MoveIt initialization can leave this Python subscriber cache1.29s old.
Fix availability-versus-freshness waiting without relaxing the500ms criterion.

Further clearance details will be recorded before activating their behavior.
Independent checks and version records are limited to what directly validates
robot geometry and execution; no new evidence/lifecycle framework.

## Reviewed bounded execution revision: chassis-clearance-v1

Use a planning-only copy of the authoritative URDF chassis collision BOX,
preserving its origin/rotation, enlarged by12mm on each half-extent. This rounds
the prior four recordings' maximum11.491mm corresponding surface displacement
up to1mm. It is an observed development allowance, not calibrated uncertainty
or a guarantee for new configurations. Exempt only the all-fixed chain from
the model root; arm-mounted fixed camera/pads remain blocked. Keep the original
physical robot and empty-group collision checks. Target/guard remains forbidden
through world, narrow finger-contact and attached-payload phases. No universal
margin is asserted for other pairs and no extra payload/floor margin is added.

Before Ground handoff, evaluate at most four already-confirmed exact per-cell
winners, in the existing relevance/stable order. Each uses two brick-symmetry
grasp yaws and at most three fixed IK seeds (measured/current, original validated
RM4D configuration when available, deterministic alternate). No unconfirmed
candidate or non-winner is introduced. Both methods use this identical screen.
Aerial perceived geometry only predicts execution; replan and check again
after actual arrival and accepted D435 refinement. Do not retry execution after
a physical/controller failure to conceal that outcome.

Whole manipulation preflight includes pregrasp approach, descent, closure and
prospective attached lift. Nominal pad contact remains geometry-derived; also
check the observed loaded articulation up to0.522rad, rounding the previous
0.521259581 maximum up1mrad. This changes neither the0.70 force command nor real
contact confirmation; it is not a universal load bound. Hypothetical attachment
has a separate planning API and restores the original scene before actuation;
the actual attachment still requires fresh real grasp confirmation/measured TCP.
Check each actual retimed arm command using the controller's cubic/quintic
curve sampled at the explicit SIM1ms step, with bounded wall time and no PASS
for incomplete traversal. Preserve MoveIt's collision-aware path generation.

### Start 2, declared before invocation

`launch-02-source624-clearance`: development slot6, archived Moderate/Generic
handoff from `operational-batch/launch-09-moderate-generic`, explicitly
conditioned on arrival at source624. AGENT `7da6fde`, installed SIM `1f47fe4`.
Enable full robot, integrated SIM velocity, chassis-clearance-v1 and native
diagnostic trace. Use fresh D435, bounded whole-chain search and actual physical
gates. Purpose: determine whether insufficient-clearance branches are rejected
before dangerous approach, or a genuinely feasible alternative exists. This
is not a natural aerial result or a new Generic/Ours effectiveness comparison.
This invocation consumes start2 of6 even if startup fails.

Start2 result: fresh D435 refinement and checked camera motion succeed. All
six bounded grasp IK branches reject (`NO_IK_SOLUTION`, -31). No pregrasp,
descent or physical closure is sent; the local task remains a failure. This
does not prove that every possible grasp at this station is impossible.

### Start 3, declared before invocation

`launch-03-source664-clearance`: development slot5, archived Moderate/Ours
handoff from `operational-batch/launch-08-moderate-ours`, explicitly conditioned
on arrival at source664. Same AGENT `7da6fde` and SIM `1f47fe4`, flags and
physical gates as start2. Purpose: test whether another recorded station can
complete actual manipulation under the new shared full-chain clearance checks;
not a retry of source624, nor evidence of policy superiority. This invocation
consumes start3 of6 even if startup fails.

Start3 remains a valid failure: actual camera, refined pregrasp/D_exec, descent
and contact-confirmed grip pass; loaded lift checking times out after60s before
any lift command. Native transport-only diagnosis confirms per-sample TCP
reconnection overhead. No timeout or sampling rule is relaxed.

### Start 4, declared before invocation

`launch-04-source664-transport`: same slot5/archived source664/arrival condition
as start3, AGENT `7da6fde`, installed SIM `fbb191b`. This single engineering
contrast changes only the serial read-only validity proxy to persistent
transport. It retains the exact same geometry, branch set, curve sampling,
60s check timeout and all physical criteria. Purpose: test actual loaded-lift
checking/runtime completion after a measured transport optimization. Preserve
start3's failure, regardless of outcome. It consumes start4 of6; do not retry
this station again just to get success. Reserve one later natural E2E, with
no archived candidate or conditioned arrival.

Start4 completes actual pregrasp, descent, real grip/attachment, loaded lift
and retention; physical target rises149.139mm, TCP149.731mm. Loaded command
checks18,152states before dispatch, without extending the60s guard. This is a
single local engineering contrast, not general reliability or policy evidence.

### Start 5, declared before invocation: final natural E2E

`launch-05-natural-clearance`, slot4 Easy/Ours, AGENT `7da6fde`, installed SIM
`fbb191b`. Same normal initial pose, active perception and three-window budget.
Enable integrated SIM velocity, full robot and shared chassis-clearance-v1;
retain native diagnostic logs. **No replay, arrival conditioning, candidate
override or manual station.** Existing confirmed exact winners must pass the
common bounded execution screen; record their actual selection or rejection.
This is the sole final natural E2E after the substantive shared revision and
local validation. Preserve its outcome; do not repeat a valid failure to win.
It consumes start5 of6. Start6 remains unused unless a genuine pre-task startup
invalidity requires the single remaining engineering allowance.

Start5 result: natural complete aerial-to-Ground retrieval passes. Program
chooses source585, rank1, after the common whole-chain preview; actual arrival
and D435 trigger revalidation before execution. Target/TCP rise148.951/149.480mm
and fresh real grasp persists through the original0.829sim-second retention
interval. Allfive invocations are VALID_TRIAL; preserve three failures and two
successes as development results, not a policy success-rate estimate. Stop at
this checkpoint with one unused startup allowance and no formal matrix.
