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

**1/6 used.** Start 1: `launch-01-natural-latest`, development slot 4
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
