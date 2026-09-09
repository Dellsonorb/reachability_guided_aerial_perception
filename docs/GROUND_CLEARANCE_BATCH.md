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

**0/6 used.** Start 1 planned: `launch-01-natural-latest`, development slot 4
(Easy/Ours), complete normal aerial pipeline, latest production implementation
before clearance changes; native physics trace enabled, no diagnostic motions.

Further clearance details will be recorded before activating their behavior.
Independent checks and version records are limited to what directly validates
robot geometry and execution; no new evidence/lifecycle framework.
