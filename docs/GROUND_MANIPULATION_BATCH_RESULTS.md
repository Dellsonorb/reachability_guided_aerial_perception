# Ground manipulation completion — bounded development results

Status: batch in progress; maximum8 Gazebo starts including failures. This is
development diagnosis, not a paired comparison or formal experiment. Historical
results are unchanged. The current operational gate, Hard coverage issue,
RM4D baseline/task asset, scene seeds and success thresholds were not changed.

## Lift: established cause and correction

The repeated wrist failure is a **SIM velocity-interface/QuickStep semantics
discrepancy under contact**, not merely a stale ROS subscriber and not a proved
real sustained wrist rotation. A failing loaded hold sampled every1ms has
native wrist velocity always negative; its integral is−0.336535rad while
actual joint and independent relative-link rotation change only−0.00001788rad.
Native rate agrees with stored link angular velocity projected on the joint
axis. Immediate BUNKER velocity setters do not rewrite the wrist state.

Gazebo11.15.1 QuickStep integrates pose with ERP constraint correction and then
removes that contribution from stored velocity. Real partial gripper opening
reduces the wrist-rate bias while the target remains held, isolating a contact
constraint contribution; it does **not** prove a payload-weight-only effect.
An ODE direct/world solver contrast numerically fails and is rejected. Neither
robot friction/contact settings nor acceptance thresholds were relaxed.

The SIM correction is an explicit opt-in hardware feedback convention:
`P450_GROUND_INTEGRATED_VELOCITY=1` reports consecutive integrated joint-position
increments divided by the actual hardware-read interval, retaining stock
position unwrapping, writeSim, PID gains, limits and physics. Native feedback
remains the default and raw diagnostic reference. No clipping, smoothing,
deadband, quiet-window selection, nonfinite concealment or invented zero is
used. This changes the reported velocity quantity; it does not repair ODE.

Finite differences were validated **before** using them for acceptance: large
unloaded wrist rotations and loaded holds agree with independent parent/child
quaternion motion. In online launch05, all six public arm joint and controller
actual velocities match same-stamp preceding1ms position increments exactly
through observation, pregrasp, descend, close, lift and retention. The wrist
rotates +2.4653/−3.1963rad during observation/pregrasp, demonstrating that real
motion is retained. Independent quaternion discrepancy is at most0.003887rad/s
in observation and <8e-8 in loaded lift/retention. There are no timing gaps or
unmatched/nonfinite samples. Other arm joints have position references, not
independent simultaneous per-link quaternion references; the gripper has only
public records and shared implementation/numeric tests, not native1kHz traces.

Physical checks, independently from the action result in05:

- TCP rises149.755mm; target rises149.413mm by the unchanged physical checker.
- Whole lift-stage target-in-hand displacement maximum1.466mm.
- Existing0.797s retention stage: target height change−0.00217mm and relative
  movement maximum0.145mm. This is not a new prolonged-hold test.
- Real wrist pose-rate during retention has1kHz maximum0.08399rad/s; it is not
  motionless. The old native velocity bias remains visible in the raw data.
- One successful contrast establishes neither a reliability percentage nor
  coverage of all loaded contacts and all seven joints.

Full sample tables and scripts: [launch05 analysis](../outputs/development/ground-manipulation-batch/dynamics-analysis/launch-05-public-full/NOTES.md),
[initial contrasts](../outputs/development/ground-manipulation-batch/dynamics-analysis/COMPARISON.md),
[independent pose validation](../outputs/development/ground-manipulation-batch/dynamics-analysis/pose-velocity-validation/NOTES.md).

## Separate gripper opening defect

After a contact-stalled close, desired0.700rad versus measured≈0.517rad creates
an immediate discontinuity when opening starts implicitly from desired state.
The new opening trajectory starts from fresh measured q/dq, retaining the
original endpoint, duration and0.08rad path bound. Launch03 removes the initial
discontinuity; later actual tracking lag still aborts opening. Full release is
not claimed. Original lift failures and post-failure probe failures remain
recorded separately, and a probe cannot replace the task outcome.

## Moderate: robot and target geometry

At archived Generic source624, fully open AG95 fingers/knuckles intersect the
BUNKER chassis by≈12mm at the prescribed TCP regardless of arm IK. A180-degree
equivalent grasp yaw only swaps the colliding fingers. The RM4D bare-arm model
does not certify chassis/AG95/D435/payload clearance; FK calibration itself
agrees with the SIM arm model. Six IK failures are not proof that every branch
is infeasible, and a collision-disabled diagnostic solution is never executed.

The shared opt-in `--full-robot-manipulation` path now adds accepted runtime
aerial/refined target geometry to the existing complete MoveIt robot model.
It generates a target-width preshape using the old conservative aperture rule
and tracking tolerance, checks actual aperture and whole-robot state, keeps
target collision checks during observation/approach/descent, enables only the
four intended finger/pad contact links during closure, then carries a perceived
planning payload after real grasp confirmation using measured TCP geometry.
It creates no Gazebo weld and uses no simulator target pose for control.

Nominal preshape is not a clearance certificate: at source624 its nominal
hand/chassis separation is only0.625mm, and the old tracking interval includes
real collisions. Therefore actual full-model checks remain required. Gripper
sweep checks are spatially sampled, explicitly **not** continuous-clearance
proofs. Attached geometry is retained in lift/start states. Native MoveIt
round-trip regression caught and fixed object-level pose composition and
redundant world-REMOVE defects before online activation.

[Offline robot geometry and limitations](../outputs/development/ground-manipulation-batch/moderate-diagnostic/REPORT.md).

## Online ledger

Ground replays reuse a historically confirmed candidate and condition on its
arrival; they do not re-estimate confirmation or test Ground navigation. Only
the explicitly marked full E2E invocation can establish the entire chain.

| Start | Scene/station and contrast | D_exec | Physical manipulation result |
|---|---|---|---|
|01|Easy Ours station, original feedback/control|yes|grasp/lift/retention pass; TCP149.76mm, target149.40mm; success control, not fix evidence|
|02|Easy Generic station, original feedback|yes|grasp and physical rise; controller lift fails0.1897rad/s; original failure retained|
|03|same Generic station, measured-start opening|yes|lift fails0.2105rad/s; partial opening removes bias but later release tracking fails|
|04|same station, ODE world-solver contrast|no|LCP numerical failure during initial opening; rejected physics alternative|
|05|same station, integrated-pose feedback|yes|grasp/lift/retention pass with independent motion and holding evidence above|
|06|Moderate Generic/source624, full-robot target/preshape/payload|pending|running|

The original04 attempt record is preserved; its counterfactual solver failure
is a platform numerical failure, not evidence that the Ground task is
kinematically infeasible. There are no outcome-selected retries or substituted
successes. Generic/Ours share both development flags, planning and execution;
these different historical stations are not a method-effect estimate.

## Final review and checkpoint

Pending remaining bounded runs and final review. No formal matrix is authorized
or started by this batch. Source and derived reports will be committed on the
existing AGENT development branch and a separate SIM correction branch; raw
bags/native traces stay locally available, not added to Git. No new evidence,
security, benchmark or approval framework is introduced.
