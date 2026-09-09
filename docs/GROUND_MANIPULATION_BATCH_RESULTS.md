# Ground manipulation completion — bounded development results

Status: **8/8 starts complete; stop at the development checkpoint.** This is
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
|06|Moderate Generic/source624, full-robot target/preshape/payload|yes|first IK/approach succeeds; actual pregrasp finger/chassis collision stops descend; no close|
|07|Easy Ours, sole complete aerial E2E, full-robot path|yes|3 windows→confirmed→real navigation/refine/pregrasp/descend; predicted knuckle/target collision stops before close|
|08|Moderate Ours/candidate000009, contact-height correction|yes|full Ground sequence including modeled payload, real grasp/lift/retention passes; TCP149.537mm, target148.902mm|

The original04 attempt record is preserved; its counterfactual solver failure
is a platform numerical failure, not evidence that the Ground task is
kinematically infeasible. There are no outcome-selected retries or substituted
successes. Generic/Ours share both development flags, planning and execution;
these different historical stations are not a method-effect estimate.

## Whole-robot failure findings and one geometry correction

06 is an actual clearance-limited execution failure, not a scene API error:
public link TF and independent JointState FK find0.023113/0.022485mm interior
mesh vertices. Nominal pregrasp has only0.404698mm separation. Actual TCP is
2.359mm closer to the chassis, with approximately0.28deg rotation error.
Gripper q drifts more closed and partly compensates, rather than causing the
overlap. The unchanged execution tolerance permits this small tracking error;
zero-collision nominal IK is insufficient for robust execution at this station.
The explicit pre-descend check correctly stops. This is not proof that every
IK/approach at this base is impossible, nor a reason to ignore tiny collisions.

07 has3 confirmed candidates after0→0→3 across the three actual windows.
Active time52.793sim seconds; Ground navigation33.228s, refine12.183s,
pregrasp4.677s and descend5.147s. Its original D_exec is true but retrieval is
false: D_exec marks the pregrasp stage, not complete grasp feasibility.

The rejected07 closure sweep is hypothetical: no physical close command ran,
and one pad would contact before the predicted knuckle collision. Offline
triangle/box checks reproduce forbidden right-inner-knuckle overlap in7 of41
closure samples at the measured pose, but not the nominal pose. Excessive
insertion plus tracking error explains this veto; a real object might move or
stall under one-sided contact, which this static sweep does not predict.

The old0.0156m pad-edge calibration is for the fully-open AG95. Rendered URDF
and actual pad mesh independently give

`w(q)=.0952+2L[cos(alpha+q)-cos(alpha)], L=.055m, alpha=44.691deg`.

For the physical53mm target, q_contact=.4573744071rad and downward pad shift
`delta=L[sin(alpha+q_contact)-sin(alpha)]=.0132905621m`.
Using the contact-configuration edge `.0156-delta=.0023094379m` raises the
refined Ground pregrasp/grasp/lift equally by13.290562mm while keeping the
original20mm overlap, all relative heights, yaw and XY. It is not a fitted
offset or a relaxed collision/force criterion. Preshape, physical target size,
real close command and all thresholds stay unchanged.

The corrected measured07 geometry has zero forbidden AG95/target or chassis
intersections across the same41 samples. Nominal pad insertion is20.040562mm;
the40.56µm discrepancy is existing open-edge rounding. Measured insertion is
23.96–24.42mm, and≈3.99mm lateral misalignment remains: this alone does not
prove bilateral grip or loaded lift. The last online run tests that distinct
question at the other archived Moderate station, not a retry of07.

Source before this correction: AGENT6134508/SIM86abd7a; corrected Ground
generator/cache: AGENT6d1dba2/SIMc12d8ba. A5 reverse/forward approach and actual
execution share the generator. Initial aerial/RM4D queries remain unchanged;
fresh Ground refinement is revalidated with the new physical insertion.
All original06/07 results are preserved.

[Measured source624 clearance](../outputs/development/ground-manipulation-batch/moderate-diagnostic/LAUNCH06_REPORT.md) ·
[Recorded closure geometry comparison](../outputs/development/ground-manipulation-batch/moderate-diagnostic/LAUNCH07_REPORT.md).

## Final Moderate Ground result and remaining limits

08 uses the **other archived exact base pose**, not a reselected alternative
at source624: `(2.530400777,-0.301563679,-2.617993878)` in map. Fresh D435
refinement gives target center `(2.058472707,.079374840,.058529088)` and
yaw `.055768414`. Corrected grasp TCP map z is `.093719650m`; actual preshape
opening65.418mm remains strictly above required55mm.

The first IK, reverse approach, pregrasp trajectory and forward continuation
pass. Actual pregrasp reaches D_exec; descend completes, five whole-model
pre-contact closure samples pass, physical fingers close, and real grasp
confirmation precedes planning attachment. `GROUND_PAYLOAD_MODELED` records
the fresh measured TCP and only four intended finger/pad touch links. The
attached-payload current-state check and collision-aware Cartesian lift then
pass. All original physical grasp/lift and controller gates remain active.

The gripper action itself reports the expected **contact-stalled path abort**
(-4,0.080008rad), not ordinary trajectory success. The unchanged existing
stall gate accepts it only with fresh real grasp confirmation and measured
closure (q≈0.4666rad). Bagged confirmation remains true in all122 recorded
samples from payload modeling to task end. Four arm actions, including the
camera move, return SUCCESS. No planning-scene/service snapshot was recorded:
payload verification is from gated events and reviewed code, not an independent
scene dump. The event follows successful ApplyPlanningScene and whole-robot
payload validity; lift separately rechecks that attached state.

- TCP rises149.537mm and physical target148.902mm; both the execution and
  independent physical checker pass. No physical weld or target teleportation.
- The older derived metrics field `physical_success` remains null; it is not
  rewritten. These physical numbers/status come from `physical_summary.json`,
  `attempt.json` and the separate native dynamics audit.
- Whole lift-stage relative target-in-hand movement maximum2.164mm.
- Existing0.581s retention stage: target height changes+0.00444mm and relative
  target-in-hand displacement maximum0.0404mm; actual grasp remains confirmed.
- All six public arm feedback streams again exactly match same-stamp1ms
  position increments. Independent wrist quaternion-rate error is at most
  1.02e-7rad/s during lift. Raw native bias persists (native lift integral
  −0.45796rad versus actual angle change+0.00016666rad), so success was not
  inferred from removing or overwriting the raw diagnostic discrepancy.
- These are two independently checked successful feedback-mode trajectories
  (Easy05 and Moderate08), not a population reliability estimate. Retention is
  the existing short check, not a long-duration payload-holding study.

In the assessed Ground executions in this batch, the remaining failures concern
**whole-robot clearance under execution and perception error**. This does not
clear the unchanged, separately parked Hard sensing/support limitation or
reassess the operational gate. Source624 has a
near-zero planned hand/chassis margin and can collide within normal accepted
TCP tracking errors. The other Moderate station supports an executed full
Ground chain, so the scene is not globally manipulation-infeasible. Real
bilateral closure at the recorded07 lateral error and full corrected aerial
E2E remain unverified: the sole aerial invocation failed before the insertion
correction and is not retroactively relabeled. No ninth start was used to
obtain a more favorable complete-E2E result.

Further development should integrate whole-manipulator/hand/payload clearance
and measured tracking accuracy into candidate/approach feasibility, then use
a separately budgeted complete E2E regression. It should not treat one
zero-collision IK or D_exec as a robust full-chain certificate. No such
selection change, additional run, operational-gate change, or formal study
was implemented in this batch.

## Verification and disposition

Final method/execution implementation: AGENT6d1dba2 and SIMc12d8ba (later
checkpoint commits add results/documentation). Independent reviews cover
feedback storage/control semantics, native MoveIt scene application, shared
Ground geometry, development-only arguments and preservation of failures.
Native ROS adapter suite133 and SIM suite109 pass with no skips; final core
suite645 passes with22 expected environment skips (overlapping suites, not
summed). Actual Pluginlib construction,
rendered-mesh geometry and native C++ PlanningScene round-trip tests are
included, not only mock messages. Catkin builds/install succeed.

There are8 actual startup invocations, including the rejected numerical
solver contrast04; none is hidden or replaced. No startup-invalid repeat or
ninth invocation. All valid failures are retained; seven arrival-conditioned
Ground invocations are distinguished from the single complete aerial attempt.
No Generic/Ours effectiveness inference is drawn from this diagnostic batch.

Owned ports11951/11952 are no longer listening; no owned runtime remains.
Pre-existing unrelated Gazebo PIDs1201811/1201814 and four unrelated old A5
directories were preserved. Large raw bags/native traces remain local and
ignored; compact records, observations, scripts and reports are versioned.
Default native-feedback and legacy planning modes remain reproducible; the
new development mode uses explicit `--integrated-joint-velocity` and
`--full-robot-manipulation` for either method. Keep PRs Draft, no merge or formal
matrix. No evidence/security/benchmark/approval framework was added.

Detailed last-run audits: [full-robot execution](../outputs/development/ground-manipulation-batch/moderate-diagnostic/LAUNCH08_REPORT.md),
[physical motion/feedback/retention](../outputs/development/ground-manipulation-batch/dynamics-analysis/launch-08-public-full/NOTES.md).
