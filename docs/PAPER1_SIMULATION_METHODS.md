# Paper 1 simulation methods — manuscript draft

This methods-only draft describes the prospectively frozen formal study
(`b8848ae`). Experiments are still running; it makes no outcome or superiority
claim. The [formal protocol](A6_FORMAL_PROTOCOL.md) and serialized
`configs/a6_formal.json` remain authoritative. Historical Pilot-1/2 results
are not part of the formal sample.

## System and policy

The simulated system combines a P450 aerial observer carrying RGB-D and MID360
sensors with a BUNKER mobile base, D435 refinement camera, AUBO arm and AG95
gripper. Research decisions use runtime perception and public map-frame TF,
not simulator ground truth. Simulator model state is used only to construct
the prescribed scenes, describe them offline and measure physical outcomes.
The frozen RM4D literature baseline remains unchanged. Ground retrieval uses
its separately documented task-domain runtime asset and explicit frame bridge;
this is not a modification or replacement of the original baseline map.

All ground fields use the same fixed 0.10 m map grid. A1 assigns a base cell
the maximum accepted candidate quality, with quality equal to minimum joint
margin divided by 0.5 rad and clipped to [0,1]. Reachability, valid IK, collision
and minimum-margin checks precede this quality. Solver residuals are diagnostics,
not capability weights. The 256-candidate validation budget leaves unexamined
cells UNASSESSED; this is not a complete capability map or an infeasibility claim.

A2 accumulates actual transformed LiDAR endpoints, without XY ray carving.
Ground and occupied height thresholds remain 0.02 m and 0.05 m, respectively,
relative to the known horizontal ground. At most one occupied or free vote per
cell and completed observation window enters raw A2, with occupied priority.
Its observation-deficit score is `u_N(x)=exp(-N(x)/2)`, not a calibrated
probability or entropy. Two ground votes without occupied evidence establish
raw FREE, whose deficit score need not be zero.

The approved object-aware v1.1 layer leaves all raw A2 arrays unchanged. It
conservatively associates occupied endpoints with TARGET, ENVIRONMENT or
AMBIGUOUS evidence using the accepted aerial RGB-D segmentation, registered
depth, public TF and known brick dimensions. The sensor/pixel-derived allowance
formula is fixed before trials; its value follows actual accepted depth and
intrinsics, never method outcomes. ENVIRONMENT and AMBIGUOUS block overlapped
footprints. TARGET requires continuous intersection of the expanded perceived
object and padded BUNKER rectangle; target-associated grid aliasing alone does
not imply collision. Real ground-support votes remain necessary for handoff.

A3 projects manipulation support over the padded BUNKER footprint, not one base
cell alone. A1 cell-center/best-yaw poses are discrete representatives; they are
not claimed to be the original IK-validated poses. Operational support excludes
blocked representatives without rewriting A1 relevance. The resulting field is
`U_task(x)=u_N(x) M_operational(x)`. The same operational-footprint function
assesses representatives and original exact handoff candidates.

For viewpoint v, Ours uses

`G_task(v)=(1-exp(-1/2)) sum_x V(v,x) U_task(x)`.

Generic replaces `U_task` with `u_N` over the A2 grid. Both subtract the same
flight-cost term, `0.25 * (distance_m + 0.25 * abs(wrapped_yaw_change_rad))`.
The common generator uses XY offsets {-2,0,2} m at
current altitude and eight yaw samples, subject to the same frozen flight
bounds. Tilted MID360 mounting makes yaw geometrically relevant. Visibility
tests ground-cell centers against the sensor range/FOV and 3D intersections
with 1 m assumed-height occupied-cell prisms. That height is a surrogate, not
a measured obstacle height; UNKNOWN does not occlude. Gain is an idealized
one-informative-endpoint observation opportunity, not expected entropy reduction
or a guarantee that a real scan will hit every predicted cell.

## Comparisons and execution

Four core methods run on every scene: RM4D-only, Fixed-view + RM4D, Generic NBV
+ RM4D and Ours. Generic/Ours share candidate generation, visibility, cost,
acquisition, v1.1 gating, Ground selection and execution; only gain weighting
differs. Their later closed-loop states can differ. Saved same-state dual
rankings check the shared-input identity, not independent performance samples.

The public initial observation pose is map (-1.4,0,1.2), yaw 0; the observer
depth gate is 4 m. Environment-gated methods receive three total completed
5-s simulation-time windows, including the initial window and stationary
rescans. Generic/Ours stop on no valid viewpoint, no positive own gain, best
own score at most zero, or budget exhaustion, not first confirmation. Fixed
takes three stationary windows. RM4D-only takes no MID360 decision windows
and uses original exact top-one ranking; its real aerial bootstrap is counted.

At stopping, environment-gated methods select the maximum-relevance confirmed
original exact candidate in stable first-tie order, with no fallback candidate.
Confirmation requires real ground support and both representative/exact
operational clearance. BUNKER navigation, D435 refinement, collision-aware
pregrasp execution, descend, grasp and lift then use the common frozen pipeline.
Operational confirmation is not full navigation or execution feasibility.

## Experimental units and outcomes

The prespecified sample is 120 independent scene seeds, 40 per Easy/Moderate/Hard
template. Four core methods give 480 valid slots. Two additional ablations,
without occlusion reasoning and without flight cost, run on all 40 Hard scenes,
giving 560 slots in total. Generic already supplies the no-task-weighting
ablation. Exact seeds and balanced core-method positions were serialized before
any formal method outcome. Target XY/yaw and initial BUNKER XY/yaw use the
unchanged narrow uniform perturbations; tier obstacle templates are fixed.

Each scene first undergoes a separate method-independent setup check using
RGB-D acceptance, public pose/hover validity and at least 100 distinct real
ground-observed cells. No score, candidate count or retrieval result selects
or replaces seeds. One fresh simulator runs at a time. Demonstrated platform
invalidity can be repeated in a new retained activation; valid budget,
perception, planning and execution failures remain failures.

The sole primary endpoint is physical E2E retrieval success: executed grasp and
retention plus brick and TCP lift of at least 0.10 m, measured by the unchanged
independent checker. A LIFT status alone is insufficient. Secondary endpoints
include environmental confirmation, its first window/time and `D_exec`, which
requires actual arrival, fresh refinement and executed collision-aware refined
pregrasp. RM4D-only environmental confirmation is not applicable. Stage failures,
view windows/moves, UAV distance/time and Ground execution are reported for all
valid trials, with explicit available denominators. All robot-efficiency times
use simulation time. Missing online paths remain missing; causal native-bag
reconstructions are separately labeled secondary measurements.

## Analysis and limits of inference

The scene is the paired unit for the sole primary Ours-versus-Generic comparison.
The final analysis reports all paired binary categories and the two-sided exact
McNemar test at 0.05. With fixed tier counts, its exact null is directional
symmetry within every tier, not merely cancellation to a zero average effect.
Equal-tier risk difference is accompanied by both an explicitly approximate
paired stratified normal interval and a conservative simultaneous exact-binomial
bound; sparse/zero-variance warnings are retained. Neither bound is selectively
chosen by significance. The two Hard ablation tests form a secondary Holm family.
First-activation INVALID-as-failure sensitivity is separate. No interim efficacy
testing, outcome-driven sample-size adjustment or pilot pooling is permitted.

Observed task durations include short failed attempts; they are not automatically
time-to-success or evidence of improved efficiency. Fixed scene templates,
narrow pose perturbations, imperfect scan opportunity prediction, accumulated
static evidence and the common unmodified execution backend limit generalization.
In particular, this study does not establish performance across arbitrary
obstacle layouts, dynamic/nonhorizontal environments or real robots. The
prospective protocol records the retained common navigation configuration
limitation and the distinction between physical failures and invalid measurements.
