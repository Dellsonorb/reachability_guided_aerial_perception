# A6 pilot protocol — 2026-09-08

Operational rules below are fixed before any pilot attempt. A1–A5 remain at
`060a4e2`; SIM public correction and the shared task-domain RM4D asset remain
unchanged. This authorizes **14 scheduled pilot method slots**, not a formal
matrix. The subsequent overnight authorization permits rerunning independently
demonstrated INVALID platform activations; every activation remains recorded.

**Frozen initialization:** common map sensing destination `(-1.4,0,1.2), yaw=0`;
common public UAV launch `(-.5,0,.15), yaw=0`. All three original seeds passed
sensor-only qualification with both poses before any pilot method executed.
See `A6_INITIALIZATION.md` for the depth/FOV/ground-return checks and the common
launch-origin collision repair; neither used RM4D/discovery/scores/outcomes.
The earlier `(-2.5,0,1.5)` range defect is historical (`A6_PREFLIGHT_GEOMETRY.md`).
No further initialization changes after pilot execution begins.

## 1. Exact three-window semantics

- One window is one successfully acquired, stable-hover cloud spanning **at
  least 5.0 simulation seconds** from its first to last real MID360 packet stamp.
  It is one A2 observation, regardless of packet/point count. Each cell receives
  at most one occupied or free vote per window, with occupied priority.
- The first window, a translated/yaw-changing view's window, and a stay/rescan
  window each consume **one** of three total windows. There are at most two
  decisions that can initiate another sensing action. This is not three extra
  viewpoints after the initial one.
- The frozen capture code keeps its 20 wall-second capture guard, stable-pose
  and stamped-TF rules. A disrupted partial window is discarded, not voted.
  Its within-call recovery cannot extend that guard. A failed capture call
  ends the attempt; no A6 acquisition retry. Capture calls, discarded windows
  and completed/voted windows are recorded separately.
- Every new completed window has strictly newer real stamps. No cloud replay
  as new evidence; bounded history is rebuilt into an empty A2 mapper exactly
  once per decision, as in A5.
- Fixed-view obtains three independent windows at its common initial sensing
  location, with no commanded sensing translation/yaw update. Inherited hover
  drift is allowed and measured; stay/hover commands do not fake zero travel.
- RM4D-only uses zero MID360 windows. Its real initial aerial RGB-D view and
  all bootstrap/query hovering still count towards total aerial resources.

## 2. Common stopping and selection

Ours and Generic evaluate the **same stopping procedure using their own gain**:

1. No geometrically valid candidate: `NO_VALID_CANDIDATE`.
2. No valid candidate with positive selected gain: `NO_PREDICTED_TASK_GAIN`
   for Ours or `NO_PREDICTED_GENERIC_GAIN` for Generic.
3. Best selected-method score ≤ 0: `NONPOSITIVE_SCORE`.
4. Three completed observation windows: `VIEW_BUDGET_REACHED`.
5. Otherwise execute the best selected-method candidate and acquire once more.

This precedence preserves frozen A5. Generic must not inherit a task-only
no-gain stop when its own gain is positive. There is **no first-confirmed
candidate early stop**. Stay/rescan with positive gain and zero flight cost is
legitimate and may consume all windows. Fixed-view stops after its third
window; RM4D-only after its one RM4D query. All methods terminate on fatal
interface/action/acquisition failure. Return/landing are inherited and included
in total costs. There is one Ground handoff and no post-failure top-K fallback,
RM4D requery or second aerial mission.

Generic and Ours share the same current-pose-relative ordered candidate
generator, near-position/yaw filter, bounds, actual sensor mount, FOV/range,
1 m occupied-prism prediction, flight penalty and tie-breaks, A2 updater,
capture windows and Ground backend. Only the observation gain differs:

\[
G_O=(1-e^{-1/2})\sum_x\mathcal V(v,x)u(x)M_{operational}(x),\quad
G_G=(1-e^{-1/2})\sum_x\mathcal V(v,x)u(x).
\]

The exact original A4 masks and cell-sum units apply. No normalization or
retuning. Common score is `gain - .25 * (distance_3d + .25*abs(wrapped_yaw))`.
Equal scores use lower flight cost, then original candidate index. Closed-loop
current poses may differ, but identical inputs must give identical candidate
order, visibility and cost. Both scores are retained for each visited snapshot.

The fixed numeric settings are serialized under `shared_a5_settings` in the
pilot JSON: .10 m map grid, 4×4 m area centered on the initial live grasp;
XY offsets {-2,0,2}, fixed measured altitude, eight yaws/current-first; bounds
[-4,4]×[-3,3]×[.5,3] m; .05 m flight-facade tolerance, 3 m Ground travel guard
and 120 s navigation guard. Other capture/TF/core guards are also listed there.
Original A2 tau=2, ground tolerance=.02 m, occupied height minimum=.05 m and
two-free-vote rule, A3 footprint, A4 sensor mount/FOV and inherited manipulation
settings remain frozen. `no_cost` alone removes the scoring flight penalty;
`no_occlusion` alone uses range/FOV masks without prism intersection rejection.

### Exact Ground candidate

Fixed/Generic/Ours and the two ablations use frozen A5:

1. Traverse RM4D `evaluated_candidates` in original order; retain valid positive
   A1 relevance candidates in the grid. Per A1 cell retain the highest relevance
   exact original candidate; equal relevance retains the first evaluation.
2. Order this catalog by original evaluation index. This is not all valid
   candidates and not the discretized cell-center pose.
3. A candidate is confirmed only when its exact closed-rectangle BUNKER
   footprint (half-length .52 m, half-width .39 m; inclusive cell contact)
   has nonempty, unclipped grid coverage, **every covered cell is
   A2 FREE**, and the corresponding A3 representative exists and is not
   A2-occupied-blocked. An occupied/unknown exact-footprint cell disqualifies
   handoff; positive unknown_score on a FREE cell does not disqualify it.
4. At stopping, select maximum relevance among confirmed candidates, preserving
   first catalog order on ties. No confirmed candidate means method failure.
5. Send the original exact map x/y/yaw to BUNKER. Do not send the representative
   cell center or imply that it was candidate-IK-validated.

RM4D-only instead selects `result.candidates[0]` in original frozen RM4D order,
retaining its exact map pose and original score; it has no A2 gate or catalog
fallback. Its travel/final_score never enters A1 relevance or NBV gain. Empty
returned candidates mean method failure, not a replacement query.

All methods use the shared task-domain asset and original collision-aware
Ground/refine/arm/AG95/lift backend. Existing arm-branch attempts are not extra
Ground-candidate attempts. `D_exec=1` only after actual Ground arrival, fresh
D435 refine and successful collision-aware refined continuation to verified
pregrasp; it is separate from physical retrieval success.

## 3. Scheduled slots, invalidity and failure

The fixed schedule in `configs/a6_pilot.json` has 12 baseline slots and two Hard
ablation slots. w/o task weighting **is Generic**, not a fifteenth method slot. Easy
Fixed-view retains its first position. For the remaining
positions, `Random(20260908).shuffle([generic,ours,rm4d_only])` defines the first
block; subsequent tiers left-rotate that order by one/two positions. This is
restricted randomization with a deliberate first Fixed-view position,
not unrestricted random ordering. The two secondary ablations run last in the
serialized order. An activation begins when its fresh SIM launch starts and is
recorded even if platform startup later fails. An unlaunched slot is `NOT_RUN`,
never a failure or success. The latest overnight authorization permits a fresh
activation of the **same slot/seed/method** only after independently establishing
INVALID_TRIAL. Preserve its invalid record, reason, and rerun link; do not replace
or rerun valid method failures. No extra method slots or development E2E trials.
Method-independent sensor setup checks are separately labeled and do not execute
RM4D, NBV, Ground candidate selection or retrieval; they are not pilot trials.

Platform readiness precedes task start: advancing `/clock`, Ground runtime
ready, live UAV state, fresh public map TF for UAV/Ground and reachable common
flight/navigation/MoveIt interfaces. Task time begins immediately before the
inherited preflight/air phase after that shared readiness check. A common
1200 wall-second task guard bounds a stuck run; original stage guards remain.
Wall guards do not become robot-efficiency measurements.

- `VALID_TRIAL, S=0`: any normal task/algorithm failure or timeout after start,
  including initial RGB-D failure, RM4D no result, capture/TF/settle failure,
  policy no handoff, navigation, refine, planning, grasp, retention or lift.
  Missing optional evidence-supported discovery for RM4D-only is N/A, not zero.
- `INVALID_TRIAL`: independently demonstrated method-unrelated startup failure,
  external host/simulator failure, or loss of the **primary physical outcome**
  measurement. Record the concrete evidence/reason; "runtime error" alone is
  insufficient. If method independence cannot be established, count `S=0`.
- An already-established physical failure remains failure if later cleanup
  fails. Missing resource TF samples invalidate the affected efficiency metric,
  not an otherwise observable primary outcome. Do not exclude an unsuccessful
  trial simply because some resource metrics are missing.
- Report the earliest terminal failure stage and reason; downstream stages not
  entered are `NOT_REACHED`. Retain costs incurred before termination. Cleanup
  resources are separate; a failed run is not silently made successful by
  eventual landing or checker teardown.
- Success requires real BUNKER arrival, fresh D435 refine, actual collision-aware
  controller execution, AG95 confirmation/retention, and physical brick and TCP
  lifts each ≥.10 m under the existing checker. Gazebo truth is outcome-only.

## 4. Scene-seed admission — method-blind and before outcomes

The three seeds are prelisted, not searched for Ours wins. Target and parking
perturbations are generated once by Python `random.Random(scene_seed)` in this
order: target dx, dy in ±.15 m; target yaw in ±30°; BUNKER dx, dy in ±.10 m;
BUNKER yaw in π±5°. Serialized values, not regenerating from a runtime RNG,
define each scene. Three pilot seeds are reserved and cannot become independent
formal samples. Physics/scan/planner randomness not publicly seedable is labeled
uncontrolled rather than falsely made deterministic.

The replacement initial pose may use **only** aerial RGB-D depth/FOV, raw MID360
initial ground-observation usability and basic UAV/SIM hover geometry. It must
not use A1 relevance, candidate discovery, catalog/footprint completion, NBV
scores, retrieval outcomes or method wins. Prefer one fixed map pose; any fallback
rule must be deterministic from setup metadata only, never runtime GT policy
input. On each original seed, test the unchanged live RGB-D observer and frozen
hover/capture geometry; raw endpoint ground coverage in the common 4×4 m setup
ROI is a sensor diagnostic, not A2 voting or confirmed-candidate selection.
After all three pass, freeze immediately before any pilot method executes.

Original scene geometry remains fixed: finite in-domain poses, horizontal ground,
unchanged brick/dynamics and 1 m ground-supported boxes. Ground access and tier
qualification are descriptive checks, not grounds for optimizing the initial
pose or replacing a seed. Geometric opportunity is not a success guarantee. A1
UNASSESSED is never called unreachable.

Difficulty qualification uses the complete high-support patch from design §5,
true scene-box geometry (not A4's assumed belief geometry), and the initial
sensor FOV. Easy has ≤.10 blocked eligible task fraction; Moderate .25–.50;
Hard .60–.85, at least 3 m² candidate-relative irrelevant initial unobserved
area, and irrelevant/task-unobserved ratio ≥2. Hard also retains Ground access
and a geometrically distinct side view. No map-size change by method. Actual
first-window N=0 areas are manipulation checks, not state-UNKNOWN counts or
post-outcome exclusion rules.

Do not resample scenes or modify the frozen pose after method execution starts.
A noisy admitted live query with no support or insufficient full-footprint FREE
evidence within budget remains a method failure, not retroactive scene rejection.
Report discrepancies against difficulty targets without changing scenes. Ordinary
platform/orchestration problems may be repaired and proven INVALID activations
rerun. Stop only for the user's specified substantive conflicts: required frozen
method/protocol changes, unfair primary comparison, unfixable method-dependent
scene bias, core-claim changes, or irreconcilable physical/frame semantics.

## 5. Time, distance and denominators

**Simulation time is primary for all robot-efficiency durations.** Store raw
ROS `/clock`-based stamps. Monotonic wall durations are diagnostics for compute,
startup and guards, not substitutes for robot time. Paused simulation does not
accumulate robot time. A clock reset within a task makes durations unavailable;
do not subtract across it or replace with wall time.

- `T_task_sim`: common task start to physical LIFT or first terminal failure.
- `T_active_sim`: first capture-call start to active stopping/failure decision;
  includes settling, all captures, compute and inter-view flight. RM4D-only=0.
- `T_first_env_sim`: same active origin to first decision with any confirmed
  exact candidate. Missing if never reached; this is not an actual early stop.
- `T_exec_ready_sim`: task start to successful refined pregrasp; missing if not
  reached. Ground nav/refine/pregrasp/descend/close/hold/lift stage intervals
  use simulation stamps and retain their success/failure/not-reached states.
- Sample public map UAV and Ground TF at 10 Hz **simulation time**, using the
  same freshness rule. Accumulate UAV 3D and Ground XY path lengths between
  consecutive valid samples, with no smoothing; yaw is separate. Record missing
  samples/gaps. A gap over .3 simulation seconds or unavailable TF marks the
  full affected distance incomplete; report observed-segment distance separately,
  never a zero-filled complete path. Never bridge clock resets.
- UAV total path: task start to actual landed, or failure if not yet landed;
  later recovery landing is separately recorded. Active path uses active bounds.
  Bootstrap visit counts once; first lidar window shares that visit. Each later
  sensing action including rescan adds one visit; translated NBV commands are
  counted separately from return/landing and initial outbound flight.

Report every activated attempt, invalid counts, physical success and resources
jointly. No first-discovery=0 imputation. No efficiency claim from faster
failures. Success-only statistics are explicitly conditional; paired resource
differences use common-success pairs and state that subset's size.

## 6. Sole primary paired comparison and stopping this work

Primary endpoint is binary physical E2E success. **Ours vs Generic is the only
primary contrast.** Pair by scene seed. Report both success, neither success,
Ours-only success `b`, and Generic-only success `c`; paired risk difference is
`(b-c)/number_of_complete_valid_pairs`. Incomplete pairs are listed, not treated
as independent unpaired observations. Also provide sensitivity results treating
activated INVALID outcomes as failures, explicitly excluding unrun slots.

For future approved formal data, preplanned binary inference is the two-sided
exact McNemar/binomial test on `b+c` discordant pairs with p=.5; no discordants
means no evidence of difference, not equivalence. Equal-weight tier reporting,
secondary baselines and ablations are descriptive here. This pilot does not
support significance, power or paper-superiority claims.

After the 14 scheduled slots (plus separately retained proven INVALID reruns), deliver
all outcomes, failure/invalid reasons, simulation-time resources and admission
diagnostics. Stop for review. No frozen-parameter tuning, further method slots,
formal matrix, or new experiment/evidence framework is authorized.
