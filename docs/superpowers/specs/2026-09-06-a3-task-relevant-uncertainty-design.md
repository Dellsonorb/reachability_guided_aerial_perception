# A3 — Task-Relevant Uncertainty (approved)

User approved this design and its three explicit semantic qualifications:
blocked means **A2-occupied-blocked**, not navigation infeasible; a cell-center
pose is a discrete representative, not a newly IK-validated pose; FREE does not
force unknown_score to zero. A2 PR #2 was merged as `8d53cae` before creating
`feature/a3-task-relevant-uncertainty`. A1 and A2 remain unchanged.

## Inputs and geometry

Use the existing ManipulationInterestField and EnvironmentBeliefGrid as inputs
to a pure `build_task_uncertainty(a1_field, a2_belief, footprint)` function in a
new `task_relevant_uncertainty` package. Require map frame, 0.10 m resolution,
identical origin and shape (numeric equality within 1e-9 m for metadata only).
No resampling or world/map alias. A2 grid metadata is the common output grid.

Project only A1 HIGH/LOW cells with finite positive relevance and finite
best_yaw. Keep A1 INFEASIBLE and UNASSESSED separate in an unchanged copied
assessment-state array; neither produces a footprint. Preserve A1 field status
and assessment coverage, including NO_INVERSE_REACHABLE.

For base cell (r,c), use q=(x0+(c+.5)d,y0+(r+.5)d), yaw=best_yaw[r,c]. This is
the cell representative, not the exact winning candidate XY or an IK claim.
Only this stored yaw is considered; no alternate-yaw search or new validation.

FootprintSpec is a centered base_link rectangle with half-length 0.52 m and
half-width 0.39 m. These already include the frozen RM4D 0.02 m padding around
its nominal 1.00 x 0.74 m BUNKER footprint. Source: RM4D_AUBO/configs/
mr4_offline_base_placement.json, consistent with SIM's navigation footprint.
No new inflation/padding. Positive finite alternate half-dimensions may be
passed explicitly for geometry tests or a future caller, not inferred from ROS.

Rotate its corners by yaw, then translate by q. An environment cell is covered
when its closed XY rectangle intersects the transformed footprint, including
edge/corner contact. Use the separating-axis test on the two grid axes and two
footprint axes, with a 1e-12 m arithmetic tolerance for contact. This is cell
overlap rasterization, not smoothing, convolution, or a center-only inclusion
test. Clip only the output cell list to the map; flag footprint_clipped when
any part extends beyond the grid. Outside is unobserved, never FREE.

## Projection and combination

For every supporting representative pose q let F_q be its covered in-grid cell
set. b(q) is true iff any cell in F_q has A2 state OCCUPIED. UNKNOWN does not
block. Out-of-grid coverage alone does not block, but prevents full observed
ground-support status. A2 occupied includes its known conservative overhead
projection; no new height/clearance interpretation is added.

M_nominal(x) = max R(q) over supporting q with x in F_q.
M_operational(x) = max R(q) over supporting q with x in F_q and b(q)=false.
U_task(x) = unknown_score(x) * M_operational(x).

No A1 mutation or relevance recomputation. A blocked high-relevance pose loses
its support over its entire footprint; overlapping unblocked lower-relevance
poses may still support shared cells. Not blocked is not a clearance or path
feasibility certificate. No task weighting enters A2.

Environment-cell support state:
- NO_VALIDATED_SUPPORT: no nominal contributor; all three field values NaN.
  This does not mean task-irrelevant or globally manipulation-infeasible.
- BLOCKED_ONLY: at least one nominal contributor, none operational;
  M_nominal is preserved, M_operational=U_task=0.
- SUPPORTED: at least one operational contributor; apply the maxima/product.

Per-pose environment state (on the base grid): NOT_PROJECTED,
A2_OCCUPIED_BLOCKED, UNCONFIRMED (some UNKNOWN or clipped),
OBSERVED_GROUND_SUPPORT (all footprint cells FREE, not clipped). Even the last
state inherits A2's limited ground-support meaning, not whole-body clearance.
Original A1 and A2 states are separate output arrays, not collapsed enums.

## Outputs and support sources

TaskRelevantUncertaintyField: common grid, A1 grasp/status/coverage, footprint,
M_nominal, task_relevance_at_environment_cell (=M_operational),
task_relevant_uncertainty, support_state, nominal_support_count,
operational_support_count, best_nominal_source, best_operational_source,
a1_cell_state, environment_state, unknown_score, pose_environment_state,
and a tuple of PoseSupport records. Arrays are detached read-only copies.

Each PoseSupport records flat row-major base-cell id, row/col, q XY, yaw,
relevance, A2-occupied-blocked flag, footprint_clipped, FREE/UNKNOWN/OCCUPIED
covered-cell counts, and the row-major flat ids of covered environment cells.
These small records expose all support relationships, not only the winner.
Best-source arrays use base-cell id; -1 means no contributor. Maxima ties keep
the first row-major base source deterministically.

Save compact field.npz and summary.json (metadata/config/coverage, state counts,
score semantics) plus supports.json (pose records), and headless PNGs displaying
A1 R/footprints, A2 state/score, nominal/operational projections, U_task and
support state. Raw grid rendering, fixed [0,1] score scales, no interpolation.
No ROS or new dependencies; NumPy core, local Matplotlib Agg for outputs.

## Minimal offline verification

Four deterministic scenes share one explicit map grid and use the existing A1
offline builder with synthetic evaluated candidates (NOT a new RM4D/IK run),
and actual A2 updates with synthetic endpoint observations:
1. high relevance / unknown: R=.9, N=0 -> U=.9 throughout its footprint.
2. same high relevance / repeated ground: N=2 then N=8 -> U=.9*exp(-N/2),
   nonzero even when FREE. Geometry/relevance unchanged between snapshots.
3. low relevance / unknown: R=.2, N=0 -> U=.2.
4. occupied blocking with overlap: high pose blocked by an off-center occupied
   cell; all its nominal support remains, its operational support disappears;
   overlapping unblocked low pose survives at .2, uncovered-by-low portions
   become BLOCKED_ONLY. A1/A2 inputs are unchanged.

Focused tests cover yaw 0/pi/2/pi/4, edge and corner contact, cells intersected
without their center lying inside footprint, clipping, grid mismatch, missing
yaw/invalid inputs, no inverse support, INFEASIBLE/UNASSESSED independence,
max rather than sum, stable ties, sources/counts, snapshots and serialization.
Run the existing A1/A2 test suite once with the final A3 tests; do not rerun RM4D.

## Scope boundary

This is a heuristic task-relevant observation-deficit field, not probability,
entropy or expected information gain. Future A4 could read U_task, support
relations, A1 coverage and pose environment status to evaluate observations;
view visibility, sensing model, travel costs and NBV are NOT implemented here.
No UAV flight, SIM/MID360 ROS adapter, RL/world model, benchmark, lifecycle,
provenance or safety/evidence framework. Complete the local prototype and stop.
