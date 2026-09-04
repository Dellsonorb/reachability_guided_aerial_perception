# A1 Validated Manipulation Interest Field Design

## Objective and scope

Build a minimal, ROS-independent research prototype that maps one Brick grasp
TCP pose in `map` through the frozen RM4D AUBO baseline into a two-dimensional
Validated Manipulation Interest Field (VMIF). The field describes the value of
placing the BUNKER `base_link` reference point at a ground-plane location
`q = (x, y)` for the requested manipulation.

The prototype calls the frozen `BasePlacementAPI` without modifying RM4D. It
uses all returned `evaluated_candidates`, not only the ranked `candidates`
top-K. It does not implement UAV NBV, active mapping, information gain,
learning, uncertainty-aware RM4D, benchmarking, or execution/safety evidence
infrastructure.

## Frozen RM4D boundary

The external baseline is `rm4d-aubo-baseline-v1` at
`e9d431299053f38a4a4319aed3dfeccc261b9fac`. Its integration call is:

```python
result = rm4d_api.plan(grasp_request, top_k=1)
```

The VMIF builder intentionally ignores `result["candidates"]` and consumes
`result["evaluated_candidates"]`. `top_k=1` only minimizes the unused ranked
output; it does not change the baseline's validation budget.

The request contains only:

```json
{
  "frame_id": "map",
  "grasp_id": "brick-001",
  "position_xyz": [0.0, 0.0, 0.0],
  "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]
}
```

`current_bunker_pose` is omitted so travel does not bias which candidates are
validated. `obstacle_map` is omitted so environmental belief is not folded
into the manipulation field before the later MID360 fusion stage.

The frozen API's `map` mode is a numeric identity alias of its baseline
`world` coordinates. This prototype therefore accepts only an already-resolved
`map` grasp TCP. Estimating or applying `T_map_world` is outside A1.

## Why assessment coverage is explicit

The RM4D inverse query can produce tens of thousands of positive SE(2)
candidates, while the frozen planner validates at most 256. A cell absent from
`evaluated_candidates` is consequently not evidence of unreachability.

The field has two top-level statuses:

- `PARTIALLY_ASSESSED`: the inverse query produced candidates and the field
  contains only the subset admitted to the frozen validation budget.
- `NO_INVERSE_REACHABLE`: `summary.inverse_reachable == 0`; the frozen inverse
  query found no candidate for any sampled BUNKER yaw.

For a partially assessed field, cell states are:

- `UNASSESSED`: no evaluated candidate was assigned to the cell;
- `INFEASIBLE`: at least one candidate was evaluated in the cell, but none
  passed the manipulation-validity gate;
- `LOW`: at least one candidate passed and the cell relevance is below the
  display threshold;
- `HIGH`: at least one candidate passed and the cell relevance meets the
  display threshold.

For `NO_INVERSE_REACHABLE`, relevance remains undefined (`NaN`) and the
field-level status is authoritative. The renderer displays the field-level
condition explicitly instead of making it look like ordinary unassessed data.

Coverage metadata records:

- raw inverse-reachable count;
- deduplicated candidate count;
- validation budget;
- evaluated and valid candidate counts;
- evaluated grid-cell count and total grid-cell count;
- evaluated-candidate fraction of the deduplicated set;
- assessed-cell fraction of the output grid;
- whether validation was truncated.

No coverage number is interpreted as statistical confidence.

## Relevance definition

Let evaluated candidate `i` have BUNKER ground pose
`c_i = (x_i, y_i, psi_i)` and minimum joint-limit margin `m_i` in radians.
Define the validity gate:

```text
g_i = rm4d_reachable
      and ik_valid
      and collision_free
      and not footprint_collision
      and joint_margin_rad >= 0.01
      and valid
```

The continuous candidate relevance is margin-dominant and contains no IK/FK
residual term:

```text
v_i = 0                                      if not g_i
v_i = clip(joint_margin_rad / 0.5, 0, 1)     if g_i
```

The `0.01 rad` acceptance threshold and `0.5 rad` saturation scale match the
frozen planner. The saturation is a transparent normalization convention, not
a probability calibration or manipulability metric.

Let `E(q)` be the evaluated candidates whose `(x_i, y_i)` fall into grid cell
`q`. BUNKER yaw is controllable, so the two-dimensional field is the optimistic
yaw envelope:

```text
R(q) = max(v_i for i in E(q))
```

If `E(q)` is empty, `R(q) = NaN` and the cell is `UNASSESSED`. If `E(q)` is
nonempty but every `g_i` is false, `R(q) = 0` and the cell is `INFEASIBLE`.
Otherwise the best candidate's yaw and diagnostics are retained. The display
split is `HIGH` for `R(q) >= 0.8` and `LOW` for `0 < R(q) < 0.8`. This threshold
does not alter the continuous field.

FK position residual, FK orientation residual, IK attempt count, and rejection
reason remain candidate/cell diagnostics. They do not contribute to `v_i`
after the acceptance gate has passed.

There is no smoothing, interpolation, kernel spreading, or forced continuity.

## Grid and data model

The default grid is a `3.0 m x 3.0 m` square centered on the grasp TCP XY
position, with `0.10 m` cells. The caller may provide explicit origin, width,
height, and resolution so a future MID360 belief grid can be matched exactly.
Arrays use ROS OccupancyGrid row-major convention: shape `(height, width)`,
with increasing column along map X and increasing row along map Y.

Public ROS-independent types:

```python
GraspTCP
GridSpec
FieldConfig
AssessmentCoverage
ManipulationInterestField
OccupancyGridPayload
```

`ManipulationInterestField` contains:

```text
grasp_id
frame_id = "map"
status
origin_x, origin_y, resolution, width_cells, height_cells
relevance[H, W]
cell_state[H, W]
evaluated_count[H, W]
feasible_count[H, W]
best_yaw[H, W]
best_joint_margin_rad[H, W]
best_fk_position_residual_m[H, W]
best_fk_orientation_residual_rad[H, W]
coverage
```

Pure and live entry points are separated:

```python
build_field_from_result(grasp, result, grid, config) -> ManipulationInterestField
build_field(grasp, rm4d_api, grid, config) -> ManipulationInterestField
```

The pure entry point enables fast unit tests and reuse of existing offline
results. The live entry point performs exactly one frozen RM4D call.

## Offline output and RViz seam

Each offline run writes only research-useful products:

- `field.npz`: field arrays;
- `summary.json`: geometry, scoring semantics, status, and coverage;
- `candidate_diagnostics.csv`: the 256-or-fewer evaluated candidate records;
- `field.png`: relevance map, categorical state map, and evaluated candidate
  scatter with the grasp location.

The RViz seam converts a field into an `OccupancyGridPayload` without importing
ROS:

- `-1`: unassessed;
- `0`: assessed but infeasible;
- `1..100`: feasible relevance;
- payload metadata carries the field-level status, including
  `NO_INVERSE_REACHABLE`.

A later thin ROS wrapper can publish this as `nav_msgs/OccupancyGrid` plus a
status topic. A ROS node is not part of A1.

## Offline validation

Three map-frame scenarios use the frozen baseline:

1. `nominal`: a known reachable grasp; verify assessed cells, feasible cells,
   and at least one high-value cell.
2. `boundary`: a known workspace-boundary grasp; verify a mixture of assessed
   manipulation outcomes and that every raster value is exactly traceable to
   an evaluated RM4D candidate in the same cell.
3. `no_inverse`: a grasp above the RM4D z range; verify
   `NO_INVERSE_REACHABLE`, zero evaluated candidates, and no cells incorrectly
   reported as ordinary assessed or feasible cells.

The acceptance criterion is spatial and semantic consistency with the
evaluated RM4D/AUBO candidates. Nominal continuity and smooth appearance are
not requirements.

Fast unit tests cover input validation, margin-only scoring, residual
independence, validity gates, yaw maximization, unassessed preservation,
coverage calculations, field-level no-inverse status, serialization, and the
RViz payload mapping. The real three-scenario script is a small offline
research check, not a benchmark harness.

## Future MID360 fusion boundary

The field and environmental belief share explicit map-grid geometry. Given
aligned free/occupied/unknown probabilities, a later stage can evaluate a
BUNKER pose using cells under its oriented padded footprint, while retaining
the manipulation value separately.

One task-relevant uncertainty definition is:

```text
U_task(x) = P_unknown(x) * max R(q)
            over validated (q, yaw) whose BUNKER footprint contains x
```

This makes an unknown environmental cell important only when it can affect a
high-value validated manipulation placement. Occupied-space rejection,
free-space support, footprint projection, information gain, and viewpoint
selection remain outside A1.
