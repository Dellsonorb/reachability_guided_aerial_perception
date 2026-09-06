# A2 — MID360 Environment Belief: proposed minimal design

Status: approved for implementation, including the `unknown_score` naming correction.

## Objective and scope

Build an independent, ROS-independent ground-plane environment belief from
MID360 return endpoints and their explicitly supplied map-frame transform.
The module consumes no RM4D output, A1 relevance, grasp, task weight, or travel
cost. Existing A1 implementation and artifacts remain frozen at `1bb1fcf`.

This first version assumes a static scene and a known horizontal ground plane
`z = ground_z_m` in map. Default ground height is 0 m. Ground fitting, slopes,
negative-obstacle classification, moving-object clearing, ROS subscriptions,
SIM integration, A3 fusion, NBV, and benchmarks are outside this prototype.

## Options and selected approach

1. **Return-endpoint evidence (recommended).** Classify observed endpoints
   relative to the ground plane, accumulate one vote per cell per observation,
   and retain unobserved cells. It is simple to explain and does not infer free
   ground from a ray passing overhead. It deliberately sacrifices free-space
   coverage and cannot certify full vertical clearance.
2. **3D voxel ray integration, then projection.** Represents traversed 3D space,
   but requires vertical bounds, voxel storage and a rule for sufficient column
   coverage before declaring ground free. A single traversed voxel is not a
   free ground column. More machinery than this A2 prototype needs.
3. **Height-layer clearance evidence.** A middle option that tracks coverage
   at several heights. It still requires layer definitions and a coverage rule;
   it is not necessary to establish the endpoint baseline.

Use option 1. No 2D ray carving, 3D ray carving, interpolation, dilation, or
hole filling occurs in the belief updater. Ray/box intersections used by the
synthetic scene generator are only for producing physically occluded inputs.

## Geometry and input contract

The public input is a cloud of return endpoints in the LiDAR sensor frame,
with a supplied rigid transform at the observation reference time:

```text
PointCloudObservation
  points_xyz: float[N, 3], meters in frame_id
  frame_id: nonblank sensor-frame name
  stamp_s: finite observation reference time
  T_map_sensor: float[4, 4], maps this sensor frame into map
  valid_return: optional bool[N], defaults to all true
```

`T_map_sensor` is required, including for an identity pose. A caller with UAV
pose and calibrated extrinsics composes

`T_map_sensor = T_map_uav @ T_uav_sensor`.

For every usable endpoint `p_i`, compute `p_i_map = R p_i + t`. Ground-relative
height is `h_i = p_i_map.z - ground_z_m`. The transform must be finite, have a
homogeneous last row and a proper orthonormal rotation. The mapper is bound to
one configured sensor-frame name and rejects an observation naming a different
frame. No implicit world/map alias or TF lookup is performed.

The caller supplies a cloud already deskewed to its reference frame/time, or a
stationary/short synthetic observation for which one pose is appropriate. This
prototype neither estimates pose nor corrects scan motion. A timestamp is
metadata associating the cloud and transform, not a lifecycle mechanism.

Ignore rows masked invalid, nonfinite XYZ, zero-range returns, and returns
outside a configurable usable sensor range (proposed synthetic defaults:
`0.20 < range_m < 40.0`). Range is measured in the sensor frame before
transformation. Missing returns and clipped returns never imply free space.
Report discarded counts with the update so input mistakes can be diagnosed.

The output grid has caller-supplied origin and dimensions:

```text
EnvironmentGridSpec
  frame_id = "map"
  origin_xy: (x0, y0), meters
  resolution_m = 0.10
  width_cells, height_cells: positive integers
```

Cell `(r,c)` is the half-open rectangle
`[x0+c*d, x0+(c+1)*d) x [y0+r*d, y0+(r+1)*d)`; arrays are `[y,x]`.
Out-of-grid endpoints provide no votes to this grid. The grid origin never
moves with the UAV or the latest cloud. An A3 caller supplies identical origin,
resolution and shape to A1 and A2; resolution alone does not establish alignment.
The A2 code does not import the A1 package to obtain geometry.

## Evidence rules

Proposed configurable synthetic defaults:

- Ground tolerance `epsilon_g = 0.02 m`.
- Minimum obstacle height `h_occ = 0.05 m`, strictly above `epsilon_g`.
- Free classification requires two supporting observations per cell.
- Occupied classification requires one obstacle-supporting observation.
- Unknown evidence scale `tau = 2` observations.

These are prototype parameters, not calibrated MID360 accuracy or traversal
limits. Objects below `h_occ` are not resolved as obstacles by this model.

For a usable return in a cell:

- `abs(h_i) <= epsilon_g`: ground-support evidence.
- `h_i >= h_occ`: occupied evidence for its XY cell.
- All other heights: ambiguous/out-of-model; no free or occupied evidence.

There is deliberately no upper cutoff that could make a tall obstacle vanish.
An observed roof or overhead surface is conservatively projected as occupied.
The module does not infer that the full volume beneath that surface is solid,
and does not establish underpass clearance. UAV self-returns must be excluded
by the input adapter/valid-return mask when connecting actual sensor data.

For observation `t` and cell `C`, define Boolean votes:

```text
o_t(C) = 1 if any usable obstacle endpoint falls in C, otherwise 0
f_t(C) = 1 if any usable ground endpoint falls in C AND o_t(C) == 0,
         otherwise 0
```

The occupied vote therefore wins within a mixed observation. Ten thousand
points in the same cell still provide only one observation vote. No point
count is treated as ten thousand independent measurements.

Starting from zero arrays:

```text
O_t(C) = O_(t-1)(C) + o_t(C)
F_t(C) = F_(t-1)(C) + f_t(C)
N_t(C) = N_(t-1)(C) + [o_t(C) OR f_t(C)]
```

`occupied_evidence = O`, `free_evidence = F`, and `observation_count = N`.
`N = O + F` under these exclusive per-observation voting rules. An ambiguous
height, an overhead ray without an endpoint in C, or an empty cloud does not
increase any of these arrays.

State uses explicit occupied priority, including across observations:

```text
OCCUPIED (100)  if O >= 1
FREE       (0)  if O == 0 and F >= 2
UNKNOWN   (-1)  otherwise
```

Once observed occupied, a cell cannot be cleared by later free votes in this
static-scene version. Later free votes remain visible in F to expose mixed
evidence. Reordering observations gives the same final evidence and state.

FREE means **repeated ground support has been observed, with no observed
above-ground obstacle in the cell**. It is a sampled ground-surface belief,
not proof of complete cell-volume clearance or a navigation command.
UNKNOWN includes both never observed cells and insufficiently supported ground
cells; their evidence arrays distinguish the two.

## Unknown confidence

Output a deterministic observation-deficit index:

`unknown_score(C) = u_unknown(C) = exp(-N(C) / tau)`, with `tau = 2` by default.

It is 1 before informative observation and decreases as supporting observations
accumulate. It is not a calibrated probability of occupancy, nor a claim that
observations from the same viewpoint are statistically independent. In
particular, it measures lack of evidence, not contradiction between O and F;
the latter remains visible in the two evidence arrays. State and confidence
are separate: one obstacle vote can produce OCCUPIED while confidence is still
limited. Empty/occluded cells retain `unknown_score = 1`.

One call denotes one delivered observation. The caller must not replay the same
scan as a new measurement. No scan-ID registry, evidence lock, timestamp barrier,
decay model or deduplication framework is introduced.

## Public API and module boundary

Place the implementation in a separate `src/environment_belief/` Python
package, using the existing NumPy/Matplotlib dependencies. A1 source and exports
are unchanged.

```text
EnvironmentBeliefMapper(grid, config, sensor_frame)
  update(observation) -> UpdateSummary
  snapshot() -> EnvironmentBeliefGrid

EnvironmentBeliefGrid
  frame_id, origin_xy, resolution_m, width_cells, height_cells
  state[H,W]
  occupied_evidence[H,W], free_evidence[H,W]
  observation_count[H,W], unknown_score[H,W]
  config (ground plane and evidence parameters)

save_belief(belief, output_dir)
render_belief(belief, output_path)
```

`update` modifies one mapper in place after validating observation geometry;
`snapshot` returns independent arrays for downstream use. Minimal checks cover
shape, finite geometry/config values, correct rotation/frame, valid ranges and
grid dimensions. Ordinary invalid return rows are filtered and counted.

Persist compact NPZ arrays, a JSON summary of parameters/counts, and a PNG with
state, unknown confidence and observation count. The offline demonstration also
shows the scene geometry and observed endpoints, including a side view of the
overflight case. Use local Agg canvases and discrete raster rendering. There is
no ROS node, topic publisher or new dependency/framework in this stage.

## Offline validation to implement

Use deterministic synthetic rays intersecting a flat plane and axis-aligned
solid boxes. Return only the nearest physical hit. Provide sensor-frame XYZ
and poses to the updater, not ground-truth labels. The generator is a compact
fixture helper, not a scan-pattern emulator or simulator replacement.

Three scene groups cover the required behavior:

1. **Clear ground:** two observations cover a selected ground patch. One
   observation leaves insufficiently supported cells UNKNOWN; the second
   promotes repeated ground observations to FREE and reduces unknown_score.
   Unobserved cells outside the patch remain UNKNOWN.
2. **Obstacle and airborne overflight:** obstacle hits become OCCUPIED. A
   separate high ray crosses the XY location of an as-yet unobserved low box
   and lands beyond it; the crossed cell remains UNKNOWN. An already observed
   occupied cell stays OCCUPIED despite later ground evidence. A side view
   makes the ray-height distinction explicit.
3. **Occlusion and multiple viewpoints:** first-return occlusion leaves a
   designated behind-box ground patch UNKNOWN. Subsequent viewpoints directly
   observe that patch; sufficient new ground observations make it FREE, while
   the box remains OCCUPIED. Missing/no-hit rays do not clear the hidden patch.

Focused tests additionally exercise a nonzero translation and nontrivial
rotation, explicit UAV/extrinsic transform composition, half-open grid edges,
zero/no-return filtering, empty scans, ambiguous heights, per-scan vote caps,
occupied dominance, observation-order invariance, snapshot isolation and JSON
round trips. Run the existing A1 tests once with the completed A2 suite to verify
coexistence; do not rerun the expensive RM4D scenarios or add a benchmark.

## A3 integration contract, design only

A1 answers manipulation relevance for candidate BUNKER base positions. A2
answers environmental ground-surface evidence. Both expose map-frame grids;
the caller must align origin, resolution and shape explicitly.

A3 can later consume A1 relevance/assessment masks and A2 state/evidence/
unknown_score. A1 UNASSESSED (unvalidated manipulation) remains semantically
distinct from A2 UNKNOWN (insufficient environmental evidence). Environmental
occupancy is defined at a surface cell, whereas a BUNKER pose occupies a
footprint; A3 must make this spatial relation explicit rather than interpreting
a single FREE cell as footprint clearance. No fusion function or task weighting
is implemented in A2.

## Sources inspected

- SIM frame contract:
  `/media/lu/P450_PAPER/SIM/p450_sim_v1/src/platform/sim_platform_bringup/config/p450_mid360_tf_contract.yaml`.
  It specifies `/uav1/livox/lidar`, `prometheus_msgs/LivoxCustomMsg`, and the
  composite `uav1/base_link -> uav1/lidar_link` transform, including rotation.
- SIM plugin `PublishLivoxROSDriverCustomMsg` in
  `src/p450/livox_laser_gazebo_plugins/src/livox_points_plugin.cpp` replaces
  invalid/out-of-range ranges with zero before publishing XYZ. These zero-range
  rows must be filtered before map transformation.
- [Livox MID-360 product documentation](https://www.livoxtech.com/mid-360)
  describes 3D coverage and angular resolution improving with integration time;
  it also flags limited accuracy at the closest 0.1–0.2 m ranges. This motivates
  explicit usable-range filtering and accumulation without point-count voting.
- [Official MID-360 protocol](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md)
  describes point-cloud formats, timestamps and scan modes. A2 consumes an
  adapter-level observation rather than parsing the wire protocol.
