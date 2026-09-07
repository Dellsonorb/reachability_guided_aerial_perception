# A5 task-domain RM4D map: geometry and collision-world decision

Status: **approved for implementation**. User authorized a separately owned
integration collision floor at the derived -0.472 m, with workspace coverage
[-0.25,1.3] m. Frozen assets, algorithms, robot definitions and the baseline
validator/collision world remain unchanged.

## What the z index means

Frozen source `e9d431299053f38a4a4319aed3dfeccc261b9fac`:

- `rm4d/robots/aubo_i5.py`: endpoint is physical `wrist_3_link`, not TCP.
- `rm4d/robots/base.py`: standalone AUBO base is at `[0,0,0.01]`.
- `rm4d/construction.py`: uniform joint-space sampling, collision-free rejection,
  FK to flange, then `get_indices_for_ee_pose` and `mark_reachable`.
- `rm4d/reachability_map.py`: `get_p_z(T)` returns `T[2,3]`;
  `get_z_index` computes `int((p_z-z_min)/0.05)`.
- `rm4d/base_placement.py`: TCP -> flange, then query height is adjusted by
  standalone robot-base height minus BUNKER-to-AUBO mounting height.

Thus the asset coordinate is **flange height in the standalone AUBO map world**,
whose robot-base height is 0.01 m. It is not map TCP z or BUNKER z.
For the current horizontal mount/reference convention:

```
z_asset_flange = z_map_flange - h_BUNKER - h_mount + 0.01
z_map_flange = z_map_TCP + 0.20             # current vertical top-down grasp
h_BUNKER = 0.36; h_mount = 0.122
z_asset_flange = z_map_TCP - 0.272
```

The production frame bridge derives the heights from public TF/frozen mounting
configuration. The existing 1e-6 rad query-only regularization changes the above
vertical flange offset by approximately 1e-13 m, not a task-domain-scale amount.

## Smallest task-derived extension

SIM `src/demos/air_ground_pick_demo/config/demo.yaml` and its
`src/air_ground_pick_demo/grasp.py` define:

- brick vertical height 0.115 m, nominal ground-supported center z=0.0575 m;
- existing accepted center-height tolerance +/-0.030 m;
- finger-pad lower-edge offset 0.0156 m and contact overlap 0.020 m;
- pregrasp and lift offsets 0.15 m.

The +/-0.030 m check is enforced by the existing near-field target validation,
not by the generic grasp generator or every aerial observation. This defines
the current near-field-accepted ground-task band, not a guaranteed bound on all
raw aerial estimates or a new uncertainty/terrain assumption. The actual aerial
TCPs observed so far fall inside this band. An out-of-band aerial estimate must
not silently expand the runtime map's claimed task domain.

| Quantity | Range (m) |
|---|---:|
| Accepted center height | [0.0275, 0.0875] |
| Exact grasp TCP | [0.0494, 0.1094] |
| Asset flange at grasp | [-0.2226, -0.1626] |
| Asset flange at pregrasp/lift | [-0.0726, -0.0126] |

At unchanged 0.05 m RM4D resolution, the minimum aligned lower boundary is
**-0.25 m**. Preserving the original upper bound gives a proposed asset range
`[-0.25,1.3]`: five additional z slices, with the original 26 slices retained.
XY remains `[-1.15,1.15]`, theta 36 bins. A1/A2 remain 0.10 m map grids.
This envelope only claims support for the current static horizontal-ground,
side-up brick retrieval task, not arbitrary real-world targets or terrain.

## Why z coverage alone is insufficient

`Simulator._load_plane_and_gravity()` loads `plane.urdf` at z=0. Both
`RobotBase.get_random_joint_config(prevent_collisions=True)` and frozen
`AuboIkValidator.validate()` reject robot/plane collision, in addition to
self-collision. Collision threshold remains the frozen -0.001 m.
The installed `pybullet_data/plane.urdf` defines a 200x200x10 m collision box
centered at z=-5: its solid collision region spans [-10,0], rather than just a
zero-thickness surface. Independent source/geometry review confirmed that the
required grasp band places the wrist collision mesh inside this region.

In one bounded, diagnostic-only read of the unchanged simulator:

- 1,024 uniform joint proposals (seed 42) included 376 flange poses below
  z=-0.001 m; all 376 collided with the frozen plane.
- 96 of those had no self-collision. In the required flange band, 19 samples
  had no self-collision, but all 19 collided with the plane.
- 256 samples accepted by the standard collision-free sampler had minimum
  flange z=0.0168072 m.
- A representative task target at flange z=-0.190471318 m was solved by the
  unchanged M-R4 validator with FK position residual `7.6164e-16 m`, orientation
  residual `9.2805e-16 rad`, and joint margin `1.11561 rad`; it was nevertheless
  rejected with `collision_free=false`, `valid=false` after 16 starts.

These are focused diagnostics, not a sampling coverage proof or benchmark.
They show a second, independent obstacle beyond the map-array bounds. Adding
negative bins alone does not make the required task targets valid. Populating
bins without the plane gate would still fail frozen M-R4 validation and would
silently change collision semantics, so neither is done.

## Additional runtime collision-world definition required

The physical SIM ground is map z=0. In the standalone asset coordinates its
height follows from exactly the same frame chain:

```
z_asset_ground = 0 - 0.36 - 0.122 + 0.01 = -0.472 m
```

Moving **only a separately owned integration simulator's floor** to that value
would keep the actual robot, mount, FK, sampling proposal method, resolution,
IK thresholds and ranking unchanged. The frozen repository/map/tag and M-R1–M-R4
would remain untouched. However, the integration asset's robot-to-floor relation
would differ from the frozen baseline's collision-world physical definition;
this is more than changing workspace coverage. Sampling AND runtime validation
must agree on that explicit task floor. Self-collision and floor collision
must remain enabled; actual SIM collision-aware planning remains necessary.

The user authorized **task-floor calibration in the new runtime asset and its
validator only**, not modifying the baseline or turning off collision checks.
Generation and overlap/RM4D/SIM validation are implementation work; authorization
alone does not establish their success.

## Validation after that decision

1. Keep the original [0,1.3] baseline occupancy as an explicit unchanged overlap;
   check loaded slice and forward/inverse query consistency. New negative slices
   must have actual sampled collision-free witnesses under the task floor.
2. Use current negative task target plus a few positive/negative target poses
   with unchanged planner/IK/FK thresholds, recording all successes and failures.
3. Cross-check local geometry and actual SIM collision-aware IK/planning through
   the existing bridge before returning to the natural A5 E2E.
4. Sampling-window timeout remains separate. No A1–A4 edits, benchmark, new
   large dependency, or safety/evidence framework is proposed.
