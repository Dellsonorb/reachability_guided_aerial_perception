# A5 Ground-task integration RM4D asset

This is a **runtime integration asset, not the frozen literature baseline**.
The original 10M map and `rm4d-aubo-baseline-v1` are unchanged.

`rmap.npy` uses the unchanged RM4D flange-coordinate reduction and discretizer.
`metadata.json` records two different geometric quantities:

- Workspace flange coverage: **[-0.25, 1.3] m**, resolution 0.05 m.
- Collision plane: **-0.472 m**, derived as `map_ground_z - nominal_BUNKER_height - mount_height + internal_robot_base_z = 0 - 0.36 - 0.122 + 0.01`.

The robot remains at its frozen internal base z=0.01. Only the independently
owned integration simulator's floor is relocated. Sampler and runtime validator
share this calibration; all original self/floor-collision, joint-limit and
IK/FK gates remain active.

The 26 original positive slices (455,974 occupied bins) are copied exactly.
100,000 collision-free uniform-joint samples, seed 42, supplied 14,683 FK
witnesses in [-0.25,0), occupying 12,793 negative bins. This sampling budget is
not 10M, and neither the asset nor its A1 output claims complete capability.
Positive slices are not resampled under the lowered floor; they retain the
original, more restrictive floor's sampled support.

Exact floating-point voxel-boundary queries may choose adjacent slices with
the changed z origin under frozen `int((z-z_min)/resolution)` indexing.
Bit-exact copied occupancy and interior-query agreement do not imply universal
boundary-query equality. No indexing algorithm is changed here.

Generate in a separate output directory:

```bash
PYTHONPATH=src "$RM4D_PYTHON" -B scripts/build_a5_task_map.py \
  --rm4d-root "$RM4D_ROOT" \
  --rm4d-config "$RM4D_ROOT/configs/mr4_offline_base_placement.json" \
  --rm4d-map "$RM4D_MAP" \
  --calibration-json outputs/a5/frame-calibration-1jFoXh/initial.json \
  --output-dir assets/rm4d_ground_task_v1 --samples 100000 --seed 42
```

For A5 use `--rm4d-task-asset assets/rm4d_ground_task_v1`. The worker retains
the exact public-map grasp and uses current public Ground calibration. Do not
load this asset through the frozen baseline `from_files` factory: that factory
intentionally owns the baseline map and original floor.

Validation and limitations: [A5 task-domain validation](../../docs/A5_TASK_DOMAIN_ASSET.md).
