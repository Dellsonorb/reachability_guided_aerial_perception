# A6 preflight: common initial view fails necessary RGB-D admission

2026-09-08. **0 activated attempts / 14 maximum; no Gazebo pilot launched.**
This is a scene-setup check, not an INVALID_TRIAL or a method failure. All slots
remain NOT_RUN. A6 ROS execution and robot-efficiency instrumentation are not
yet completed or validated; no pilot-ready/E2E claim is made.

## Reproduction from unchanged SIM contracts

SIM files inspected at the corrected platform checkout:

- `src/platform/sim_platform_bringup/launch/p450_runtime.launch`: exact public
  `T_base_optical` translation `(0.13614773689465415,0,0.11272472554168546)` and
  RPY `(-1.9207963267948966,0,-1.5707963267948966)`.
- `src/demos/air_ground_pick_demo/config/air_observer.yaml`: depth `(0.25,4.0) m`.
- `src/demos/air_ground_pick_demo/config/demo.yaml`: target dimensions
  `(0.240,0.053,0.115) m`.
- `src/demos/air_ground_pick_demo/src/air_ground_pick_demo/perception.py`:
  `backproject_mask` keeps only finite optical depths strictly inside those
  bounds. `red_target_observer.py` actually passes the configured limits to it.

The installed public launch and observer configuration compare equal to tracked
source. The issue is the new A6 setup proposal, not a SIM calibration regression.

For the design's level UAV at `(-2.5,0,1.5), yaw=0`, the optical center is
`(-2.363852263105346,0,1.6127247255416854)`. The optical depth of a map point is

\[
d_{opt}(p)=\cos(0.35)(p_x+2.363852263105346)
          -\sin(0.35)(p_z-1.6127247255416854).
\]

Depth is affine over the rotated brick cuboid, so minima/maxima over its eight
corners bound **all points of the entire cuboid**, not merely its center.

| Setup | Optical depth interval (m) | Frozen gate |
|---|---:|---|
| Nominal target `(2,0)`, yaw 0 | 4.5001–4.7650 | outside 4.0 m |
| Easy seed 2026090801 | 4.4465–4.7163 | outside |
| Moderate seed 2026090802 | 4.4967–4.7637 | outside |
| Hard seed 2026090803 | 4.6165–4.8868 | outside |

An independent direct call to the unchanged SIM `backproject_mask`, with a
synthetic 25-pixel mask set to each seed's **minimum** target depth, rejects all
three with `target mask has no valid depth`. This isolates the existing depth
gate. It is not a rendered Gazebo observation, a trial, or an E2E result.

Run the necessary check with existing system Python/PyYAML (no new dependency):

```bash
/usr/bin/python3 scripts/a6_scene.py \
  --sim-root /media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation \
  --config configs/a6_pilot.json \
  --output outputs/a6/preflight-20260908/depth_admission.json
```

Exit **2 is the expected inadmissible-setup result**; JSON retains the values and
source paths. A passed depth check would prove only depth opportunity, not FOV,
pixel count, occlusion, live bootstrap or retrieval success.

## Why no silent correction or doomed 14-run comparison

The approved A6 design §5/§10 explicitly calls for common-setup review if the
initial RGB-D/bootstrap and basic ground-observation geometry cannot both hold.
The proposal at x=-2.5 fails even the RGB-D necessary condition. No packets,
seed, extra waiting or policy weighting can correct that nominal range mismatch.

This check does **not** refute reachability-guided NBV. It occurs before the
initial RM4D/A1 query and is common to every method. It also does not justify
dropping actual unsuccessful trials: no trial has started. Remaining obstacle
template/tier qualification is pending, not claimed passed.

Simply restoring the A5 initial x=-0.5 is not an accepted fix: its RGB-D worked,
but initial fixed-view MID360 ground support was insufficient in A5. A diagnostic
using `outputs/a5/natural-approach-j87FQA/initial.json` shows the tradeoff:
rebuild with frozen `worker.make_field`, recover `core.candidate_catalog`, create
an empty A2 belief on that grid, and count nonempty/unclipped exact
`footprint_cells` whose cells all pass A4 `predict_visibility(...).range_fov`.
Yaw is zero and height is 1.5 m for this calculation. At that height, x=-2.5
has 36 complete exact footprints inside idealized MID360 range/FOV; x=-1.7 has
only one and its nominal brick optical interval straddles 4.0 m; x=-1.6 has zero
complete footprints. These are setup diagnostics, not policy selection or live
FREE evidence, and none is silently substituted into the pilot.

Requested review: authorize revising only the **single common initial scene
setup** using RGB-D/MID360 joint geometry, then freeze it before any attempt.
Retain all A1–A5 parameters, sensor gates, three-window semantics, seeds and
gain-only primary comparison. No threshold, algorithm or sensing-budget tuning;
no choice based on Ours/Generic outcomes.
