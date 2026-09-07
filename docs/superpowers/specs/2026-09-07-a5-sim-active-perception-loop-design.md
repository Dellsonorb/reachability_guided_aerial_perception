# A5 — minimal SIM active-perception integration

The user authorized autonomous ordinary design/implementation/review/Git decisions on 2026-09-07. A4 is frozen by PR #4, main `431a907`; A5 starts on `feature/a5-sim-active-perception-loop`. No A1/A2/A3/A4 method or SIM source changes are planned.

## Selected approach

Use a small A5-local subclass of the existing SIM `AirGroundPickDemo`, replacing its aerial phase and exact candidate provider only. Robot commands remain on public flight, navigation, perception, MoveIt and gripper interfaces. The alternative of fixing the SIM yaw-only flight facade is deferred: filter these currently unexecutable goals in A5, retain current-pose rescan and translated viewpoints with yaw. Do not force movement or suppress a winning rescan just to obtain a positive demonstration.

The public/E2E SIM checkout is `/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation`. It publishes `/uav1/livox/lidar` as PointCloud2 (the earlier parent checkout used LivoxCustomMsg). Its composite sensor transform is still pitch .35 rad, xyz (.14714489037,0,.27696863564).

## Loop and input semantics

1. Use the existing aerial RGB-D observer and grasp generator. Preserve exact observed map TCP. Use the existing SIM query-only local-Y 1e-6 regularization when calling frozen BasePlacementAPI; this is already part of the SIM/RM4D interface, not a new A1 quality term. Preserve the exact TCP separately for execution.
2. Call RM4D once and retain the complete raw result. A1 is built from evaluated candidates, not the lossy top-K ROS service. Grid remains map/0.10 m and fixed for the run.
3. At each stable hover collect a NEW short MID360 scan window (default 5 s). Each PointCloud2 is transformed using its OWN header-stamp T_map_sensor and checked against current and stamped hover pose. Express all endpoints in the last accepted sensor frame/time for the existing A2 observation API. Zero/no-return and nonfinite points are not evidence. Do not consume moving clouds, substitute latest TF, or attempt within-packet moving-scan deskew. The window is an ordinary nonrepetitive-LiDAR acquisition setting, introduced after real single-message captures proved too sparse for footprint assessment.
4. Each whole hover window is ONE A2 observation: multiple endpoint messages do not grant extra per-cell votes. A2 accumulates these observation votes; rebuild A3 from that snapshot; rank with frozen A4. Default max_viewpoints=3 counts initial observation and rescans. Stop on Score<=0, no predicted task gain, or the bounded observation count. Reaching the guard is not convergence or success.
5. FLY_TO carries map-frame UAV base pose and yaw quaternion. Wait for public action success AND measured pose/yaw/low speed before sampling. Filter yaw-only goals within the facade position-arrival tolerance; no native command bypass. Small explicit flight bounds are an operating-area constraint, not path planning or an obstacle-height model.
6. Recover the original per-A1-cell winner from raw candidates using A1's relevance and first-tie rule. Recheck its EXACT footprint, not its representative cell-center footprint: require all covered cells FREE, no clipping, and current A3 representative support not occupied-blocked. FREE need not have unknown_score=0. Pick highest original relevance with stable original-order ties; do not use final_score/travel/residual as relevance.
7. If no confirmed exact candidate exists when the loop stops, report that outcome, do not execute an unknown footprint. Otherwise return UAV to the existing clear initial observation/landing location and LAND, then reuse existing BUNKER -> D435 refinement -> AUBO/AG95 -> lift stages with the selected exact candidate unchanged. Navigation and MoveIt still decide actual execution feasibility.

## Minimal Python boundary

Noetic/MoveIt stays in system Python; frozen research modules use the existing Python 3.10 environment. A simple request/response-file subprocess runs the core. Initial RM4D output is cached once. With at most three observations, replaying the small ordered NPZ cloud history from an empty A2 mapper reproduces the exact accumulated belief without adding a new state-loading API or persistent IPC framework. Each historical scan is applied once to that reconstructed mapper; histories are not applied to an already accumulated mapper. Point clouds and result snapshots are normal debug outputs, not evidence infrastructure.

Request interface for `scripts/a5_core_worker.py --request PATH --response PATH`:

- `init`: `output_dir`, `sim_root`, `rm4d_root`, `rm4d_config`, `rm4d_map`, exact `grasp` (A1 four-field schema), `current_bunker_pose` [x,y,yaw], optional `config`.
- `observe`: `initial_file`, `observations` (ordered NPZ paths), `uav_pose` [x,y,z,yaw], `output_dir`.
- NPZ observation fields: `points_xyz`, `T_map_sensor`, scalar `stamp_s`, scalar `frame_id`. The merged points and reference transform/time use the last accepted sensor pose. Chunk timestamps, point counts and stamped transforms are retained only for ordinary geometry debugging; the worker consumes one merged observation, not one vote per chunk.
- Response always has `ok`; init returns `initial_file` and A1 summary; observe returns `round`, `stop_reason` (null to continue), `next_viewpoint` [x,y,z,yaw] or null, `selected_candidate` {candidate_id,x,y,yaw,relevance,source_id} or null, and geometric belief/candidate diagnostics. Errors return `ok=false,error` and a nonzero exit.

## Validation and research boundary

Unit tests cover exact-candidate identity/ties, exact versus representative footprint, FREE/nonzero-unknown, observation accumulation, stale ordering, stop/candidate filtering and core protocol. A real Gazebo run must show a nonzero A4-selected aerial displacement, real MID360 belief change affecting/confirming a ground candidate, and physical retained grasp plus lift. Gazebo model states may be used only by an external success check, never algorithm input or goal selection. Initial scene configuration is allowed; no object/base teleporting during the run.

The outcome may be an exposed integration failure or a method limitation. Do not alter frozen methods, loosen semantics, fabricate observations, or expand to formal experiments to force success. Stop for the user's stated research-decision conditions; ordinary execution issues are investigated autonomously.
