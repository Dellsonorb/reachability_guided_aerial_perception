# A5 — SIM active-perception integration (not yet E2E complete)

Current boundary: the authorized **Ground reference-frame bridge is verified**;
the repaired UAV public TF also passes real ground-height checks. Correctly
calibrated natural ground-brick queries are outside the frozen RM4D map's
`z=[0,1.3] m` domain and return `NO_INVERSE_REACHABLE`. See the
[geometry, actual IK/planning validation and coverage boundary](A5_GROUND_RM4D_HEIGHT_BOUNDARY.md).
No Ground/grasp/lift success is claimed, and A5 remains a WIP feature branch.

A4 was merged through PR #4 and is frozen at main `431a907`. A5 lives on
`feature/a5-sim-active-perception-loop`. This implementation adds only an
integration/orchestration layer; A1/A2/A3/A4 are unchanged. The subsequently
authorized public-map correctness repair is isolated in SIM branch
`feature/fix-sim-uav-map-localization`, not in these research modules.

## Implemented boundary

`scripts/run_a5_sim.py` subclasses the existing SIM `AirGroundPickDemo`. Only
the aerial phase and RM4D candidate provider are replaced. Ground navigation,
D435 refinement, MoveIt, AG95 confirmation and lift retain the existing SIM
implementation. No Gazebo model state enters the algorithm.

The Python 3.8 ROS adapter consumes actual `/uav1/livox/lidar` PointCloud2,
filters nonfinite/zero returns, waits for stable position/yaw/velocity, and
collects a 5-second scan window per observation. Every packet uses its own
header timestamp for the full `T_map_sensor`. The mount includes
pitch 0.35 rad and the ray-sensor translation, not just a yaw rotation. A delayed
cloud must also have an anchor-consistent UAV pose at its own timestamp.
Flight arrival is checked against the requested viewpoint. After a potentially
long RM4D query, capture stability uses a fresh measured pose as a fixed anchor,
not the old request; the request/anchor difference is logged. It is not re-anchored
during a window or its retries. A4 receives the last actual stamped UAV pose.
Packets are re-expressed in the last sensor frame/time. Each entire window is
one A2 observation, never extra votes proportional to packet count. This is
between-packet coordinate alignment during hover, not within-packet deskew.
If current/stamped hover checks fail during collection, discard the partial
window, wait for the same continuous settling interval, and start with fresh
packets. The original 20-second wall capture deadline is not reset. An incomplete
window never updates A2; frame/numeric errors are not reinterpreted as evidence.

The Python 3.10 worker calls frozen RM4D once using the existing SIM query-only
regularization and retains the complete evaluated result and unmodified grasp
TCP. The new A5-only `FrameBridge` reads the public nominal Ground reference
height and verifies its mount against frozen RM4D; it translates the query into
the original RM4D `world` frame and translates global result transforms back
to public map. `initial.json` retains `baseline_result`, map `result`, and
`frame_calibration` separately. This is not an A2 ground correction: A2's ground
remains z=0 and A1–A4 are unchanged.
Each observation request replays its small ordered cloud history into an
empty A2 mapper, then invokes the frozen A3 and A4 functions. This does not add
extra votes to an already accumulated belief.

Defaults: fixed map/0.10 m grid, 4 m square around the observed grasp, 3 total
observations including initial/rescans, XY offsets {-2,0,2} m, flight weight
0.25, operating bounds x[-4,4], y[-3,3], z[0.5,3] m. These are integration run
settings, not a new planner. A4 yaw-only goals within the current flight facade's
position tolerance are excluded because that facade may acknowledge them without
issuing a yaw command. Translated yaw goals and current-pose rescans remain.

Stop on `NONPOSITIVE_SCORE`, `NO_PREDICTED_TASK_GAIN`, or
`VIEW_BUDGET_REACHED`. The latter is a guard, not convergence. Recover the original
per-A1-cell winning candidate (including continuous x/y/yaw and first-tie order),
not the cell center. Select the highest-relevance candidate whose **exact**
unclipped footprint is all A2 FREE and whose A3 representative is not occupied
blocked. FREE may retain positive unknown_score. This is observed-ground support,
not a proof of navigation/clearance feasibility. At a stop without a confirmed
candidate, abort; do not execute an unknown footprint or substitute generic NBV.

The request/response API is documented in the
[design](superpowers/specs/2026-09-07-a5-sim-active-perception-loop-design.md).
Saved point clouds, per-request responses and plots are ordinary debugging data,
not an evidence framework. The current renderer saves the latest A2/A3/A4 snapshot;
individual round response JSONs and cloud history remain available for replay.

## Actual Gazebo attempt: `gazebo-WwmQis`

The natural SIM scene used the existing brick at (2,0,0.0575), BUNKER at
(3,0,0.36), yaw pi, and initial aerial view (-0.5,0,1.5), yaw 0. No object or base
was teleported during execution. The UAV took off, reached the initial view,
obtained the aerial grasp estimate, ran RM4D and captured one real MID360 scan.

| Quantity | Observed result |
|---|---:|
| RM4D evaluated / valid | 256 / 180 |
| A1 per-cell winning candidates | 21 |
| MID360 valid endpoint rows | 1608 |
| A2 UNKNOWN / FREE / OCCUPIED cells | 1523 / 0 / 77 |
| A3 occupied-blocked representatives | 21 / 21 |
| Task uncertainty mass | 0 |
| Stop | `NO_PREDICTED_TASK_GAIN` |
| Confirmed exact candidate | none |
| A4-selected flight / ground execution / grasp / lift | **not reached** |

The adapter reported failure and used inherited landing cleanup. The dedicated
runtime was then stopped. This is an unsuccessful integration attempt, not a
successful active-perception demonstration and not evidence that A4 improves
ground decisions.

- [Decision and exact candidate diagnostics](../outputs/a5/gazebo-WwmQis/decision.json)
- [Frozen RM4D input/result](../outputs/a5/gazebo-WwmQis/initial.json)
- [Actual cloud and stamped transform](../outputs/a5/gazebo-WwmQis/observation_01.npz)
- [A2 summary](../outputs/a5/gazebo-WwmQis/a2/summary.json)
- [Existing SIM physical checker: FAIL](../outputs/a5/gazebo-WwmQis/physical_summary.json)

![First real input, frozen A4 prediction](../outputs/a5/gazebo-WwmQis/nbv.png)

The reused frozen A4 renderer's footer “no real scan or flight” describes its
candidate visibility/gain prediction, not the origin of this A5 input: the input
scan above is real SIM data; proposed NBV observations are not measurements.

## Geometric prerequisite exposed by the original run

The original public TF did not place the observed horizontal ground within
A2's fixed map-ground band. With the correctly stamped composite transform,
distant returns in the saved hover scan form a near-plane at approximately
`z = 0.00249*x + 0.00921*y + 0.04940` m. At (2.8,0), that is about **0.0564 m**,
above frozen A2's **0.05 m occupied threshold**, rather than within its
ground band `0 +/- 0.02 m`. This plane fit was performed only as an offline
diagnostic; it was not used to correct the input or update A2.

A separate read-only check after landing found:

| Quantity | map/world z (m) |
|---|---:|
| Public `map -> uav1/base_link` TF | 0.1452550120 |
| Gazebo physical UAV pose, external diagnostic only | 0.0446725372 |
| Difference | 0.1005824748 |
| Distant return plane intercept in public map | 0.09991238 |

The stationary cloud timestamp was 279.553 s and the diagnostic model-state
timestamp 279.597 s. The close agreement between height offset and apparent
ground elevation supports a localization/frame-datum issue, not an A5 rotation
or timestamp substitution. At the time, SIM used a static `map -> uav1/odom` offset
based on spawn position plus the onboard estimated pose; see SIM
`air_ground_standalone.launch:34`. Sensor extrinsics include the expected ray
offset and match the published contract.

The current BUNKER also occupies part of the candidate area. Those body returns
were not masked or relabeled FREE. Their contribution must be separated from
misregistered ground once the TF prerequisite is resolved; this run does not
justify treating every occupied cell as a false obstacle.

The failure invalidates the **direct-input assumption for this SIM run**, not the
paper's task-weighting hypothesis. Changing A2 thresholds, adding adaptive ground
segmentation, or substituting Gazebo truth would alter the tested assumptions or
cross the platform boundary. None was done. This prompted the separately
authorized SIM repair below, after which A5 resumed with frozen A2 semantics.
A5 is not merged or frozen as complete.

## Authorized platform repair and corrected retry

The WIP A5 checkpoint `eb33ea4` has been pushed to its feature branch without
merging main. In SIM, the incorrect static spawn-based map/odom edge has been
replaced by a same-time full-SE(3) localization correction. A SIM-private stamped
physical body pose is consumed only inside that platform adapter. AGENT still
uses only public map TF; no empirical Z offset, threshold change, body masking
or Gazebo pose substitution was added to A2/A5.

In corrected run `calibrated-mexIjS`, actual known-ground returns had maximum
absolute height error **0.000000402 m landed** (5 frames) and **0.0006364 m in
hover** (10 frames), both within the unchanged A2 0.02 m tolerance. Physical
landed base_link height remained approximately 0.04484 m. Public P450/Ground TF
and P450 D435 frame checks passed.

The first fresh A5 scan now provided 126 ground/free votes and 50 occupied votes.
All 31 representative candidates were still blocked, but the relevant occupied
endpoints were real current BUNKER/arm returns at z=0.394..1.028 m, rather than
misregistered ground. This run correctly stopped with no executable candidate;
it is not a completed A5 result.

The next natural run parks BUNKER initially at (3,-2.5), yaw pi, outside the
candidate region. This is an explicit initial scene setting, not a mid-run
teleport or self-body evidence suppression. `--max-ground-travel 3.0` exposes the
existing SIM travel guard for that parking distance; it does not change RM4D
quality, A1 relevance, A4 cost or candidate ranking. The default remains SIM's
1.10 m unless explicitly overridden. Exact footprints must still be entirely
A2 FREE before Ground execution.

That clear-parking run (`clear-parking-iaOxLx`) executed two A4-selected flights.
Across three single-packet observations, task uncertainty mass changed
295.19 -> 274.15 -> 247.80, and FREE cells changed 0 -> 29 -> 86. However,
the best exact footprint had only 17/108 FREE cells; the adapter stopped at the
view budget without executing Ground. This motivated the 5-second scan-window
acquisition above, without changing the per-observation A2 vote or all-FREE gate.

## Running with the repaired SIM platform

Use the public/E2E SIM checkout, not its older parent checkout. Existing machine
paths used here:

```bash
export SIM_ROOT=/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation
export P450_PX4_ROOT=/media/lu/P450_PAPER/P450-PAPER/workspaces/dependencies/px4
export RM4D_ROOT=/tmp/rm4d-aubo-baseline-v1.n9oGee/repo
export RM4D_PYTHON=/media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python
export RM4D_MAP=/media/lu/P450_PAPER/RM4D_AUBO/runs/formal-10m/data/rm4d_aubo_i5_joint_42/10000000/rmap.npy
export A5_RUN_DIR="$(mktemp -d -p "$PWD/outputs/a5" gazebo-XXXXXX)"
bash scripts/run_a5_gazebo.bash "$A5_RUN_DIR"
```

`RM4D_ROOT` must name an available frozen baseline checkout; the temporary path
above is the checkout used for this attempt, not a portable installation promise.
The runtime-only wrapper delegates startup and Ctrl-C teardown to existing
roslaunch. In another terminal, use the same `A5_RUN_DIR` and dependency variables:

```bash
source /opt/ros/noetic/setup.bash
source "$SIM_ROOT/install/p450-clean/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11951
/usr/bin/python3 -B scripts/run_a5_sim.py \
  --output-dir "$A5_RUN_DIR" --core-python "$RM4D_PYTHON" \
  --rm4d-root "$RM4D_ROOT" \
  --rm4d-config "$RM4D_ROOT/configs/mr4_offline_base_placement.json" \
  --rm4d-map "$RM4D_MAP"
```

The existing SIM `scripts/check_air_ground_pick_demo.py` can be started before
the adapter to observe retained AG95 grasp and physical brick lift independently.
It must not supply control inputs. This attempt's checker correctly reported FAIL.

## Verification and review

Independent adapter spec review and whole-integration quality review found no
remaining blocking code issue. The latter checked exact identity, first ties,
representative/exact occupied gates, stamped transforms and stopped-response
semantics. The review does **not** establish Gazebo E2E completion.

Latest local verification: **219 tests passed** across frozen modules and A5;
the 47 ROS-independent adapter tests also passed separately under system Python
3.8. Shell syntax and diff whitespace checks passed; the frozen source-directory
diff against main was empty.

```bash
PYTHONPATH=src MPLCONFIGDIR=/tmp/a5-mpl XDG_CACHE_HOME=/tmp/a5-cache \
  "$RM4D_PYTHON" -m unittest discover -s tests -q
/usr/bin/python3 -m unittest tests.test_a5_ros_support -q
bash -n scripts/run_a5_gazebo.bash
```

No additional dependencies, baseline/ablation benchmark, security/evidence
framework, A6 stage or frozen-method changes were introduced.

## Calibrated fresh natural-scene retry

After the bridge's real SIM planning-only check passed, run
`calibrated-domain-run-XaSgKR` used the same natural scene and unchanged initial
parking, took off, and obtained a fresh aerial observation. Exact map TCP z was
0.082807036 m; reference query TCP z was -0.277192964 m. The frozen planner
returned inverse_reachable=0, evaluated=0, valid=0; A1 saved
`NO_INVERSE_REACHABLE`, consistent with the independent frozen-domain check.

The adapter then attempted its existing real MID360 capture. A partial window
of 26 packets was discarded after its unchanged hover gate failed, and the
20-second capture deadline expired. No full observation was accepted, so this
run did not update A2 or enter NBV/Ground/grasp/lift. This separate sampling
failure cannot explain or fix the already completed RM4D out-of-domain query.
No sampling gate, baseline map, exact target, or A2 threshold was changed.

- [Fresh actual initialization](../outputs/a5/calibrated-domain-run-XaSgKR/initial.json)
- [Fresh A1 status](../outputs/a5/calibrated-domain-run-XaSgKR/a1/summary.json)
- [Attempt summary](../outputs/a5/calibrated-domain-run-XaSgKR/run_summary.json)
- [Post-cleanup landed ground check](../outputs/a5/calibrated-domain-run-XaSgKR/landed_ground_check.json)

Inherited cleanup landed the UAV. Five actual ground-return frames remained
within the unchanged 0.02 m A2 tolerance (maximum absolute error 9.94e-6 m).
The task-owned runtime was then stopped.

The frame-calibration subtask is validated; A5 remains incomplete. Continuing
the ground-brick task needs a decision on map coverage versus task/scene scope,
not another height offset. This retry is not an E2E success.
