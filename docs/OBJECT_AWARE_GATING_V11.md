# A3/A5 v1.1 — object-aware operational gating

Status: implementation and natural E2E regression complete; review checkpoint,
not Pilot-2. The rule below was
recorded before the natural Gazebo regression. Pilot-1 remains the immutable-in-
meaning **v1 protocol/debugging result**; this revision does not recalculate its
method outcomes or failure taxonomy. No formal sample size is selected.

## Scope and shared rule

Explicit invocation: `run_a5_sim.py --operational-gating v1.1`. The default is v1.
A2 code, raw evidence/counts/state and thresholds are unchanged. A1 relevance,
A4 visibility/scoring/cost and RM4D/task-domain assets are unchanged.

For each original MID360 observation window, apply the same finite-return,
sensor-range, full map TF, grid and height filters as A2. Independently derive
occupied endpoint classes `TARGET`, `ENVIRONMENT`, `AMBIGUOUS`. Each class gets
at most one vote per cell/window; a mixture retains every observed class.
ENVIRONMENT or AMBIGUOUS always blocks. Missing positive target association
never grants an exception.

A ground vote requires an actual ground-height endpoint in that cell/window
and no ENVIRONMENT or AMBIGUOUS occupied endpoint in the same cell/window.
TARGET is neither a ground observation nor a reason to suppress real ground.
Persistent ground votes must meet the original A2 `free_observations` threshold.
No raw A2 counter is subtracted and no raw target cell becomes FREE.

For the existing padded BUNKER rectangle F(q,psi), define

```
blocked(F) = any raster-overlapped cell with ENVIRONMENT or AMBIGUOUS votes
             OR continuous_intersection(F, expanded_perceived_target)
ground_supported(F) = fully inside grid AND every raster-overlapped cell
                      has sufficient real ground votes
confirmed_exact = NOT blocked(representative) AND NOT blocked(exact)
                  AND ground_supported(exact)
```

The common `assess_footprint` implements both A3 representative and A5 exact
checks; Generic/Ours receive the same derived view. Closed rectangle SAT counts
touching as collision. The unchanged padded half-extents are .52/.39 m. Grid
aliasing alone from positively identified TARGET cells does not block a
continuously separated footprint. A3 preserves M_nominal and uses this blocking
for M_operational; U_task remains raw A2 unknown_score times M_operational.
UNKNOWN still does not block, FREE still need not have zero unknown_score.
This is operational blocking, not a complete navigation-feasibility claim.

## Runtime target association and physical allowance

Only the accepted initial aerial AIR_HANDOFF object center/yaw, known brick
dimensions (.240, .053, .115 m), synchronized RGB-D, CameraInfo and public
image-time map TF are used. The exact accepted color stamp selects the cached
reference. Missing data/TF gives UNAVAILABLE. There is no Gazebo GT input.
The object/reference is static during the active-observation phase; reference
reuse stops before manipulation. Unseen faces remain ambiguous.

Positive TARGET association requires all of:

1. LiDAR endpoint lies inside the expanded oriented known-size object box.
2. Map endpoint projected through the full image-time camera transform lies in
   the **one-pixel-eroded** red segmented component (never a dilated mask).
3. Registered RGB-D optical depth agrees with that projected depth within the
   declared metric budget, and passes the original observer depth gate.
4. The backprojected registered red surface point also lies in the expanded box.

Depth/color image-time transforms are composed through map for registration,
accounting for image skew in a stationary scene without an empirical Z offset.
The observer's .25–4.0 m gate and original segmentation acceptance are unchanged.

For optical depth z and actual focal lengths, the allowance is

```
tau(z) = 3*.010 + .002 + .001
         + .5*z*(hypot(1/fx_color,1/fy_color)
                  + hypot(1/fx_depth,1/fy_depth)) metres.
```

Sources in the existing SIM checkout:

- `src/platform/sim_platform_assets/models/MID360/MID360.sdf`: declared range
  resolution .002 m and Gaussian stddev .010 m. The custom Livox plugin uses
  ODE ray lengths; application of that declared Gaussian noise is **not verified**.
  Its 3-sigma term is a conservative declared-model budget, not a measured error.
- `src/p450/realsense_ros_gazebo/src/RealSensePlugin.cpp`: .001 m depth scaling
  and integer quantization. Camera SDF image noise is not a metric depth accuracy.
- Half-pixel diagonals from both actual CameraInfo matrices account for discrete
  projection/registration support. Observer stability spread is not used as an
  accuracy measurement.

The largest tau over the eight nominal object-box corners expands the object
in **all three axes**, including the continuous XY collision rectangle. Thus
association does not receive a tolerance that collision ignores. This is an
explicit deterministic engineering allowance, not calibrated uncertainty or a
guarantee of pose accuracy; it is not fitted to discovery, score or method wins.
At about 3.5 m with nominal SIM intrinsics it is approximately .046 m. The rule
and actual computed value are written to each operational summary. Do not tune
this rule from Pilot-2 outcomes; freeze it at review before any Pilot-2 launch.

## API and output

- `PerceivedTarget(center_xyz, yaw_rad, size_xyz, geometry_allowance_m)`.
- `derive_operational_evidence(grid, observations, target, labels, config)`
  returns detached derived TARGET/ENVIRONMENT/AMBIGUOUS and ground vote arrays.
- `assess_footprint(view, xy, yaw, footprint)` returns blocking causes, continuous
  target collision, clipping and real ground-support counts.
- Optional `operational=` argument on A3 construction, A5 assessment/decision
  and A6 policy; absent means v1.
- `target_reference.npz/json`: initial runtime RGB-D reference or unavailability.
- `operational_evidence.npz` and `operational_summary.json`: separate from raw A2.
- v1.1 decisions include per-exact operational diagnostics and version; raw
  occupied/free/unknown counts remain diagnostics, not the new confirmation test.

## Regression boundaries

Synthetic tests independently exercise aliasing, real intersection/contact,
mixed/ambiguous evidence, real per-window ground votes, full camera rotations,
representative/exact consistency and Generic/Ours sharing.

Pilot-1 Hard records have no saved initial RGB-D reference. Their replay cannot
prove positive TARGET identity. Report that limitation and retain conservative
blocking; continuous-separation diagnostics are not corrected trial outcomes.

The natural regression uses the original successful A5 scene/settings from
`outputs/a5/natural-approach-j87FQA`: initial UAV (-.5,0,1.5,0), BUNKER (3,-2.5,pi),
three 5 s observation windows, navigation timeout 120 s, max Ground travel 3 m,
flight/facade tolerance .05 m, unchanged D435/arm/grasp/lift parameters. Only the
explicit object-aware option is added. Physical outcome checking may observe
simulator state but provides no geometry or control input to the algorithm.

No Pilot-2, formal runs, navigation/D435/descend adjustment, XY-latch fix, SIM
feature or new experiment infrastructure is included.

## Recorded Hard regression results

[Full per-round/per-candidate report](../outputs/a6/operational-v11/hard-replay/replay.json)
and [geometry overlay](../outputs/a6/operational-v11/hard-replay/ours-cell-781-geometry.png).
All 5 saved Hard slots / 15 rounds reproduce the ten stored raw A2 arrays/grid
metadata and v1 candidate assessments exactly. M_nominal is unchanged.

| Hard method | Exact blocked v1 → v1.1 | Representative blocked | Confirmed |
|---|---:|---:|---:|
| Generic | 35 → 35 | 34 → 34 | 0 → 0 |
| Fixed | 35 → 35 | 34 → 34 | 0 → 0 |
| Ours | 33 → 33 | 32 → 32 | 0 → 0 |
| No occlusion | 35 → 35 | 34 → 34 | 0 → 0 |
| No cost | 34 → 34 | 33 → 33 | 0 → 0 |

Of 172 exact candidates, 135 intersect the nominal continuous perceived target
and 37 are separated. All remain blocked without positive target association.
In the Ours candidate-000008/source-585 case, the continuous gap is
**.1178201863 m**, but the exact footprint overlaps occupied cell 781; the
representative does not. The regression retains this known grid-alias geometry.
It does **not** claim the archived cell is positively segmented TARGET, nor
that these historical trials would succeed after revision.

Synthetic positive-association tests separately verify the intended change:
grid-alias-only blocked/unconfirmed becomes unblocked/confirmed **only with real
ground votes**. True continuous intersection/contact, ENVIRONMENT, AMBIGUOUS,
mixed evidence, missing ground votes and clipping remain blocked or unconfirmed
as appropriate. They are not presented as corrected Hard trial results.

## Original natural A5 regression result

[Run summary and status events](../outputs/a6/operational-v11/natural-a5-attempt-01/run_summary.json),
[independent physical check](../outputs/a6/operational-v11/natural-a5-attempt-01/physical_summary.json),
[operational evidence summary](../outputs/a6/operational-v11/natural-a5-attempt-01/operational_summary.json),
[final field/ranking visualization](../outputs/a6/operational-v11/natural-a5-attempt-01/nbv.png).

**PASS, one activation**: fresh aerial observation → RM4D/A1 → three real
MID360/A2 windows and A3/A4 → two NBV flights → confirmed exact candidate →
return/land → BUNKER → D435 refine → pregrasp/descend → AG95 grasp → lift.
The original runtime was shut down normally; dedicated ports were released;
the existing checker then finalized its CHECKS_PASS as PASS.

The accepted runtime reference has 373 red pixels / 189 valid red depth points,
color/depth stamps 135.093/135.096 simulation seconds. The derived allowance is
**.0439527451 m**. Final derived vote sums: TARGET 11, AMBIGUOUS 13, ENVIRONMENT 0,
ground 2634. Ambiguous votes were retained. There is no mask/counter clearing.

Identical acquired states replayed through both gates give:

| Window | Exact blocked v1 → v1.1 (37 candidates) | Representative blocked | Confirmed |
|---|---:|---:|---:|
| 1 | 0 → 31 | 0 → 34 | 0 → 0 |
| 2 | 31 → 32 | 32 → 34 | 0 → 0 |
| 3 | 34 → 34 | 35 → 35 | 2 → 2 |

Early additional blocking is intended continuous collision against the expanded
perceived target, even before enough occupied endpoints arrive. This scene does
not exercise a newly unlocked target-associated candidate. It verifies that
the conservative revision preserves the natural E2E path; it is **not** a v1/v1.1
performance comparison or evidence that the Hard bottleneck has been resolved.
All five final raw A2 arrays reproduce exactly; all three saved v1.1 candidate
assessments reproduce exactly after ordinary JSON tuple/list normalization.

Selected exact candidate: candidate-000008, source 544, map
(2.3994017, -.5971624, 2.0943951 rad), R=1.0, 107/107 supported cells, no
operational blocker. Ground travel 1.9747686 m; D435 refinement accepted;
three post-observation arm-controller goals succeeded; AG95 confirmed grasp;
TCP lift .1497568 m; independently observed physical target lift **.1485801 m**.
The active loop stopped at the unchanged 3-window budget. The plot's inherited
A4 “no real scan or flight” subtitle describes the *prediction model*, not this
run; the inputs above are actual Gazebo observations and the flights executed.

## Review, engineering corrections and next boundary

TDD caught and corrected three local integration issues before runtime:
optional-view/array variable shadowing; a cached representative gate using old
evidence while exact assessment used the current view; and retained registered
depth outside the unchanged observer depth gate. No navigation, D435-refine or
descend parameter was adjusted. The single natural runtime activation passed;
two pre-execution command mistakes (incomplete import-check arguments and an
incorrect passive-checker script path) were corrected before robot execution.
They are not method trials and do not alter Pilot-1 INVALID records.

Independent reviews of both the shared core/worker and runtime capture boundary
found no remaining Critical/Important/Minor issues. Missing/ambiguous historical
target association remains the substantive validation limit. Retain all six
Pilot-1 execution failures and their original classifications. No XY-latch
configuration change or focused runtime reproduction was performed here.

Fresh final verification: full core discovery **427 tests, 421 passed / 6
OpenCV-dependent skips** under Python 3.10 with the actual frozen RM4D checkout;
all **11/11** RGB-D boundary tests (including those six) passed under system
Python 3.8/OpenCV. Thus every discovered test was exercised in its supported
environment. Python 3.8 boundary syntax, v1.1 ROS import check, source whitespace
checks and the frozen-source/Pilot-1 no-diff checks passed. Generated CSV uses
the unchanged writer's CRLF convention; the full staged whitespace check uses
`git -c core.whitespace=cr-at-eol diff --cached --check`. SIM and frozen RM4D source
worktrees remained clean.

```bash
PYTHONPATH=src:tests MPLCONFIGDIR=/tmp/a6-mpl XDG_CACHE_HOME=/tmp/a6-cache \
RM4D_ROOT=/tmp/rm4d-aubo-baseline-v1.14uZXq/repo \
/media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
  -m unittest discover -s tests -q
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -W error \
  -m unittest tests.test_a5_target_support -q
```

The natural runtime and adapter commands are exactly the repaired-platform
commands in [A5 running instructions](A5_SIM_ACTIVE_PERCEPTION.md#running-with-the-repaired-sim-platform),
using `outputs/a6/operational-v11/natural-a5-attempt-01`, adding
`--operational-gating v1.1 --wait-for-status-subscriber`, and starting the
existing `SIM_ROOT/scripts/check_air_ground_pick_demo.py` before the adapter.
The actual command sequence is recorded here for reproduction. Use a fresh
output name instead of overwriting the retained run; launch/checker/adapter run
in separate terminals with the same variables:

```bash
export SIM_ROOT=/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation
export P450_PX4_ROOT=/media/lu/P450_PAPER/P450-PAPER/workspaces/dependencies/px4
export RM4D_ROOT=/tmp/rm4d-aubo-baseline-v1.14uZXq/repo
export CORE_PY=/media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python
export RM4D_MAP=/media/lu/P450_PAPER/RM4D_AUBO/runs/formal-10m/data/rm4d_aubo_i5_joint_42/10000000/rmap.npy
export RUN_DIR="$PWD/outputs/a6/operational-v11/natural-a5-attempt-01"

bash scripts/run_a5_gazebo.bash "$RUN_DIR" bunker_x:=3.0 bunker_y:=-2.5 \
  flight_position_tolerance:=0.05

# Passive checker, started before the adapter:
"$SIM_ROOT/scripts/with_p450_env.bash" /usr/bin/env \
  ROS_MASTER_URI=http://127.0.0.1:11951 GAZEBO_MASTER_URI=http://127.0.0.1:11952 \
  /usr/bin/python3 -B "$SIM_ROOT/scripts/check_air_ground_pick_demo.py" \
  --summary "$RUN_DIR/physical_summary.json" --timeout 1200 --maximum-ground-travel 3.0

# Adapter, using only the new gating opt-in:
"$SIM_ROOT/scripts/with_p450_env.bash" /usr/bin/env \
  ROS_MASTER_URI=http://127.0.0.1:11951 GAZEBO_MASTER_URI=http://127.0.0.1:11952 \
  MPLCONFIGDIR=/tmp/a6-mpl XDG_CACHE_HOME=/tmp/a6-cache \
  /usr/bin/python3 -u -B scripts/run_a5_sim.py --output-dir "$RUN_DIR" \
  --core-python "$CORE_PY" --rm4d-root "$RM4D_ROOT" \
  --rm4d-config "$RM4D_ROOT/configs/mr4_offline_base_placement.json" \
  --rm4d-map "$RM4D_MAP" --rm4d-task-asset assets/rm4d_ground_task_v1 \
  --max-ground-travel 3.0 --navigation-timeout 120 --facade-position-tolerance 0.05 \
  --operational-gating v1.1 --wait-for-status-subscriber

# Historical Hard: separate output; never writes into the input trial tree.
PYTHONPATH=src MPLCONFIGDIR=/tmp/a6-mpl XDG_CACHE_HOME=/tmp/a6-cache \
  "$CORE_PY" scripts/replay_operational_gating.py \
  --pilot-root outputs/a6/pilot-20260908 \
  --output-dir outputs/a6/operational-v11/hard-replay
```

After successful checker/adapter exit and normal roslaunch shutdown, with the
dedicated ports released, the existing checker's `--finalize-summary` operation
was used on this run's physical summary. No new finalization mechanism was added.

At the review checkpoint, decide whether to approve a bounded Pilot-2 with this
explicit association rule frozen. Do not tune it from outcomes or infer that
this one successful regression validates Hard-scene discovery. No Pilot-2 or
formal matrix is authorized by this implementation result.
