# A5 Ground / RM4D reference calibration and coverage boundary

A5 is **not E2E complete**. The user-authorized integration frame bridge is
implemented and checked against public TF and actual SIM IK/planning. It does
not modify frozen M-R3/M-R4 or A1–A4. Correct calibration exposes a separate
limitation: the natural ground-brick query is outside the frozen map's z domain.

## Measured public frames versus frozen candidates

Public TF was sampled read-only during `measured-hover-jgZZo5`. The BUNKER was
parked at (3,-2.5), yaw pi, and had not moved during this check.

| map height | Frozen RM4D candidate | SIM public TF |
|---|---:|---:|
| BUNKER reference / base_link | 0 m | 0.36 m |
| AUBO base | 0.122 m | 0.482 m |
| Relative BUNKER -> AUBO translation | 0.122 m | 0.122 m |

The relative mount agrees. The omitted quantity in the nominal placement chain
is the absolute BUNKER base_link height. The physical structural height must not
be erased by moving SIM's map or pretending base_link is on the ground.

- [Public TF values](../outputs/a5/measured-hover-jgZZo5/ground_mount_tf.json)
- [Frozen query and evaluated candidates](../outputs/a5/measured-hover-jgZZo5/initial.json)

## Source and numerical check

In the frozen `e9d4312` RM4D checkout:

- `configs/mr4_offline_base_placement.json` sets `T_bunker_aubo.z=0.122`.
- `rm4d/base_placement.py`, `_generate`, places `world_aubo` at that mount height.
- `rm4d/base_placement_geometry.py`, `matrix_to_se2`, requires the recovered
  BUNKER reference to lie at z=0.
- `rm4d_robot_base_z_m=0.01` is the standalone PyBullet AUBO's internal map/IK
  origin offset. It is added to the arm-relative validation target; it does not
  supply the missing BUNKER height.
- `rm4d/base_placement_api.py` relabels map/world without a spatial conversion.

SIM's `rm4d_sim_integration/geometry.py:84-108` preserves TCP position and applies
only its existing query-orientation regularization. Before the bridge, A5 passed
that request to the frozen API without height conversion. Ground launch and request
geometry were already this way at SIM `30b645b`, before the UAV localization fix.

For nominally valid `candidate-000005` in the saved run:

```
T_map_aubo T_aubo_flange T_flange_tcp
    -> TCP = (1.996866893, 0.005384276, 0.081528682) m

same arm-relative target, at SIM AUBO mount height 0.482 m
    -> TCP = (1.996866893, 0.005384276, 0.441528682) m

delta = (0, 0, 0.36) m
```

This uses saved arm-relative target transforms; it is not a new physical FK or
MoveIt failure measurement. The saved IK configuration/joint margin validate the
nominal chain, not the physical target at the other mount height. A5 sends only
the exact candidate x/y/yaw to the existing ground stage, and MoveIt would plan
again using actual public TF. Another arm configuration might therefore succeed.
That possibility does not make the nominal validation a same-target certificate.

## Verified integration bridge

The exact 0.36 m is the existing **nominal reference plane convention**, not a
fit to LiDAR or a claim about exact collision settlement:

- `air_ground_standalone.launch` passes `bunker_z` (default 0.36) to both
  `map -> ground/odom` and `ground_robot_runtime.launch` spawn z.
- `bunker_planar_move_plugin.cpp` publishes local odom/base z=0.
- The runtime renderer's base collision box bottom is
  `-0.160420741 - 0.395154782/2 = -0.357998132 m` relative to base.
- `ground_robot.urdf.xacro` mounts AUBO at `0.042 + 0.08 = 0.122 m`.
- Fresh public TF and `/ground/spawn_ground_robot/z` confirmed these values.

A5 reads the public transforms at a common stamp, checks their horizontal
convention and equal frozen mounting transform, and derives h from TF. No 0.36
constant is used in production AGENT code. With B denoting BUNKER and A AUBO:

```
C = T_reference_map = Trans(0,0,-h)
T_reference_grasp = C T_map_grasp
T_map_B = inverse(C) T_reference_B
inverse(T_map_B T_B_A) T_map_grasp
    = inverse(T_reference_B T_B_A) T_reference_grasp
```

The frozen API is called in its original `world` convention. Returned global
transforms are translated and renamed to public-map transforms; arm-local
transforms, validation, joints, margins and ordering are unchanged. The original
exact map grasp remains unchanged for perception, visualization and manipulation.
M-R3/M-R4 validation remains valid in its original frame. This is additional
integration validation, not a redefinition of baseline validation.

### Actual validation

For an explicitly synthetic in-domain representative TCP at map z=0.441528682 m,
the frozen query yielded 256 evaluated / 192 valid candidates. This diagnostic
target is **not** the scene brick and was never substituted into A5.
Across the 192 valid candidates, the maximum difference between the RM4D and
SIM-mount-derived arm-local query TCP matrices was `5.5511e-16`.

Actual SIM `/compute_fk` for candidate-000014 matched the exact local TCP
position within `7.33e-16 m`; its rotation-matrix difference was `1e-6` from the
existing query-only regularization. Actual collision-aware `/compute_ik`
succeeded, and RRTConnect produced a 28-point trajectory in 0.0417 s. No arm/base
motion was executed: the candidate-local target was expressed under the parked
SIM arm base for this planning-only check, with its floor included.

Candidates 000185 and 000005 had approximately 0.10 mm FK residual and returned
no exact MoveIt IK solution even with collisions disabled and a frozen-joint
seed. These failures are retained; numerical acceptance by frozen RM4D does not
guarantee exact MoveIt IK/planning for every candidate. No tolerance was changed.

- [In-domain representative input/results](../outputs/a5/in-domain-geometry-xvoUOw/initial.json)
- [Relative geometry check](../outputs/a5/in-domain-geometry-xvoUOw/geometry_check.json)
- [Actual SIM FK / IK diagnostic](../outputs/a5/in-domain-geometry-xvoUOw/fk_seed_diagnostic.json)
- [Actual successful SIM IK / planning](../outputs/a5/in-domain-geometry-xvoUOw/moveit_check_candidate14.json)

## Frozen map coverage blocks the natural ground target

For the unchanged previously observed map TCP z=0.081528682 m:

| Stage | z (m) |
|---|---:|
| Public exact map TCP | 0.081528682 |
| Calibrated reference TCP | -0.278471318 |
| Reference flange | -0.078471318 |
| Internal RM4D flange lookup (minus 0.122 mount plus 0.01 robot origin) | -0.190471318 |
| Frozen map's stored z domain | [0, 1.3] |

`get_z_index()` raises `IndexError` before inverse generation; the frozen planner
returns inverse_reachable=0, evaluated=0, valid=0. A1 retains the correct
`NO_INVERSE_REACHABLE` field status. This is a **map coverage-domain limit**,
not physical unreachability, an exhausted 256 validation budget, or A2 UNKNOWN.
It is independent of sampling-window timeouts.

- [Calibrated natural target input/result](../outputs/a5/frame-calibration-1jFoXh/initial.json)
- [Exact domain calculation and frozen exception](../outputs/a5/frame-calibration-1jFoXh/domain_check.json)

Continuing this ground-brick E2E would require a decision about the frozen map's
coverage or the validation scene/task. Neither is silently changed: no negative-z
map extension, physical target raising, replacement candidate generator, ground-z
shift, threshold relaxation, or baseline model edit has been made.

## Earlier independent sampling issue

Ordinary acquisition work also remains: native odometry hover drifts physically
during the long RM4D query. The adapter now distinguishes flight arrival from a
fixed fresh measured capture anchor, preserves each packet's timestamped TF,
and discards incomplete windows. A 5-second window was captured successfully
once (51 packets, 85,210 endpoints, one A2 observation), but the subsequent view
timed out. A 2-second / five-observation run also timed out before its first full
window. These are unsuccessful attempts, not proof of ground selection or lift.

The earlier clear-parking run executed two A4-selected flights and changed belief,
but its best exact footprint had only 17/108 FREE cells. No run in this repair
session reached BUNKER navigation, D435 refinement, grasp or lift. All task-owned
Gazebo/adapter processes were stopped; diagnostic files remain available locally.
