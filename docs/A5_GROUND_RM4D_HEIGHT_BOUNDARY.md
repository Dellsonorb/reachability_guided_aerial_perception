# A5 pause: Ground/RM4D absolute-height assumption

A5 is **not E2E complete**. The UAV public-map localization repair passed its
physical ground tests, but a separate, pre-existing Ground/RM4D geometry mismatch
requires an explicit decision before validation can be interpreted as applying
to the same physical SIM target. No frozen model, method or geometry parameter
has been changed to hide this mismatch.

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
only its existing query-orientation regularization. A5 `worker.py:45` passes that
request to the frozen API without height conversion. Ground launch and request
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

## Decision boundary and unfinished work

An explicit Ground/RM4D frame/model reconciliation might preserve algorithm code,
but it changes the current frozen integration/geometry assumption and requires
rechecking what `validated` means for that configuration. This is separate from
the already fixed UAV localization edge. It is not authorized here by silently
adding an AGENT Z offset, changing A2 ground_z/thresholds, or editing the baseline.

Request: authorize reconciling the frozen RM4D call's Ground height/reference
assumption with SIM while retaining A1-A4 algorithm definitions and baseline code,
then resume A5 from fresh observations.

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
