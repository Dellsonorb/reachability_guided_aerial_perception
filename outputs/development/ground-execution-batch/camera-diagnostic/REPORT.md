# Offline Ground camera and observation-position diagnosis

2026-09-09. Development diagnosis, **zero new starts**. Findings use only public
runtime TF, CameraInfo, RGB-D, observer outputs, recorded trajectories, and robot
model/configuration. No ground-truth poses, contact topics, Gazebo services, ROS
nodes, or live services were used. Historical inputs are unchanged. Numbers and
exact sampled camera transforms are in [findings.json](findings.json).

Inputs are the `launch-06-easy-generic`, `launch-08-moderate-ours`,
`launch-09-moderate-generic`, and `launch-05-hard002-v14` directories beneath
`outputs/development/operational-batch`. Each supplies `diagnostics.bag`,
`diagnostics-images.bag`, `data/target_reference.json`, `data/events.jsonl`, and
`data/trajectory.jsonl`. The historical checkpoint and outcomes remain those in
`docs/DEV_OPERATIONAL_BATCH_RESULTS.md`; this analysis does not replace trials.
SIM source inspected is `ece36ecd7419f49596a42bb7502b741b1ad24269`, whose latch
configuration change is separate from the historical runtime.

## Exact mounting and frame convention

`T_A_B` maps coordinates in B into A: `p_A = R_A_B p_B + t_A_B`. Quaternion
arrays use `(x,y,z,w)`; opposite quaternion signs represent the same rotation.
ROS optical axes are right/down/forward; map +z is up. TCP and ee orientations
coincide. Public static TF agrees with the model:

```text
T_ee_camera:  t = ( 0.105, 0, 0.075)
T_tcp_camera: t = (-0.095, 0, 0.075)
R_ee_camera = R_tcp_camera = [[ 0,  0,  1],
                             [-1,  0,  0],
                             [ 0, -1,  0]]
q = (0.5, -0.5, 0.5, -0.5)
camera-frame TCP position = (0, 0.075, 0.095)
```

Sources within SIM: `src/platform/ground_manipulator_runtime/urdf/d435.xacro`
lines 8, 46, 62; `urdf/ground_robot.urdf.xacro` lines 32–37. The mount offsets
are 55 mm + 50 mm along ee +X, and 75 mm along ee +Z; TCP is 200 mm along ee +X.
Color and depth optical transforms coincide. CameraInfo gives image 640×480,
`fx=fy=462.1379497504639`, principal point `(320.5,240.5)`.

The top-down grasp uses ee +X downward. Its camera is consequently 95 mm above
TCP and 75 mm along the target long axis, looking vertically downward. It is
not centered over the TCP target. This offset makes part of the nominal target
project past the lower image edge at the current pregrasp height.

## What the bags establish

| Run | Evidence | Interpretation |
|---|---|---|
| 06 Easy Generic | At t158.531, 4,658 red pixels; bbox `[431,412,566,479]`; reproduced center z0.015179 m | Clipped side face is treated as full top |
| 08 Moderate Ours | Arm result succeeds t172.381; 190 saved RGB frames t172.987–185.119 have zero red | Actual post-motion visibility fails |
| 09 Moderate Generic | No arm goal recorded; fresh Ground poses continue through sensor t198.402 | Failure occurs before Ground-refine wait |
| 05 Hard-002 | No arm goal recorded; fresh Ground poses continue through sensor t190.946 | Failure occurs before Ground-refine wait |

For launch06, applying the production estimator to the exact synchronized
t158.531 RGB-D frame gives `(1.886204,-0.244543,0.015179,-0.531302)`. The selected
highest band has SVD singular values `(0.225303,0.112025,2.06e-7)` and plane
normal `(-0.504311,-0.863515,-0.003586)`: a planar **vertical** face. This is not
a noisy horizontal top. The earlier accepted sequence produced the documented
0.013816 m center height. The unchanged height gate correctly rejects it.

For launch08, the t162.987 red RGB-D surface has 5,136 pixels and bbox
`[0,440,132,479]`. Measured map z reaches 0.11551 m. The existing estimator gives
`(1.844145,-0.273160,0.057936,-1.370465)`; its position differs by about 0.41 m
from the aerial reference `(2.052203,0.081630,0.060225)`. The partial observation
is not asserted to be a fully valid cuboid center, but its measured surface
points are sufficient for the following projection calculation.

Transforming all these measured points through map into the actual camera at
t173.0 gives u637.8–1240.3, v626.8–954.2 and depth0.20917–0.32428 m: **0/5,136
points inside 640×480**. Conditional on the target remaining stationary during
arm motion, the previously measured red surface is outside the final FOV.

The older aerial reference, in contrast, projects at `(312.19,375.41)` at
t172.987. Its predicted depth is0.263074 m; measured depth at rounded pixel
`(312,375)` is0.321146 m. Thus that ray does not show a nearer self-occluder.
The aerial top corners span u251.05–370.36 and v143.14–683.59, including clipping.
Neither an in-frame reference center nor an arm-motion success establishes
visibility of the current target. Self-occlusion elsewhere is not ruled out.

## Observation depends prematurely on grasp feasibility

SIM `src/demos/air_ground_pick_demo/scripts/run_air_ground_pick_demo.py:827`
generates a pregrasp and grasp from the aerial estimate and calls
`_execute_pregrasp(pregrasp, grasp)` before `_wait_for_ground_target`.
AGENT `scripts/a5_manipulation.py:47` first requests collision-aware IK for the
grasp, then a reverse grasp-to-pregrasp Cartesian path, an arm plan, and a
forward continuation. Hence failed grasp IK can prevent an observation move.
Existing service response codes were not recorded, so collision rejection and
no IK solution cannot be separated for launch09/05.

## Public swept-reference geometry: contact remains a hypothesis

The rendered base collision model uses origin
`(0.018058912,0.001357451,-0.160420741)` and size
`(1.026335219,0.782744936,0.395154782)` from SIM
`src/platform/ground_manipulator_runtime/src/ground_manipulator_runtime/renderer.py:13`.
At public map base z0.36 its vertical interval is approximately
`[0.002002,0.397157]` m, overlapping the perceived target's height interval.

At each available recorded trajectory sample between GROUND_APPROACH and
GROUND_STOPPED, rotate this collision rectangle by the measured base yaw and
translate its offset by the measured base pose. Test it against the old aerial
target rectangle with its stored center, yaw and dimensions. For every unit
axis `a` from either rectangle, compute
`g(a)=|a·(c_target-c_base)|-sum_i h_base[i]|a·u_base[i]|-sum_i h_target[i]|a·u_target[i]|`.
All four gaps negative means rectangle intersection. The reported penetration
is `-max_a g(a)` at an intersecting sample; it is an SAT measure, not a measured
contact deformation. No interpolation or claims about missing samples are made.

| Run | Intersecting samples | First / last simulation time | Largest minimum-axis penetration |
|---|---:|---|---:|
| 06 | 0 | — | — |
| 08 | 99 | 144.706 / 157.105 | 0.102393 m |
| 09 | 60 | 146.988 / 159.788 | 0.052254 m |
| 05 | 57 | 140.062 / 153.062 | 0.050677 m |

The costmap obstacle source is only `/ground/scan`
(`src/ground/bunker_navigation/config/costmap_common.yaml:5`). The lidar is
0.25 m above a base at map z0.36, approximately z0.61 m, above the 0.115 m target.
The sweep therefore supports a possible unprotected low-target interaction and
reference-displacement mechanism. It does **not** prove physical contact,
physical target motion, or a platform defect. Perception/reference error remains
an alternative. Easy06 has no overlap with this old reference envelope.

## Minimal camera-aware observation synthesis review

Generate a bounded family of **camera** poses around a public measured aiming
cue. Prefer fresh, finite, registered red support from the current stationary
pose when available. A partial surface can guide a new view without being
accepted as a grasp pose; do not silently treat its centroid as the complete
object center. Use the aerial reference as a fallback and explicitly preserve
disagreement rather than assuming it remains current.

For camera position `c` and aim `p`, set `forward=(p-c)/||p-c||`. Project a
deterministic roll-reference axis into the plane perpendicular to forward to
obtain `right`; select another axis if nearly parallel. Set
`down=forward×right` and `R_map_camera=[right down forward]`. Check finite
orthonormal columns and determinant +1. The usual `forward×world_up` construction
needs a fallback for a camera looking straight down.

Then compute **`T_map_tcp=T_map_camera inverse(T_tcp_camera)`**, using the full
extrinsic, not a direct assignment of camera pose to TCP. Bound distance and
candidate count; project the declared target/aiming envelope with CameraInfo
and require finite positive depth and useful image coverage. Plan the TCP pose
collision-aware from current joint feedback without demanding a final grasp IK
or grasp continuation. After execution, check actual TCP/camera TF, freshness,
and measured view quality. Actual RGB-D establishes visibility; predicted FOV
and arm feasibility do not. A mesh-visibility framework is not required for this
targeted implementation.

Sanity example: for a nominal top-down camera centered in target XY at map
z0.60 m, TCP is z0.505 m and 75 mm opposite the target long-axis direction.
With a nominal target center z0.0575, the top projects approximately to
u295–346, v125–355. This is a transform/projection example, not a demonstrated
collision-free IK solution. Collision-aware planning and measured observation
still decide whether a candidate works.

The observer needs a geometric validity check before interpreting the highest
sampled band as top: horizontal-plane support, nondegenerate two-dimensional
extent, and enough observed boundaries to identify center/yaw. Otherwise report
partial/ambiguous support and acquire another view. Do not relax the existing
height, collision, freshness, or success criteria. Grasp planning follows a
valid fresh refinement.

## Reproduction

Use native ROS imports offline: source `/opt/ros/noetic/setup.bash` and the SIM
`install/p450-clean/setup.bash`, then run `/usr/bin/python3`. No `rospy.init_node`
is needed. Feed bag `/tf_static` into `tf2_py.BufferCore.set_transform_static`
and `/tf` into `set_transform`, with a cache longer than the bag. Read only
public topics; do not iterate or inspect target contact/GT payloads.

For the exact timestamps in findings.json, decode RGB as rgb8 and depth as
32FC1, select red using `air_ground_pick_demo.perception.select_red_component`
with min_pixels40, and backproject with `backproject_mask(mask,depth,K,0.12,2.5)`.
The sampled RGB/depth timestamps coincide and their optical extrinsics are
identity. Transform with image-stamped public TF. Reproduce the existing pose
with `estimate_target_pose(points_map,0.115,0.015)`. For the launch06 plane test,
select `z>=percentile(z,95)-0.015`, subtract its mean, and use the last right
singular vector. For launch08 transport, transform t162.987 measured map points
into the t173 camera; project `u=fx*x/z+cx`, `v=fy*y/z+cy`. Count finite positive
depth points within `0<=u<640,0<=v<480`. The swept test uses the formula above on
the saved JSONL poses and target_reference.json. Small stored numbers are
rounded where indicated; exact sampled TF and reference poses are retained.
