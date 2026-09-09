# Offline MoveIt clearance diagnosis

Scope: native installed API and library checks only. SIM source is `625b84c`;
the reviewed AGENT checkpoint is `c97298f`. No ROS/Gazebo launch, ROS action,
planning-scene service write, production edit, package download or new dependency
was used. The source624 reconstruction uses archived public JointState/action
endpoints and accepted runtime perception; no simulator target pose is consumed.

## Result

A small native batch checker can cover the complete existing MoveIt robot,
perceived world and attached payload. It needs **both** self and world queries.
The actual native full-model probe reproduces the archived source624 clearance:

| State | Native boolean collision | Closest reported robot pair | Distance |
|---|---|---|---|
| Recorded planned pregrasp | false | base_link / left_finger | 0.404697884629 mm |
| Recorded measured pregrasp | true | base_link / left_finger | collision sentinel `-1` |

The current rendered URDF/SRDF loads 21 collision-bearing links. Expected wheel
visual-only warnings reflect the existing renderer's consolidated chassis
collision model, not missing geometry introduced by the probe. Other recorded
planned chassis gaps include left_outer_knuckle 0.771168216310 mm and AG95 body
4.643487029435 mm. The world target is the accepted refined cuboid; no world
distance is returned below the 20 mm reporting threshold at these pregrasp
states. The recorded failure is therefore captured by the **self** query.

This reconstructs selected states, not the full historical planning-service
scene or the entire executed trajectory. The old run recorded no full scene
snapshot. Native model loading emits a harmless failed attempt to refresh the
read-only user's rospack cache; the query still resolves the existing meshes.

## Installed API evidence

`dpkg-query` reports `ros-noetic-moveit-core` version
`1.1.16-1focal.20250520.013113`; commander is
`1.1.16-1focal.20250521.014903`; moveit_msgs is
`0.11.4-1focal.20250520.001550`. The installed `moveit/version.h` declares
`MOVEIT_VERSION_STR "1.1.16"`, and the linked native FCL reports 0.6.1.
`nm -D -C` confirms the distance entry points in
`libmoveit_collision_detection_fcl.so.1.1.16`.

Required calls, with updated full state and the unchanged current scene ACM:

```cpp
collision_detection::DistanceRequest req;
req.type = collision_detection::DistanceRequestType::SINGLE;
req.enable_nearest_points = true;
req.enable_signed_distance = false;
req.group_name.clear();
req.active_components_only = nullptr;
req.acm = &scene.getAllowedCollisionMatrix();
req.distance_threshold = reporting_band; // >= every required pair margin
state.update();
collision_detection::DistanceResult self, world;
scene.getCollisionEnvUnpadded()->distanceSelf(req, self, state);
scene.getCollisionEnv()->distanceRobot(req, world, state);
```

Use separate results or explicitly clear them between queries. Keep the
existing whole-scene `checkCollision`/bounds/constraint checks as well.

Local source references:

- `/opt/ros/noetic/include/moveit/collision_detection/collision_common.h:196`:
  `GLOBAL` is one minimum; `SINGLE` is a minimum for each body pair. Results
  include distance, ordered `link_names`, `body_types`, and nearest points.
  The map key may sort names differently from those ordered result fields.
- `/opt/ros/noetic/include/moveit/collision_detection_fcl/collision_env_fcl.h:85`:
  native self/world distance overloads. Its protected robot-object constructor
  explicitly includes attached bodies.
- `/opt/ros/noetic/include/moveit/planning_scene/planning_scene.h:687`:
  `PlanningScene::distanceToCollision` explicitly ignores self-collisions.
- `/opt/ros/noetic/share/moveit_msgs/srv/GetStateValidity.srv`: validity,
  contacts, cost sources and constraint results; no clearance request/result.
  The installed commander PlanningSceneInterface exposes scene read/write and
  object access, not native distance querying.
- `/opt/ros/noetic/include/moveit/robot_trajectory/robot_trajectory.h:245`:
  `setRobotTrajectoryMsg(reference_state, trajectory)` copies a full reference
  state and overwrites only trajectory joints. This preserves non-arm joints
  and attached bodies if the reference is complete.

The toy native probe verifies `SINGLE` reports multiple nearby pairs; attached
payload appears as `ROBOT_ATTACHED` in both self and robot/world results;
`touch_links` excludes only its intended touch pair; ACM pair exemptions remove
the corresponding distance; applying attachment removes its world copy.

`DistanceResult` with an empty thresholded map has `DBL_MAX` as its initialized
minimum, which is finite. Report "no pair returned within band", not that value
as a measured distance. Unsigned collision distance `-1` is a sentinel, not a
metre of penetration. A collision can terminate collection early, so its map
must not be claimed to enumerate every offending pair.

MoveIt link padding does not fix this self-collision failure. In the native
toy probe, adding 5 mm hand padding leaves `PlanningScene::checkCollision`
false and unpadded self distance 0.405 mm. Calling the **padded collision
environment directly** does see the overlap. Do not change the production
padding and assume standard self checks inherited it.

## FCL distance accuracy limitation and bounded check

The installed default FCL `GST_LIBCCD` distance calculation has a reproducible
argument-order error for unequal boxes. For side lengths 0.2 m and 0.1 m,
centres separated along X by 0.350405 m, exact surface gap is 0.200405 m.
One argument order reports 0.200405 m; the reverse reports 0.291825571232 m.
The installed `GST_INDEP` solver reports the exact value in both orders.
This is reproduced by **direct FCL calls**, not introduced by MoveIt's map or
scene handling. MoveIt's public DistanceRequest provides no GJK solver selector.

The relevant small band was checked separately with analytic exact box-face
gaps 0.405, 1, 3 and 4.9 mm, 64 roll/pitch/yaw combinations, both argument
orders, and two box-size sets, including the production chassis dimensions
1.026335219 x 0.782744936 x 0.395154782 m and the 0.24 x 0.053 x 0.115 m target.
The small box fits inside the other box's projected face, making its exact gap
known despite rotation. For **each** solver, all 1024 cases agree within 1 um;
maximum overestimate is 1.27e-16 m and no violation of a 5 mm band is missed.
The 5 mm value here is a diagnostic band, not a proposed certified threshold.

Thus the demonstrated large-gap defect did **not** affect these small-band
cases or the actual archived mesh/chassis witness. This is not an exhaustive
proof for edge/edge, concave mesh or arbitrary primitive configurations.
Do not advertise native per-pair numbers as universally exact clearance bounds.

Practical conservative addition, if a guaranteed guard is required at the
known failure anchors: use native collision queries against a **private copy**
with the chassis BOX and/or perceived target/payload BOX half-extents enlarged
by the requested margin. Enlarging a box on every axis contains its Euclidean
margin volume, so this conservatively rejects any geometry within that margin
using the existing FCL collision engine. A separate private padded collision
environment can enlarge the chassis and be called directly for self checks.
World/attached box geometry must be enlarged explicitly in the private copy;
link padding does not pad an attached shape. Retain the original unmodified
collision check and original ACM/contact rules. This is a supplemental margin
check, never a replacement physical geometry or altered grasp aperture.
Do not assume mesh vertex padding is a conservative Minkowski expansion.

For other close primitive pairs, an independent native `GST_INDEP` check can
serve as a numerical cross-check if narrowly added, but constructing a second
general collision traversal is more work than the primary batch helper.
The installed Bullet backend is not a drop-in distance alternative: its shared
library explicitly says both `distanceSelf` and `distanceRobot` are unimplemented.

## Native distance integration option

One native executable/helper, or one typed service if persistent robot-model
loading is needed, should accept a full scene snapshot, complete start state,
and a batch of candidate states/trajectories. Return pass/fail, stage, first
violating sample, pair, reported distance and applicable margin. Reuse standard
ROS message serialization and existing MoveIt dependencies. One small service
definition is sufficient; no evidence store or general framework is needed.

An offline helper subprocess has the smallest lifecycle: no extra ROS node,
and the Python caller already fetches the full scene. Batch a whole candidate
to amortize model loading. A persistent service caches model/meshes and is
preferable if many candidates are checked, at the cost of a launch entry and
service build plumbing. In either form, pass a snapshot explicitly to avoid
the checker silently evaluating a different ACM/target/payload from its caller.

Use the same helper in existing Generic/Ours candidate/IK/approach checks and
at execution planning points:

1. Reject a candidate/IK solution when its whole robot or pregrasp/approach
   path lacks the chosen pair margin; continue only within existing bounded
   seed/plan attempts. One zero-collision IK endpoint is insufficient.
2. Check the complete pregrasp trajectory plus its continuation before
   execution. Recheck the actually generated, retimed descend trajectory from
   measured state. Keep the target's contact exceptions disabled at these
   stages, as they are now.
3. At the measured grasp pose, retain the current physically justified sweep
   endpoint `q_contact`; check all non-exempt pairs over the sampled closure
   with only the existing four finger/pad target exceptions. Do not extend
   the geometric sweep to the controller's unattainable free 0.70 rad goal
   or treat this static sweep as proof of actual grasp/contact evolution.
4. Only after fresh real grasp confirmation, keep the current measured-TCP
   attachment. Check the complete lift with that payload and recheck current
   payload state. Preserve all existing controller, force/stall, grasp,
   displacement, lift and retention gates.

The floor needs a specific lift-departure rule: the accepted payload can begin
only about 1 mm above its support. A global positive pair margin would reject
otherwise valid lift starts. Keep the original payload/floor **collision**
check active from the beginning. During the explicitly bounded initial support
departure, require nondecreasing payload/floor separation; require the normal
margin after leaving that region. This changes the positive-margin schedule,
not the collision ACM or contact criteria. Other payload pairs keep their
ordinary margin throughout. Do not apply this exception to arbitrary obstacles.

Margins should reflect measured tracking/perception error and body-relative
motion. The archived 2.36 mm TCP error and approximately 0.28-degree rotation
explain why 0.405 mm fails, but do not establish a population-wide error bound.
One uniform margin also wastes clearance on rigid pairs whose relative pose
does not change. Preserve the physical collision gate for every non-exempt pair.

## Path interpolation and proof boundary

`RobotState::interpolate` and
`RobotTrajectory::getStateAtDurationFromStart` provide useful nominal state
interpolation; the latter explicitly uses **linear** interpolation. The
installed controller's `trajectory_interface::QuinticSplineSegment` uses
linear interpolation for positions only, cubic for position/velocity and
quintic for position/velocity/acceleration. Consequently, checking only the
linear path between retimed waypoints can miss the commanded spline path.
Use the installed controller segment class to sample the exact final commanded
trajectory, including its measured start bridge, or bound its deviation.

Bound sampling by **whole-body displacement**, not TCP translation alone:
joint rotation can sweep arm, hand, camera and payload while the TCP barely
moves. A conservative kinematic bound uses each body's relevant joint radii
times joint-angle excursion, plus translational motion. For pair A/B, include
both bodies' possible displacement between samples in its margin reserve.
Use an explicit maximum sample count/time; exhausted checking must reject or
report incomplete, not silently accept. Without a valid interpolation bound,
label the result a discretely sampled clearance guard, not continuous safety.

The older Python vertex/triangle-box scripts remain useful independent
witnesses and regressions. Extending them into all mesh/mesh pairs, mimic
joints, multi-DOF transforms, ACM and attached payload handling duplicates
MoveIt's collision system and is a larger, more fragile change than this
native batch checker.

## Preferred bounded revision after design review

For this revision, the smaller choice is a planning-only world collision BOX
copied from the authoritative BUNKER collision, with **12 mm added to every
half-extent** (24 mm to each full dimension). The native collision engine then
enforces a conservative chassis margin for arm, AG95, D435 and attached payload
without relying on distance accuracy or adding a native service. This covers
the known chassis risk; other robot/world pairs retain their existing physical
collision checks and do not acquire a claimed universal positive margin.

The proxy must preserve the original box origin and orientation in
`ground/base_link`. Exempt only links connected to the robot root through an
all-fixed joint chain. Selecting every individually fixed joint would wrongly
exempt the camera and finger pads downstream of the moving arm. In the actual
model, the root-rigid links bearing collision geometry are exactly
`ground/base_link`, `ground/aubo_mount_link`, and `ground/aubo_i5_base_link`.
Other rigid root descendants currently have no separate collision geometry.
The proxy/target pair must remain forbidden through every target ACM update,
so it is checked against the attached payload after the world target is removed.

The native probe's optional `guard` argument applies this exact 12 mm proxy
offline, preserving original physical robot geometry. Results:

| State | Full-robot collision | Manipulator-group collision |
|---|---|---|
| Production SRDF home, gripper open | false | false |
| Archived source624 observation endpoint, gripper open | false | false |
| Archived source624 planned pregrasp | true | true |
| Archived source624 measured pregrasp | true | true |

The planned pregrasp is rejected for proxy contact with AG95 body, left finger
and left outer knuckle. The actual manipulator group's updated links include
D435 body/mount/optical frames and both finger/pad assemblies. Keep explicit
empty-group whole-state validation at the revised gates regardless; these
group findings are not a reason to narrow the checks. This verifies two
specific starting poses, not the entire historical observation motion or every
possible parked pose. Validate actual start before planning and do not rely on
OMPL start-state perturbation to hide an invalid measured pose.

The reviewed bounded search uses at most four existing confirmed exact per-cell
winners in original relevance/stable-index order. At each candidate, consider
two equivalent grasp yaws and three deterministic IK seeds: current, original
RM4D q when available, and a deterministic alternate. This changes neither
confirmation criteria nor candidate coordinates. Reject before executing the
candidate, and repeat feasibility against actual navigation/refinement geometry
before manipulation. A rejected plan is not an execution retry.

Check full approach, descent, closure and prospective lift before committing
motion. Preserve the actual force goal; the proposed extra post-contact
geometric envelope is q_contact through 0.522 rad, where 0.522 is the separately
observed 0.521259 rad rounded upward by the chosen 1 mrad increment. This is a
bounded development envelope, not a proven maximum loaded closure. Static
target geometry can reject a path that real contact would alter; it does not
predict bilateral contact, slip or object motion. Actual grasp/force/stall and
payload retention gates remain necessary.

Prospective payload checking needs an explicitly hypothetical planning context.
Do not fake `grasp_confirmed=True` to bypass the real attachment helper. The
world copy must be removed while the prospective attachment exists; otherwise
the attached object may collide with its same-ID world duplicate. Restore the
exact prior world/robot-state/ACM context on every success, rejection and error
before any actuation, and fail closed if restoration cannot be verified. The
actual attachment still follows fresh real grasp confirmation and measured TCP.

The final retimed command is sampled using the installed controller's matching
cubic/quintic spline at the SIM 1 ms interval through whole-state validity;
retain MoveIt's ordinary collision-aware interpolation too. Include the actual
measured start bridge and gripper/payload states, and recheck each newly planned
executed segment. A time step alone remains a sampled check, not a continuous
clearance proof. Calling a ROS service for each 1 ms sample can require thousands
of requests per candidate: use an explicit checking wall-time limit, and never
accept an incomplete traversal. No new native helper is required initially.

The world proxy remains correctly located because this robot's planning frame
is its fixed root `ground/base_link`; a future virtual/base joint or different
planning frame would require keeping the proxy's base transform synchronized.
Its overlap with the floor or perceived target while both are world objects is
not checked by robot/world collision queries. Prospective payload validation is
therefore essential. The chassis-only proxy introduces no extra payload/floor
margin and avoids the global floor-margin problem described above.

## Reproduction

Sources in this directory:

- `distance_api_probe.cpp`: synthetic primitive geometry, installed FCL solver
  contrast, 2048 small-band checks, ACM/touch/attachment/padding behavior.
- `recorded_state_probe.cpp` and `recorded_state_probe.py`: current rendered
  full model with archived planned/measured source624 states and accepted
  refined target. Python performs only input reconstruction and serialization.

Both C++ sources were built with `g++ -std=c++14` and installed
`pkg-config --cflags --libs moveit_core`, then run as ordinary offline processes.
The recorded-state runner uses `/usr/bin/python3` and the system ROS Python
message libraries. Temporary executables are
`/tmp/p450-ground-clearance-distance-api-probe` and
`/tmp/p450-ground-clearance-recorded-state-probe`. All invoked probe processes
completed with exit code 0. No production implementation is included here.
