# Moderate full-robot manipulation: bounded offline diagnosis

Baselines inspected: AGENT `4d5116d`; SIM `7d2abf6`. No production edits, ROS master/services/nodes, Gazebo/Bullet starts, or candidate reselection. Geometry uses the archived accepted `GROUND_REFINED` measurement and public base/joint data, not launch target coordinates, GT topics, or raw surface cues.

## Outcome

The currently fully-open AG95 has a **proved chassis collision at the required Moderate TCP pose**, independent of arm IK branch. A 180-degree symmetric top-grasp yaw swaps the colliding finger names without removing the overlap. A target-sized preshape derived from the existing aperture calibration removes that particular geometric obstruction. **Neither result verifies a full-arm observation-to-grasp path or attached-payload lift.** Keep the native Moderate failure and the separate wrist/lift investigation unchanged.

## Recorded failure and model comparison

`ground-execution-batch/launch-04-moderate-camera` and `launch-08-moderate-grasp-diagnostic` each report six production grasp IK failures (`-31`). Launch08 subsequently obtains one collision-disabled IK solution, then `check_state_validity` reports collisions between `ground/base_link` and `ground/{forearm_link,wrist_1_link,left_finger,left_outer_knuckle}`. That solution was not executed. Six failed searches do not prove all configurations infeasible, but the returned colliding state is not an executable alternative.

Source624 is base pose `[2.5368278045875257,-0.3799571971654724,-pi]`; launch08 fresh target is `[2.05857697767611,0.079610809295626,0.0589739777528906,0.055804534739021916]` in map `(x,y,z,yaw)`. Public nominal base height is 0.36 m; `T_base_arm` translates `(0.15,0,0.122)` with identity rotation. The target center in base coordinates is `(0.478250827,-0.459568006,-0.301026022)`; in arm-base coordinates `(0.328250827,-0.459568006,-0.423026022)`. Default grasp TCP base z is −0.279126022 m; pregrasp/lift-height z is −0.129126022 m.

An offline FK comparison using the archived RM4D joint branch gives identical SIM/RM4D six-joint arm matrices (maximum element difference 0); RM4D recorded flange residual is `7.77e-16`. SIM flange-to-TCP agrees with the existing RM4D calibration to `1.84e-16`. This is not evidence for another frame offset correction.

The important difference is collision model scope:

- RM4D `rm4d/robots/aubo_i5.py` loads the seven-link bare AUBO URDF and checks arm self-collision/floor. `src/sim_active_perception/task_map.py` retains that robot and relocates only the floor. Candidate `collision_free=true` therefore does **not** certify BUNKER, AG95, D435, target, or attached payload clearance.
- SIM MoveIt and simulator both use `ground_manipulator_runtime/renderer.py` on `ground_robot.urdf.xacro`, including BUNKER, AUBO, AG95, and D435. The renderer replaces BUNKER/wheel collisions with the authoritative chassis box; it does not omit the base.
- `ground_robot.srdf` keeps the recorded base-versus-arm/gripper pairs enabled. Its adjacency/mount/gripper closed-chain exemptions are not permission to ignore those real collisions.

## Positive geometry evidence and required opening

The rendered chassis collision box in `ground/base_link` is:

`x=[-0.4951086975,0.5312265215]`, `y=[-0.390015017,0.392729919]`, `z=[-0.357998132,0.03715665]` m.

Public joint feedback immediately before refinement has AG95 master q=0.0000334163 rad. At the prescribed TCP, actual transformed collision-mesh vertices are strictly inside this box: left outer knuckle up to 11.866 mm from the closest box face, left finger 12.178 mm. This is positive mesh-in-box evidence, not merely overlapping bounding boxes. The same x/y overlap exists at pregrasp and nominal lift height. Yaw+pi gives the corresponding right-side overlaps, so arm seed changes alone cannot remove the fully-open hand collision.

The existing size and margin give required opening `0.053+0.002=0.055 m`. Existing `conservative_jaw_opening` returns `0.0952*(1-q/0.93)`. Its inverse yields an aperture bound:

`q_limit = 0.93*(1-0.055/0.0952) = 0.392710084 rad`.

This is independently geometry-derived, not selected to fit collision outcomes. It is **not** a robust actuator command by itself: select an independently justified tracking allowance, require `q_command + delta_q_tracking <= q_limit`, and still validate the fresh measured aperture and collision scene after preshaping. The existing opening gate rejects `<=0.055`; commanding equality and assuming it passes is incorrect. The broad existing joint-goal tolerance does not certify aperture safety or chassis clearance.

| Fixed hand geometry | Actual mesh inner-pad gap | Minimum AG95/chassis horizontal separation | Target result at nominal grasp |
|---|---:|---:|---|
| Measured fully open | 95.198 mm | proved overlap 12.178 mm | no target intersection |
| q_limit preshape | 59.642 mm | at least 4.762 mm | all meshes separated; smallest axis gap 0.447 mm |
| Ideal 53 mm pad-contact q | 53.0001 mm | at least 4.762 mm | no triangle/target-box intersections in evaluated pose |

For preshape, finger/chassis separation is at least 5.572 mm and outer-knuckle/chassis 5.955 mm. Every AG95 mesh has a positive horizontal separating axis from the chassis, so its fixed-q vertical sweep preserves that hand/chassis clearance. Target clearance during the fixed-orientation preshape descend follows from the positive target-box separating axes at the lowest pose. The perceived target itself has 36.401 mm horizontal separation from the chassis, preserved during a vertical 0.15 m payload translation. This checks **hand/payload versus chassis only**, not payload versus the rest of the arm or surrounding scene.

The 55 mm finger lever and 44.691-degree outer-joint angle in the URDF give ideal pad-contact q≈0.457374407 rad. At that pose the inner-knuckle bounding box overlaps the target bounding box by 0.590 mm, but a triangle-versus-oriented-target-box separating-axis test finds zero intersections. Therefore this audit does **not** claim a knuckle collision from the AABB alone. Actual contact q, any deformation, and closure dynamics remain runtime measurements; intermediate closure and full-arm paths are not certified here.

Preshape changes the pad's vertical offset. With the existing fully-open insertion geometry, pad/target vertical overlap changes from 20.042 mm to 32.059 mm at preshape and 33.331 mm at ideal pad contact. If preserving the intended 20 mm overlap exactly is required, the URDF-derived contact pad offset is about 2.309 mm, yielding a 13.291 mm higher TCP—not a nominal-target-center change. This is an optional geometry-consistency correction, not necessary evidence for removing the proved open-hand/chassis overlap, and not a blanket license for tilted grasps.

## Actual planning-scene gaps

SIM `run_air_ground_pick_demo.py::_initialize_moveit` adds only `air_ground_floor`. No production perceived-target `CollisionObject`, target ACM phase handling, or attached-object lifecycle is present. `_pick_and_lift` executes pregrasp, Cartesian descend, close, then Cartesian lift without attaching the perceived target. Thus current collision-aware motion covers robot/floor, not the requested robot-plus-target/payload chain. No environment perception geometry feed to MoveIt was found either; navigation costmaps are not automatically a manipulator collision scene.

The existing `_robot_state_from_joint_feedback` also returns a fresh `RobotState` with no attached objects and default `is_diff=false`. MoveIt's installed message definition explicitly associates that flag with clearing attached bodies. Any new attachment integration must address this request-state path; adding a world/attached object alone is insufficient.

AGENT `scripts/a5_manipulation.py` uses a collision-aware grasp IK, reverse Cartesian approach, joint-space plan from the current posture, and forward continuation check. However all six outer attempts use the same measured seed and same grasp pose; KDL may search internally, but the outer loop does not explicitly diversify branches. The accepted forward continuation is checked then discarded; descend/lift are recomputed later. Lift feasibility and attached payload are absent from this pre-execution branch decision.

## Minimal implementation recommendation and tests

Prioritize the root-owned lift diagnosis first. For the next bounded manipulation change, keep the current candidate and robot model:

1. Use one shared SIM-owned manipulation helper, with the AGENT adapter calling it. Add accepted perceived target geometry in the correct planning frame; raw surface cues remain camera-aim-only. Update target pose from accepted Ground refinement before manipulation.
2. Preshape using known short span plus existing margin, with an independently specified actuator tracking allowance and fresh measured aperture. Check the actual full robot state after this change. Do not infer full-arm clearance from the mesh results above.
3. Bound explicit seed/branch alternatives at the current base (for example current-state plus a small deterministic set within limits, paired with the two physically equivalent top-grasp yaws). Require collision-aware observation-to-pregrasp, descend, and hypothetical attached-payload lift before committing the branch. Preserve the selected orientation/poses across all stages; switching only pregrasp yaw while descending to the old grasp would invalidate the branch check.
4. Allow only intended target/pad contacts during closure; keep target-to-base/arm/body/knuckle collisions enabled. After fresh grasp confirmation, transform the perceived cuboid into the measured TCP frame and attach it **in the planning scene only**, with narrow pad touch links. Preserve attachment state in all subsequent RobotState/Cartesian requests, and validate/replan the lift from actual measured joints. This attachment must not create a physical simulator weld or manufacture success.
5. Report exhaustion as infeasible/unresolved at the current base. Candidate reselection is deferred; no reselection implementation is proposed for this immediate batch.

Focused tests should cover: calibration inverse/limits/nonfinite inputs; measured aperture fail-closed despite nominal command success; correct Ground-to-planning-frame target box; no cue-to-cuboid promotion; enabled base/arm/hand collisions; exact open-hand collision regression and preshape geometry; coherent selected yaw across all stages; rejection when any descend/lift fraction or endpoint is invalid; correct world-object-to-attached-object transition only after confirmation; attachments retained in Cartesian start states; and no execution after any full-chain collision failure. Existing robot model and collision semantics must remain unchanged.

## Reproduction and limits

From AGENT repository:

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 outputs/development/ground-manipulation-batch/moderate-diagnostic/analyze_geometry.py
```

`findings.json` stores the resulting compact geometry evidence. The script only reads archived public joint/accepted-target/base data, renders the existing URDF offline, and transforms existing collision meshes. Its triangle-box checks include simple intersection/separation assertions. It performs no IK, MoveIt planning, runtime closure, or payload execution. These limits are deliberate: the saved positive hand-collision evidence and necessary preshape geometry must not be promoted into a full manipulation-success claim.
