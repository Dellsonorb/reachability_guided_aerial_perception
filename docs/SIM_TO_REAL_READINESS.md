# SIM-to-real readiness: architecture audit

Audit date: 2026-09-12. This is a source and documentation audit, with a proposed future first stage. No ROS runtime, simulator, robot, driver, networked hardware, CAN interface, or experiment was started. No runtime/configuration/result files were changed.

**Assessment:** the project has reusable task logic and implemented public ROS interfaces exercised by simulation. It does **not** yet have an evidenced, integrated real-robot backend. Keeping topic/action names is feasible; transferring localization, calibration, actuator feedback and success evidence is substantial work. The current evidence supports simulation claims, not physical deployment readiness or real-world retrieval performance.

## Scope and reference versions

`AGENT` below means `/media/lu/P450_PAPER/AGENT/reachability_guided_aerial_perception`; `SIM` means `/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation`. Source references give repository-relative paths and actual line numbers at the audited checkout; expand these prefixes when opening a file. SIM HEAD was verified as `a0ae8e32889e92a86f24d2794c7cb143999915fd`; AGENT results HEAD was `6fec29b6ad54f76f5bdceb6b0fbee4e709fd374b`, with formal data collected at `3d13a95`. This document is added on `docs/paper1-assets-sim-to-real`.

The release tag and data-collection revision are distinct provenance fields. The older `PAPER1_SIMULATION_METHODS.md` describes a historical experiment draft and is not the authority for this audit. Current evaluation context comes from [EVALUATION_VERSION.md](EVALUATION_VERSION.md), [PAPER1_FORMAL_EXPERIMENT_PLAN.md](PAPER1_FORMAL_EXPERIMENT_PLAN.md), the preserved results, and executable source. Historical SIM design documents describe intended real interfaces; they do not establish that a real implementation exists.

“Implemented” below means a concrete current source/launch connection exists, not that this audit reran it. “Missing” means missing from the inspected task composition and source tree, not a claim about all software on the host or hardware owned by the team. Upstream documentation was checked on the audit date; its current defaults are not the frozen local versions.

## Implemented boundary and missing adapters

| Function / public interface | Actual current implementation | Real integration still missing |
|---|---|---|
| Flight action `/uav1/runtime/flight`, `robot_runtime_interfaces/FlightCommandAction` | Server translates TAKEOFF/FLY_TO/HOVER/LAND to Prometheus; reads native state/control/odom, transforms goals to `uav1/odom`, checks freshness and completion. [S1–S3] | Real Prometheus/MAVROS/PX4 bringup, compatible firmware/messages, estimator origin and frame verification. No integrated real launch evidenced. |
| Flight native topics `/uav1/prometheus/{command,setup,state,control_state,odom}` | `UAVCommand`, `UAVSetup`, `UAVState`, `UAVControlState`, `nav_msgs/Odometry`; controller publishes MAVROS setpoints and uses arming/mode services. Current launch sets `sim_mode=true`. [S1–S4] | Hardware transport/FCU configuration and calibrated native odometry backend. Public action compatibility alone does not validate these. |
| Ground action `/ground/move_base`, `move_base_msgs/MoveBaseAction` | Common ROS navigation produces `/ground/nav_cmd_vel`; velocity guard produces `/ground/cmd_vel`, both `geometry_msgs/Twist`. [S5–S6] | Bind guarded output to official `bunker_ros` → `ugv_sdk` → CAN; explicitly remap upstream absolute topics and configure frames. |
| `/ground/odom`, `/ground/bunker_status`, `ground/odom → ground/base_link` | Gazebo planar plugin publishes `nav_msgs/Odometry`, `bunker_msgs/BunkerStatus` and TF from simulated motion. [S7] | Real measured odometry/status and one odom TF owner, correct protocol/model, consistent message version. SIM status is not a physical CAN health measurement. |
| `/ground/runtime/stop`, `std_srvs/Trigger` | Cancels navigation and publishes zero through `nav_cmd_vel`; same guard path. [S5–S6] | Verify native command expiry and measured stop with real base feedback. Service success is not a physical stopping measurement. |
| `/ground/scan`, `sensor_msgs/LaserScan` | Move-base costmap explicitly consumes simulated 2-D laser in `ground/lidar_2d_link`. [S7–S8] | Actual navigation scan source, mount calibration and obstacle coverage. Neither BUNKER base driver nor UAV MID360 automatically supplies this scan. |
| `/uav1/livox/lidar`, `sensor_msgs/PointCloud2` | MID360 Gazebo plugin's type **2 = XYZRTLT**; `uav1/lidar_link`; AGENT consumes XYZ and transforms each cloud at its header time. [S9–S11, A1] | Livox ROS Driver 2 composition, actual acquisition timestamps and calibrated extrinsics; optional typed conversion if native CustomMsg is needed elsewhere. No such real adapter evidenced. |
| UAV `/uav1/camera/{color,depth}/{image_raw,camera_info}` | RGB/depth `sensor_msgs/Image` plus `CameraInfo`; observer expects the SIM optical convention `uav1/camera_link`. [S12–S14] | RealSense camera selection, actual intrinsics, depth/color registration, optical frame mapping, body-camera calibration and timestamp verification. |
| Ground `/ground/d435/{color,depth}/{image_raw,camera_info}` | Simulated wrist D435; `ground/d435_color_optical_frame` and `ground/d435_depth_optical_frame`; RGB-D observer. [S12–S15] | Real wrist camera backend, hand-eye calibration, depth scale and registration, exact image/CameraInfo frame and timestamp agreement. |
| `/air_observer/target_pose`, `/ground_observer/target_pose`, `geometry_msgs/PoseStamped` in `map` | Synchronized RGB-D geometry, validity and freshness checks; measured Ground surface cue is distinct from accepted target pose. [S12–S14] | Real image-stream binding and validation of the known red-target/size assumptions; inherited observers also wait on `/ground/runtime_ready`. |
| `map → uav1/odom`, `map → ground/odom` | UAV correction recomposes private Gazebo truth with estimator TF at a common stamp; Ground transform is static spawn pose. [S16–S17] | A real shared-map localization/alignment source for each robot and a declared single owner for each edge. This is a major missing backend, not a remap. |
| `/ground/arm_controller/follow_joint_trajectory`, `control_msgs/FollowJointTrajectoryAction` | MoveIt controller mapping to six named joints; Gazebo position trajectory controller with `/ground/joint_states`. [S18–S20] | AUBO real driver/controller binding, compatible joint naming/order/signs/limits, actual joint feedback and calibrated model/mount/TCP. |
| `/ground/gripper_controller/follow_joint_trajectory`, same action type | Position trajectory control of `left_outer_knuckle_joint`; simulated mimic linkage. [S18–S20] | AG95 native command/status conversion, measured opening mapping and action completion/cancellation semantics. Upstream DH command topic is not this action. |
| `/ground/gripper/grasp_confirmed`, `std_msgs/Bool` | Fresh bilateral target-pad Gazebo contact evidence plus fresh sufficiently closed joint position. [S21] | Real evidence producer from available gripper feedback/sensors; false, stale and slip cases need measured characterization. A closed command or action success is insufficient. |
| `/ground/runtime_ready`, `std_msgs/Bool` | SIM spawn/controller readiness gates observers. [S14, S20] | Equivalent real readiness producer for joint feedback/controllers/sensors. Reusing a Gazebo spawn flag would be incorrect. |
| Physical outcome checker | Separate process reads Gazebo target height plus public event/action/grasp evidence. [S23, A3] | An independent real measurement of object lift/retention; no real scoring backend exists in this composition. |

The traced flight path is `AGENT/demo → FlightCommandAction → Prometheus → MAVROS → PX4`. The traced base path is `MoveBaseAction → move_base → nav_cmd_vel → existing guard → cmd_vel → Gazebo planar plugin`. The proposed replacement after the guard is the manufacturer driver and CAN SDK. Arm planning already ends at a standard trajectory action, but both actuator execution and gripper evidence remain simulated.

## Specific transfer findings

### Flight: usable facade, SIM-native composition

The facade is real code, not only an interface definition. TAKEOFF requests arming, Prometheus command control, PX4 OFFBOARD and initial-position hover. FLY_TO transforms a stamped pose into odometry coordinates and sends `UAVCommand::Move/XYZ_POS`; HOVER and LAND use native commands. The facade observes receipt freshness with monotonic time, while TF/goals and its publication loop use ROS time. A public action success therefore means the configured native state satisfied the facade's conditions; it does not establish calibration or physical accuracy. [S1–S3]

The underlying controller already uses `/uav1/mavros/setpoint_raw/local`, `/uav1/mavros/cmd/arming` and `/uav1/mavros/set_mode`. The inspected launch starts SITL-oriented MAVROS and sets `sim_mode=true`, so reusing the facade requires an actual native real bringup. PX4's documented OFFBOARD requirements include continuous incoming proof-of-life setpoints and valid state estimates; the native stack must provide these even after a facade action completes. The facade's 10 Hz loop is not evidence that a real FCU is receiving a valid stream. [S4; official [PX4 OFFBOARD documentation](https://docs.px4.io/main/en/flight_modes/offboard)]

### Ground: message compatibility does not reproduce chassis behavior

Official `bunker_ros` documents its `bunker_base` wrapper over `ugv_sdk`, CAN bringup, and configurable odometry. Its current source subscribes to absolute `/cmd_vel`, passes linear-x/angular-z to `SetMotionCommand`, and publishes absolute `/bunker_status`; placing it inside a `ground` namespace alone does not fix absolute names. Explicit remaps, frame settings and SDK/protocol compatibility are required. This establishes a credible upstream path, not an installed or tested adapter here. [Official [BUNKER README](https://github.com/agilexrobotics/bunker_ros), [messenger source](https://github.com/agilexrobotics/bunker_ros/blob/master/bunker_base/src/bunker_messenger.cpp), [UGV SDK](https://github.com/agilexrobotics/ugv_sdk)]

The SIM base renderer fixes wheel joints and the planar plugin acts on the base; its odometry starts from simulated base pose. This differs from tracked-vehicle slip, encoder integration and physical stopping. Existing geometry/costmap checks remain useful, but arrival/clearance tolerances and stopping observations need measurement on the selected base. The real base driver does not provide the additional navigation laser; the existing costmap requires `/ground/scan`. [S7–S8, S20]

### MID360: preserve type, frame and time together

The active SDF selects `publish_pointcloud_type=2`. In this local plugin, 2 means `PointCloud2` containing XYZ, intensity, tag, line and timestamp; 3 is `prometheus_msgs/LivoxCustomMsg`. Its type-2 header uses `ros::Time::now()` and point timestamps are synthesized as header nanoseconds plus an index-based interval. AGENT reads only `x,y,z` and applies one TF at each cloud header stamp, then combines accepted chunks. This is cloud-wise alignment, not per-point motion deskew. Hover/settling checks reduce motion but do not validate real timestamp latency or remove intracloud distortion. [S9–S11, A1]

The real driver uses different format-number meanings: `xfer_format=0` is its PointCloud2 format, 1 is CustomMsg, and 2 is PCL PointXYZI in ROS1. Its `livox_ros_driver2/CustomMsg` carries a timebase and points with offset times. Do not copy the SIM integer value, assume two custom message package names are interchangeable, or merely relabel timestamps. A real adapter must preserve physical measurement time, frame and XYZ units; a localization consumer that needs native timing can retain a separate native stream. [Official [Livox driver format documentation](https://github.com/Livox-SDK/livox_ros_driver2#32-livox-ros-driver-2-internal-main-parameter-configuration-instructions), [CustomMsg](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/msg/CustomMsg.msg), [CustomPoint](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/msg/CustomPoint.msg)]

Current `uav1/base_link → uav1/lidar_link` is owned by `/uav_control_main_1`, with translation `[0.1471448904, 0, 0.2769686356] m` and pitch `0.35 rad`, derived from the simulated mount and ray sensor. These are exact SIM-model values, not a physical calibration. Real driver extrinsic settings and TF must not both apply the same mount correction. [S9, S11]

### D435: optical conventions and calibration are part of the interface

The UAV SIM uses `uav1/camera_link` as an already optical-coordinate frame, with identity links to optical aliases. Its body-camera transform is a composite of the imported model mount. A normal real camera body frame must not be renamed to this label without checking the actual coordinates. Ground D435 optical joints explicitly include the ROS optical rotation; the simulated color/depth optical centers are coincident, which is not evidence of real depth/color registration. [S12–S15]

The observer synchronizes four streams, verifies image/CameraInfo frame and dimensions, checks stamps, and accepts 16-bit millimetre depth or 32-bit metre depth. Those checks are reusable. RealSense's ROS1 wrapper exposes image/CameraInfo topics and an optional aligned-depth stream; the exact stream pairing must match the observer's projection assumptions. Actual intrinsics/distortion handling, depth scale, camera-to-camera extrinsics, and wrist hand-eye/body-camera calibration remain to be verified. The official current README identifies ROS1 as a legacy unsupported branch, so a Noetic driver version must be pinned and evaluated separately from current ROS2 instructions. [S14; official [ROS1 RealSense documentation](https://github.com/realsenseai/realsense-ros/tree/ros1-legacy), [current support statement](https://github.com/realsenseai/realsense-ros#ros1-and-ros2-legacy)]

### Shared map and clock: largest architectural dependency

IMU messages are not themselves a shared-map solution. The UAV estimator subscribes to `/uav1/mavros/imu/data` alongside MAVROS local position and velocity; the real replacement is the selected physical FCU/MAVROS estimator chain, with verified sensor axes, attitude convention, acquisition time, bias and covariance behavior. The full Ground robot model also publishes simulated `/ground/imu/data` in `ground/imu_link`, but the inspected Ground navigation composition obtains odometry from the planar plugin, not an evidenced wheel/IMU fusion pipeline. A real IMU publisher alone would not replace that odometry or establish either `map → odom` edge. Real measured base odometry and any chosen estimator must be characterized separately; this audit selects or implements no localization algorithm. [S25, S7]

The current SIM composition publishes identity `world → map`, static spawn-derived `map → ground/odom`, and a dynamic UAV correction computed as `T_map_world × T_world_body × inverse(T_uav_odom_body)`. The UAV correction uses `/sim/uav1/base_pose` and estimator TF at the latest common fresh stamp. This creates a consistent public map while idealizing the localization backend. Removing the Gazebo subscriber without a replacement would break the global pose contract. [S16–S17]

The future real composition needs one owner for each map-to-odom edge, one owner for each odom-to-base edge, and one calibrated owner for each sensor mount. The two robots must share map scale, gravity/up, heading, origin and time basis; two independent local odometries do not supply that relation. The AGENT reference-frame bridge also explicitly requires horizontal Ground transforms, zero local Ground z, and the frozen BUNKER–AUBO mount. Its reuse is conditional on these assumptions; uneven-terrain or changed-mount operation would require a separate evaluated change. [A1–A2]

SIM uses `/use_sim_time` and Gazebo `/clock`; the formal runner waits for an advancing ROS clock. Flight/grasp receipt ages use monotonic wall time, while sensor freshness, TF and the velocity guard use ROS time. Real acquisition clocks, host clocks and TF stamps must be related and measured; simply setting `use_sim_time=false` does not establish synchronization. The current runner also forces loopback ROS addressing for its single-host SIM, which cannot serve an unmodified distributed real deployment. No clock synchronization or real map alignment was configured by this audit. [S1, S6, S16–S17, A3]

### AUBO and AG95: execution and evidence must be adapted separately

MoveIt maps `/ground/arm_controller/follow_joint_trajectory` to `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`; the gripper action controls `left_outer_knuckle_joint`. These are concrete contracts. AUBO's current official driver documents real IP-based bringup and model calibration, but this repository imports descriptions/MoveIt configuration and starts Gazebo controllers. The older official AUBO MoveIt mapping uses another controller name and different joint names, demonstrating why same action type is insufficient. The selected real driver version, controller model and name/kinematics mapping remain unresolved. [S18–S20; official [AUBO driver](https://github.com/AuboRobot/aubo_ros_driver), [older controller mapping](https://github.com/AuboRobot/aubo_robot/blob/master/aubo_i5_moveit_config/config/controllers.yaml)]

The SIM `IntegratedVelocityRobotHWSim` optionally reports interval-average joint velocities from successive integrated positions when `P450_GROUND_INTEGRATED_VELOCITY=1`; it retains stock writeSim, limits and PID behavior. The runner records `joint_velocity_feedback` for each attempt. This is a Gazebo feedback adaptation across controlled joints, not a real AG95 velocity-control mode, and cannot be claimed as encoder/drive feedback. Real joint-state velocity and settling semantics need verification independently. [S22, A3]

Current `grasp_confirmed` is produced from separately fresh left/right target-pad contact receipts and a sufficiently closed joint. It does not read a latch topic and does not directly measure retained target height. The upper task checks TCP lift then holds fresh confirmation before emitting `LIFT`. Historical latch/effort-coupling designs and standalone test fixtures are not proof of the active composition. [S21, S23]

DH Robotics' official repository includes AG95 assets and a native driver command interface using `dh_gripper_msgs/GripperCtrl` on `/gripper/ctrl`, with model/port settings. It does not thereby implement the project's trajectory action or `grasp_confirmed`. Its README also describes a default test sequence that opens/closes the gripper; consequently even a future feedback-only stage must inspect the selected driver's startup behavior before treating launch as read-only. No launch or device access occurred here. A real producer should translate only available measured evidence into the public Bool and characterize retention/slip; required sensors cannot be assumed to exist. [Official [DH Robotics driver documentation](https://github.com/DH-Robotics/dh_gripper_ros)]

## What the simulation evidence actually establishes

There are three separate uses of simulator state:

1. **Task decisions:** accepted LiDAR/RGB-D observations and public TF/robot feedback feed environment belief, candidate ranking, finite scanning, handoff, planning and execution checks. Scene-generation truth and checker target height are not passed as direct task observations. This boundary is stated in the formal protocol and visible in the cloud/TF adapter. [A1, A4]
2. **SIM backend support:** public UAV global TF is backed by private Gazebo body truth; Ground global alignment is spawn-derived; public gripper confirmation is backed by simulated contacts. Thus “sensor-only task decisions” does not mean the entire simulation stack performs real sensor-only localization or real tactile inference. [S16–S17, S21]
3. **Independent physical scoring:** `check_air_ground_pick_demo.py` runs separately and reads `/gazebo/model_states` for target baseline/final height, alongside observation/event/flight/controller/grasp evidence. It checks actual target lift rather than accepting a launcher exit or TCP motion alone. The checker is not physics-only in all inputs: only its independent object-motion measurement is simulator-physics truth. Short retention is supported by the task's confirmation hold, not an independent continuous object-height trace throughout the hold. [S23, A3]

A real scoring replacement would need independent object-lift/retention evidence with explicit measurement limits. Keep this separate from decision inputs and the online grasp adapter. The current checker and contact/localization backends are valid simulation infrastructure; they do not establish real sensing, localization, friction, backlash, grasp force, chassis slip or object-retention performance.

Reusable candidates are the pure belief/NBV/finite-scan logic, ranking/accounting schemas, task sequencing, and ROS-facing action clients/flight facade, subject to preserved input semantics. The RM4D map, mount model, task geometry, collision shapes, camera assumptions and numerical thresholds are calibrated assets/assumptions, not hardware-independent constants. Existing source/configuration must remain frozen for the reported simulation results.

## Risk order and a future first real stage

This ordering is an engineering inference from the source dependencies, not an estimated failure probability or a new safety framework.

| Order | Evidence-backed issue | Why resolve it before integrated task execution |
|---|---|---|
| 1 | No real shared-map authority; SIM truth and spawn transforms supply it; mixed ROS/device/host time remains unresolved. | Every sensor endpoint, viewpoint and base/arm handoff depends on these transforms at the right time. |
| 2 | SIM-derived camera/LiDAR/mount/TCP calibration and planar/frozen-mount assumptions. | Systematic frame/depth errors can yield apparently valid but physically incorrect candidates and grasps. |
| 3 | Missing real BUNKER/AUBO/AG95 bindings, navigation laser source and readiness producer. | Standard ROS message names cannot validate drive protocol, feedback, motion direction, completion or start-up behavior. |
| 4 | Simulated contact-based grasp confirmation and independent Gazebo height scoring. | Command completion, grip evidence and successful physical retrieval are different propositions. |
| 5 | Sensor statistics and dynamics transfer: MID360 scan timing, RGB-D appearance/depth, chassis slip and manipulation tracking. | Simulation task/efficiency results may change even after interface equivalence is achieved. |

**Proposed first stage, not executed:** a stationary, sensing/feedback-only contract characterization using whatever physical equipment the team actually confirms. It would not issue flight, base, arm or gripper motion commands.

The concrete deliverable would be a small recorded dataset plus a contract report: selected hardware/firmware/driver revisions; actual topic types and field schemas; camera identities and CameraInfo; LiDAR field/timestamp semantics; declared TF authorities; measured extrinsics; and timestamp/TF age distributions. Record real images/clouds, available joint/base/native flight feedback and TF only after checking that driver startup itself performs no actuator test. If a driver cannot start without motion, use an already recorded dataset or defer that driver from this first stage.

First resolve physical inventory and pin compatible upstream driver versions in a separate future hardware composition. Then map native data into the existing public observation/feedback names and inspect a stationary target with independently measured geometry. Replay that dataset offline through the existing perception/geometry logic, documenting frame agreement, target pose errors, invalid/missing observations and time alignment. Keep actuator interfaces disconnected during this stage. A meaningful exit is reproducible measured agreement and a complete mapping of missing interfaces; it is not an end-to-end retrieval success.

Only subsequent, separately scoped work would verify native flight/base/arm/gripper behavior and then compose a bounded retrieval trial with independent outcome measurement. This audit defines no new supervisory framework and implements no real adapter. It leaves the frozen SIM runtime, thresholds, configuration and results untouched.

## Source index

Line numbers identify the relevant implementation entry, with additional ranges described in text where useful. All entries were read from the audited local trees.

| ID | Concrete local source |
|---|---|
| S1 | `SIM/src/platform/p450_flight_facade/scripts/p450_flight_facade_node.py:46` (topics/server), `:129` (health), `:213` (preemption), `:254` (TF). |
| S2 | `SIM/src/platform/p450_flight_facade/src/p450_flight_facade/translation.py:28` (native command mapping), `:78` (position/speed completion). |
| S3 | `SIM/src/platform/robot_runtime_interfaces/action/FlightCommand.action:1`. |
| S4 | `SIM/src/p450/prometheus_uav_control/src/uav_controller.cpp:134` (MAVROS setpoints/services); `SIM/src/platform/sim_platform_bringup/launch/p450_runtime.launch:83` (controller SIM configuration). |
| S5 | `SIM/src/ground/bunker_navigation/launch/ground_navigation.launch:3`; `SIM/src/ground/bunker_navigation/scripts/ground_stop_server.py:13`. |
| S6 | `SIM/src/platform/bunker_sim_runtime/scripts/velocity_guard.py:10`; `SIM/src/platform/ground_manipulator_runtime/launch/ground_robot_runtime.launch:15`. |
| S7 | `SIM/src/platform/bunker_sim_runtime/src/bunker_planar_move_plugin.cpp:60`; `SIM/src/platform/ground_manipulator_runtime/urdf/ground_robot.urdf.xacro:92`. |
| S8 | `SIM/src/ground/bunker_navigation/config/costmap_common.yaml:1`; `SIM/src/platform/ground_manipulator_runtime/urdf/ground_robot.urdf.xacro:72`. |
| S9 | `SIM/src/platform/sim_platform_bringup/config/p450_mid360_tf_contract.yaml:5`; `SIM/src/platform/sim_platform_bringup/launch/p450_runtime.launch:111`. |
| S10 | `SIM/src/p450/prometheus_gazebo/gazebo_models/uav_models/p450_D435i_mid360/p450_D435i_mid360.sdf.jinja:458`; `SIM/src/p450/livox_laser_gazebo_plugins/include/livox_laser_simulation/livox_points_plugin.h:78`. |
| S11 | `SIM/src/p450/livox_laser_gazebo_plugins/src/livox_points_plugin.cpp:112` (publisher type), `:476` (active XYZRTLT data/timestamps). |
| S12 | `SIM/src/demos/air_ground_pick_demo/config/air_observer.yaml:1`; `SIM/src/demos/air_ground_pick_demo/config/ground_observer.yaml:1`. |
| S13 | `SIM/src/platform/sim_platform_bringup/launch/p450_runtime.launch:57` (optical aliases), `:102` (body-camera transform). |
| S14 | `SIM/src/demos/air_ground_pick_demo/scripts/red_target_observer.py:33` (readiness), `:91` (synchronization), `:120` (metadata), `:158` (depth units); `SIM/src/demos/air_ground_pick_demo/launch/target_observers.launch:1`. |
| S15 | `SIM/src/platform/ground_manipulator_runtime/urdf/d435.xacro:5` (mount), `:43` (optical joints), `:68` (simulated image streams). |
| S16 | `SIM/src/platform/sim_platform_bringup/launch/air_ground_standalone.launch:20` (SIM clock), `:29` (TF authorities), `:49` (SIM robot/common layers). |
| S17 | `SIM/src/platform/sim_platform_bringup/scripts/sim_uav_localization.py:24` (composition), `:83` (common-time lookup), `:124` (private truth input), `:153` (public TF output). |
| S18 | `SIM/src/ground/bunker_aubo_moveit_config/config/ground_controllers.yaml:1`. |
| S19 | `SIM/src/platform/ground_manipulator_runtime/config/controllers.yaml:5` (arm), `:27` (gripper); `SIM/src/vendor/dh_ag95_description/urdf/ag95_gazebo.xacro:6` (mimic). |
| S20 | `SIM/src/platform/ground_manipulator_runtime/launch/ground_robot_runtime.launch:9`; `SIM/src/platform/ground_manipulator_runtime/scripts/spawn_ground_robot.py:193` (readiness); `SIM/src/platform/ground_manipulator_runtime/urdf/ground_robot.urdf.xacro:18` (mount/TCP), `:104` (SIM hardware); `SIM/src/platform/ground_manipulator_runtime/src/ground_manipulator_runtime/renderer.py:128` (fixed wheels). |
| S21 | `SIM/src/platform/ground_manipulator_runtime/scripts/gazebo_grasp_confirmation.py:18`; `SIM/src/platform/ground_manipulator_runtime/src/ground_manipulator_runtime/grasp_confirmation.py:63` (contact receipts), `:88` (predicate). |
| S22 | `SIM/src/platform/ground_manipulator_runtime/src/integrated_velocity_robot_hw_sim.cpp:12`; `SIM/docs/GROUND_MANIPULATION_COMPLETION.md:27` (scope of feedback correction). |
| S23 | `SIM/scripts/check_air_ground_pick_demo.py:278` (target height), `:333` (subscriptions), `:366` (combined checks); `SIM/src/demos/air_ground_pick_demo/scripts/run_air_ground_pick_demo.py:1534` (lift/confirmation hold). |
| S24 | `SIM/config/runtime_sources.json:20` (imported SIM packages/descriptions); `SIM/docs/upstream/Ground-upstream.repos:1` (historical upstream locks, not active bringup). |
| S25 | `SIM/src/p450/prometheus_uav_control/src/uav_estimator.cpp:68` (MAVROS pose/velocity/IMU subscriptions), `:640` (IMU callback); `SIM/src/platform/ground_manipulator_runtime/urdf/ground_robot.urdf.xacro:45` (Ground IMU mount and simulated publisher). |
| A1 | `AGENT/scripts/run_a5_sim.py:189` (ROS adapter), `:231` (PointCloud2 subscriber), `:256` (Ground frame calibration), `:375` (capture), `:449` (cloud-time TF), `:479` (XYZ read). |
| A2 | `AGENT/src/sim_active_perception/frame_bridge.py:35` (planar/frozen mount requirements), `:47` (reference height). |
| A3 | `AGENT/scripts/run_a6_attempt.py:292` (velocity mode), `:298` (single-host environment), `:361` (SIM clock readiness), `:525` (recorded mode), `:613` (separate checker), `:644` (outcome accounting). |
| A4 | `AGENT/docs/PAPER1_FORMAL_EXPERIMENT_PLAN.md:79` (runtime observation boundary), `:161` (physical outcome); `AGENT/docs/EVALUATION_VERSION.md:13` (version distinction). |

Upstream repository references establish that suitable vendor interfaces are documented. Neither a provenance lock nor an upstream feature is evidence that the corresponding physical device or real adapter is present in this workspace.
