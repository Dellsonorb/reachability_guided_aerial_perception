# Offline wrist-3 lift-settling diagnosis

## Conclusion

All three archived Easy runs reproducibly fail the native arm controller's **velocity settling criterion** after lift. Their sampled wrist positions and reported velocities are strongly inconsistent with ordinary sustained rotation. This localizes the issue to the simulation/control feedback or dynamics boundary, but **does not identify a proven Gazebo/backend defect or establish that the reported velocity is false**. Unresolved high-frequency motion, numerical constraint correction, contact/load dynamics, and feedback semantics remain distinguishable hypotheses, not established causes.

Do not count these runs as successful lift or E2E execution. This audit made no production edits, changed no gain or tolerance, substituted no derived velocity, launched no ROS node/simulator, and used no target or robot GT topics.

## Recorded evidence

Inputs: `../launch-03-easy-camera/diagnostics.bag`, `../launch-06-easy-ground-latched/diagnostics.bag`, and `../launch-07-easy-ground-origin/diagnostics.bag`, plus their `runtime.log` files. Public topics inspected: `/ground/arm_controller/state`, `/ground/joint_states`, arm `follow_joint_trajectory/{goal,result}`, and `/clock`.

The table uses native controller-state header time and the window from estimated lift end + 100 ms to action-result receipt − 10 ms. A trajectory header of zero requests immediate start; receipt + trajectory duration approximates its end to controller scheduling precision. Radians are joint-coordinate angles, not map-frame quantities; velocity is rad/s; all timestamps are ROS simulation seconds. No target frame or ground-truth transform is used.

| Run | Sample interval (s), count | Median reported velocity | Max absolute 10 ms position-difference velocity | Position span (µrad) | Net position change (rad) | Integral of sampled velocity (rad) |
|---|---|---:|---:|---:|---:|---:|
| 03 | 45.694–47.574, 189 | −0.16535 | 0.00674 | 84.03 | −0.000018865 | −0.31150 |
| 06 | 67.231–69.121, 190 | −0.16716 | 0.01345 | 147.15 | −0.000002735 | −0.32194 |
| 07 | 71.913–73.793, 189 | −0.15311 | 0.00613 | 73.91 | +0.000012838 | −0.29113 |

Every sampled wrist velocity in these windows exceeds the unchanged 0.10 rad/s stopped-velocity limit. Desired velocity is exactly zero; maximum absolute position error is only 0.0000811, 0.0001006, and 0.0000715 rad, versus the configured 0.05 rad position limit. Every other arm joint's sampled speed remains below 0.014 rad/s. The pre-lift post-descend/grasp hold has zero velocity-limit violations in each run, so this is not a constant offset throughout execution.

Native final results are status 4 / error −5 (`GOAL_TOLERANCE_VIOLATED`), reporting `wrist_3_joint goal error 0.000035`, `0.000033`, and `0.000037`. Result times are 47.587, 69.131, 73.811 s: respectively 2.0011, 2.0027, 2.0026 s after estimated trajectory end, matching the existing 2.0 s settling allowance. Empty action goal tolerances and zero action goal-time override preserve configured defaults.

Joint-state feedback corroborates the sampled anomaly: its wrist velocity violates 0.10 rad/s in all corresponding samples too. Its header is consistently 1 ms before controller-state feedback. Across those paired stamps, median position-difference velocities are −0.00862, +0.00843, +0.00782 rad/s, while paired reported means are −0.15922, −0.17356, −0.16021. These streams share simulated hardware state: they are **not independent sensors**. The published efforts are small, but commanded/reported effort is not a measurement of every contact or constraint impulse and does not exclude dynamics.

## Controller and simulator evidence

SIM root: `/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation`.

- `src/platform/ground_manipulator_runtime/config/controllers.yaml:5–25,46`: position trajectory controller, 100 Hz state publication, 0.05 rad goal-position tolerance, 0.10 rad/s stopped-velocity tolerance, 2.0 s goal time; wrist-3 PID P=100, I=0, D=0.1. Archived runtime parameter dumps independently show these values, so they are not inferred solely from the mutable current worktree.
- Installed `/opt/ros/noetic/include/joint_trajectory_controller/joint_trajectory_controller_impl.h:774–783` directly reads joint-handle position/velocity and computes desired minus actual velocity. Lines 424–446 test goal tolerances, then unconditionally print **position** error in the abort string, even for a velocity failure. Thus a tiny printed “goal error” is not evidence that the controller rejected that tiny position error.
- Installed `.../joint_trajectory_controller/tolerances.h:299–315` assigns `stopped_velocity_tolerance` to every joint's goal-velocity bound. `.../hardware_interface_adapter.h:120–127` forwards desired position only for the PositionJointInterface. Position actuation and velocity-based acceptance are separate; the observed abort is consistent with controller implementation, not a demonstrated controller-tolerance bug.
- Installed `gazebo_ros_control/default_robot_hw_sim.h:125–135` has separate position/velocity/effort buffers. Read-only disassembly of `libdefault_robot_hw_sim.so`, `DefaultRobotHWSim::readSim` at `0x2c5d0`, shows a `Joint::Position` call and separate virtual feedback reads, rather than finite-differencing the published ROS positions.
- For the configured default ODE backend, installed `libgazebo_physics.so.11.15.1` directly calls `dJointGetHingeAngle` in `ODEHingeJoint::PositionImpl` (`0x18ccd0`) and `dJointGetHingeAngleRate` in `GetVelocity` (`0x18cd90`). This was checked against installed binary symbols/disassembly; full upstream implementation sources are not locally installed. `air_ground_standalone.launch:20–27` includes Gazebo `empty_world.launch` without overriding its `physics=ode` default. This establishes separate angle/rate paths, not that either is incorrect.
- The model's wrist-3 is a revolute joint with axis `(0,0,1)`, limits ±3.04 rad (`aubo_i5_macro.xacro:241–246`); its nonzero inertial definition is at lines 57–60. Current base plugin source directly imposes base-link linear/angular velocity every update (`bunker_planar_move_plugin.cpp:180–188`). Coupled-body/constraint effects are therefore a relevant investigation boundary, but this source inspection does not prove that plugin causes the wrist symptom, nor establish identical plugin revisions across the three runs.

Installed versions: Gazebo `11.15.1-1~focal`; gazebo_ros_control `2.9.3-1focal.20250521.005335`; joint_trajectory_controller `0.22.0-1focal.20250521.005320`.

## Limits and next discriminating evidence

The 100 Hz position/velocity traces and two adjacent publication phases do not capture all physics-step states; `/clock` advances principally in 1 ms steps. A trapezoid integral of sampled velocity is **not** an integral of the missing physics-step velocity. Aliasing, angle/rate evaluation phase, off-axis constraint residuals, contact response, or a backend implementation defect cannot be separated by these bags. Small position displacement also does not establish a safely settled object or wrist. There is no justified success-forcing patch from this evidence.

A separately authorized future diagnostic would need synchronized physics-step hinge angle/rate, parent/child angular velocities and axes, applied control effort, and contact/constraint information around lift settling; compare their kinematic consistency before considering a fix. This report requests no additional startup and changes no settings.

## Reproduction

From the AGENT repository, with native ROS Python installed:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages /usr/bin/python3 outputs/development/ground-execution-batch/wrist-diagnostic/analyze_wrist.py
```

The script opens bags read-only, checks finite samples/monotonic stamps and exact result-to-goal IDs, and prints the data stored in `findings.json`. It does not initialize ROS, query services, alter files, or access GT. The comparison is an offline diagnostic, not a successful lift verification.
